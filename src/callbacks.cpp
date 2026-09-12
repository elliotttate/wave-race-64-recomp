// Phase 03: the platform callbacks ultramodern asks the project to supply.
//
// ultramodern reimplements libultra but deliberately owns no platform I/O, so
// everything that touches a window, a gamepad, a speaker or an OS dialog is
// provided from here. SDL2 does the work; RT64 already vendors and links it.
//
// Audio and the RSP microcode were honest placeholders through phase 04 --
// neither is needed to reach the boot gate -- and are real as of phase 05:
// samples go to an SDL audio device, and RSP tasks run the audio microcode
// recompiled from the cartridge.

#include "wr64/callbacks.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <SDL.h>
#if defined(_WIN32) || defined(__APPLE__)
#   include <SDL_syswm.h>
#endif
#if defined(__APPLE__)
#   include <SDL_metal.h>
#   include <pthread.h>
#endif

#include <ultramodern/ultramodern.hpp>
#include <ultramodern/input.hpp>
#include <ultramodern/rsp.hpp>
#include <ultramodern/events.hpp>
#include <ultramodern/error_handling.hpp>
#include <ultramodern/config.hpp>
#include <ultramodern/threads.hpp>
#include <librecomp/rsp.hpp>

#include "wr64/crash_handler.h"
#if WR64_WITH_FRONTEND
#   include "wr64/frontend.h"
#   include <recompinput/input_events.h>
#endif
#include "wr64/display.h"
#include "wr64/testdrive.h"
#include "wr64/renderer.h"
#include "wr64/water.h"
#include "wr64/music.h"
#include "wr64/haptics.h"

// The recompiled audio microcode, produced by RSPRecomp from the cartridge.
//
// Declared at global scope and with C++ linkage, matching how RSPRecomp emits
// it. Inside the anonymous namespace below it would get internal linkage and
// fail to resolve; with extern "C" it would get a different mangled name. Both
// produce the same undefined symbol at link time and neither is obvious.
RspExitReason aspMain_run(uint8_t* rdram, uint32_t ucode_addr);

namespace {

SDL_Window* g_window = nullptr;
SDL_GameController* g_controller = nullptr;

// ---------------------------------------------------------------- input ----

#if WR64_WITH_FRONTEND
// Finds the first connected SDL game controller, without tracking add/remove
// events ourselves.
//
// With the frontend on, recompinput::handle_events() (below) is the only thing
// draining SDL's event queue -- SDL_PollEvent removes what it returns, so a
// second loop here would never see anything, including CONTROLLERDEVICEADDED
// and CONTROLLERDEVICEREMOVED, which this code used to react to directly.
// SDL_GameControllerOpen on an already-open device returns the existing
// handle rather than opening it again, so rescanning here every poll is cheap
// and correct rather than a workaround.
void refresh_primary_controller() {
    if (g_controller != nullptr && !SDL_GameControllerGetAttached(g_controller)) {
        g_controller = nullptr;
    }
    if (g_controller == nullptr) {
        for (int i = 0; i < SDL_NumJoysticks(); ++i) {
            if (SDL_IsGameController(i)) {
                g_controller = SDL_GameControllerOpen(i);
                if (g_controller != nullptr) {
                    break;
                }
            }
        }
    }
}
#endif

void poll_input() {
#if defined(__APPLE__)
    // osContStartReadData also invokes this callback on the game thread.
    // Cocoa events must be pumped by update_gfx on the main thread only.
    if (!pthread_main_np()) {
        return;
    }
#endif
#if WR64_WITH_FRONTEND
    // recompinput::handle_events() is the library's own polling loop, and it
    // has to be the only thing draining SDL's event queue: a hand-rolled loop
    // here that just forwarded events to recompui skipped bookkeeping later
    // event handling depends on. Concretely, it never registered a connected
    // controller with recompinput's profile system, so
    // profiles::get_input_profile_for_player(0, Controller) kept returning -1
    // for "no profile assigned" -- and the first real button press then
    // indexed a profile vector with that -1. MSVC's hardened STL reports that
    // as "vector subscript out of range" and fails the process immediately,
    // uncatchably, with no stack trace. Keyboard input never does that lookup,
    // which is why only a gamepad triggered it.
    recompinput::handle_events();
    refresh_primary_controller();
#else
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                // Traced during phase 04: a clean exit-code-0 shutdown and a
                // crash look the same from outside, so it matters whether the
                // quit came from here or from the runtime deciding to stop.
                std::fprintf(stderr, "[wr64] SDL_QUIT received; asking the runtime to quit\n");
                std::fflush(stderr);
                ultramodern::quit();
                break;
            case SDL_CONTROLLERDEVICEADDED:
                if (g_controller == nullptr) {
                    g_controller = SDL_GameControllerOpen(event.cdevice.which);
                }
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                if (g_controller != nullptr &&
                    event.cdevice.which ==
                        SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(g_controller))) {
                    SDL_GameControllerClose(g_controller);
                    g_controller = nullptr;
                }
                break;
            default:
                break;
        }
    }
#endif
}

// N64 controller button bits, as libultra defines them.
enum : uint16_t {
    BTN_A       = 0x8000,
    BTN_B       = 0x4000,
    BTN_Z       = 0x2000,
    BTN_START   = 0x1000,
    BTN_DUP     = 0x0800,
    BTN_DDOWN   = 0x0400,
    BTN_DLEFT   = 0x0200,
    BTN_DRIGHT  = 0x0100,
    BTN_L       = 0x0020,
    BTN_R       = 0x0010,
    BTN_CUP     = 0x0008,
    BTN_CDOWN   = 0x0004,
    BTN_CLEFT   = 0x0002,
    BTN_CRIGHT  = 0x0001,
};

float axis_to_n64(Sint16 value) {
    // The N64 stick reports roughly +/-80 at full deflection rather than the
    // +/-127 an SDL axis suggests, and Wave Race is unusually sensitive to how
    // the stick is scaled -- steering is analogue throughout.
    constexpr float kDeadzone = 0.12f;
    constexpr float kN64Range = 80.0f;

    float normalized = static_cast<float>(value) / 32767.0f;
    if (normalized > -kDeadzone && normalized < kDeadzone) {
        return 0.0f;
    }
    // Rescale so the stick still reaches full range once past the deadzone.
    const float sign = normalized < 0.0f ? -1.0f : 1.0f;
    const float magnitude = (std::abs(normalized) - kDeadzone) / (1.0f - kDeadzone);
    return sign * magnitude * kN64Range;
}

bool get_input(int controller_num, uint16_t* buttons, float* x, float* y) {
    static const bool twoPlayerFixture=[]() { const char *v=std::getenv("WR64_TEST_PLAYERS");return v && std::strcmp(v,"2")==0; }();
    if (controller_num==1 && twoPlayerFixture) { *buttons=BTN_A; *x=0; *y=0; return true; }
    if (controller_num != 0) {
        return false;
    }

#if WR64_WITH_FRONTEND
    // While a menu has input, the game gets none. Otherwise the button that
    // closes a menu also reaches the game behind it -- Start to leave the
    // settings would pause the race underneath.
    if (wr64::frontend::capturing_input() && !wr64::input_script_exclusive()) {
        *buttons = 0;
        *x = 0.0f;
        *y = 0.0f;
        return true;
    }
#endif

    uint16_t pressed = 0;
    float stick_x = 0.0f;
    float stick_y = 0.0f;

    if (g_controller != nullptr) {
        struct { SDL_GameControllerButton sdl; uint16_t n64; } map[] = {
            { SDL_CONTROLLER_BUTTON_A,             BTN_A },
            { SDL_CONTROLLER_BUTTON_X,             BTN_B },
            { SDL_CONTROLLER_BUTTON_START,         BTN_START },
            { SDL_CONTROLLER_BUTTON_LEFTSHOULDER,  BTN_L },
            { SDL_CONTROLLER_BUTTON_RIGHTSHOULDER, BTN_R },
            { SDL_CONTROLLER_BUTTON_DPAD_UP,       BTN_DUP },
            { SDL_CONTROLLER_BUTTON_DPAD_DOWN,     BTN_DDOWN },
            { SDL_CONTROLLER_BUTTON_DPAD_LEFT,     BTN_DLEFT },
            { SDL_CONTROLLER_BUTTON_DPAD_RIGHT,    BTN_DRIGHT },
        };
        for (const auto& entry : map) {
            if (SDL_GameControllerGetButton(g_controller, entry.sdl)) {
                pressed |= entry.n64;
            }
        }
        // Z lives on the left trigger; it is a button on the N64.
        if (SDL_GameControllerGetAxis(g_controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT) > 8000) {
            pressed |= BTN_Z;
        }
        // The C buttons map to the right stick, which is how every other N64
        // port does it and what players expect.
        const Sint16 cx = SDL_GameControllerGetAxis(g_controller, SDL_CONTROLLER_AXIS_RIGHTX);
        const Sint16 cy = SDL_GameControllerGetAxis(g_controller, SDL_CONTROLLER_AXIS_RIGHTY);
        if (cx < -16000) pressed |= BTN_CLEFT;
        if (cx >  16000) pressed |= BTN_CRIGHT;
        if (cy < -16000) pressed |= BTN_CUP;
        if (cy >  16000) pressed |= BTN_CDOWN;

        stick_x = axis_to_n64(SDL_GameControllerGetAxis(g_controller, SDL_CONTROLLER_AXIS_LEFTX));
        stick_y = -axis_to_n64(SDL_GameControllerGetAxis(g_controller, SDL_CONTROLLER_AXIS_LEFTY));
    }

    // Keyboard fallback, so the port is testable without a gamepad attached.
    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    if (keys != nullptr) {
        if (keys[SDL_SCANCODE_X])      pressed |= BTN_A;
        if (keys[SDL_SCANCODE_C])      pressed |= BTN_B;
        if (keys[SDL_SCANCODE_Z])      pressed |= BTN_Z;
        if (keys[SDL_SCANCODE_RETURN]) pressed |= BTN_START;
        if (keys[SDL_SCANCODE_A])      pressed |= BTN_L;
        if (keys[SDL_SCANCODE_S])      pressed |= BTN_R;
        // The C buttons work the camera, which Wave Race uses constantly, so
        // they need to be reachable without a pad. I/J/K/L keeps them under the
        // right hand while the left drives with the arrow keys.
        if (keys[SDL_SCANCODE_I])      pressed |= BTN_CUP;
        if (keys[SDL_SCANCODE_K])      pressed |= BTN_CDOWN;
        if (keys[SDL_SCANCODE_J])      pressed |= BTN_CLEFT;
        if (keys[SDL_SCANCODE_L])      pressed |= BTN_CRIGHT;
        if (keys[SDL_SCANCODE_T])      pressed |= BTN_DUP;
        if (keys[SDL_SCANCODE_G])      pressed |= BTN_DDOWN;
        if (keys[SDL_SCANCODE_F])      pressed |= BTN_DLEFT;
        if (keys[SDL_SCANCODE_H])      pressed |= BTN_DRIGHT;
        if (keys[SDL_SCANCODE_LEFT])   stick_x = -80.0f;
        if (keys[SDL_SCANCODE_RIGHT])  stick_x =  80.0f;
        if (keys[SDL_SCANCODE_UP])     stick_y =  80.0f;
        if (keys[SDL_SCANCODE_DOWN])   stick_y = -80.0f;
    }

    // Tick replays isolate hardware input; wall-time scripts can still be
    // nudged by hand. Script values use the documented N64 +/-80 range.
    uint16_t scripted_buttons = 0;
    float scripted_x = 0.0f;
    float scripted_y = 0.0f;
    wr64::input_script_state(&scripted_buttons, &scripted_x, &scripted_y);
    if (wr64::input_script_exclusive()) { pressed=0; stick_x=stick_y=0; }
    pressed |= scripted_buttons;
    if (scripted_x != 0.0f) { stick_x = scripted_x; }
    if (scripted_y != 0.0f) { stick_y = scripted_y; }

    *buttons = pressed;
    // ultramodern::convert_to_n64_range expects normalized input and performs
    // the final N64 octagonal-gate conversion itself.
    *x = stick_x / 80.0f;
    *y = stick_y / 80.0f;
    wr64::trace_input(controller_num,*buttons,*x,*y);
    return true;
}

void set_rumble(int controller_num, bool rumble) {
    // No Rumble Pak is advertised to this pre-Rumble-Pak game. Host effects
    // own both motors on the main thread; a guest boolean must not overwrite
    // their envelopes or invoke SDL through a borrowed pad on a game thread.
    (void)controller_num;
    (void)rumble;
}

ultramodern::input::connected_device_info_t get_connected_device_info(int controller_num) {
    static const bool twoPlayerFixture=[]() { const char *v=std::getenv("WR64_TEST_PLAYERS");return v && std::strcmp(v,"2")==0; }();
    if (controller_num == 0 || (controller_num==1 && twoPlayerFixture)) {
        return { ultramodern::input::Device::Controller, ultramodern::input::Pak::None };
    }
    return { ultramodern::input::Device::None, ultramodern::input::Pak::None };
}

// ---------------------------------------------------------------- audio ----
//
// The RSP's audio microcode writes finished stereo samples into RDRAM and the
// game hands them here; all that is left is to put them on the sound card.
//
// SDL's queue API is used rather than a pull callback because the interface
// ultramodern expects is a queue: the game asks how much is still buffered and
// decides how much more to generate from the answer. Mirroring that directly
// keeps the two in step, where a callback would need its own ring buffer in
// between and a second place for the sample count to drift.

SDL_AudioDeviceID g_audio_device = 0;
uint32_t g_audio_frequency = 32000;

// The N64 mixes 16-bit stereo, and the game's own sample rate is whatever it
// asks for through set_frequency.
constexpr int kAudioChannels = 2;
constexpr int kBytesPerFrame = kAudioChannels * static_cast<int>(sizeof(int16_t));

void close_audio_device() {
    if (g_audio_device != 0) {
        SDL_CloseAudioDevice(g_audio_device);
        g_audio_device = 0;
    }
}

// Opens (or reopens) the output device at the game's current sample rate.
//
// SDL is asked for exactly this format with SDL_AUDIO_ALLOW_ANY_CHANGE unset,
// so it resamples and converts internally if the hardware disagrees. Letting
// SDL change the format instead would mean converting here, and the sample rate
// is the one thing that must not silently differ: the game paces itself against
// how fast the queue drains, so a device running at 48000 while the game
// believes it is feeding 32000 makes the whole audio thread run at the wrong
// speed.
bool open_audio_device() {
    close_audio_device();

    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        std::fprintf(stderr, "[wr64] SDL_InitSubSystem(AUDIO) failed: %s\n", SDL_GetError());
        return false;
    }

    SDL_AudioSpec want{};
    want.freq = static_cast<int>(g_audio_frequency);
    want.format = AUDIO_S16SYS;
    want.channels = kAudioChannels;
    // Roughly a 60Hz frame's worth, rounded up to a power of two. Smaller than
    // the game's own buffering, so the queue depth we report back stays
    // dominated by what the game queued rather than by SDL's own latency.
    want.samples = 1024;
    want.callback = nullptr;  // queue-driven

    SDL_AudioSpec have{};
    g_audio_device = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
    if (g_audio_device == 0) {
        std::fprintf(stderr, "[wr64] SDL_OpenAudioDevice failed: %s\n", SDL_GetError());
        return false;
    }

    SDL_PauseAudioDevice(g_audio_device, 0);
    std::fprintf(stderr, "[wr64] audio device open at %d Hz, %d channels\n",
                 have.freq, have.channels);
    std::fflush(stderr);
    return true;
}

void queue_samples(int16_t* audio_data, size_t sample_count) {
    if (g_audio_device == 0) {
        return;
    }

    // sample_count counts individual 16-bit samples, not stereo frames -- see
    // ultramodern::queue_audio_buffer, which divides a byte count by
    // sizeof(int16_t).
    //
    // The two channels arrive swapped, and this is the one place in the project
    // where that is visible. librecomp stores RDRAM byte-swapped so that the
    // MEM_* macros can read big-endian N64 words as native little-endian ones,
    // which it does by XORing the low bits of every address. A pointer handed
    // out raw, as this one is, skips that: within each 32-bit word the two
    // 16-bit halves sit in the opposite order to the cartridge's. Each word
    // holds one left and one right sample, so the audible result is the stereo
    // image mirrored -- correct-sounding music with the channels reversed,
    // which is exactly the kind of bug that survives casual listening.
    // Mirrors the "first display list" log on the graphics side: it separates
    // "the game never asked for audio" from "the audio it asked for is silent",
    // which are different bugs with the same symptom.
    static bool announced_first = false;
    if (!announced_first) {
        announced_first = true;
        std::fprintf(stderr, "[wr64] first audio buffer queued: %zu frames at %u Hz\n",
                     sample_count / 2, g_audio_frequency);
        std::fflush(stderr);
    }

    static std::vector<int16_t> unswapped;
    unswapped.resize(sample_count);
    for (size_t i = 0; i + 1 < sample_count; i += 2) {
        unswapped[i + 0] = audio_data[i + 1];
        unswapped[i + 1] = audio_data[i + 0];
    }
    if (sample_count & 1) {
        unswapped[sample_count - 1] = audio_data[sample_count - 1];
    }

    wr64::music::mix(unswapped.data(), sample_count, g_audio_frequency);
    SDL_QueueAudio(g_audio_device, unswapped.data(),
                   static_cast<Uint32>(sample_count * sizeof(int16_t)));

    // A silent port and a working one queue samples equally often, so the
    // count alone proves nothing; the amplitude is what separates the microcode
    // actually mixing from it dutifully producing zeroes. Reported once, then
    // the scan stops paying for itself -- this runs on the audio thread.
    static bool announced = false;
    if (!announced) {
        for (size_t i = 0; i < sample_count; ++i) {
            if (unswapped[i] != 0) {
                announced = true;
                std::fprintf(stderr, "[wr64] audio is audible (first non-silent buffer"
                                     " after %zu frames)\n", sample_count / 2);
                std::fflush(stderr);
                break;
            }
        }
    }
}

size_t get_frames_remaining() {
    if (g_audio_device == 0) {
        // No device: report the queue permanently drained, so the game keeps
        // generating audio at its normal cadence instead of stalling on a
        // buffer that will never empty.
        return 0;
    }
    return SDL_GetQueuedAudioSize(g_audio_device) / kBytesPerFrame;
}

void set_frequency(uint32_t frequency) {
    if (frequency == g_audio_frequency && g_audio_device != 0) {
        return;
    }
    g_audio_frequency = frequency;
    open_audio_device();
}

// ------------------------------------------------------------------ rsp ----

// librecomp does not ask us to run a task; it asks which recompiled microcode
// function should run it. Graphics tasks never reach here -- ultramodern routes
// those to the renderer -- so what arrives is the audio microcode, aspMain.
//
// Phase 03 answered with a stub that reported every task finished without
// running it, which is why the port was silent. aspMain is now recompiled by
// RSPRecomp from the cartridge (see recomp/aspMain.rsp.toml), and this returns
// the real thing.

// Where aspMain's text sits in RDRAM, from the ELF symbol aspMainTextStart.
constexpr uint32_t kAspMainTextStart = 0x800D37B0;

// Retained for tasks that are not aspMain: reporting the task complete keeps
// the game running, where returning nullptr would make librecomp print and
// exit. Anything landing here is logged once, because a task we cannot run is
// worth knowing about rather than silently dropping.
RspExitReason rsp_unknown_ucode(uint8_t* rdram, uint32_t ucode_addr) {
    (void)rdram;
    static bool reported = false;
    if (!reported) {
        reported = true;
        std::fprintf(stderr, "[wr64] unrecognised RSP microcode at 0x%08X; "
                             "reporting its tasks complete without running them\n",
                     ucode_addr);
        std::fflush(stderr);
    }
    return RspExitReason::Broke;
}

// Runs the microcode with a watchdog on it, from a private copy of its command
// list, with a net under it.
//
// The watchdog. A microcode that spins forever is the worst failure this code
// has: it faults nothing, prints nothing and returns nothing, so the RSP task
// thread simply stops. The game then freezes with no error at all -- the
// scheduler waits for an SP-complete event that will never come, while every
// other thread carries on, so the audio thread keeps building tasks and the
// window keeps swapping the same finished frame. A wrong microcode load address
// produced exactly that during phase 05; the watchdog makes the next one
// announce itself.
//
// The private copy. The game double-buffers its audio command lists on the
// assumption that the RSP is done with a list long before that buffer's turn
// comes round again, two frames later. That holds on hardware, where the task
// takes about a millisecond, and it holds here of an optimized build. It did
// not hold of the Debug build this port ran as through phase 05 and most of
// phase 06: a task then took 5 to 24 ms against a 16.7 ms frame, and about one
// task in a thousand had its list rewritten by the game while the microcode was
// still DMAing it in, 0x140 bytes at a time. The microcode saw a splice of two
// frames' commands, and one splice in particular -- an ENVMIXER stripped of its
// own SETBUFFs, so inheriting the frame-end SAVEBUFF's 0x200-byte count --
// walked the wet-right channel buffer from 0xE40 off the end of DMEM, wrapping
// onto the command jump table at 0x10. That was the "audio frame dropped"
// click phase 05 could characterise but not place. Copying the list before the
// run costs a memcpy of a few KB per frame and makes the outcome independent
// of how late the task runs. The copy lives in the top of the 8 MB the runtime
// reports; this is a 4 MB cartridge that never reads osMemSize, so nothing of
// the game's is there.
//
// The net. librecomp treats a bad dispatch as fatal: it asserts, and the RSP
// task thread dies with it, stopping the game exactly as a hang would. DMEM is
// reloaded from RDRAM before every task, so the damage never outlives the task
// that caused it, and reporting the task complete costs one frame of audio
// rather than the run. With the copy in place it should never fire; if it
// does, bisect_audio_task explains which command did what.

// The command list of the audio task about to run, recorded by
// get_rsp_microcode so a failed task can be dumped afterwards.
uint32_t g_audio_task_data = 0;
uint32_t g_audio_task_size = 0;
uint32_t g_audio_task_original_ptr = 0;   // the game's buffer; g_audio_task_data may point at our copy

const char* rsp_exit_name(RspExitReason r) {
    switch (r) {
    case RspExitReason::Invalid: return "Invalid";
    case RspExitReason::Broke: return "Broke";
    case RspExitReason::ImemOverrun: return "ImemOverrun";
    case RspExitReason::UnhandledJumpTarget: return "UnhandledJumpTarget";
    case RspExitReason::Unsupported: return "Unsupported";
    case RspExitReason::SwapOverlay: return "SwapOverlay";
    case RspExitReason::UnhandledResumeTarget: return "UnhandledResumeTarget";
    }
    return "?";
}

// Dumps the task's audio command list (ABI 1: eight bytes per command, opcode in
// the top byte) so the command that overruns DMEM can be identified from what
// the game asked for rather than from where the store landed.
void dump_audio_task(const uint8_t* rdram, RspExitReason reason) {
    static const char* const kOps[16] = {
        "SPNOOP", "ADPCM", "CLEARBUFF", "ENVMIXER", "LOADBUFF", "RESAMPLE", "SAVEBUFF", "SEGMENT",
        "SETBUFF", "SETVOL", "DMEMMOVE", "LOADADPCM", "MIXER", "INTERLEAVE", "POLEF", "SETLOOP" };
    const uint32_t count = g_audio_task_size / 8;
    std::fprintf(stderr, "[wr64-audio] task failed with %s: %u commands at 0x%08X\n",
                 rsp_exit_name(reason), count, g_audio_task_data);
    for (uint32_t i = 0; i < count && i < 200; ++i) {
        const uint32_t addr = g_audio_task_data + i * 8;
        const uint32_t w0 = *reinterpret_cast<const uint32_t*>(rdram + (addr & 0x00FFFFFFu));
        const uint32_t w1 = *reinterpret_cast<const uint32_t*>(rdram + ((addr + 4) & 0x00FFFFFFu));
        const uint32_t op = w0 >> 24;
        std::fprintf(stderr, "[wr64-audio]  %3u %-10s f=%02X a=%04X  b=%04X c=%04X   (%08X %08X)\n",
                     i, kOps[op & 15], (w0 >> 16) & 0xFF, w0 & 0xFFFF, w1 >> 16, w1 & 0xFFFF, w0, w1);
    }
    std::fflush(stderr);
}

OSTask g_audio_task{};

// Re-runs the failed task with only its first `count` commands, exactly as
// librecomp sets a task up (the OSTask copied to DMEM 0xFC0, the microcode's
// data DMA'd to DMEM 0). DMEM is rebuilt from RDRAM on every run, so the
// corruption a failed run leaves behind does not carry over.
RspExitReason rerun_audio_task(uint8_t* rdram, uint32_t ucode_addr, uint32_t count) {
    OSTask copy = g_audio_task;
    copy.t.data_size = count * 8;
    std::memcpy(&dmem[0xFC0], &copy, sizeof(OSTask));
    dma_rdram_to_dmem(rdram, 0x0000, copy.t.ucode_data, 0xF80 - 1);
    return aspMain_run(rdram, ucode_addr);
}

}  // namespace
namespace {

// Finds the first command that damages DMEM outside the audio buffers: the
// microcode's constant pool (which holds the command jump table at 0x10) and
// the OSTask copy at 0xFC0. Each prefix of the command list is run from a
// fresh DMEM and both regions compared with what was loaded, so the first
// prefix that dirties either ends with the command that did it. A failed
// dispatch, by contrast, happens whenever the next command's table entry
// happens to be the corrupted one, which can be many commands later.
void bisect_audio_task(uint8_t* rdram, uint32_t ucode_addr) {
    const uint32_t total = g_audio_task_size / 8;
    uint8_t pristine[0x40];
    OSTask copy = g_audio_task;
    dma_rdram_to_dmem(rdram, 0x0000, copy.t.ucode_data, 0xF80 - 1);
    std::memcpy(pristine, dmem, sizeof(pristine));

    uint32_t bad = 0;
    const char* what = nullptr;
    bool top_reported = false;
    for (uint32_t n = 1; n <= total && what == nullptr; ++n) {
        const RspExitReason r = rerun_audio_task(rdram, ucode_addr, n);
        copy.t.data_size = n * 8;
        if (!top_reported && std::memcmp(&dmem[0xFC0], &copy, sizeof(OSTask)) != 0) {
            // Report once which bytes of the top of DMEM the microcode itself
            // writes (scratch use is legitimate; only what it says matters).
            top_reported = true;
            const uint8_t* a = &dmem[0xFC0];
            const uint8_t* b = reinterpret_cast<const uint8_t*>(&copy);
            uint32_t lo = 0x40, hi = 0;
            for (uint32_t i = 0; i < 0x40; ++i) {
                if (a[i] != b[i]) { lo = std::min(lo, i); hi = std::max(hi, i); }
            }
            std::fprintf(stderr, "[wr64-audio] (top of DMEM 0x%03X-0x%03X first written after command %u)\n",
                         0xFC0 + lo, 0xFC0 + hi, n - 1);
        }
        // Only the table itself (0x10-0x2F, the sixteen halfword entries
        // "lh $2, 0x10($2)" indexes into) matters for dispatch correctness.
        // The rest of this 0x40-byte window includes DMA chunk-remainder
        // bookkeeping that legitimately varies with how many bytes of the
        // command list get consumed -- comparing the whole window flagged
        // that as "corruption" and pointed at innocent commands.
        if (std::memcmp(&dmem[0x10], &pristine[0x10], 0x20) != 0) {
            what = "the constant pool / jump table at DMEM 0x000-0x040";
            // Byte-swizzled storage (see RSP_MEM_B in rsp.hpp): logical byte N
            // of DMEM lives at raw offset N^3. A plain reinterpret_cast here
            // would compare the wrong bytes; go through the same swizzle the
            // microcode's own loads and stores use.
            std::fprintf(stderr, "[wr64-audio] bytes changed in 0x000-0x040:");
            for (uint32_t i = 0; i < 0x40; ++i) {
                const uint8_t dv = dmem[i ^ 3];
                const uint8_t pv = pristine[i ^ 3];
                if (dv != pv) {
                    std::fprintf(stderr, " [0x%02X] %02X->%02X", i, pv, dv);
                }
            }
            std::fprintf(stderr, "\n");
        }
        else if (r != RspExitReason::Broke) {
            what = "nothing visible, yet the run failed";
        }
        bad = n;
    }
    if (what == nullptr) {
        std::fprintf(stderr, "[wr64-audio] every prefix ran clean on re-run: the failure depends on"
                             " state the first run changed, not on the command list alone\n");
        std::fflush(stderr);
        return;
    }
    std::fprintf(stderr, "[wr64-audio] first prefix to damage %s is %u commands:"
                         " command %u did it. Context:\n", what, bad, bad - 1);
    static const char* const kOps[16] = {
        "SPNOOP", "ADPCM", "CLEARBUFF", "ENVMIXER", "LOADBUFF", "RESAMPLE", "SAVEBUFF", "SEGMENT",
        "SETBUFF", "SETVOL", "DMEMMOVE", "LOADADPCM", "MIXER", "INTERLEAVE", "POLEF", "SETLOOP" };
    const uint32_t from = bad >= 40 ? bad - 40 : 0;
    for (uint32_t i = from; i < bad + 2 && i < total; ++i) {
        const uint32_t addr = g_audio_task_data + i * 8;
        const uint32_t w0 = *reinterpret_cast<const uint32_t*>(rdram + (addr & 0x00FFFFFFu));
        const uint32_t w1 = *reinterpret_cast<const uint32_t*>(rdram + ((addr + 4) & 0x00FFFFFFu));
        std::fprintf(stderr, "[wr64-audio]  %s%3u %-10s f=%02X a=%04X  b=%04X c=%04X   (%08X %08X)\n",
                     i == bad - 1 ? ">" : " ", i, kOps[(w0 >> 24) & 15], (w0 >> 16) & 0xFF, w0 & 0xFFFF,
                     w1 >> 16, w1 & 0xFFFF, w0, w1);
    }
    std::fflush(stderr);
}

// Where the private copy of the command list lives: physical 0x7E0000, in the
// top of the 8 MB the runtime reports. See the comment above.
constexpr uint32_t kCommandListScratch = 0x807E0000u;
constexpr uint32_t kCommandListScratchSize = 0x10000u;

RspExitReason asp_main_watched(uint8_t* rdram, uint32_t ucode_addr) {
    const uint32_t original_base = g_audio_task_original_ptr & 0x00FFFFFFu;

    // Run from a private copy (see above). Both addresses are 8-byte aligned, so
    // a raw copy preserves RDRAM's byte swizzling. The microcode reads the
    // list's address exactly once, from the OSTask librecomp placed at DMEM
    // 0xFC0, so redirecting that word is all it takes. The diagnostics below
    // are pointed at the copy too: it is what actually ran.
    if (g_audio_task_size <= kCommandListScratchSize) {
        std::memcpy(rdram + (kCommandListScratch & 0x00FFFFFFu), rdram + original_base, g_audio_task_size);
        RSP_MEM_W_STORE(0x30, 0xFC0, kCommandListScratch);
        g_audio_task_data = kCommandListScratch;
        g_audio_task.t.data_ptr = kCommandListScratch;
    }

    // Health metric, kept on purpose: did the game rewrite the original list
    // while the task ran? Harmless now, but it is the measurement that found
    // the bug, and a machine slow enough to trip it is worth knowing about.
    static std::vector<uint8_t> before;
    before.assign(rdram + original_base, rdram + original_base + g_audio_task_size);

    wr64::watch_for_hang("the audio microcode", 5);
    const RspExitReason reason = aspMain_run(rdram, ucode_addr);
    wr64::watch_done();

    {
        static uint64_t changed_runs = 0, runs = 0;
        ++runs;
        if (std::memcmp(before.data(), rdram + original_base, g_audio_task_size) != 0) {
            ++changed_runs;
            if (changed_runs <= 3) {
                std::fprintf(stderr, "[wr64] the game rewrote an audio command list while its task was running"
                                     " (%llu of %llu tasks so far). The task ran from its own copy, so nothing was"
                                     " lost, but audio tasks are running more than a frame late on this machine.\n",
                             static_cast<unsigned long long>(changed_runs),
                             static_cast<unsigned long long>(runs));
                std::fflush(stderr);
            }
        }
    }

    if (reason != RspExitReason::Broke) {
        static uint64_t dropped = 0;
        ++dropped;
        if (dropped <= 40 || dropped % 100 == 0) {
            std::fprintf(stderr, "[wr64] audio frame dropped: the microcode did not reach"
                                 " its break (%llu so far). See src/callbacks.cpp.\n",
                         static_cast<unsigned long long>(dropped));
            std::fflush(stderr);
        }
        if (dropped <= 6) {
            std::fprintf(stderr, "[wr64-audio] task failed with %s: %u commands at 0x%08X\n",
                         rsp_exit_name(reason), g_audio_task_size / 8, g_audio_task_data);
            bisect_audio_task(rdram, ucode_addr);
        }
        return RspExitReason::Broke;
    }
    return reason;
}

RspUcodeFunc* get_rsp_microcode(const OSTask* task) {
    // Match on the microcode's address rather than the task type: type numbers
    // are a game-level convention, while the address is what actually
    // identifies the code about to run.
    const uint32_t ucode = static_cast<uint32_t>(task->t.ucode) & 0x00FFFFFFu;
    if (ucode == (kAspMainTextStart & 0x00FFFFFFu)) {
        g_audio_task_original_ptr = static_cast<uint32_t>(task->t.data_ptr);
        g_audio_task_data = static_cast<uint32_t>(task->t.data_ptr);
        g_audio_task_size = task->t.data_size;
        g_audio_task = *task;
        return asp_main_watched;
    }
    static uint64_t others = 0;
    if (++others <= 5) {
        std::fprintf(stderr, "[wr64] non-audio RSP task: type %u ucode 0x%08X\n",
                     static_cast<unsigned>(task->t.type),
                     static_cast<uint32_t>(task->t.ucode));
        std::fflush(stderr);
    }
    return rsp_unknown_ucode;
}

// --------------------------------------------------------------- events ----

void vi_callback() {
}

void gfx_init_callback() {
}

// -------------------------------------------------------- error handling ----

void message_box(const char* msg) {
    std::fprintf(stderr, "%s\n", msg);
    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Wave Race 64: Recompiled", msg, g_window);
}

// -------------------------------------------------------------- threads ----

std::string get_game_thread_name(const OSThread* t) {
    // Kept under 16 bytes including the terminator, which is the limit on
    // several platforms.
    char buf[16];
    std::snprintf(buf, sizeof(buf), "wr64_%d", t != nullptr ? t->id : -1);
    return std::string(buf);
}

// ------------------------------------------------------------------ gfx ----

ultramodern::gfx_callbacks_t::gfx_data_t create_gfx() {
    if (SDL_InitSubSystem(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "SDL_InitSubSystem failed: %s\n", SDL_GetError());
    }
    return nullptr;
}

ultramodern::renderer::WindowHandle create_window(ultramodern::gfx_callbacks_t::gfx_data_t) {
    // Size the window, and decide whether it opens fullscreen.
    //
    // Fullscreen is not just a flag here: RT64 derives the aspect ratio it
    // expands the game into from the swap chain's dimensions. A window created
    // at a 4:3 multiple gives it a 4:3 swap chain, and "Expand" then has
    // nothing to expand into -- the game stays pillarboxed however wide the
    // display is. Opening at the display's own size means the swap chain is the
    // display's shape from the first frame.
    //
    // Windowed, the largest whole multiple of 320x240 that fits is used rather
    // than a hardcoded size. A fixed 2x of 640x480 is 1280x960, taller than a
    // 1536x864 laptop panel: Windows then places the window partly off-screen
    // and the game is cropped with no indication anything is wrong.
    const bool fullscreen = ultramodern::renderer::get_graphics_config().wm_option ==
                            ultramodern::renderer::WindowMode::Fullscreen;

    int width = 320 * 4;
    int height = 240 * 4;
    Uint32 flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
#if defined(__APPLE__)
    flags |= SDL_WINDOW_METAL;
#elif defined(__linux__)
    flags |= SDL_WINDOW_VULKAN;
#endif

    SDL_Rect display{};
    if (fullscreen && SDL_GetDisplayBounds(0, &display) == 0) {
        width = display.w;
        height = display.h;
        flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
        std::fprintf(stderr, "[wr64] window %dx%d fullscreen (the display's own size)\n",
                     width, height);
        std::fflush(stderr);
    }
    else {
        SDL_Rect usable{};
        if (SDL_GetDisplayUsableBounds(0, &usable) == 0) {
            int scale = 4;
            while (scale > 1 && (320 * scale > usable.w || 240 * scale > usable.h)) {
                --scale;
            }
            width = 320 * scale;
            height = 240 * scale;
            std::fprintf(stderr, "[wr64] window %dx%d (%dx upscale; display has %dx%d usable)\n",
                         width, height, scale, usable.w, usable.h);
            std::fflush(stderr);
        }
    }

    g_window = SDL_CreateWindow("Wave Race 64: Recompiled",
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                width, height, flags);
    if (g_window == nullptr) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return {};
    }

    SDL_AddEventWatch([](void *, SDL_Event *event) -> int {
        if (event->type == SDL_KEYDOWN && !event->key.repeat) {
            if (event->key.keysym.sym == SDLK_F9) wr64::water::toggle();
            if (event->key.keysym.sym == SDLK_F10) wr64::water::cycle_debug();
        }
        return 0;
    }, nullptr);

#if WR64_WITH_FRONTEND
    // recompui reads the window through a global of its own; publish ours so
    // the UI measures and draws into the same one the game does.
    wr64::frontend::publish_window(g_window);
#endif

    // Present the region the game draws into, not the framebuffer's borders.
    // The renderer does not exist yet; this only sets what it will read.
    wr64::display::crop_to_content();

#if defined(_WIN32) || defined(__APPLE__)
    SDL_SysWMinfo wm_info;
    SDL_VERSION(&wm_info.version);
    if (SDL_GetWindowWMInfo(g_window, &wm_info) != SDL_TRUE) {
        std::fprintf(stderr, "SDL_GetWindowWMInfo failed: %s\n", SDL_GetError());
        return {};
    }
#if defined(__APPLE__)
    SDL_MetalView view = SDL_Metal_CreateView(g_window);
    if (!view) {
        std::fprintf(stderr, "SDL_Metal_CreateView failed: %s\n", SDL_GetError());
        return {};
    }
    // Plume's view field expects the CAMetalLayer, not SDL's NSView wrapper.
    return {wm_info.info.cocoa.window, SDL_Metal_GetLayer(view)};
#else
    return ultramodern::renderer::WindowHandle{ wm_info.info.win.window, GetCurrentThreadId() };
#endif
#else
    return g_window;
#endif
}

void update_gfx(ultramodern::gfx_callbacks_t::gfx_data_t) {
    // This is librecomp's main loop body, so counting it distinguishes "the
    // loop never ran" from "the loop ran and then something ended it".
    static uint64_t ticks = 0;
    if (ticks == 0 || ticks == 1000 || ticks == 10000) {
        std::fprintf(stderr, "[wr64] main loop tick %llu\n",
                     static_cast<unsigned long long>(ticks));
        std::fflush(stderr);
    }
    ++ticks;
    poll_input();
    bool feedback_allowed = g_window && (SDL_GetWindowFlags(g_window) & SDL_WINDOW_INPUT_FOCUS);
#if WR64_WITH_FRONTEND
    feedback_allowed = feedback_allowed && !wr64::frontend::capturing_input();
#endif
    wr64::haptics::update_output(g_controller, feedback_allowed);
    wr64::poll_game_state();
}

}  // namespace

namespace wr64 {

ultramodern::input::callbacks_t input_callbacks() {
    return { poll_input, get_input, set_rumble, get_connected_device_info };
}

ultramodern::audio_callbacks_t audio_callbacks() {
    return { queue_samples, get_frames_remaining, set_frequency };
}

recomp::rsp::callbacks_t rsp_callbacks() {
    return { get_rsp_microcode };
}

ultramodern::gfx_callbacks_t gfx_callbacks() {
    return { create_gfx, create_window, update_gfx };
}

ultramodern::events::callbacks_t events_callbacks() {
    return { vi_callback, gfx_init_callback };
}

ultramodern::error_handling::callbacks_t error_handling_callbacks() {
    return { message_box };
}

ultramodern::threads::callbacks_t threads_callbacks() {
    return { get_game_thread_name };
}

ultramodern::renderer::callbacks_t renderer_callbacks() {
    ultramodern::renderer::callbacks_t callbacks{};
    callbacks.create_render_context = create_render_context;
    return callbacks;
}

void shutdown_platform() {
    wr64::haptics::shutdown(g_controller);
    if (g_controller != nullptr) {
        SDL_GameControllerClose(g_controller);
        g_controller = nullptr;
    }
    if (g_window != nullptr) {
        SDL_DestroyWindow(g_window);
        g_window = nullptr;
    }
    SDL_Quit();
}

}  // namespace wr64
