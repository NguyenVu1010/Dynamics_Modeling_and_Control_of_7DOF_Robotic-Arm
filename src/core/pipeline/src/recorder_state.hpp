#pragma once

#include <cstdint>

// RecorderControl::swap_state values, shared by Recorder (recorder.cpp) and the black-box export
// that reads a region written by a Recorder (recorder_shm.cpp).
namespace cobot::pipeline::recorder_state {

constexpr std::uint32_t kSwapIdle = 0;       // no swap pending, no retired buffer held
constexpr std::uint32_t kSwapRequested = 1;  // the next record() swaps
constexpr std::uint32_t kSwapDone = 2;       // the other buffer is retired until released

}  // namespace cobot::pipeline::recorder_state
