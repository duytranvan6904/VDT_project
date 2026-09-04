#pragma once
#include <array>
#include <cstdint>
#include "rc_parser/rc_types.hpp"

namespace rc_parser
{

constexpr size_t SBUS_FRAME_LEN = 25;
constexpr uint8_t SBUS_START_BYTE = 0x0F;

RcChannels sbus_decode_frame(const std::array<uint8_t, SBUS_FRAME_LEN> & frame);

}  // namespace rc_parser