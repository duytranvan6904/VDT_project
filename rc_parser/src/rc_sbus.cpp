#include "rc_parser/rc_sbus.hpp"

namespace rc_parser
{

namespace
{
constexpr uint8_t SBUS_FAILSAFE_BIT = 0x08;

int sbus_to_us(uint16_t raw)
{
  return static_cast<int>(raw * 0.625f) + 880;
}
}  // namespace

RcChannels sbus_decode_frame(const std::array<uint8_t, SBUS_FRAME_LEN> & frame)
{
  RcChannels out;

  const uint16_t raw[16] = {
    static_cast<uint16_t>((frame[1] | frame[2] << 8) & 0x07FF),
    static_cast<uint16_t>((frame[2] >> 3 | frame[3] << 5) & 0x07FF),
    static_cast<uint16_t>((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF),
    static_cast<uint16_t>((frame[5] >> 1 | frame[6] << 7) & 0x07FF),
    static_cast<uint16_t>((frame[6] >> 4 | frame[7] << 4) & 0x07FF),
    static_cast<uint16_t>((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF),
    static_cast<uint16_t>((frame[9] >> 2 | frame[10] << 6) & 0x07FF),
    static_cast<uint16_t>((frame[10] >> 5 | frame[11] << 3) & 0x07FF),
    static_cast<uint16_t>((frame[12] | frame[13] << 8) & 0x07FF),
    static_cast<uint16_t>((frame[13] >> 3 | frame[14] << 5) & 0x07FF),
    static_cast<uint16_t>((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF),
    static_cast<uint16_t>((frame[16] >> 1 | frame[17] << 7) & 0x07FF),
    static_cast<uint16_t>((frame[17] >> 4 | frame[18] << 4) & 0x07FF),
    static_cast<uint16_t>((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF),
    static_cast<uint16_t>((frame[20] >> 2 | frame[21] << 6) & 0x07FF),
    static_cast<uint16_t>((frame[21] >> 5 | frame[22] << 3) & 0x07FF)
  };

  for (int i = 0; i < 16; ++i) {
    out.ch[i] = sbus_to_us(raw[i]);
  }

  out.failsafe = (frame[23] & SBUS_FAILSAFE_BIT) != 0;
  out.valid = true;
  return out;
}

}  // namespace rc_parser