#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "rc_parser/rc_sbus.hpp"

namespace rc_parser
{

class SbusUart
{
public:
  SbusUart(const std::string & device, int baudrate);
  ~SbusUart();

  bool is_open() const;
  bool read_frame(std::array<uint8_t, SBUS_FRAME_LEN> & frame_out);

private:
  bool configure_port(int baudrate);

  int fd_;
  std::vector<uint8_t> buffer_;
};

}  // namespace rc_parser