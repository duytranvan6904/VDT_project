#include "rc_parser/rc_uart.hpp"
#include <algorithm>
#include <asm/termios.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace rc_parser
{

SbusUart::SbusUart(const std::string & device, int baudrate)
: fd_(-1)
{
  fd_ = open(device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd_ >= 0 && !configure_port(baudrate)) {
    close(fd_);
    fd_ = -1;
  }
}

SbusUart::~SbusUart()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool SbusUart::is_open() const
{
  return fd_ >= 0;
}

bool SbusUart::configure_port(int baudrate)
{
  struct termios2 tio{};
  if (ioctl(fd_, TCGETS2, &tio) != 0) {
    return false;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;

  tio.c_cflag &= ~PARODD;
  tio.c_cflag |= PARENB;
  tio.c_cflag |= CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag |= CLOCAL | CREAD;

  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;

  return ioctl(fd_, TCSETS2, &tio) == 0;
}

bool SbusUart::read_frame(std::array<uint8_t, SBUS_FRAME_LEN> & frame_out)
{
  if (fd_ < 0) {
    return false;
  }
  uint8_t byte;
  while (true) {
    const ssize_t result = read(fd_, &byte, 1);
    if (result != 1) {
      if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return false;
      }
      return false;
    }
    buffer_.push_back(byte);
    if (buffer_.size() > SBUS_FRAME_LEN) {
      buffer_.erase(buffer_.begin());
    }
    if (buffer_.size() == SBUS_FRAME_LEN && buffer_.front() == SBUS_START_BYTE) {
      std::copy(buffer_.begin(), buffer_.end(), frame_out.begin());
      buffer_.clear();
      return true;
    }
  }
  return false;
}

}  // namespace rc_parser