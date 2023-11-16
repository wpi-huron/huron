#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>

namespace huron {
namespace driver {
namespace serial {

enum class Parity {
  None = 0,
  Odd = 1,
  Even = 2,
  Mark = 3,
  Space = 4
};

enum class StopBits {
  One = 1,
  Two = 2,
  OnePointFive
};

enum class FlowControl {
  None = 0,
  Software,
  Hardware
};

class SerialBase {
 public:
  SerialBase(std::string port,
             uint32_t baudrate,
             Parity parity,
             StopBits stopbits,
             FlowControl flowcontrol) :
    port_(port),
    baudrate_(baudrate),
    parity_(parity),
    stopbits_(stopbits),
    flowcontrol_(flowcontrol) {}
  SerialBase(const SerialBase&) = delete;
  SerialBase& operator=(const SerialBase&) = delete;
  virtual ~SerialBase() = default;

  virtual void Open() = 0;
  virtual bool IsOpen() = 0;
  virtual void Close() = 0;
  virtual size_t Available() = 0;
  virtual bool WaitReadable() = 0;
  virtual size_t Read(uint8_t *buffer, size_t nbytes) = 0;
  virtual size_t Read(std::vector<uint8_t> &buffer, size_t nbytes = 1) = 0;
  virtual size_t Read(std::string &buffer, size_t nbytes = 1) = 0;
  virtual std::string Read(size_t nbytes = 1) = 0;
  virtual size_t ReadLine(std::string &buffer,
                          size_t nbytes = 65536,
                          std::string eol = "\n") = 0;
  virtual std::string ReadLine(size_t nbytes = 65536,
                               std::string eol = "\n") = 0;
  virtual std::vector<std::string> ReadLines(size_t nbytes = 65536,
                                             std::string eol = "\n") = 0;
  virtual size_t Write(const uint8_t *data, size_t nbytes) = 0;
  virtual size_t Write(const std::vector<uint8_t> &data) = 0;
  virtual size_t Write(const std::string &data) = 0;
  virtual void SetPort(const std::string &port) = 0;
  virtual std::string GetPort() const = 0;
  virtual void SetTimeout(uint32_t inter_byte_timeout,
                          uint32_t read_timeout_constant,
                          uint32_t read_timeout_multiplier,
                          uint32_t write_timeout_constant,
                          uint32_t write_timeout_multiplier) = 0;
  virtual void SetBaudrate(uint32_t baudrate) = 0;
  virtual uint32_t GetBaudrate() const = 0;
  virtual void SetParity(Parity parity) = 0;
  virtual Parity GetParity() const = 0;
  virtual void SetStopbits(StopBits stopbits) = 0;
  virtual StopBits GetStopbits() const = 0;
  virtual void SetFlowcontrol(FlowControl flowcontrol) = 0;
  virtual FlowControl GetFlowcontrol() const = 0;
  virtual void Flush() = 0;
  virtual void FlushInput() = 0;
  virtual void FlushOutput() = 0;
  virtual void SendBreak(int duration) = 0;

 protected:
  std::string port_;
  uint32_t baudrate_;
  Parity parity_;
  StopBits stopbits_;
  FlowControl flowcontrol_;
};

}  // namespace serial
}  // namespace driver
}  // namespace huron
