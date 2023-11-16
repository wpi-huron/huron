#pragma once

#include <memory>

#include "serial.h"
#include <serial/serial.h>

namespace huron {
namespace driver {
namespace serial {

class Serial : public SerialBase{
 public:
  Serial(std::string port,
         uint32_t baudrate,
         Parity parity,
         StopBits stopbits,
         FlowControl flowcontrol);
  Serial(const Serial&) = delete;
  Serial& operator=(const Serial&) = delete;
  virtual ~Serial() = default;

  void Open() override;
  bool IsOpen() override;
  void Close() override;
  size_t Available() override;
  bool WaitReadable() override;
  size_t Read(uint8_t *buffer, size_t nbytes) override;
  size_t Read(std::vector<uint8_t> &buffer, size_t nbytes = 1) override;
  size_t Read(std::string &buffer, size_t nbytes = 1) override;
  std::string Read(size_t nbytes = 1) override;
  size_t ReadLine(std::string &buffer,
                  size_t nbytes = 65536,
                  std::string eol = "\n") override;
  std::string ReadLine(size_t nbytes = 65536,
                       std::string eol = "\n") override;
  std::vector<std::string> ReadLines(size_t nbytes = 65536,
                                     std::string eol = "\n") override;
  size_t Write(const uint8_t *data, size_t nbytes) override;
  size_t Write(const std::vector<uint8_t> &data) override;
  size_t Write(const std::string &data) override;
  void SetPort(const std::string &port) override;
  std::string GetPort() const override;
  void SetTimeout(uint32_t inter_byte_timeout,
                  uint32_t read_timeout_constant,
                  uint32_t read_timeout_multiplier,
                  uint32_t write_timeout_constant,
                  uint32_t write_timeout_multiplier) override;
  void SetBaudrate(uint32_t baudrate) override;
  uint32_t GetBaudrate() const override;
  void SetParity(Parity parity) override;
  Parity GetParity() const override;
  void SetStopbits(StopBits stopbits) override;
  StopBits GetStopbits() const override;
  void SetFlowcontrol(FlowControl flowcontrol) override;
  FlowControl GetFlowcontrol() const override;
  void Flush() override;
  void FlushInput() override;
  void FlushOutput() override;
  void SendBreak(int duration) override;

 private:
  std::unique_ptr<::serial::Serial> wjwwood_serial_;

  static ::serial::flowcontrol_t ConvertFlowControl(FlowControl flowcontrol) {
    return static_cast<::serial::flowcontrol_t>(flowcontrol);
  }
  static ::serial::parity_t ConvertParity(Parity parity) {
    return static_cast<::serial::parity_t>(parity);
  }
  static ::serial::stopbits_t ConvertStopBits(StopBits stopbits) {
    return static_cast<::serial::stopbits_t>(stopbits);
  }
};

}  // namespace serial
}  // namespace driver
}  // namespace huron
