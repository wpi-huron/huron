#include "huron/driver/serial/wjwwood_serial.h"
#include <utility>
#include "serial/serial.h"

namespace huron {
namespace driver {
namespace serial {

Serial::Serial(std::string port,
               uint32_t baudrate,
               Parity parity,
               StopBits stopbits,
               FlowControl flowcontrol)
  : SerialBase(port, baudrate, parity, stopbits, flowcontrol),
    wjwwood_serial_(std::make_unique<::serial::Serial>(
      port,
      baudrate,
      ::serial::Timeout(),
      ::serial::eightbits,
      ConvertParity(parity),
      ConvertStopBits(stopbits),
      ConvertFlowControl(flowcontrol))) {}


void Serial::Open() {
  wjwwood_serial_->open();
}

bool Serial::IsOpen() {
  return wjwwood_serial_->isOpen();
}

void Serial::Close() {
  wjwwood_serial_->close();
}

size_t Serial::Available() {
  return wjwwood_serial_->available();
}

bool Serial::WaitReadable() {
  return wjwwood_serial_->waitReadable();
}

size_t Serial::Read(uint8_t *buffer, size_t nbytes) {
  return wjwwood_serial_->read(buffer, nbytes);
}

size_t Serial::Read(std::vector<uint8_t> &buffer, size_t nbytes) {
  return wjwwood_serial_->read(buffer, nbytes);
}

size_t Serial::Read(std::string &buffer, size_t nbytes) {
  return wjwwood_serial_->read(buffer, nbytes);
}

std::string Serial::Read(size_t nbytes) {
  return wjwwood_serial_->read(nbytes);
}

size_t Serial::ReadLine(std::string &buffer,
                        size_t nbytes,
                        std::string eol) {
  return wjwwood_serial_->readline(buffer, nbytes, eol);
}

std::string Serial::ReadLine(size_t nbytes, std::string eol) {
  return wjwwood_serial_->readline(nbytes, eol);
}

std::vector<std::string> Serial::ReadLines(size_t nbytes, std::string eol) {
  return wjwwood_serial_->readlines(nbytes, eol);
}

size_t Serial::Write(const uint8_t *data, size_t nbytes) {
  return wjwwood_serial_->write(data, nbytes);
}

size_t Serial::Write(const std::vector<uint8_t> &data) {
  return wjwwood_serial_->write(data);
}

size_t Serial::Write(const std::string &data) {
  return wjwwood_serial_->write(data);
}

void Serial::SetPort(const std::string &port) {
  wjwwood_serial_->setPort(port);
  port_.assign(port);
}

std::string Serial::GetPort() const {
  return port_;
}

void Serial::SetTimeout(uint32_t inter_byte_timeout,
                        uint32_t read_timeout_constant,
                        uint32_t read_timeout_multiplier,
                        uint32_t write_timeout_constant,
                        uint32_t write_timeout_multiplier) {
  wjwwood_serial_->setTimeout(inter_byte_timeout,
                              read_timeout_constant,
                              read_timeout_multiplier,
                              write_timeout_constant,
                              write_timeout_multiplier);
}

void Serial::SetBaudrate(uint32_t baudrate) {
  wjwwood_serial_->setBaudrate(baudrate);
  baudrate_ = baudrate;
}

uint32_t Serial::GetBaudrate() const {
  return baudrate_;
}

void Serial::SetParity(Parity parity) {
  wjwwood_serial_->setParity(ConvertParity(parity));
  parity_ = parity;
}

Parity Serial::GetParity() const {
  return parity_;
}

void Serial::SetStopbits(StopBits stopbits) {
  wjwwood_serial_->setStopbits(ConvertStopBits(stopbits));
  stopbits_ = stopbits;
}

StopBits Serial::GetStopbits() const {
  return stopbits_;
}

void Serial::SetFlowcontrol(FlowControl flowcontrol) {
  wjwwood_serial_->setFlowcontrol(ConvertFlowControl(flowcontrol));
  flowcontrol_ = flowcontrol;
}

FlowControl Serial::GetFlowcontrol() const {
  return flowcontrol_;
}

void Serial::Flush() {
  wjwwood_serial_->flush();
}

void Serial::FlushInput() {
  wjwwood_serial_->flushInput();
}

void Serial::FlushOutput() {
  wjwwood_serial_->flushOutput();
}

void Serial::SendBreak(int duration) {
  wjwwood_serial_->sendBreak(duration);
}

}  // namespace serial
}  // namespace driver
}  // namespace huron
