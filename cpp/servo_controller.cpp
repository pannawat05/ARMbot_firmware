#include "servo_controller.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>

// PCA9685 register map
namespace {
constexpr uint8_t MODE1      = 0x00;
constexpr uint8_t MODE2      = 0x01;
constexpr uint8_t PRESCALE    = 0xFE;
constexpr uint8_t LED0_ON_L   = 0x06;
constexpr uint8_t ALL_LED_OFF_H = 0xFD;
constexpr uint8_t RESTART     = 0x80;
constexpr uint8_t SLEEP       = 0x10;
constexpr uint8_t AI          = 0x20;
constexpr uint8_t OUTDRV      = 0x04;

void sleep_us(unsigned int us) { usleep(us); }
}

Pca9685Controller::Pca9685Controller(const std::string& device, int address, double frequency_hz)
    : device_(device), address_(address), fd_(-1), frequency_hz_(frequency_hz) {
    if (address < 0x03 || address > 0x77) {
        throw std::invalid_argument("I2C address must be between 0x03 and 0x77");
    }
    if (frequency_hz < 24.0 || frequency_hz > 1526.0) {
        throw std::invalid_argument("PCA9685 frequency out of supported range");
    }

    fd_ = ::open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        throw std::runtime_error("Cannot open " + device_ + ": " + std::strerror(errno));
    }
    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        const std::string err = std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("Cannot select I2C address: " + err);
    }

    initialize();
}

Pca9685Controller::~Pca9685Controller() {
    if (fd_ >= 0) ::close(fd_);
}

void Pca9685Controller::write8(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    if (::write(fd_, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data))) {
        throw std::runtime_error("I2C write failed: " + std::string(std::strerror(errno)));
    }
}

uint8_t Pca9685Controller::read8(uint8_t reg) {
    if (::write(fd_, &reg, 1) != 1) {
        throw std::runtime_error("I2C register select failed");
    }
    uint8_t value = 0;
    if (::read(fd_, &value, 1) != 1) {
        throw std::runtime_error("I2C read failed");
    }
    return value;
}

void Pca9685Controller::initialize() {
    write8(MODE1, 0x00);
    write8(MODE2, OUTDRV);
    setPwmFrequency(frequency_hz_);
    allOff();
}

void Pca9685Controller::setPwmFrequency(double frequency_hz) {
    // PCA9685 oscillator is nominally 25 MHz; prescale register is rounded.
    const int prescale = static_cast<int>(std::round(25000000.0 / (4096.0 * frequency_hz) - 1.0));
    const uint8_t old_mode = read8(MODE1);
    const uint8_t sleep_mode = static_cast<uint8_t>((old_mode & 0x7F) | SLEEP);
    write8(MODE1, sleep_mode);
    write8(PRESCALE, static_cast<uint8_t>(std::clamp(prescale, 3, 255)));
    write8(MODE1, old_mode);
    sleep_us(5000);
    write8(MODE1, static_cast<uint8_t>(old_mode | RESTART | AI));
    frequency_hz_ = frequency_hz;
}

void Pca9685Controller::setPwm(int channel, int on_tick, int off_tick) {
    if (channel < 0 || channel > 15) throw std::out_of_range("PCA9685 channel must be 0..15");
    if (on_tick < 0 || on_tick > 4095 || off_tick < 0 || off_tick > 4095)
        throw std::out_of_range("PWM ticks must be 0..4095");

    const uint8_t reg = static_cast<uint8_t>(LED0_ON_L + 4 * channel);
    uint8_t data[5] = {
        reg,
        static_cast<uint8_t>(on_tick & 0xFF),
        static_cast<uint8_t>((on_tick >> 8) & 0x0F),
        static_cast<uint8_t>(off_tick & 0xFF),
        static_cast<uint8_t>((off_tick >> 8) & 0x0F)
    };
    if (::write(fd_, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data))) {
        throw std::runtime_error("I2C PWM write failed: " + std::string(std::strerror(errno)));
    }
}

void Pca9685Controller::setServoPulseUs(int channel, double pulse_us) {
    // Servo pulse widths are hardware-specific. Validate conservative range first.
    if (!std::isfinite(pulse_us) || pulse_us < 500.0 || pulse_us > 2500.0) {
        throw std::out_of_range("Pulse must be within conservative 500..2500 us range");
    }
    const double period_us = 1000000.0 / frequency_hz_;
    const int off_tick = static_cast<int>(std::round(pulse_us * 4096.0 / period_us));
    setPwm(channel, 0, std::clamp(off_tick, 0, 4095));
}

void Pca9685Controller::setServoAngle(int channel, double angle_deg,
                                      double min_angle_deg, double max_angle_deg,
                                      double min_pulse_us, double max_pulse_us) {
    if (max_angle_deg <= min_angle_deg || max_pulse_us <= min_pulse_us) {
        throw std::invalid_argument("Invalid angle or pulse calibration range");
    }
    if (!std::isfinite(angle_deg) || angle_deg < min_angle_deg || angle_deg > max_angle_deg) {
        throw std::out_of_range("Requested angle is outside configured limits");
    }
    const double ratio = (angle_deg - min_angle_deg) / (max_angle_deg - min_angle_deg);
    const double pulse = min_pulse_us + ratio * (max_pulse_us - min_pulse_us);
    setServoPulseUs(channel, pulse);
}

void Pca9685Controller::allOff() {
    // ALL_LED_OFF_H bit 4 sets full-off for every channel.
    write8(ALL_LED_OFF_H, 0x10);
}
