#pragma once

#include <cstdint>
#include <string>

class Pca9685Controller {
public:
    // Typical address 0x40, frequency 50 Hz. Use the I2C bus shown by i2cdetect.
    explicit Pca9685Controller(const std::string& device = "/dev/i2c-1",
                               int address = 0x40,
                               double frequency_hz = 50.0);
    ~Pca9685Controller();

    Pca9685Controller(const Pca9685Controller&) = delete;
    Pca9685Controller& operator=(const Pca9685Controller&) = delete;

    void setPwmFrequency(double frequency_hz);
    void setPwm(int channel, int on_tick, int off_tick);
    void setServoPulseUs(int channel, double pulse_us);
    void setServoAngle(int channel, double angle_deg,
                       double min_angle_deg, double max_angle_deg,
                       double min_pulse_us = 1000.0,
                       double max_pulse_us = 2000.0);
    void allOff();

private:
    void initialize();
    void write8(uint8_t reg, uint8_t value);
    uint8_t read8(uint8_t reg);

    std::string device_;
    int address_;
    int fd_;
    double frequency_hz_;
};
