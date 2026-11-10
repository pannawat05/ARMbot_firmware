// Set center (90 degrees) for a servo on a PCA9685.
//
// Usage: servo_health_test [i2c_dev] [addr] [channel] [min_us] [max_us]
//   defaults: /dev/i2c-1 0x40 0 1000 2000

#include "servo_controller.h"

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string dev = argc > 1 ? argv[1] : "/dev/i2c-1";
    const int addr = argc > 2 ? static_cast<int>(std::strtol(argv[2], nullptr, 0)) : 0x40;
    const int ch = argc > 3 ? std::atoi(argv[3]) : 0;
    const double min_us = argc > 4 ? std::atof(argv[4]) : 1000.0;
    const double max_us = argc > 5 ? std::atof(argv[5]) : 2000.0;
    const double mid_us = (min_us + max_us) / 2.0; // 90 degrees center pulse width

    try {
        std::cout << "Connecting to PCA9685 at " << dev << " (addr 0x" << std::hex << addr
                  << std::dec << "), channel " << ch << "...\n";
        
        Pca9685Controller pca(dev, addr, 50.0);

        std::cout << "Setting servo on channel " << ch << " to center (90° / " << mid_us << " us).\n";
        pca.setServoPulseUs(ch, mid_us);
        
        std::cout << "Done. Servo successfully moved to center.\n";
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n"
                  << "  Check: I2C enabled, wiring (SDA/SCL), and user permissions.\n";
        return 1;
    }

    return 0;
}
