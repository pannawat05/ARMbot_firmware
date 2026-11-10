#include "servo_controller.h"

#include <iostream>
#include <string>

int main() {
    try {
        // Safety-first starter: test only one channel at a time.
        // These pulse values and angle limits are EXAMPLES, not calibrated values.
        Pca9685Controller pca("/dev/i2c-1", 0x40, 50.0);

        std::cout << "PCA9685 ready at 0x40, 50 Hz.\n"
                  << "Enter: channel pulse_us (e.g. 0 1500)\n"
                  << "Type 'off' to disable outputs, or 'q' to quit.\n";

        std::string command;
        while (std::cin >> command) {
            if (command == "q" || command == "quit") break;
            if (command == "off") {
                pca.allOff();
                std::cout << "All outputs disabled.\n";
                continue;
            }

            int channel = -1;
            try {
                channel = std::stoi(command);
            } catch (...) {
                std::cout << "Invalid channel. Use 0..15, 'off', or 'q'.\n";
                continue;
            }

            double pulse_us = 0.0;
            if (!(std::cin >> pulse_us)) {
                std::cout << "Expected pulse width in microseconds.\n";
                return 1;
            }

            try {
                pca.setServoPulseUs(channel, pulse_us);
                std::cout << "Set channel " << channel << " to " << pulse_us << " us.\n";
            } catch (const std::exception& e) {
                std::cerr << "Command rejected: " << e.what() << '\n';
            }
        }

        pca.allOff();
        std::cout << "Outputs disabled. Exit.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << '\n';
        return 1;
    }
}
