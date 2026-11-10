#include "servo_controller.h"
#include <array>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

// IMPORTANT: These are PLACEHOLDER calibrations, not values for the real arm.
// Do not enable real movement until each servo's direction, safe angle and pulse
// range are calibrated. Python integration defaults to dry-run.
static constexpr int CHANNELS[6] = {0, 1, 2, 3, 4, 5};
static constexpr double MIN_ANGLE[6] = {-90, -60, -90, -90, -90, -90};
static constexpr double MAX_ANGLE[6] = { 90,  90,  90,  90,  90,  90};
static constexpr double MIN_PULSE_US[6] = {1000,1000,1000,1000,1000,1000};
static constexpr double MAX_PULSE_US[6] = {2000,2000,2000,2000,2000,2000};

int main(int argc, char** argv) {
    const std::string device = argc > 1 ? argv[1] : "/dev/i2c-1";
    const int address = argc > 2 ? std::stoi(argv[2], nullptr, 0) : 0x40;
    try {
        Pca9685Controller pca(device, address, 50.0);
        std::cout << "READY\n" << std::flush;
        std::string line;
        while (std::getline(std::cin, line)) {
            std::istringstream in(line);
            std::string cmd;
            in >> cmd;
            if (cmd == "QUIT") {
                pca.allOff();
                std::cout << "BYE\n" << std::flush;
                break;
            } else if (cmd == "OFF") {
                pca.allOff();
                std::cout << "OK OFF\n" << std::flush;
            } else if (cmd == "MOVE") {
                std::array<double,6> a{};
                bool valid = true;
                for (double& v : a) if (!(in >> v) || !std::isfinite(v)) valid = false;
                if (!valid) {
                    std::cout << "ERR expected six finite angles in degrees\n" << std::flush;
                    continue;
                }
                try {
                    for (int i=0;i<6;i++) {
                        pca.setServoAngle(CHANNELS[i], a[i], MIN_ANGLE[i], MAX_ANGLE[i],
                                          MIN_PULSE_US[i], MAX_PULSE_US[i]);
                    }
                    std::cout << "OK MOVE\n" << std::flush;
                } catch (const std::exception& e) {
                    std::cout << "ERR " << e.what() << "\n" << std::flush;
                }
            } else {
                std::cout << "ERR unknown command\n" << std::flush;
            }
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "servo_daemon: " << e.what() << "\n";
        return 1;
    }
}
