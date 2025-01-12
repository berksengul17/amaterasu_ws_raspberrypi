#include "encoder.h"
#include <pigpiod_if2.h>
#include <iostream>
#include <thread>
#include <chrono>

#define ENCODER1_GPIO_PIN 16 // GPIO pin for the first encoder
#define ENCODER2_GPIO_PIN 26 // GPIO pin for the second encoder

int main() {
    // Initialize pigpio
    int pigpio_handle = pigpio_start(NULL, NULL);
    if (pigpio_handle < 0) {
        std::cerr << "Failed to initialize pigpio: " << pigpio_handle << std::endl;
        return 1;
    }

    try {
        // Create instances of the Encoder class
        Encoder encoder1(ENCODER1_GPIO_PIN, pigpio_handle);
        Encoder encoder2(ENCODER2_GPIO_PIN, pigpio_handle);
        std::cout << "Encoder 1 initialized on GPIO pin: " << ENCODER1_GPIO_PIN << std::endl;
        std::cout << "Encoder 2 initialized on GPIO pin: " << ENCODER2_GPIO_PIN << std::endl;

        // Run a loop to monitor the encoder pulses
        std::cout << "Testing encoders. Rotate the wheels and observe the output." << std::endl;
        for (int i = 0; i < 100; ++i) {
            int32_t pulses1 = encoder1.get_pulses();
            int32_t pulses2 = encoder2.get_pulses();
            std::cout << "Encoder 1 Pulses: " << pulses1 << " | Encoder 2 Pulses: " << pulses2 << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    // Cleanup pigpio
    pigpio_stop(pigpio_handle);
    std::cout << "Test completed. Exiting." << std::endl;

    return 0;
}