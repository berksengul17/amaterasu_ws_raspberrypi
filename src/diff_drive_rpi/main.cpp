#include "stdio.h"
#include "robot.h"
#include "robot_pins.h"
#include "dc_motor.h"
#include "encoder.h"
#include <pigpiod_if2.h>
#include <chrono>
#include <thread>

#define MOTOR_PPR 40.0f
#define SAMPLE_TIME_MS 20
#define PWM_FREQUENCY 25

// UART buffer
char in_buffer[100];
uint16_t char_idx = 0;

// PID parameters
float kp1 = 0.04;
float ki1 = 0.01;
float kd1 = 0;

// Control variables
float linear = 0.0;
float angular = 0.0;

// Handle for pigpio
int pigpio_handle;

// Robot pins setup
RobotPins robot_pins = {
    {
        PWM_FREQUENCY,
        L_ENA_PIN,
        L_IN1_PIN,
        L_IN2_PIN
    },{
        PWM_FREQUENCY,
        R_ENB_PIN,
        R_IN3_PIN,
        R_IN4_PIN
    }
};

// Instantiate robot and encoders
Robot robot(pigpio_handle, kp1, kd1, ki1, SAMPLE_TIME_MS, robot_pins);
Encoder left_encoder(L_ENC_PIN, pigpio_handle);
Encoder right_encoder(R_ENC_PIN, pigpio_handle);

// Print robot state and odometry
void printState(const RobotState &state, const RobotOdometry &odometry) {
    printf(
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        state.l_ref_speed, state.r_ref_speed, state.l_speed, state.r_speed,
        state.l_effort, state.r_effort,
        odometry.x_pos, odometry.y_pos, odometry.theta,
        odometry.v, odometry.w
    );
}

// Setup GPIO and peripherals
void setup() {
    // Initialize pigpio
    pigpio_handle = pigpio_start(NULL, NULL);
    if (pigpio_handle < 0) {
        throw std::runtime_error("Failed to initialize pigpio.");
    }

    // Initialize encoders
    left_encoder.set_pulses(0);
    right_encoder.set_pulses(0);

    printf("Setup complete.\n");
}

// Periodic task for updating the robot
void timerTask() {
    while (true) {
        // Update robot's state using encoder pulses
        robot.setUnicycle(linear, angular);
        robot.updatePid(left_encoder.get_pulses(), right_encoder.get_pulses());

        // Wait for the next iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(SAMPLE_TIME_MS));
    }
}

int main() {
    try {
        setup();

        // Start the periodic timer task in a separate thread
        std::thread timer_thread(timerTask);

        printf("Welcome!\n");

        // UART handling
        while (true) {
            int ch = getchar(); // Read character input
            if (ch != EOF) {
                putchar(ch); // Echo character back
                in_buffer[char_idx++] = ch;

                if (ch == '/') {
                    // Null-terminate the string and parse linear/angular inputs
                    in_buffer[char_idx] = 0;
                    char_idx = 0;

                    char *ch_ptr;
                    linear = strtof(in_buffer, &ch_ptr);
                    angular = strtof(ch_ptr + 1, NULL);

                    // Print robot state and odometry
                    printState(robot.getState(), robot.getOdometry());
                }
            }
        }

        // Wait for timer thread to finish (unlikely in a running program)
        timer_thread.join();

    } catch (const std::exception &e) {
        // Handle errors and clean up
        fprintf(stderr, "Error: %s\n", e.what());
        if (pigpio_handle >= 0) {
            pigpio_stop(pigpio_handle);
        }
        return 1;
    }

    // Clean up pigpio
    pigpio_stop(pigpio_handle);
    return 0;
}
