#include <Arduino.h>

#include "pid.h"
#include "encoders.h"
#include "motor.h"

#define CONTROL_SERIAL Serial
#define CONTROL_SERIAL_BAUD_RATE 115200

// Define all pins
const byte ENC_PINS[TOTAL_ENCODER_CHANNELS][TOTAL_ENCODER_PINS] = {
  {25, 26},
  {27, 14}
};
const byte MOT_PINS[TOTAL_ENCODER_CHANNELS][TOTAL_MOTOR_PINS] = {
  {18, 5},
  {2, 15}
};

Motor motor_left, motor_right;
PidControl left_pid_controller, right_pid_controller;

// Constants to set loop rates
const float PID_LOOP_RATE = 5.0;
const float PID_LOOP_INTERVAL_MS = (1.0 / PID_LOOP_RATE) * 1000;
const float LPF_ALPHA = 0.3; // Low-pass filter smoothing factor

// Variables to store encoder readings
float left_filtered_speed = 0;
float right_filtered_speed = 0;

unsigned long next_millis_pid = 0;

void setup() {
  CONTROL_SERIAL.begin(CONTROL_SERIAL_BAUD_RATE);
  delay(2000);
  CONTROL_SERIAL.println("Serial interface initialized");

  // Configure encoders
  configure_encoders(ENC_PINS[0][0], ENC_PINS[0][1], ENC_PINS[1][0], ENC_PINS[1][1]);

  // Configure motors
  configure_motor(&motor_left, MOT_PINS[0][0], MOT_PINS[0][1]);
  configure_motor(&motor_right, MOT_PINS[1][0], MOT_PINS[1][1]);

  // Configure PID controllers
  configure_pid_control(&left_pid_controller, 1.0 / PID_LOOP_RATE, 1, 5, 0.1);
  configure_pid_control(&right_pid_controller, 1.0 / PID_LOOP_RATE, 1, 5, 0.1);
}

void process_serial_commands() {
  char command;
  float arg1;
  float arg2;
  float arg3;

  if (CONTROL_SERIAL.available() > 0) {
    command = CONTROL_SERIAL.read();

    switch (command) {
      case 'e':  // Get encoder values
        CONTROL_SERIAL.print(read_enc_count_a());
        CONTROL_SERIAL.print(" ");
        CONTROL_SERIAL.println(read_enc_count_b());
        break;

      case 'c':  // Set PID target values
        arg1 = CONTROL_SERIAL.parseFloat();
        arg2 = CONTROL_SERIAL.parseFloat();
        set_pid_target(&left_pid_controller, arg1);
        set_pid_target(&right_pid_controller, arg2);
        CONTROL_SERIAL.println("OK");
        break;

      case 'p':  // Set PID constants
        arg1 = CONTROL_SERIAL.parseFloat();
        arg2 = CONTROL_SERIAL.parseFloat();
        arg3 = CONTROL_SERIAL.parseFloat();
        set_pid_constants(&left_pid_controller, arg1, arg2, arg3);
        set_pid_constants(&right_pid_controller, arg1, arg2, arg3);
        CONTROL_SERIAL.println("OK");
        break;
    }
  }
}

void pid_control_loop() {
  update_enc_values();
  int left_encoder_delta = read_enc_delta_a() * PID_LOOP_RATE;
  int right_encoder_delta = read_enc_delta_b() * PID_LOOP_RATE;

  left_filtered_speed = (LPF_ALPHA * left_encoder_delta) + ((1 - LPF_ALPHA) * left_filtered_speed);
  right_filtered_speed = (LPF_ALPHA * right_encoder_delta) + ((1 - LPF_ALPHA) * right_filtered_speed);

  int left_control_level = calculate_pid_output(&left_pid_controller, left_filtered_speed);
  int right_control_level = calculate_pid_output(&right_pid_controller, right_filtered_speed);

  control_motor(&motor_left, left_control_level);
  control_motor(&motor_right, right_control_level);
}

void loop() {
  process_serial_commands();

  if (millis() >= next_millis_pid) {
    next_millis_pid += PID_LOOP_INTERVAL_MS;
    pid_control_loop();
  }
}
