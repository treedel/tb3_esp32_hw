#ifndef MOTOR_H
    #define MOTOR_H

    // Constants
    #define TOTAL_MOTOR_PINS 2
    #define MIN_MOTOR_PWM 0
    #define MAX_MOTOR_PWM 255

    // Struct for handling motor
    struct Motor {
        byte pin_a;
        byte pin_b;
        int level;
        bool direction; // true - CW, false - CCW
    };

    void run_motor(Motor* motor) {
        if (motor->direction) {
            analogWrite(motor->pin_a, motor->level);
            analogWrite(motor->pin_b, 0);
            return;
        }
        analogWrite(motor->pin_a, 0);
        analogWrite(motor->pin_b, motor->level);
    }

    // Configure motor
    void configure_motor(Motor* motor, byte pin_a, byte pin_b) {
        motor->pin_a = pin_a;
        motor->pin_b = pin_b;
        motor->level = 0;
        motor->direction = true;

        pinMode(motor->pin_a, OUTPUT);
        pinMode(motor->pin_b, OUTPUT);
        run_motor(motor);
    }

    void set_motor_direction(Motor* motor, bool direction) {
        motor->direction = direction;
    }

    // Set motor PWM level (with dead zone compensation)
    void set_motor_level(Motor* motor, int level) {
        if (level > 0 && level < MIN_MOTOR_PWM) level += MIN_MOTOR_PWM;
        level = constrain(level, 0, MAX_MOTOR_PWM);
        motor->level = level;
    }

    void control_motor(Motor* motor, int control_value) {
        set_motor_direction(motor, control_value >= 0);
        set_motor_level(motor, abs(control_value));
        run_motor(motor);
    }

#endif