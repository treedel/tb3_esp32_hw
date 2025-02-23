#ifndef PID_H
    #define PID_H

    #define PID_OUTPUT_LIMIT 255

    // Struct for handling PID control loop
    typedef struct PidControl {
        float loop_delay;
        float kp;
        float ki;
        float kd;
        float current_value;
        float target_value;
        float e;
        float e_prev;
        float e_integral;
        float e_derivative;
        long u;
        bool enable;
    } PidControl;

    void set_pid_constants(PidControl* pid_control, float kp, float ki, float kd) {
        pid_control->kp = kp;
        pid_control->ki = ki;
        pid_control->kd = kd;
        pid_control->e = 0;
        pid_control->e_prev = 0;
        pid_control->e_integral = 0;
        pid_control->e_derivative = 0;
        pid_control->u = 0;
    }

    void reset_pid_variables(PidControl* pid_control) {
        pid_control->e = 0;
        pid_control->e_prev = 0;
        pid_control->e_integral = 0;
        pid_control->e_derivative = 0;
        pid_control->u = 0;
    }

    long calculate_pid_output(PidControl* pid_control, long current_value) {
        Serial.print("Error: ");
        Serial.print(pid_control->e);
        Serial.print(" Integral: ");
        Serial.print(pid_control->e_integral);
        Serial.print(" Derivative: ");
        Serial.println(pid_control->e_derivative);

        pid_control->current_value = current_value;
        
        if (pid_control->enable) {
            pid_control->e = pid_control->target_value - pid_control->current_value;

            // Integral term with anti-windup
            if (pid_control->u > -PID_OUTPUT_LIMIT && pid_control->u < PID_OUTPUT_LIMIT) {
                pid_control->e_integral += (pid_control->e) * pid_control->loop_delay;
            }

            /* pid_control->e_derivative = (pid_control->e - pid_control->e_prev) / pid_control->loop_delay; */
            pid_control->e_derivative = -(pid_control->current_value - pid_control->e_prev) / pid_control->loop_delay;
            
            pid_control->u = (pid_control->kp * pid_control->e) +
                             (pid_control->ki * pid_control->e_integral) +
                             (pid_control->kd * pid_control->e_derivative);
        }
        else {
            reset_pid_variables(pid_control);
        }
        pid_control->e_prev = pid_control->current_value;
        return pid_control->u;
    }

    void enable_pid_control(PidControl* pid_control) {
        pid_control->enable = true;
    }

    void disable_pid_control(PidControl* pid_control) {
        pid_control->enable = false;
    }

    void set_pid_target(PidControl* pid_control, long target_value) {
        pid_control->target_value = target_value;
        if (pid_control->target_value == 0) {
            disable_pid_control(pid_control);
            reset_pid_variables(pid_control);
            return;
        }
        if(!(pid_control->enable)) enable_pid_control(pid_control);
    }

    void configure_pid_control(PidControl* pid_control, float loop_delay, float kp, float ki, float kd) {
        set_pid_constants(pid_control, kp, ki, kd);
        pid_control->loop_delay = loop_delay;
        enable_pid_control(pid_control);
    }

#endif
