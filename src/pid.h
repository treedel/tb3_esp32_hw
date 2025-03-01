#ifndef PID_H
    #define PID_H

    #include <PID_v1.h>

    #define MOTOR_CUTOFF_COUNTS 50

    typedef struct PidControl {
        double current_value;
        double target_value;
        double u;
        bool enable;
        PID* controller;
    } PidControl;

    void enable_pid_control(PidControl* pid_control) {
        if (!(pid_control->enable)) {
            (pid_control->controller)->SetMode(AUTOMATIC);
            pid_control->enable = true;
        }
    }

    void disable_pid_control(PidControl* pid_control) {
        if ((pid_control->enable)) {
            (pid_control->controller)->SetMode(MANUAL);
            pid_control->enable = false;
        }
    }

    double calculate_pid_output(PidControl* pid_control, long current_value) {
        pid_control->current_value = current_value;

        (pid_control->controller)->Compute();

        // Slow halt and stop completely
        if (pid_control->target_value == 0 && abs(pid_control->current_value) < MOTOR_CUTOFF_COUNTS) {
            disable_pid_control(pid_control);
            pid_control->u = 0;
        }
        else {
            enable_pid_control(pid_control);
        }
        
        return pid_control->u;
    }

    void set_pid_target(PidControl* pid_control, long target_value) {
        pid_control->target_value = target_value;
        if (target_value != 0) {
            enable_pid_control(pid_control);
        }
    }

    void configure_pid_control(PidControl* pid_control, float loop_delay, float kp, float ki, float kd) {
        pid_control->controller = new PID(&pid_control->current_value, &pid_control->u, &pid_control->target_value, kp, ki, kd, P_ON_E, DIRECT);
        pid_control->controller->SetOutputLimits(-255, 255);
        pid_control->controller->SetSampleTime(loop_delay);
        enable_pid_control(pid_control);
    }

    void set_pid_constants(PidControl* pid_control, float kp, float ki, float kd) {
        pid_control->controller->SetTunings(kp, ki, kd);
    }

#endif // PID_H