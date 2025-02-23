# tb3_esp32_hw

Hardware interface to work with ros2_control. Inspired from joshnewans/ros_arduino_bridge.

## Current features
- ros2_control interfacing using serial
- PID based motor speed control
- Hall effect incremental encoder based feedback
- Encoder output filtering using low-pass filter to reduce noise

## Todo
- [ ] Make this work with microRos
- [ ] Use RTOS to make it more flexible
