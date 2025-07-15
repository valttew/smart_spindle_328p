# Smart Spindle Controller (Klipper Interface)
  - Smart spindle controller with PID control and auto tune. Runs on atmega328p. 
  - To be used between Klipper and CNC spindle BLDC motor controller.
  - Receives target speed from Klipper as G-code via UART, sends back debug data.
  - Reads 3 Hall sensors on motor, calculates speed and adjusts PWM signal to motor driver to match target speed.
  - PID Auto tune based on Ziegler-Nichols ultimate gain tuning method triggered via button.
  - EEPROM save of Kp, Ki. Command `P0.03,0.15` to set.
  - Command like `S4000` sets target RPM.

## Usage
    make
    make flash

Use `screen /dev/ttyUSB0 115200` to monitor output.
