# smart_spindle_328p
  - Smart spindle controller with PID control and auto tune. Runs on atmega328p. 
  - To be used between Klipper and CNC spindle BLDC motor controller.
  - Receives target speed from Klipper as G-code via UART, sends back debug data.
  - Reads 3 Hall sensors on motor, calculates speed and adjusts PWM signal to motor driver to match target speed.
  - PID Auto tune based on Ziegler-Nichols ultimate gain tuning method is triggered by push button.
