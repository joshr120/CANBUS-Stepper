# CANBUS-Stepper
Daisy Chain-able Closed Loop Stepper motor driver and controller.


<p float="left">
  <img src="https://github.com/user-attachments/assets/72307142-61fa-448f-8601-798ca4e54502" width="48%" /> 
  <img src="https://github.com/user-attachments/assets/f4183227-76ab-452c-9f88-e1573f00b47e" width="48%" />
</p>

The CANBUS Stepper integrates all essential components required to drive a NEMA 17 stepper
motor into a single compact board, with control handled over CAN bus. Its CAN-based interface,
combined with high-current connectors, enables straightforward daisy chaining of multiple boards.


Built around the ESP32-S3 microcontroller, the design incorporates a TMC2209 stepper driver for
quiet and precise motion, along with a 14-bit rotary position sensor to enable closed-loop feedback.
It is optimised for scalable, multi-axis systems requiring compact integration, low-noise operation,
and precise closed-loop control.

A web GUI allows easy configuration and debugging of a whole system over a single USB connection to any node.

### Key Features
● Daisy Chainable

● ESP32-S3 Processor

● TMC2209 Silent Stepper Driver

● 14 Bit Magnetic Absolute Rotary Encoder

● Qwiic / Stemma QT Compatible

● Web Based GUI For Configuration

● Open-Source software with example code

● WiFi and BLE

● Works with ESPHome

● Klipper intergration in the works

Web GUI:

<img width="1696" height="1272" alt="GUI" src="https://github.com/user-attachments/assets/4bc0999e-9a1c-4a68-9058-b91453482c07" />
