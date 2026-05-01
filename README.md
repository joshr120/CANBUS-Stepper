# CANBUS Stepper
Daisy Chain-able Closed Loop Stepper motor driver and controller. Made by Things by Josh

Kits available shortly at [thingsbyjosh.com](thingsbyjosh.com)

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
- Daisy Chainable
- ESP32-S3 Processor
- High current power pass through connectors
- TMC2209 Silent Stepper Driver
- 14 Bit Magnetic Absolute Rotary Encoder
- Qwiic / Stemma QT Compatible
- Configurable AUX Connector.
- Web Based GUI For Configuration
- Open-Source software with example code
- WiFi and BLE
- Works with ESPHome
- Klipper intergration in the works

## Web GUI: ##

[Click here to visit](https://joshr120.github.io/CANBUS_Stepper_Web_GUI/)

The custom web GUI allows any board to be configured, controlled and monitored over a single USB Serial connection directly to the PCB with no software installs required. It also allows the connected board to be flashed with the most recent software.

<img width="1696" height="1272" alt="GUI" src="https://github.com/user-attachments/assets/4bc0999e-9a1c-4a68-9058-b91453482c07" />

## Node Software: ##
The default CANbus Node software is what ships on all boards. 

The Node ID is set by holding down button "SW1" on power up. Release the button after the desired number of "LED 1" flashes (number of flashes = Node ID). Upto 31 boards can be on a single bus and they all need a unique Node ID.

The software is open-source and available in the software folder. You can use it as is, modify it for your needs or run your own software entirety.

Please refer to the CAN protocol document for more information.

##  Control Software: ##
There are multiple ways to contol a CANBUS Stepper board.

The default CAN node software allows a Node to be controlled over CAN, serial (UART) or USB from a seperate controller (Microcontroller or computer).

The full CAN protocol is available here.

For USB Control & monitoring from a PC there is a python example script (as well as the [Web GUI ](https://joshr120.github.io/CANBUS_Stepper_Web_GUI/)for basic controls)

Serial control is done through the AUX connector. This will need to be configured as a serial port through the [Web GUI](https://joshr120.github.io/CANBUS_Stepper_Web_GUI/). Each board will act as a serial to CAN bridge so you can connect to one Node over serial and any others can be daisy chained with CAN.
The protocol for this is defined in the CAN protocol doc.
Examples also coming soon...

For CAN bus control from a seperate microcontroller, you will need a CAN Tranceiver/bridge. 
If your microcontroller has a CAN / TWAI peripheral then you need a tranceiver. If it does not have a native CAN peripheral (E.g Arduino Uno) then you can use a SPI bridge such as the common MCP2551.

Alternatively you can also run the CANBUS Stepper with no external controller by running your code directly on the onbaord ESP32-S3 (programmable with the Arduino IDE)

The CANBUS Stepper can also run ESPHome, as a DC powered alternative to the [PD Stepper](https://thingsbyjosh.com/products/pd-stepper). Example coming soon...

##  Electrical Connections: ##
Refer to the [CANBUS Stepper datasheet](https://github.com/joshr120/CANBUS-Stepper/blob/main/CANBUS%20Stepper%20Datasheet.pdf) for all electrical specifications and connections.

