# CANBUS Stepper
Daisy Chain-able Closed Loop Stepper motor driver and controller. Made by Things by Josh

Kits available now at [thingsbyjosh.com](https://thingsbyjosh.com/products/canbus-stepper)

<p float="left">
  <img src="https://github.com/user-attachments/assets/fefb0139-d6a3-4d52-bf37-3dc5fb4ed737" width="48%" /> 
  <img src="https://github.com/user-attachments/assets/8bb68218-ea98-4dee-93d4-5d94d039e8ba" width="48%" />
</p>


<p float="left">
  <img src="https://github.com/user-attachments/assets/30050754-ccb5-4073-8082-37bbb190f874" width="48%" /> 
  <img src="https://github.com/user-attachments/assets/28f53542-6b5a-46ec-b893-50fd9634da21" width="48%" />
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
- Sensorless Homing
- Qwiic / Stemma QT Compatible
- Configurable AUX Connector.
- Web Based GUI For Configuration
- Open-Source software with example code
- WiFi and BLE
- Works with ESPHome
- Klipper integration in the works

## Web GUI: ##

[Click here to visit](https://thingsbyjosh.com/canbus_gui)

The custom web GUI allows a system to be configured, controlled and monitored over a single USB connection directly to the PCB (or wirelessly with WiFi) with no software installs required. It also allows firmware to be flashed directly from the web GUI over USB.

<img width="2259" height="1301" alt="Screenshot 2026-07-06 205250" src="https://github.com/user-attachments/assets/58e332eb-70e8-4398-b68c-6d29b4a4deb5" />

---

### Refer to the [CANBUS Stepper datasheet](https://github.com/joshr120/CANBUS-Stepper/blob/main/CANBUS%20Stepper%20Datasheet.pdf) for all electrical specifications and connections. ###

--- 

## Node Software: ##
The default CANbus Node software is what ships on all boards. 

The primary idea is you can connect to one node (over USB or Serial or CAN) and then control all other nodes in the chain from this single connection.

The Node ID is set by holding down button "SW1" on power up. Release the button after the desired number of "LED 1" flashes (number of flashes = Node ID). Upto 31 boards can be on a single bus and they all need a unique Node ID.

The software is open-source and available in the software folder. You can use it as is, modify it for your needs or run your own software entirety.

Please refer to the [CAN protocol document](https://github.com/joshr120/CANBUS-Stepper/blob/main/Software/CAN%20Protocol.md) for more information on the protocol, and the [getting started guide](https://github.com/joshr120/CANBUS-Stepper/tree/main/Getting%20Started) for more info on the GUI and Node Software.

##  Control Software: ##
There are multiple ways to contol a CANBUS Stepper board.

The default CAN node software allows a Node to be controlled over CAN, serial (UART), USB or WiFi from a separate controller (Microcontroller or computer).

The full CAN protocol is available [here](https://github.com/joshr120/CANBUS-Stepper/blob/main/Software/CAN%20Protocol.md).

### Web GUI

The web GUI allows you to setup and control nodes, it can communicate via USB (webserial) or WiFi. The motion studio allows you to string together a basic set of movements.

### Python:

For more advanced use cases there is a [python library](https://pypi.org/project/canbus-stepper/) which allows you to easily control a system over either USB or WiFi.

Example Usage:

```
from canbus_stepper import CANBus
```
USB:
```
with CANBus.serial("COM4") as bus:
```

WiFI
```
with CANBus.websocket("canbus-stepper-1.local") as bus:
```

```
  node1 = bus.node(1)
  node1.set_position(360.0)
  node1.set_velocity(90.0)
  node1.enable()
  
  # Broadcast to all nodes
  bus.broadcast.estop()
  
  # Request telemetry
  angle = node1.get_angle()
  voltage = node1.get_voltage()

```

See full examples for more details.

### Serial (UART):
Serial control is done through the AUX connector. This will need to be configured as a serial port through the [Web GUI](https://joshr120.github.io/CANBUS_Stepper_Web_GUI/). Each board will act as a serial to CAN bridge so you can connect to one Node over serial and any others can be daisy chained with CAN.
The protocol for this is defined in the [CAN protocol doc](https://github.com/joshr120/CANBUS-Stepper/blob/main/Software/CAN%20Protocol.md).
Examples coming soon.


### CAN:
For CAN bus control from a seperate microcontroller, you will need a CAN Tranceiver/bridge. 
If your microcontroller has a CAN / TWAI peripheral then you need a tranceiver. If it does not have a native CAN peripheral (E.g Arduino Uno) then you can use a SPI bridge such as the common MCP2551.


### OnBoard:
Alternatively you can run the CANBUS Stepper with no external controller by running your code directly on the onbaord ESP32-S3 (programmable with the Arduino IDE). 
You can use the CANBUS_Stepper_Node.ino as a starting point to get the pin definitions.

### ESPHome:
The CANBUS Stepper can also run ESPHome, as a DC powered alternative to the [PD Stepper](https://thingsbyjosh.com/products/pd-stepper). Examples coming soon.


