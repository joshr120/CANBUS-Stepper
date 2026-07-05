# Overview

This document describes the CAN header and frame structure for communicating with and between CANBUS Stepper boards. 

The format uses a single **11-bit CAN Standard ID** that encodes both the **Node ID** and the **Message Type**. The payload contains the data specific to that message type, up to 8 bytes.

An **RTR** (Remote Transmission Request) bit is used to request a current value. When a message is sent to a node with this bit set, the node will reply with the same ID, RTR of 0 and the corresponding payload. RTR can be used to request current command values (E.g setpoint or microsteps) as well as telemetry values (E.g Current Position, or Voltage)

Setting the NODE_ID is done by holding down SW1 on boot for X seconds. E.g Holding down for 2 sec will make the LED flash twice and set the NODE_ID to 2. This is then saved in non-volatile memory.

All multi-byte values (int16, int32, int64, float, double) are sent in **little-endian** format.


## CAN Header Format (11-bit Standard ID)

- Bits: 10 9 8 7 6 | 5 4 3 2 1 0
- NodeID | MsgType
- **NodeID (5 bits):** 0–31 (max 31 CANBUS Steppers on one bus)
- **MsgType (6 bits):** 0–63  
- **Total possible IDs:** 32 nodes × 64 message types = **2048 IDs** (full CAN11 range)


## NodeID Meaning

- Each board uses **one NodeID** for both sending and receiving.  
- **NodeID 0** is reserved for **broadcast** messages. This will send the message to ALL nodes.
- **NodeID 1–31** represent individual nodes on the network.



## MsgType Meaning

The MsgType field determines whether the frame is a **command** or **telemetry** message.

**Convention:**
- **0–31:** Command types
- **32–63:** Telemetry types

## CAN ID Calculation

`CAN_ID = (NodeID << 6) | MsgType`

<br>

# Command Message Types (0–31)


| MsgType (dec) | Name                     | Payload (Bytes 0–7)                                        | Notes                                             | Stored in EEPROM | Default Value   | Implemented |
| ------------- | ------------------------ | ---------------------------------------------------------- | ------------------------------------------------- | ---------------- | --------------- | ----------- |
| 0             | NOP                      | None                                                       | No operation; filler or keep-alive                | No               | -               | Yes         |
| 1             | Set Position (deg)       | 0–7: double target position (deg)                          | Target angle in degrees, 64-bit double            | No               | -               | Yes         |
| 2             | Set Position (steps)     | 0–7: int64 target position (steps)                         | 64-bit signed integer                             | No               | -               | Yes         |
| 3             | Set Velocity (deg/sec)   | 0–3: float target velocity<br>4–7: optional                | 32-bit float                                      | No               | -               | Yes         |
| 4             | Set Current              | 0–1: uint16 target current/torque<br>2–7: optional         | Set motor current, percentage 0–100               | Yes              | 30 (%)          | Yes         |
| 5             | Enable                   | 0: 0 = disable, 1 = enable<br>1–7: optional                | Enable/disable motor                              | No               | 1 (Enabled)     | Yes         |
| 6             | Emergency Stop           | 0–7: optional                                              | Any payload immediately stop motor/driver, re-enable motor after (MsgType 5) | No               | -               | Yes         |
| 7             | Stallguard Behaviour     | 0-1: uint16 stall behaviour <br>2–7: optional              | Action when stallgaurd triggered (sensorless homing) <br> 0 = Do Nothing <br> 1 = Stop <br> 2 = Stop and set as zero position <br> 3 = Stop ONCE (resets to mode 0 after) <br> 4 = Stop and set as zero position ONCE (resets to mode 0 after) | No               | 0 (Do Nothing)               | Yes    |
| 8             | Zero Encoder at boot     | 0: 0 = disabled, 1 = enabled<br>1–7: optional              | Sets the power on position as "0" by applying an offset | Yes        | 0 (disabled)    | Yes         |
| 9             | LED State's              | 0: LED1 State, 1: LED2 State<br>2–7: optional              | Control onboard LED's                             | Yes              | LEDs OFF        | Yes         |
| 10            | Steps per Rev            | 0–1: uint16 steps/rev<br>2–7: optional                     | Configure steps per revolution                    | Yes              | 200             | Yes         |
| 11            | Microsteps               | 0–1: uint16 microsteps<br>2–7: optional                    | Set microstepping                                 | Yes              | 16              | Yes         |
| 12            | Stallguard Threshold     | 0–1: int16 threshold<br>2–7: optional                      | Threshold for stall detection                     | Yes              | 10              | Yes         |
| 13            | Closed Loop Type         | 0-1: int16 control type<br>2–7: optional                   | Set loop mode, 0 = open loop, 1 = closed loop     | Yes              | 1 (closed)      | Yes         |
| 14            | Standstill Mode          | 0-1: uint16 mode <br>2–7: optional                         | set standstill mode <br>0=NORMAL <br>1=FREEWHEELING <br>2=BRAKING <br>3= STRONG_BRAKING| Yes | 0 (normal) | Yes         |
| 15            | Map Direction            | 0: direction<br>1–7: optional                              | Invert direction <br> 0=NORMAL <br> 1=INVERTED    | Yes              | 0 (normal)      | Yes         |
| 16            | Position Speed           | 0–3: float speed<br>4–7: optional                          | Max speed for position moves (deg/sec)            | Yes              | 2000.0          | Yes         |
| 17            | Acceleration             | 0–3: float accel<br>4–7: optional                          | Acceleration for moves (deg/sec²)                 | Yes              | 720.0           | Yes         |
| 18            | Deceleration             | 0–3: float decel<br>4–7: optional                          | Deceleration for moves (deg/sec²)                 | Yes              | 720.0           | Yes         |
| 19            | Report Frequency         | 0-1: uint16 Angle Deg (Hz) <br>2-3: uint16 Other (Hz) <br>4-7: optional| How often angle telemetry is sent (0 to disable)  | Yes    | Angle 10 (Hz) <br> Other 1 (Hz) | Yes         |
| 20            | Enabled/Disabled on Boot | 0: 0 = disabled, 1 = enabled<br>1–7: optional              | Power-on state                                    | Yes              | 1 (enabled)     | Yes         |
| 21            | 3V3 LED Disable          | 0: 0 = normal, 1 = disable<br>1–7: optional                | Disable 3.3V LED                                  | Yes              | 0 (enabled)     | Yes         |
| 22            | TBC
| 23            | Reset to Default         | 0–7: optional                                              | Restore node to default configuration             | No              | -               | Yes         |
| 24            | Save Config              | 0–7: optional                                              | Save current configuration to non-volatile memory | No              | -               | Yes         |
| 25            | Set Node ID              | 0–1: uint16 Node ID<br>2–7: optional                       | Set NODE ID of commanded ID <br> Save config will need to be called to new Node address for it to persist| -              | 1               | Yes         |
| 26            | AUX Connector            | 0–1: AUX1 Function (uint16)<br>2–3: AUX1 Value (uint16)<br>4–5: AUX2 Function (uint16)<br>6–7: AUX2 Value (uint16) | Each pin configured independently. Serial mode (1) requires both pins set to function 1. See AUX Connector section. | Yes            | 0 (Disabled)    | Yes       |

22, 26-31 for future expansion


# Telemetry Message Types (32-63)
Encoder angle (deg) is constantly ouptput at the primary (1) set rate, the other Periodic messages are sent at the secondary (2) rate. The fault state and button states are also sent on state change.

| MsgType (dec) | Name                 | Payload (Bytes 0–7)                          | Notes                            | Periodic | Implemented |
| ------------- | -------------------- | -------------------------------------------- | -----------------------------    | -------- | ----------- |
| 32            | Encoder Counts       | 0–7: int64 encoder counts                    | 64-bit signed integer            | Yes (2)  | Yes         |
| 33            | Encoder Angle (deg)  | 0–7: double angle (deg)                      | 64-bit double                    | Yes (1)  | Yes         |
| 34            | Voltage              | 0–3: float voltage (V)<br>4–7: optional      | 32-bit float                     | Yes (2)  | Yes         |
| 35            | Button States        | 0: SW1<br>1: SW2<br>2–7: optional            | To be interrupt driven in future | Yes (2)  | Yes         |
| 36            | Stallguard Value     | 0–3: uint32 stallguard<br>4–7: optional      | Load / stall detection           | Yes (2)  | Yes     |
| 37            | Stallguard Triggered | 0: 0 = no stall, 1 = stall<br>1–7: optional  | Boolean indicator, sent on stall interupt| No   | Yes     |
| 38            | Board Temperature    | 0–3: float temperature (°C)<br>4–7: optional | NTC on the underside of the PCB  | Yes (2)  | Yes         |
| 39            | Fault Code           | 0–1: uint16 fault code<br>2–7: optional      | Bitfield or enumerated faults    | Yes (2)  | Not Yet     |
| 40            | Software Version     | 0–3: float version <br>4–7: optional         | Node firmware version            | Yes (2)  | Yes         |
| 41            | ESP32 Temperature    | 0–3: float temperature (°C)<br>4–7: optional | ESP32 internal temp sensor       | Yes (2)  | Yes         |
| 42            | Current Velocity (deg/sec)| 0–3: float velocity <br>4–7: optional   | 32-bit float                     | Yes (2)  | Yes         |


## Examples:
## CAN Frame Examples

### Example 1 – Set Position (deg) for Node 3
- **NodeID:** 3  
- **MsgType:** 1 (Set Position (deg))  
- **CAN ID:** `(3 << 6) | 1 = 0xC1`  
- **RTR:** 0
- **Payload:**  
  `00 00 00 00 00 00 56 40`

| CAN ID (hex) | RTR | Payload (bytes)                | Notes               |
|--------------| --- |--------------------------------|---------------------|
| 0xC1         |  0  | 00 00 00 00 00 00 56 40        | Move to 90 deg      |

---

### Example 2 – Request Telemetry (Voltage) from Node 5
- **NodeID:** 5  
- **MsgType:** 34 (Voltage Telemetry)  
- **CAN ID:** `(5 << 6) | 34 = 0x162`  
- **RTR:** 1 (Request) 
- **Payload:**  
  `00 00 00 00 00 00 00 00`  

| CAN ID (hex) | RTR | Payload (bytes)                | Notes                         |
|--------------| --- |--------------------------------|-------------------------------|
| 0x162        |  1  | 00 00 00 00 00 00 00 00        | Request Voltage telemetry     |

---

### Example 3 – Periodic Telemetry: Angle (deg) from Node 2 (Little Endian)
- **NodeID:** 2  
- **MsgType:** 33 (Angle (deg))  
- **CAN ID:** `(2 << 6) | 33 = 0x91`  
- **Payload (double 45.5 deg, little endian):**  
  `00 00 00 00 00 B8 46 40`

| CAN ID (hex) | RTR | Payload (bytes)                | Notes                         |
|--------------| --- |--------------------------------|-------------------------------|
| 0x91         |  0  | 00 00 00 00 00 B8 46 40        | Node 2 sending current angle  |


## Serial Control:

For sending CAN packets over the USB serial link it is the same format: `CAN_ID` + `RTR` + `Payload`  
If the commanded node is the serial bridge it will still send the message onto the bus but also run the command on the Node.
Any Node can have a USB serial connection to send commands and read telemetry data.

E.g: 
enable motor on all nodes:  
=`05 0 0100000000000000`

disable motor on all nodes:  
=`05 0 0000000000000000`

Node 1, set velocity to 100 deg/s:  
=`43 0 0000C84200000000`


## AUX Connector (Work in progress):

The AUX connector can be setup for various different use cases.

This is the 4 pin JST SH connector on the underside of the PCB. AUX1 is Pin 3 on this connector, and AUX2 is Pin 4.

Each pin is configured independently. The 8-byte payload is split into two 4-byte blocks - the first for AUX1 and the second for AUX2:

| Bytes | Field              | Description                        |
|-------|--------------------|------------------------------------|
| 0–1   | AUX1 Function Code | uint16, little-endian              |
| 2–3   | AUX1 Value         | uint16, little-endian (if applicable) |
| 4–5   | AUX2 Function Code | uint16, little-endian              |
| 6–7   | AUX2 Value         | uint16, little-endian (if applicable) |

**Exception:** Serial Control (function code 1) requires both pins and must be set on both AUX1 and AUX2 simultaneously (both function codes set to 1). The value bytes are unused in this mode.

### Pin Function Codes

| Function Code (dec) | Description                                  | Value (bytes 2–3 / 6–7)              | Implemented |
|---------------------|----------------------------------------------|---------------------------------------|-------------|
| 0                   | Disabled (High Impedance)                    | N/A                                   | YES         |
| 1                   | Serial Control (both pins, AUX1=TX, AUX2=RX) | N/A                                   | YES         |
| 2                   | Reserved                                     | N/A                                   | N/A         |
| 3                   | Digital Output                               | 0 = LOW, 1 = HIGH                     | YES         |
| 4                   | Digital Input with Pullup (use RTR to read)  | Read-back: 0 = LOW, 1 = HIGH          | NOT YET     |
| 5                   | Analog Input millivolts (use RTR to read)    | Read-back: uint16 mV                  | NOT YET     |
| 6                   | PWM Output (12-bit resolution)               | uint16 duty cycle (0–4095)            | YES         |

### Examples

Disable both pins - HIGH-Z (Node 1):  
=`5A 0 0000000000000000`

AUX1=Digital Output HIGH, AUX2=Digital Output LOW (Node 1):  
=`5A 0 0300010003000000`

Byte by byte breakdown:

| Byte | Value (Hex) | Description | Explanation |
| ---- | ----------- | ----------- | ----------- |
| 0 | 03 | AUX1 Function (Low) | Part of 0x0003 (Digital Output) |
| 1 | 00 | AUX1 Function (High) | |
| 2 | 01 | AUX1 Value (Low) | HIGH |
| 3 | 00 | AUX1 Value (High) | |
| 4 | 03 | AUX2 Function (Low) | Part of 0x0003 (Digital Output) |
| 5 | 00 | AUX2 Function (High) | |
| 6 | 00 | AUX2 Value (Low) | LOW |
| 7 | 00 | AUX2 Value (High) | |

AUX1=PWM 25% (1024), AUX2=PWM 75% (3072) (Node 1):  
=`5A 0 060000040600000C`

Byte by byte breakdown:

| Byte | Value (Hex) | Description | Explanation |
| ---- | ----------- | ----------- | ----------- |
| 0 | 06 | AUX1 Function (Low) | Part of 0x0006 (PWM Output) |
| 1 | 00 | AUX1 Function (High) | |
| 2 | 00 | AUX1 Value (Low) | Part of 0x0400 (1024 = 25%) |
| 3 | 04 | AUX1 Value (High) | |
| 4 | 06 | AUX2 Function (Low) | Part of 0x0006 (PWM Output) |
| 5 | 00 | AUX2 Function (High) | |
| 6 | 00 | AUX2 Value (Low) | Part of 0x0C00 (3072 = 75%) |
| 7 | 0C | AUX2 Value (High) | |

AUX1=Digital Output HIGH, AUX2=PWM 50% (2048) (Node 1):  
=`5A 0 0300010006000008`

Enable Serial Control on both pins (Node 1):  
=`5A 0 0100000001000000`

Byte by byte breakdown:

| Byte | Value (Hex) | Description | Explanation |
| ---- | ----------- | ----------- | ----------- |
| 0 | 01 | AUX1 Function (Low) | Part of 0x0001 (Serial Control) |
| 1 | 00 | AUX1 Function (High) | |
| 2 | 00 | AUX1 Value (Low) | Unused |
| 3 | 00 | AUX1 Value (High) | Unused |
| 4 | 01 | AUX2 Function (Low) | Part of 0x0001 (Serial Control) |
| 5 | 00 | AUX2 Function (High) | |
| 6 | 00 | AUX2 Value (Low) | Unused |
| 7 | 00 | AUX2 Value (High) | Unused |

AUX1=Analog Input, AUX2=Disabled (Node 1):  
=`5A 0 0500000000000000`

AUX1=Digital Input, AUX2=Digital Input (Node 1):  
=`5A 0 0400000004000000`

AUX1=Digital Input, AUX2=Digital Output LOW (Node 1):  
=`5A 0 0400000003000000`

AUX1=PWM 100% (4095), AUX2=PWM 0% (0) (Node 1):  
=`5A 0 0600FF0F06000000`

AUX1=Digital Output LOW, AUX2=Analog Input (Node 1):  
=`5A 0 0300000005000000`

AUX1=Analog Input, AUX2=Analog Input (Node 1):  
=`5A 0 0500000005000000`

Request AUX pin states via RTR (Node 1):  
=`5A 1 0000000000000000`