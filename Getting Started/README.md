# Work In Progress. Please refer to steps 1-7 in the [PD Stepper Guide ](https://github.com/joshr120/PD-Stepper/tree/main/Getting%20Started)for assembling a kit #

To use the web GUI visit [thingsbyjosh.com/canbus_gui](https://thingsbyjosh.com/canbus_gui)

## Power on the Board

Connect a 12V or 24V supply to the "PWR IN" connector.

You can power the board via the USB Type-C connector for configuration and monitoring, but you will NOT be able to power/move the motor. (See the PD [Stepper](https://thingsbyjosh.com/products/pd-stepper) if you want a USB PD powered stepper motor driver)

## Daisy Chaining Boards
Pay attention to the IN vs OUT labels on the Power connectors. 

When daisy chaining nodes together set the "CAN TERM" DIP switch ON for the two boards at each end of the string, and OFF for all others.

## Using the Web GUI

The Web GUI can be used to control, configure and program the CANBUS Stepper.

It is viewable online at [thingsbyjosh.com/canbus_gui](https://thingsbyjosh.com/canbus_gui)

1. Connect your CANBUS Stepper board to a PC via the USB type-C connector on the PCB.
2. Open the Web GUI, press "Connect" and select your board. On windows machines this will come up as something like "USB JTAG/serial debug unit (COMxx)". You can also find the COM port number by opening device manager on your PC.

<p float="left">
  <img src="https://github.com/user-attachments/assets/b991ab38-7422-4ce3-9e48-f80628b457d9" width="53%" />
  <img src="https://github.com/user-attachments/assets/e7d89f7a-5c0b-406d-8234-d92349b7a5e2" width="43%" /> 
</p>

3. View all discovered Nodes by pressing "x Nodes Found" on the bottom right. Click on a node to view it (or select the node ID with "TARGET NODE" on the top right). Setting the "TARGET NODE" to "0" will send the command to all nodes on the bus.
4. Configure Node Device Settings by double clicking the value. After testing the change behaves as expected press "SAVE TO FLASH" to keep the values between power cycles.

## Closed Loop Control

Enable closed loop control in the web GUI

1. Change the "Set Position (deg)" to 360 by double clicking the value.
2. Under "LIVE TELEMETRY, the "Encoder Angle (deg)" should read 360 deg ± 1 deg. If this instead reads -360 set the "Map Direction" to 1. If it reads 180 degrees, set the "Steps per Rev" to 400.

## Sensorless Homing

Sensorless homing is available via the StallGuard feature of the TMC2209 driver. This allows the motor to find a home position by detecting an increase in the motor load once it reaches a hard stop. The CAN Node software exposes this for easy use.

The procedure for sensorless homing it is as follows:

  1. Set "Stallguard Behaviour" (MsgType 7) to 4.

  2. Set "Set Velocity (deg/sec)" (MsgType 3) to your desired speed / direction.

Once the stall guard is triggered it will stop the motor and reset the zero position. (other stall behaviours can also be set by changing the "Stallguard Behaviour")

You can adjust the "Stallguard Threshold" (MsgType 12) to change at what motor load this is triggered, and live view "Stallguard Value" to view the live value. All configurable via the web GUI.

## Serial Control

To control a CANBUS Stepper node (or systems of nodes) over hardware serial, the AUX connector can be configured for serial Control.

Double click the "AUX Connector" setting, and select "Serial Control"

<img width="625" height="396" alt="image" src="https://github.com/user-attachments/assets/398788b0-9e78-45b8-9e4f-32d16c716592" />

Serial control is at 115200 Baud, format 8N1. Serial messages follow the same format as CAN packets. See the CAN Protocol document for more information.

## Custom CAN Messages

There is a CAN message builder on the top right which allows CAN messaged to be built up. This is useful for debugging CAN messages when building your own control interface.

See the CAN Protocol document for for info on the CAN packet structure.

