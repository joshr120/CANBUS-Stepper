# **CANBUS Stepper** - Getting Started
If you just received your CANBUS Stepper this will show you how to spin a motor and configure the driver using the web GUI.

Web GUI: [thingsbyjosh.com/canbus_gui](https://thingsbyjosh.com/canbus_gui)

## Assembling a kit onto a Nema 17 ##
For easiest assembly your motor should have the stardard 6 pin JST PH connector, and the included screws (M3x40mm) are best suited for a 34mm length motor.
For example the Hanpose 17HS3401S, available on [Amazon](https://www.amazon.com/Captive-Stepper-17HS3401S-4-Lead-Printer/dp/B0G9KRJFTY) and [Aliexpress](https://www.aliexpress.com/item/1005006577156384.html).

### 1. Install the magnet onto your motor (only needed for reading encoder position)
   
   Carefully superglue the included diametrically magnitised magnet onto the centre of the motor shaft.  

   Only a very small amount of glue is required, take care not to get any in the motor bearing.  
   
   <img src="https://github.com/user-attachments/assets/c32ad5e4-c292-4e67-a0ee-864bf49cce15" width="40%" style="margin: 15px;"/>

### 2. Remove the original motor screws

<img src="https://github.com/user-attachments/assets/aff3b972-af76-4eab-b2e3-df1b70109dc4" width="40%" />

### 3. Peel off the protective film and add one of the included heat pads onto the heat spreader

  Ensure this is added on the recessed side of the heat spreader

<img src="https://github.com/user-attachments/assets/619f5212-02ab-4623-b3c7-b2ffd994ca20" width="40%" />
<img src="https://github.com/user-attachments/assets/c62a9630-885b-43b8-9ef4-58c4579a1af6" width="40%" />

### 4. Place the spacer onto the motor ensuring it is in the orientation show relative to the motor connector

<img src="https://github.com/user-attachments/assets/975f0007-0633-43b5-bc02-90deb9b1e985" width="40%" />

### 5. Add the PCB

Again ensure the PCB is installed in the orientation shown so the motor connectors as well as the heat pad line up

<img src="https://github.com/user-attachments/assets/a98cf78d-519c-4a04-afc4-1bb1bafb2742" width="40%" />

### 5. Add the cover and new countersunk screws

   The included screws are best suited for a 34mm length Nema 17 you may need longer or shorter ones depending on your motor. (included ones are M3x40mm)
   
   <img src="https://github.com/user-attachments/assets/90658808-802d-4ca8-b0fa-40012e529a51" width="40%" />
   <img src="https://github.com/user-attachments/assets/860b4957-be2f-4ece-8546-8102fa0447d0" width="40%" />

### 6. Add the heatsink

   Use the larger heatsink if you plan on running the motor for long periods and/or at high currents
   
   <img src="https://github.com/user-attachments/assets/281de296-c433-4fc9-ba4a-39a087a0c895" width="40%" />
   <img src="https://github.com/user-attachments/assets/5bc483ff-dd3c-4c62-a775-2643c6d06de9" width="40%" />


### 7. Plug in motor loom

 There are 2 looms provided in the kit for different motors, try one and if in the next steps your motor stutters/doesnt spin, power off the board and try the other one. 

  Note. Do not disconnect/connect the motor while the board is powered.

  <img src="https://github.com/user-attachments/assets/fc9cb31f-0f76-4702-9303-2ef88845c0f4" width="40%" />
  


## Power on the Board

Connect a 12V or 24V supply to the "PWR IN" connector.

<img src="https://github.com/user-attachments/assets/b363f0bc-ffd2-418a-9c67-79c06eeb4b52" width="50%" />


You can power the board via the USB Type-C connector for configuration and monitoring, but you will NOT be able to power/move the motor. (See the [PD Stepper](https://thingsbyjosh.com/products/pd-stepper) if you want a USB PD powered stepper motor driver)

## Daisy Chaining Boards
Pay attention to the IN vs OUT labels on the Power connectors. 

<img src="https://github.com/user-attachments/assets/44bd9c1c-b871-4c80-a022-6918a9807678" width="40%" />
<img src="https://github.com/user-attachments/assets/4861487d-afa7-44a8-a55b-be03286a4ab3" width="40%" />


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

