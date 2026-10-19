"""

thingsbyjosh.com

Minimal example: connect, move node 1, read its angle back -- over either
USB serial or WiFi, your choice (see below). See python_example_advanced.py
for telemetry callbacks, full configuration, and broadcast commands.

WiFi requires the node already configured via the Web GUI's "Configure
WiFi" dialog (over USB, one time)

Note that when using the USB serial connection, the node will reset when the serial connection is lost (when the code ends).

Install:
    pip install canbus-stepper
"""

from canbus_stepper import CANBus

import time

# -- Choose how to connect: comment out the line you're NOT using -----------
with CANBus.serial("COM4") as bus:                        # USB serial: adjust to your COM port (Device Manager)
# with CANBus.websocket("canbus-stepper-1.local") as bus:   # WiFi: adjust to your node's hostname or IP
    node1 = bus.node(1)
    node1.enable()

    print(f"Angle: {node1.get_angle():.2f}°")

    node1.set_position(360.0)

    time.sleep(3.0)

    print(f"Angle: {node1.get_angle():.2f}°")
    
