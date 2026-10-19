"""

thingsbyjosh.com

Advanced example: full configuration, telemetry callbacks, timeout
handling, and a broadcast command -- over either USB serial or WiFi, your
choice (see below). See python_example.py for the minimal connect/move/read
version.

WiFi requires the node already configured via the Web GUI's "Configure
WiFi" dialog (over USB, one time) -- this script then only talks to the
control WebSocket on port 81 once that node has joined your network; it
can't provision credentials itself (WIFI_SET is USB-serial-only by design,
see CAN Protocol.md). Any command sent to a node over WiFi is relayed onto
the CAN bus exactly like a USB command would be, so every other node on
the bus is reachable too, not just the one that's actually on WiFi.

Note that when using the USB serial connection, the node will reset when the serial connection is lost (when the code ends).

Install:
    pip install canbus-stepper
"""

import struct
import time

from canbus_stepper import CANBus, MsgType

if __name__ == "__main__":
    # -- Choose how to connect: comment out the line you're NOT using -------
    with CANBus.serial("COM4") as bus:                          # USB serial: adjust to your COM port (Device Manager)
    # with CANBus.websocket("canbus-stepper-1.local") as bus:   # WiFi: adjust to your node's hostname or IP
    
        node1 = bus.node(1)

        # Register a callback for continuous angle telemetry from node 1
        def on_angle(node_id, msg_type, payload):
            angle = struct.unpack("<d", payload[:8])[0]
            print(f"[telemetry] node {node_id} angle: {angle:.2f}°")

        bus.on_telemetry(1, MsgType.ENCODER_ANGLE_DEG, on_angle)

        # Configure node 1
        node1.set_microsteps(16)
        node1.set_current(40)
        node1.set_acceleration(720.0)
        node1.set_deceleration(720.0)
        node1.set_position_speed(360.0)
        node1.save_config() 

        # Enable and move
        node1.enable()
        node1.set_position(180.0)
        time.sleep(2.0)
        node1.set_position(0.0)
        time.sleep(2.0)

        # Request telemetry
        try:
            angle   = node1.get_angle()
            voltage = node1.get_voltage()
            temp    = node1.get_board_temperature()
            version = node1.get_software_version()
            print(f"Angle:       {angle:.2f}°")
            print(f"Voltage:     {voltage:.2f} V")
            print(f"Temperature: {temp:.1f} °C")
            print(f"FW Version:  {version:.2f}")
        except TimeoutError as e:
            print(f"Telemetry timeout: {e}")

        # Broadcast emergency stop to all nodes
        bus.broadcast.estop()
