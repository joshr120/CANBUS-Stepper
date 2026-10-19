"""

thingsbyjosh.com

Python interface for the CANBUS Stepper Motor Controllers over WebSocket (WiFi),
instead of USB serial. Same line protocol, same high-level API as
python_example.py (Python USB Serial Interface) -- only the transport changed, so
code written against one is almost a drop-in fit for the other.

Requires one node already configured for WiFi via the Web GUI's "Configure WiFi"
dialog (over USB, one time) -- this script only talks to the control WebSocket on
port 81 once that node has joined your network; it can't provision credentials
itself (WIFI_SET is USB-serial-only by design, see CAN Protocol.md). Any command
sent to a node over this WebSocket is relayed onto the CAN bus exactly like a USB
command would be, so every other node on the bus is reachable too, not just the
one that's actually on WiFi.

Install the WebSocket client library first (note the exact name -- "websocket"
alone is a different, unrelated package):
    pip install websocket-client

Protocol overview:
  - 11-bit CAN Standard ID encodes NodeID (5 bits) and MsgType (6 bits)
  - CAN_ID = (NodeID << 6) | MsgType
  - WebSocket text frame: "<CAN_ID_hex> <RTR> <payload_16hex_chars>\n"
  - All multi-byte values are little-endian
  - NodeID 0 = broadcast to all nodes

Example usage: (see bottom of page for actual code example)
    bus = CANBus("canbus-stepper-1.local")   # hostname or IP of the WiFi-connected node

    node1 = bus.node(1)
    node1.set_position(360.0)
    node1.set_velocity(90.0)
    node1.enable()

    # Broadcast to all nodes
    bus.broadcast.estop()

    # Request telemetry
    angle = node1.get_angle()
    voltage = node1.get_voltage()

    bus.close()
"""

import struct
import sys
import threading
import time
from collections import defaultdict
from typing import Callable, Dict, Optional

import websocket  # pip install websocket-client


# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

BROADCAST_NODE_ID = 0

WS_PORT = 81  # matches WS_PORT in the firmware


class CANBusConnectionError(Exception):
    """
    Raised when the WebSocket connection to a node can't be established --
    bad hostname/IP, node offline/unreachable, or nothing listening on the
    port. Always has a clean, human-readable message (see CANBus.__init__).
    """
    pass


class MsgType:
    """Command message types (0–31)."""
    NOP                = 0
    SET_POSITION_DEG   = 1
    SET_POSITION_STEPS = 2
    SET_VELOCITY       = 3
    SET_CURRENT        = 4
    ENABLE             = 5
    ESTOP              = 6
    SENSORLESS_HOME    = 7
    ZERO_ON_BOOT       = 8
    LED_STATE          = 9
    STEPS_PER_REV      = 10
    MICROSTEPS         = 11
    STALLGUARD_THRESH  = 12
    CLOSED_LOOP_TYPE   = 13
    STANDSTILL_MODE    = 14
    MAP_DIRECTION      = 15
    POSITION_SPEED     = 16
    ACCELERATION       = 17
    DECELERATION       = 18
    REPORT_FREQUENCY   = 19
    ENABLE_ON_BOOT     = 20
    DISABLE_3V3_LED    = 21
    RESET_TO_DEFAULT   = 23
    SAVE_CONFIG        = 24
    SET_NODE_ID        = 25
    AUX_CONNECTOR      = 26

    """Telemetry message types (32–63)."""
    ENCODER_COUNTS     = 32
    ENCODER_ANGLE_DEG  = 33
    VOLTAGE            = 34
    BUTTON_STATES      = 35
    STALLGUARD_VALUE   = 36
    STALLGUARD_TRIGGER = 37
    BOARD_TEMPERATURE  = 38
    FAULT_CODE         = 39
    SOFTWARE_VERSION   = 40
    ESP32_TEMPERATURE  = 41
    CURRENT_VELOCITY   = 42


class StandstillMode:
    NORMAL         = 0
    FREEWHEELING   = 1
    BRAKING        = 2
    STRONG_BRAKING = 3


class AuxFunction:
    DISABLED       = 0
    SERIAL_CONTROL = 1
    DIGITAL_OUTPUT = 3
    DIGITAL_INPUT  = 4
    ANALOG_INPUT   = 5
    PWM_OUTPUT     = 6


# ---------------------------------------------------------------------------
# Frame encoding / decoding (identical to the USB serial version -- the line
# protocol itself doesn't change between transports)
# ---------------------------------------------------------------------------

def build_can_id(node_id: int, msg_type: int) -> int:
    return (node_id << 6) | msg_type


def parse_can_id(can_id: int):
    """Returns (node_id, msg_type)."""
    return (can_id >> 6) & 0x1F, can_id & 0x3F


def encode_frame(node_id: int, msg_type: int, rtr: int, payload: bytes) -> str:
    """
    Encode a WebSocket text frame.
    Format: "<CAN_ID_hex> <RTR> <payload_16hex>"
    Payload is always zero-padded to 8 bytes.
    """
    can_id = build_can_id(node_id, msg_type)
    padded = payload.ljust(8, b'\x00')[:8]
    return f"{can_id:X} {rtr} {padded.hex().upper()}"


def decode_frame(line: str):
    """
    Decode a frame string.
    Returns (node_id, msg_type, rtr, payload_bytes) or None on error.
    """
    parts = line.strip().split()
    if len(parts) != 3:
        return None
    try:
        can_id = int(parts[0], 16)
        rtr    = int(parts[1])
        payload = bytes.fromhex(parts[2])
        node_id, msg_type = parse_can_id(can_id)
        return node_id, msg_type, rtr, payload
    except (ValueError, AttributeError):
        return None


# ---------------------------------------------------------------------------
# CANBus — WebSocket connection + receive loop
# ---------------------------------------------------------------------------

class CANBus:
    """
    Manages the WebSocket connection to a CANBUS stepper network, over WiFi.

    Args:
        host:          Hostname or IP of the WiFi-connected node, e.g.
                        "canbus-stepper-1.local" or "192.168.1.42" (same value
                        you'd type into the Web GUI's "Connect via WiFi" dialog).
        port:           WebSocket port (default 81, matches the firmware).
        connect_timeout: Seconds to wait for the WebSocket handshake to complete.

    Raises:
        CANBusConnectionError: if the host can't be resolved/reached, or the
            WebSocket handshake otherwise fails -- with a plain-English reason
            instead of a raw socket/websocket traceback.
    """

    def __init__(self, host: str, port: int = WS_PORT, connect_timeout: float = 5.0):
        url = f"ws://{host}:{port}/"
        try:
            self._ws = websocket.create_connection(url, timeout=connect_timeout)
        except websocket.WebSocketAddressException as e:
            raise CANBusConnectionError(
                f'Could not resolve host "{host}" ({e}). '
                f'If this is a "<name>.local" hostname: Windows does not resolve mDNS '
                f'(.local) names by default the way macOS/Linux do -- either install '
                f'Bonjour for Windows, or use the node\'s IP address instead (shown in '
                f'the Web GUI, or your router\'s DHCP client list).'
            ) from None
        except (websocket.WebSocketTimeoutException, TimeoutError):
            raise CANBusConnectionError(
                f'Timed out connecting to {host}:{port} after {connect_timeout:g}s -- '
                f'check the node is powered on and still connected to WiFi.'
            ) from None
        except ConnectionRefusedError:
            raise CANBusConnectionError(
                f'Connection refused by {host}:{port} -- the host answered, but nothing '
                f'is listening on that port (check the port number; firmware default is {WS_PORT}).'
            ) from None
        except OSError as e:
            raise CANBusConnectionError(
                f'Could not connect to {host}:{port} ({e}). Check the host is correct '
                f'and reachable on your network.'
            ) from None
        except websocket.WebSocketException as e:
            raise CANBusConnectionError(
                f'WebSocket handshake with {host}:{port} failed ({e}).'
            ) from None

        self._lock = threading.Lock()

        # Registered callbacks: {(node_id, msg_type): [callable, ...]}
        self._callbacks: Dict[tuple, list] = defaultdict(list)

        # One-shot response events for RTR requests: {(node_id, msg_type): threading.Event}
        self._pending: Dict[tuple, dict] = {}
        self._pending_lock = threading.Lock()

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Expose a broadcast node
        self.broadcast = StepperNode(self, BROADCAST_NODE_ID)

    # -- Public API ----------------------------------------------------------

    def node(self, node_id: int) -> "StepperNode":
        """Get a StepperNode handle for the given node_id (1–31)."""
        if not 1 <= node_id <= 31:
            raise ValueError(f"node_id must be 1–31, got {node_id}")
        return StepperNode(self, node_id)

    def on_telemetry(self, node_id: int, msg_type: int, callback: Callable):
        """
        Register a callback for incoming telemetry frames.
        callback(node_id, msg_type, payload_bytes)
        """
        self._callbacks[(node_id, msg_type)].append(callback)

    def close(self):
        self._running = False
        try:
            self._ws.close()
        except Exception:
            pass
        self._rx_thread.join(timeout=1.0)

    # -- Internal ------------------------------------------------------------

    def _send(self, node_id: int, msg_type: int, rtr: int, payload: bytes):
        frame = encode_frame(node_id, msg_type, rtr, payload)
        with self._lock:
            self._ws.send(frame)

    def _request(self, node_id: int, msg_type: int, timeout: float = 1.0):
        """Send an RTR request and block until the response arrives."""
        key = (node_id, msg_type)
        event = threading.Event()
        result = {}

        with self._pending_lock:
            self._pending[key] = {"event": event, "result": result}

        self._send(node_id, msg_type, rtr=1, payload=b'\x00' * 8)

        if not event.wait(timeout):
            with self._pending_lock:
                self._pending.pop(key, None)
            raise TimeoutError(
                f"No response from node {node_id} for msg_type {msg_type}"
            )

        return result.get("payload")

    def _handle_line(self, line: str):
        parsed = decode_frame(line)
        if parsed is None:
            return
        node_id, msg_type, rtr, payload = parsed

        # Resolve any pending RTR request
        key = (node_id, msg_type)
        with self._pending_lock:
            pending = self._pending.pop(key, None)
        if pending:
            pending["result"]["payload"] = payload
            pending["event"].set()

        # Fire registered callbacks
        for cb in self._callbacks.get(key, []):
            try:
                cb(node_id, msg_type, payload)
            except Exception as e:
                print(f"[canbus] callback error: {e}")

    def _rx_loop(self):
        while self._running:
            try:
                raw = self._ws.recv()
                if not raw:
                    continue
                # Each queued telemetry line is sent as its own WebSocket text frame
                # (see broadcastLineToWS() in the firmware), but split on '\n' anyway
                # in case more than one line ever arrives in a single frame -- the
                # Web GUI does the same defensively.
                for line in raw.split("\n"):
                    line = line.strip()
                    if line:
                        self._handle_line(line)
            except websocket.WebSocketConnectionClosedException:
                break
            except Exception:
                if self._running:
                    continue
                break

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def connect(host: str, port: int = WS_PORT, connect_timeout: float = 5.0) -> "CANBus":
    """
    Convenience wrapper around CANBus() for scripts/CLIs: on a connection
    failure, prints a clean one-line reason (instead of a traceback) and
    exits the process, so callers (like the example below) don't need their
    own try/except just to stay out of a traceback.

    Use CANBus(...) directly instead if you want to catch
    CANBusConnectionError yourself (e.g. to retry).
    """
    try:
        return CANBus(host, port=port, connect_timeout=connect_timeout)
    except CANBusConnectionError as e:
        print(f"Could not connect to {host}: {e}")
        sys.exit(1)


# ---------------------------------------------------------------------------
# StepperNode — per-node command and telemetry API
# (identical to the USB serial version -- fully transport-agnostic, it only
# ever calls back into CANBus._send()/_request())
# ---------------------------------------------------------------------------

class StepperNode:
    """
    High-level interface for a single CANBUS stepper node.

    All send methods return self to allow chaining, e.g.:
        node1.set_acceleration(500).set_position(180)
    """

    def __init__(self, bus: CANBus, node_id: int):
        self._bus = bus
        self.node_id = node_id

    # -- Convenience ---------------------------------------------------------

    def _send(self, msg_type: int, payload: bytes = b''):
        self._bus._send(self.node_id, msg_type, rtr=0, payload=payload)
        return self

    def _request(self, msg_type: int, timeout: float = 1.0) -> bytes:
        return self._bus._request(self.node_id, msg_type, timeout=timeout)

    # -- Motion commands -----------------------------------------------------

    def set_position(self, degrees: float):
        """Move to an absolute position in degrees (64-bit double)."""
        payload = struct.pack("<d", float(degrees))
        return self._send(MsgType.SET_POSITION_DEG, payload)

    def set_position_steps(self, steps: int):
        """Move to an absolute position in steps (64-bit signed int)."""
        payload = struct.pack("<q", int(steps))
        return self._send(MsgType.SET_POSITION_STEPS, payload)

    def set_velocity(self, deg_per_sec: float):
        """Set continuous velocity in deg/sec (32-bit float)."""
        payload = struct.pack("<f", float(deg_per_sec))
        return self._send(MsgType.SET_VELOCITY, payload)

    def set_position_speed(self, deg_per_sec: float):
        """Set the max speed used during position moves (deg/sec)."""
        payload = struct.pack("<f", float(deg_per_sec))
        return self._send(MsgType.POSITION_SPEED, payload)

    def set_acceleration(self, deg_per_sec2: float):
        """Set acceleration (deg/sec²)."""
        payload = struct.pack("<f", float(deg_per_sec2))
        return self._send(MsgType.ACCELERATION, payload)

    def set_deceleration(self, deg_per_sec2: float):
        """Set deceleration (deg/sec²)."""
        payload = struct.pack("<f", float(deg_per_sec2))
        return self._send(MsgType.DECELERATION, payload)

    # -- Motor state ---------------------------------------------------------

    def enable(self):
        """Enable the motor driver."""
        return self._send(MsgType.ENABLE, struct.pack("<B", 1))

    def disable(self):
        """Disable the motor driver."""
        return self._send(MsgType.ENABLE, struct.pack("<B", 0))

    def estop(self):
        """Immediately stop the motor (emergency stop)."""
        return self._send(MsgType.ESTOP)

    def nop(self):
        """Send a no-op / keep-alive frame."""
        return self._send(MsgType.NOP)

    # -- Configuration -------------------------------------------------------

    def set_current(self, percent: int):
        """Set motor current as a percentage (0–100)."""
        if not 0 <= percent <= 100:
            raise ValueError("Current must be 0–100%")
        payload = struct.pack("<H", int(percent))
        return self._send(MsgType.SET_CURRENT, payload)

    def set_microsteps(self, microsteps: int):
        """Set microstepping value (e.g. 1, 2, 4, 8, 16, 32, 64, 128, 256)."""
        payload = struct.pack("<H", int(microsteps))
        return self._send(MsgType.MICROSTEPS, payload)

    def set_steps_per_rev(self, steps: int):
        """Set full steps per revolution (default 200)."""
        payload = struct.pack("<H", int(steps))
        return self._send(MsgType.STEPS_PER_REV, payload)

    def set_closed_loop(self, enabled: bool = True):
        """Enable (True) or disable (False) closed-loop control."""
        payload = struct.pack("<h", 1 if enabled else 0)
        return self._send(MsgType.CLOSED_LOOP_TYPE, payload)

    def set_standstill_mode(self, mode: int = StandstillMode.NORMAL):
        """Set standstill mode. Use StandstillMode constants."""
        payload = struct.pack("<H", int(mode))
        return self._send(MsgType.STANDSTILL_MODE, payload)

    def set_direction(self, inverted: bool = False):
        """Set motor direction. True = inverted."""
        payload = struct.pack("<B", 1 if inverted else 0)
        return self._send(MsgType.MAP_DIRECTION, payload)

    def set_zero_on_boot(self, enabled: bool = True):
        """Set whether the encoder position is zeroed on boot."""
        payload = struct.pack("<B", 1 if enabled else 0)
        return self._send(MsgType.ZERO_ON_BOOT, payload)

    def set_enable_on_boot(self, enabled: bool = True):
        """Set whether the motor is enabled on power-on."""
        payload = struct.pack("<B", 1 if enabled else 0)
        return self._send(MsgType.ENABLE_ON_BOOT, payload)

    def set_report_frequency(self, angle_hz: int = 10, other_hz: int = 1):
        """Set telemetry report rates. 0 disables that telemetry stream."""
        payload = struct.pack("<HH", int(angle_hz), int(other_hz))
        return self._send(MsgType.REPORT_FREQUENCY, payload)

    def set_stallguard_threshold(self, threshold: int):
        """Set the stallguard detection threshold (int16)."""
        payload = struct.pack("<h", int(threshold))
        return self._send(MsgType.STALLGUARD_THRESH, payload)

    def set_led(self, led1: int, led2: int):
        """Control onboard LEDs (0 = off, 1 = on)."""
        payload = struct.pack("<BB", int(led1), int(led2))
        return self._send(MsgType.LED_STATE, payload)

    def sensorless_home(self, direction: int = 1):
        """Trigger sensorless homing. direction: 0 = negative, 1 = positive."""
        payload = struct.pack("<B", int(direction))
        return self._send(MsgType.SENSORLESS_HOME, payload)

    def save_config(self):
        """Save current configuration to non-volatile memory."""
        return self._send(MsgType.SAVE_CONFIG)

    def reset_to_default(self):
        """Restore node to default configuration."""
        return self._send(MsgType.RESET_TO_DEFAULT)

    # -- AUX connector -------------------------------------------------------

    def set_aux(
        self,
        aux1_function: int = AuxFunction.DISABLED,
        aux1_value: int = 0,
        aux2_function: int = AuxFunction.DISABLED,
        aux2_value: int = 0,
    ):
        """
        Configure the AUX connector pins.

        Example — AUX1 PWM 50%, AUX2 digital output HIGH:
            node.set_aux(AuxFunction.PWM_OUTPUT, 2048,
                         AuxFunction.DIGITAL_OUTPUT, 1)
        """
        payload = struct.pack("<HHHH",
            aux1_function, aux1_value,
            aux2_function, aux2_value,
        )
        return self._send(MsgType.AUX_CONNECTOR, payload)

    def set_aux_serial(self):
        """Enable hardware serial control on the AUX connector (both pins)."""
        return self.set_aux(
            AuxFunction.SERIAL_CONTROL, 0,
            AuxFunction.SERIAL_CONTROL, 0,
        )

    # -- Telemetry (RTR requests) --------------------------------------------

    def get_angle(self, timeout: float = 1.0) -> float:
        """Request and return current encoder angle in degrees."""
        payload = self._request(MsgType.ENCODER_ANGLE_DEG, timeout)
        return struct.unpack("<d", payload[:8])[0]

    def get_encoder_counts(self, timeout: float = 1.0) -> int:
        """Request and return raw encoder counts (int64)."""
        payload = self._request(MsgType.ENCODER_COUNTS, timeout)
        return struct.unpack("<q", payload[:8])[0]

    def get_voltage(self, timeout: float = 1.0) -> float:
        """Request and return bus voltage in volts."""
        payload = self._request(MsgType.VOLTAGE, timeout)
        return struct.unpack("<f", payload[:4])[0]

    def get_board_temperature(self, timeout: float = 1.0) -> float:
        """Request and return PCB temperature in °C."""
        payload = self._request(MsgType.BOARD_TEMPERATURE, timeout)
        return struct.unpack("<f", payload[:4])[0]

    def get_esp32_temperature(self, timeout: float = 1.0) -> float:
        """Request and return ESP32 internal temperature in °C."""
        payload = self._request(MsgType.ESP32_TEMPERATURE, timeout)
        return struct.unpack("<f", payload[:4])[0]

    def get_velocity(self, timeout: float = 1.0) -> float:
        """Request and return current velocity in deg/sec."""
        payload = self._request(MsgType.CURRENT_VELOCITY, timeout)
        return struct.unpack("<f", payload[:4])[0]

    def get_software_version(self, timeout: float = 1.0) -> float:
        """Request and return firmware version."""
        payload = self._request(MsgType.SOFTWARE_VERSION, timeout)
        return struct.unpack("<f", payload[:4])[0]

    def get_aux_states(self, timeout: float = 1.0) -> bytes:
        """Request and return AUX pin states (raw 8-byte payload)."""
        return self._request(MsgType.AUX_CONNECTOR, timeout)

    def get_microsteps(self, timeout: float = 1.0) -> int:
        """Request current microstepping setting."""
        payload = self._request(MsgType.MICROSTEPS, timeout)
        return struct.unpack("<H", payload[:2])[0]

    def get_current(self, timeout: float = 1.0) -> int:
        """Request current motor current setting (%)."""
        payload = self._request(MsgType.SET_CURRENT, timeout)
        return struct.unpack("<H", payload[:2])[0]

    def __repr__(self):
        label = "broadcast" if self.node_id == 0 else f"node_id={self.node_id}"
        return f"<StepperNode {label}>"


# ---------------------------------------------------------------------------
# Example usage
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    HOST = "canbus-stepper-1.local"  # adjust to your node's hostname or IP

    with connect(HOST) as bus:
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