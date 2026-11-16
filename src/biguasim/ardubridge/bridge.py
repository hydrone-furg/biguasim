"""
ArduPilot JSON SITL UDP bridge — vehicle-agnostic protocol handler.

Receives servo packets from ArduPilot (16 or 32 channels), converts PWM to simulator commands, converts the
simulator state to ArduPilot JSON, and sends it back. No BiguaSim dependency; wired to the environment by
ArduBiguaSimRunner.

ArduPilot runs in lockstep with the physics: it sends one servo packet per physics step and waits for the state.
The bridge answers every packet with exactly one state, so the simulator must advance one step per packet.
"""

from __future__ import annotations

import json
import socket
import struct
from typing import Optional

import numpy as np

from .frame import imu_from_dynamics, pos_nwu_to_ned, quat_glu2nwu_to_frd2ned, vel_nwu_to_ned
from .vehicle import VehicleProfile

_PACKETS = {   # magic → (struct format, size)
    18458: ("<HHI16H", struct.calcsize("<HHI16H")),   # 40 bytes
    29569: ("<HHI32H", struct.calcsize("<HHI32H")),   # 72 bytes (SERVO_32_ENABLE)
}
_MAX_PACKET = max(size for _, size in _PACKETS.values())


class ArduPilotBridge:
    """
    UDP bridge implementing the ArduPilot JSON SITL FDM protocol.

    Responsible for establishing two-way communication between ArduPilot and the simulator,
    receiving actuator commands (PWM) and sending the vehicle's sensory data (state) back.
    """

    def __init__(
        self,
        profile: VehicleProfile,
        address: str = "127.0.0.1",
        port: int = 9002,
        origin: tuple = (0.0, 0.0, 0.0),
    ):
        """
        Initializes the UDP communication bridge.

        :param profile: Vehicle profile containing motor mapping, converters, and constraints.
        :type profile: VehicleProfile
        :param address: Local IP address to bind the UDP socket. Defaults to "127.0.0.1".
        :type address: str
        :param port: Local port to listen for ArduPilot packets. Defaults to 9002 (SITL instance 0).
        :type port: int
        :param origin: World point (x, y, z in m) that stands for ArduPilot's home. Its height is the water surface
            for ArduSub, whose barometer reads the depth below home.
        :type origin: tuple
        """

        self._profile = profile
        self._address = address
        self._port = port
        self._origin = tuple(float(v) for v in origin)

        self._fcu_address: Optional[tuple] = None
        self._online = False
        self._last_frame = -1

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_buf = bytearray(_MAX_PACKET)  # pre-allocated; avoid hot-loop GC

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def bind(self) -> None:
        """
        Binds the UDP socket and drains any stale packets.

        :raises RuntimeError: If it fails to bind to the specified UDP socket.
        """
        try:
            self._sock.bind((self._address, self._port))
        except socket.error as exc:
            raise RuntimeError(f"Failed to bind UDP socket {self._address}:{self._port}: {exc}") from exc
        self._drain_stale_packets()

    def receive_pwm(self, timeout: float = 1.0) -> tuple:
        """
        Waits for one servo packet.

        :returns: ``(frame_count, pwm_array)``, or ``(None, None)`` on timeout or an invalid packet.
        :rtype: tuple
        """
        self._sock.settimeout(timeout)
        try:
            nbytes, addr = self._sock.recvfrom_into(self._recv_buf)
        except socket.timeout:
            if self._online:
                print("ArduPilot stopped sending servo packets.")
                self._online = False
            return None, None
        except OSError:
            return None, None

        magic = struct.unpack_from("<H", self._recv_buf)[0] if nbytes >= 2 else None
        if magic not in _PACKETS or nbytes != _PACKETS[magic][1]:
            print(f"Ignoring a {nbytes}-byte packet with magic {magic}")
            return None, None

        if self._fcu_address != addr:
            self._fcu_address = addr
            print(f"ArduPilot connected @ {addr[0]}:{addr[1]}")

        unpacked = struct.unpack_from(_PACKETS[magic][0], self._recv_buf)
        frame_count = unpacked[2]
        pwm = np.array(unpacked[3:], dtype=np.float32)

        if frame_count < self._last_frame:
            print("ArduPilot reset detected")
        self._online = True
        self._last_frame = frame_count
        return frame_count, pwm

    def pwm_to_motor_cmds(self, pwm: np.ndarray) -> list:
        """
        Converts the PWM values of one servo packet into the BiguaSim command of the vehicle.

        :param pwm: NumPy array containing the 16 (or 32) PWM channels received from ArduPilot.
        :type pwm: np.ndarray
        :returns: The command, one value per actuator of the vehicle's control abstraction.
        :rtype: list
        """
        return self._profile.pwm_to_command(pwm)

    def build_json_state(self, dynamics, sim_time: float, rangefinders=None) -> dict:
        """
        Converts one DynamicsSensor vector (NWU world, UseRPY false) to the ArduPilot JSON state (NED/FRD).

        :param dynamics: DynamicsSensor ``sensor_data`` (19 dynamics items, anything after them is ignored).
        :param sim_time: Current simulation time in seconds.
        :type sim_time: float
        :param rangefinders: Optional distances (m) sent as ``rng_1``.. ``rng_6``.
        :returns: Formatted dictionary ready to be serialized into JSON.
        :rtype: dict
        """
        d = np.asarray(dynamics, dtype=np.float64).ravel()
        if d.size < 19:
            raise ValueError(f"Need a DynamicsSensor with UseRPY false (19 items), got {d.size} items")
        if not np.all(np.isfinite(d[:19])):
            # ArduPilot's SITL traps floating point exceptions: a NaN would abort the firmware
            raise ValueError(f"Non-finite DynamicsSensor state at t={sim_time:.3f}: {np.round(d[:19], 4).tolist()}")
        accel, gyro = imu_from_dynamics(d[0:3], d[12:15], d[15:19])
        state = {
            "timestamp": float(sim_time),
            "imu": {"gyro": gyro, "accel_body": accel},
            "position": pos_nwu_to_ned(d[6:9], self._origin),
            "velocity": vel_nwu_to_ned(d[3:6]),
            "quaternion": quat_glu2nwu_to_frd2ned(d[15:19]),
        }
        for i, distance in enumerate(rangefinders or []):
            state[f"rng_{i + 1}"] = float(distance)
        return state

    def send_state(self, json_state: dict) -> None:
        """
        Serializes the state dictionary into JSON and sends it via UDP to the Flight Controller Unit (FCU).

        :param json_state: Dictionary generated by the :meth:`build_json_state` method.
        :type json_state: dict
        """
        if self._fcu_address is None:
            return
        payload = ("\n" + json.dumps(json_state, separators=(",", ":")) + "\n").encode("utf-8")
        self._sock.sendto(payload, self._fcu_address)

    @property
    def is_online(self) -> bool:
        """
        Checks the current connection status with ArduPilot.

        :returns: ``True`` if ArduPilot is actively sending packets, ``False`` otherwise.
        :rtype: bool
        """
        return self._online

    def close(self) -> None:
        """
        Closes the UDP communication socket.
        """
        self._sock.close()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _drain_stale_packets(self) -> None:
        """
        Internal method to empty the UDP receive buffer and avoid processing old packets.
        """
        self._sock.setblocking(False)
        try:
            while True:
                self._sock.recvfrom(_MAX_PACKET)
        except (BlockingIOError, socket.timeout):
            pass
        self._sock.setblocking(True)
