"""
ArduPilot JSON SITL UDP bridge — vehicle-agnostic protocol handler.

Receives 40-byte servo packets from ArduPilot, converts PWM to simulator
commands, converts simulator state to ArduPilot JSON, and sends it back.
No BiguaSim dependency; wired to the environment by ArduBiguaSimRunner.
"""

from __future__ import annotations

import json
import socket
import struct
from typing import Optional

import numpy as np

from .frame import (
    depth_to_pressure,
    imu_glu_to_frd,
    pos_nwu_to_ap,
    quat_glu2nwu_to_frd2ned,
    vel_nwu_to_ned,
)
from .vehicle import VehicleProfile

_SERVO_PACKET_FMT = "<HHI16H"
_SERVO_PACKET_SIZE = struct.calcsize(_SERVO_PACKET_FMT)  # 40 bytes
_SERVO_MAGIC = 18458


class ArduPilotBridge:
    """UDP bridge implementing the ArduPilot JSON SITL FDM protocol."""

    def __init__(
        self,
        profile: VehicleProfile,
        address: str = "127.0.0.1",
        port: int = 9002,
        gps_origin: tuple = (33.810313, -118.393867),
    ):
        self._profile = profile
        self._address = address
        self._port = port
        self._gps_origin = gps_origin

        self._fcu_address: Optional[str] = None
        self._fcu_port: Optional[int] = None
        self._online = False
        self._timeout_count = 0
        self._last_frame = -1

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_buf = bytearray(_SERVO_PACKET_SIZE)  # pre-allocated; avoid hot-loop GC

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def bind(self) -> None:
        """Bind UDP socket and drain any stale packets. Call once before run()."""
        try:
            self._sock.bind((self._address, self._port))
            print(f"ArduPilot bridge bound @ {self._address}:{self._port}")
        except socket.error as exc:
            raise RuntimeError(f"Failed to bind UDP socket: {exc}") from exc
        self._drain_stale_packets()
        print("Drained stale packets – waiting for ArduPilot...")

    def receive_pwm(self) -> tuple:
        """
        Non-blocking receive of one servo packet.
        Returns (frame_count, pwm_array[16]) or (None, None) on timeout/error.
        """
        self._sock.settimeout(0.001 if not self._online else 0.01)
        try:
            nbytes, (addr, port) = self._sock.recvfrom_into(self._recv_buf)
        except socket.timeout:
            self._handle_timeout()
            return None, None
        except OSError:
            return None, None

        if nbytes != _SERVO_PACKET_SIZE:
            return None, None

        if self._fcu_address is None:
            self._fcu_address, self._fcu_port = addr, port
            print(f"ArduPilot connected @ {addr}:{port}")

        unpacked = struct.unpack_from(_SERVO_PACKET_FMT, self._recv_buf)
        magic, _frame_rate, frame_count = unpacked[0], unpacked[1], unpacked[2]
        pwm = np.array(unpacked[3:], dtype=np.float32)

        if magic != _SERVO_MAGIC:
            print(f"Unexpected magic: {magic}")
            return None, None

        self._check_frame_continuity(frame_count)
        if not self._online:
            print("ArduPilot online")
            self._online = True
        self._timeout_count = 0
        self._last_frame = frame_count
        return frame_count, pwm

    def pwm_to_motor_cmds(self, pwm: np.ndarray, frame_count: int) -> list:
        """Apply warmup guard, motor_mapping, pwm_converters, and motor_signs."""
        if frame_count < self._profile.warmup_frames:
            return [0.0] * self._profile.num_motors

        return [
            self._profile.motor_signs[i]
            * self._profile.pwm_converters[i](pwm[self._profile.motor_mapping[i]])
            for i in range(self._profile.num_motors)
        ]

    def build_json_state(self, agent_state: dict, sim_time: float) -> dict:
        """Convert BiguaSim agent state (NWU/GLU) to ArduPilot JSON state (NED/FRD)."""
        accel, gyro = imu_glu_to_frd(agent_state["IMUSensor"])
        pos = pos_nwu_to_ap(agent_state["LocationSensor"].tolist(), self._gps_origin)
        vel = vel_nwu_to_ned(agent_state["VelocitySensor"].tolist())
        quat = quat_glu2nwu_to_frd2ned(agent_state["DynamicsSensor"][-4:].tolist())

        state = {
            "timestamp": sim_time,
            "imu": {"gyro": gyro, "accel_body": accel},
            "position": pos,
            "velocity": vel,
            "quaternion": quat,
        }

        if "DepthSensor" in agent_state:
            depth_val = agent_state["DepthSensor"]
            z_up = float(depth_val[0]) if hasattr(depth_val, "__len__") else float(depth_val)
            state["pressure"] = depth_to_pressure(z_up)

        return state

    def send_state(self, json_state: dict) -> None:
        """Serialise and send JSON state to ArduPilot FCU."""
        if self._fcu_address is None:
            return
        payload = ("\n" + json.dumps(json_state, separators=(",", ":")) + "\n").encode("utf-8")
        self._sock.sendto(payload, (self._fcu_address, self._fcu_port))

    @property
    def is_online(self) -> bool:
        return self._online

    def close(self) -> None:
        self._sock.close()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _drain_stale_packets(self) -> None:
        self._sock.setblocking(False)
        try:
            while True:
                self._sock.recvfrom(_SERVO_PACKET_SIZE)
        except BlockingIOError:
            pass
        self._sock.setblocking(True)

    def _handle_timeout(self) -> None:
        if self._online:
            self._timeout_count += 1
            if self._timeout_count > 5:
                print("ArduPilot connection timeout — resetting.")
                self._online = False
                self._timeout_count = 0

    def _check_frame_continuity(self, frame_count: int) -> None:
        if frame_count < self._last_frame:
            print("ArduPilot reset detected")
        elif self._last_frame >= 0 and frame_count > self._last_frame + 1:
            print(f"Missed {frame_count - self._last_frame - 1} frame(s)")
