"""
GroundStation — a minimal MAVLink ground station (pymavlink) to script ArduPilot missions against BiguaSim: wait
until the autopilot can arm, change mode, arm, take off, send position targets and RC overrides, read the state.

    gcs = GroundStation(sitl.mavlink_url)
    gcs.wait_ready()
    gcs.set_mode("GUIDED"); gcs.arm(); gcs.takeoff(10)
"""

from __future__ import annotations

import threading
import time

from pymavlink import mavutil


class GroundStation:

    def __init__(self, url: str = "tcp:127.0.0.1:5760", timeout: float = 60.0):
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.mav = mavutil.mavlink_connection(url, source_system=255, autoreconnect=True)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.5)
        if self.mav.wait_heartbeat(timeout=timeout) is None:
            raise TimeoutError(f"No heartbeat from {url}")
        # Heartbeats once a second, as a ground station does: without them ArduSub's GCS failsafe disarms.
        # pymavlink's sends are not thread safe, so every send goes through one lock.
        lock, send = threading.Lock(), self.mav.mav.send

        def locked_send(*args, **kwargs):
            with lock:
                send(*args, **kwargs)

        self.mav.mav.send = locked_send
        self._closed = threading.Event()
        self._rc = None
        threading.Thread(target=self._heartbeats, daemon=True).start()
        self.mav.mav.request_data_stream_send(self.mav.target_system, self.mav.target_component,
                                              mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
        self.messages = []          # STATUSTEXT messages received

    def _heartbeats(self):
        """Heartbeat at 1 Hz, and the RC override at 5 Hz: ArduPilot drops an override after RC_OVERRIDE_TIME."""
        tick = 0
        while not self._closed.wait(0.2):
            try:
                if tick % 5 == 0:
                    self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                                0, 0, 0)
                if self._rc is not None:
                    self.mav.mav.rc_channels_override_send(self.mav.target_system, self.mav.target_component,
                                                           *self._rc)
            except OSError:
                return
            tick += 1

    # ------------------------------------------------------------------
    # Reading
    # ------------------------------------------------------------------

    def recv(self, kind: str, timeout: float = 5.0):
        """The next message of a type (blocking), recording STATUSTEXT on the way."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.mav.recv_match(type=[kind, "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                self.messages.append(msg.text)
                if kind != "STATUSTEXT":
                    continue
            return msg
        raise TimeoutError(f"No {kind} message within {timeout} s")

    def latest(self, kind: str, timeout: float = 5.0):
        """The newest message of a type: the queued ones are skipped (they can be seconds old)."""
        newest = None
        while True:
            msg = self.mav.recv_match(type=[kind, "STATUSTEXT"], blocking=False)
            if msg is None:
                break
            if msg.get_type() == "STATUSTEXT":
                self.messages.append(msg.text)
            else:
                newest = msg
        return newest if newest is not None else self.recv(kind, timeout)

    def position(self):
        """(north, east, down) in m from the EKF origin."""
        msg = self.latest("LOCAL_POSITION_NED")
        return msg.x, msg.y, msg.z

    def attitude(self):
        """(roll, pitch, yaw) in rad."""
        msg = self.latest("ATTITUDE")
        return msg.roll, msg.pitch, msg.yaw

    def depth(self) -> float:
        """Depth below the surface (ArduSub: altitude from the barometer), m."""
        return -self.latest("VFR_HUD").alt

    def param(self, name: str) -> float:
        self.mav.mav.param_request_read_send(self.mav.target_system, self.mav.target_component, name.encode(), -1)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            msg = self.mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is not None and msg.param_id == name:
                return msg.param_value
        raise TimeoutError(f"No value for {name}")

    def set_param(self, name: str, value: float) -> None:
        self.mav.mav.param_set_send(self.mav.target_system, self.mav.target_component, name.encode(), float(value),
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        if abs(self.param(name) - value) > 1e-4 * max(1.0, abs(value)):
            raise RuntimeError(f"{name} did not take {value}")

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def wait_ready(self, timeout: float = 180.0) -> None:
        """Waits until the pre-arm checks pass and the EKF uses the GPS: an absolute horizontal position, not the
        constant-position mode it starts in (ArduSub accepts GUIDED and AUTO only then)."""
        deadline = time.monotonic() + timeout
        prearm_ok = position_ok = False
        while time.monotonic() < deadline:
            msg = self.mav.recv_match(type=["SYS_STATUS", "EKF_STATUS_REPORT", "STATUSTEXT"], blocking=True,
                                      timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                self.messages.append(msg.text)
            elif msg.get_type() == "SYS_STATUS":
                prearm_ok = bool(msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
            else:
                position_ok = bool(msg.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS
                                   and not msg.flags & mavutil.mavlink.EKF_CONST_POS_MODE)
            if prearm_ok and position_ok:
                return
        raise TimeoutError(f"Not ready after {timeout} s (pre-arm {prearm_ok}, EKF position {position_ok}): "
                           f"{self.messages[-5:]}")

    def set_mode(self, mode: str, timeout: float = 10.0) -> None:
        mode_id = self.mav.mode_mapping()[mode]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.mav.set_mode(mode_id)
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is not None and msg.custom_mode == mode_id:
                return
        raise TimeoutError(f"Mode {mode} not set: {self.messages[-5:]}")

    def arm(self, timeout: float = 30.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.mav.arducopter_arm()
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is not None and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return
            time.sleep(0.5)
        raise TimeoutError(f"Not armed: {self.messages[-5:]}")

    def disarm(self) -> None:
        self.mav.arducopter_disarm()

    def takeoff(self, altitude: float) -> None:
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    def vtol_takeoff(self, altitude: float) -> None:
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    def goto_local(self, north: float, east: float, down: float, yaw: float | None = None) -> None:
        """GUIDED position target, NED from the EKF origin (m); ``yaw`` in rad, None to keep the yaw free."""
        # type_mask: use the position, ignore velocity and acceleration (bits 3-8) and the yaw rate (bit 11),
        # and the yaw (bit 10) unless given
        mask = 0b1001_1111_1000 | (0 if yaw is not None else 1 << 10)
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask, north, east, down, 0, 0, 0, 0, 0, 0, yaw or 0.0, 0)

    def goto_global(self, lat: float, lon: float, alt: float, yaw: float | None = None) -> None:
        """GUIDED position target at a latitude/longitude (deg) and an altitude above mean sea level (m).

        Local NED targets count from the EKF origin, where the vehicle was when the EKF got its first GPS fix; this
        one is absolute: with the bridge, the AMSL altitude is the world height above the ``origin`` the runner
        maps to ArduPilot's home (``--home`` altitude 0), so for ArduSub ``alt=-3`` is 3 m below the surface."""
        mask = 0b1001_1111_1000 | (0 if yaw is not None else 1 << 10)
        self.mav.mav.set_position_target_global_int_send(
            0, self.mav.target_system, self.mav.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_INT, mask,
            int(lat * 1e7), int(lon * 1e7), alt, 0, 0, 0, 0, 0, 0, yaw or 0.0, 0)

    def rc_override(self, channels: dict | None) -> None:
        """RC override, ``{channel (1-based): pwm}``, repeated until changed; channels not given are released.
        ``None`` releases them all."""
        if channels is None:
            self._rc = None
            values = [0] * 18
        else:
            values = [65535] * 18
            for channel, pwm in channels.items():
                values[channel - 1] = int(pwm)
            self._rc = values
        self.mav.mav.rc_channels_override_send(self.mav.target_system, self.mav.target_component, *values)

    def close(self) -> None:
        self._closed.set()
        self.mav.close()
