"""
Launches an ArduPilot SITL firmware for a BiguaSim vehicle: the JSON physics backend, the vehicle's parameter files
and Lua scripts.

Ports of instance ``I``: JSON physics on UDP ``9002 + 10 I`` (the bridge listens there), MAVLink on TCP
``5760 + 10 I`` (SERIAL0, does not wait for a client) and ``5762 + 10 I`` (SERIAL1), and MAVLink streamed to UDP
``14550`` (SERIAL2, where QGroundControl and MAVProxy listen by default).
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import tempfile
from pathlib import Path

from .vehicle import FIRMWARE_DIR, VehicleProfile

_DEFAULT_HOME = (33.810313, -118.393867, 0.0, 0.0)   # lat, lon, alt (m), heading (deg)


def ardupilot_home() -> Path:
    """The ArduPilot checkout: ``$ARDUPILOT_HOME``, else the ``ardupilot`` submodule next to this package."""
    return Path(os.environ.get("ARDUPILOT_HOME", Path(__file__).resolve().parent / "ardupilot"))


class SITL:
    """One ArduPilot SITL process for a vehicle profile. Use as a context manager, or call start() / stop()."""

    def __init__(
        self,
        profile: VehicleProfile,
        instance: int = 0,
        home: tuple = _DEFAULT_HOME,
        ardupilot_dir: str | Path | None = None,
        run_dir: str | Path | None = None,
        wipe: bool = True,
        extra_params: dict | None = None,
        extra_args: tuple = (),
        log_file: str | Path | None = None,
    ):
        """
        :param profile: Vehicle profile: firmware, parameter files and scripts.
        :param instance: SITL instance (``-I``); shifts every port by ``10 * instance``.
        :param home: ``(lat, lon, alt, heading)`` of ArduPilot's home, which stands for the bridge's ``origin``.
        :param ardupilot_dir: ArduPilot checkout with the SITL binaries built (default :func:`ardupilot_home`).
        :param run_dir: Working directory (EEPROM, logs, ``scripts/``). Default: a new temporary directory.
        :param wipe: Start from the default parameters (``--wipe``) instead of the EEPROM of a previous run.
        :param extra_params: Parameters set on top of the profile's files.
        :param extra_args: More command line arguments for the SITL binary.
        :param log_file: Where the firmware's console output goes (default ``<run_dir>/sitl.log``).
        """
        self.profile = profile
        self.instance = instance
        self.home = tuple(home)
        self.ardupilot_dir = Path(ardupilot_dir) if ardupilot_dir else ardupilot_home()
        self.run_dir = Path(run_dir) if run_dir else Path(tempfile.mkdtemp(prefix=f"biguasim-{profile.name}-"))
        self.wipe = wipe
        self.extra_params = dict(extra_params or {})
        self.extra_args = tuple(extra_args)
        self.log_file = Path(log_file) if log_file else self.run_dir / "sitl.log"
        self._process = None
        self._log = None

    # ------------------------------------------------------------------

    @property
    def binary(self) -> Path:
        return self.ardupilot_dir / "build" / "sitl" / "bin" / self.profile.firmware

    @property
    def json_port(self) -> int:
        return 9002 + 10 * self.instance

    @property
    def mavlink_url(self) -> str:
        """pymavlink connection string of SERIAL0."""
        return f"tcp:127.0.0.1:{5760 + 10 * self.instance}"

    def command(self) -> list:
        defaults = [str(p) for p in self.profile.param_files(self.ardupilot_dir)]
        if self.extra_params:
            extra = self.run_dir / "extra.parm"
            extra.write_text("".join(f"{k} {v}\n" for k, v in self.extra_params.items()))
            defaults.append(str(extra))
        lat, lon, alt, heading = self.home
        cmd = [str(self.binary), "--model", "JSON:127.0.0.1", "-I", str(self.instance),
               "--home", f"{lat},{lon},{alt},{heading}", "--defaults", ",".join(defaults),
               "--serial0", "tcp:0", "--serial2", "udpclient:127.0.0.1:14550"]
        if self.wipe:
            cmd.append("--wipe")
        return cmd + list(self.extra_args)

    def check(self) -> None:
        """Raises with what to do when the firmware is not built, or not built with the profile's patches."""
        if not self.binary.is_file():
            raise FileNotFoundError(
                f"{self.binary} not found: build the SITL firmwares with {FIRMWARE_DIR / 'build.sh'}")
        for patch in self.profile.patches:
            marker = _PATCH_MARKERS.get(patch)
            if marker and marker not in self.binary.read_bytes():
                raise RuntimeError(f"{self.binary.name} was built without {patch}: run {FIRMWARE_DIR / 'build.sh'}")
        for file in self.profile.param_files(self.ardupilot_dir) + self.profile.script_files():
            if not Path(file).is_file():
                raise FileNotFoundError(file)

    def start(self) -> "SITL":
        self.check()
        self.run_dir.mkdir(parents=True, exist_ok=True)
        scripts = self.profile.script_files()
        if scripts:
            # SITL runs the Lua scripts of ./scripts
            (self.run_dir / "scripts").mkdir(exist_ok=True)
            for script in scripts:
                shutil.copy(script, self.run_dir / "scripts" / script.name)
        self._log = open(self.log_file, "w")
        self._process = subprocess.Popen(self.command(), cwd=self.run_dir, stdout=self._log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
        return self

    def poll(self):
        """None while running, else the exit code."""
        return None if self._process is None else self._process.poll()

    def stop(self, timeout: float = 5.0) -> None:
        if self._process is not None and self._process.poll() is None:
            self._process.send_signal(signal.SIGTERM)
            try:
                self._process.wait(timeout)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
        self._process = None
        if self._log is not None:
            self._log.close()
            self._log = None

    def __enter__(self) -> "SITL":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


# Strings a firmware patch adds to the binary, to tell a patched build from a stock one.
_PATCH_MARKERS = {
    "ardusub-torpedo-frame.patch": b"TORPEDO",
}
