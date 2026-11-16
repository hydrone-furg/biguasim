#!/usr/bin/env bash
# Builds the ArduPilot SITL firmwares the BiguaSim vehicles fly with (arducopter, ardusub, ardurover, arduplane),
# with the firmware patches of firmware/patches applied to the checkout.
#
#   firmware/build.sh [ardupilot checkout]      (default: $ARDUPILOT_HOME, else the ardupilot submodule)
#
# ArduPilot's code generators need empy 3.x (empy 4 breaks them), pexpect, future, dronecan and pymavlink. They do
# not have to be in the BiguaSim environment, e.g.:
#   conda create -n ardupilot python=3.11 && conda run -n ardupilot pip install "empy==3.3.4" pexpect future \
#       dronecan pymavlink
#   PYTHON=$(conda run -n ardupilot which python) firmware/build.sh
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ardupilot="${1:-${ARDUPILOT_HOME:-$here/../ardupilot}}"
python="${PYTHON:-python3}"
cd "$ardupilot"

# Only the submodules a SITL build needs
git submodule update --init --recursive --depth 1 modules/waf modules/mavlink modules/DroneCAN modules/littlefs \
    modules/lwip modules/Micro-CDR modules/Micro-XRCE-DDS-Client

for patch in "$here"/patches/*.patch; do
    if git apply --reverse --check "$patch" 2>/dev/null; then
        echo "already applied: $(basename "$patch")"
    else
        git apply "$patch"
        echo "applied: $(basename "$patch")"
    fi
done

export PATH="$(dirname "$("$python" -c 'import sys; print(sys.executable)')"):$PATH"
./waf configure --board sitl
# ArduHydrone (the Hydrone's ArduCopter + ArduSub fusion) lives on the submodule's hydrone branch
targets="copter sub rover plane"
[[ -d ArduHydrone ]] && targets="$targets hydrone"
./waf -j "$(nproc)" $targets
ls -l build/sitl/bin
