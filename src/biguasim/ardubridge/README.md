# biguasim.ardubridge

Flies each BiguaSim vehicle with its ArduPilot firmware (SITL, JSON physics backend), in lockstep with the client
dynamics. Full guide: `biguasim_docs/ardubridge/ardubridge.rst`.

| Vehicle | Firmware |
|---|---|
| DjiMatrice | ArduCopter (quad X) |
| BlueBoat | Rover (boat, skid steering) |
| BlueROV2, BlueROV2Heavy | ArduSub (vectored, vectored 6DOF) |
| TorpedoAUV | ArduSub, torpedo frame (`firmware/patches`) |
| Hydrone | ArduPlane QuadPlane + `firmware/scripts/hydrone_medium.lua` (water mode) |

```bash
# once: the SITL firmwares, with BiguaSim's patches (needs a Python with empy 3.3.4)
PYTHON=$(conda run -n ardupilot which python) firmware/build.sh

python -m biguasim.ardubridge --list
python -m biguasim.ardubridge BlueROV2 --package SkyDive --world Pier-Harbor --location 38 -301 0
python -m biguasim.ardubridge DjiMatrice --offline          # no engine
```

Then connect QGroundControl / MAVProxy to UDP 14550 (or TCP 5760).
