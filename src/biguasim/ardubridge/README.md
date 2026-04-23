# ardupilot_holoocean

## Test Mock Simulator

1.  sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
2.  python bridge_mavlink.py
3.  In MAVProxy terminal:
    -   Type status, alt, or gps and see data changing.
    -   That confirms MAVLink messages are being received.


