#!/usr/bin/python

import dronekit
import time
from dronekit import connect, mavutil

vehicle = connect('/dev/ttyS7', baud=115200, wait_ready=False)

vehicle.gimbal.rotate(0, 0, 0)

camcontrol_msg = vehicle.message_factory.command_long_encode(
    target_system = 0,
    target_component = 1,
    confirmation = 0,
    command = mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
    param1 = 1,
    param2 = 0,
    param3 = 0,
    param4 = 0,
    param5 = 1,
    param6 = 0,
    param7 = 0
)

# vehicle.send_mavlink(camcontrol_msg)
# time.sleep(8)

count = 0
pitch = 30
yaw = -90
roll = 0

while(True):
    vehicle.gimbal.rotate(pitch, roll, yaw)
    if(count % 2 == 0):
        pitch = -90
        yaw = 90
        # roll = -20
    else:
        pitch = 30
        yaw = -90
        # roll = 20
    time.sleep(2)
    count = count + 1
    if(count > 90):
        break
    print("count :", count)
    vehicle.send_mavlink(camcontrol_msg)

print("Finished")
