import pynmea2
from pymavlink.dialects.v10 import common
from pymavlink import mavutil
from datetime import datetime, timedelta
import time
import binascii

mav_serial = mavutil.mavserial(device="/dev/gps_node", source_system=1, source_component=common.MAV_COMP_ID_AUTOPILOT1)
# mav.disable_signing()
# MAV_COMP_ID_USER1 shall be the component ID for the UWB device

while True:
    msg = mav_serial.recv_msg()
    if msg:
        print(msg)
    time.sleep(1)
