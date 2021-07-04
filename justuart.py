import pynmea2
from pymavlink.dialects.v10 import common
from pymavlink import mavutil
from datetime import datetime, timedelta
import time
import binascii
import serial

mav_serial = mavutil.mavserial(device="/dev/uwb_node_0001", source_system=1, source_component=common.MAV_COMP_ID_AUTOPILOT1)
#mav_serial = mavutil.mavserial(device="/dev/ttyUSB2", source_system=1, source_component=common.MAV_COMP_ID_AUTOPILOT1)
# mav.disable_signing()
# MAV_COMP_ID_USER1 shall be the component ID for the UWB device

FRAMING_OK = 1
FRAMING_BAD_CRC = 2
FRAMING_BAD_SIGNATURE = 3

MAVLINK_V10 = 254
MAVLINK_V20 = 253

while True:
    r = mav_serial.recv_msg()
    if r is not None:
        # if r.get_msgId() == common.MAVLINK_MSG_ID_HEARTBEAT:
        print(r)
