import pynmea2
from pymavlink.dialects.v10 import common
from pymavlink import mavutil
from datetime import datetime, timedelta
import time
import binascii
import serial

mav_serial = mavutil.mavserial(device="/dev/uwb_node_0001", source_system=1, source_component=common.MAV_COMP_ID_AUTOPILOT1)
# mav.disable_signing()
# MAV_COMP_ID_USER1 shall be the component ID for the UWB device

FRAMING_OK = 1
FRAMING_BAD_CRC = 2
FRAMING_BAD_SIGNATURE = 3

MAVLINK_V10 = 254
MAVLINK_V20 = 253

def mav_to_dict(mavmsg):
    return dict(
            framing_status=FRAMING_OK,
            magic=MAVLINK_V10,
            len=len(mavmsg.get_payload()),
            seq=mavmsg.get_seq(),
            sysid=mavmsg.get_srcSystem(),
            compid=mavmsg.get_srcComponent(),
            msgid=mavmsg.get_msgId(),
            checksum=mavmsg.get_crc(),
            payload16=binascii.hexlify(mavmsg.get_payload())
        )

for line in open("log.txt", "r"):
    if line.startswith("$GPGGA"):
        gp = pynmea2.parse(line)
        msg = common.MAVLink_gps_raw_int_message(
            time_usec=int(timedelta(hours=gp.timestamp.hour, minutes=gp.timestamp.minute, seconds=gp.timestamp.second).total_seconds()*1e6),
            #time_usec=int(datetime.utcnow().timestamp()*1e6),
            fix_type=gp.gps_qual*2, # 0=no fix, 1=2D fix, 2=DGPS
            lat=int(gp.latitude*1e7),
            lon=int(gp.longitude*1e7),
            alt=int(gp.altitude*1000),
            eph=int(float(gp.horizontal_dil)*100),
            epv=0,
            vel=0xFFFF,
            cog=0xFFFF,
            satellites_visible=int(gp.num_sats)
        )
        print(msg)
        mav_serial.mav.send(msg)
        time.sleep(0.5)

    r = mav_serial.recv_msg()
    if r is not None:
        print(r)
