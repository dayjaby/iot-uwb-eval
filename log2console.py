import pynmea2
from pymavlink.dialects.v10 import common
from datetime import datetime, timedelta
import time
import binascii

mav = common.MAVLink(None, srcSystem=1, srcComponent=mav.MAV_COMP_ID_AUTOPILOT1)
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

with open("log.txt", "r") as f:
    for line in f:
        gp = pynmea2.parse(line)
        if line.startswith("$GPGGA"):
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
            msg.pack(mav)
            print(mav_to_dict(msg))

