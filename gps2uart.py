import pynmea2
from pymavlink.dialects.v20 import common
from pymavlink import mavutil
from datetime import datetime, timedelta
import time
import binascii
import serial
import math
import json
import paho.mqtt.client as mqtt

client = mqtt.Client()

client.connect("127.0.0.1", 1883, 60)

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

def quaternion_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2 * (w * y - z  * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(90, sinp)
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw

ser = serial.Serial("/dev/gps_node_A10JWLOY", 9600)
while True:
    client.loop() 
    try:
        line = ser.readline().decode("utf8").strip()
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
            #msg.pack(mav_serial.mav) # not necessary when we send the message, however helpful when we want to explore the serialized data without sending it
            mav_serial.mav.send(msg)

        r = mav_serial.recv_msg()
        if r is not None:
            if r.get_msgId() == common.MAVLINK_MSG_ID_DISTANCE_SENSOR:
                print(r)
                roll, pitch, yaw = quaternion_to_euler(r.quaternion)
                data = {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "distance": r.current_distance # TODO: / 100.0 # cm to m
                }
                client.publish("drone/{}/peer/{}/distance".format(r.get_srcSystem(), r.id), json.dumps(data))
    except:
        pass # silently ignore any NMEA parsing errors
