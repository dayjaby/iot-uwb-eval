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
mav_serial = mavutil.mavserial(device="/dev/uwb_node_0001", source_system=1, source_component=common.MAV_COMP_ID_AUTOPILOT1)

def on_connect(mqtt_client, userdata, flags, rc):
    print("subscribing...")
    mqtt_client.subscribe("owntracks//m20lte", 2)

def on_message(mqtt_client, userdata, message):
    if message.topic != "owntracks//m20lte":
        return
    data = json.loads(message.payload)
    try:
        msg = common.MAVLink_gps_raw_int_message(
            time_usec=int(data["created_at"]*1e3),
            fix_type=2, # 0=no fix, 2=2D fix, 4=DGPS
            lat=int(data["lat"]*1e7),
            lon=int(data["lon"]*1e7),
            alt=int(data["alt"]*1000),
            eph=int(float(data["vac"])*100),
            epv=0,
            vel=0xFFFF,
            cog=0xFFFF,
            satellites_visible=0xFF
        )
        print(msg)
        #msg.pack(mav_serial.mav) # not necessary when we send the message, however helpful when we want to explore the serialized data without sending it
        mav_serial.mav.send(msg)
    except Exception as e:
        print(e)

client.on_message = on_message
client.on_connect = on_connect
client.connect("127.0.0.1", 1883, 60)


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

client.loop_start()
while True:
    try:
        r = mav_serial.recv_msg()
        if r is not None:
            if r.get_msgId() == common.MAVLINK_MSG_ID_DISTANCE_SENSOR:
                roll, pitch, yaw = quaternion_to_euler(r.quaternion)
                data = {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "distance": r.current_distance / 100.0, # cm to m
                    "gps_distance": r.max_distance / 100.0, # cm to m
                }
                print(data)
                client.publish("drone/{}/peer/{}/distance".format(r.get_srcSystem(), r.id), json.dumps(data))
            elif r.get_msgId() in [common.MAVLINK_MSG_ID_NAMED_VALUE_INT, common.MAVLINK_MSG_ID_NAMED_VALUE_FLOAT]:
                print("{}={}".format(r.name, r.value))
    except Exception as e:
        print("Error parsing MAVLink: ", e)
client.loop_stop()
