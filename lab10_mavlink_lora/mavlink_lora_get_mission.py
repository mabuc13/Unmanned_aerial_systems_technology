#!/usr/bin/env python

'''
This script requests and prints the battery parameters

It has been tested using a Pixhawk 2.1 flight controller running PX4

Revision
2018-04-18 Mark Buch, Anders Karlsen, Mathias Madsenn
'''
# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
update_interval = 1
target_system = 1 # Pixhawk2 PX4
#target_system = 66 # AutoQuad
target_component = 0

# defines
MAVLINK_MSG_ID_MISSION_REQUEST_LIST = 43
MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN = 2

MAVLINK_MSG_ID_MISSION_REQUEST = 40
MAVLINK_MSG_ID_MISSION_REQUEST_LEN = 3

MAVLINK_MSG_ID_MISSION_ITEM = 39
MAVLINK_MSG_ID_MISSION_ITEM_LEN = 14

MAVLINK_MSG_ID_MISSION_COUNT = 44
MAVLINK_MSG_ID_MISSION_REQUEST_LEN = 3

MAVLINK_MSG_ID_MISSION_ACK = 47
MAVLINK_MSG_ID_MISSION_ACK_LEN = 3

# imports
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg

# variables
msg = mavlink_lora_msg()
request_sent = False
global mission_count
global current_mission

def on_mavlink_msg (msg):
    print msg.msg_id
    global mission_count
    global current_mission
    if msg.msg_id == MAVLINK_MSG_ID_MISSION_COUNT:
        (count, target_system, target_component) = struct.unpack('<HBB', msg.payload)
        print 'Number of missions recieved: ',count
        mission_count = count
        current_mission = 0
        send_mavlink_mission_item_request(current_mission)

    if msg.msg_id == MAVLINK_MSG_ID_MISSION_ITEM:
        (param1, param2, param3, param4, x, y, z, seq, command, target_system, target_component, frame, current, autocontinue) = struct.unpack('<fffffffHHBBBBB', msg.payload)
        print 'Mission item: ' , param1, param2, param3, param4, x, y, z, seq, command, target_system, target_component, frame, current, autocontinue
        current_mission = current_mission + 1
        if not(current_mission > mission_count):
            send_mavlink_mission_item_request(current_mission)
        else:
            send_mission_ack()

def send_mavlink_mission_request(): #Send request for number of missions
    # no need to set sys_id, comp_id or checksum, this is handled by the mavlink_lora node.
    msg.header.stamp = rospy.Time.now()
    msg.msg_id = MAVLINK_MSG_ID_MISSION_REQUEST_LIST
    msg.payload_len = MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN
    msg.payload = struct.pack('<BB', target_system, target_component)
    mavlink_msg_pub.publish(msg)

def send_mavlink_mission_item_request(seq):#Send request for one mission
    msg.header.stamp = rospy.Time.now()
    msg.msg_id = MAVLINK_MSG_ID_MISSION_REQUEST
    msg.payload_len = MAVLINK_MSG_ID_MISSION_REQUEST_LEN
    msg.payload = struct.pack('<HBB', seq, target_system, target_component)
    mavlink_msg_pub.publish(msg)

def send_mission_ack():
    msg.header.stamp = rospy.Time.now()
    msg.msg_id = MAVLINK_MSG_ID_MISSION_ACK
    msg.payload_len = MAVLINK_MSG_ID_MISSION_ACK_LEN
    msg.payload = struct.pack('<BB', target_system, target_component)
    mavlink_msg_pub.publish(msg)

# launch node
rospy.init_node('mavlink_lora_mission_list')
mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0) # mavlink_msg publisher
rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, on_mavlink_msg) # mavlink_msg subscriber
rate = rospy.Rate(update_interval)
rospy.sleep (1) # wait until everything is running

# loop until shutdown
while not (rospy.is_shutdown()):
    # do stuff
    if request_sent == False:
        print 'Requesting mission'
        send_mavlink_mission_request()
        request_sent = True

    # sleep the defined interval
    rate.sleep()

