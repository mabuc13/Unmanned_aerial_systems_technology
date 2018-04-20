#!/usr/bin/env python

'''
This script requests and prints the battery parameters

It has been tested using a Pixhawk 2.1 flight controller running PX4

Revision
2018-04-18 Mark Buch, Anders Karlsen, Mathias Madsen
'''
# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
update_interval = 1
target_system = 1 # Pixhawk2 PX4
#target_system = 66 # AutoQuad
target_component = 0

# defines
MAVLINK_MSG_ID_BATTERY_STATUS_MESSAGE = 147
MAVLINK_MSG_ID_BATTERY_STATUS_MESSAGE_LEN = 36
MAVLINK_MSG_ID_PARAM_VALUE = 147

# imports
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg

# variables
msg = mavlink_lora_msg()
request_sent = False

def on_mavlink_msg (msg):
    voltages = [0, 0 ,0 ,0, 0 ,0 ,0 ,0 ,0 ,0]
    if msg.msg_id == MAVLINK_MSG_ID_PARAM_VALUE:
        (current_consumed, energy_consumed, temperature, voltages[0], voltages[1], voltages[2], voltages[3], voltages[4], voltages[5], voltages[6], voltages[7], voltages[8], voltages[9], current_battery, lid, battery_function, ltype, battery_remaining) = struct.unpack('<iih10HhBBBb', msg.payload)
        print current_consumed, energy_consumed, temperature, voltages[0], voltages[1], voltages[2], voltages[3], voltages[4], voltages[5], voltages[6], voltages[7], voltages[8], voltages[9], current_battery, lid, battery_function, ltype, battery_remaining

def send_mavlink_battery_status_message():
    # no need to set sys_id, comp_id or checksum, this is handled by the mavlink_lora node.
    msg.header.stamp = rospy.Time.now()
    msg.msg_id = MAVLINK_MSG_ID_BATTERY_STATUS_MESSAGE
    msg.payload_len = MAVLINK_MSG_ID_BATTERY_STATUS_MESSAGE_LEN
    msg.payload = struct.pack('<BB', target_system, target_component)
    mavlink_msg_pub.publish(msg)

# launch node
rospy.init_node('mavlink_lora_get_battery_status_message')
mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0) # mavlink_msg publisher
rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, on_mavlink_msg) # mavlink_msg subscriber
rate = rospy.Rate(update_interval)
rospy.sleep (1) # wait until everything is running

# loop until shutdown
while not (rospy.is_shutdown()):
    # do stuff
    if request_sent == False:
        print 'Requesting battery status message'
        send_mavlink_battery_status_message()
        request_sent = True

    # sleep the defined interval
    rate.sleep()

