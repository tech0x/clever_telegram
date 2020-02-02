#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

"""

"""

import sys, select, os
os.environ['MAVLINK20'] = '1'

import rospy
import mavros

#import pdb

from os import listdir
from os.path import isfile, join
from threading import Event
from functools import wraps
from pymavlink import mavutil
from mavros import mavlink, param, command

from mavros_msgs.msg import State, OpticalFlowRad, Mavlink, ParamValue

def mavlink_message_handler(msg):
    print link.decode(mavlink.convert_to_bytes(msg))

if __name__ == '__main__':
    mavros.set_namespace()
    recv_event = Event()
    link = mavutil.mavlink.MAVLink('', 255, 1)
    pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
    mavlink_recv = ''
    rospy.init_node('mavlink_dump')
    sub = rospy.Subscriber('mavlink/from', Mavlink, mavlink_message_handler)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
       msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
       msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
       ros_msg = mavlink.convert_to_rosmsg(msg)
       pub.publish(ros_msg)
       r.sleep()

