#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

"""

"""

from time import sleep
import sys, signal, select, os
os.environ['MAVLINK20'] = '1'
from timeit import default_timer as timer

import sched, time
import datetime
import rospy
import mavros
import rospkg

import pdb

from os import listdir
from os.path import isfile, join
from functools import wraps
from pymavlink import mavutil
from mavros_msgs.msg import State, OpticalFlowRad, Mavlink, ParamValue
from mavros import mavlink, param, command

filename = ''

class MavrosLogStreaming():
    def __init__(self, mav, output_filename=".test.ulg"):
        self.mav = mav

        self.got_ulog_header = False
        self.got_header_section = False
        self.ulog_message = []
        self.file = open(output_filename,'wb')
        self.start_time = timer()
        self.last_sequence = -1
        self.logging_started = False
        self.num_dropouts = 0
        self.target_component = 1

        self.start_time = timer()
        self.measure_time_start = timer()
        self.measured_data = 0
        self.m = False
        self.ulog = False

    def check_sequence(self, seq):
        if self.last_sequence == -1:
            return True, 0
        if seq == self.last_sequence:
            return False, 0
        if seq > self.last_sequence:
            if seq - self.last_sequence > (1<<15):
                return False, 0
            return True, seq - self.last_sequence - 1
        else:
            if self.last_sequence - seq > (1<<15):
                return True, (1<<16) - self.last_sequence - 1 + seq
            return False, 0

    def process_streamed_ulog_data(self, data, first_msg_start, num_drops):
        if not self.got_ulog_header:
            if len(data) < 16:
                raise Exception('first received message too short')
            self.file.write(bytearray(data[0:16]))
            data = data[16:]
            self.got_ulog_header = True

        if self.got_header_section and num_drops > 0:
            if num_drops > 25: num_drops = 25
            self.file.write(bytearray([ 2, 0, 79, num_drops*10, 0 ]))

        if num_drops > 0:
            self.write_ulog_messages(self.ulog_message)
            self.ulog_message = []
            if first_msg_start == 255:
                return
            data = data[first_msg_start:]
            first_msg_start = 0

        if first_msg_start == 255 and len(self.ulog_message) > 0:
            self.ulog_message.extend(data)
            return

        if len(self.ulog_message) > 0:
            self.file.write(bytearray(self.ulog_message + data[:first_msg_start]))
            self.ulog_message = []

        data = self.write_ulog_messages(data[first_msg_start:])
        self.ulog_message = data

    def write_ulog_messages(self, data):
        while len(data) > 2:
            message_length = data[0] + data[1] * 256 + 3
            if message_length > len(data):
                break
            self.file.write(bytearray(data[:message_length]))
            data = data[message_length:]
        return data

    def message_handler(self, msg):
        is_newer = False
        num_drops = 0

        if msg.msgid == 266 or msg.msgid == 77 or msg.msgid == 267:
            x = link.decode(self.mav.convert_to_bytes(msg))
            #pdb.set_trace()

            m, first_msg_start, num_drops = self.read_message(x)
            if m is not None:
                self.process_streamed_ulog_data(m, first_msg_start, num_drops)

                # status output
                if self.logging_started:
                    self.measured_data += len(m)
                    measure_time_cur = timer()
                    dt = measure_time_cur - self.measure_time_start
                    if dt > 1:
                        sys.stdout.write('\rData Rate: {:0.1f} KB/s  Drops: {:} \033[K'.format( self.measured_data / dt / 1024, self.num_dropouts))
                        sys.stdout.flush()
                        self.measure_time_start = measure_time_cur
                        self.measured_data = 0
            #if not self.logging_started and timer()-self.start_time > 4:
            #    raise Exception('Start timed out. Is the logger running in MAVLink mode?')

    def read_message(self, m):
        if m is not None:
            if m.get_type() == 'COMMAND_ACK':
                if m.command == mavutil.mavlink.MAV_CMD_LOGGING_START and not self.got_header_section:
                    if m.result == 0:
                        self.logging_started = True
                        print('Logging started. Waiting for Header...')
                    else:
                        raise Exception('Logging start failed', m.result)
                return None, 0, 0

            is_newer, num_drops = self.check_sequence(m.sequence)
            if m.get_type() == 'LOGGING_DATA_ACKED':
                ackmsg = mavutil.mavlink.MAVLink_logging_ack_message( target_system=m.target_system, target_component=self.target_component, sequence=m.sequence )
                ackmsg.pack(link)
                ros_msg = mavlink.convert_to_rosmsg(ackmsg)
                mavlink_pub.publish(ros_msg)

            if is_newer:
                if num_drops > 0:
                    self.num_dropouts += num_drops

                if m.get_type() == 'LOGGING_DATA':
                    if not self.got_header_section:
                        print('\rHeader received in {:0.2f}s'.format(timer()-self.start_time))
                        self.logging_started = True
                        self.got_header_section = True
                self.last_sequence = m.sequence
                return m.data[:m.length], m.first_message_offset, num_drops
            else:
                print('dup/reordered message '+str(m.sequence))

        return None, 0, 0

    def closeFile(self):
        if os.path.isfile(self.file.name) and os.path.getsize(self.file.name) == 0:
           try:
              self.file.close()
              os.unlink(self.file.name)
           except:
              pass
        else:
           print(self.file.name+"\n")

def sigterm_handler(_signo = "", _stack_frame = ""):
    print("Debug request received")
    mavroslog.closeFile()
    #pdb.set_trace()

if __name__ == '__main__':
    signal.signal(signal.SIGTERM, sigterm_handler)
    signal.signal(signal.SIGINT, sigterm_handler)
    signal.signal(signal.SIGQUIT, sigterm_handler)
    signal.signal(signal.SIGABRT, sigterm_handler)

    rospack = rospkg.RosPack()
    mavros.set_namespace()
    link = mavutil.mavlink.MAVLink('', 1, 1)
    pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
    mavlink_recv = ''
    rospy.init_node('mavlink_logger')
    filename = os.path.dirname(os.path.realpath(__file__))+datetime.datetime.now().strftime("/logs/%Y-%m-%d_%H-%M-%S.ulg")
    mavroslog = MavrosLogStreaming(mavlink, filename)
    sub = rospy.Subscriber('mavlink/from', Mavlink, mavroslog.message_handler)
    mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)

    rospy.on_shutdown(sigterm_handler)
    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
       msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
       msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
       ros_msg = mavlink.convert_to_rosmsg(msg)
       pub.publish(ros_msg)
       r.sleep()
