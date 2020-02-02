#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

"""

"""

import rospy
import rospkg
import sched, time
import logging
import os
import cv2
import math
import subprocess
import re
import traceback
import numpy
import rostopic
import mavros
import csv
import sys

import pdb

from os import listdir
from os.path import isfile, join
from threading import Event
from functools import wraps
from telegram import InlineKeyboardButton, InlineKeyboardMarkup, KeyboardButton, ReplyKeyboardMarkup, ChatAction
from telegram.ext import Updater, CommandHandler, CallbackQueryHandler, MessageHandler, Filters
from telegram.ext.dispatcher import run_async
from cv_bridge import CvBridge, CvBridgeError
from pymavlink import mavutil
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, Imu, Range
from mavros_msgs.msg import State, OpticalFlowRad, Mavlink, ParamValue
from mavros_msgs.srv import ParamPull, ParamPush, ParamGet, ParamSet
from mavros import mavlink, param, command
#from mavros.utils import *
from rostopic import get_topic_class, ROSTopicHz
from clever import srv

class ROSTopicHzLocal(ROSTopicHz):
    def print_hz(self):
        """
        print the average publishing rate to screen
        """
        if not self.times:
            return
        elif self.msg_tn == self.last_printed_tn:
            print("no new messages")
            return
        with self.lock:
            n = len(self.times)
            #rate = (n - 1) / (rospy.get_time() - self.msg_t0)
            mean = sum(self.times) / n
            rate = 1./mean if mean > 0. else 0

            #std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.times) /n)

            # min and max
            max_delta = max(self.times)
            min_delta = min(self.times)

            self.last_printed_tn = self.msg_tn

        return("rate: %.3f min: %.3fs max: %.3fs std dev: %.5fs window: %s\n"%(rate, min_delta, max_delta, std_dev, n+1))

def _rostopic_hz(topic, window_size=-1, filter_expr=None):
    msg_class, real_topic, _ = get_topic_class(topic, blocking=True)
    #rospy.init_node(NAME, anonymous=True)
    rt = ROSTopicHzLocal(window_size, filter_expr=filter_expr)
    if filter_expr is not None:
        sub = rospy.Subscriber(real_topic, msg_class, rt.callback_hz)
    else:
        sub = rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback_hz)
    x=0
    result = ''
    while x < 3:
        time.sleep(1.0)
        result = result+rt.print_hz()
        x+=1
    return result

def send_typing_action(func):
    """Sends typing action while processing func command."""

    @wraps(func)
    def command_func(*args, **kwargs):
        bot, update = args
        bot.send_chat_action(chat_id=update.message.chat_id, action=ChatAction.TYPING)
        func(bot, update, **kwargs)

    return command_func

def send_action(action):
    """Sends `action` while processing func command."""

    def decorator(func):
        @wraps(func)
        def command_func(update, context, *args, **kwargs):
            context.bot.send_chat_action(chat_id=update.effective_message.chat_id, action=action)
            return func(update, context,  *args, **kwargs)
        return command_func
    
    return decorator

def get_topics(ttyp = 'sensor_msgs/Image'):
    topics_and_types = rospy.get_published_topics()
    topics = []
    for top, typ in topics_and_types:
        if typ == ttyp:
            topics.append(top)
    return topics

def get_image(image_topic=None):
    rospy.loginfo("Getting image...")
    if image_topic is None:
        image_topic = "/main_camera/image_raw"
    image_msg = rospy.wait_for_message(image_topic, Image, 3)
    rospy.loginfo("Got image!")

    cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    img_file_path = "/tmp/telegram_last_image.jpg"
    cv2.imwrite(img_file_path, cv2_img)
    rospy.loginfo("Saved to: " + img_file_path)
    return img_file_path

def get_image_file(bot, update, image_file=None):
    global gupdate

    image_topics = get_topics()
    distance_topics = get_topics("sensor_msgs/Range")
    query = update.callback_query
    bot.send_photo(chat_id=query.message.chat_id, photo=open(image_file, 'rb'), caption="Image "+query.data)
    
    return True

def get_data(topic=None):
    rospy.loginfo("Getting topic...")
    msg = rospy.wait_for_message(topic, Range, 3)
    rospy.loginfo("Got data! %s" % msg.range)
    return msg

def get_data_service(service=None):
    rospy.loginfo("Getting service...")
    try:
       rospy.wait_for_service('get_telemetry')
       telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
       result = telemetry(frame_id="")
       print result
       return result
    except rospy.ServiceException, e:
       print(sys.exc_info())
       #pdb.set_trace()
       pass

def mavlink_message_handler(msg):
    global gupdate
    global mavlink_recv
    global ht_time
    global logauto

    if ht_time+4 < time.time():
        mavlink_pub.publish(ros_msg)
        ht_time = time.time()



    if msg.msgid == 126 or msg.msgid == 120 or msg.msgid == 253 :
        mav_bytes_msg = mavlink.convert_to_bytes(msg)
        mav_msg = link.decode(mav_bytes_msg)
        if hasattr(mav_msg, 'data'):
            mavlink_recv += ''.join(chr(x) for x in mav_msg.data[:mav_msg.count])
            if 'nsh>' in mavlink_recv:
               mvr = mavlink_recv.replace("\x1b[K", "").split("\n")
               if "nsh> " in mvr:
                  mvr.remove("nsh> ")
               mavlink_recv = ''
               print mvr
               if hasattr(gupdate, 'message') and hasattr(gupdate, 'message'):
                  gupdate.message.reply_text(str.join("\n", mvr))
                  recv_event.set()

        if hasattr(mav_msg, 'text') and hasattr(gupdate, 'message'):
            if logauto != None and msg.msgid == 253 and  'ARMED' == mav_msg.text[:5]:
               startLog("", gupdate)

            if logauto != None and msg.msgid == 253 and  'DISARMED' == mav_msg.text[:8]:
               stopLog("", gupdate)

            gupdate.message.reply_text(mav_msg.text)
            print mav_msg
            #print ("%s %s" % (msg.msgid,mavlink_recv))

def mavlink_exec(cmd, timeout=3.0):
    global mavlink_recv
    mavlink_recv = ''
    recv_event.clear()
    if not cmd.endswith('\n'):
        cmd += '\n'
    msg = mavutil.mavlink.MAVLink_serial_control_message(
        device=mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
        flags=mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI,
        timeout=3,
        baudrate=0,
        count=len(cmd),
        data=map(ord, cmd.ljust(70, '\0')))
    msg.pack(link)
    ros_msg = mavlink.convert_to_rosmsg(msg)
    mavlink_pub.publish(ros_msg)
    recv_event.wait(timeout)
    return mavlink_recv

def button(bot, update):
    global gupdate

    image_topics = get_topics()
    distance_topics = get_topics("sensor_msgs/Range")
    query = update.callback_query
    print ("Button: %s" % query.data)
    try:
       if query.data in image_topics:
          bot.editMessageText(text="Image of topic: %s" % query.data, chat_id=query.message.chat_id, message_id=query.message.message_id)
          i = 0
          while (i < 11 and query.data != '/aruco_map/image') or (query.data == '/aruco_map/image' and i < 1) :
             img_file_path = get_image(image_topic=query.data)
             #bot.editMessageText(text="Uploading captured image of topic: %s" % query.data, chat_id=query.message.chat_id, message_id=query.message.message_id)
             bot.send_photo(chat_id=query.message.chat_id, photo=open(img_file_path, 'rb'), caption="Topic "+query.data)
             time.sleep(1)
             i = i + 1
       elif query.data in distance_topics:
          getDistanceData(bot,update, query.data)
       elif query.data == '..':
           start(bot,update)
    except:
       print(sys.exc_info())
       #pdb.set_trace()
       pass

def getDistanceData(bot, update, topic):
    query = update.callback_query
    logger(bot, update)

    i = 1
    min = 0
    max = 0
    sum = 0
    while i < 21:
       range = get_data(topic).range
       if range > max:
          max = range
       if range < min or min == 0:
          min = range
       sum = range + sum
       print "Range %s %s" % (i, range)
       update.message.reply_text(text="#%s range: %s " %(i, format(range, "^-8.4f")))
       #bot.editMessageText(text="#%s range: %s " %(i, format(range, "^-8.4f")), chat_id=query.message.chat_id, message_id=query.message.message_id)
       time.sleep(0.5)
       i = i + 1
    update.message.reply_text(text="Finished min/max/avg %s/%s/%s" %(format(min,"^-8.4f"),format(max,"^-8.4f"),format(sum/20, "^-8.4f")))
    #bot.editMessageText(text="Finished min/max/avg %s/%s/%s" %(format(min,"^-8.4f"),format(max,"^-8.4f"),format(sum/20, "^-8.4f")), chat_id=query.message.chat_id, message_id=query.message.message_id)

@send_typing_action
def mavlinkCmd(bot, update):
    if str(update.message.chat.id) != chatid:
       #pdb.set_trace()
       update.message.reply_text(text="Incorrect chatid, please update chatid to %s in token.yaml, and restart node " %update.message.chat.id )
       return

    logger(bot, update)
    try:
       ignoreCmd = { ">": "", "<":"", "..":"", ".":"" }
       cmds = {
              "arm":"commander arm", "disarm":"commander disarm", "com takeoff":"commander takeoff", "com land":"commander land", "com check":"commander check", "com status":"commander status",
              "calibrate level":"commander calibrate level", "calibrate gyro":"commander calibrate gyro", "calibrate accel": "commander calibrate accel", "calibrate mag":"commander calibrate mag"
           }

       rospack = rospkg.RosPack()
       #pathSystem = rospack.get_path(rospy.get_name()[1:])+"/../../lib/"+rospy.get_name()[1:]+"/scripts/files/system"
       pathSystem = rospack.get_path(rospy.get_name()[1:])+"/scripts/files/system"
       filesSystem = [f for f in listdir(pathSystem) if isfile(join(pathSystem, f))]
       #pathNavigate = rospack.get_path(rospy.get_name()[1:])+"/../../lib/"+rospy.get_name()[1:]+"/scripts/files/navigate"
       pathNavigate = rospack.get_path(rospy.get_name()[1:])+"/scripts/files/navigate"
       filesNavigate = [f for f in listdir(pathNavigate) if isfile(join(pathNavigate, f))]

       print filesSystem
       print filesNavigate
       print update.message.text

       if update.message.text == 'system':
         systemSub(bot, update)
       elif update.message.text == 'commands':
          commandsSub(bot, update)
       elif update.message.text == 'calibrate':
          calibrateSub(bot, update)
       elif update.message.text == 'profiles':
          profilesSub(bot, update)
       elif update.message.text == 'distance':
          distanceSub(bot, update)
       elif update.message.text == 'images':
          testImagesSub(bot, update)
       elif update.message.text == 'preflight':
          preflightSub(bot, update)
       elif update.message.text == 'navigate':
          testFlySub(bot, update)
       elif update.message.text == 'selfcheck':
          selfCheck(bot, update)
       elif update.message.text == 'get telemetry':
          testTelemetry(bot)
       elif update.message.text == 'get topic hz':
          testTopicHZ(bot)
       elif 'distance' in update.message.text:
          getDistanceData(bot, update, update.message.text.replace("distance ", ""))
       elif 'upload profile' in update.message.text:
          profileid = update.message.text.replace("upload profile ","")
          loadProfile(bot, update, "profile"+profileid)
       elif 'save profile' in update.message.text:
          profileid = update.message.text.replace("save profile ","")
          saveProfile(bot, update, "profile"+profileid)
       elif 'diff profile' in update.message.text:
          profileid = update.message.text.replace("diff profile ","")
          diffProfile(bot, update, "profile"+profileid)
       elif update.message.text == 'back' or update.message.text == '..':
          start(bot,update)
       elif update.message.text[:1] == '#':
          """ Execute simple shell commands """
          result = os.popen("%s"%(update.message.text[1:])).read()
          print result
          update.message.reply_text(text="%s\n%s" % (result[0:3950], "..." if len(result)>3950 else ""))
       elif update.message.text != "" and update.message.text in cmds:
          mavlink_exec(cmds[update.message.text])
       elif update.message.text != "" and update.message.text in filesSystem:
          result = os.popen("bash %s/%s"%(pathSystem,update.message.text)).read()
          print result
          update.message.reply_text(text="%s\n%s" % (result[0:3950], "..." if len(result)>3950 else ""))
       elif update.message.text != "" and update.message.text in filesNavigate:
          result = os.popen("bash %s/%s"%(pathNavigate,update.message.text)).read()
          print result
          update.message.reply_text(text="%s\n%s" % (result[0:3950], "..." if len(result)>3950 else ""))
       elif update.message.text == "debug_bot":
          pdb.set_trace()
       elif update.message.text != "" and update.message.text not in ignoreCmd:
          mavlink_exec(update.message.text)
    except:
       print(sys.exc_info())
       update.message.reply_text(text=sys.exc_info())
       #pdb.set_trace()
       pass

def getTopicsMenu(cols = 1, type = "sensor_msgs/Range", prefix = "distance"):
    topics = get_topics(type)
    root = []
    data = []
    for topicname in topics:
        data.append({'text': "%s %s"%(prefix, topicname)})

        if len(data) == cols and len(root) < 3:
            root.extend([data])
            data = []

    if len(data) > 0:
        root.extend([data])

    if len(root) < 3:
        data = []
        for x in range(cols):
           data.append({'text': " "})
        for r in range(3-len(root)):
           root.extend([data])

    return root

def getFilesMenu(cols = 1, prefix = "system"):
    rospack = rospkg.RosPack()
    #path = rospack.get_path(rospy.get_name()[1:])+"/../../lib/"+rospy.get_name()[1:]+"/scripts/files/"+prefix+"/"
    path = rospack.get_path(rospy.get_name()[1:])+"/scripts/files/"+prefix+"/"
    files = [f for f in listdir(path) if isfile(join(path, f))]
    print files
    root = []
    data = []
    for topicname in files:
        #topicname = topicname.replace(prefix+".", "")
        data.append({'text': "%s"%(topicname)})

        if len(data) == cols and len(root) < 3:
            root.extend([data])
            data = []

    if len(data) > 0:
        for x in range(cols-len(data)):
           data.append({'text': " "})
        root.extend([data])

    if len(root) < 3:
        data = []
        for x in range(cols):
           data.append({'text': " "})
        for r in range(3-len(root)):
           root.extend([data])

    return root

def menu(type = "main"):
    reboot = KeyboardButton("reboot")
    disarm = KeyboardButton("disarm")
    arm = KeyboardButton("arm")
    prev = KeyboardButton("<")
    next = KeyboardButton(">")
    back = KeyboardButton("..")
    empty = KeyboardButton(" ")

    commandsSub = KeyboardButton("commands")
    getImage = KeyboardButton("images")
    getDistance = KeyboardButton("distance")
    selfCheck = KeyboardButton("selfcheck")
    preflightSub = KeyboardButton("preflight")
    systemSub = KeyboardButton("system")
    calibrateSub = KeyboardButton("calibrate")
    testflySub = KeyboardButton("navigate")
    profilesSub = KeyboardButton("profiles")
    getTelemetry = KeyboardButton("get telemetry")
    getTopicHZ = KeyboardButton("get topic hz")

    takeoff = KeyboardButton("com takeoff")
    land = KeyboardButton("com land")
    status = KeyboardButton("com status")
    ekf2 = KeyboardButton("ekf2 status")
    sensors = KeyboardButton("sensors status")
    check = KeyboardButton("com check")
    px4io = KeyboardButton("px4io status")
    mavlink = KeyboardButton("mavlink status")
    fmu = KeyboardButton("fmu status")

    level = KeyboardButton("calibrate level")
    gyro = KeyboardButton("calibrate gyro")
    accel = KeyboardButton("calibrate accel")
    mag = KeyboardButton("calibrate mag")

    loadProfile0 = KeyboardButton("upload profile 0")
    loadProfile1 = KeyboardButton("upload profile 1")
    loadProfile2 = KeyboardButton("upload profile 2")
    saveProfile0 = KeyboardButton("save profile 0")
    saveProfile1 = KeyboardButton("save profile 1")
    saveProfile2 = KeyboardButton("save profile 2")
    diffProfile0 = KeyboardButton("diff profile 0")
    diffProfile1 = KeyboardButton("diff profile 1")
    diffProfile2 = KeyboardButton("diff profile 2")

    if type == 'main':
       keyboard = [[ empty, reboot, disarm, arm, empty ], [commandsSub, systemSub, calibrateSub], [getDistance, getImage, profilesSub], [preflightSub, selfCheck, testflySub] ]
       return ReplyKeyboardMarkup(keyboard)

    if type == 'commands':
       #keyboard = [[ back, reboot, disarm, arm, back ], [takeoff, land, status], [ekf2, sensors, check], [px4io, mavlink, fmu] ]
       keyboard = [[ back, reboot, disarm, arm, back ], [takeoff, land, empty], [empty, empty, empty], [empty, empty, empty] ]
       return ReplyKeyboardMarkup(keyboard)

    if type == 'calibrate':
       keyboard = [[ back, reboot, disarm, arm, back ], [level, gyro], [accel, mag], [empty, empty, empty]]
       return ReplyKeyboardMarkup(keyboard)

    if type == 'profiles':
       keyboard = [[ back, reboot, disarm, arm, back ], [loadProfile0, loadProfile1, loadProfile2], [diffProfile0, diffProfile1, diffProfile2], [saveProfile0,saveProfile1,saveProfile2] ]
       return ReplyKeyboardMarkup(keyboard)

    if type == 'preflight':
       keyboard = [[ back, reboot, disarm, arm, back ], [getTelemetry, getTopicHZ, status], [ekf2, sensors, check], [px4io, mavlink, fmu] ]
       return ReplyKeyboardMarkup(keyboard)

    if type == 'distance':
       topics = getTopicsMenu()
       keyboard = [[ back, reboot, disarm, arm, back ]]
       keyboard.extend(topics)
       return ReplyKeyboardMarkup(keyboard)

    if type == 'system':
       topics = getFilesMenu(3,"system")
       keyboard = [[ back, reboot, disarm, arm, back ]]
       keyboard.extend(topics)
       return ReplyKeyboardMarkup(keyboard)

    if type == 'navigate':
       topics = getFilesMenu(3,"navigate")
       keyboard = [[ back, reboot, disarm, arm, back ]]
       keyboard.extend(topics)
       return ReplyKeyboardMarkup(keyboard)

def start(bot, update):
    logger(bot, update)
    update.message.reply_text('Main menu', reply_markup=menu("main"))

def commandsSub(bot, update):
    logger(bot, update)
    update.message.reply_text('Command options', reply_markup=menu( "commands"))

def calibrateSub(bot, update):
    logger(bot, update)
    update.message.reply_text('Calibrate options', reply_markup=menu( "calibrate"))

def systemSub(bot, update):
    logger(bot, update)
    update.message.reply_text('System options', reply_markup=menu( "system"))

def profilesSub(bot, update):
    logger(bot, update)
    update.message.reply_text('Profiles options', reply_markup=menu( "profiles"))

def imagesSub(bot, update):
    logger(bot, update)
    update.message.reply_text('Images topics', reply_markup=menu( "images"))

def preflightSub(bot, update):
    logger(bot, update)
    update.message.reply_text('Prefligt checks', reply_markup=menu( "preflight"))

def testFlySub(bot, update):
    logger(bot, update)
    update.message.reply_text('Flight Scripts', reply_markup=menu( "navigate"))

def distanceSub(bot, update):
    logger(bot, update)
    #testDistance(bot)
    update.message.reply_text('Range topics', reply_markup=menu( "distance"))

def selfCheck(bot, update):
    logger(bot, update)
    result = os.popen("rosrun clever selfcheck.py 2>&1").read()
    lines = result.splitlines()
    try:
       for x in lines:
          l=x.split(": ")
          if "WARN" in l[0]:
             update.message.reply_text("<b>"+x.replace(l[0]+": ","")+"</b>", parse_mode='HTML')
          else:
             update.message.reply_text(""+x.replace(l[0]+": ","")+"", parse_mode='HTML')
    except:
       pass

def logger(bot, update):
    global gupdate, last
    gupdate = update
    last = time.time()

def testImagesSub(bot, update):
    image_topics = get_topics()

    keyboard = []
    for topicname in image_topics:
        keyboard.append([InlineKeyboardButton(
            topicname, callback_data=topicname)])

    reply_markup = InlineKeyboardMarkup(keyboard)
    update.message.reply_text('Topic Images', reply_markup=reply_markup)

def testDistance(bot, update):
    distance_topics = get_topics("sensor_msgs/Range")
    keyboard = []
    for topicname in distance_topics:
        keyboard.append([InlineKeyboardButton(
            topicname, callback_data=topicname)])
    
    reply_markup = InlineKeyboardMarkup(keyboard)
    update.message.reply_text('Topic Distance', reply_markup=reply_markup)

def testTelemetry(bot):
    global gupdate
    update = gupdate
    rospy.loginfo("Getting service...")
    rospy.wait_for_service('get_telemetry', 3)
    try:
       telem = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
       i = 0
       while i < 11:
          t = telem(frame_id="")
          gupdate.message.reply_text("%s x:y:z %s,%s,%s vx:vy:vz %s,%s,%s alt:%s p:r:y %s,%s,%s " % (t.mode, format(t.x, "^-8.2f"), format(t.y, "^-8.2f"), format(t.z, "^-8.2f"), format(t.vx,"^-8.2f"), format(t.vy,"^-8.2f"), format(t.vz,"^-8.2f"), t.alt, format(t.pitch,"^-8.2f"), format(t.roll,"^-8.2f"), format(t.yaw,"^-8.2f")))
          time.sleep(1)
          i = i + 1

       print result
       return result
    except rospy.ServiceException, e:
       rospy.loginfo("Service call failed: %s"%e)

def testNavigate(bot):
    global gupdate
    update = gupdate
    rospy.loginfo("Getting service...")
    rospy.wait_for_service('navigate', 3)
    try:
       navi = rospy.ServiceProxy("navigate", srv.Navigate)
       #n = navi(x=0, y=0, z=1, frame_id='body')
       gupdate.message.reply_text("/navigate %s  " % (navi(x=0, y=0, z=1, frame_id='body')))
       print result
       return result
    except rospy.ServiceException, e:
       rospy.loginfo("Service call failed: %s"%e)

def testTopicHZ(bot):
    global gupdate
    update = gupdate
    rospy.loginfo("Main Camera ...")
    result = _rostopic_hz("/main_camera/image_raw", window_size=-1, filter_expr=None)
    #result = _rostopic_hz("/optical_flow/debug", window_size=-1, filter_expr=None)
    print result
    gupdate.message.reply_text("hz main_camera\n%s " % (result))
    return result

def help(bot, update):
    update.message.reply_text("Use /start to test this bot.")

def error(bot, update, error):
    logging.warning('Update "%s" caused error "%s"' % (update, error))

@send_typing_action
def listParams(bot, update):
    found = False
    mavros.param.param_get_all(force_pull=True)
    cmd = update._effective_message.text
    print cmd
    logger(bot, update)
    params = rospy.get_param(mavros.get_topic('param'))

    search = cmd[5:]
    print "Launch search %s " % search
    #update.message.reply_text(" Launch search %s " %search)
    if search == "":
       return False

    for p in params:
       if p.find(search.strip().upper()) != -1:
           found = True
           print "Found %s %s => %s" % (search, p, params.get(p))
           update.message.reply_text(" %s => %s " % (p, params.get(p)))

    if found == False:
       update.message.reply_text(" Nothing found for %s " % search)


@send_typing_action
def setParam(bot, update):
    cmd = update._effective_message.text
    print cmd
    logger(bot, update)
    params = rospy.get_param(mavros.get_topic('param'))

    print "Set %s => new value %s, old value %s " % (cmd.split()[1], cmd.split()[3], params.get(cmd.split()[1]))
    if cmd.split()[1] == "" or cmd.split()[3] == "":
       update.message.reply_text("Missing data  %s => %s " % (cmd.split()[1], cmd.split()[3]))
       return

    if cmd.split()[1] != "" and cmd.split()[3] != "":
       print "Result:  %s" % mavros.param.param_set(cmd.split()[1],int(cmd.split()[3]))
       update.message.reply_text("Set %s => new value %s, old value %s " % (cmd.split()[1], cmd.split()[3], params.get(cmd.split()[1])))

#@send_typing_action
def saveProfile(bot, update, profile = "profile0"):
    cmd = update._effective_message.text
    print cmd
    logger(bot, update)

    param_received, param_list = mavros.param.param_get_all(True)
    update.message.reply_text("Parameters received  %s " % param_received)
    param_file = mavros.param.QGroundControlParam(True)
    file = open(profile, "w")
    param_file.write(file, param_list)
    update.message.reply_text("Parameters profile %s saved  " % profile)

#@send_typing_action
def loadProfile(bot, update, profile = "profile0"):
    cmd = update._effective_message.text
    print cmd
    logger(bot, update)

    param_received, param_list = mavros.param.param_get_all(True)
    param_file = mavros.param.QGroundControlParam(True)

    if os.path.isfile(profile): 
       update.message.reply_text("Please wait, uploading in progress")
    else:
       update.message.reply_text("Missing profile data for %s " %profile)

    file = open(profile, "r")
    param_transfered = mavros.param.param_set_list(param_file.read(file))
    update.message.reply_text("Parameters received  %s " % param_received)
    update.message.reply_text("Parameters profile %s uploaded " % profile)

#@send_typing_action
def diffProfile(bot, update, profile = "profile0"):
    cmd = update._effective_message.text
    print cmd
    logger(bot, update)

    if os.path.isfile(profile): 
       update.message.reply_text("Please wait, operation in progress")
    else:
       update.message.reply_text("Missing profile data for %s " %profile)

    param_received, param_list = mavros.param.param_get_all(True)
    param_hash_file = {}
    param_hash = {}
    for x in param_list:
       param_hash[x.param_id] = x.param_value

    param_file_q = mavros.param.QGroundControlParam(True)
    file = open(profile, "r")
    param_list_file = param_file_q.read(file)
    for p in param_list_file:
       if p.param_id in param_hash and param_hash[p.param_id] != p.param_value:
           param_hash_file[p.param_id] = p.param_value
       elif p.param_id in param_hash:
           del param_hash[p.param_id]
       else:
           param_hash_file[p.param_id] = p.param_value

    if len(param_hash_file) == 0 and len(param_hash) == 0:
        print "No changes %s " % profile
        update.message.reply_text("No changes %s " % profile)

    for z in param_hash_file:
       if z in param_hash and z in param_hash_file:
           print "Changes %s %s => %s" % (z, param_hash_file[z], param_hash[z])
           update.message.reply_text("Changes %s %s => %s" % (z, param_hash_file[z], param_hash[z]))

    for z in param_hash_file:
       if z not in param_hash:
           print "Removed %s = %s" % (z, param_hash_file[z])
           update.message.reply_text("Removed %s = %s" % (z, param_hash_file[z]))

    for z in param_hash:
       if z not in param_hash_file:
           print "Added %s = %s" % (z, param_hash[z])
           update.message.reply_text("Added %s = %s" % (z, param_hash[z]))

    print param_hash_file
    print param_hash

@send_typing_action
def diffProfile0(bot, update, profile = "profile0"):
    diffProfile(bot, update, profile)

@send_typing_action
def saveProfile0(bot, update, profile = "profile0"):
    saveProfile(bot, update, profile)

@send_typing_action
def loadProfile0(bot, update, profile = "profile0"):
    loadProfile(bot, update, profile)

#@send_typing_action
def startLog(bot, update):
    update.message.reply_text("Mavlink logger starting")
    rospack = rospkg.RosPack()
    cmd = rospack.get_path(rospy.get_name()[1:])+"/scripts/run_screen.sh \"rosrun clever_telegram mavlink_logger.py\" ";
    print(cmd)
    #result = subprocess.call("'", shell=True)
    result = os.popen(cmd).read()
    print(result)
    print(command.long( broadcast=0, command=2511, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0))
    time.sleep(3)
    print(command.long( broadcast=0, command=2510, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0))

#@send_typing_action
def stopLog(bot, update):
    update.message.reply_text("Mavlink logger stopping")
    print(command.long( broadcast=0, command=2511, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0))
    time.sleep(2)
    result = os.popen("rosnode kill mavlink_logger 2>&1").read()

@run_async
def heartbeat():
    print "Heartbeat: %s" % time.time()
    mavlink_pub.publish(ros_msg)

bridge = CvBridge()
gupdate = False
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', level=logging.INFO)
mavros.set_namespace()
recv_event = Event()
link = mavutil.mavlink.MAVLink('', 255, 1)
mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
mavlink_recv = ''
rospy.init_node('clever_telegram')
logauto = rospy.get_param('/logger/autostart', None)
token = rospy.get_param('/telegram/token', None)
chatid = rospy.get_param('/telegram/chatid', None)
mavlink_sub = rospy.Subscriber('mavlink/from', Mavlink, mavlink_message_handler)

msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
ros_msg = mavlink.convert_to_rosmsg(msg)
mavlink_pub.publish(ros_msg)

ht_time = time.time()

updater = Updater(token)
updater.dispatcher.add_handler(MessageHandler(Filters.text, mavlinkCmd))
updater.dispatcher.add_handler(CallbackQueryHandler(button))
updater.dispatcher.add_handler(CommandHandler('selfcheck', selfCheck))
updater.dispatcher.add_handler(CommandHandler('list', listParams))
updater.dispatcher.add_handler(CommandHandler('set', setParam))
updater.dispatcher.add_handler(CommandHandler('save', saveProfile0))
updater.dispatcher.add_handler(CommandHandler('upload', loadProfile0))
updater.dispatcher.add_handler(CommandHandler('diff', diffProfile0))
updater.dispatcher.add_handler(CommandHandler('stopLog', stopLog))
updater.dispatcher.add_handler(CommandHandler('startLog', startLog))
updater.dispatcher.add_handler(CommandHandler('start', start))
updater.dispatcher.add_handler(CommandHandler('help', help))
updater.dispatcher.add_error_handler(error)
updater.start_polling()
updater.idle()

