#!/usr/bin/env python
import os
import math, time
import rospy
import subprocess, shlex
from std_msgs.msg import Int32
from kobuki_msgs.msg import Sound

pub_start = None
sound_pub = None
ost_pub = None

def init():
    global sound_pub, pub_start, ost_pub
    rospy.init_node('radio_node_manager')
    rospy.Subscriber('radio_node_manager_main_controller/instruction', Int32, instructionCallback)
    sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size=1)
    ost_pub = rospy.Publisher("motion_analysis/object_state", Int32, queue_size=1)

    #Run here all the initial nodes
    time.sleep(50)
    initial_pose()
    time.sleep(10)
    sound_msg = Sound()
    sound_msg.value = 6
    sound_pub.publish(sound_msg)

    while not rospy.is_shutdown():  
        rospy.spin()

def instructionCallback(msg):
    if msg.data == 0:
        HPR()
    elif msg.data == 1:
        motionAnalysisHuman()
    elif msg.data == 2:
        motionAnalysisObject(1)
    elif msg.data == 22:
        motionAnalysisObject(2)
    elif msg.data == 3:
        rosVisual()
    elif msg.data == 4:
        command = "roslaunch kobuki_auto_docking activate.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)
    elif msg.data == 5:
        initial_pose()
        time.sleep(2)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)

def HPR():
    global sound_pub
    print 'Starting HPR'
    command = "roslaunch human_pattern_recognition hpr.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)

def motionAnalysisHuman():
    global sound_pub, ost_pub
    print 'Starting motion_analysis human'
    command = "roslaunch motion_analysis human_event_detection.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(10)
    ost_pub.publish(2)
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)

def motionAnalysisObject(mode):
    global sound_pub
    print 'Starting motion_analysis object'
    command = "roslaunch motion_analysis object_event_detection.launch"
    if mode == 2:
        command = "roslaunch motion_analysis object_event_detection2.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(10)
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)

def rosVisual():
    global sound_pub
    print 'Starting ros_visual'
    command = "roslaunch ros_visual ros_visual_classifier.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)

def initial_pose():
    command = "rosservice call /marker_mapping_node/init_pose_from_marker \"id: []\""
    command = shlex.split(command)
    subprocess.Popen(command)

if __name__ == '__main__':
    init() 

