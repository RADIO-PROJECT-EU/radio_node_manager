#!/usr/bin/env python
import os
import math, time
import rospy
import subprocess, shlex
from std_msgs.msg import Int32
from kobuki_msgs.msg import Sound
#from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
#from tf.transformations import quaternion_from_euler, euler_from_quaternion

pub_start = None
sound_pub = None

def init():
    global sound_pub, pub_start
    rospy.init_node('radio_node_manager')
    rospy.Subscriber('radio_node_manager_main_controller/instruction', Int32, instructionCallback)
    sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size=1)

    #Run here all the initial nodes
    time.sleep(15)
    initial_pose()
    time.sleep(10)
    #pub_start = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    #start_point = PoseWithCovarianceStamped()
    #start point position x
    #start_point.pose.pose.position.x = -6.329
    #start point position y     
    #start_point.pose.pose.position.y = 10.087
    #start_point.header.stamp = rospy.Time.now()
    #start_point.pose.pose.orientation.z = -2.122
    #quat = quaternion_from_euler(0.0, 0.0, -2.122) # roll, pitch, yaw
    #start_point.pose.pose.orientation = Quaternion(*quat.tolist())
    #start_point.pose.pose.orientation.w = 0
    #start_point.header.frame_id = 'map'
    #rospy.sleep(1)
    #pub_start.publish(start_point)
    sound_msg = Sound()
    sound_msg.value = 6
    sound_pub.publish(sound_msg)

    while not rospy.is_shutdown():  
        rospy.spin()
    #we reach this point only after Ctrl-C
    #HPR(False)
    #rosVisual(False)
    #motionAnalysisHuman(False)
    #motionAnalysisObject(False)

def instructionCallback(msg):
    if msg.data == 0:
        HPR(True)
    elif msg.data == 1:
        motionAnalysisHuman(True)
    elif msg.data == 2:
        # TODO change configuration
        motionAnalysisObject(True)
    elif msg.data == 22:
        # TODO change configuration
        motionAnalysisObject(True)
    elif msg.data == 3:
        rosVisual(True)
    elif msg.data == 4:
        command = "roslaunch kobuki_auto_docking minimal.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        command = "roslaunch kobuki_auto_docking activate.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)
    elif msg.data == 5:
        initial_pose()

def HPR(start):
    global sound_pub
    if start:
        print 'Starting HPR'
        command = "roslaunch human_pattern_recognition hpr.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)

def motionAnalysisHuman(start):
    global sound_pub
    if start:
        print 'Starting motion_analysis human'
        command = "roslaunch motion_analysis human_event_detection.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        time.sleep(10)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)

def motionAnalysisObject(start):
    global sound_pub
    if start:
        print 'Starting motion_analysis object'
        command = "roslaunch motion_analysis object_event_detection.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        time.sleep(10)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)

def rosVisual(start):
    global sound_pub
    if start:
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

