#!/usr/bin/env python
import os
import math, time
import rospy
import subprocess, shlex
from std_msgs.msg import Int32
from kobuki_msgs.msg import Sound
from radio_services.srv import InstructionWithAnswer

pub_start = None
sound_pub = None

def init():
    global sound_pub, pub_start
    rospy.init_node('radio_node_manager_robot')
    rospy.Service('robot_instruction_receiver', InstructionWithAnswer, reactToInstruction)
    sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size=1)

    #Run here all the initial nodes
    time.sleep(50)
    initial_pose()
    time.sleep(10)
    sound_msg = Sound()
    sound_msg.value = 6
    sound_pub.publish(sound_msg)

    while not rospy.is_shutdown():
        rospy.spin()

def startSound():
    global sound_pub
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)

def endSound():
    global sound_pub
    sound_msg = Sound()
    sound_msg.value = 1
    sound_pub.publish(sound_msg)

def errorSound():
    global sound_pub
    sound_msg = Sound()
    sound_msg.value = 4
    sound_pub.publish(sound_msg)

##
## @brief      The callback for the instruction service.
##
## @param      instruction  The instruction
##              Current possible values are:
##              0:  Start HPR
##              10: Stop HPR
##              1:  Start motion analysis in human mode
##              2:  Start motion analysis in pill mode
##              11: Stop motion analysis (any mode)
##              3:  Start ros_visual
##              13: Stop ros_visual
##              4:  Start auto docking
##              5:  Reset initial pose
##              -1: Play "start" sound
##              -2: Play "end" sound
##              -3: Play "error" sound
##
## @return     { description_of_the_return_value }
##
def reactToInstruction(instruction):
    global sound_pub
    if instruction.command == 0:
        HPR(True)
        return True
    elif instruction.command == 10:
        HPR(False)
        return True
    elif instruction.command == 1:
        motionAnalysis(True, 10)
        return True
    elif instruction.command == 2:
        motionAnalysis(True, 12)
        return True
    elif instruction.command == 11:
        motionAnalysis(False)
        return True
    elif instruction.command == 3:
        rosVisual(True)
        return True
    elif instruction.command == 13:
        rosVisual(False)
        return True
    elif instruction.command == 4:
        command = "roslaunch kobuki_auto_docking activate.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)
        return True
    elif instruction.command == 5:
        initial_pose()
        time.sleep(2)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)
        return True
    elif instruction.command == -1:
        startSound()
        return True
    elif instruction.command == -2:
        endSound()
        return True
    elif instruction.command == -3:
        errorSound()
        return True
    return False

##
## @brief      This function handles the HPR service
##
## @param      start  True to start and false to stop the node
##
def HPR(start):
    test = InstructionWithAnswer()
    test.answer = not start
    command = 0
    if start:
        command = 1
        rospy.wait_for_service('/human_pattern_recognition/laser_wall_extraction/node_state_service', timeout = 10)
        try:
            service = rospy.ServiceProxy('/human_pattern_recognition/laser_wall_extraction/node_state_service', InstructionWithAnswer)
            test = service(command)
            print test.answer
        except rospy.ServiceException, e:
            print e
    else:
        command = 0
        rospy.wait_for_service('/human_pattern_recognition/laser_wall_extraction/node_state_service', timeout = 10)
        try:
            service = rospy.ServiceProxy('/human_pattern_recognition/laser_wall_extraction/node_state_service', InstructionWithAnswer)
            test = service(command)
        except rospy.ServiceException, e:
            print e
    if start == test.answer:
        if start:
            startSound()
        else:
            endSound()
    else:
        print 'error'
        errorSound()

##
## @brief      This function handles the motion_analysis service
##
## @param      start  True to start and false to stop the node
## @param      mode   Which mode to start (if start == true)
##
def motionAnalysis(start, mode=0):
    test = InstructionWithAnswer()
    test.answer = not start
    command = 0
    if start:
        command = 11
        rospy.wait_for_service('/motion_analysis/node_state_service', timeout = 10)
        try:
            service = rospy.ServiceProxy('/motion_analysis/node_state_service', InstructionWithAnswer)
            test = service(command)
            if test.answer:
                time.sleep(3)
                command = mode
                rospy.wait_for_service('/motion_analysis/node_state_service', timeout = 10)
                try:
                    service = rospy.ServiceProxy('/motion_analysis/node_state_service', InstructionWithAnswer)
                    test = service(command)
                except rospy.ServiceException, e:
                    print e
        except rospy.ServiceException, e:
            print e
    else:
        command = 0
        rospy.wait_for_service('/motion_analysis/node_state_service', timeout = 10)
        try:
            service = rospy.ServiceProxy('/motion_analysis/node_state_service', InstructionWithAnswer)
            test = service(command)
        except rospy.ServiceException, e:
            print e
    if start == test.answer:
        if start:
            startSound()
        else:
            endSound()
    else:
        errorSound()

##
## @brief      This function handles the ros_visual service
##
## @param      start  True to start and false to stop the node
##
def rosVisual(start):
    test = InstructionWithAnswer()
    test.answer = not start
    command = 0
    if start:
        command = 1
        rospy.wait_for_service('/ros_visual/chroma/node_state_service', timeout = 10)
        try:
            service = rospy.ServiceProxy('/ros_visual/chroma/node_state_service', InstructionWithAnswer)
            test = service(command)
        except rospy.ServiceException, e:
            print e
    else:
        command = 0
        rospy.wait_for_service('/ros_visual/chroma/node_state_service', timeout = 10)
        try:
            service = rospy.ServiceProxy('/ros_visual/chroma/node_state_service', InstructionWithAnswer)
            test = service(command)
        except rospy.ServiceException, e:
            print e
    if start == test.answer:
        if start:
            startSound()
        else:
            endSound()
    else:
        errorSound()

def initial_pose():
    command = "rosservice call /marker_mapping_node/init_pose_from_marker \"id: []\""
    command = shlex.split(command)
    subprocess.Popen(command)

if __name__ == '__main__':
    init() 

