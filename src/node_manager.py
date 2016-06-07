#!/usr/bin/env python
import roslib, rospy
import subprocess, shlex
from motion_detection_sensor_msgs.msg import SensorStatusMsg
import math
from actionlib_msgs.msg import GoalStatusArray

motion_analysis_package_name = ''
motion_analysis_launch_filename = ''
running_motion_analysis = False
motion_analysis_process = None
goal_point = []
#first_detect becomes false the first time we see 'ok'
#from the motion detection sensor, to ensure that
#the first 'detect' values are not from the human's
#movements approaching and lying to bed.
first_detect = True
movement_sensor_sub = None
navigating = False


#TODO on launch, set initial navigation pose. We know that we are in
#the docking station, charging.


def init():
    global movement_sensor_sub
    rospy.init_node('radio_node_manager')
    movement_sensor_sub = rospy.Subscriber('motion_detection_sensor_status_publisher/status', SensorStatusMsg, movementStatus)
    rospy.Subscriber('move_base/status', GoalStatusArray, currentRobotPosition)
    while not rospy.is_shutdown():  
        rospy.spin()

'''
0   # The goal has yet to be processed by the action server
1   # The goal is currently being processed by the action server
2   # The goal received a cancel request after it started executing
    # and has since completed its execution (Terminal State)
3   # The goal was achieved successfully by the action server (Terminal State)
4   # The goal was aborted during execution by the action server due
    # to some failure (Terminal State)
5   # The goal was rejected by the action server without being processed,
    # because the goal was unattainable or invalid (Terminal State)
6   # The goal received a cancel request after it started executing
    # and has not yet completed execution
7   # The goal received a cancel request before it started executing,
    # but the action server has not yet confirmed that the goal is canceled
8   # The goal received a cancel request before it started executing
    # and was successfully cancelled (Terminal State)
9   # An action client can determine that a goal is LOST. This should not be
    # sent over the wire by an action server
'''


def currentRobotPosition(current_status_msg):
    global goal_point, goal_reached, navigating
    status =  current_status_msg.status_list[0].status
    if navigating:
        if status == 3:
            navigating = False
            print 'Starting motion_analysis'
            print 'Starting HPR'

        if status > 3:
            print 'Send navigation error to the user'
    else:
        if status == 1:
            print 'Stopping motion analysis'
            print 'Stopping HPR'
            navigating = True


def movementStatus(ssm):
    global running_motion_analysis, motion_analysis_process, first_detect
    global movement_sensor_sub
    cur_st = ssm.status
    if cur_st == 'ok':
        print 'Got an ok!'
        if first_detect:
            first_detect = False
            print 'Waiting for an actual detection now'
        '''
        if running_motion_analysis:
            command = "rosnode kill motion_analysis"
            command = shlex.split(command)
            motion_analysis_process = subprocess.Popen(command)
            running_motion_analysis = 
        '''

    elif not first_detect and cur_st == 'detect':
        print 'Unsubscribing from the movement sensor...'
        movement_sensor_sub.unregister()
        print 'Starting motion_analysis'
        print 'Starting HPR'
        '''
        if not running_motion_analysis:
            command = "roslaunch motion_analysis event_detection.launch"
            command = shlex.split(command)
            motion_analysis_process = subprocess.Popen(command)
            running_motion_analysis = True
        '''


if __name__ == '__main__':
    init() 
