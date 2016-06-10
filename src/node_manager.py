#!/usr/bin/env python
import math, time
import roslib, rospy
import subprocess, shlex
from actionlib_msgs.msg import GoalID
from kobuki_msgs.msg import SensorState
from sensor_msgs.msg import BatteryState
from actionlib_msgs.msg import GoalStatusArray
from motion_detection_sensor_msgs.msg import SensorStatusMsg

motion_analysis_launch_filename = ''
motion_analysis_package_name = ''
running_motion_analysis = False
motion_analysis_process = None
movement_sensor_sub = None
pc_needs_to_charge = False
kobuki_battery_sub = None
kobuki_max_charge = 164 #Validated fully charged battery value
check_batteries = False
nav_status_sub = None
pc_battery_sub = None
navigating = False
charging = False
goal_point = []

#first_detect becomes false the first time we see 'ok'
#from the motion detection sensor, to ensure that
#the first 'detect' values are not from the human's
#movements approaching and lying to bed.
first_detect = True


def init():
    global movement_sensor_sub, nav_status_sub, pub_stop, pc_battery_sub, kobuki_battery_sub
    global check_batteries
    rospy.init_node('radio_node_manager')
    rospy.get_param("~check_batteries", False)
    movement_sensor_sub = rospy.Subscriber('motion_detection_sensor_status_publisher/status', SensorStatusMsg, motionSensorStatus)
    nav_status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, currentNavStatus)
    pub_stop = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    if check_batteries:
        #pc_battery_sub = rospy.Subscriber('placeholder', PlaceHolderMsg, pcBatteryCallback)
        kobuki_battery_sub = rospy.Subscriber('mobile_base/sensors/core', SensorState, kobukiBatteryCallback)

    #TODO ask the main server about what the initial state should be
    #Set initial pose on launch based on the main server's last saved position.
    #Maybe the robot itself could do this too. A save_current_state node would be useful.

    #Run here all the initial nodes
    command = "roslaunch turtlebot_radio_bringup radio_bringup.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(30)
    command = "roslaunch turtlebot_navigation radio_move_base.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(2)
    command = "roslaunch turtlebot_navigation radio_amcl_demo.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(2)
    command = "rosrun map_server map_server /home/turtlebot/smart_room.yaml"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(2)
    #map server also needed. It should be included in one of the above packages with the final map.
    command = "roslaunch turtlebot_teleop logitech.launch"
    command = shlex.split(command)
    subprocess.Popen(command)

    while not rospy.is_shutdown():  
        rospy.spin()
    #we reach this point only after Ctrl-C
    command = "rosnode kill turtlebot_teleop"
    command = shlex.split(command)
    subprocess.Popen(command)
    command = "rosnode kill map_server"
    command = shlex.split(command)
    subprocess.Popen(command)
    command = "rosnode kill move_base"
    command = shlex.split(command)
    subprocess.Popen(command)
    command = "rosnode kill amcl"
    command = shlex.split(command)
    subprocess.Popen(command)
    command = "rosnode kill turtlebot_teleop"
    command = shlex.split(command)
    subprocess.Popen(command)
    command = "rosnode kill turtlebot_radio_bringup"
    command = shlex.split(command)
    subprocess.Popen(command)



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
def currentNavStatus(current_status_msg):
    global goal_point, goal_reached, navigating
    if len(current_status_msg.status_list) > 0:
        status =  current_status_msg.status_list[0].status
        if navigating:
            if status == 3:
                navigating = False
                print 'Starting motion_analysis'
                print 'Starting HPR'

            if status > 3:
                navigating = False
                print 'Send navigation error to the user'
        else:
            if status == 1:
                print 'Stopping motion analysis'
                print 'Stopping HPR'
                navigating = True


def motionSensorStatus(ssm):
    global running_motion_analysis, motion_analysis_process, first_detect
    global movement_sensor_sub
    cur_st = ssm.status
    if cur_st == 'ok':
        print 'Got an ok!'
        if first_detect:
            first_detect = False
            print 'Waiting for an actual detection now'
    elif not first_detect and cur_st == 'detect':
        print 'Unsubscribing from the movement sensor...'
        movement_sensor_sub.unregister()
        print 'Starting motion_analysis'
        print 'Starting HPR'
        if not running_motion_analysis:
            command = "roslaunch motion_analysis event_detection.launch"
            command = shlex.split(command)
            motion_analysis_process = subprocess.Popen(command)
            running_motion_analysis = True

#TODO
#Get the goal from tha radio GUI and then decide 
#if we should forward it to the navigation.
#e.g. if we are still charging and we are below a certain threshold,
#just send a message to the user informing them about our battery state.
def goalHandler(goal_msg):
    print goal_msg
    #pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

#this method only changes the pc_needs_to_charge value. The rest are left for
#the kobukiBatteryCallback method.
def pcBatteryCallback(msg):
    global pc_needs_to_charge
    print msg
    if msg.percentage*100 < 0.05:
        pc_needs_to_charge = True
    else:
        pc_needs_to_charge = False


def kobukiBatteryCallback(msg):
    global kobuki_max_charge, pub_stop, charging, pc_needs_to_charge
    #print msg
    if (msg.battery/kobuki_max_charge*100) < 0.05 or pc_needs_to_charge: #less that 5% battery on kobuki
        if msg.charger == 0:
            charging = False
            print 'I am not charging, and I definitely need to!'
            if navigating:
                navigating = False
                pub_stop.publish(GoalID())
                #TODO
                print 'Stopping motion_analysis'
                print 'Stopping HPR'
                print 'Message the user about my current battery state'
                print 'Now I need to navigate back to my base. Publish such message!'
        else:
            print 'Charging'
            charging = True


if __name__ == '__main__':
    init() 
