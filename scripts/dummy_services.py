#!/usr/bin/env python

import rospy
from franka_msgs.srv import *
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryResult
import actionlib

def ft_cb(req):
    return SetForceTorqueCollisionBehaviorResponse(False,'not implemented')

def ee_cb(req):
    return SetEEFrameResponse(False,'not implemented')

def load_cb(req):
    return SetLoadResponse(False,'not implemented')

def error_cb(goal):
    result = ErrorRecoveryResult
    global recover_ac
    recover_ac.set_succeeded(result)
    return

rospy.init_node('franka_control')
ft_svc = rospy.Service('~set_force_torque_collision_behavior', SetForceTorqueCollisionBehavior, ft_cb)
set_ee_frame_svc = rospy.Service('~set_EE_frame', SetEEFrame, ee_cb)
set_load_svc = rospy.Service('~set_load', SetLoad, load_cb)
recover_ac = actionlib.SimpleActionServer('~error_recovery',ErrorRecoveryAction,execute_cb=error_cb,auto_start=False)
recover_ac.start()
rospy.spin()
