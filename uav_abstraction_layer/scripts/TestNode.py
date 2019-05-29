#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------------------------------------------------
# ROS-MAGNA
# ----------------------------------------------------------------------------------------------------------------------
# The MIT License (MIT)

# Copyright (c) 2016 GRVC University of Seville

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
# Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
# OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# ----------------------------------------------------------------------------------------------------------------------


"""
Created on Thur 2 May 2019

@author: josmilrom
"""

import sys, os
import rospy
import std_msgs.msg
import time
import math
import numpy as np
import roslaunch
import shutil
import subprocess
import signal
import rospkg
from uav_abstraction_layer.srv import *
from uav_abstraction_layer.msg import *
from geometry_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import *
import rospkg
from enum import Enum

class TestNode(object):
    def __init__(self):

        self.ID = 1

        rospy.init_node('test_node', anonymous=True)     # Start node

        self.WaypointSetType = {
            "TAKEOFF_POSE" : 0,
            "TAKEOFF_AUX" : 1,
            "PASS" : 2,
            "LOITER_UNLIMITED" : 3,
            "LOITER_TURNS" : 4,
            "LOITER_TIME" : 5,
            "LOITER_HEIGHT" : 6,
            "LAND_POSE" : 7,
            "LAND_AUX" : 8    }

        self.TestFullMission()

        # time.sleep(10)

        # self.TestAirMission()


    def TestFullMission(self):

        new_mission = []

        takeoff_set = WaypointSet()
        takeoff_set.type = self.WaypointSetType["TAKEOFF_POSE"]
        params = [Param_float("minimum_pitch",0.0),Param_float("yaw_angle",0.0)]
        takeoff_set.params = params
        takeoff_pose = PoseStamped()
        takeoff_pose.pose.position = Point(0.0,0.0,40.0)
        takeoff_set.posestamped_list = [takeoff_pose]

        new_mission = [takeoff_set]

        pass_set = WaypointSet()
        pass_set.type = self.WaypointSetType["PASS"]
        params = [Param_float("acceptance_radius",0.0),Param_float("orbit_distance",0.0),Param_float("yaw_angle",0.0)]
        pass_set.params = params
        pass_pose = PoseStamped()
        pass_pose.pose.position = Point(200.0,100.0,70.0)
        pass_set.posestamped_list = [pass_pose]
        pass1_pose = PoseStamped()
        pass1_pose.pose.position = Point(200.0,-100.0,100.0)
        pass_set.posestamped_list.append(pass1_pose)
        pass2_pose = PoseStamped()
        pass2_pose.pose.position = Point(-200.0,100.0,100.0)
        pass_set.posestamped_list.append(pass2_pose)
        pass3_pose = PoseStamped()
        pass3_pose.pose.position = Point(-200.0,-100.0,100.0)
        pass_set.posestamped_list.append(pass3_pose)
        new_mission.append(pass_set)

        # loiter_set = WaypointSet()
        # loiter_set.type = self.WaypointSetType["LOITER_TIME"]
        # params = [Param_float("time",90.0),Param_float("radius",60.0),Param_float("forward_moving",0.0)]
        # loiter_set.params = params
        # loiter_pose = PoseStamped()
        # loiter_pose.pose.position = Point(0.0,0.0,100.0)
        # loiter_set.posestamped_list = [loiter_pose]
        # new_mission.append(loiter_set)

        # land_set = WaypointSet()
        # land_set.type = self.WaypointSetType["LAND_POSE"]
        # params = [Param_float("loit_heading",0.0),Param_float("loit_radius",40.0),Param_float("loit_forward_moving",0.0),
        #          Param_float("precision_mode",0.0),Param_float("abort_alt",0.0),Param_float("yaw_angle",0.0) ]
        # land_set.params = params
        # land_pose = PoseStamped()
        # land_pose.pose.position = Point(0.0,0.0,0.0)
        # land_set.posestamped_list = [land_pose]
        # land1_pose = PoseStamped()
        # land1_pose.pose.position = Point(25.0,-100.0,10.0)
        # land_set.posestamped_list.append(land1_pose)
        # new_mission.append(land_set)

        print(new_mission)

        self.SetMissionServiceCall(new_mission)

    def TestAirMission(self):

        new_mission = []


        pass_set = WaypointSet()
        pass_set.type = self.WaypointSetType["PASS"]
        params = [Param_float("acceptance_radius",0.0),Param_float("orbit_distance",0.0),Param_float("yaw_angle",0.0)]
        pass_set.params = params
        pass_pose = PoseStamped()
        pass_pose.pose.position = Point(400.0,100.0,70.0)
        pass_set.posestamped_list = [pass_pose]
        pass1_pose = PoseStamped()
        pass1_pose.pose.position = Point(200.0,-100.0,100.0)
        pass_set.posestamped_list.append(pass1_pose)
        pass2_pose = PoseStamped()
        pass2_pose.pose.position = Point(-200.0,100.0,100.0)
        pass_set.posestamped_list.append(pass2_pose)
        pass3_pose = PoseStamped()
        pass3_pose.pose.position = Point(-200.0,-100.0,100.0)
        pass_set.posestamped_list.append(pass3_pose)
        new_mission.append(pass_set)

        loiter_set = WaypointSet()
        loiter_set.type = self.WaypointSetType["LOITER_TIME"]
        params = [Param_float("second",90.0),Param_float("radius",60.0),Param_float("forward_moving",0.0)]
        loiter_set.params = params
        loiter_pose = PoseStamped()
        loiter_pose.pose.position = Point(0.0,0.0,100.0)
        loiter_set.posestamped_list = [loiter_pose]
        new_mission.append(loiter_set)

        land_set = WaypointSet()
        land_set.type = self.WaypointSetType["LAND_POSE"]
        params = [Param_float("loit_heading",0.0),Param_float("loit_radius",40.0),Param_float("loit_forward_moving",0.0),
                 Param_float("precision_mode",0.0),Param_float("abort_alt",0.0),Param_float("yaw_angle",0.0) ]
        land_set.params = params
        land_pose = PoseStamped()
        land_pose.pose.position = Point(0.0,0.0,0.0)
        land_set.posestamped_list = [land_pose]
        land1_pose = PoseStamped()
        land1_pose.pose.position = Point(25.0,-100.0,10.0)
        land_set.posestamped_list.append(land1_pose)
        new_mission.append(land_set)

        print(new_mission)

        self.SetMissionServiceCall(new_mission)


    def SetMissionServiceCall(self,_new_mission):
        rospy.wait_for_service('/ual/set_mission'.format(self.ID))
        print("set_mission service active")
        try:
            del_model_prox = rospy.ServiceProxy('/ual/set_mission'.format(self.ID), SetMission)
            del_model_prox(_new_mission,False)
            time.sleep(0.1)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            print "error in delete_model agent"
        return


def main():

    test_node = TestNode()

if __name__ == '__main__':
    main()
