#!/usr/bin/env python3.5
import rospy
import time
import numpy as np
import math
import copy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_srvs.srv import *
from uav_abstraction_layer.srv import *
from uav_abstraction_layer.msg import *

class FwQgc(object):
    def __init__(self):

        rospy.init_node('fw_qgc', anonymous=True)

        # self.listener()

        mission_wps = self.createMission()

        self.sendMission(mission_wps)


    def createMission(self):

        wps = []
        take_off_wps = WaypointSet()
        take_off_wps.type = WaypointSet.TAKEOFF_POSE
        take_off_wps.posestamped_list = [PoseStamped(std_msgs.msg.Header(),Pose(Point(50,0,20),Quaternion(0,0,0,1)))]
        take_off_wps.params = [Param_float("minimum_pitch", 10.0), Param_float("yaw_angle", 0.0)]

        wps.append(take_off_wps)

        return wps

        
    def sendMission(self, wps):

        mission_srv_request = SetMissionRequest()
        mission_srv_request.waypoint_set = wps
        mission_srv_request.blocking = True

        self.serverClient(mission_srv_request, "/ual/set_mission", SetMission)



    def serverClient(self, request, address, Type, print_request = False, print_response = False):

        if print_request == True:
            print(request)

        rospy.wait_for_service(address)

        try:
            client = rospy.ServiceProxy(address, Type)
            response = client(request)
            if print_response == True:
                print(response)

            return response
        
        except rospy.ServiceException:
            return "Error"

if __name__ == "__main__":
    FwQgc()
