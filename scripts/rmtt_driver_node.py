#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 20 2022

@author: Sylvain BERTRAND
"""

#import time
import robomaster
from robomaster import robot

import rospy
#import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

# node init
# --------------
rospy.init_node('rmtt_driver', anonymous=True)


# node frequency 
# -----------------
frequency = 10.0
Ts = 1.0/frequency
nodeRate = rospy.Rate(frequency)


# parameters
# ------------
IP_ADDRESS_STR = rospy.get_param('~IP_ADRESS_STR', "192.168.10.2")


# init global variables
# ----------------------


# publishers
# ----------------
pubBottomTof = rospy.Publisher('bottom_tof', Float32, queue_size=10)
pubRollAngle = rospy.Publisher('roll_deg', Float32, queue_size=10)
pubPitchAngle = rospy.Publisher('pitch_deg', Float32, queue_size=10)
pubYawAngle = rospy.Publisher('yaw_deg', Float32, queue_size=10)
pubVel = rospy.Publisher('vel', Vector3, queue_size=10)
pubAcc = rospy.Publisher('acc', Vector3, queue_size=10)

# call backs for subscribers
# ----------------------------




# subscribers
# ------------



# main node loop
# ---------------



# handlers to SDK
# -----------------


# ----- bottom TOF sensor (in meters) -----
def sub_bottom_tof_info_handler(tof_cm):
    #print("     drone tof (cm): {0}".format(tof_cm))
    if (tof_cm>0):
        pubBottomTof.publish(Float32(tof_cm/100.0))
    else:
        pubBottomTof.publish(Float32(0.0))
    

# ----- roll, pitch, yaw angles (in  deg) -----
def sub_attitudeRPY_info_handler(attitute_angles):
    yaw, pitch, roll = attitute_angles
    print("     drone attitude (deg): roll:{0}, pitch:{1}, yaw:{2} ".format(roll, pitch, yaw))
    pubRollAngle.publish(Float32(roll))
    pubPitchAngle.publish(Float32(pitch))
    pubYawAngle.publish(Float32(yaw))

# --- accelerations in body frame (m/s2) and velocity in world frame (m/S  TBC)  ----
def sub_imu_info_handler(imu_info):
    vgx, vgy, vgz, agx, agy, agz = imu_info
    agx = 0.01*agx
    agy = 0.01*agy
    agz = 0.01*agz
    print("     drone speed (unit ?): vgx {0}, vgy {1}, vgz {2}".format(vgx, vgy, vgz))
    print("     drone acceleration (m/s2): agx {0}, agy {1}, agz {2}".format(agx, agy, agz))
    pubVel.publish(Vector3(vgx,vgy,vgz))
    pubAcc.publish(Vector3(agx,agy,agz))
    

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    
    robomaster.config.LOCAL_IP_STR = IP_ADDRESS_STR
    
    print("\n**** RMTT ROS DRIVER ****")
    drone = robot.Drone()
    drone.initialize()
    print("  connexion to "+IP_ADDRESS_STR+" ..... ok")
    drone_version = drone.get_sdk_version()
    print("  drone sdk version: {0}".format(drone_version))
    
    
    drone.sub_tof(freq=20, callback=sub_bottom_tof_info_handler)
    drone.flight.sub_attitude(10, sub_attitudeRPY_info_handler)
    drone.flight.sub_imu(10, sub_imu_info_handler)
    
    rospy.spin()    
    
    '''
    while not rospy.is_shutdown():
        
        
#        tof_info = drone.sensor.get_ext_tof()
        
#		print("ext tof: {0}".format(tof_info))
        
        nodeRate.sleep()
    '''



    drone.unsub_tof()
    drone.flight.unsub_attitude()
    drone.flight.unsub_imu()

    drone.close()



