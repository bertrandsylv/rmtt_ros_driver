#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 20 2022

@author: Sylvain BERTRAND
"""

import time
import robomaster
from robomaster import robot

import rospy
import numpy as np
from std_msgs.msg import Float32, Empty, ColorRGBA
from geometry_msgs.msg import Vector3, Twist

# node init
# --------------
rospy.init_node('rmtt_driver', anonymous=False)



# main variables
# ---------------
drone = robot.Drone()

frequency = 10.0
Ts = 1.0/frequency
nodeRate = rospy.Rate(frequency)


# parameters
# ------------
IP_ADDRESS_STR = rospy.get_param('~IP_ADRESS_STR', "192.168.10.2")
V_XY_MAX = 30   # betwen 0 and 100
V_Z_MAX = 60
V_YAW_RATE_MAX = 30


# init global variables
# ----------------------
global drone_state
drone_state = "LANDED"  # LANDED, HOVERFLYING





# publishers
# ----------------
pubBottomTof = rospy.Publisher('btm_range', Float32, queue_size=10)
pubForwardTof = rospy.Publisher('fwd_range', Float32, queue_size=10)
pubRollPitchYawAngle = rospy.Publisher('roll_pitch_yaw_deg', Vector3, queue_size=10)
pubVel = rospy.Publisher('vel', Vector3, queue_size=10)
pubAcc = rospy.Publisher('acc', Vector3, queue_size=10)

# call backs for subscribers
# ----------------------------
def callBackTakeOff(data):
    global drone_state
    if (drone_state=="LANDED"):
        print("     Taking off ...")
        drone.flight.takeoff().wait_for_completed()
        drone_state="FLYING"
        print("                ... Flying")
        #time.sleep(0.5)

def callBackLand(data):
    global drone_state
    if (drone_state=="FLYING"):
        print("     Landing ...")
        drone.flight.land().wait_for_completed()
        drone_state="LANDED"
        print("             ... Landed")
        #time.sleep(0.5)
        
        
def callBackCmdVel(data):
    # cmdvel linear(x,y,z)  angular(z)  all assumed to be in [-1,1]
    #roll, pitch, accelerate, yaw:  a,b,c,d [-100,100]
    vx = np.rint(100*np.clip(data.linear.x, -1.0, 1.0))
    vy = np.rint(100*np.clip(data.linear.y, -1.0, 1.0))
    vz = np.rint(100*np.clip(data.linear.z, -1.0, 1.0))
    v_yaw_rate = np.rint(100*np.clip(data.angular.z, -1.0, 1.0))
    
    # saturate for safety
    vx = np.clip(vx, -V_XY_MAX, V_XY_MAX)
    vy = np.clip(vy, -V_XY_MAX, V_XY_MAX)
    vz = np.clip(vz, -V_Z_MAX, V_Z_MAX)
    v_yaw_rate = np.clip(v_yaw_rate, -V_YAW_RATE_MAX, V_YAW_RATE_MAX)
    
    if (drone_state=="FLYING"):
        drone.flight.rc(a=-vy, b=vx, c=vz, d=-v_yaw_rate)


def callBackRGBLed(data):
    drone.led.set_led(r=data.r, g=data.g, b=data.b)


# subscribers
# ------------
rospy.Subscriber("takeoff", Empty, callBackTakeOff)
rospy.Subscriber("land", Empty, callBackLand)
rospy.Subscriber("cmd_vel", Twist, callBackCmdVel)
rospy.Subscriber("rgb_led", ColorRGBA, callBackRGBLed)

# handlers to SDK
# -----------------

# ----- bottom TOF sensor (in meters) -----
def sub_bottom_tof_info_handler(tof_cm):
    
    tof_fwd_cm = drone.sensor.get_ext_tof()
    
    #print("     drone bottom tof (cm): {0}".format(tof_cm))
    if (tof_cm>0):
        pubBottomTof.publish(Float32(tof_cm/100.0))
    else:
        pubBottomTof.publish(Float32(0.0))

    #print("     drone forward tof (cm): {0}".format(tof_fwd_cm))
    if (tof_fwd_cm==None):
        pubForwardTof.publish(Float32(np.nan))
    else:
        if (tof_fwd_cm>0):
            pubForwardTof.publish(Float32(tof_fwd_cm/100.0))
        else:
            pubForwardTof.publish(Float32(0.0))
       

# ----- roll, pitch, yaw angles (in  deg) -----
def sub_attitudeRPY_info_handler(attitute_angles):
    yaw, pitch, roll = attitute_angles
    #print("     drone attitude (deg): roll:{0}, pitch:{1}, yaw:{2} ".format(roll, pitch, yaw))
    pubRollPitchYawAngle.publish(Vector3(roll,pitch,yaw))

# --- accelerations in body frame (m/s2) and velocity in world frame (m/S  TBC)  ----
def sub_imu_info_handler(imu_info):
    vgx, vgy, vgz, agx, agy, agz = imu_info
    agx = 0.01*agx
    agy = 0.01*agy
    agz = 0.01*agz
    #print("     drone speed (unit ?): vgx {0}, vgy {1}, vgz {2}".format(vgx, vgy, vgz))
    #print("     drone acceleration (m/s2): agx {0}, agy {1}, agz {2}".format(agx, agy, agz))
    pubVel.publish(Vector3(vgx,vgy,vgz))
    pubAcc.publish(Vector3(agx,agy,agz))
    

# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    
    robomaster.config.LOCAL_IP_STR = IP_ADDRESS_STR
    
    print("\n**** RMTT ROS DRIVER ****")

    drone.initialize()
    print("  connexion to "+IP_ADDRESS_STR+" ..... ok")
    drone_version = drone.get_sdk_version()
    print("  drone sdk version: {0}".format(drone_version))
    
    
    drone.sub_tof(freq=10, callback=sub_bottom_tof_info_handler)
    drone.flight.sub_attitude(10, sub_attitudeRPY_info_handler)
    drone.flight.sub_imu(10, sub_imu_info_handler)
    
    
    rospy.spin()    
    
    '''
    while not rospy.is_shutdown():
        
        
#        tof_info = drone.sensor.get_ext_tof()
        
#        print("ext tof: {0}".format(tof_info))
        
        nodeRate.sleep()
    '''



    drone.unsub_tof()
    drone.flight.unsub_attitude()
    drone.flight.unsub_imu()

    drone.close()



