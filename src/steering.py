#!/usr/bin/env python3

import numpy as np
import rospy

from math import pi, radians, atan2

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, WrenchStamped

'''Joint Limits'''
J0_LIM = radians(30) # Steering Limit on Omni

J1_MIN = radians(10)  # Min Accelerator Pos
J1_MAX = radians(50) # Max Accelerator Pos

J2_TAR = radians(0)  # Lock Target Pos

'''Vehicle Limits'''
MAX_LIN_VEL = 0.2
MAX_ROT_VEL = 0.2

'''Joint Spring Constants'''
K0 = 0#3
C0 = -15

K1 = 0
C1 = 1

K2 = -3
C2 = 0

def limitVal(value, minVal, maxVal):
    return min(value, maxVal) if (value > minVal) else minVal

class SteerController:
    def __init__(self):        
        rospy.init_node('steer_ctrl', anonymous=True)

        # Subscribers
        self.sub_joint_pos = rospy.Subscriber('/arm/measured_js', JointState, self.listen)
        
        self.pub_speed = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pub_force = rospy.Publisher('/arm/servo_cf', WrenchStamped, queue_size=10)
        
        self.isAwake = False    
        
        self.vel_msg = Twist()
        self.tor_msg = WrenchStamped()

    def listen(self, data):            
        if len(data.position) > 2:
            j0 = limitVal(data.position[0], -J0_LIM, J0_LIM) / J0_LIM
            j1 = (limitVal(data.position[1], J1_MIN, J1_MAX) - J1_MIN) / (J1_MAX - J1_MIN)
            j2 = data.position[2]
                        
            self.vel_msg.angular.z = MAX_ROT_VEL * j0
            self.vel_msg.linear.x = MAX_LIN_VEL * j1

            t0 = K0 * j1/(abs(j0) if abs(j0)>0.1 else 0.1) + C0 * j0
            t1 = K1 * j1 + C1
            t2 = K2 * (j2 - J2_TAR) + C2 

            self.tor_msg.wrench.force.x = t0/10

            self.pub_speed.publish(self.vel_msg)
            self.pub_force.publish(self.tor_msg)

if __name__ == '__main__':
    try:
        vNode = SteerController()

        while not rospy.is_shutdown():
            pass
        
    except rospy.ROSInterruptException:
        pass
