#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(data):
    #transform position joint simulacion to position dinamixel.
    # joint1 to MA
    joint_1 = data.position[0] 
    Motor_A = ((joint_1 - (-3.1416)) / (3.1416 - (-3.1416))) * (1023 - 0) + 0
    # joint2 to MB
    joint_2 = data.position[1] 
    Motor_B = ((joint_2 - (-1.7453)) / (1.9199 - (-1.7453))) * (1023 - 0) + 0
    # joint3 to MC
    joint_3 = data.position[2] 
    Motor_C = ((joint_3 - (-1.0471)) / (1.1345 - (-1.0471))) * (1023 - 0) + 0
    # joint4 to MD
    joint_4 = data.position[3] 
    Motor_D = ((joint_4 - (-3.4900)) / (3.4900 - (-3.4900))) * (1023 - 0) + 0

    # Imprimir el valor convertido
    rospy.loginfo(rospy.get_caller_id()+" Valor MA es %s"+" Valor MB es %s"+" Valor MC es %s"+" Valor MD es %s", Motor_A,Motor_B,Motor_C,Motor_D)



def listener():

    rospy.init_node('traductor')
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()