#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
from std_msgs.msg import String
from sensor_msgs.msg import JointState
base_movement_info =""
# Configura la conexión Serial para recibir datos
serial_port = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 9600, 8, 'N', 1, 0)

def joint_states_callback(data):
    #transform position joint simulacion to position dinamixel.
    # joint1 to MA
    joint_1 = data.position[0] 
    Motor_A = int(round(((joint_1 - (-3.1416)) / (3.1416 - (-3.1416))) * (1023 - 0) + 0))
    # joint2 to MB
    joint_2 = data.position[1] 
    Motor_B = int(round(((joint_2 - (-1.7453)) / (1.9199 - (-1.6453))) * (1023 - 0) + 0))
    # joint3 to MC
    joint_3 = data.position[2] 
    Motor_C = int(round(10+((joint_3 - (-1.0471)) / (1.1345 - (-1.0071))) * (1023 - 0) + 0))
    # joint4 to MD
    joint_4 = data.position[3] 
    Motor_D = int(round(((joint_4 - (-3.4900)) / (3.4900 - (-3.4900))) * (1023 - 0) + 0))
    command_string= "MA{:d}MB{:d}MC{:d}MD{:d}\n".format(Motor_A, Motor_B, Motor_C, Motor_D)
    #command_string= "MA{:d}\n".format(Motor_A,)
    send_data(command_string)
    #read_serial()
    #command_string_empty ='\n'
    #serial_port.write(command_string_empty.encode())
    # Imprimir el valor convertido
    
    #received_data = serial_port.read(1).decode()
    #rospy.loginfo(received_data)

def read_serial():
    global base_movement_info 
    received_data = serial_port.read(1).decode()
    while len(received_data ) > 0 :
        
        if (received_data == "\n"):
            rospy.loginfo("received_data")
            rospy.loginfo(received_data)

       
        elif (received_data  == "\x00") :
             pass

        else:
            # Build up base_movement_info character by character
            base_movement_info += received_data 
        
        received_data  = serial_port.read(1).decode()
    rospy.loginfo(base_movement_info)

def listener():
    rospy.init_node('traductor')
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    rospy.spin()

def send_data(command_string):
    rospy.loginfo(command_string)
    serial_port.write(command_string.encode())
    #serial_port.write(("").encode())
    rospy.loginfo(command_string)
    
    

def scale(data):
    #transform position joint simulacion to position dinamixel.
    # joint1 to MA
    joint_1 = data.position[0] 
    Motor_A = int(round(((joint_1 - (-3.1416)) / (3.1416 - (-3.1416))) * (1023 - 0) + 0))
    # joint2 to MB
    joint_2 = data.position[1] 
    Motor_B = int(round(((joint_2 - (-1.7453)) / (1.9199 - (-1.7453))) * (1023 - 0) + 0))
    # joint3 to MC
    joint_3 = data.position[2] 
    Motor_C = int(round(((joint_3 - (-1.0471)) / (1.1345 - (-1.0471))) * (1023 - 0) + 0))
    # joint4 to MD
    joint_4 = data.position[3] 
    Motor_D = int(round(((joint_4 - (-3.4900)) / (3.4900 - (-3.4900))) * (1023 - 0) + 0))
    command_string= "MA{:d}MB{:d}MC{:d}MD{:d}\n".format(Motor_A, Motor_B, Motor_C, Motor_D)
    #command_string= "MA{:d}\n".format(Motor_A,)
    return command_string

if __name__ == '__main__':
    listener()