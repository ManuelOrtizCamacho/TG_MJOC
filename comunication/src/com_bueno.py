#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
from std_msgs.msg import String
received_data = ""
base_movement_info = " "

def talker():
    pub = rospy.Publisher('chatter_str', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Configura la conexión Serial para recibir datos
    serial_port = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 9600, 8, 'N', 1, 0)
   
    while not rospy.is_shutdown():
            
            read_serial(serial_port)
            rospy.loginfo("Received: %s", base_movement_info )
            pub.publish(base_movement_info )

            rate.sleep()

    
def read_serial(serial_port):
     global base_movement_info 
     received_data = serial_port.read().decode()
     while len(received_data ) > 0 :
        rospy.loginfo("ENTRE")
        if (received_data == "\n"): 
             pass

       
        elif (received_data  == "\x00") :
             pass

        else:
            # Build up base_movement_info character by character
            base_movement_info += received_data 
        
        received_data  = serial_port.read().decode()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
