
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import os

class BMControllerG00:
    def __init__(self):
        rospy.init_node("comunication") #inicializo el nodo con el nombre que le doy b

        self.VERSION = '0.1'
        self.SERIAL_PORT = '/dev/tttACMO'
        self.PWD_MIN = 10  # Minimum PWD supported by the base  que debe hacer si le llega el mensaje.
        self.LINEAR_TO_PWD = 100  # If linear.x = 2 m/s then set speed (pwd) to 100

        if not os.path.exists(self.SERIAL_PORT): # si no econtramos comunicacion por el serial nos mostrar este error
            rospy.logerr("Serial Port not found: %s, bm_controller not started" % self.SERIAL_PORT)
            rospy.signal_shutdown("Serial Port not found") # paro todo 
            return

        self.ser = serial.Serial(self.SERIAL_PORT, 9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0) # aqui defino la comunicacion, si es paridad o no , bit de parada , y cuantos.

        self.send_serial("")  # Send to clear out any noise in the serial buffer para limpiar el buffer

        self.twist_subs = rospy.Subscriber('base/cmd_vel', Twist, self.send_cmd_vel) # creo un subscriptor que se va a pegar a la variable twist
        self.base_movement_info = ""# esto ni idea que sea es un basio en

        rospy.loginfo("bm_controller has started: " + self.VERSION) # que imprima esto si llega 

    def send_cmd_vel(self, msg): # creamos la funcion de enviar mensajes.
        rospy.loginfo('Twist: linear: {:.3f}, angular: {:.3f}'.format(msg.linear.x, msg.angular.z)) # esta sera la forma de enviar la informacion.

        if msg.linear.x != 0:
            direction = 1
            if msg.linear.x < 0:
                direction = -1
            pwd = int(msg.linear.x * self.LINEAR_TO_PWD)
            if abs(pwd) < self.PWD_MIN:
                pwd = self.PWD_MIN * direction
            if abs(pwd) > 100:
                pwd = 100 * direction
            rospy.loginfo("sending serial:" + "RV" + "%+4d" % pwd)
            self.send_serial("RV" + "%+4d" % pwd) # Con esto estoy enviando informacion. por serial. ## toca tener cuidado 
        else: 
            rospy.loginfo("sending serial:" + "RV+0")
            self.send_serial("RV" + "%+4d" % 0) # Con esto estoy enviando informacion. por serial.

    def send_serial(self, send):
        self.ser.write((send + "\n").encode())# esto es para enviar la trama y el /n es lo que lo termina 

    def read_serial(self):     # con esto es la informacion que envia 
        read_chr = self.ser.read(1).decode()
        while len(read_chr) > 0:
            if read_chr == "\n":
                self.process_pose()
                self.base_movement_info = ""
            elif read_chr == "\x00":
                pass
            else:
                self.base_movement_info += read_chr
            read_chr = self.ser.read(1).decode()

    def process_pose(self):
        if len(self.base_movement_info) != 7:
            rospy.logerr("Misformed base_feedback message:" + self.base_movement_info)
        else:
            base_mode = self.base_movement_info[0:1]
            if base_mode not in ["R"]:
                rospy.logerr("Misformed base_feedback baseMode :" + base_mode)
            else:
                ticks_string = self.base_movement_info[1:7]
                if not ticks_string.isnumeric():
                    rospy.logerr("Misformed base_feedback ticks value :" + ticks_string)
                else:
                    ticks = int(ticks_string)
                    if base_mode == "R":
                        self.pose_angle = (ticks)
                        rospy.loginfo(" pose_angle:" + str(self.pose_angle))


if __name__ == '__main__':
    bm_controller = BMControllerG00()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        bm_controller.read_serial()
        rate.sleep()
