#!/usr/bin/env python
import sys
import rospy
import numpy as np
from math import *
import serial
import time
import timeit   
from geometry_msgs.msg import Twist
from xbee import XBee
global w,v
looprate=20 # 30 hz
port = serial.Serial("/dev/ttyUSB1", baudrate=57600)
xbee = XBee(port)
numbot = 5;
v = np.zeros(numbot)
w = np.zeros(numbot)
def node_cb(data):                                                      #callback for the suscriber to initialise the packet variables
    global w,v
    msg = data._connection_header['topic']                              #extracting topic from connection header
    bot_id = ord(msg[4])-49                                             #extracting bot_id from the suscribed topic 
    rospy.loginfo(bot_id)                                               #debug
    v[bot_id] = data.linear.x
    w[bot_id] = data.angular.z


def generate_packet(v,w,i):                                             #function to generate the messges that need to be sent
    ##""let send both in 4 bits..can be in 8""
    v_mag = (round(v[i],2)*10000);
    w_mag = (round(w[i],3)*1000);
    if v_mag >= 0:                                                      #final packet is a string of characters, hence need to change sign to char
        v_sign = '+'
    if v_mag <= 0:
        v_sign = '-'
        v_mag = v_mag*-1                                                #v be positive now B)
    if w_mag >= 0:
        w_sign = '+'
    if w_mag <= 0:
        w_sign = '-'
        w_mag = w_mag*-1                                                #w be positive now B)

    v_net = str(int(v_mag)).zfill(4)                                    #f2a function for making a composite string of 4 bits
    w_net = str(int(w_mag)).zfill(4)    
    final_packet = '#'  + v_sign + v_net + w_sign + w_net + '$'         #making packet (#) is the start bit and ($) is the stop bit
    dest_16bit = '\x00'+chr(i+1)
    xbee.tx(dest_addr=dest_16bit, data=final_packet)                    #send packets 

def bot_driver():
    global w,v
    global numbot
    

    for i in range(0,numbot):                                           #setup Suscribers and addresses for data transfer                                       
        rospy.Subscriber('/bot'+chr(i+49)+'/cmd_vel',Twist,node_cb)     #Creating Suscribers for each of the bots

    rate=rospy.Rate(looprate)                                           #define the loop rate
    while not rospy.is_shutdown():
        for i in range(0,numbot):
            generate_packet(v,w,i)   
        rate.sleep()
#define data logging to bag file function
if __name__ == '__main__':
    rospy.init_node('controller_node',anonymous=True)                   #initialize node
    numbot = int(sys.argv[1])
    rospy.loginfo(numbot)
    v = np.zeros(numbot)
    w = np.zeros(numbot)
    try:
        bot_driver()
    except rospy.ROSInterruptException:
        pass
