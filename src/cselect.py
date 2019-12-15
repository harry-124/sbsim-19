#!/usr/bin/env python
"""
Program to choose the controller for the individual robots.
This is to reduce processing overhead by eliminating the running of the other controllers when one is being used.
"""

import rospy
import sys
import physics as p
import pygame as pg
import pid
import math as m
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sbsim.msg import dribble
from sbsim.msg import game
from sbsim.msg import hw
import tf

pc = []
pc.append(rospy.Publisher('/robot1n0/cselect',Int32,queue_size=20))
pc.append(rospy.Publisher('/robot1n1/cselect',Int32,queue_size=20))
pc.append(rospy.Publisher('/robot2n0/cselect',Int32,queue_size=20))
pc.append(rospy.Publisher('/robot2n1/cselect',Int32,queue_size=20))

def gamefun():
    """
    r_s robot selection flag
    c_s controller selection flag
    """
    global pc
    r = rospy.Rate(20)
    while(True):
        r_s = input('select robot \n1. robot1n0\n2.  robot1n1\n3.  robot2n0\n4.  robot2n1')
        c_s = input('select controller \n1. PID P2P \n2. Trajectory controller \n3. Extremal controller')
        pc[r_s].publish(c_s)
        r.sleep()

if __name__ == '__main__':
    rospy.init_node('ik',anonymous=True)
    try:
        gamefun()
    except rospy.ROSInterruptException:
        pass