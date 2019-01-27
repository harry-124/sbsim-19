#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import pid
import rospy
import math as m
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg
import controller as c
from std_msgs.msg import Int32

r10 = Pose()
r11 = Pose()
r20 = Pose()
r21 = Pose()


def robotsubinit():
    rospy.Subscriber('robot1n0/pose',Pose,r10callback)
    rospy.Subscriber('robot1n1/pose',Pose,r11callback)
    rospy.Subscriber('robot2n0/pose',Pose,r20callback)
    rospy.Subscriber('robot2n1/pose',Pose,r21callback)

def r10callback(msg):
    global r10
    r10 = msg
    return 0

def r11callback(msg):
    global r11
    r11 = msg
    return 0

def r20callback(msg):
    global r20
    r20 = msg
    return 0

def r21callback(msg):
    global r21
    r21 = msg
    return 0

if __name__ == '__main__':
    rospy.init_node('rules',anonymous=True)

