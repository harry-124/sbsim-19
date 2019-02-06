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
from std_msgs.msg import Float64
from sbsim.msg import dribble
import random as rnd
import time

flag = 1
bpose =  Pose()
btwist = Twist()
d = 0



def summa():
    global flag
    global bpose
    global btwist
    rospy.init_node('movcheck',anonymous=True)
    r1 = rospy.Publisher('robot1n0/ptg',goalmsg, queue_size = 10)
    r = goalmsg()
    rate = rospy.Rate(10)
    r.posetogo.position.x = 300
    r.posetogo.position.y = 300
    r.posetogo.orientation.z = 0  
    r.posetogo.orientation.w = 1
    r.status =1
    r1.publish(r)
    rate.sleep()
    rate.sleep()
    rate.sleep()
    i = 0
    while(True):
        r.posetogo.position.x = 300+40*m.sin(2*3.14*i/100)
        r.posetogo.position.y = 300+40*m.cos(2*3.14*i/100)
        r.posetogo.orientation.z = 0  
        r.posetogo.orientation.w = 1
        r.status = 1
        r1.publish(r)
        i = i+1
        rate.sleep()

if __name__ == '__main__':
    summa()
    