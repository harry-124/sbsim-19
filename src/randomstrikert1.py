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

def callback(msg):
    global flag
    flag = 0


def summa():
    global flag
    rospy.init_node('randomstrikert1',anonymous=True)
    r1 = rospy.Publisher('robot1n0/ptg',goalmsg, queue_size = 10)
    rospy.Subscriber('robot1n0/reached',Int32,callback)
    rate = rospy.Rate(0.5)
    r = goalmsg()
    rate.sleep()
    r.posetogo.position.x = 50
    r.posetogo.orientation.w = 1
    r.status = 1
    r1.publish(r)
    rate.sleep()
    r.posetogo.position.x = 150
    r.posetogo.orientation.w = 1
    r.status = 1
    r1.publish(r)
    rate.sleep()
    flag = 1
    r.posetogo.position.x = 250
    a = rnd.randint(1,2)
    if a == 2:
        a = -1
    b = rnd.randint(1,80)
    n = a*b
    r.posetogo.position.y = n
    r.posetogo.orientation.z = -a*m.sin(b/200.0)
    r.posetogo.orientation.w = 1
    r.status = 2
    r1.publish(r)
    rate.sleep()



if __name__ == '__main__':
    summa()
    

