#!/usr/bin/env python
import geometry as g
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
from sbsim.msg import dirvect
import random as rnd
import time

r10v = dirvect()
r11v = dirvect()
r20v = dirvect()
r21v = dirvect()

r10p = g.point(0,0)
r11p = g.point(0,0)
r20p = g.point(0,0)
r21p = g.point(0,0)

def r10vcallback(msg):
    global r10v
    r10v = msg
    return 0

def r11vcallback(msg):
    global r11v
    r11v = msg
    return 0

def r20vcallback(msg):
    global r20v
    r20v = msg
    return 0

def r21vcallback(msg):
    global r21v
    r21v = msg
    return 0

def r10pcallback(msg):
    global r10p
    r10p.x = msg.position.x
    r10p.y = msg.position.y
    return 0

def r11pcallback(msg):
    global r11p
    r11p.x = msg.position.x
    r11p.y = msg.position.y
    return 0

def r20pcallback(msg):
    global r20p
    r20p.x = msg.position.x
    r20p.y = msg.position.y
    return 0

def r21pcallback(msg):
    global r21p
    r21p.x = msg.position.x
    r21p.y = msg.position.y
    return 0

def findgoal(v,robot):
    go = goalmsg()
    if v.status == 1:
        r = v.speed
        qz = m.tan(v.ptheta/2)
        po = Pose()
        po.position.x = robot.x + r*m.cos(v.vtheta)
        po.position.y = robot.y + r*m.sin(v.vtheta)
        po.orientation.w = 1
        po.orientation.z = qz
        go.posetogo = po
        go.status = 1
        return go 
    else:
        qz = m.tan(v.ptheta/2)
        po = Pose()
        po.position.x = robot.x
        po.position.y = robot.y
        po.orientation.w = 1
        po.orientation.z = qz
        go.posetogo = po
        go.status = 1
        return go 
        return go



def run():
    rospy.Subscriber('/robot1n0/vtg',dirvect,r10vcallback)
    rospy.Subscriber('/robot1n1/vtg',dirvect,r11vcallback)
    rospy.Subscriber('/robot2n0/vtg',dirvect,r20vcallback)
    rospy.Subscriber('/robot2n1/vtg',dirvect,r21vcallback)

    rospy.Subscriber('/robot1n0/pose',Pose,r10pcallback)
    rospy.Subscriber('/robot1n1/pose',Pose,r11pcallback)
    rospy.Subscriber('/robot2n0/pose',Pose,r20pcallback)
    rospy.Subscriber('/robot2n1/pose',Pose,r21pcallback)

    r10pub = rospy.Publisher('/robot1n0/ptg', goalmsg, queue_size = 10)
    r11pub = rospy.Publisher('/robot1n1/ptg', goalmsg, queue_size = 10)
    r20pub = rospy.Publisher('/robot2n0/ptg', goalmsg, queue_size = 10)
    r21pub = rospy.Publisher('/robot2n1/ptg', goalmsg, queue_size = 10)

    r10ptg = goalmsg()
    r11ptg = goalmsg()
    r20ptg = goalmsg() 
    r21ptg = goalmsg()   

    rate = rospy.Rate(20)

    while(True):
        r10ptg = findgoal(r10v,r10p)
        r11ptg = findgoal(r11v,r11p)
        r20ptg = findgoal(r20v,r20p)
        r21ptg = findgoal(r21v,r21p)
        if not r10ptg.status == 0:
                r10pub.publish(r10ptg)
                r10ptg.status = 0
        if not r11ptg.status == 0:
                r11pub.publish(r11ptg)
                r11ptg.status = 0
        if not r20ptg.status == 0:
                r20pub.publish(r20ptg)
                r20ptg.status = 0
        if not r21ptg.status == 0:
                r21pub.publish(r21ptg)
                r21ptg.status = 0
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ptg_converter',anonymous=True)
    run()