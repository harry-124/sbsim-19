#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import pid
import rospy
import math as m
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg
from sbsim.msg import dribble
import controller as c
from std_msgs.msg import Int32
from std_msgs.msg import Float64

r10 = Pose()
r11 = Pose()
r20 = Pose()
r21 = Pose()
ball = Pose()
d =  dribble()



def subinit():
    rospy.Subscriber('ballpose',Pose,ballcallback)
    rospy.Subscriber('robot1n0/pose',Pose,r10callback)
    rospy.Subscriber('robot1n1/pose',Pose,r11callback)
    rospy.Subscriber('robot2n0/pose',Pose,r20callback)
    rospy.Subscriber('robot2n1/pose',Pose,r21callback)
    rospy.Subscriber('game/dribdist',Float64,ddcallback)
    rospy.Subscriber('game/dribbler',Int32,drcallback)

    

def boundcheck(a):
    dir = [0,0]

    if a.x >= 470:
        dir[0] = 1
        fx= 1
    elif a.x <= -470:
        dir[0] = -1
        fx = 1
    else:
        dir[0] = 0
        fx = 0
    if a.y >= 360:
        dir[0] = 1
        fy = 1
    elif a.y <= -360:
        dir[0] = -1
        fy = 1
    else:
        dir[0] = 0
        fy = 0

    f = fx + fy

    if f ==2:
        f =1

    return f

def ddcallback(msg):
    return 0

def drcallback(msg):
    return 0

def ballcallback(msg):
    global ball
    ball = msg
    return 0

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

def updatebpose(a,b):
    b.x = a.position.x 
    b.y = a.position.y 

def updaterpose(a,b):
    b.x = a.position.x 
    b.y = a.position.y 
    b.theta = 2 * m.atan(a.orientation.z) 

if __name__ == '__main__':
    rospy.init_node('rules',anonymous=True)
    statuspub = rospy.Publisher('game/status', Int32, queue_size=10)
    rate = rospy.Rate(30)
    subinit()
    b = p.ball(x = ball.position.x,y = ball.position.y)
    r1 = p.robot(x =0 ,y =0,yaw =0 ,ball =b)
    r2 = p.robot(x =0 ,y =0,yaw =0 ,ball =b)
    r3 = p.robot(x =0 ,y =0,yaw =0 ,ball =b)
    r4 = p.robot(x =0 ,y =0,yaw =0 ,ball =b)
    updaterpose(r10,r1)
    updaterpose(r11,r2)
    updaterpose(r20,r3)
    updaterpose(r21,r4)
    i = 0
    while(True):
        i = i+1
        b.x = ball.position.x 
        b.y = ball.position.y 
        updaterpose(r10,r1)
        updaterpose(r11,r2)
        updaterpose(r20,r3)
        updaterpose(r21,r4)
        f = boundcheck(b)
        if f == 1:
            if b.x>470 and b.y<80 and b.y>-80:
                print 'goal for team 1'
                f = 2
            if b.x<-470 and b.y<80 and b.y>-80:
                print 'goal for team 2'
                f = 3
        statuspub.publish(f)
        f = 0
        rate.sleep()
