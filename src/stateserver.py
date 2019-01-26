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

r10msg = goalmsg()
r11msg = goalmsg()
r20msg = goalmsg()
r21msg = goalmsg() 

gs = 0

def robotpubinit(t,n):
    namepose = 'robot'+str(t)+'n'+str(n)+'/pose'
    nametwist = 'robot'+str(t)+'n'+str(n)+'/twist'
    namepossess = 'robot'+str(t)+'n'+str(n)+'/possess'
    return rospy.Publisher(namepose,Pose, queue_size = 10)

def robotsubinit():
    rospy.Subscriber('robot1n0/ptg',goalmsg,r10callback)
    rospy.Subscriber('robot1n1/ptg',goalmsg,r11callback)
    rospy.Subscriber('robot2n0/ptg',goalmsg,r20callback)
    rospy.Subscriber('robot2n1/ptg',goalmsg,r21callback)

def r10callback(msg):
    global r10msg
    r10msg = msg
    return 0
def r11callback(msg):
    global r11msg
    r11msg = msg
    return 0
def r20callback(msg):
    global r20msg
    r20msg = msg
    return 0
def r21callback(msg):
    global r21msg
    r21msg  = msg
    return 0    


#callback of subscriber to intelligence

#def freekick

#def throwin

#def goal

def updaterpose(a,b):
    a.position.x = b.x
    a.position.y = b.y
    a.orientation.z = m.tan(b.theta/2)
    a.orientation.w = 1


def updatebpose(a,b):
    a.position.x = b.x
    a.position.y = b.y
    a.orientation.w = 1


def game(t1,t2):
    global r10msg
    global r11msg
    global r20msg
    global r21msg
    robotsubinit()
    pubball = rospy.Publisher('ballpose', Pose, queue_size=10)
    pr1 = []
    pr2 = []
    a = robotpubinit(1,0)
    pr1.append(a)
    a = robotpubinit(1,1)
    pr1.append(a)
    a = robotpubinit(2,0)
    pr2.append(a)
    a = robotpubinit(2,1)
    pr2.append(a)
    ball = p.ball(x = 0,y = 0)
    bpose = Pose()
    btwist = Twist()
    rate = rospy.Rate(60)
    r1 = []
    r2 = []
    r1.append(p.robot(x= t1[0][0],y= t1[0][1], yaw  = 0, ball = ball))
    r1.append(p.robot(x= t1[1][0],y= t1[1][1], yaw  = 0, ball = ball))
    r2.append(p.robot(x= t2[0][0],y= t2[0][1], yaw  = 3.14, ball = ball))
    r2.append(p.robot(x= t2[1][0],y= t2[1][1], yaw  = 3.14, ball = ball))

    rpose = [Pose(),Pose(),Pose(),Pose()]
    updatebpose(bpose,ball)
    updaterpose(rpose[0],r1[0])
    updaterpose(rpose[1],r1[1])
    updaterpose(rpose[2],r2[0])
    updaterpose(rpose[3],r2[1])
    while not rospy.is_shutdown():
        c.control(r10msg,r1[0],ball)
        c.control(r11msg,r1[1],ball)
        c.control(r20msg,r2[0],ball)
        c.control(r21msg,r2[1],ball)
        p.collRR(r1[0],r2[0])
        p.collRR(r1[0],r2[1])
        p.collRR(r1[0],r1[1])
        p.collRR(r1[1],r2[0])
        p.collRR(r1[1],r2[1])
        p.collRR(r2[0],r2[1])
        updatebpose(bpose,ball)
        updaterpose(rpose[0],r1[0])
        updaterpose(rpose[1],r1[1])
        updaterpose(rpose[2],r2[0])
        updaterpose(rpose[3],r2[1])
        pr1[0].publish(rpose[0])
        pr1[1].publish(rpose[1])
        pr2[0].publish(rpose[2])
        pr2[1].publish(rpose[3])
        pubball.publish(bpose)
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('state_server',anonymous=True)
    print 'Select Formation for team 1'
    print '1. Striker + Defender'
    print '2. Dynamic Duo'
    a = input('Enter 1 or 2')
    print 'Select Formation for team 2'
    print '1. Striker + Defender'
    print '2. Dynamic Duo'
    b = input('Enter 1 or 2')
    if a == 1:
        posa = [[-50,0],[-250,0]]
    else:
        posa = [[-125,100],[-125,-100]]
    if b == 1:
        posb = [[50,0],[250,0]]
    else:
        posb = [[125,100],[125,-100]]
    try:
        game(posa,posb)
    except rospy.ROSInterruptException:
        pass