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

d = 0

r10msg = goalmsg()
r11msg = goalmsg()
r20msg = goalmsg()
r21msg = goalmsg() 
r1f = 1
r2f = 1
r3f =1
r4f =1

gs = 0
 
def dribbletest(r1,r2,r3,r4):
    global d
    if r1.dribble == 1:
        d = 1
    elif r2.dribble == 1:
        d = 2
    elif r3.dribble == 1:
        d = 3
    elif r4.dribble == 1:
        d = 4
    else:
        d =0
    return 0


def robotpubinit(t,n):
    namepose = 'robot'+str(t)+'n'+str(n)+'/pose'
    nametwist = 'robot'+str(t)+'n'+str(n)+'/twist'
    namer = 'robot'+str(t)+'n'+str(n)+'/reached'
    return rospy.Publisher(namepose,Pose, queue_size = 10),rospy.Publisher(namer,Int32,queue_size = 10)

def robotsubinit():
    rospy.Subscriber('robot1n0/ptg',goalmsg,r10callback)
    rospy.Subscriber('robot1n1/ptg',goalmsg,r11callback)
    rospy.Subscriber('robot2n0/ptg',goalmsg,r20callback)
    rospy.Subscriber('robot2n1/ptg',goalmsg,r21callback)

def r10callback(msg):
    global r10msg
    global r1f
    r10msg = msg
    r1f = 1
    return 0

def r11callback(msg):
    global r11msg
    global r2f
    r11msg = msg
    r2f = 1
    return 0

def r20callback(msg):
    global r3f
    global r20msg
    r20msg = msg
    r3f = 1
    return 0

def r21callback(msg):
    global r4f
    global r21msg
    r21msg  = msg
    r4f = 1
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
    if b.distdribbled != 0:
        return b.distdribbled


def updatebpose(a,b):
    a.position.x = b.x
    a.position.y = b.y
    a.orientation.w = 1

def rulecheck(msg):
    global gs
    gs = msg.data
    return 0

def reset(t1,t2,r1,r2,r3,r4,ball):
    global gs
    r1 = p.robot(x= t1[0][0],y= t1[0][1], yaw  = 0, ball = ball)
    r2 = p.robot(x= t1[1][0],y= t1[1][1], yaw  = 0, ball = ball)
    r3 = p.robot(x= t2[0][0],y= t2[0][1], yaw  = 3.14, ball = ball)
    r4 = p.robot(x= t2[1][0],y= t2[1][1], yaw  = 3.14, ball = ball)
    ball = p.ball(x = 0,y = 0)
    gs = 0


def game(t1,t2):
    global r10msg
    global r11msg
    global r20msg
    global r21msg
    global d
    global gs
    global r1f
    global r2f
    global r3f
    global r4f
    rospy.Subscriber('game/status',Int32,rulecheck)
    robotsubinit()
    pubball = rospy.Publisher('ballpose', Pose, queue_size=10)
    drib = rospy.Publisher('game/dribbler', Int32, queue_size=10)
    yis = rospy.Publisher('game/dribdist', Float64, queue_size=10)
    pr1 = []
    pr2 = []
    a,r1r = robotpubinit(1,0)
    pr1.append(a)
    a,r2r = robotpubinit(1,1)
    pr1.append(a)
    a,r3r = robotpubinit(2,0)
    pr2.append(a)
    a,r4r = robotpubinit(2,1)
    pr2.append(a)
    btwist = Twist()
    rate = rospy.Rate(60)
    while True:
        ball = p.ball(x = 0,y = 0)
        bpose = Pose()
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
        pr1[0].publish(rpose[0])
        pr1[1].publish(rpose[1])
        pr2[0].publish(rpose[2])
        pr2[1].publish(rpose[3])
        pubball.publish(bpose)
        while not rospy.is_shutdown():
            if gs == 0:
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
                dribbletest(r1[0 ],r1[1],r2[0],r2[1])
                dribbletest(r1[0],r1[1],r2[0],r2[1])
                updatebpose(bpose,ball)
                x1 = updaterpose(rpose[0],r1[0])
                x2 = updaterpose(rpose[1],r1[1])
                x3 = updaterpose(rpose[2],r2[0])
                x4 = updaterpose(rpose[3],r2[1])
                x = [x1,x2,x3,x4]
                y = max(x)
                yis.publish(y)
                r1r.publish(r1f)
                r2r.publish(r2f)
                r3r.publish(r3f)
                r4r.publish(r4f)
                pr1[0].publish(rpose[0])
                pr1[1].publish(rpose[1])
                pr2[0].publish(rpose[2])
                pr2[1].publish(rpose[3])
                pubball.publish(bpose)
                drib.publish(d)
                rate.sleep()
            else:
                dribbletest(r1[0],r1[1],r2[0],r2[1])
                updatebpose(bpose,ball)
                x1 = updaterpose(rpose[0],r1[0])
                x2 = updaterpose(rpose[1],r1[1])
                x3 = updaterpose(rpose[2],r2[0])
                x4 = updaterpose(rpose[3],r2[1])
                x = [x1,x2,x3,x4]
                y = max(x)
                yis.publish(y)
                r1r.publish(r1f)
                r2r.publish(r2f)
                r3r.publish(r3f)
                r4r.publish(r4f)
                pr1[0].publish(rpose[0])
                pr1[1].publish(rpose[1])
                pr2[0].publish(rpose[2])
                pr2[1].publish(rpose[3])
                pubball.publish(bpose)
                drib.publish(d)
                rate.sleep()
                break



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