#!/usr/bin/env python
"""
program to receive the 3 wheel speeds convert to global speeds
with noise and delay to simulate actual results 
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
import numpy as np
import tf

d = 0

r1f = 1
r2f = 1
r3f =1
r4f =1
ball = p.ball(x = 0,y = 0)

r1=[p.robot(x=0,y=0,ball=ball),p.robot(x=0,y=0,ball=ball)]
r2=[p.robot(x=0,y=0,ball=ball),p.robot(x=0,y=0,ball=ball)]

gs = 0

pctrl = rospy.Publisher('ctrl_rx',game,queue_size=20)

def botcallback1(msg):
    global r1
    global ball
    r1[0].x = msg.position.x
    r1[0].y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    r1[0].theta = euler[2]

def botcallback2(msg):
    global r1
    global ball
    r1[1].x = msg.position.x
    r1[1].y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    r1[1].theta = euler[2]

def botcallback3(msg):
    global r2
    global ball
    r2[0].x = msg.position.x
    r2[0].y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    r2[0].theta = euler[2]

def botcallback4(msg):
    global r2
    global ball
    r2[1].x = msg.position.x
    r2[1].y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    r2[1].theta = euler[2]



def hwcallback(msg):
    global pctrl
    global r1
    global r2
    global ball
    R = 46.6  # radius of robot in pixels
    r = 15.7 # radius of wheels in pixels
    w1 = msg.w1
    w2 = msg.w2
    w3 = msg.w3
    tag = msg.tag
    kick = msg.kick
    k = np.random.normal(0,0.01,3)
    w1 = w1 + k[0]
    w2 = w2 + k[1]
    w3 = w3 + k[2]
    #forward kinematics
    vx = -r*m.sqrt(3)/2*(w3-w1)
    vy = -r*(w1/2-w2+w3/2)
    wz = (r/(3*R))*(w1+w2+w3)
    #local to global conversion
    if tag==0:
        bcos = m.cos(r1[0].theta)
        bsin = m.sin(r1[0].theta)
    if tag==1:
        bcos = m.cos(r1[1].theta)
        bsin = m.sin(r1[1].theta)
    if tag==2:
        bcos = m.cos(r2[0].theta)
        bsin = m.sin(r2[0].theta)
    if tag==3:
        bcos = m.cos(r2[1].theta)
        bsin = m.sin(r2[1].theta)
    vxg = vx*bcos - vy*bsin
    vyg = vy*bcos + vx*bsin
    a = game(kx=vxg,ky=vyg,thetad=wz,tag= tag,kick=kick)
    pctrl.publish(a)
    

def gamefun():
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('fk',anonymous=True)
    rospy.Subscriber('wheel_vel_cmd',hw,hwcallback)
    rospy.Subscriber('robot1n0/pose', Pose, botcallback1)  
    rospy.Subscriber('robot1n1/pose', Pose, botcallback2)  
    rospy.Subscriber('robot2n0/pose', Pose, botcallback3)  
    rospy.Subscriber('robot2n1/pose', Pose, botcallback4)  
    try:
        gamefun()
    except rospy.ROSInterruptException:
        pass