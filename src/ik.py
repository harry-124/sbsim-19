#!/usr/bin/env python
"""
Program to convert control commands into 3 wheel velocities and publish that
receiver will add noise and delay to emulate real world.
will be used if extremal control is not used
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

d = 0

r1f = 1
r2f = 1
r3f =1
r4f =1
ball = p.ball(x = 0,y = 0)

r1=[p.robot(x=0,y=0,ball=ball),p.robot(x=0,y=0,ball=ball)]
r2=[p.robot(x=0,y=0,ball=ball),p.robot(x=0,y=0,ball=ball)]

gs = 0

phw = rospy.Publisher('wheel_vel_cmd',hw,queue_size=20)

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



def ctrlcallback(msg):
    global phw
    global r1
    global r2
    global ball
    R = 46.6  # radius of robot in pixels
    r = 15.7 # radius of wheels in pixels
    vgx = msg.kx
    vgy = msg.ky
    wz = msg.thetad
    tag = msg.tag
    kick = msg.kick
    # global to local transformations
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
    vx = vgx*bcos + vgy*bsin
    vy = -vgx*bsin + vgy*bcos
    # inverse kinematics of 3 wheeled system
    w1 = ((vx/1.732)/r) - ((vy/3)/r) + ((wz*R)/r)
    w2 = ((vy/1.5)/r) + ((R*wz)/r)
    w3 = ((wz*R)/r) - ((vx/1.732)/r) - ((vy/3)/r)
    a = hw(w1=w1,w2=w2,w3=w3,tag=tag,kick=kick)
    phw.publish(a)

def gamefun():
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ik',anonymous=True)
    rospy.Subscriber('ctrl',game,ctrlcallback)
    rospy.Subscriber('robot1n0/pose', Pose, botcallback1)  
    rospy.Subscriber('robot1n1/pose', Pose, botcallback2)  
    rospy.Subscriber('robot2n0/pose', Pose, botcallback3)  
    rospy.Subscriber('robot2n1/pose', Pose, botcallback4)  
    try:
        gamefun()
    except rospy.ROSInterruptException:
        pass