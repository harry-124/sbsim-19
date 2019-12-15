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
r10cs = 1
r11cs = 1
r20cs = 1
r21cs = 1

def r10selcallback(msg):
    global r10cs
    if msg.data == 3:
        r10cs = 3
    else:
        r10cs = 0
    return 0

def r11selcallback(msg):
    global r11cs
    if msg.data == 3:
        r11cs = 3
    else:
        r11cs = 0
    return 0

def r20selcallback(msg):
    global r20cs
    if msg.data == 3:
        r20cs = 3
    else:
        r20cs = 0
    return 0

def r21selcallback(msg):
    global r21cs
    if msg.data == 3:
        r21cs = 3
    else:
        r21cs = 0
    return 0

def exctrl10callback(msg):
    global phw
    phw.publish(msg)
    return 0

def exctrl11callback(msg):
    global phw
    phw.publish(msg)
    return 0

def exctrl20callback(msg):
    global phw
    phw.publish(msg)
    return 0

def exctrl21callback(msg):
    global phw
    phw.publish(msg)
    return 0

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
    global r10cs
    global r11cs
    global r20cs
    global r21cs
    R = 46.6  # radius of robot in pixels
    r = 15.7 # radius of wheels in pixels
    vgx = msg.kx
    vgy = msg.ky
    wz = msg.thetad
    tag = msg.tag
    kick = msg.kick
    flag = 0
    # global to local transformations
    if tag==0:
        if not r10cs == 3: #checking if not in extremal control mode
            flag = 1
        bcos = m.cos(r1[0].theta)
        bsin = m.sin(r1[0].theta)
    if tag==1:
        if not r11cs == 3: #checking if not in extremal control mode
            flag = 1
        bcos = m.cos(r1[1].theta)
        bsin = m.sin(r1[1].theta)
    if tag==2:
        if not r20cs == 3: #checking if not in extremal control mode
            flag = 1
        bcos = m.cos(r2[0].theta)
        bsin = m.sin(r2[0].theta)
    if tag==3:
        if not r21cs == 3: #checking if not in extremal control mode
            flag = 1
        bcos = m.cos(r2[1].theta)
        bsin = m.sin(r2[1].theta)
    if flag == 1:
        vx = vgx*bcos + vgy*bsin
        vy = -vgx*bsin + vgy*bcos
        # inverse kinematics of 3 wheeled system
        w1 = ((vx/1.732)/r) - ((vy/3)/r) + ((wz*R)/r)
        w2 = ((vy/1.5)/r) + ((R*wz)/r)
        w3 = ((wz*R)/r) - ((vx/1.732)/r) - ((vy/3)/r)
        a = hw(w1=w1,w2=w2,w3=w3,tag=tag,kick=kick)
        phw.publish(a)
    return 0

def gamefun():
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ik',anonymous=True)
    rospy.Subscriber('ctrl',game,ctrlcallback)
    rospy.Subscriber('robot1n0/extremal_ctrl',hw,exctrl10callback)
    rospy.Subscriber('robot1n1/extremal_ctrl',hw,exctrl11callback)
    rospy.Subscriber('robot2n0/extremal_ctrl',hw,exctrl20callback)
    rospy.Subscriber('robot2n1/extremal_ctrl',hw,exctrl21callback)
    rospy.Subscriber('robot1n0/pose', Pose, botcallback1)  
    rospy.Subscriber('robot1n1/pose', Pose, botcallback2)  
    rospy.Subscriber('robot2n0/pose', Pose, botcallback3)  
    rospy.Subscriber('robot2n1/pose', Pose, botcallback4)  
    rospy.Subscriber('robot1n0/cselect',Int32,r10selcallback)
    rospy.Subscriber('robot1n1/cselect',Int32,r11selcallback)
    rospy.Subscriber('robot2n0/cselect',Int32,r20selcallback)
    rospy.Subscriber('robot2n1/cselect',Int32,r21selcallback)
    try:
        gamefun()
    except rospy.ROSInterruptException:
        pass