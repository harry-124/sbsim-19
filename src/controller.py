#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import pid
import rospy
import math as m
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg
"""
if gmsg.status == 0 bot is stationary
if gmsg.status == 1 bot moves to location in gmsg
if gmsg.status == 2 ball kicked at target location

"""
def control(gmsg,robot,ball):
    mybotpid = pid.pid(x=robot.x,y=robot.y,ball = ball,angle=robot.theta)
    if gmsg.status == 1 or gmsg.status == 2:
        th = 2*m.atan(gmsg.posetogo.orientation.z)
        if abs(robot.x - gmsg.posetogo.position.x)<20 and abs(robot.y - gmsg.posetogo.position.y)<20 and abs(robot.theta - 2*m.atan(gmsg.posetogo.orientation.z))< 1:
            if gmsg.status == 2:
                print('ball kicked')
                robot.kick(ball,3)
                gmsg.status = 0
        mybotpid.gtg(gmsg.posetogo.position.x,gmsg.posetogo.position.y,robot,ball,thtg=th)
        if robot.dribble == 0:
            p.collRb(robot,ball)
        p.walleffect(robot)
        p.walleffect(ball)

    if gmsg.status == 0:
        mybotpid.gtg(robot.x,robot.y,robot,ball,thtg=robot.theta)
        if robot.dribble == 0:
            p.collRb(robot,ball)
            p.walleffect(robot)
            p.walleffect(ball)

            mybotpid.gtg(robot.x,robot.y,robot,ball,thtg=robot.theta)
            if robot.dribble == 0:
                p.collRb(robot,ball)
                p.walleffect(robot)
                p.walleffect(ball)
            gmsg.status = 0
    


        
