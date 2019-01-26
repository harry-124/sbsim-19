#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import pid
import rospy
import math as m
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg


def control(gmsg,robot,ball):
    mybotpid = pid.pid(x=robot.x,y=robot.y,ball = ball,angle=robot.theta)
    if gmsg.status == 1:
        mybotpid.gtg(gmsg.posetogo.position.x,gmsg.posetogo.position.y,robot,ball,thtg=2*m.atan(gmsg.posetogo.orientation.z))
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
    if abs(robot.x - gmsg.posetogo.position.x)<2 and abs(robot.y - gmsg.posetogo.position.y)<2 and abs(robot.theta - 2*m.atan(gmsg.posetogo.orientation.z)):
        gmsg.status = 0 


        