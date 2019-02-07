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

class vorenoiglobal:
    def __init__(self,r10,r11,r20,r21):


class coordinate:
    def __init__(self,x,y):
        self.x = x
        self.y = y