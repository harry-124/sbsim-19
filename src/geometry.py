#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import pid
import rospy
import math as m
import numpy as np
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg
import controller as c
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sbsim.msg import dribble
import random as rnd
import time

class vorenoiglobal:
    def __init__(self,r10,r11,r20,r21,ball):
        self.r10 = point(r10.x,r10.y)
        self.r11 = point(r11.x,r11.y)
        self.r20 = point(r20.x,r20.y)
        self.r21 = point(r21.x,r21.y)

class vorenoiteam:
    def __init__(self,r0,r1):
        self.r0 = point(r0.x,r0.y)
        self.r1 = point(r1.x,r1.y)

def midpt(d1,d2):
    x = d1.x + d2.x
    y = d1.y + d2.y
    x = x/2
    y = y/2
    return point(x,y)

class line:
    def __init__(self, type,  d1, d2): # type 1 for 2 point for, type 2 for slope point, type 3 for slope intercept
        self.type = type
        if self.type == 1:
            self.s = 1
            if (d1.x - d2.x) == 0:
                self.s = 0
                self.m = 1
                self.c = -d1.x
            else:
                self.m = (d1.y - d2.y)/(d1.x - d2.x)
                self.c = d2.y - self.m*d2.x

        if self.type == 2:
            self.s = 1
            self.m = d1
            self.c = d2.y - self.m * d2.x

        if self.type == 3:
            self.s = 1
            self.m = d1
            self.c = d2

    def perpend(self):
        if self.s == 0:
            return 0
        else:
            if self.m == 0:
                return 10000
            else:
                ma = -1.0/self.m
                return ma

    def solutioncheck(self,d1): # checks whether point lies on line
        if ((self.m*d1.x + self.c - self.s*d1.y) == 0):
            return 1
        else:
            return 0

class point:
    def __init__(self,x,y):
        self.x = x
        self.y = y

def linters(lin1,lin2):
    A = np.array([[lin1.m,-lin1.s],[lin2.m,-lin2.s]])
    B = np.array([[-lin1.c],[-lin2.c]])
    Ainv = np.linalg.inv(A)
    X = Ainv.dot(B)
    x = X[0][0]
    y = X[1][0]
    return point(x,y)

def perpbisector(pt1,pt2):
    mdpt = midpt(pt1,pt2)
    l = line(type = 1,d1 = pt1, d2 =pt2)
    m = l.perpend()
    lf = line(type = 2,d1 = m,d2 = mdpt)
    return lf
