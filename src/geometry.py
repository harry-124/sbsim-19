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

def ptoposeconv(pt,ang):
    pose = Pose()
    pose.position.x = pt.x
    pose.position.y = pt.y
    pose.orientation.z = m.tan(ang/2)
    pose.orientation.w = 1
    return pose

def potwalls(pt):
    lbda = 0.001
    x = pt.x
    y = pt.y

    l = 980
    w = 720

    lby2 = l/2.0
    wby2 = w/2.0

    taplha1 = (lby2 - x)/(wby2 - y)
    taplha2 = (wby2 - y)/(lby2 - x)
    taplha3 = (lby2 + x)/(wby2 + y)
    taplha4 = (wby2 + y)/(lby2 - x)
    tbeta1 = (lby2 + x)/(wby2 - y)
    tbeta2 = (wby2 + y)/(lby2 - x)
    tbeta3 = (lby2 - x)/(wby2 + y)
    tbeta4 = (wby2 - y)/(lby2 - x)

    pot1 = np.log(taplha1*tbeta1)
    pot2 = np.log(taplha2*tbeta2)
    pot3 = np.log(taplha3*tbeta3)
    pot4 = np.log(taplha4*tbeta4)

    potf = pot1 + pot2 + pot3 + pot4

    return lbda*potf

def findmingrad(own,op1,op2,te1,goal):
    #print goal.x
    #print goal.y
    #print own.x,own.y,op1.x,op1.y,op2.x,op2.y,te1.x,te1.y
    mini = 1000000000000000
    ang = float('nan')
    for i in range(101):
        theta = i*2*3.14/100
        pt = point(0,0)
        pt.x = own.x + 40*m.cos(theta)
        pt.y = own.y + 40*m.sin(theta)
        potw = potwalls(pt)
        #print potw
        pot1 = findpot(pt,op1,50)
        #print pot1
        pot2 = findpot(pt,op2,50)
        #print pot2
        gpot = findpot(goal,pt,-30)
        #print gpot
        potf = -gpot + pot1 + pot2
        if (potf < mini):
            mini = potf
            ang = theta
    #print 'ang',ang
    return ang+3.14

def findpot(pt1,pt2,q):
    d = distb(pt1,pt2)
    pot =  q/d    
    return pot

def posetoptconv(pose):
    pt = point(pose.position.x,pose.position.y)
    ang = pose.orientation.z
    ang = 2*m.atan(ang)
    return pt,ang

def distb(p1,p2):
    return m.sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

def angb(p1,p2):
    c = p2.x-p1.X
    c /= distb(p1,p2)
    s = p2.y - p1.y
    s /= distb(p1,p2)
    ang = m.atan2(s,c)
    return -ang

class vect:
    def __init__(self,d1,d2): #angle from d1 to d2
        self.tail = d1
        self.head = d2 
        self.mag = distb(d1,d2)
        self.ang = angb(d1,d2)
        self.xcomp = m.cos(self.ang)
        self.ycomp = m.sin(self.ang)

    def getline(self,pt):
        m = m.tan(self.ang)
        l1 = line(type = 2,d1 = m, d2 = pt)
        return l1

def dotprod(v1,v2):
    dp = v1.xcomp*v2.xcomp + v1.ycomp*v2.ycomp
    dp = v1.mag* v2.mag * dp
    return dp

def crosprod(v1,v2):
    cp = v1.xcomp*v2.ycomp -  v1.ycomp*v2.xcomp
    cp = v1.mag*v2.mag*cp
    return cp

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
