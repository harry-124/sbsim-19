import geometry as g
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

if __name__ == '__main__':
    a = g.point(1,1)
    b = g.point(3,3)
    c = g.point(1,0)
    d = g.point(0,1) 
    l1 = g.line(type = 1,d1 = a,d2 = b)
    l2 = g.line(type = 1,d1 = c,d2 = d)
    X = g.linters(l1,l2)
    print X.x,X.y
    w = g.point(4,0)
    z = g.point(0,4)
    l3 = g.perpbisector(c,d)
    l4 = g.line(1,w,z)
    Y = g.linters(l3,l4)
    print Y.x,Y.y