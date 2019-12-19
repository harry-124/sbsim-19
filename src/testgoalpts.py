#!/usr/bin/env python
from __future__ import print_function
from numpy import *
import rospy
from sbsim.msg import path
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sbsim.msg import dribble
from sbsim.msg import game
def getpts():
    a = input('Enter number of points')
    pts = []
    for i in range(a):
        x = input('Enter x value of point'+str(i))
        y = input('Enter y value of point'+str(i))
        single = game()
        single.kx = x
        single.ky = y
        pts.append(single)

    return pts

def run():
    rospy.init_node('testgoalpts',anonymous=True)
    path_pub=rospy.Publisher('robot1n0/goalpoints',path,queue_size = 20)
    pts=getpts()
    path_pub.publish(pts)

if __name__ == '__main__':
    run()