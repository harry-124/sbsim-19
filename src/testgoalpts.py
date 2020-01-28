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
    robott = input('Enter team \n1\n2\n')
    robotn = input('Enter bot number \n0\n1\n')
    st = 'robot'+str(robott)+'n'+str(robotn)+'/goalpoints'
    path_pub=rospy.Publisher(st,path,queue_size = 20)
    for i in range(a):
        x = input('Enter x value of point'+str(i))
        y = input('Enter y value of point'+str(i))
        single = game()
        single.kx = x
        single.ky = y
        pts.append(single)
    path_pub.publish(pts)

def run():
    rospy.init_node('testgoalpts',anonymous=True)
    getpts()

if __name__ == '__main__':
    run()