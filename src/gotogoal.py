#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import rospy
import math as m
from geometry_msgs.msg import Pose

def posepub(xtg,ytg,x,y,bx,by):
    prod = 0.055
    kpp = 0.1
    kpv = prod/kpp
    pub = rospy.Publisher('robot1n1/pose', Pose, queue_size=10)
    pubball = rospy.Publisher('ballpose', Pose, queue_size=10)
    pose  = Pose()
    bpose = Pose()
    rate = rospy.Rate(60)
    pg.init()
    mybot = p.robot(x=x,y=y)
    ball = p.ball(x = bx,y = by)
    while not rospy.is_shutdown():
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()
        vrx = (kpp*(xtg - mybot.x))
        vry = (kpp*(ytg - mybot.y))
        kx = (kpv*(vrx/10 - mybot.xd))
        ky = (kpv*(vry/10- mybot.yd))
        mybot.movebot(kx,ky,thetad=0)
        p.collRb(mybot,ball)
        bpose.position.x = ball.x
        bpose.position.y = ball.y
        p.walleffect(mybot)
        p.walleffect(ball)
        pose.position.x = mybot.x
        pose.position.y = mybot.y
        pose.orientation.z = m.tan(mybot.theta/2)
        pose.orientation.w =1
        bpose.orientation.w =1
        pub.publish(pose)
        pubball.publish(bpose)

        if(p.dist(mybot.x,mybot.y,xtg,ytg)<10 and ball.speed <= 1):
            return mybot.x,mybot.y,ball.x,ball.y
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('collisiontest', anonymous=True)
    x = -100
    y = 0
    bx =0
    by =0
    while True:
        xtg = input('Enter x value')
        ytg = input('Enter y value')
        try:
            x,y,bx,by = posepub(xtg,ytg,x,y,bx,by)
        except rospy.ROSInterruptException:
            pass