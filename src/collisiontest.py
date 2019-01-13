#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import rospy
import math as m
from geometry_msgs.msg import Pose

def posepub():
    pub = rospy.Publisher('robot1n1/pose', Pose, queue_size=10)
    pubball = rospy.Publisher('ballpose', Pose, queue_size=10)
    pose  = Pose()
    bpose = Pose()
    rate = rospy.Rate(60)
    pg.init()
    x = -50
    y = 0
    mybot = p.robot(x=x,y=y)
    ball = p.ball()
    i =0
    while not rospy.is_shutdown():
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()
        if (i < 10):
            mybot.movebot(1,0,0.2)
            i =i+1
        [ball.xd,ball.yd] = p.collRb(mybot,ball)
        ball.updatestate()
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
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('collisiontest', anonymous=True)
    try:
        posepub()
    except rospy.ROSInterruptException:
        pass