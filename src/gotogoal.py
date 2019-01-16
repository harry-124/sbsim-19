#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import pid
import rospy
import math as m
from geometry_msgs.msg import Pose

def posepub(xtg,ytg,x,y,ango,bx,by,ang,k):
    pub = rospy.Publisher('robot1n1/pose', Pose, queue_size=10)
    pubball = rospy.Publisher('ballpose', Pose, queue_size=10)
    pose  = Pose()
    bpose = Pose()
    k=k
    rate = rospy.Rate(60)
    pg.init()
    ball = p.ball(x = bx,y = by)
    mybot = p.robot(x=x,y=y,yaw = ango, ball = ball)
    mybotpid = pid.pid(x=x,y=y,ball = ball,angle=ango)
    ko = 0
    while not rospy.is_shutdown():
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()
        mybotpid.gtg(xtg,ytg,mybot,ball,thtg=ang)
        if mybot.dribble == 0:
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
        if(p.dist(mybot.x,mybot.y,xtg,ytg)<10 and abs(mybot.theta-angle)<0.1 and ball.speed <= 3 and ko ==0):
            if k==1 and mybot.dribble == 1:
                mybot.kick(ball,5)
            ko = 1
        #print ball.speed
        if(p.dist(mybot.x,mybot.y,xtg,ytg)<10 and abs(mybot.theta-angle)<0.1 and ball.speed <= 3 and ko ==1):
            return mybot.x,mybot.y,mybot.theta,ball.x,ball.y
        oldx = mybot.x
        oldy = mybot.y
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('collisiontest', anonymous=True)
    x = -100
    y = 0
    bx =0
    by =0
    ango =0
    while True:
        xtg = input('Enter x value')
        ytg = input('Enter y value')
        if xtg > 460 or xtg <-460 or ytg > 340 or ytg < -340:
            print 'Out of bounds'
            continue
        angle = input('Enter theta in rad')
        angle = p.restricang(angle)
        k = raw_input('Kick ball at destination? y/n')
        if k[0]=='y':
            k1 = 1
        else:
            k1 =0
        #print k1
        try:
            x,y,ango,bx,by = posepub(xtg,ytg,x,y,ango,bx,by,angle,k=k1)
        except rospy.ROSInterruptException:
            pass