#!/usr/bin/env python
import sys
import pygame as pg
import rospy
from geometry_msgs.msg import Pose

pg.init()
screen = pg.display.set_mode((1000,760))
pg.display.set_caption("simulation screen")


class ball:
    def __init__(self,xp=500,yp=380):
        self.name = 'ballpose'
        self.ball_sub = rospy.Subscriber(self.name,Pose, self.ballcallback)
        self.radius = 8
        self.xo = int(xp)
        self.yo = int(yp)
        self.color = [255,100,150]

    def refr(self):
        global screen
        pg.draw.circle(screen, self.color,[self.x,self.y],self.radius,0)

    def update(self,xp=None,yp=None):
        if xp is not None and yp is not None:
            self.x = int(xp)
            self.y = int(yp)
        else:
            self.x = int(self.xo)
            self.y = int(self.yo)

    def ballcallback(self,msg):
        self.xo = msg.position.x
        self.yo = msg.position.y


class robot:
    def __init__(self,xp,yp,team,n):
        self.name = 'robot'+str(team)+'n'+str(n)+'/pose'
        self.ball_sub = rospy.Subscriber(self.name,Pose, self.botcallback)
        self.radius = 40
        self.xo = int(xp)
        self.yo = int(yp)
        if team == 1:
            self.color = [255,0,0]
        elif team == 2:
            self.color = [0,0,255]
        else:
            self.color = [0,0,0]

    def refr(self):
        global screen
        pg.draw.circle(screen, self.color,[self.x,self.y],self.radius,0)

    def update(self,xp=None,yp=None):
        if xp is not None and yp is not None:
            self.x = int(xp)
            self.y = int(yp)
        else:
            self.x = int(self.xo)
            self.y = int(self.yo)

    def botcallback(self,msg):
        self.xo = msg.position.x
        self.yo = msg.position.y


def run_display():
    global screen
    x = 30
    y = 30
    rects = (20,20,960,720)
    clock = pg.time.Clock()
    team1inits = [[270,205],[270,575]]
    team2inits = [[770,205],[770,575]]
    print(clock)
    b = ball()
    r = []
    i = 0
    while(i<2):
        r.append(robot(team1inits[i][0],team2inits[i][1],1,i))
        i =i+1
    i = 0
    while(i<2):
        r.append(robot(team2inits[i][0],team2inits[i][1],2,i))
        i =i+1
    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()        
        screen.fill((0, 0, 0))
        pg.draw.rect(screen,[60,200,60],rects)
        pg.draw.rect(screen,[0,0,255],(0,352,20,56))
        pg.draw.rect(screen,[255,255,255],(496,0,8,760))
        pg.draw.rect(screen,[255,0,0],(980,352,20,56))
        pg.draw.circle(screen,[255,255,255],[500,380],80,8)
        b.update()
        b.refr()
        i = 0
        while(i<4):
            r[i].update()
            r[i].refr()
            i =i+1
        pg.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    rospy.init_node('displayer',anonymous=True)
    run_display()
