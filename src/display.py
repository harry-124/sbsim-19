#!/usr/bin/env python
import sys
import pygame as pg
import rospy
import numpy
import tf
from geometry_msgs.msg import Pose
import math as m

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
        [self.xo,self.yo] = disptf(self.xo,self.yo)
        


class robot:
    def __init__(self,xp,yp,yaw=0,team=3,n=None):
        self.name = 'robot'+str(team)+'n'+str(n)+'/pose'
        self.n = n
        self.ball_sub = rospy.Subscriber(self.name,Pose, self.botcallback)
        self.radius = 40
        self.xo = int(xp)
        self.yo = int(yp)
        self.yaw = yaw
        if team == 1:
            self.color = [255,0,0]
        elif team == 2:
            self.color = [0,0,255]
        else:
            self.color = [60,200,60]

    def refr(self):
        global screen
        pg.draw.circle(screen, self.color,[self.x,self.y],self.radius,0)
        pg.draw.circle(screen,[0,0,0],[self.x+int(20*m.cos(self.yaw)),self.y+int(20*m.sin(self.yaw))],10,0)
        font = pg.font.Font('freesansbold.ttf',20)
        txtsf, txtre = text_objects(str(self.n),font)
        txtre.center = (self.x,self.y)
        screen.blit(txtsf,txtre)

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
        quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        self.yaw = euler[2]
        [self.xo,self.yo] = disptf(self.xo,self.yo)


def text_objects(text,font):
    txtsurf = font.render(text,True,(0,0,0))
    return txtsurf, txtsurf.get_rect()

def disptf(x,y):
    a = [(x+500),(380-y)]
    return a

def setuparena():
    screen.fill((0, 0, 0))
    pg.draw.rect(screen,[60,200,60],(20,20,960,720))
    pg.draw.rect(screen,[255,255,255],(20,20,960,720),8)
    pg.draw.rect(screen,[0,0,255],(0,300,20,160))
    pg.draw.rect(screen,[255,255,255],(496,0,8,760))
    pg.draw.rect(screen,[255,0,0],(980,300,20,160))
    pg.draw.circle(screen,[255,255,255],[500,380],80,8)

def run_display():
    global screen
    clock = pg.time.Clock()
    team1inits = [disptf(-420,0),disptf(-100,0)]
    team2inits = [[770,205],[770,575]]
    print(clock)
    b = ball()
    r = []
    i = 0
    while(i<2):
        r.append(robot(team1inits[i][0],team1inits[i][1],team=1,n=i))
        i =i+1
    i = 0
    while(i<2):
        r.append(robot(team2inits[i][0],team2inits[i][1],team=2,n=i))
        i =i+1
    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()        
        setuparena()
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
    try:
        run_display()
    except KeyboardInterrupt:
        exit()
