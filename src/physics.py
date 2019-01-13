#!/usr/bin/env python
import sys
import numpy as np
import math as m




class ball:
    def __init__(self,x=0,y=0,xd=0,yd=0,xdd=0,ydd=0):
        self.x =x
        self.y = y
        self.xd =xd
        self.yd = yd
        self.xdd = xdd
        self.ydd = ydd
        self.cr = 0.8
        self.r = 8
        self.nu = 0.5
        self.speed = 0
        self.vthresh = 10

    def mtest(self):
        if self.xd != 0 and self.yd != 0:
            self.mflag = 1
        else:
            self.mflag = 0

    def updatestate(self):
        tmvx = self.xd
        tmvy = self.yd
        self.x+=self.xd
        self.y+=self.yd
        self.xd = self.xd + self.xdd
        self.yd = self.yd + self.ydd
        self.speed =m.sqrt(self.xd*self.xd+self.yd*self.yd)
        if self.xdd==0:
            self.xd = tmvx
        if self.ydd==0:
            self.yd = tmvy
        self.xdd =0
        self.ydd =0

    def moveball(self):
        if (self.xd>0):
            self.dirx =1
        elif (self.xd<0):
            self.dirx =-1
        else:
            self.dirx =0
        if (self.yd>0):
            self.diry =1
        elif (self.yd<0):
            self.diry =-1
        else:
            self.diry =0

        if(self.dirx ==0 and self.diry ==0):
            self.friction()
        elif (self.diry == 0 and self.dirx!=0):
            self.frictionx()
        elif (self.dirx == 0 and self.diry!=0):
            self.frictiony()

    def impulse(self,acx,acy):
        self.xdd = acx
        self.ydd = acy
        self.updatestate()
        self.xdd = 0
        self.ydd = 0

    def frictiony(self):
        self.mtest()
        if self.speed != 0: 
            #print(self.xd,self.yd)
            self.xdd = -((self.xd/self.speed)*self.nu)
            #self.ydd = -round((self.yd/self.speed)*nu)
            #print(self.xdd)
            #print(self.ydd)
            #self.updatestate()
            #print("xd",self.xd)
            #print("yd",self.yd)
            #self.xdd = 0
            #self.ydd = 0
        else:
            self.xdd=0
            #self.ydd=0
            self.mtest()

    def frictionx(self):
        self.mtest()
        if self.speed != 0: 
            #print(self.xd,self.yd)
            #self.xdd = -round((self.xd/self.speed)*nu)
            self.ydd = -((self.yd/self.speed)*self.nu)
            #print(self.xdd)
            #print(self.ydd)
            #self.updatestate()
            #print("xd",self.xd)
            #print("yd",self.yd)
            #self.xdd = 0
            #self.ydd = 0
        else:
            #self.xdd=0
            self.ydd=0
            self.mtest()

    def friction(self):
        self.mtest()
        if self.speed != 0: 
            #print(self.xd,self.yd)
            self.xdd = -((self.xd/self.speed)*self.nu)
            self.ydd = -((self.yd/self.speed)*self.nu)
            #print(self.xdd)
            #print(self.ydd)
            #self.updatestate()
            #print("xd",self.xd)
            #print("yd",self.yd)
            #self.xdd = 0
            #self.ydd = 0
        else:
            self.xdd=0
            self.ydd=0
            self.mtest()

        




class robot:
    def __init__(self,x,y,xd=0,yd=0,xdd=0,ydd=0,yaw=0):
        self.x =x
        self.y = y
        self.xd =xd
        self.yd = yd
        self.xdd = xdd
        self.ydd = ydd
        self.theta = yaw
        self.thetad = 0
        self.cr = 0.5
        self.r = 40
        self.nu = 1.5
        self.vthresh = 3
        self.dirx =0
        self.diry =0
        self.speed =0

    def mtest(self):
        if self.xd != 0 and self.yd != 0:
            self.mflag = 1
        else:
            self.mflag = 0

    def updatestate(self):
        tmvx = self.xd
        tmvy = self.yd
        self.x+=self.xd
        self.y+=self.yd
        self.xd = self.xd + self.xdd
        self.yd = self.yd + self.ydd
        if self.xd>self.vthresh:
            self.xd = tmvx
        if self.yd>self.vthresh:
            self.yd = tmvy
        self.theta = self.theta + self.thetad
        self.thetad = 0
        self.speed =m.sqrt(self.xd*self.xd+self.yd*self.yd)
        if self.xdd==0:
            self.xd = tmvx
        if self.ydd==0:
            self.yd = tmvy

    def impulse(self,acx,acy):
        self.xdd = acx
        self.ydd = acy
        self.updatestate()
        self.xdd = 0
        self.ydd = 0

    def rotate(self,thetad):
        self.thetad = thetad

    def frictiony(self):
        self.mtest()
        if self.speed != 0: 
            #print(self.xd,self.yd)
            self.xdd = -((self.xd/self.speed)*self.nu)
            #self.ydd = -round((self.yd/self.speed)*nu)
            #print(self.xdd)
            #print(self.ydd)
            self.updatestate()
            #print("xd",self.xd)
            #print("yd",self.yd)
            self.xdd = 0
            #self.ydd = 0
        else:
            self.xdd=0
            #self.ydd=0
            self.mtest()

    def frictionx(self):
        self.mtest()
        if self.speed != 0: 
            #print(self.xd,self.yd)
            #self.xdd = -round((self.xd/self.speed)*nu)
            self.ydd = -((self.yd/self.speed)*self.nu)
            #print(self.xdd)
            #print(self.ydd)
            self.updatestate()
            #print("xd",self.xd)
            #print("yd",self.yd)
            #self.xdd = 0
            self.ydd = 0
        else:
            #self.xdd=0
            self.ydd=0
            self.mtest()

    def friction(self):
        self.mtest()
        if self.speed != 0: 
            #print(self.xd,self.yd)
            self.xdd = -((self.xd/self.speed)*self.nu)
            self.ydd = -((self.yd/self.speed)*self.nu)
            #print(self.xdd)
            #print(self.ydd)
            self.updatestate()
            #print("xd",self.xd)
            #print("yd",self.yd)
            self.xdd = 0
            self.ydd = 0
        else:
            self.xdd=0
            self.ydd=0
            self.mtest()

    def teleop(self,dir,k=0):
        if(dir[0]==1 or dir[1]==1 or dir[2]==1 or dir[3]==1):
            if dir[0]==1:
                self.impulse(0,k)
                self.diry = -1
            if dir[1]==1:
                self.impulse(0,-k)
                self.diry = 1
            if dir[2]==1:
                self.impulse(-k,0)
                self.dirx = -1
            if dir[3]==1:
                self.impulse(k,0)
                self.dirx = 1
        if(self.dirx ==0 and self.diry ==0):
            self.friction()
        elif (self.diry == 0 and self.dirx!=0):
            self.frictionx()
        elif (self.dirx == 0 and self.diry!=0):
            self.frictiony()

    def movebot(self,kx,ky,thetad=0):
        if (self.xd>0):
            self.dirx =1
        elif (self.xd<0):
            self.dirx =-1
        else:
            self.dirx =0
        if (self.yd>0):
            self.diry =1
        elif (self.yd<0):
            self.diry =-1
        else:
            self.diry =0

        self.rotate(thetad)
        self.impulse(kx,ky)

        if(self.dirx ==0 and self.diry ==0):
            self.friction()
        elif (self.diry == 0 and self.dirx!=0):
            self.frictionx()
        elif (self.dirx == 0 and self.diry!=0):
            self.frictiony()


        








def dist(x1,y1,x2,y2):
    return m.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

def disto(a,b):
    return m.sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))

def angcs(x1,y1,x2,y2):
    c = (x2 - x1)/dist(x1,y1,x2,y2)
    s = (y2 - y1)/dist(x1,y1,x2,y2)
    return [c,s]

def angcso(a,b):
    c = (a.x - b.x)/disto(a,b)
    s = (a.y - b.y)/disto(a,b)
    return [c,s]

def wallcheck(a):
    if a.x >= 500:
        ox = 1
    elif a.x <= -500:
        ox = -1
    else:
        ox = 0
    if a.y >= 380:
        oy = 1
    elif a.y <= -380:
        oy = -1
    else:
        oy = 0
    return [ox,oy]

def colcheck(a,b):
    if disto(a,b)<=(a.r+b.r):
        return 1
    else: return 0

def collRb(R,b):
    if(colcheck(R,b)==1):
        ux = R.xd - b.xd
        uy = R.yd - b.yd
        kc = (R.x-b.x)/(R.r+b.r)
        ks = (R.y-b.y)/(R.r+b.r)
        vpa = ux*kc+uy*ks
        vpd = ux*ks*uy*kc
        vx = b.cr*(vpa*kc+vpd*ks)
        vy = b.cr*(vpa*ks+vpd*kc)
        b.xd = vx + R.xd
        b.yd = vy + R.yd
    else:
        b.moveball()
    return [b.xd,b.yd]


def collRR(a,b):
    if(colcheck(a,b)==1):
        ux = a.xd - b.xd
        uy = a.yd - b.yd
        kc = (a.x-b.x)/(a.r+b.r)
        ks = (a.y-b.y)/(a.r+b.r)
        vpa = 0
        vpd = ux*ks*uy*kc
        vxb = b.cr(vpa*kc+vpd*ks)
        vyb = b.cr(vpa*ks+vpd*kc)
        b.xd = vxb + a.xd
        b.yd = vyb + a.yd
        a.xd = vyb + a.xd
        a.yd = vxb + a.yd

    return [a.xd,a.yd,b.xd,b.yd]


def walleffect(a):
    [ox,oy] = wallcheck(a)
    if ox == 1 or ox == -1:
        a.xd = -a.xd*a.cr
    if oy == 1 or oy == -1:
        a.yd = -a.yd*a.cr
    return [a.xd,a.yd]


