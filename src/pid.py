import physics as p

class pid:
    def __init__(self,x,y,angle):
        self.bot = p.robot(x=x,y=y,yaw=angle)
        prod = 0.055
        self.kpp = 0.1
        self.kpv = prod/self.kpp
        self.kpr = 0.02

    def gtg(self,xtg,ytg,bot,thtg=0):
        self.bot = bot
        vrx = (self.kpp * (xtg - self.bot.x))
        vry = (self.kpp * (ytg - self.bot.y))
        kx = (self.kpv * (vrx / 10 - self.bot.xd))
        ky = (self.kpv * (vry / 10 - self.bot.yd))
        if (abs(thtg-bot.theta)>0.1):
            f = (thtg-bot.theta)/abs(thtg-bot.theta)
            e = abs(thtg-bot.theta)
            thetad = (e*self.kpr)*(f)
        else:
            thetad = 0
        bot.movebot(kx=kx,ky=ky,thetad=thetad)

