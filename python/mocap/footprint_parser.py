'''
Read the file coming from the Oscar footprint extraction, and 
display the position at the proper time interval, using the RF and LF
objects of robotviewer.
'''
from numpy import *


# --- FOOT PRINT PARSER ------------------------------------------------------------
class FootPrintParser:
    def __init__(self,filename,dt,robotviewer,offset=None):
        self.file  = open(filename,'r')
        self.dt=dt
        self.events ={}
        self.clt = robotviewer

        R=eye(3); a=0
        if offset!=None:
            if len(offset)==2:
                R[0:2,2]=offset
            elif len(offset)==3:
                R[0:2,2]=offset[0:2]
                a=offset[2]
                c=cos(a); s=sin(a)
                R[0,0]=c; R[1,1]=c
                R[0,1]=s; R[1,0]=-s
            elif len(offset)==4:
                R[0:2,2]=offset[0:2]
                a=offset[2]; scale=offset[3]
                c=cos(a)*scale; s=sin(a)*scale
                R[0,0]=c; R[1,1]=c
                R[0,1]=s; R[1,0]=-s
        R=matrix(R)

        for l in self.file.readlines():
            (t,foot,x,y,theta)=[ float(x) for x in l.split()]
            if foot==1: foot='LF'
            else: foot='RF'

            if (x,y,theta) == (0,0,0): (x,y,theta)= (-100,-100,0)
            p=R*matrix((x,y,1)).T; theta-=a
            x=float(p[0,0]); y=float(p[1,0])

            t=int(round(t*dt))
            if not t in self.events.keys(): self.events[t] = []
            self.events[t].append( (foot, [x,y,0,0,0,theta] ) )
        self.update(0)    

    def update(self,iterTime):
        if not iterTime in self.events.keys(): return
        for (foot, pos) in self.events[iterTime]:
            self.clt.updateElementConfig(foot,pos)

