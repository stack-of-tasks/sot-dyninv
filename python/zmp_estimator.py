from dynamic_graph.sot.dyninv import ZmpEstimator
from numpy import *

def computeZmp(sot,dyn):
    p=zeros((4,0))
    f=zeros((0,1))
    if '_RF_p' in [s.name for s in sot.signals()]:
        Mr=matrix(dyn.rf.value)
        fr=matrix(sot._RF_fn.value).transpose()
        pr=matrix(sot._RF_p.value+((1,1,1,1),))
        p=hstack((p,Mr*pr))
        f=vstack((f,fr))
    if '_LF_p' in [s.name for s in sot.signals()]:
        Ml=matrix(dyn.lf.value)
        fl=matrix(sot._LF_fn.value).transpose()
        pl=matrix(sot._LF_p.value+((1,1,1,1),))
        p=hstack((p,Ml*pl))
        f=vstack((f,fl))
    zmp=p*f/sum(f)
    return zmp
def zmpup(zmp):
    if zmp.sot == None and zmp.warnSot:
        zmp.warnSot = False
        print 'Declare solver for zmp update'
    else:
        zmp.zmp.value = tuple(computeZmp(zmp.sot,zmp.dyn)[0:3].transpose().tolist()[0])
        zmp.zmp.time = zmp.sot.solution.time
def zmpDeclare(zmp,sot,dyn):
    zmp.sot = sot
    zmp.dyn = dyn

ZmpEstimator.refresh = zmpup
ZmpEstimator.warnSot = True
ZmpEstimator.declare = zmpDeclare




def leftSupport(sot,dyn):
    if '_LF_p' in [s.name for s in sot.signals()]:
        Ml=matrix(dyn.lf.value)
        pl=matrix(sot._LF_p.value+((1,1,1,1),))
        return  (Ml*pl)[0:3,:]
def rightSupport(sot,dyn):
    if '_RF_p' in [s.name for s in sot.signals()]:
        Mr=matrix(dyn.rf.value)
        pr=matrix(sot._RF_p.value+((1,1,1,1),))
        return (Mr*pr)[0:3,:]

ZmpEstimator.rightSupport = lambda x: rightSupport(x.sot,x.dyn)
ZmpEstimator.leftSupport = lambda x: leftSupport(x.sot,x.dyn)
