from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple, rotate, matrixToRPY, rpy2tr
from numpy import matrix, identity, zeros, eye, array, pi


def setGain(gain,val):
    if val!=None:
        if isinstance(val,int) or isinstance(val,float):
            gain.setConstant(val)
        if len(val)==1:
            gain.setConstant(val[0])
        elif len(val)==3: gain.set( val[0],val[1],val[2])
        elif len(val)==4: gain.setByPoint( val[0],val[1],val[2],val[3])

def goto6d(task,position,gain=None):
    M=eye(4)
    if isinstance(position,matrix): position = vectorToTuple(position)
    if( len(position)==3 ): M[0:3,3] = position
    else:
        #print "Position 6D with rotation ... todo" # done?
        M = array( rpy2tr(*position[3:7]) )
        M[0:3,3] = position[0:3]
    task.feature.selec.value = "111111"
    setGain(task.gain,gain)
    task.featureDes.position.value = matrixToTuple(M)
    task.task.resetJacobianDerivative()

def gotoNd(task,position,selec,gain=None,resetJacobian=True):
    M=eye(4)
    if isinstance(position,matrix): position = vectorToTuple(position)
    if( len(position)==3 ): M[0:3,3] = position
    else:
        #print "Position 6D with rotation ... todo" # done?
        M = array( rpy2tr(*position[3:7]) )
        M[0:3,3] = position[0:3]
    if isinstance(selec,str):   task.feature.selec.value = selec
    else: task.feature.selec.value = toFlags(selec)
    task.featureDes.position.value = matrixToTuple(M)
    if resetJacobian: task.task.resetJacobianDerivative()
    setGain(task.gain,gain)

def addContactFromMetaTask(sot,contact,task=None):
    sot.addContactFromTask(contact.task.name,contact.name)
    sot.signal("_"+contact.name+"_p").value = contact.support
    if task!= None: sot.rm(task.task.name)
    contact.task.resetJacobianDerivative()
def addContactMethod( solverClass ):
    solverClass.addContactFromMetaTask = addContactFromMetaTask

class AddContactHelper:
    def __init__(self,sot):
        addContactMethod(sot.__class__)
        self.sot=sot
    def __call__(self,*a): self.sot.addContactFromMetaTask(*a)



class MetaTaskDynPosture(object):
    postureRange = { \
        "rleg": range(6,12), \
        "lleg": range(12,18), \
        "chest": range(18,20), \
        "head": range(20,22), \
        "rarm": range(22,28), \
        "rhand": [28], \
        "larm": range(29,35), \
        "lhand": [35], \
            }
    nbDof = None
    def __init__(self,dyn,dt,name="posture"):
        self.dyn=dyn
        self.name=name

        self.feature    = FeatureGeneric('feature'+name)
        self.featureDes = FeatureGeneric('featureDes'+name)
        self.task = TaskDynPD('task'+name)
        self.gain = GainAdaptive('gain'+name)

        plug(dyn.position,self.feature.errorIN)
        robotDim = len(dyn.position.value)
        self.feature.jacobianIN.value = matrixToTuple( identity(robotDim) )
        self.feature.setReference(self.featureDes.name)

        self.task.add(self.feature.name)
        plug(dyn.velocity,self.task.qdot)
        self.task.dt.value = dt

        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)


    @property
    def ref(self):
        return self.featureDes.errorIN.value

    @ref.setter
    def ref(self,v):
        self.featureDes.errorIN.value = v

    def gotoq(self,gain=None,**kwargs):
        act=list()
        if MetaTaskDynPosture.nbDof==None:
            MetaTaskDynPosture.nbDof = len(self.feature.errorIN.value)
        qdes = zeros((MetaTaskDynPosture.nbDof,1))
        for n,v in kwargs.items():
            r = self.postureRange[n]
            act += r
            if isinstance(v,matrix): qdes[r,0] = vectorToTuple(v)
            else: qdes[r,0] = v
        self.ref = vectorToTuple(qdes)
        self.feature.selec.value = toFlags(act)
        setGain(self.gain,gain)


class MetaTaskDynCom(object):
    def __init__(self,dyn,dt,name="com"):
        self.dyn=dyn
        self.name=name
        dyn.setProperty('ComputeCoM','true')

        self.feature    = FeatureGeneric('feature'+name)
        self.featureDes = FeatureGeneric('featureDes'+name)
        self.task = TaskDynPD('task'+name)
        self.gain = GainAdaptive('gain'+name)

        plug(dyn.com,self.feature.errorIN)
        plug(dyn.Jcom,self.feature.jacobianIN)
        self.feature.setReference(self.featureDes.name)

        self.task.add(self.feature.name)
        plug(dyn.velocity,self.task.qdot)
        self.task.dt.value = dt

        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)

    @property
    def ref(self):
        return self.featureDes.errorIN.value

    @ref.setter
    def ref(self,v):
        self.featureDes.errorIN.value = v


class MetaTaskDynLimits(object):
    def __init__(self, dyn, dt, name="lim"):
        self.dyn  = dyn
        self.name = name

        self.task = TaskDynLimits('task'+name)
        plug(dyn.position, self.task.position)
        plug(dyn.velocity, self.task.velocity)

        self.task.dt.value = dt
        self.task.controlGain.value = 10.0

        dyn.upperJl.recompute(0)
        dyn.lowerJl.recompute(0)
        self.task.referencePosInf.value = dyn.lowerJl.value
        self.task.referencePosSup.value = dyn.upperJl.value

        dqup = (0, 0, 0, 0, 0, 0, 
                200, 220, 250, 230, 290, 520,
                200, 220, 250, 230, 290, 520, 
                250, 140, 390, 390, 
                240, 140, 240, 130, 270, 180, 330, 
                240, 140, 240, 130, 270, 180, 330)
        dqup = (1000,)*len(dyn.upperJl.value)
        self.task.referenceVelInf.value = tuple([-val*pi/180 for val in dqup])
        self.task.referenceVelSup.value = tuple([ val*pi/180 for val in dqup])



