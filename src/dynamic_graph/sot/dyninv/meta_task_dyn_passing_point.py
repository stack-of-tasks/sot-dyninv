from dynamic_graph import plug
from dynamic_graph.sot.core import GainAdaptive, OpPointModifier
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, rpy2tr
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d
from dynamic_graph.sot.dyninv import TaskDynPassingPoint

from numpy import matrix, identity, zeros, eye, array, sqrt, radians, arccos, linalg, inner


class MetaTaskDynPassingPoint(MetaTask6d):
    def createTask(self):
        self.task = TaskDynPassingPoint('task'+self.name)
        self.task.dt.value = 1e-3
    def createGain(self):
        pass
    def plugEverything(self):
        self.feature.setReference(self.featureDes.name)
        plug(self.dyn.signal(self.opPoint), self.feature.signal('position'))
        plug(self.dyn.signal('J'+self.opPoint), self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.dyn.velocity, self.task.qdot)
        ''' Dummy initialization '''
        self.task.duration.value = 1
        self.task.velocityDes.value = (0,0,0,0,0,0)
        self.task.initialTime.value = 0
        #self.task.currentTime.value = 0
    
    def __init__(self,*args):
        MetaTask6d.__init__(self,*args)


def goto6dPP(task, position, velocity, duration, current):
    if isinstance(position,matrix): position = vectorToTuple(position)
    if( len(position)==3 ):
        ''' If only position specification '''
        M=eye(4)
        M[0:3,3] = position
    else:
        ''' If there is position and orientation '''
        M = array( rpy2tr(*position[3:7]) )
        M[0:3,3] = position[0:3]
    task.feature.selec.value = "111111"
    task.task.controlSelec.value = "111111"
    task.featureDes.position.value = matrixToTuple(M)
    task.task.duration.value = duration
    task.task.initialTime.value = current
    task.task.velocityDes.value = velocity
    task.task.resetJacobianDerivative()


def gotoNdPP(task, position, velocity, selec, duration, current):
    if isinstance(position,matrix): position = vectorToTuple(position)
    if( len(position)==3 ):
        M=eye(4)
        M[0:3,3] = position
    else:
        M = array( rpy2tr(*position[3:7]) )
        M[0:3,3] = position[0:3]
    if isinstance(selec,str):  
        task.feature.selec.value = selec
        task.task.controlSelec.value = selec
    else:
        task.feature.selec.value = toFlags(selec)
        task.task.controlSelec.value = toFlags(selec)
    task.featureDes.position.value = matrixToTuple(M)
    task.task.duration.value = duration
    task.task.initialTime.value = current
    task.task.velocityDes.value = velocity
    task.task.resetJacobianDerivative()
