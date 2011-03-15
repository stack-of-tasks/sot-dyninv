from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *

def toFlags(arr):
    l=max(arr)+1
    lres=[0]*l
    for i in arr:
        lres[i]=1
    lres.reverse()
    res=''
    for i in lres:
        res+=str(i)
    return res

class MetaTask6d(object):
    name=''
    opPoint=''
    dyn=0
    derivator=0
    task=0
    feature=0
    featureDes=0

    def opPointExist(self,opPoint):
        sigsP = filter(lambda x: x.getName().split(':')[-1] == opPoint, self.dyn.signals())
        sigsJ = filter(lambda x: x.getName().split(':')[-1] == 'J'+opPoint, self.dyn.signals())
        return len(sigsP)==1 & len(sigsJ)==1

    def defineDynEntities(self,dyn):
        self.dyn=dyn

    def createOpPoint(self,opPoint,opPointRef = 'right-wrist'):
        self.opPoint=opPoint
        if self.opPointExist(opPoint): return
        self.dyn.createOpPoint(opPoint,opPointRef)

    def createFeatures(self):
        self.feature    = FeaturePoint6d('feature'+self.name)
        self.featureDes = FeaturePoint6d('feature'+self.name+'_ref')
        self.feature.selec.value = '111111'
        self.feature.frame('current')

    def createTask(self):
        self.task = TaskDynPD('task'+self.name)
        self.task.dt.value = 1e-3

    def createGain(self):
        self.gain = GainAdaptive('gain'+self.name)
        self.gain.set(1050,45,125e3)

    def plugEverything(self):
        self.feature.sdes.value = self.featureDes.name
        plug(self.dyn.signal(self.opPoint),self.feature.signal('position'))
        plug(self.dyn.signal('J'+self.opPoint),self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.dyn.velocity,self.task.qdot)
        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)

    def __init__(self,name,dyn,opPoint,opPointRef='right-wrist'):
        self.name=name
        self.defineDynEntities(dyn)
        self.createOpPoint(opPoint,opPointRef)
        self.createFeatures()
        self.createTask()
        self.createGain()
        self.plugEverything()

    @property
    def ref(self):
        return self.featureDes.position.value

    @ref.setter
    def ref(self,m):
        self.featureDes.position.value = m
