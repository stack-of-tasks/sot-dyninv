from dynamic_graph import plug
from dynamic_graph.sot.core import GainAdaptive, OpPointModifier
from dynamic_graph.sot.dyninv import TaskDynPD
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d

class MetaTaskDyn6d(MetaTask6d):
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
    
    def __init__(self,*args):
        MetaTask6d.__init__(self,*args)
        self.opPointModif = OpPointModifier('opmodif'+self.name)
        plug(self.dyn.signal(self.opPoint),self.opPointModif.signal('positionIN'))
        plug(self.dyn.signal('J'+self.opPoint),self.opPointModif.signal('jacobianIN'))
        self.opPointModif.activ = False

    @property
    def opmodif(self):
        if not self.opPointModif.activ: return False
        else: return self.opPointModif.getTransformation()

    @opmodif.setter
    def opmodif(self,m):
        if not self.opPointModif.activ:
            plug(self.opPointModif.signal('position'),self.feature.position )
            plug(self.opPointModif.signal('jacobian'),self.feature.Jq)
            self.opPointModif.activ = True
        self.opPointModif.setTransformation(m)
