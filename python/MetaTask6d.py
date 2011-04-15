from dynamic_graph import plug
from dynamic_graph.sot.core import GainAdaptive
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
