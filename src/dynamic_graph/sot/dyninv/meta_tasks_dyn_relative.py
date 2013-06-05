from dynamic_graph import plug
from dynamic_graph.sot.core.meta_tasks import setGain,generic6dReference
from dynamic_graph.sot.core import GainAdaptive, OpPointModifier
from dynamic_graph.sot.dyninv import TaskDynPD
from dynamic_graph.sot.core.meta_tasks_kine_relative import MetaTaskKine6dRel, goto6dRel, gotoNdRel 
from dynamic_graph.sot.core.matrix_util import *

class MetaTaskDyn6dRel(MetaTaskKine6dRel):
	def createTask(self):
		self.task = TaskDynPD('task'+self.name)
		self.task.dt.value = 1e-3
	def createGain(self):
		self.gain = GainAdaptive('gain'+self.name)
		self.gain.set(1050,45,125e3)
	def plugEverything(self):
		self.feature.setReference(self.featureDes.name)
		plug(self.dyn.signal(self.opPoint),self.feature.signal('position'))
		plug(self.dyn.signal('J'+self.opPoint),self.feature.signal('Jq'))
		plug(self.dyn.signal(self.opPointBase),self.feature.signal('positionRef'))
		plug(self.dyn.signal('J'+self.opPointBase),self.feature.signal('JqRef'))
		self.task.add(self.feature.name)
		plug(self.dyn.velocity,self.task.qdot)
		plug(self.task.error,self.gain.error)
		plug(self.gain.gain,self.task.controlGain)
	
	def __init__(self,*args):
		MetaTaskKine6dRel.__init__(self,*args)


	@property
	def opmodifBase(self):
		if not self.opPointModifBase.activ: return False
		else: return self.opPointModifBase.getTransformation()

	@opmodifBase.setter
	def opmodifBase(self,m):
		if isinstance(m,bool) and m==False:
			plug(self.dyn.signal(self.opPointBase),self.feature.signal('positionRef'))
			plug(self.dyn.signal('J'+self.opPointBase),self.feature.signal('JqRef'))
			self.opPointModifBase.activ = False
		else:
			if not self.opPointModifBase.activ:
				plug(self.opPointModifBase.signal('position'),self.feature.positionRef )
				plug(self.opPointModifBase.signal('jacobian'),self.feature.JqRef)
			self.opPointModifBase.setTransformation(m)
			self.opPointModifBase.activ = True





