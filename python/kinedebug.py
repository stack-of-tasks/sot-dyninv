from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from MetaTask6d import MetaTask6d

# --- Dynamic parameters ---
hrp2_14_pkgdatarootdir = "/home/nmansard/compil/devgiri/hpp2/share/hrp2_14"
modelDir = hrp2_14_pkgdatarootdir
xmlDir = hrp2_14_pkgdatarootdir
modelName = 'HRP2JRLmainsmall.wrl'
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath = xmlDir + '/HRP2LinkJointRankSmall.xml'
robotDim=36

from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)



# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

robot = RobotSimu("robot")
robot.resize(robotDim)
#robot.set( (0,0,0,0,0,0, -0.0091163033083694940822,-0.091409963793829082657,-0.47197874328281280709,0.84038019261713603481,-0.47023279905304871118,0.089662407859063653071,0.0095078180225693850747,0.091110286804129178573,-0.4694503518480188653,0.83530799538565336793,-0.46768619090392338222,-0.093802946636709280681,-8.7564658296357018321e-05,0.0032643187737393364843,-7.8338082008638037324e-06,0.00019474257801330629915,0.25837025731361307201,-0.17509910214200960499,-6.1173425555032122825e-05,-0.52495354876847799552,3.1825099712999219606e-06,-0.00025760004742291479777,-3.4121048192112107608e-06,0.25836727162795458668,0.17432209754621175168,-8.8902395499734237117e-05,-0.52498369084585783106,-3.4610294148795152394e-07,-0.00026540143977199129972,1.004984814529256132e-06) )
robot.set( ( 0.0274106623863, 0.143843868989, 0.646921914726, 0.00221064938462, 0.101393756965, 1.36729741242e-05, -0.00911630330837, -0.0914099637938, -0.471978743283, 0.840380192617, -0.470232799053, 0.0896624078591, 0.00950781802257, 0.0911102868041, -0.469450351848, 0.835307995386, -0.467686190904, -0.0938029466367, -8.75646582964e-05, 0.00326431877374, -7.83380820086e-06, 0.000194742578013, 0.258370257314, -0.175099102142, -6.1173425555e-05, -0.524953548768, 3.1825099713e-06, -0.000257600047423, -3.41210481921e-06, 0.258367271628, 0.174322097546, -8.89023954997e-05, -0.524983690846, -3.46102941488e-07, -0.000265401439772, 1.00498481453e-06 ) )

dt=5e-3

# --- VIEWER -------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
try:
    import robotviewer

    def stateFullSize(robot):
        simulation_angles = [float(val) for val in robot.state.value]
        simulation_angles += 10*[0.0]
        return simulation_angles
    RobotSimu.stateFullSize = stateFullSize

    robot.viewer = robotviewer.client('XML-RPC')
    robot.viewer.updateElementConfig('hrp',robot.stateFullSize())

    def incrementView( robot,dt ):
        robot.incrementNoView(dt)
        robot.viewer.updateElementConfig('hrp',robot.stateFullSize())
    RobotSimu.incrementNoView = RobotSimu.increment
    RobotSimu.increment = incrementView
except:
    print "No robot viewer, sorry."
    robot.viewer = None

# --- MAIN LOOP ------------------------------------------
def inc():
    robot.increment(dt)
    tr.triger.recompute( robot.control.time )
from ThreadInterruptibleLoop import *
@loopInThread
def loop():
    inc()
runner=loop()

@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): runner.once()

# --- DYN ----------------------------------------------------------------------
# --- DYN ----------------------------------------------------------------------
# --- DYN ----------------------------------------------------------------------

dyn = Dynamic("dyn")
dyn.setFiles(modelDir, modelName,specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = (0,0,0,0,0,0,1.01e-4,6.96e-4,1.34e-4,1.34e-4,6.96e-4,6.96e-4,1.01e-4,6.96e-4,1.34e-4,1.34e-4,6.96e-4,6.96e-4,6.96e-4,6.96e-4,1.10e-4,1.10e-4,6.96e-4,6.60e-4,1.00e-4,6.60e-4,1.10e-4,1.00e-4,1.00e-4,6.96e-4,6.60e-4,1.00e-4,6.60e-4,1.10e-4,1.00e-4,1.00e-4)
dyn.gearRatio.value = (0,0,0,0,0,0,384.0,240.0,180.0,200.0,180.0,100.0,384.0,240.0,180.0,200.0,180.0,100.0,207.69,381.54,100.0,100.0,219.23,231.25,266.67,250.0,145.45,350.0,200.0,219.23,231.25,266.67,250.0,145.45,350.0,200.0)

plug(robot.state,dyn.position)
dyn.velocity.value = robotDim*(0.,)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

#dyn.setProperty('ComputeBackwardDynamics','true')
#dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# --- Task Dyn -----------------------------------------
class MetaTaskKine6d( MetaTask6d ):
    def createTask(self):
        self.task = Task('task'+self.name)

    def createGain(self):
        self.gain = GainAdaptive('gain'+self.name)
        self.gain.set(0.1,0.1,125e3)
    def plugEverything(self):
        self.feature.sdes.value = self.featureDes.name
        plug(self.dyn.signal(self.opPoint),self.feature.signal('position'))
        plug(self.dyn.signal('J'+self.opPoint),self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)
    def keep(self):
        self.feature.position.recompute(self.dyn.position.time)
        self.feature.keep()

# Task right hand
taskRH=MetaTaskKine6d('rh',dyn,'rh','right-wrist')
task.ref = ((0,0,-1,0.22),(0,1,0,-0.37),(1,0,0,.74),(0,0,0,1))
taskLH=MetaTaskKine6d('lh',dyn,'lh','left-wrist')
task.ref = ((0,0,-1,0.22),(0,1,0,0.37),(1,0,0,.74),(0,0,0,1))

# Task LFoot: Move the right foot up.
taskRF=MetaTaskKine6d('rf',dyn,'rf','right-ankle')
taskLF=MetaTaskKine6d('lf',dyn,'lf','left-ankle')

# --- TASK COM ------------------------------------------------------
dyn.setProperty('ComputeCoM','true')

featureCom    = FeatureGeneric('featureCom')
featureComDes = FeatureGeneric('featureComDes')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureCom.sdes.value = 'featureComDes'
featureComDes.errorIN.value = (0.0478408688115,-0.0620357207995,0.684865189311)

taskCom = Task('taskCom')
taskCom.add('featureCom')

gCom = GainAdaptive('gCom')
plug(taskCom.error,gCom.error)
plug(gCom.gain,taskCom.controlGain)
gCom.set(1,1,1)

# --- TASK POSTURE --------------------------------------------------
featurePosture    = FeatureGeneric('featurePosture')
featurePostureDes = FeatureGeneric('featurePostureDes')
plug(dyn.position,featurePosture.errorIN)
featurePosture.sdes.value = 'featurePostureDes'
featurePosture.jacobianIN.value = totuple( identity(robotDim) )
featurePostureDes.errorIN.value = dyn.position.value

taskPosture = Task('taskPosture')
taskPosture.add('featurePosture')

gPosture = GainAdaptive('gPosture')
plug(taskPosture.error,gPosture.error)
plug(gPosture.gain,taskPosture.controlGain)
gPosture.set(1,1,1)

featurePosture.selec.value = 0
featurePosture.selec.value = '|0:3'     # right leg
featurePosture.selec.value = '|6:9'     # left leg
featurePosture.selec.value = '|12:15'   # chest+head
featurePosture.selec.value = '|16:19'   # right arm
featurePosture.selec.value = '|23:26'   # left arm

# --- TASK JL ------------------------------------------------------
taskJL = TaskJointLimits('taskJL')
plug(dyn.position,taskJL.position)
plug(dyn.lowerJl,taskJL.referenceInf)
plug(dyn.upperJl,taskJL.referenceSup)
taskJL.dt.value = dt
taskJL.selec.value = '|6:'

# --- SOT Dyn OpSpaceH --------------------------------------
# SOT controller.
sot = SolverKine('sot')
sot.setSize(36)
#sot.damping.value = 2e-2

plug(sot.control,robot.control)

def sot_addContact( sot,metaTask ):
    metaTask.keep()
    sot.push(metaTask.task.name)
import new
sot.addContact = new.instancemethod(sot_addContact, sot, sot.__class__)


# --- TRACE ----------------------------------------------

from dynamic_graph.tracer import *
from dynamic_graph.tracer_real_time import *
tr = TracerRealTime('tr')
tr.setBufferSize(10485760)
tr.open('/tmp/','dyn_','.dat')
tr.start()

#robot.periodicCall addSignal tr.triger

#tr.add('p6.error','position')
tr.add('featureCom.error','comerror')
tr.add('dyn.com','com')
tr.add('dyn.position','state')
# tr.add('gCom.gain','')
# tr.add('gCom.error','gerror')

tr.add('sot.control','')

# --- RUN ------------------------------------------------
featureComDes.errorIN.value = (0.06,0,0.8)
sot.push(taskJL.name)
sot.addContact(taskLF)
sot.addContact(taskRF)
sot.push(taskCom.name)

#inc()
tr.add('taskJL.normalizedPosition','qn')


robot.after.addSignal('taskJL.normalizedPosition')

qn=taskJL.normalizedPosition
q=taskJL.position
comref=featureComDes.errorIN

sot.damping.value=.1

comref.value=(0,0,0.45)
