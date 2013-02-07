# ______________________________________________________________________________
# ******************************************************************************
# 
# ______________________________________________________________________________
# ******************************************************************************

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor,specificitiesName,jointRankName

from numpy import *

# --- ROBOT SIMU ---------------------------------------------------------------
robotName = 'romeo'
robotDim=robotDimension[robotName]
robot = RobotDynSimu("romeo")
robot.resize(robotDim)
dt=5e-3

robot.set( initialConfig[robotName] )
addRobotViewer(robot,small=True,small_extra=24,verbose=False)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
qs=[]
@loopInThread
def inc():
    robot.increment(dt)
    qs.append(robot.state.value)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# --- shortcuts -------------------------------------------------
@optionalparentheses
def q():
    if 'dyn' in globals(): print dyn.ffposition.__repr__()
    print robot.state.__repr__()
@optionalparentheses
def qdot(): print robot.control.__repr__()
@optionalparentheses
def t(): print robot.state.time-1
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay

# --- DYN ----------------------------------------------------------------------
modelDir = pkgDataRootDir[robotName]
xmlDir = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/' + specificitiesName[robotName]
jointRankPath = xmlDir + '/' + jointRankName[robotName]

dyn = Dynamic("dyn")
dyn.setFiles(modelDir,modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value = gearRatio[robotName]

plug(robot.state,dyn.position)
plug(robot.velocity,dyn.velocity)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# --- Task Dyn -----------------------------------------
# Task right hand
task=MetaTaskDyn6d('rh',dyn,'task')
task.ref = ((0,0,-1,0.22),(0,1,0,-0.37),(1,0,0,.74),(0,0,0,1))

# Task LFoot: Move the right foot up.
taskLF=MetaTaskDyn6d('lf',dyn,'lf','left-ankle')
taskLF.ref = ((1,0,0,0.0),(0,1,0,+0.29),(0,0,1,.15),(0,0,0,1))

# --- TASK COM ------------------------------------------------------
dyn.setProperty('ComputeCoM','true')

featureCom    = FeatureGeneric('featureCom')
featureComDes = FeatureGeneric('featureComDes')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureCom.setReference('featureComDes')
featureComDes.errorIN.value = (0.0478408688115,-0.0620357207995,0.684865189311)

taskCom = TaskDynPD('taskCom')
taskCom.add('featureCom')
plug(dyn.velocity,taskCom.qdot)
taskCom.dt.value = 1e-3

gCom = GainAdaptive('gCom')
plug(taskCom.error,gCom.error)
plug(gCom.gain,taskCom.controlGain)
# Work from .04 to .09 gCom.set 1050 45 125e3
# Good behavior gCom.set 1080 15 125e3
# Low gains gCom.set 400 1 39e3
# Current choice
gCom.set(1050,45,125e3)

# ---- CONTACT -----------------------------------------
# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.feature.frame('desired')

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactRF.feature.frame('desired')

# --- SOT Dyn OpSpaceH --------------------------------------
# SOT controller.
sot = SolverOpSpace('sot')
sot.setSize(robotDim-6)
#sot.damping.value = 2e-2
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

plug(sot.control,robot.control)
# For the integration of q = int(int(qddot)).
plug(sot.acceleration,robot.acceleration)

# --- RUN ------------------------------------------------
#  changes the posture of romeo while asserting that it keeps its balance
featureComDes.errorIN.value = (0.05,  0.1,   0.675)

sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = contactLF.support
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = contactRF.support
sot.push('taskCom')

# go

