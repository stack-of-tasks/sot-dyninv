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
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d

from numpy import *


# Create the robot romeo.
from dynamic_graph.sot.romeo.romeo import *
robot = Robot('romeo', device=RobotDynSimu('romeo'))
plug(robot.device.state, robot.dynamic.position)

# Binds with ROS. assert that roscore is running.
# if you prefer a ROS free installation, please comment those lines.
from dynamic_graph.ros import *
ros = Ros(robot)

# Alternate visualization tool
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer
addRobotViewer(robot.device,small=True,small_extra=24,verbose=False)

# --- ROBOT SIMU ---------------------------------------------------------------
dt=5e-3


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# --- DYN ----------------------------------------------------------------------
plug(robot.device.state,   robot.dynamic.position)
plug(robot.device.velocity,robot.dynamic.velocity)
robot.dynamic.acceleration.value = robot.dimension*(0.,)

#unplug the waist: it is not fixed in the universe anymore.
robot.dynamic.ffposition.unplug()
robot.dynamic.ffvelocity.unplug()
robot.dynamic.ffacceleration.unplug()

robot.dynamic.setProperty('ComputeBackwardDynamics','true')
robot.dynamic.setProperty('ComputeAccelerationCoM','true')

robot.device.control.unplug()

# --- Task Dyn -----------------------------------------
# Task right hand
task=MetaTaskDyn6d('rh',robot.dynamic,'task')
task.ref = ((0,0,-1,0.2),(0,1,0,-0.2),(1,0,0,1.00),(0,0,0,1))

# Task LFoot: Move the left foot up.
taskLF=MetaTaskDyn6d('lf',robot.dynamic,'lf','left-ankle')
taskLF.ref = ((1,0,0,0.0),(0,1,0,+0.29),(0,0,1,.15),(0,0,0,1))

# Task RFoot: Move the right foot up.
taskRF=MetaTaskDyn6d('rf',robot.dynamic,'rf','right-ankle')
taskRF.ref = ((1,0,0, 0.0102),(0,1,0,-0.096),(0,0,1,0.1),(0,0,0,1))

# --- TASK COM ------------------------------------------------------
robot.dynamic.setProperty('ComputeCoM','true')

featureCom    = FeatureGeneric('featureCom')
featureComDes = FeatureGeneric('featureComDes')
plug(robot.dynamic.com,featureCom.errorIN)
plug(robot.dynamic.Jcom,featureCom.jacobianIN)
featureCom.setReference('featureComDes')
featureComDes.errorIN.value = (0.0478408688115,-0.0620357207995,0.684865189311)

taskCom = TaskDynPD('taskCom')
taskCom.add('featureCom')
plug(robot.dynamic.velocity,taskCom.qdot)
taskCom.dt.value = 1e-3

gCom = GainAdaptive('gCom')
plug(taskCom.error,gCom.error)
plug(gCom.gain,taskCom.controlGain)
gCom.set(1050,45,125e3)

# ---- CONTACT -----------------------------------------
# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',robot.dynamic,'lf','left-ankle')
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.feature.frame('desired')

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',robot.dynamic,'rf','right-ankle')
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactRF.feature.frame('desired')

# --- SOT Dyn OpSpaceH --------------------------------------
# SOT controller.
sot = SolverOpSpace('sot')
sot.setSize(robot.dimension-6)
#sot.damping.value = 2e-2
sot.breakFactor.value = 10

# plug the dynamic entity to the sot
plug(robot.dynamic.inertiaReal,sot.matrixInertia)
plug(robot.dynamic.dynamicDrift,sot.dyndrift)
plug(robot.dynamic.velocity,sot.velocity)

plug(sot.control,robot.device.control)
# For the integration of q = int(int(qddot)).
plug(sot.acceleration,robot.device.acceleration)

# --- RUN ------------------------------------------------
#  changes the posture of romeo while asserting that it keeps its balance
featureComDes.errorIN.value = (0.05,  0.1,   0.675)

sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = contactLF.support
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = contactRF.support

# add the com task.
# sot.push('taskCom')

# add a task for the left hand.
# sot.push('taskrh')

# Start the simulation
# go

