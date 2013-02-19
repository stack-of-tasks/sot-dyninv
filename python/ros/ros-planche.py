#_____________________________________________________________________________________________
#********************************************************************************************
#
#  Robot motion (HRP2 14 small) using:
#  - ONLY OPERATIONAL TASKS
#  - Joint limits (position and velocity)
#_____________________________________________________________________________________________
#*********************************************************************************************

import sys
from optparse import OptionParser
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import *
from dynamic_graph.sot.core.utils.attime import attime
from numpy import *
from dynamic_graph.sot.dyninv.robot_specific import postureRange
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.tracer import *

#-----------------------------------------------------------------------------
# --- ROBOT SIMU -------------------------------------------------------------
#-----------------------------------------------------------------------------

# create the robot. Use a RobotDynSimu for the integration.
from dynamic_graph.sot.romeo.romeo import *
robot = Robot('romeo', device=RobotDynSimu('romeo'))
plug(robot.device.state, robot.dynamic.position)

# Binds with ROS. assert that roscore is running.
# if you prefer a ROS free installation, please comment those lines.
# Export joint_state into ros.
from dynamic_graph.ros import *
ros = Ros(robot)

# Alternate visualization tool
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer
addRobotViewer(robot.device,small=True,small_extra=24,verbose=False)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
dt=5e-3
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.device.increment(dt)
    attime.run(robot.device.control.time)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)


#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

robot.dynamic.lowerJl.recompute(0)
robot.dynamic.upperJl.recompute(0)
llimit = matrix(robot.dynamic.lowerJl.value)
ulimit = matrix(robot.dynamic.upperJl.value)
dlimit = ulimit-llimit
mlimit = (ulimit+llimit)/2
llimit[6:18] = mlimit[6:12] - dlimit[6:12]*0.48
robot.dynamic.upperJl.value = vectorToTuple(ulimit)

plug(robot.device.state, robot.dynamic.position)
plug(robot.device.velocity,robot.dynamic.velocity)
robot.dynamic.acceleration.value = robot.dimension*(0.,)

# set the free flyer free.
robot.dynamic.ffposition.unplug()
robot.dynamic.ffvelocity.unplug()
robot.dynamic.ffacceleration.unplug()

robot.dynamic.setProperty('ComputeBackwardDynamics','true')
robot.dynamic.setProperty('ComputeAccelerationCoM','true')

#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)---------------------------------------------
#-----------------------------------------------------------------------------

# --- Op task for the waist ------------------------------
taskWaist = MetaTaskDyn6d('taskWaist',robot.dynamic,'waist','waist')
taskChest = MetaTaskDyn6d('taskChest',robot.dynamic,'chest','chest')
taskHead = MetaTaskDyn6d('taskHead',robot.dynamic,'head','gaze')
taskrh = MetaTaskDyn6d('rh',robot.dynamic,'rh','right-wrist')
tasklh = MetaTaskDyn6d('lh',robot.dynamic,'lh','left-wrist')
taskrf = MetaTaskDyn6d('rf',robot.dynamic,'rf','right-ankle')

for task in [ taskWaist, taskChest, taskHead, taskrh, tasklh, taskrf ]:
    task.feature.frame('current')
    task.gain.setConstant(50)
    task.task.dt.value = dt

# --- TASK COM ------------------------------------------------------
taskCom = MetaTaskDynCom(robot.dynamic,dt)

# --- TASK POSTURE --------------------------------------------------
taskPosture = MetaTaskDynPosture(robot.dynamic,dt)

# --- Task lim ---------------------------------------------------
taskLim = TaskDynLimits('taskLim')
plug(robot.dynamic.position,taskLim.position)
plug(robot.dynamic.velocity,taskLim.velocity)
taskLim.dt.value = dt

robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskLim.referencePosInf.value = robot.dynamic.lowerJl.value
taskLim.referencePosSup.value = robot.dynamic.upperJl.value

dqup = (1000,)*robot.dimension
taskLim.referenceVelInf.value = tuple([-val*pi/180 for val in dqup])
taskLim.referenceVelSup.value = tuple([val*pi/180 for val in dqup])

#-----------------------------------------------------------------------------
# --- SOT Dyn OpSpaceH: SOT controller  --------------------------------------
#-----------------------------------------------------------------------------

sot = SolverDynReduced('sot')
contact = AddContactHelper(sot)

sot.setSize(robot.dimension-6)
#sot.damping.value = 1e-2
sot.breakFactor.value = 10

plug(robot.dynamic.inertiaReal,sot.matrixInertia)
plug(robot.dynamic.dynamicDrift,sot.dyndrift)
plug(robot.dynamic.velocity,sot.velocity)

# replug the control signal
robot.device.control.unplug()
plug(sot.solution, robot.device.control)

#For the integration of q = int(int(qddot)).
plug(sot.acceleration,robot.device.acceleration)

#-----------------------------------------------------------------------------
# ---- CONTACT: Contact definition -------------------------------------------
#-----------------------------------------------------------------------------

# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg', robot.dynamic,'lf','left-ankle')
contactLF.feature.frame('desired')
contactLF.gain.setConstant(1000)
contactLF.name = "LF"

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg', robot.dynamic,'rf','right-ankle')
contactRF.feature.frame('desired')
contactRF.name = "RF"

contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactLF.support = ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))


#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

DEG = 180.0/pi

# Angles for both knee, plus angle for the chest wrt the world.
MAX_EXT = 5/DEG
MAX_SUPPORT_EXT = 25/DEG
CHEST = 80/DEG # 80 ... 90?
WITH_FULL_EXTENTION=True
'''
MAX_EXT = 5/DEG
MAX_SUPPORT_EXT = 25/DEG
CHEST = 70/DEG # 80 ... 90?
WITH_FULL_EXTENTION=False
'''

'''
MAX_EXT = 45/DEG
MAX_SUPPORT_EXT = 45/DEG
CHEST = 50/DEG # 80 ... 90?
WITH_FULL_EXTENTION=False
'''

sot.clear()
contact(contactLF)
contact(contactRF)

taskCom.feature.selec.value = "11"
taskCom.gain.setByPoint(100,10,0.005,0.8)

taskrf.feature.frame('desired')
taskrf.gain.setByPoint(40,2,0.002,0.8)

taskChest.feature.selec.value='111000'
taskChest.ref = rotate('y',CHEST)
taskChest.gain.setByPoint(30,3,1/DEG,0.8)

taskPosture.gain.setByPoint(30,3,1/DEG,0.8)

ql = matrix(robot.dynamic.lowerJl.value)
ql[0,15] = MAX_SUPPORT_EXT
taskLim.referencePosInf.value = vectorToTuple(ql)

#sot.push(taskLim.name)
plug(robot.device.state,sot.position)

q0 = robot.device.state.value
rf0 = matrix(taskrf.feature.position.value)[0:3,3]

MetaTaskDynPosture.postureRange = postureRange['romeo']

# --- Events ---------------------------------------------
# The sequence of tasks.
sigset = ( lambda s,v : s.__class__.value.__set__(s,v) )
refset = ( lambda mt,v : mt.__class__.ref.__set__(mt,v) )

attime(2
       ,(lambda : sot.push(taskCom.task.name),"Add COM")
       ,(lambda : refset(taskCom, ( 0.01, 0.09,  0.7 )), "Com to left foot")
       )

attime(140
       ,(lambda: sot.rmContact("RF"),"Remove RF contact" )
       ,(lambda: sot.push(taskrf.task.name), "Add RF")
       ,(lambda: gotoNd(taskrf, (0,0,0.65),"000110" ), "Up foot RF")
       )

attime(500,lambda: gotoNd(taskrf, (-0.55,0,0.7),"000101" ), "Extend RF")

attime(800,lambda: sot.push(taskChest.task.name), "add chest")

attime(1000
       ,(lambda: sot.rm(taskrf.task.name), "rm RF")
       ,(lambda: sot.push(taskPosture.task.name), "add posture")
       ,(lambda: taskPosture.gotoq(chest=(0,),rleg=(0,0,0,MAX_EXT,0,0),head=(0,0,0,0)), "extend body")
       )

if WITH_FULL_EXTENTION:
    attime(1250
           ,lambda: taskPosture.gotoq(chest=(0,),rleg=(0,0,0,MAX_EXT,0,0),head=(0,0,0,0),rarm=(0,-pi/2,0,0,0,0,0),larm=(0,pi/2,0,0,0,0,0)), "extend arm")

    attime(2080
           ,lambda: taskPosture.gotoq(chest=(0,),rleg=(0,0,0,MAX_EXT,0.73,0),head=(0,-pi/4,0,0),rarm=(0,-pi/2,0,0,0,0,0),larm=(0,pi/2,0,0,0,0,0)), "extend foot")
    tex=1000
else: tex=0

rangeRL = postureRange['romeo']['rleg']
qRL0 = q0[min(rangeRL):max(rangeRL)+1]
rangeRA = postureRange['romeo']['rarm']
qRA0 = q0[min(rangeRA):max(rangeRA)+1]
rangeLA = postureRange['romeo']['larm']
qLA0 = q0[min(rangeLA):max(rangeLA)+1]

attime(1500 + tex
       ,(lambda: sot.rm(taskChest.task.name),"rm chest")
       ,(lambda: taskPosture.gotoq((30,3,1/DEG,0.8),chest=(0,),rleg=qRL0,head=(0,0,0,0),rarm=qRA0,larm=qLA0),"fold arms&leg")
       )

attime(1800+tex
       ,(lambda: sot.rm(taskPosture.task.name),"Remove posture" )
       ,(lambda: sot.push(taskrf.task.name), "Add RF")
       ,(lambda: gotoNd(taskrf, (-0.1,-0.1,0.25),"111" ), "Back RF")
       )

attime(2200+tex  ,lambda: goto6d(taskrf,vectorToTuple(rf0.T+(0,0,0.07)),(80,15,0.005,0.8))  , "RF to upper ground"       )
attime(2400+tex  ,lambda: goto6d(taskrf,vectorToTuple(rf0),(100,15,0.005,0.8))  , "RF to ground"       )
attime(2550+tex  
       ,(lambda: refset(taskCom,(0.01,0,0.65))  , "COM to zero"       )
       ,(lambda: taskCom.gain.setConstant(3)  , "lower com gain"       )
)
attime(2650+tex, lambda: sigset(sot.posture,q0), "Robot to initial pose")
attime(2650+tex  ,(lambda: contact(contactRF)  , "RF to contact"       )
       ,(lambda: sigset(taskCom.feature.selec,"111")  , "COM XYZ"       )
       ,(lambda: taskCom.gain.setByPoint(100,10,0.005,0.8)  , "upper com gain"       )
       )

attime(3200+tex,stop)

# start the motion 
# go()



