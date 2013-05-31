# ______________________________________________________________________________
# ******************************************************************************
#
#    BASIC EXAMPLE OF THE DYNAMIC SOT
#       Robot: HRP-2 N.14
#       Tasks: Rotate the head and move the arms
# 
# ______________________________________________________________________________
# ******************************************************************************  

from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph import plug

from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir, modelName, robotDimension, initialConfig, gearRatio, inertiaRotor
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture, AddContactHelper, gotoNd

from numpy import *


# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim   = robotDimension[robotName]
RobotClass = RobotDynSimu
robot      = RobotClass("robot")
robot.resize(robotDim)
robot.set( initialConfig[robotName] )

addRobotViewer(robot,small=True,verbose=False)

dt=5e-3

# ------------------------------------------------------------------------------
# --- MAIN LOOP ----------------------------------------------------------------
# ------------------------------------------------------------------------------

def inc():
    robot.increment(dt)
    attime.run(robot.control.time)
    # verif.record()

@loopInThread
def loop():
    inc()
runner=loop()


# --- shortcuts -------------------------------------------------
@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc()
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay


#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

modelDir          = pkgDataRootDir[robotName]
xmlDir            = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath     = xmlDir + '/HRP2LinkJointRankSmall.xml'

dyn = Dynamic("dyn")
dyn.setFiles(modelDir,modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value    = gearRatio[robotName]

plug(robot.state,dyn.position)
plug(robot.velocity,dyn.velocity)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()


#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)-----------------------------------------
#-----------------------------------------------------------------------------

taskWaist = MetaTaskDyn6d('taskWaist', dyn, 'waist', 'waist')
taskChest = MetaTaskDyn6d('taskChest', dyn, 'chest', 'chest')
taskHead  = MetaTaskDyn6d('taskHead', dyn, 'head', 'gaze')
taskrh    = MetaTaskDyn6d('rh', dyn, 'rh', 'right-wrist')
tasklh    = MetaTaskDyn6d('lh', dyn, 'lh', 'left-wrist')

for task in [ taskWaist, taskChest, taskHead, taskrh, tasklh]:
    task.feature.frame('current')
    task.gain.setConstant(50)
    task.task.dt.value = dt
    task.featureDes.velocity.value=(0,0,0,0,0,0)

# CoM Task
taskCom = MetaTaskDynCom(dyn,dt)

# Posture Task
taskPosture = MetaTaskDynPosture(dyn,dt)

# Angular position and velocity limits
taskLim = TaskDynLimits('taskLim')
plug(dyn.position,taskLim.position)
plug(dyn.velocity,taskLim.velocity)
taskLim.dt.value = dt

dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskLim.referencePosInf.value = dyn.lowerJl.value
taskLim.referencePosSup.value = dyn.upperJl.value

#dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240, 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
dqup = (1000,)*robotDim
taskLim.referenceVelInf.value = tuple([-val*pi/180 for val in dqup])
taskLim.referenceVelSup.value = tuple([ val*pi/180 for val in dqup])


#-----------------------------------------------------------------------------
# --- Stack of tasks controller  ---------------------------------------------
#-----------------------------------------------------------------------------

sot     = SolverDynReduced('sot')
contact = AddContactHelper(sot)

sot.setSize(robotDim-6)
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

plug(sot.solution, robot.control)
plug(sot.acceleration,robot.acceleration)


#-----------------------------------------------------------------------------
# ---- CONTACT: Contact definition -------------------------------------------
#-----------------------------------------------------------------------------

# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.featureDes.velocity.value=(0,0,0,0,0,0)
contactLF.feature.frame('desired')
contactLF.name = "LF"

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.featureDes.velocity.value=(0,0,0,0,0,0)
contactRF.feature.frame('desired')
contactRF.name = "RF"

contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))

# Imposed erordot = 0
contactLF.feature.errordot.value=(0,0,0,0,0,0)
contactRF.feature.errordot.value=(0,0,0,0,0,0)



#-----------------------------------------------------------------------------
# --- TRACE ------------------------------------------------------------------
#-----------------------------------------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','dyn_','.dat')

tr.add('sot.acceleration','')
tr.add('sot.torque','')
tr.add('sot.reducedControl','')
tr.add('sot.reducedForce','')
tr.add('sot.forces','')
tr.add('sot.forcesNormal','')
tr.add('sot.reducedForce','')
tr.start()

robot.after.addSignal('sot.reducedForce')
robot.after.addSignal('sot.forces')
robot.after.addSignal('sot.torque')

robot.after.addSignal('tr.triger')
robot.after.addSignal(contactLF.task.name+'.error')
robot.after.addSignal('dyn.rf')
robot.after.addSignal('dyn.lf')
robot.after.addSignal('dyn.chest')
robot.after.addSignal('dyn.lh')
robot.after.addSignal('dyn.com')
robot.after.addSignal('sot.forcesNormal')
robot.after.addSignal('dyn.waist')
robot.after.addSignal('taskLim.normalizedPosition')


#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------


dyn.lh.recompute(0)
dyn.rh.recompute(0)
lh0 = tuple([x[3] for x in dyn.lh.value])
rh0 = tuple([x[3] for x in dyn.rh.value])
r = radians

sot.clear()
contact(contactLF)
contact(contactRF)

sot.push(taskLim.name)
plug(robot.state,sot.position)

attime(2
       ,(lambda: sot.push(taskChest.task.name), "Add Chest to the SoT")
       ,(lambda: taskChest.feature.keep(), "Keep Chest")
       ,(lambda: sot.push(taskHead.task.name), "Add Head to the SoT")
       ,(lambda: gotoNd(taskHead, (0, 0, 0, 0, r(25), -r(40)), "111000"), "Rotate Head to the right")
       ,(lambda: sot.push(taskrh.task.name), "Add Right hand to the SoT")
       ,(lambda: gotoNd(taskrh, (rh0[0]-0.3, rh0[1]-0.4, 0), "000011"), "Move right hand")
       )

attime(200
       ,(lambda: gotoNd( taskHead, (0, 0, 0, 0, r(25), r(40)), "111000"), "Rotate Head to the left")
       ,(lambda: sot.push(tasklh.task.name), "Add Left hand to the SoT")
       ,(lambda: gotoNd(tasklh, (lh0[0]+0.3, lh0[1]+0.4, 0), "000011"), "Move left hand")
       )

attime(400
       ,(lambda: gotoNd(taskHead, (0, 0, 0, 0, 0, 0), "111000"), "Rotate Head to the center")
       ,(lambda: gotoNd(tasklh, (lh0[0], lh0[1], 0), "000011"),  "Return left hand to initial position")
       ,(lambda: gotoNd(taskrh, (rh0[0], rh0[1], 0), "000011"),  "Return right hand to initial position")
       )

attime(600, stop, "Stopped")

