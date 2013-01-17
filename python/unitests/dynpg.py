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
sys.path.append('..')
from dynamic_graph.sot.core.meta_task_6d import toFlags
from meta_task_dyn_6d import MetaTaskDyn6d
from meta_tasks_dyn import *
from attime import attime
from numpy import *
from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
from matrix_util import matrixToTuple, vectorToTuple,rotate
from history import History
from zmp_estimator import ZmpEstimator
from viewer_helper import addRobotViewer,VisualPinger

#-----------------------------------------------------------------------------
# --- ROBOT SIMU -------------------------------------------------------------
#-----------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim  = robotDimension[robotName]
RobotClass = RobotDynSimu
robot      = RobotClass("robot")
robot.resize(robotDim)
addRobotViewer(robot,small=True,verbose=False)
dt = 5e-3

# Similar initial position with hand forward
initialConfig['hrp14small'] = (0,0,0.648697398115,0,0,0)+(0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, 0, 0, 0.2618, -0.1745, 0, -0.5236, 0, 0, 0, 0.2618, 0.1745, 0, -0.5236, 0, 0,  0 )

robot.set(initialConfig[robotName])

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

def inc():
    robot.increment(dt)
    # Execute a function at time t, if specified with t.add(...)
    if 'refresh' in ZmpEstimator.__dict__: zmp.refresh()
    attime.run(robot.control.time)
    robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
    if dyn.com.time >0:
        robot.viewer.updateElementConfig('com',[dyn.com.value[0],dyn.com.value[1],0,0,0,0])
    history.record()

from ThreadInterruptibleLoop import *
@loopInThread
def loop():
#    try:
        inc()
#    except:
#        print robot.state.time,': -- Robot has stopped --'
runner=loop()

@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc()

# --- shortcuts -------------------------------------------------
@optionalparentheses
def q():
    print robot.state.__repr__()
@optionalparentheses
def qdot(): print robot.control.__repr__()
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay

@optionalparentheses
def pl():         print  matlab(  matrixToTuple(zmp.leftSupport()) ).resstr
@optionalparentheses
def pr():         print  matlab(  matrixToTuple(zmp.rightSupport()) ).resstr

@optionalparentheses
def dump():    history.dumpToOpenHRP('openhrp/screenwash')

#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

modelDir  = pkgDataRootDir[robotName]
xmlDir    = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath     = xmlDir + '/HRP2LinkJointRankSmall.xml'

dyn = Dynamic("dyn")
dyn.setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.lowerJl.recompute(0)
dyn.upperJl.recompute(0)
llimit = matrix(dyn.lowerJl.value)
ulimit = matrix(dyn.upperJl.value)
dlimit = ulimit-llimit
mlimit = (ulimit+llimit)/2
llimit[6:18] = mlimit[6:12] - dlimit[6:12]*0.48
dyn.upperJl.value = vectorToTuple(ulimit)

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

# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator import *

pg = PatternGenerator('pg')
pg.setVrmlDir(modelDir+'/')
pg.setVrml(modelName[robotName])
pg.setXmlSpec(specificitiesPath)
pg.setXmlRank(jointRankPath)
pg.buildModel()

# Standard initialization
pg.parseCmd(":samplingperiod 0.005")
pg.parseCmd(":previewcontroltime 1.6")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":omega 0.0")
pg.parseCmd(":stepheight 0.05")
pg.parseCmd(":singlesupporttime 0.780")
pg.parseCmd(":doublesupporttime 0.020")
pg.parseCmd(":armparameters 0.5")
pg.parseCmd(":LimitsFeasibility 0.0")
pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

plug(dyn.position,pg.position)
plug(dyn.com,pg.com)
pg.motorcontrol.value = robotDim*(0,)
pg.zmppreviouscontroller.value = (0,0,0)

pg.initState()

# --- REFERENCES ---------------------------------------------------------------
# --- Selector of Com Ref: when pg is stopped, pg.inprocess becomes 0
comRef = Selector('comRef'
                  ,['vector','ref',dyn.com,pg.comref])
plug(pg.inprocess,comRef.selec)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.parseCmd(':SetAlgoForZmpTrajectory Herdt')
pg.parseCmd(':doublesupporttime 0.1')
pg.parseCmd(':singlesupporttime 0.7')
# When velocity reference is at zero, the robot stops all motion after n steps
pg.parseCmd(':numberstepsbeforestop 4')
# Set constraints on XY
pg.parseCmd(':setfeetconstraint XY 0.09 0.06')
# Start the robot with a speed of 0.1 m/0.8 s.
pg.parseCmd(':HerdtOnline 0.1 0.0 0.0')
# You can now modifiy the speed of the robot using set pg.velocitydes [3]( x, y, yaw)
pg.velocitydes.value =(0.1,0.0,0.0)

#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)---------------------------------------------
#-----------------------------------------------------------------------------

# --- Op task for the waist ------------------------------
taskWaist = MetaTaskDyn6d('taskWaist',dyn,'waist','waist')
taskChest = MetaTaskDyn6d('taskChest',dyn,'chest','chest')
taskHead = MetaTaskDyn6d('taskHead',dyn,'head','gaze')
taskRH = MetaTaskDyn6d('rh',dyn,'rh','right-wrist')
taskLH = MetaTaskDyn6d('lh',dyn,'lh','left-wrist')
taskRF = MetaTaskDyn6d('rf',dyn,'rf','right-ankle')
taskLF = MetaTaskDyn6d('lf',dyn,'lf','left-ankle')

for task in [ taskWaist, taskChest, taskHead, taskRH, taskLH, taskRF, taskLF ]:
    task.feature.frame('current')
    task.gain.setConstant(50)
    task.task.dt.value = dt

# --- TASK COM ------------------------------------------------------
taskCom = MetaTaskDynCom(dyn,dt)

# --- TASK POSTURE --------------------------------------------------
taskPosture = MetaTaskDynPosture(dyn,dt)

# --- Task lim ---------------------------------------------------
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
taskLim.referenceVelSup.value = tuple([val*pi/180 for val in dqup])

#-----------------------------------------------------------------------------
# --- SOT Dyn OpSpaceH: SOT controller  --------------------------------------
#-----------------------------------------------------------------------------

sot = SolverDynReduced('sot')
contact = AddContactHelper(sot)

sot.setSize(robotDim-6)
#sot.damping.value = 1e-2
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

plug(sot.solution, robot.control)

#For the integration of q = int(int(qddot)).
plug(sot.acceleration,robot.acceleration)

#-----------------------------------------------------------------------------
# ---- CONTACT: Contact definition -------------------------------------------
#-----------------------------------------------------------------------------

# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.feature.frame('desired')
contactLF.gain.setConstant(1000)
contactLF.name = "LF"

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')
contactRF.gain.setConstant(1000)
contactRF.name = "RF"

contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))

#--- ZMP ---------------------------------------------------------------------
zmp = ZmpEstimator('zmp')
zmp.declare(sot,dyn)

#-----------------------------------------------------------------------------
# --- TRACE ------------------------------------------------------------------
#-----------------------------------------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','pg_','.dat')

tr.add('dyn.com','com')
tr.add(taskCom.feature.name+'.error','ecom')
tr.add(taskCom.featureDes.name+'.errorIN','comref')
tr.add('dyn.waist','waist')
tr.add('dyn.rh','rh')
tr.add('zmp.zmp','')
tr.add('dyn.position','')
tr.add('dyn.velocity','')
tr.add('robot.acceleration','robot_acceleration')
tr.add('robot.control','')
tr.add(taskCom.gain.name+'.gain','com_gain')
tr.add(taskRF.gain.name+'.gain','rf_gain')

tr.add('dyn.lf','lf')
tr.add('dyn.rf','rf')

tr.add('pg.rightfootcontact','rfcontact')


tr.start()
robot.after.addSignal('tr.triger')
robot.after.addSignal(contactLF.task.name+'.error')
robot.after.addSignal('dyn.rf')
robot.after.addSignal('dyn.lf')
robot.after.addSignal('dyn.com')
robot.after.addSignal('sot.forcesNormal')
robot.after.addSignal('dyn.waist')

robot.after.addSignal('taskLim.normalizedPosition')
tr.add('taskLim.normalizedPosition','qn')

history = History(dyn,1,zmp.zmp)

#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------
# --- TASK WAIST ---
# Build the reference waist pos homo-matrix from PG.
waistReferenceVector = Stack_of_vector('waistReferenceVector')
plug(pg.initwaistposref,waistReferenceVector.sin1)
plug(pg.initwaistattref,waistReferenceVector.sin2)
waistReferenceVector.selec1(0,3)
waistReferenceVector.selec2(0,3)
waistReference=PoseRollPitchYawToMatrixHomo('waistReference')
plug(waistReferenceVector.sout,waistReference.sin)
plug(waistReference.sout,taskWaist.featureDes.position)

taskWaist.feature.selec.value = '011100'
taskWaist.task.controlGain.value = 1000

# --- TASK COM ---
plug(comRef.ref,taskCom.featureDes.errorIN)
plug(pg.dcomref,taskCom.featureDes.errordotIN)
taskCom.feature.selec.value = '011'
taskCom.gain.setConstant(10)

# --- TASKS FOOT ---
plug(pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 1000
plug(pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 1000

# --- SOT ---
sot.clear()
contact(contactLF)
contact(contactRF)

#sot.push(taskLim.name)
sot.push(taskCom.task.name)
plug(robot.state,sot.position)
sot.breakFactor.value = 2

# --- SELECTER ---
selec = ContactSelecter('contactSelec')
selec.setSolver(sot.name)

selec.setContactAndTask(contactRF.name,contactRF.task.name,taskRF.task.name)
selec.RFSupport.value = contactRF.support
plug(pg.rightfootcontact,selec.RFActivation)
selec.setContactStatus(contactRF.name,True)

selec.setContactAndTask(contactLF.name,contactLF.task.name,taskLF.task.name)
selec.LFSupport.value = contactLF.support
plug(pg.leftfootcontact,selec.LFActivation)
selec.setContactStatus(contactLF.name,True)

robot.before.addSignal(selec.name+'.trigger')

selec.verbose(True)

# --- Events ---------------------------------------------
sigset = ( lambda s,v : s.__class__.value.__set__(s,v) )
refset = ( lambda mt,v : mt.__class__.ref.__set__(mt,v) )

robot.before.addSignal(selec.name+'.trigger')


#for i in range(10): next()
#
go()
attime(646,stop)

