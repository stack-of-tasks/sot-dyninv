# ______________________________________________________________________________
# ******************************************************************************
# ______________________________________________________________________________
# ******************************************************************************

sys.path.append('..')
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks_kine import *
from viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from attime import attime
from numpy import *

from history import History

from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor

# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

robotName = 'hrp14small'
robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)
dt=5e-3

from robotSpecific import halfSittingConfig
x0=-0.00949035111398315034
y0=0
z0=0.64870185118253043  #0.6487018512
halfSittingConfig[robotName] = (x0,y0,z0,0,0,0)+halfSittingConfig[robotName][6:]

initialConfig[robotName]=halfSittingConfig[robotName]

robot.set( initialConfig[robotName] )
addRobotViewer(robot,small=True,verbose=False)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
from ThreadInterruptibleLoop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.increment(dt)
    attime.run(robot.control.time)
    updateComDisplay(robot,dyn.com)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

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

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value    = gearRatio[robotName]

plug(robot.state,dyn.position)
dyn.velocity.value = robotDim*(0.,)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# --- PG ---------------------------------------------------------
# --- PG ---------------------------------------------------------
# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator.meta_pg import MetaPG
pg = MetaPG(dyn)
pg.plugZmp(robot)


# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------
# The solver SOTH of dyninv is used, but normally, the SOT solver should be sufficient
from dynamic_graph.sot.dyninv import SolverKine
sot = SolverKine('sot')
sot.setSize(robotDim)
plug(sot.control,robot.control)

# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------

# ---- WAIST TASK ---
taskWaist=MetaTask6d('waist',dyn,'waist','waist')
pg.plugWaistTask(taskWaist)
taskWaist.task.controlGain.value = 5

# --- TASK COM ---
taskCom = MetaTaskKineCom(dyn,"compd")
plug(pg.comRef.ref,taskCom.featureDes.errorIN)
plug(pg.pg.dcomref,taskCom.featureDes.errordotIN)
taskCom.task = TaskPD('taskComPD')
taskCom.task.add(taskCom.feature.name)
plug(taskCom.feature.errordot,taskCom.task.errorDot)
plug(taskCom.task.error,taskCom.gain.error)
plug(taskCom.gain.gain,taskCom.task.controlGain)
taskCom.gain.setConstant(40)
taskCom.task.setBeta(-1)

# --- TASK FEET
taskRF=MetaTask6d('rf',dyn,'rf','right-ankle')
plug(pg.pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 5

taskLF=MetaTask6d('lf',dyn,'lf','left-ankle')
plug(pg.pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 5

# --- TRACER -----------------------------------------------------------------
from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')

tr.add(taskRF.featureDes.name+'.position','refr')
tr.add(taskLF.featureDes.name+'.position','refl')

# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------

sot.push(taskWaist.task.name)
sot.push(taskRF.task.name)
sot.push(taskLF.task.name)
sot.push(taskCom.task.name)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.startHerdt()
# You can now modifiy the speed of the robot using set pg.pg.velocitydes [3]( x, y, yaw)
pg.pg.velocitydes.value =(0.1,0.0,0.0)


go()

