# ______________________________________________________________________________
# ******************************************************************************
# The simplest robot task: just go and reach a point with the right hand.
# ______________________________________________________________________________
# ******************************************************************************

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from numpy import *

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor,specificitiesName,jointRankName

# --- ROBOT SIMU ---------------------------------------------------------------
robotName = 'romeo'
robotDim=robotDimension[robotName]
robot = RobotSimu("romeo")
robot.resize(robotDim)
dt=5e-3

#q0=list(initialConfig[robotName])
#q0[0]=-0.027827
#initialConfig[robotName]=tuple(q0)

robot.set( initialConfig[robotName] )
addRobotViewer(robot,small=True,small_extra=24,verbose=False)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.increment(dt)
    updateComDisplay(robot,dyn.com)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------
modelDir  = pkgDataRootDir[robotName]
xmlDir    = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/' + specificitiesName[robotName]
jointRankPath     = xmlDir + '/' + jointRankName[robotName]

dyn = Dynamic("dyn")
dyn.setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value    = gearRatio[robotName]

plug(robot.state,dyn.position)
dyn.velocity.value = robotDim*(0.,)
dyn.acceleration.value = robotDim*(0.,)

# --- PG ---------------------------------------------------------
# --- PG ---------------------------------------------------------
# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator.meta_pg import MetaPG
pg = MetaPG(dyn)
pg.plugZmp(robot)

# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------

sot = SOT('sot')
sot.setNumberDofs(robotDim)
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

#go()
next()
