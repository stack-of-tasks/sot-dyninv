# ______________________________________________________________________________
# ******************************************************************************
#
# The simplest robot task: just go and reach a point with the right hand.
#
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

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor


# ------------------------------------------------------------------------------
# --- ROBOT SIMULATION ---------------------------------------------------------
# ------------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim  = robotDimension[robotName]
robot     = RobotSimu("robot")
robot.resize(robotDim)

dt=5e-3

from dynamic_graph.sot.dyninv.robot_specific import halfSittingConfig
x0 = -0.00949035111398315034
y0 = 0
z0 = 0.64870185118253043
halfSittingConfig[robotName] = (x0,y0,z0,0,0,0)+halfSittingConfig[robotName][6:]

q0 = list(halfSittingConfig[robotName])
initialConfig[robotName] = tuple(q0)

robot.set( initialConfig[robotName] )
addRobotViewer(robot, small=True, verbose=True)


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)


#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

modelDir          = pkgDataRootDir[robotName]
xmlDir            = pkgDataRootDir[robotName]
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


# ------------------------------------------------------------------------------
# ---- Kinematic Stack of Tasks (SoT)  -----------------------------------------
# ------------------------------------------------------------------------------

sot = SOT('sot')
sot.setSize(robotDim)
plug(sot.control,robot.control)


# ------------------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ------------------------------------------------------------------------------

# ---- TASK GRIP
taskRH    = MetaTaskKine6d('rh',dyn,'rh','right-wrist')
handMgrip = eye(4); handMgrip[0:3,3] = (0,0,-0.21)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')

# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(dyn)
dyn.com.recompute(0)
taskCom.featureDes.errorIN.value = dyn.com.value
taskCom.task.controlGain.value = 10

# --- CONTACTS
# define contactLF and contactRF
for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
    contact = MetaTaskKine6d('contact'+name,dyn,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact'+name] = contact


# ----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
# ----------------------------------------------------------------------------

target = (0.5,-0.2,1.3)
robot.viewer.updateElementConfig('zmp',target+(0,0,0))
gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))

sot.push(contactRF.task.name)
sot.push(contactLF.task.name)
sot.push(taskCom.task.name)
sot.push(taskRH.task.name)

go()
