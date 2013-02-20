# ______________________________________________________________________________
# ******************************************************************************
# A simple Herdt walking pattern generator for Romeo.
# ______________________________________________________________________________
# ******************************************************************************

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
from dynamic_graph.sot.dyninv import SolverKine

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor,specificitiesName,jointRankName

from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)


# --- ROBOT SIMU ---------------------------------------------------------------
robotName = 'romeo'
robotDim=robotDimension[robotName]
robot = RobotSimu("romeo")
robot.resize(robotDim)
dt=5e-3

robot.set( initialConfig[robotName] )
addRobotViewer(robot,small=True,small_extra=24,verbose=False)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
@loopInThread
def inc():
    robot.increment(dt)
    updateComDisplay(robot,dyn.com)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

#---- DYN --------------------------------------------------------------------
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
from dynamic_graph.sot.pattern_generator.meta_pg import MetaPG
pg = MetaPG(dyn)
pg.plugZmp(robot)

# ---- SOT ---------------------------------------------------------------------
# The basic SOT solver would work too.
sot = SolverKine('sot')
sot.setSize(robotDim)
plug(sot.control,robot.control)

# ---- TASKS -------------------------------------------------------------------
# ---- WAIST TASK ---
taskWaist=MetaTask6d('waist',dyn,'waist','waist')
pg.plugWaistTask(taskWaist)
taskWaist.task.controlGain.value = 5
taskWaist.feature.selec.value = '011100'

# --- TASK COM ---
taskCom = MetaTaskKineCom(dyn,"compd")
pg.plugComTask(taskCom)
taskCom.feature.selec.value = '011'

# --- TASK FEET
taskRF=MetaTask6d('rf',dyn,'rf','right-ankle')
plug(pg.pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 40

taskLF=MetaTask6d('lf',dyn,'lf','left-ankle')
plug(pg.pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 40


# ---- WAIST TASK ORIENTATION ---
#  set the orientation of the waist to be the same as the one of the foot.
taskWaistOr=MetaTask6d('waistOr',dyn,'waist','waist')
plug(pg.pg.rightfootref,taskWaistOr.featureDes.position)
taskWaistOr.task.controlGain.value = 40
taskWaistOr.feature.selec.value = '100000'


taskHead=MetaTask6d('head',dyn,'gaze','gaze')
plug(taskRF.featureDes.position, taskHead.featureDes.position)
taskHead.feature.selec.value = '111000'
taskHead.task.controlGain.value = 5

# --- TASK POSTURE --------------------------------------------------
# set a default position for the joints. 
featurePosition = FeatureGeneric('featurePosition')
featurePositionDes = FeatureGeneric('featurePositionDes')
featurePosition.setReference('featurePositionDes')
plug(dyn.position,featurePosition.errorIN)
featurePositionDes.errorIN.value = initialConfig['romeo']
featurePosition.jacobianIN.value = totuple( identity(robotDim) )

taskPosition = Task('taskPosition')
taskPosition.add('featurePosition')
# featurePosition.selec.value = toFlags((6,24))

gainPosition = GainAdaptive('gainPosition')
gainPosition.set(0.1,0.1,125e3)
gainPosition.gain.value = 5
plug(taskPosition.error,gainPosition.error)
plug(gainPosition.gain,taskPosition.controlGain)
featurePosition.selec.value = '000000000000001111111111111100000000000'


# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
sot.push(taskWaist.task.name)
sot.push(taskRF.task.name)
sot.push(taskLF.task.name)
sot.push(taskCom.task.name)
sot.push(taskWaistOr.task.name)

# Stun the upper part of the body.
sot.push(taskHead.task.name)		# constraint the head orientation: look straight ahead
sot.push('taskPosition')			# stun the arms.



# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.startHerdt(False)
# You can now modifiy the speed of the robot using set pg.pg.velocitydes [3]( x, y, yaw)
pg.pg.velocitydes.value =(0.1,0.0,0.0)

go()


