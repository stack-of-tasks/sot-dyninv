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
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.dyninv import SolverKine

from numpy import *

# Create the robot.
from dynamic_graph.sot.romeo.romeo import *
robot = Robot('romeo', device=RobotSimu('romeo'))
plug(robot.device.state, robot.dynamic.position)

# Binds with ros, export joint_state.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a solver.
from dynamic_graph.sot.dynamics.solver import Solver
solver = Solver(robot)

# Alternate visualization tool
addRobotViewer(robot.device,small=True,small_extra=24,verbose=False)


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

dt=5e-3

@loopInThread
def inc():
    robot.device.increment(dt)
    updateComDisplay(robot.device,robot.dynamic.com)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator.meta_pg import MetaPG
pg = MetaPG(robot.dynamic)
pg.plugZmp(robot.device)

# ---- TASKS -------------------------------------------------------------------
# ---- WAIST TASK ---
#  This task fix the motion of the waist around the z axis, the roll and the pitch
taskWaist=MetaTask6d('waist',robot.dynamic,'waist','waist')
pg.plugWaistTask(taskWaist)
taskWaist.task.controlGain.value = 5
taskWaist.feature.selec.value = '011100'

# --- TASK COM ---
# the x,y coords of the centor of mass are given by the pattern generator
taskCom = MetaTaskKineCom(robot.dynamic,"compd")
pg.plugComTask(taskCom)
taskCom.feature.selec.value = '011'

# --- TASK FEET
# The feet are constrained by two 6dofs tasks.
taskRF=MetaTask6d('rf',robot.dynamic,'rf','right-ankle')
plug(pg.pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 40

taskLF=MetaTask6d('lf',robot.dynamic,'lf','left-ankle')
plug(pg.pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 40


############################################################################
# Complete the task definition here.
# ---- WAIST TASK ORIENTATION ---
#  set the orientation of the waist to be the same as the one of the foot.
# Define a metaTask for a 6d task controlling the waistOrientation.
taskWaistOr=MetaTask6d('waistOr',robot.dynamic,'waist','waist')
taskWaistOr.task.controlGain.value = 40
# 1\ define the rightfootrefence given by the entity pg
# as the desired position for the taskWaistOrientation.


# 2\ only constrain the Yaw for the waist.


# ---- HEAD ORIENTATION ---
#  set the orientation of the gaze (head) to be the same as the one of the foot.
# Define a metaTask for a 6d task controlling the waistOrientation.
# 1\ Define a MetaTask6d taskHead, constaining the head, attached to the gaze link

# 2\ Link the orientation of the right foot to the desired position of the head.

# 3\ Only constraint the rotation of the head.

# 4\ Choose the gain

# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
solver.push(taskWaist.task)
solver.push(taskRF.task)
solver.push(taskLF.task)
solver.push(taskCom.task)
# Activate your new tasks.
# solver.push(taskWaistOr.task)
# solver.push(taskHead.task)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.startHerdt(False)

print('You can now modifiy the speed of the robot by setting pg.pg.velocitydes')
print('example : pg.pg.velocitydes.value =(0.1,0.0,0.0)\n')

go()


