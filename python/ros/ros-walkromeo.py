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
from dynamic_graph.sot.dyninv import SolverKine

# Create the robot.
from dynamic_graph.sot.romeo.romeo import *
robot = Robot('robot')
plug(robot.device.state, robot.dynamic.position)

# Binds with ros, export joint_state.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a solver.
from dynamic_graph.sot.dynamics.solver import Solver
solver = Solver(robot)

from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_tasks_kine import *
from numpy import *


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

dt=5e-3

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator.meta_pg import MetaPG
pg = MetaPG(robot.dynamic)
pg.plugZmp(robot.device)

# ---- TASKS -------------------------------------------------------------------
# ---- WAIST TASK ---
taskWaist=MetaTask6d('waist',robot.dynamic,'waist','waist')
pg.plugWaistTask(taskWaist)
taskWaist.task.controlGain.value = 5
taskWaist.feature.selec.value = '011100'

# --- TASK COM ---
taskCom = MetaTaskKineCom(robot.dynamic,"compd")
pg.plugComTask(taskCom)
taskCom.feature.selec.value = '011'

# --- TASK FEET
taskRF=MetaTask6d('rf',robot.dynamic,'rf','right-ankle')
plug(pg.pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 40

taskLF=MetaTask6d('lf',robot.dynamic,'lf','left-ankle')
plug(pg.pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 40


# ---- WAIST TASK ORIENTATION ---
#  set the orientation of the waist to be the same as the one of the foot.
taskWaistOr=MetaTask6d('waistOr',robot.dynamic,'waist','waist')
plug(pg.pg.rightfootref,taskWaistOr.featureDes.position)
taskWaistOr.task.controlGain.value = 40
taskWaistOr.feature.selec.value = '100000'

# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
solver.push(taskWaist.task)
solver.push(taskRF.task)
solver.push(taskLF.task)
solver.push(taskCom.task)
solver.push(taskWaistOr.task)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.startHerdt(False)
# You can now modifiy the speed of the robot using set pg.pg.velocitydes [3]( x, y, yaw)
pg.pg.velocitydes.value =(0.1,0.0,0.0)

# go()


