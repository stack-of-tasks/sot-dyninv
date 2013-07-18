from controller_pd import ControllerPD
from task_dyn_pd import TaskDynPD
from task_dyn_inequality import TaskDynInequality
from dynamic_integrator import DynamicIntegrator
from pseudo_robot_dynamic import PseudoRobotDynamic
from solver_op_space import SolverOpSpace
from solver_dyn_reduced import SolverDynReduced
from zmp_estimator import ZmpEstimator
from robot_dyn_simu import RobotDynSimu
from task_dyn_joint_limits import TaskDynJointLimits
from task_dyn_limits import TaskDynLimits
from task_dyn_passing_point import TaskDynPassingPoint
from task_joint_limits import TaskJointLimits
from task_inequality import TaskInequality
from feature_projected_line import FeatureProjectedLine
from contact_selecter import ContactSelecter
from task_weight import TaskWeight

from solver_kine import SolverKine
def sot_addContact( sot,metaTask ):
    metaTask.keep()
    sot.push(metaTask.task.name)
setattr(SolverKine,'addContact',sot_addContact)
