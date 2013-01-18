# ______________________________________________________________________________
# ******************************************************************************
# ______________________________________________________________________________
# ******************************************************************************

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.attime import attime,ALWAYS,refset,sigset
from numpy import *

from dynamic_graph.sot.core.utils.history import History

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor

# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

robotName = 'hrp14small'
robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)
dt=5e-3

from dynamic_graph.sot.dyninv.robot_specific import halfSittingConfig
x0=-0.00949035111398315034
y0=0
z0=0.64870185118253043  #0.6487018512
halfSittingConfig[robotName] = (x0,y0,z0,0,0,0)+halfSittingConfig[robotName][6:]

initialConfig[robotName]=halfSittingConfig[robotName]

initialConfig[robotName]=(-1.9487374883906774e-27, -1.7176468466207308e-27, 0.63516120560912981, 4.9861897636537844e-20, -1.0588941103603052e-18, 1.2364668742710116e-19, -4.8902218174850969e-18, 2.4342667073361099e-18, -0.48729641533290902, 0.97459283066581781, -0.48729641533290879, -9.861508377534219e-19, -2.359096854752087e-18, -1.3129513558333748e-17, -0.48729641533290935, 0.97459283066581859, -0.48729641533290929, 1.6873551935899719e-17, 0.050059280167850009, 0.010000806681109737, -0.12770543788413571, 0.21393414582231227, 0.1568991096333438, -0.12050409251208098, -0.036557035976426684, -1.0972074690967477, -0.01444727776027852, -0.64702871259334449, 0.17550310601489358, 0.90762433294316336, 0.14017481565796389, 0.0052476440775108182, -0.23339033298135339, -0.00061821588820589444, 0.054877110564186989, 0.17414673138839104)

# Work nicely, but q0 is uggly.
initialConfig[robotName]=(0.00049921279479388854, 0.00049996060172474786, 0.63500107965860886, 8.1671945780155143e-23, 1.5720102693719503e-23, -4.8579693842636446e-23, 9.5431180449191677e-21, -0.00094329030861069489, -0.48680514099116468, 0.97549398127625753, -0.48868884028509291, 0.00094329030861069489, 6.989934206013467e-21, -0.00094334907227440175, -0.48703981005325936, 0.97596355410326363, -0.48892374405000433, 0.00094334907227440175, 0.11348807526440016, 0.20083116658525665, -0.0010545697535446316, -0.029186560447927903, 0.32171157730517314, -0.14204521593028116, -0.10741283049826153, -0.68485604962994939, -0.044874689963830129, -0.078235128020448866, 0.17272683925193663, 0.94207249294658169, 0.23722916347594097, 0.02069536375553379, -0.42067142923482798, -0.00032889180672932412, 0.019378691610578484, 0.17645563918444634)

initialConfig[robotName]=(0.00049999999992054859, 0.00049999999999627166, 0.6350000000004179, -1.3576245230323505e-16, -1.2271661091679395e-14, -7.0755021448112835e-21, 1.1590588873416236e-17, -0.00094336656172265174, -0.48680748131139595, 0.97550163614052687, -0.48869415482911871, 0.00094336656172278759, 1.159528247642137e-17, -0.0009434253350073629, -0.48704216746084605, 0.97597124353230558, -0.48892907607144726, 0.00094342533500749864, 0.0, 0.0, 0.0, 0.0, 0.26179938779914941, -0.17453292519943295, 0.0, -0.52359877559829882, 0.0, 0.0, 0.17453292519943295, 0.26179938779914941, 0.17453292519943295, 0.0, -0.52359877559829882, 0.0, 0.0, 0.17453292519943295)


robot.set( initialConfig[robotName] )
addRobotViewer(robot,small=True,verbose=True)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
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


# ---- TASK GRIP ---
taskRH=MetaTaskKine6d('rh',dyn,'rh','right-wrist')
handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.17)
taskRH.opmodif = matrixToTuple(handMgrip)

# --- STATIC COM (if not walking)
taskCom0 = MetaTaskKineCom(dyn)

# --- POSTURE ---
taskPosture = MetaTaskKinePosture(dyn)

# --- GAZE ---
taskGaze = MetaTaskKine6d('gaze',dyn,'head','gaze')
# Head to camera matrix transform
headMcam=array([[0.0,0.0,1.0,0.0825],[1.,0.0,0.0,-0.029],[0.0,1.,0.0,0.102],[0.0,0.0,0.0,1.0]])
taskGaze.opmodif = matrixToTuple(headMcam)
taskGaze.feature.frame('current')
taskGaze.feature.selec.value = '011'

# --- Task Joint Limits -----------------------------------------
dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskJL = TaskJointLimits('taskJL')
plug(dyn.position,taskJL.position)
taskJL.controlGain.value = 10
taskJL.referenceInf.value = dyn.lowerJl.value
#taskJL.referenceInf.value = ( 0, 0, 0, 0, 0, 0, -0.785398163000, -0.610865238000, -2.181661565000, -0.034906585000, -1.308996939000, -0.349065850000, -0.523598775000, -0.349065850000, -2.181661565000, -0.034906585000, -1.308996939000, -0.610865238000,                          -0.785398163000, 0.01, -0.785398163000, -0.523598775000, -3.141592654000, -1.658062789000, -1.605702912000, -2.391101075000, -1.605702912000, -1.518400000000, 0, -3.141592654000, -0.174532925000, -1.605702912000, -2.391101075000, -1.605702912000, -1.605702912000, 0 )

taskJL.referenceSup.value = dyn.upperJl.value
taskJL.dt.value = dt
taskJL.selec.value = toFlags(range(6,22)+range(22,28)+range(29,35))

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
taskCom.feature.selec.value = '011'

# --- TASK FEET
taskRF=MetaTask6d('RF',dyn,'RF','right-ankle')
plug(pg.pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 5

taskLF=MetaTask6d('LF',dyn,'LF','left-ankle')
plug(pg.pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 5

# --- CONTACTS
# define contactLF and contactRF
for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
    contact = MetaTaskKine6d('contact'+name,dyn,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    locals()['contact'+name] = contact

# --- TRACER -----------------------------------------------------------------
from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')

tr.add(taskRF.featureDes.name+'.position','refr')
tr.add(taskLF.featureDes.name+'.position','refl')
tr.add('dyn.com','com')
tr.add(taskRH.task.name+'.error','erh')

tr.add(taskJL.name+".normalizedPosition","qn")
tr.add("robot.state","q")
tr.add("robot.control","qdot")
robot.after.addSignal(taskJL.name+".normalizedPosition")

# --- SHORTCUTS ----------------------------------------------------------------
qn = taskJL.normalizedPosition
@optionalparentheses
def pqn(details=True):
    ''' Display the normalized configuration vector. '''
    qn.recompute(robot.state.time)
    s = [ "{0:.1f}".format(v) for v in qn.value]
    if details:
        print("Rleg: "+" ".join(s[:6]))
        print("Lleg: "+" ".join(s[6:12]))
        print("Body: "+" ".join(s[12:16]))
        print("Rarm: "+" ".join(s[16:22]))
        print("Larm :"+" ".join(s[22:28]))
    else:
        print(" ".join(s[:30]))


def jlbound(t=None):
    '''Display the velocity bound induced by the JL as a double-column matrix.'''
    if t==None: t=robot.state.time
    taskJL.task.recompute(t)
    return matrix([ [float(x),float(y)] for x,y
                    in [ c.split(',') for c in taskJL.task.value[6:-3].split('),(') ] ])

# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------

#sot.addContact(contactRF)
#sot.addContact(contactLF)
#gotoNd(taskWaist,(0,0,0),'111011',(1,))
#taskWaist.feature.frame('desired')
#sot.push(taskWaist.task.name)
#taskCom0.featureDes.errorIN.value = dyn.com.value
#taskCom0.task.controlGain.value = 10
#sot.push(taskCom0.task.name)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.startHerdt()
# You can now modifiy the speed of the robot using set pg.pg.velocitydes [3]( x, y, yaw)
pg.pg.velocitydes.value =(0.1,0.0,0.0)

#ball0 = (2.5,-0.3,0.5)
ball0 = (0.5,-2.3,0.5)
w=(1/1.3,1/1.8,1/3.9)
h=(0.1,0.07,0.3)

sot.damping.value = 5e-2
sot.push(taskJL.name)
sot.push(taskWaist.task.name)
sot.push(taskRF.task.name)
sot.push(taskLF.task.name)
sot.push(taskCom.task.name)
taskPosture.gotoq((5,1,0.05,0.9),rhand=[1,])
sot.push(taskPosture.task.name)
gotoNd(taskGaze,ball0,'011',(10,1,0.01,0.9))
sot.push(taskGaze.task.name)
gotoNd(taskRH,ball0,'011',(10,1,0.01,0.9))
sot.push(taskRH.task.name)

# --- SCRIPT EVENTS ----------------------------------------------------------
#from random import random
import numpy.random as nprand


class BallTraj:
    def __init__(self,dt,ball0,v0=(0.0,0.0,0.0)):
        self.dt = dt
        self.pball=array(ball0)
        self.vball=array(v0)
        self.bounce = 0
        self.z0 = 0.02
        self.elasticZ = 0.99
        self.elasticXY = 0.3
        self.gravity=(0,0,-3.0)
        nprand.seed(4)
    def __call__(self,t):
        self.vball += self.dt*array(self.gravity)
        self.pball += self.dt*self.vball
        if self.pball[2]<self.z0:
            self.vball[2] =  self.elasticZ * abs(self.vball[2])
            self.pball[2] = self.z0
            self.vball[0:2] = (nprand.rand(2)-0.5)*self.elasticXY
        return vectorToTuple(self.pball)
ballTraj = BallTraj(dt,ball0)

def trackTheBall(ball=None):
    if ball==None:
        t=robot.state.time/200.0
        #ball = h * cos(array(w)*t) + ball0
        ball = ballTraj(t)
    if isinstance(ball,ndarray): ball=vectorToTuple(ball)
    gotoNd(taskRH,ball)
    gotoNd(taskGaze,ball)
    robot.viewer.updateElementConfig('zmp',ball+(0,0,0))

fixed=False
if fixed:          trackTheBall()
else:              attime(ALWAYS,trackTheBall)

def ballInHand():
    #rh = array(dyn.rh.value)[0:3,3]
    robot.after.addSignal(taskRH.feature.name + '.position')
    rh = array(taskRH.feature.position.value)[0:3,3]
    robot.viewer.updateElementConfig('zmp',vectorToTuple(rh)+(0,0,0))

#@attime(3450)
def goForTheBall():
    '''The hand goes for the hand.'''
    taskRH.feature.selec.value = '111'
    setGain(taskRH.gain,(5,0.5,0.03,0.9))
    taskWaist.feature.selec.value = '011000'
    ballTraj.elasticXY = 0.0
    ballTraj.gravity = (0,0,-3.0)
    attime(ALWAYS,graspIf)

#@attime(3400) 
def clearAttimePeriodic():
    '''Stop the PG.'''
    del attime.events[ALWAYS]
    attime(ALWAYS,trackTheBall)
    pg.pg.velocitydes.value = (0,0,0)

#@attime(3900)
def grasp():
    '''Ball is in the hand.'''
    del attime.events[ALWAYS]
    attime(ALWAYS,ballInHand)
    sot.rm(taskRH.task.name)
    sot.rm(taskGaze.task.name)
    q0 = halfSittingConfig[robotName]
    taskPosture.gotoq((5,1,0.05,0.9),q0,rarm=[],larm=[],chest=[],head=[],rhand=[0,])
    setGain(taskWaist.gain,(2,0.2,0.03,0.9))
    taskWaist.feature.selec.value = '011100'


def graspIf():
    if linalg.norm(array(taskRH.task.error.value))<1e-2:
        print grasp.__doc__
        grasp()

@optionalparentheses
def now(t=None):
    if t==None: t=robot.state.time
    attime(t+1,clearAttimePeriodic)
    attime(t+50,goForTheBall)
    #attime(t+500,grasp)

now(3250)

# ------------------------------------------------------------------------------
from math import atan2,pi
def potFieldPG(target,pos,gain,obstacles):
    ''' Compute the velocity of the PG as the gradient of a potential field in
    the 2D plan.'''
    
    e = target-pos
    waRwo = (array(dyn.waist.value)[0:2,0:2]).T
    wo_vref = gain*e
    wa_vref = dot(waRwo , wo_vref)
    vmax = 0.2
    vmax = array([0.3,0.1])
    if abs(wa_vref[0])>vmax[0]:  wa_vref *= vmax[0]/abs(wa_vref[0])
    if abs(wa_vref[1])>vmax[1]:  wa_vref *= vmax[1]/abs(wa_vref[1])
    return wa_vref

def pgv():
    t = robot.state.time
    if t%50!=1: return
    ball = array(taskRH.ref)[0:2,3]
    v = potFieldPG( ball, array(dyn.com.value)[0:2],array((1,1)), [])
    dth = atan2(v[1],v[0])
    vref = (v[0],v[1], 0.2*dth )
    pg.pg.velocitydes.value = vref

attime(ALWAYS,pgv)

#next()
go()



