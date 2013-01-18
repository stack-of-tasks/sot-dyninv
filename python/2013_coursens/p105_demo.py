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
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
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
z0=0.64870185118253043
halfSittingConfig[robotName] = (x0,y0,z0,0,0,0)+halfSittingConfig[robotName][6:]

q0=list(halfSittingConfig[robotName])
initialConfig[robotName]=tuple(q0)

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
    history.record()

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


# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------
# ---- SOT ---------------------------------------------------------------------
# The solver SOTH of dyninv is used, but normally, the SOT solver should be sufficient
from dynamic_graph.sot.dyninv import SolverKine
def toList(sot):
    return map(lambda x: x[1:-1],sot.dispStack().split('|')[1:])
SolverKine.toList = toList
sot = SolverKine('sot')
sot.setSize(robotDim)
plug(sot.control,robot.control)

# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------


# ---- TASK GRIP ---
taskRH=MetaTaskKine6d('rh',dyn,'rh','right-wrist')
handMgrip=eye(4); handMgrip[0:3,3] = (0,0,-0.21)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')

taskLH=MetaTaskKine6d('lh',dyn,'lh','left-wrist')
taskLH.opmodif = matrixToTuple(handMgrip)
taskLH.feature.frame('desired')

# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(dyn)

# --- TASK AVOID
taskShoulder=MetaTaskKine6d('shoulder',dyn,'shoulder','LARM_JOINT0')
taskShoulder.feature.frame('desired')
gotoNd(taskShoulder,(0,0,0),'010')
taskShoulder.task = TaskInequality('taskShoulderAvoid')
taskShoulder.task.add(taskShoulder.feature.name)
taskShoulder.task.referenceInf.value = (-10,)    # Xmin, Ymin
taskShoulder.task.referenceSup.value = (0.20,)    # Xmin, Ymin
taskShoulder.task.dt.value=dt
taskShoulder.task.controlGain.value = 0.9

taskElbow=MetaTaskKine6d('elbow',dyn,'elbow','LARM_JOINT3')
taskElbow.feature.frame('desired')
gotoNd(taskElbow,(0,0,0),'010')
taskElbow.task = TaskInequality('taskElbowAvoid')
taskElbow.task.add(taskElbow.feature.name)
taskElbow.task.referenceInf.value = (-10,)    # Xmin, Ymin
taskElbow.task.referenceSup.value = (0.20,)    # Xmin, Ymin
taskElbow.task.dt.value=dt
taskElbow.task.controlGain.value = 0.9


# --- TASK SUPPORT --------------------------------------------------
featureSupport    = FeatureGeneric('featureSupport')
plug(dyn.com,featureSupport.errorIN)
plug(dyn.Jcom,featureSupport.jacobianIN)

taskSupport=TaskInequality('taskSupport')
taskSupport.add(featureSupport.name)
taskSupport.selec.value = '011'
taskSupport.referenceInf.value = (-0.08,-0.15,0)    # Xmin, Ymin
taskSupport.referenceSup.value = (0.11,0.15,0)  # Xmax, Ymax
taskSupport.dt.value=dt

# --- TASK SUPPORT SMALL --------------------------------------------
featureSupportSmall = FeatureGeneric('featureSupportSmall')
plug(dyn.com,featureSupportSmall.errorIN)
plug(dyn.Jcom,featureSupportSmall.jacobianIN)

taskSupportSmall=TaskInequality('taskSupportSmall')
taskSupportSmall.add(featureSupportSmall.name)
taskSupportSmall.selec.value = '011'
#taskSupportSmall.referenceInf.value = (-0.02,-0.02,0)    # Xmin, Ymin
#askSupportSmall.referenceSup.value = (0.02,0.02,0)  # Xmax, Ymax
taskSupportSmall.referenceInf.value = (-0.02,-0.05,0)    # Xmin, Ymin
taskSupportSmall.referenceSup.value = (0.02,0.05,0)  # Xmax, Ymax
taskSupportSmall.dt.value=dt

# --- POSTURE ---
taskPosture = MetaTaskKinePosture(dyn)

# --- GAZE ---
taskGaze = MetaTaskVisualPoint('gaze',dyn,'head','gaze')
# Head to camera matrix transform
# Camera RU headMcam=array([[0.0,0.0,1.0,0.0825],[1.,0.0,0.0,-0.029],[0.0,1.,0.0,0.102],[0.0,0.0,0.0,1.0]])
# Camera LL 
headMcam=array([[0.0,0.0,1.0,0.081],[1.,0.0,0.0,0.072],[0.0,1.,0.0,0.031],[0.0,0.0,0.0,1.0]])
headMcam = dot(headMcam,rotate('x',10*pi/180))
taskGaze.opmodif = matrixToTuple(headMcam)

# --- FOV ---
taskFoV = MetaTaskVisualPoint('FoV',dyn,'head','gaze')
taskFoV.opmodif = matrixToTuple(headMcam)

taskFoV.task=TaskInequality('taskFoVineq')
taskFoV.task.add(taskFoV.feature.name)
[Xmax,Ymax]=[0.38,0.28]
taskFoV.task.referenceInf.value = (-Xmax,-Ymax)    # Xmin, Ymin
taskFoV.task.referenceSup.value = (Xmax,Ymax)  # Xmax, Ymax
taskFoV.task.dt.value=dt
taskFoV.task.controlGain.value=0.01
taskFoV.featureDes.xy.value = (0,0)


# --- Task Joint Limits -----------------------------------------
dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskJL = TaskJointLimits('taskJL')
plug(dyn.position,taskJL.position)
taskJL.controlGain.value = 10
taskJL.referenceInf.value = dyn.lowerJl.value
taskJL.referenceSup.value = dyn.upperJl.value
taskJL.dt.value = dt
taskJL.selec.value = toFlags(range(6,22)+range(22,28)+range(29,35))


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

tr.add('dyn.com','com')

history = History(dyn,1)

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
        print("Rarm: "+" ".join(s[16:23]))
        print("Larm :"+" ".join(s[23:30]))
    else:
        print(" ".join(s[:30]))


def jlbound(t=None):
    '''Display the velocity bound induced by the JL as a double-column matrix.'''
    if t==None: t=robot.state.time
    taskJL.task.recompute(t)
    return matrix([ [float(x),float(y)] for x,y
                    in [ c.split(',') for c in taskJL.task.value[6:-3].split('),(') ] ])

def p6d(R,t):
    M=eye(4)
    M[0:3,0:3]=R
    M[0:3,3]=t
    return M

def push(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName not in sot.toList():
        sot.push(taskName)
        if taskName!="taskposture" and "taskposture" in sot.toList():
            sot.down("taskposture")


def pop(task):
    if isinstance(task,str): taskName=task
    elif "task" in task.__dict__:  taskName=task.task.name
    else: taskName=task.name
    if taskName in sot.toList(): sot.rm(taskName)


# --- DISPLAY ------------------------------------------------------------------
# --- DISPLAY ------------------------------------------------------------------
# --- DISPLAY ------------------------------------------------------------------
RAD=pi/180
comproj = [0.1,-0.95,1.6]
#robot.viewer.updateElementConfig('footproj',[0.5,0.15,1.6+0.08,0,-pi/2,0 ])
robot.viewer.updateElementConfig('footproj',comproj+[0,-pi/2,0 ])
robot.viewer.updateElementConfig('zmp2',[0,0,-10,0,0,0])


class BallPosition:
    def __init__(self,xyz=(0,-1.1,0.9)):
        self.ball = xyz
        self.prec = 0
        self.t = 0
        self.duration = 0
        self.f = 0
        self.xyz= self.ball
        
    def move(self,xyz,duration=50):
        self.prec = self.ball
        self.ball = xyz
        self.t = 0
        self.duration = duration
        self.changeTargets()

        if duration>0:
            self.f = lambda : self.moveDisplay()
            attime(ALWAYS,self.f)
        else:
            self.moveDisplay()

    def moveDisplay(self):
        tau = 1.0 if self.duration<=0 else float(self.t) / self.duration
        xyz = tau * array(self.ball) + (1-tau) * array(self.prec)
        robot.viewer.updateElementConfig('zmp',vectorToTuple(xyz)+(0,0,0))

        self.t += 1
        if self.t>self.duration and self.duration>0:
            attime.stop(self.f)
        self.xyz= xyz
        
    def changeTargets(self):
        gotoNd(taskRH,self.ball,'111',(4.9,0.9,0.01,0.9))
        taskFoV.goto3D(self.ball)

b = BallPosition()


tr.add(taskJL.name+".normalizedPosition","qn")
tr.add('dyn.com','com')
tr.add(taskRH.task.name+'.error','erh')
tr.add(taskFoV.feature.name+'.error','fov')
tr.add(taskFoV.task.name+'.normalizedPosition','fovn')
tr.add(taskSupportSmall.name+".normalizedPosition",'comsmalln')
tr.add(taskSupport.name+".normalizedPosition",'comn')

robot.after.addSignal(taskJL.name+".normalizedPosition")
robot.after.addSignal(taskSupportSmall.name+".normalizedPosition")
robot.after.addSignal(taskFoV.task.name+".normalizedPosition")
robot.after.addSignal(taskFoV.feature.name+'.error')
robot.after.addSignal(taskSupport.name+".normalizedPosition")

# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------------

dyn.com.recompute(0)
taskCom.featureDes.errorIN.value = dyn.com.value
taskCom.task.controlGain.value = 10

ball = BallPosition((0,-1.1,0.9))

push(taskRH)
ball.move((0.5,-0.2,1.3),0)

next()
next()
next()
 
def config(ref=0):
    if ref==0:
        print '''Only the task RH'''
    elif ref==1:
        print '''Task RH + foot constraint, balance is kept'''
        sot.addContact(contactRF)
        sot.addContact(contactLF)
    elif ref==2:
        print '''Task RH + foot constraint, balance is lost'''
        sot.addContact(contactRF)
        sot.addContact(contactLF)
        ball.move((0.15,-0.2,1.3),0)
    elif ref==3:
        print '''Task RH + foot constraint + COM='''
        sot.addContact(contactRF)
        sot.addContact(contactLF)
        push(taskCom)
        ball.move((0.15,-0.2,1.3),0)
    elif ref==4:
        print '''Task RH + foot constraint + COM= + JL'''
        qu =  list(dyn.upperJl.value)
        qu[23]=-0.3
        taskJL.referenceSup.value =tuple(qu)
        push(taskJL)
        sot.addContact(contactRF)
        sot.addContact(contactLF)
        push(taskCom)
        ball.move((0.15,-0.2,1.3),0)
    elif ref==5:
        print '''Task RH + foot constraint + COM<'''
        sot.addContact(contactRF)
        sot.addContact(contactLF)
        push(taskSupport)
        ball.move((0.15,-0.2,1.3),0)
    elif ref==6:
        print '''Task RH + foot constraint + COM= + SINGULARITE '''
        print '''(press 4x i to reach it)'''
        sot.addContact(contactRF)
        sot.addContact(contactLF)
        push(taskCom)
        ball.move((0.6,-0.2,1.3),0)
    elif ref==7:
        print '''Task RH + foot constraint + COM= + SINGULARITE + DAMPING'''
        sot.addContact(contactRF)
        sot.addContact(contactLF)
        push(taskCom)
        ball.move((0.9,-0.2,1.3),0)
        sot.down(taskRH.task.name)
        sot.down(taskRH.task.name)
        sot.damping.value = 0.1

    else:
        print '''Not a correct config.'''
        return
    go()
c=config

@optionalparentheses
def i():
    xyz=ball.xyz
    xyz[0] += 0.1
    ball.move(vectorToTuple(xyz),30)


def menu(ref=0):
    print '\n\n\n\n\n\n\n\n\n'
    print '''0: Only the task RH'''
    print '''1: Task RH + foot constraint, balance is kept'''
    print '''2: Task RH + foot constraint, balance is lost'''
    print '''3: Task RH + foot constraint + COM='''
    print '''4: Task RH + foot constraint + COM= + JL'''
    print '''5: Task RH + foot constraint + COM<'''
    print '''6: Task RH + foot constraint + COM= + SINGULARITE '''
    print '''7: Task RH + foot constraint + COM= + SINGULARITE + DAMPING'''
    print ''
    uinp = raw_input('   Config? >> ')
    config(int(uinp))

menu()
