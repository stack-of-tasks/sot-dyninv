from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from meta_task_dyn_6d import MetaTaskDyn6d
from attime import attime

from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
robotName = 'hrp14small'

from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)

# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

initialConfig['hrp14small'] = ( 0.0274106623863, 0.143843868989, 0.646921914726, 0.00221064938462, 0.101393756965, 1.36729741242e-05,
 -0.00911630330837, -0.0914099637938, -0.471978743283, 0.840380192617, -0.470232799053, 0.0896624078591,
0.00950781802257, 0.0911102868041, -0.469450351848, 0.835307995386, -0.467686190904, -0.0938029466367,
 0.3, 0.3, 0, 0,
 0.6, -0.3, 0, -0.52, 0, -0.1, 0,
-0.2,  0.17, 0, -0.52, 0,  0.1, 0 )

robotDim=robotDimension[robotName]
RobotClass = RobotDynSimu
robot = RobotClass("robot")
robot.resize(robotDim)

robot.set( initialConfig[robotName] )
dt=1e-3

# --- VIEWER -------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
try:
   import robotviewer

   def stateFullSize(robot):
       return [float(val) for val in robot.state.value]+10*[0.0]
   RobotClass.stateFullSize = stateFullSize
   robot.viewer = robotviewer.client('XML-RPC')
   # Check the connection
   robot.viewer.updateElementConfig('hrp',robot.stateFullSize())

   def refreshView( robot ):
       robot.viewer.updateElementConfig('hrp',robot.stateFullSize())
   RobotClass.refresh = refreshView
   def incrementView( robot,dt ):
       robot.incrementNoView(dt)
       robot.refresh()
   RobotClass.incrementNoView = RobotClass.increment
   RobotClass.increment = incrementView
   def setView( robot,*args ):
       robot.setNoView(*args)
       robot.refresh()
   RobotClass.setNoView = RobotClass.set
   RobotClass.set = setView

   robot.refresh()
except:
    #print "No robot viewer, sorry."
    robot.viewer = None

# --- MAIN LOOP ------------------------------------------
#rs=True # Regular solver
rs=False # Reduced solver

if rs:
    toBeRecomputed = ['matrixInertia','dyndrift']
else:
    toBeRecomputed = ['Jc', 'driftContact','dyndrift', 'freeForceBase', 'freeMotionBase', 'inertiaSqrootInv']
qs=[]
chronos=[]
def inc():
    tnext=robot.control.time+1
    attime.run(tnext)
    for sig in toBeRecomputed:
        sot.signal(sig).recompute( tnext )
    chrono=Chrono()
    sot.triger.recompute(tnext)
    chronos.append(chrono.tic())
    robot.increment(dt)
    qs.append(robot.state.value)

from ThreadInterruptibleLoop import *
@loopInThread
def loop():
    inc()
runner=loop()

@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): #runner.once()
    inc()
    print sot.velocity.value

# --- shortcuts -------------------------------------------------
@optionalparentheses
def q():
    if 'dyn' in globals(): print dyn.ffposition.__repr__()
    print robot.state.__repr__()
@optionalparentheses
def qdot(): print robot.control.__repr__()
@optionalparentheses
def t(): print robot.state.time-1
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay

# --- DYN ----------------------------------------------------------------------
modelDir = pkgDataRootDir[robotName]
xmlDir = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath = xmlDir + '/HRP2LinkJointRankSmall.xml'

dyn = Dynamic("dyn")
dyn.setFiles(modelDir,modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value = gearRatio[robotName]

plug(robot.state,dyn.position)
plug(robot.velocity,dyn.velocity)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# --- Task Dyn -----------------------------------------
# Task right hand
taskRH=MetaTaskDyn6d('rh',dyn,'rh','right-wrist')
taskLH=MetaTaskDyn6d('lh',dyn,'lh','left-wrist')
taskRF=MetaTaskDyn6d('rf',dyn,'rf','right-ankle')
taskLF=MetaTaskDyn6d('lf',dyn,'lf','left-ankle')
taskChest=MetaTaskDyn6d('chest',dyn,'chest','chest')
for task in (taskRH,taskLH,taskRF,taskLF): task.task.dt.value = dt

# --- TASK COM ------------------------------------------------------
dyn.setProperty('ComputeCoM','true')

featureCom    = FeatureGeneric('featureCom')
featureComDes = FeatureGeneric('featureComDes')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureCom.sdes.value = 'featureComDes'

taskCom = TaskDynPD('taskCom')
taskCom.add('featureCom')
plug(dyn.velocity,taskCom.qdot)
taskCom.dt.value = dt

gCom = GainAdaptive('gCom')
plug(taskCom.error,gCom.error)
plug(gCom.gain,taskCom.controlGain)
gCom.set(1050,45,125e3)

# ---- CONTACT -----------------------------------------
# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.feature.frame('desired')
contactLF.task.dt.value=dt

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactRF.feature.frame('desired')
contactRF.task.dt.value=dt

# --- SOT Dyn OpSpaceH --------------------------------------

# SOT controller.
if rs:
    sot = SolverOpSpace('sot')
    sot.triger = sot.control
else:
    sot = SolverDynReduced('sot')
    sot.triger = sot.acceleration

sot.setSize(robotDim-6)
#sot.damping.value = 2e-2
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

if rs: plug(sot.control,robot.control)
else: plug(sot.solution,robot.control)
plug(sot.acceleration,robot.acceleration)

sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = contactLF.support
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = contactRF.support


# --- TRACE ----------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')

tr.add('dyn.position','')
#tr.add('sot.forces','')
tr.add('sot.acceleration','')
tr.add('sot.velocity','')
#robot.after.addSignal('sot.forces')
robot.after.addSignal('sot.acceleration')

robot.after.addSignal('dyn.com')
tr.add('dyn.com','')
tr.add(contactLF.task.name+'.error','lf')
tr.add(contactRF.task.name+'.error','rf')

if not rs:
    tr.add('sot.solution','')
    robot.after.addSignal('sot.forces')
    robot.after.addSignal('sot.forcesNormal')
    tr.add('sot.forcesNormal','')
tr.add('sot._LF_f','fl')
tr.add('sot._RF_f','fr')
tr.add('sot._LF_fn','fnl')
tr.add('sot._RF_fn','fnr')


# --- CHRONO ------------------------------------------------
class Chrono:
    t0=0
    def __init__(self):
        self.t0 = time.time()
    def tic(self):
        return time.time()-self.t0
    def disptic(self):
        print 'tic?'
        print self.tic()
chrono=Chrono()
#attime(99,chrono.disptic)

# --- RUN ------------------------------------------------
matlab.prec=9
matlab.fullPrec=0


taskChest.feature.selec.value = '000101'
taskChest.feature.frame('current')
taskChest.gain.setConstant(15)
taskChest.ref = ((1,0,0,0.1),(0,1,0,0.15),(0,0,1,0.999),(0,0,0,1))

taskLH.ref = ((0,0,1,-0.1),(0,1,0,0.45),(-1,0,0,0.6),(0,0,0,1))
taskLH.feature.selec.value = '000111'
taskLH.feature.frame('current')
taskLH.gain.set(1050,45,125e3)


def moveChest2front():
    ''' -- Move the chest to the front (circularlly)'''
    alpha_d = 20*pi/180.
    taskChest.feature.position.recompute(robot.control.time)
    temp = taskChest.feature.position.value
    R = sqrt(temp[0][3]**2+temp[2][3]**2)                     # Radius in x,z
    alpha_f = arctan2(temp[2][3],temp[0][3]) - alpha_d        # alpha to decrease circularly
    Xch=R*cos(alpha_f); Zch=R*sin(alpha_f)                    # Values of X and Y \in circle
    taskChest.ref = ((1,0,0,Xch),(0,1,0,temp[1][3]),(0,0,1,Zch),(0,0,0,1))
    taskChest.feature.selec.value = '000101'
    taskChest.task.resetJacobianDerivative()

# Move somehow like in a 'circle'
def moveLh2front():
    ''' -- Move the left hand to the front (circularlly)'''
    alpha = 45*pi/180.
    taskLH.feature.position.recompute(robot.control.time)
    temp = taskLH.feature.position.value
    R = 0.55
    Xlh=temp[0][3]+R*sin(alpha);
    Zlh=temp[2][3]+R*(1-cos(alpha))                    # Values of X and Y \in circle
    taskLH.ref = ((1,0,0,Xlh),(0,1,0,0.35),(0,0,1,Zlh),(0,0,0,1))
    taskLH.feature.selec.value = '000111'
    taskLH.task.resetJacobianDerivative()

def addLeftHandContact():
    ''' -- Add left hand contact'''
    sot.addContactFromTask(taskLH.task.name,'LH')
    supportLH = ((-0.13,-0.17,-0.17,0.15),(0.55,0.55,0.45,0.45),(-0.13,-0.17,-0.17,0.15))
    sot._LH_p.value = supportLH
    sot.rm(taskLH.task.name)
    taskLH.task.resetJacobianDerivative()


sot.clear()

from functools import partial

attime(   1, (partial(sot.push,taskChest.task.name)," -- Waist Chest pushed to stack of tasks"), moveChest2front )
attime(  50, (partial(sot.push,taskLH.task.name)," -- Left hand pushed to stack of tasks"), moveLh2front )
attime( 300, moveLh2front)
attime( 800, addLeftHandContact, sot.clear )
attime(1100, sot.clear, stop )

#go()



