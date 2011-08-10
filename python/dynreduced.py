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
robotName = 'hrp10small'
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

robotDim=robotDimension[robotName]
RobotClass = RobotDynSimu
robot = RobotClass("robot")
robot.resize(robotDim)

robot.set( initialConfig[robotName] )
dt=5e-3

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

qs=[]
def inc():
    attime.run(robot.control.time+1)
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
task=MetaTaskDyn6d('rh',dyn,'task')
task.ref = ((0,0,-1,0.22),(0,1,0,-0.37),(1,0,0,.74),(0,0,0,1))

# Task LFoot: Move the right foot up.
taskLF=MetaTaskDyn6d('lf',dyn,'lf','left-ankle')
taskLF.ref = ((1,0,0,0.0),(0,1,0,+0.29),(0,0,1,.15),(0,0,0,1))

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
taskCom.dt.value = 1e-3

gCom = GainAdaptive('gCom')
plug(taskCom.error,gCom.error)
plug(gCom.gain,taskCom.controlGain)
# Work from .04 to .09 gCom.set 1050 45 125e3
# Good behavior gCom.set 1080 15 125e3
# Low gains gCom.set 400 1 39e3
# Current choice
gCom.set(1050,45,125e3)

# ---- CONTACT -----------------------------------------
# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.feature.frame('desired')

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactRF.feature.frame('desired')

# --- SOT Dyn OpSpaceH --------------------------------------
# SOT controller.
sot = SolverDynRed2('sot')
#sot = SolverOpSpace('sot')
sot.setSize(robotDim-6)
#sot.damping.value = 2e-2
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

plug(sot.torque,robot.control)
plug(sot.acceleration,robot.acceleration)

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

tr.add('taskCom.error','com')
tr.add('taskCom.task','')
tr.add(contactLF.task.name+'.error','lf')

# --- RUN ------------------------------------------------
sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = contactLF.support
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = contactRF.support

taskCom.controlGain.value = 100
#featureComDes.errorIN.value = (0.06,  0.15,  0.8)
featureComDes.errorIN.value = (0.06,  0.145,  0.8  )
sot.push('taskCom')

#@attime(5)
def rm():
    featureComDes.errorIN.value = dyn.com.value
attime(500,stop,'stop')


plug(robot.state,sot.position)
sot.posture.value = ( 0,0,0,0,0,0,  -0.009116303,  -0.091409964,  -0.471978743,   0.840380193,  -0.470232799,   0.089662408,   0.009507818,   0.091110287,  -0.469450352,   0.835307995,  -0.467686191,  -0.093802947,  -0.000087565,   0.003264319,  -0.000007834,   0.000194743,   0.258370257,  -0.175099102,  -0.000061173,  -0.524953549,   0.000003183,  -0.000257600,  -0.000003412,   0.258367272,   0.174322098,  -0.000088902,  -0.524983691,  -0.000000346,  -0.000265401,   0.3   )

matlab.prec=9
matlab.fullPrec=0

for i in range(100):    inc()
print sot.velocity.m
print sot.dyndrift.m
print sot.forceGenerator.m
print sot.Jc.m
print sot.freeMotionBase.m
print sot.freeForceBase.m
print sot.inertiaSqroot.m
print sot.inertiaSqrootInv.m
print sot.driftContact.m
print taskCom.jacobian.m
print taskCom.task.m
print taskCom.Jdot.m
print "dq=velocity; X=forceGenerator; V=freeMotionBase; K = freeForceBase; B=inertiaSqrootInv; Bi=inertiaSqroot; Jcom=jacobian; b=dyndrift; A=Bi'*Bi; dc = driftContact; J=X*Jc; G=J*B; Gc=Jc*B;"

print sot.solution.m
print sot.acceleration.m
sot.forces.recompute(sot.acceleration.time)
print sot.forces.m
print "x=solution; ddq=acceleration; f=forces; phi = X'*f;"
print "u=x(1:24); psi=x(25:end);"

#sot.solution.recompute(1)
#sot.acceleration.recompute(1)

