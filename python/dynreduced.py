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
taskCom.dt.value = dt

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
contactLF.task.dt.value=dt

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactRF.feature.frame('desired')
contactLF.task.dt.value=dt

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

tr.add('taskCom.error','com')
tr.add('taskCom.task','')
tr.add(contactLF.task.name+'.error','lf')
tr.add(contactRF.task.name+'.error','rf')

if not rs: tr.add('sot.solution','')

if rs:
    tr.add('sot._LF_f','fl')
    tr.add('sot._RF_f','fr')
    tr.add('sot._LF_fn','fnl')
    tr.add('sot._RF_fn','fnr')
else:
    tr.add('sot.forces','')
    robot.after.addSignal('sot.forces')

if not rs:
    tr.add('sot.forcesNormal','')
    robot.after.addSignal('sot.forcesNormal')
    tr.add('sot.activeForces','')
    robot.after.addSignal('sot.activeForces')



# --- CHRONO ------------------------------------------------
class Chrono:
    t0=0
    def __init__(self):
        self.t0 = time.time()
    def tic(self):
        return time.time()-self.t0
chrono=Chrono()
attime(499,chrono.tic)

# --- RUN ------------------------------------------------
matlab.prec=9
matlab.fullPrec=0


taskCom.controlGain.value = 100
#featureComDes.errorIN.value = (0.06,  0.15,  0.8)
#featureComDes.errorIN.value = (0.06,  0.145,  0.8  )

featureComDes.errorIN.value = (0.1,  0.145,  0.8  )
#Nominal: no zmp overshoot featureComDes.errorIN.value = (0.084,  0.144,  0.804  )
sot.push('taskCom')

#@attime(5)
def rm():
    featureComDes.errorIN.value = dyn.com.value
attime(500,stop,'stop')



#go()

# --- DEBUG ----------------------------------------------
for i in range(50):    inc()

print sot.velocity.m
print sot.dyndrift.m
print sot.matrixInertia.m
print sot.acceleration.m
print "dq=velocity; b=dyndrift; A=matrixInertia; ddq=acceleration;"

if rs:
    print sot.control.m
    print sot._LF_f.m
    print sot._LF_fn.m
    print sot._RF_f.m
    print sot._RF_fn.m
    print "ddq1=ddq; phil1= _LF_f; phir1 = _RF_f; phi1=[ phil1; phir1 ]; fnl1=_LF_fn; fnr1 = _RF_fn; fn1 = [fnl1; fnr1 ]; tau1=control;"

else:
    print sot.sizeForceSpatial.m
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
    print "X=forceGenerator; V=freeMotionBase; K = freeForceBase; B=inertiaSqrootInv; Bi=inertiaSqroot; Jcom=jacobian; Ab=Bi'*Bi; dc = driftContact; J=X*Jc; G=J*B; Gc=Jc*B; Sb=[eye(6) zeros(6,30)];"

    print sot.solution.m
    print sot.acceleration.m
    sot.forces.recompute(sot.acceleration.time)
    print sot.forces.m
    print "x=solution; ddq=acceleration; f=forces; phi = X'*f;"
    print "u=x(1:24); psi=x(25:end); "

    print "ddq2= ddq; phi2=phi; f2=f; fn2=f(3:3:end);"

'''
if 0: # double check
    sotreg = SolverOpSpace('sotreg')
    sotreg.setSize(robotDim-6)
    sotreg.breakFactor.value = 10

    plug(dyn.inertiaReal,sotreg.matrixInertia)
    plug(dyn.dynamicDrift,sotreg.dyndrift)
    plug(dyn.velocity,sotreg.velocity)

    sotreg.addContactFromTask(contactLF.task.name,'LF')
    sotreg._LF_p.value = contactLF.support
    sotreg.addContactFromTask(contactRF.task.name,'RF')
    sotreg._RF_p.value = contactRF.support

    sotreg.push('taskCom')
    sotreg.control.recompute(sot.acceleration.time)

    print sotreg.control.m
    print sotreg._LF_f.m
    print sotreg._LF_fn.m
    print sotreg._RF_f.m
    print sotreg._RF_fn.m
    print sotreg.acceleration.m
    print "ddq1=acceleration; phil1= _LF_f; phir1 = _RF_f; phi1=[ phil1; phir1 ]; fnl1=_LF_fn; fnr1 = _RF_fn; fn1 = [fnl1; fnr1 ]; tau1=control;"

    print taskCom.task.m
'''

