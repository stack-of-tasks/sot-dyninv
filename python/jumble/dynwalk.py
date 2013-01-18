from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from meta_task_dyn_6d import MetaTaskDyn6d

from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
robotName = 'hrp10small'

from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)

initialConfig.clear()
initialConfig['hrp10small'] = (0,0,0.648697398115,0,0,0)+(0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, 0, 0, 0.2618, -0.1745, 0, -0.5236, 0, 0, 0, 0, 0.2618, 0.1745, 0, -0.5236, 0, 0, 0, 0 )

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
    print "No robot viewer, sorry."
    robot.viewer = None

# --- MAIN LOOP ------------------------------------------

qs=[]
left=1
right=0
both=2
incontact=both
def inc():
    t=robot.state.time
#    print t,pg.SupportFoot.time, pg.doubleSupportPhase.time
#    pg.doubleSupportPhase.recompute(t)
#    pg.SupportFoot.recompute(t)
    incontac=globals()['incontact']
    if (incontac==left) & (pg.doubleSupportPhase.value==1):
        print ' ************************************************************************* t = ',t,': Added RF'
        print 'contacts = ', incontac, pg.SupportFoot.value, pg.doubleSupportPhase.value
        sot.rm(taskRF.task.name)
        sot.addContactFromTask(contactRF.task.name,'RF')
        sot._RF_p.value = contactRF.support
        incontac=both
    else:
        if (incontac==right) & (pg.doubleSupportPhase.value==1):
            print ' ************************************************************************* t = ',t,': Added LF'
            print 'contacts = ', incontac, pg.SupportFoot.value, pg.doubleSupportPhase.value
            sot.rm(taskLF.task.name)
            sot.addContactFromTask(contactLF.task.name,'LF')
            sot._LF_p.value = contactLF.support
            incontac=both
        else:
            if (incontac==both) & (pg.doubleSupportPhase.value==0):
                pg.SupportFoot.recompute(t)
                if pg.SupportFoot.value==right:
                    print ' ************************************************************************* t = ',t,': UP LF'
                    print 'contacts = ', incontac, pg.SupportFoot.value, pg.doubleSupportPhase.value
                    sot.rmContact('LF')
                    sot.push(taskLF.task.name)
                    incontac=right
                else: # pg.SupportFoot.value==left
                    print ' ************************************************************************* t = ',t,': UP RF'
                    print 'contacts = ', incontac, pg.SupportFoot.value, pg.doubleSupportPhase.value
                    sot.rmContact('RF')
                    sot.push(taskRF.task.name)
                    incontac=left
    globals()['incontact']=incontac
    robot.increment(dt)
    qs.append(robot.state.value)

from ThreadInterruptibleLoop import *
@loopInThread
def loop():
    inc()
runner=loop()

@optionalparentheses
def go():
    if runner.isQuit:
        globals()['runner']=loop()
    runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc() #runner.once()

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
@optionalparentheses
def n():
    inc()
    qdot()
@optionalparentheses
def n5():
    for loopIdx in range(5): inc()
@optionalparentheses
def n10():
    for loopIdx in range(10): inc()


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

# --- SOT Dyn OpSpaceH --------------------------------------
# SOT controller.
sot = SolverOpSpace('sot')
sot.setSize(robotDim-6)
#sot.damping.value = 2e-2
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

plug(sot.control,robot.control)
# For the integration of q = int(int(qddot)).
plug(sot.acceleration,robot.acceleration)

def sotContactList(sot):
    '''Return a list of the contact names for the solver. '''
    return map(lambda x: x[1:-1],sot.dispContacts().split('|')[1:])
SolverOpSpace.contacts = sotContactList

# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator import *

pg = PatternGenerator('pg')
pg.setVrmlDir(modelDir+'/')
pg.setVrml(modelName[robotName])
pg.setXmlSpec(specificitiesPath)
pg.setXmlRank(jointRankPath)
pg.buildModel()

# Standard initialization
pg.parseCmd(":samplingperiod 0.005")
pg.parseCmd(":previewcontroltime 1.6")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":omega 0.0")
pg.parseCmd(":stepheight 0.05")
pg.parseCmd(":singlesupporttime 0.780")
pg.parseCmd(":doublesupporttime 0.020")
pg.parseCmd(":armparameters 0.5")
pg.parseCmd(":LimitsFeasibility 0.0")
pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

plug(dyn.position,pg.position)
plug(dyn.com,pg.com)
pg.motorcontrol.value = robotDim*(0,)
pg.zmppreviouscontroller.value = (0,0,0)

pg.initState()

# --- REFERENCES ---------------------------------------------------------------
# --- Selector of Com Ref: when pg is stopped, pg.inprocess becomes 0
comRef = Selector('comRef'
                  ,['vector','ref',dyn.com,pg.comref])
plug(pg.inprocess,comRef.selec)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.parseCmd(':SetAlgoForZmpTrajectory Herdt')
pg.parseCmd(':doublesupporttime 0.1')
pg.parseCmd(':singlesupporttime 0.7')
# When velocity reference is at zero, the robot stops all motion after n steps
pg.parseCmd(':numberstepsbeforestop 4')
# Set constraints on XY
pg.parseCmd(':setfeetconstraint XY 0.09 0.06')
# Start the robot with a speed of 0.1 m/0.8 s.
pg.parseCmd(':HerdtOnline 0.1 0.0 0.0')
# You can now modifiy the speed of the robot using set pg.velocitydes [3]( x, y, yaw)
pg.velocitydes.value =(0.1,0.0,0.0)

# ---- TASKS -------------------------------------------------------------------

# ---- CONTACT -----------------------------------------
# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.feature.frame('desired')

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactRF.feature.frame('desired')

# ---- WAIST TASK ---
taskWaist=MetaTaskDyn6d('waist',dyn,'waist','waist')

# Build the reference waist pos homo-matrix from PG.
waistReferenceVector = Stack_of_vector('waistReferenceVector')
plug(pg.initwaistposref,waistReferenceVector.sin1)
plug(pg.initwaistattref,waistReferenceVector.sin2)
waistReferenceVector.selec1(0,3)
waistReferenceVector.selec2(0,3)
waistReference=PoseRollPitchYawToMatrixHomo('waistReference')
plug(waistReferenceVector.sout,waistReference.sin)
plug(waistReference.sout,taskWaist.featureDes.position)

taskWaist.feature.selec.value = '011100'
taskWaist.task.controlGain.value = 5000

# --- TASK COM ---
featureCom = FeatureGeneric('featureCom')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureComDes = FeatureGeneric('featureComDes')
featureCom.sdes.value = 'featureComDes'
plug(comRef.ref,featureComDes.errorIN)
featureCom.selec.value = '011'

taskCom = TaskDynPD('taskCom')
taskCom.add('featureCom')
#plug(pg.dcomref,featureComDes.errordotIN)
#plug(featureCom.errordot,taskCom.errorDot)
taskCom.controlGain.value = 5000
#taskCom.setBeta(-1)
plug(robot.velocity,taskCom.qdot)
taskCom.dt.value = dt

# --- TASK RIGHT FOOT
# Task right hand
taskRF=MetaTaskDyn6d('rf',dyn,'rf','right-ankle')
plug(pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 5000

taskLF=MetaTaskDyn6d('lf',dyn,'lf','left-ankle')
plug(pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 5000

# --- RUN ------------------------------------------------
sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = contactLF.support
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = contactRF.support

sot.push(taskCom.name)
sot.push(taskWaist.task.name)

# --- TRACER -----------------------------------------------------------------
from dynamic_graph.tracer import *
from dynamic_graph.tracer_real_time import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')
tr.add('pg.doubleSupportPhase','')
tr.add('pg.SupportFoot','')

robot.before.addSignal('pg.SupportFoot')
robot.before.addSignal('pg.doubleSupportPhase')
