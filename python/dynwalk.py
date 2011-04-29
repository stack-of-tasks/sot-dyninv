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
def inc():
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

dyn.createOpPoint('rf','right-ankle')
dyn.createOpPoint('lf','left-ankle')

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

# --- PG INIT FRAMES ---
geom = Dynamic("geom")
geom.setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
geom.parse()
geom.createOpPoint('rf','right-ankle')
geom.createOpPoint('lf','left-ankle')
plug(dyn.position,geom.position)
geom.ffposition.value = 6*(0,)
geom.velocity.value = robotDim*(0,)
geom.acceleration.value = robotDim*(0,)

#from dynamic_graph.sot.pattern_generator.meta_selector import SelectorPy
#import dynamic_graph.sot.pattern_generator.meta_selector
# --- Selector of the foot on the ground
SelectorPy=Selector
selecSupportFoot = Selector('selecSupportFoot'
                            ,['matrixHomo','pg_H_sf',pg.rightfootref,pg.leftfootref]
                            ,['matrixHomo','wa_H_sf',geom.rf,geom.lf]
                            )

plug(pg.SupportFoot,selecSupportFoot.selec)
sf_H_wa = Inverse_of_matrixHomo('sf_H_wa')
plug(selecSupportFoot.wa_H_sf,sf_H_wa.sin)
pg_H_wa = Multiply_of_matrixHomo('pg_H_wa')
plug(selecSupportFoot.pg_H_sf,pg_H_wa.sin1)
plug(sf_H_wa.sout,pg_H_wa.sin2)

# --- Compute the ZMP ref in the Waist reference frame.
wa_H_pg = Inverse_of_matrixHomo('wa_H_pg')
plug(pg_H_wa.sout,wa_H_pg.sin)
wa_zmp = Multiply_matrixHomo_vector('wa_zmp')
plug(wa_H_pg.sout,wa_zmp.sin1)
plug(pg.zmpref,wa_zmp.sin2)

# --- PLUGS PG --------------------------------------------------------
# --- Pass the dyn from ref left_foot to ref pg.
ffpos_from_pg = MatrixHomoToPoseRollPitchYaw('ffpos_from_pg')
plug(pg_H_wa.sout,ffpos_from_pg.sin)
plug(ffpos_from_pg.sout,dyn.ffposition)

# --- Connect the ZMPref to OpenHRP in the waist reference frame.
pg.parseCmd(':SetZMPFrame world')
plug(wa_zmp.sout,robot.zmp)

# --- Extract pose and attitude from ffpos
ffattitude_from_pg = Selec_of_vector('ffattitude_from_pg')
plug(ffpos_from_pg.sout,ffattitude_from_pg.sin)
ffattitude_from_pg.selec(3,6)
plug(ffattitude_from_pg.sout,robot.attitudeIN)

# --- REFERENCES ---------------------------------------------------------------
# --- Selector of Com Ref: when pg is stopped, pg.inprocess becomes 0
comRef = SelectorPy('comRef'
                     ,['vector','ref',dyn.com,pg.comref])
plug(pg.inprocess,comRef.selec)


footSelection = SelectorPy('refFootSelection'
                              ,[ 'matrixHomo','desFoot',pg.rightfootref,pg.leftfootref] # Ref of the flying foot
                              ,[ 'matrixHomo','desRefFoot',pg.leftfootref,pg.rightfootref] # Ref of the support foot
                              ,[ 'matrixHomo','foot',dyn.rf,dyn.lf]  # actual pos of the flying foot
                              ,[ 'matrixHomo','refFoot',dyn.lf,dyn.rf] # actual pos of the support foot
                              ,['matrix','Jfoot',dyn.Jrf,dyn.Jlf] # jacobian of .foot
                              ,['matrix','JrefFoot',dyn.Jlf,dyn.Jrf] # jacobian of .refFoot
                              )
plug(pg.SupportFoot,footSelection.selec)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.parseCmd(':SetAlgoForZmpTrajectory Herdt')
pg.parseCmd(':doublesupporttime 0.1')
pg.parseCmd(':singlesupporttime 0.7')
# When velocity reference is at zero, the robot stops all motion after n steps
pg.parseCmd(':numberstepsbeforestop 4')
# Set constraints on XY
pg.parseCmd(':setfeetconstraint XY 0.09 0.06')

# The next command must be runned after a OpenHRP.inc ... ???
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
taskWaist.task.controlGain.value = 5

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
taskCom.controlGain.value = 400
#taskCom.setBeta(-1)
plug(robot.velocity,taskCom.qdot)
taskCom.dt.value = dt

# --- TASK RIGHT FOOT
# Task right hand
taskRF=MetaTaskDyn6d('rf',dyn,'rf','right-ankle')
taskLF=MetaTaskDyn6d('lf',dyn,'lf','left-ankle')

plug(pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 5
plug(pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 5

# --- RUN ------------------------------------------------
sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = contactLF.support
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = contactRF.support

sot.push(taskCom.name)


robot.after.addSignal('comRef.ref')
#robot.control.value = (robotDim-6)*(0,)
#robot.acceleration.value = (robotDim)*(0,)
