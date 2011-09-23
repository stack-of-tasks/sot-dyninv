from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from numpy import *
from matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY, RPYToMatrix
from history import History
from zmp_estimator import ZmpEstimator
from viewer_helper import addRobotViewer,VisualPinger
from attime import attime

from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
robotName = 'hrp14small'

initialConfig.clear()
initialConfig['hrp14small'] = (0,0,0.648697398115,0,0,0)+(0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, -0.4538, 0.8727, -0.4189, 0, 0, 0, 0, 0, 0.2618, -0.1745, 0, -0.5236, 0, 0, 0.17453, 0.2618, 0.1745, 0, -0.5236, 0, 0, 0.17453 )

finalConfig = (0,0,0.648697398115,0,0,0)+(-0.00028574496353194379, 0.003829437078692717, -0.64798319907006197, 1.0552418879543297, -0.44497846451811773, -0.0038397195926406333, -0.00028578259876648154, 0.00382843982057902, -0.64712828870988759, 1.053420252598515, -0.44401173937761496, -0.0038387216246142633, 0.00014352031102940873, 0.013151503268542629, -0.00057411504064778878, -0.050871000025830684, 0.21782780288468376, -0.37965640592660427, -0.14072647716205641, -1.1942332339535875, 0.0055454863752303456, -0.66956710808063102, 0.17479818266043898, 0.21400703176401134, 0.38370527720074465, 0.14620204468511225, -1.1873407322936731, -0.0038746980026916284, -0.6643017236637716, 0.17500428384102076)

qf=array(finalConfig)
qf[6]-=1
finalConfig = tuple(qf.tolist())

# --- ROBOT SIMU ---------------------------------------------------------------

robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)
addRobotViewer(robot,small=True,verbose=False)

robot.set( initialConfig[robotName] )
dt=5e-3

# --- MAIN LOOP ------------------------------------------
def inc():
    robot.increment(dt)
    attime.run(robot.control.time)
    #robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
    if dyn.com.time >0:
        robot.viewer.updateElementConfig('com',[dyn.com.value[0],dyn.com.value[1],0,0,0,0])

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
def n():
    inc()
    qdot()
@optionalparentheses
def n5():
    for loopIdx in range(5): inc()
@optionalparentheses
def n10():
    for loopIdx in range(10): inc()
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
dyn.setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value = gearRatio[robotName]

plug(robot.state,dyn.position)
dyn.velocity.value = robotDim*(0.,)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()
robot.control.unplug()

# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator import PatternGenerator,Selector

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

# --- Selector of Com Ref: when pg is stopped, pg.inprocess becomes 0
comRef = Selector('comRef'
                    ,['vector','ref',dyn.com,pg.comref])
plug(pg.inprocess,comRef.selec)

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
# Connect the ZMPref to OpenHRP in the waist reference frame.
pg.parseCmd(':SetZMPFrame world')
plug(wa_zmp.sout,robot.zmp)

# ---- TASKS -------------------------------------------------------------------

# ---- WAIST TASK ---
taskWaist=MetaTask6d('waist',dyn,'waist','waist')

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
taskCom = Task('taskCom')
taskCom.add('featureCom')
taskCom.controlGain.value = 40

# --- TASK POSTURE ---
featurePosture = FeatureGeneric('featurePosture')
plug(dyn.position,featurePosture.errorIN)
featurePosture.jacobianIN.value = matrixToTuple(eye(robotDim))
featurePostureDes = FeatureGeneric('featurePostureDes')
featurePosture.sdes.value = 'featurePostureDes'
featurePostureDes.errorIN.value = finalConfig

gainPosture = GainAdaptive('gainPosture')
gainPosture.setByPoint(10,0.1,0.1,0.8)

taskPosture = Task('taskPosture')
taskPosture.add('featurePosture')
plug(taskPosture.error,gainPosture.error)
plug(gainPosture.gain,taskPosture.controlGain)

# --- TASK RIGHT FOOT
# Task right hand
taskRF=MetaTask6d('rf',dyn,'rf','right-ankle')
taskLF=MetaTask6d('lf',dyn,'lf','left-ankle')

plug(pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 5
plug(pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 5

# ---- SOT ---------------------------------------------------------------------
# The solver SOTH of dyninv is used, but normally, the SOT solver should be sufficient
from dynamic_graph.sot.dyninv import SolverKine
sot = SolverKine('sot')
sot.setSize(robotDim)
#sot.push(taskWaist.task.name)
sot.push(taskRF.task.name)
sot.push(taskLF.task.name)
sot.push(taskCom.name)

plug(sot.control,robot.control)




# --- TRACER -----------------------------------------------------------------
from dynamic_graph.tracer import *
from dynamic_graph.tracer_real_time import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')

#tr.add(dyn.name+'.ffposition','ff')
tr.add(taskRF.featureDes.name+'.position','refr')
tr.add(taskLF.featureDes.name+'.position','refl')


# --- DYN ----------------------------------------------------------------------
inv=linalg.inv

q={}; geom={}; rf={}; lf={};

q[0] = initialConfig[robotName]
q[1] = finalConfig
for i in range(2):
    geom[i] = Dynamic("geom"+str(i))
    geom[i].setFiles(modelDir, modelName[robotName],specificitiesPath,jointRankPath)
    geom[i].parse()
    geom[i].position.value = q[i]
    geom[i].velocity.value = robotDim*(0.,)
    geom[i].acceleration.value = robotDim*(0.,)
    geom[i].createPosition('rf','right-ankle')
    geom[i].createPosition('lf','left-ankle')
    geom[i].rf.recompute(0)
    geom[i].lf.recompute(0)
    rf[i]= array(geom[i].rf.value)
    lf[i]= array(geom[i].lf.value)



aMm0 = eye(4); aMm0[0:2,3] = (rf[0][0:2,3]+lf[0][0:2,3])/2
bMm1 = eye(4); bMm1[0:2,3] = (rf[1][0:2,3]+lf[1][0:2,3])/2
wMa = inv(aMm0)
wMm1 = eye(4); wMm1[0,3]=0.2
wMb = dot(wMm1,inv(bMm1))

wMr0 = dot( wMa, rf[0] )
wMl0 = dot( wMa, lf[0] )
wMr1 = dot( wMb, rf[1] )
wMl1 = dot( wMb, lf[1] )

if abs(wMr0[2,2]-1)>1e-3 or abs(wMl0[2,2]-1)>1e-3 or abs(wMr1[2,2]-1)>1e-3 or abs(wMl1[2,2]-1)>1e-3 :
    print 'Error: the feet '

step1 = (   dot(wMa,lf[0])   )[0:2,3]
step2 = (  dot( inv(   dot(wMa,lf[0])   ),  (dot( wMb,rf[1]))   ) )   [0:2,3]
step3 = ( dot( inv(rf[1]), lf[1] ))[0:2,3]

'''
seqpart = ''
for s in [step1,step2, step3]:
    seqpart+= str(s[0])+' '+str(s[1])+' '+str(0)+' '
pg.parseCmd(":stepseq " + seqpart)
'''

rMl0 = dot(   inv(rf[0]),  lf[0] )  
rMl1 = dot(   inv(rf[1]),  lf[1] )  
if abs(rMl1[2,2]-1)>1e-3 or abs(rMl1[2,3])>1e-3:
    print 'Error: the feet should be parallele on a same Z plane (final pose)'
if abs(rMl0[2,2]-1)>1e-3 or abs(rMl0[2,3])>1e-3:
    print 'Error: the feet should be parallele on a same Z plane (initial pose)'

a0 = arctan2(rMl0[1,0],rMl0[0,0])/2
a1 = arctan2(rMl1[1,0],rMl1[0,0])/2
t0 = rMl0[0:3,3]
t1 = rMl1[0:3,3]

Ma0 = array(rotate('z',a0))
Ma0[0:3,3] = t0/2
Ma1 = array(rotate('z',a1))
Ma1[0:3,3] = t1/2

wMa = inv(dot(Ma0,rf[0]))
wMb = inv(dot(Ma1,rf[1]))

#robot.set( tuple(matrixToRPY(   dot(wMa,RPYToMatrix(q[0][0:6])) ))+q[0][6:] )
#robot.set( tuple(matrixToRPY(   dot(wMb,RPYToMatrix(q[1][0:6])) ))+q[1][6:] )

step1 = (   dot(wMa,lf[0])   )
step2 = (  dot( inv(   dot(wMa,lf[0])   ),  (dot( wMb,rf[1]))   ) )
step3 = ( dot( inv(rf[1]), lf[1] ))

seqpart = ''
for s in [step1,step2, step3]:
    seqpart+= str(s[0,3])+' '+str(s[1,3])+' '+str(arctan2(s[1,0],s[0,0])*180/pi)+' '
pg.parseCmd(":stepseq " + seqpart)


# --- RUN -----------------------------------------------------------------

go()


featurePosture.selec.value = toFlags( range(6,36) )
attime(300,lambda: sot.push(taskPosture.name),'Add Posture')
attime(500,lambda: sot.rm(taskCom.name),'rm com')
