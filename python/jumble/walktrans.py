'''
Walking transition: give an initial posture and final posture, the robot will
make one step to go from one to the other.
'''

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
from optparse import OptionParser

from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor,halfSittingConfig
robotName = 'hrp14small'


# --- ROBOT SIMU ---------------------------------------------------------------
robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)
addRobotViewer(robot,small=True,verbose=False)
dt=5e-3

modelDir = pkgDataRootDir[robotName]
xmlDir = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath = xmlDir + '/HRP2LinkJointRankSmall.xml'

# --- OPTIONS ------------------------------------------------------------------

usage = "usage: py walktrans.py [options] args"
parser = OptionParser(usage=usage)

parser.add_option("-i", dest="init", help="Initial position, being a list of joint angles, or a filename whose last line is a list of angles, or halfsitting.", default="halfsitting")
parser.add_option("-f", dest="final", help="Final position, being a list of joint angles, or a filename whose first line is a list of angles, or halfsitting.", default="halfsitting")
parser.add_option("-d", dest="direction", help="Direction, forward or backward", default="forward")
parser.add_option("-o", dest="output", help="Prefix of the output", default="/tmp/walktrans")
(options, args) = parser.parse_args()
parser.print_help()
print '\n -------------------------------------------------------------------------'

q={}
if options.init == "halfsitting":
    q[0] = halfSittingConfig[robotName]
else:
    try:
        finit = open(options.init,'r')
        q[0] = tuple([float(s) for s in finit.readlines()[-1].split() ])
        finit.close()
    except:
        q[0] = tuple(options.init)
if len(q[0])==robotDim-6: q[0] = 6*(0,)+q[0]
if len(q[0])==robotDim-6+10+1: q[0] = 6*(0,)+q[0][1:-10]

if options.final == "halfsitting":
    q[1] = halfSittingConfig[robotName]
else:
    try:
        ffinal = open(options.final,'r')
        q[1] = tuple([float(s) for s in ffinal.readlines()[0].split() ])
        ffinal.close()
    except:
        q[1] = tuple(options.init)
if len(q[1])==robotDim-6: q[1] = 6*(0,)+q[1]
if len(q[1])==robotDim-6+10+1: q[1] = 6*(0,)+q[1][1:-10]


# --- STEP CONSTRUCTION --------------------------------------------------------
inv=linalg.inv

geom={}; rf={}; lf={};

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

rMl0 = dot(   inv(rf[0]),  lf[0] )  
rMl1 = dot(   inv(rf[1]),  lf[1] )  
if abs(rMl1[2,2]-1)>1e-3 or abs(rMl1[2,3])>1e-3:
    print 'Error: the feet should be parallele on a same Z plane (final pose)'
if abs(rMl0[2,2]-1)>1e-3 or abs(rMl0[2,3])>1e-3:
    print 'Error: the feet should be parallele on a same Z plane (initial pose)'

a0 = arctan2(rMl0[1,0],rMl0[0,0])/2
a1 = arctan2(rMl1[1,0],rMl1[0,0])/2
t0 = rMl0[0:3,3]
t1 = rMl1[0:3,3]; 

if isinstance(options.direction,int) and options.direction == 1: options.direction = "forward"
if isinstance(options.direction,int) and options.direction == -1: options.direction = "backward"
if options.direction[0:3] == "for":  t1[0]-=0.2
if options.direction[0:3] == "bac":  t1[0]+=0.2

Ma0 = array(rotate('z',a0))
Ma0[0:3,3] = t0/2
Ma1 = array(rotate('z',a1))
Ma1[0:3,3] = t1/2

wMa = inv(dot(Ma0,rf[0]))
wMb = inv(dot(Ma1,rf[1]))


Mview = dot(wMa,RPYToMatrix(q[0][0:6]))
Mup = eye(4); Mup[2,3] = 0.105
Mview  = dot( Mup,Mview )

robot.set( tuple(matrixToRPY( Mview ))+q[0][6:] )

step1 = (   dot(wMa,lf[0])   )
step2 = (  dot( inv(   dot(wMa,lf[0])   ),  (dot( wMb,rf[1]))   ) )
step3 = ( dot( inv(rf[1]), lf[1] ))

seqpart = ''
for s in [step1,step2, step3]:
    si = inv(s)
    seqpart+= str(-si[0,3])+' '+str(-si[1,3])+' '+str(-arctan2(si[1,0],si[0,0])*180/pi)+' '


# --- MAIN LOOP ------------------------------------------
def inc():
    robot.increment(dt)
    attime.run(robot.control.time)
    #robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
    if dyn.com.time >0:
        robot.viewer.updateElementConfig('com',[dyn.com.value[0],dyn.com.value[1],0,0,0,0])
    history.record()

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
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay
@optionalparentheses
def dump():
    history.dumpToOpenHRP(options.output)

# --- DYN ----------------------------------------------------------------------
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

# --- META TASKS AND OP POINT ------------------------------------------------

taskRF=MetaTask6d('rf',dyn,'rf','right-ankle')
taskLF=MetaTask6d('lf',dyn,'lf','left-ankle')


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
#pg.initState()

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
featurePostureDes.errorIN.value = q[1]

gainPosture = GainAdaptive('gainPosture')
gainPosture.setByPoint(10,0.1,0.1,0.8)

taskPosture = Task('taskPosture')
taskPosture.add('featurePosture')
plug(taskPosture.error,gainPosture.error)
plug(gainPosture.gain,taskPosture.controlGain)

# --- TASK RIGHT FOOT
# Task right hand
#plug(pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 5
#plug(pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 5

# ---- SOT ---------------------------------------------------------------------
# The solver SOTH of dyninv is used, but normally, the SOT solver should be sufficient
from dynamic_graph.sot.dyninv import SolverKine
sot = SolverKine('sot')
sot.setSize(robotDim)
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
tr.add('dyn.rf','r')
tr.add('dyn.lf','l')

tr.add('featureComDes.errorIN','comref')
tr.add('dyn.com','com')
tr.add(taskWaist.gain.name+'.gain','gainwaist')

history = History(dyn,1)

# --- RUN -----------------------------------------------------------------

featurePosture.selec.value = toFlags( range(6,36) )

sot.clear()
for task in [taskWaist, taskRF, taskLF]:
    task.feature.position.recompute(0)
    task.feature.keep()
    task.feature.selec.value = '111111'
    sot.push(task.task.name)

taskWaist.ref = matrixToTuple(eye(4))
taskWaist.feature.selec.value = '111011'
taskWaist.gain.setByPoint(18,0.1,0.005,0.8)

attime(200
       ,(lambda : sot.rm(taskWaist.task.name),'Rm waist')
       ,(lambda : pg.initState(),'Init PG')
       ,(lambda : pg.parseCmd(":stepseq " + seqpart),'Push step list')
       ,(lambda : sot.push(taskCom.name),'Push COM')
       ,(lambda : plug(pg.rightfootref,taskRF.featureDes.position),'Plug RF')
       ,(lambda : plug(pg.leftfootref,taskLF.featureDes.position),'plug LF' )
)


#attime(700,lambda: sot.push(taskPosture.name),'Add Posture')
#attime(900,lambda: sot.rm(taskCom.name),'rm com')
#attime(1200,lambda: dump(),'dump!')
#attime(1200,lambda: sys.exit(),'Bye bye')
attime(1200,dump,stop)

go()


'''

sot.clear()


if 0:
    Mwref = eye(4)
    Mwref[0:2,3] = dyn.com.value[0:2]
    taskWaist.ref = matrixToTuple(Mwref)
    taskWaist.feature.selec.value = '111011'
    
    featureComDes.errorIN.value = featureCom.errorIN.value
    sot.push(taskCom.name)

if 1:
    taskWaist.ref = matrixToTuple(eye(4))
    taskWaist.feature.selec.value = '111011'
   
'''
