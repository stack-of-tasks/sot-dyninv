#_____________________________________________________________________________________________
#********************************************************************************************
#
#  Robot motion (HRP2 14 small) using:
#  - ONLY OPERATIONAL TASKS
#  - Joint limits (position and velocity)
#_____________________________________________________________________________________________
#*********************************************************************************************

import sys
from optparse import OptionParser
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
sys.path.append('..')
from dynamic_graph.sot.core.meta_task_6d import toFlags
from meta_task_dyn_6d import MetaTaskDyn6d
from meta_tasks_dyn import *
from attime import attime
from numpy import *
from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
from matrix_util import matrixToTuple, vectorToTuple,rotate
from history import History
from zmp_estimator import ZmpEstimator
from viewer_helper import addRobotViewer,VisualPinger

#-----------------------------------------------------------------------------
# --- ROBOT SIMU -------------------------------------------------------------
#-----------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim  = robotDimension[robotName]
RobotClass = RobotDynSimu
robot      = RobotClass("robot")
robot.resize(robotDim)
addRobotViewer(robot,small=True,verbose=False)
dt = 5e-3

# Similar initial position with hand forward
robot.set((-0.033328803958899381, -0.0019839923040723341, 0.62176527349722499, 2.379901541270165e-05, 0.037719492175904465, 0.00043085147714449579, -0.00028574496353126724, 0.0038294370786961648, -0.64798319906979551, 1.0552418879542016, -0.44497846451873851, -0.0038397195926379991, -0.00028578259876671871, 0.0038284398205732629, -0.64712828871069394, 1.0534202525984278, -0.4440117393779604, -0.0038387216246160054, 0.00014352031102944824, 0.013151503268540811, -0.00057411504064861592, -0.050871000025766742, 0.21782780288481224, -0.37965640592672439, -0.14072647716213352, -1.1942332339530364, 0.0055454863752273523, -0.66956710808008013, 0.1747981826611808, 0.21400703176352612, 0.38370527720078107, 0.14620204468509851, -1.1873407322935838, -0.0038746980026940735, -0.66430172366423146, 0.17500428384087438))

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

def inc():
    robot.increment(dt)
    # Execute a function at time t, if specified with t.add(...)
    if 'refresh' in ZmpEstimator.__dict__: zmp.refresh()
    attime.run(robot.control.time)
    robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
    if dyn.com.time >0:
        robot.viewer.updateElementConfig('com',[dyn.com.value[0],dyn.com.value[1],0,0,0,0])
    history.record()

from ThreadInterruptibleLoop import *
@loopInThread
def loop():
#    try:
        inc()
#    except:
#        print robot.state.time,': -- Robot has stopped --'
runner=loop()

@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc()

# --- shortcuts -------------------------------------------------
@optionalparentheses
def q():
    print robot.state.__repr__()
@optionalparentheses
def qdot(): print robot.control.__repr__()
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay

@optionalparentheses
def pl():         print  matlab(  matrixToTuple(zmp.leftSupport()) ).resstr
@optionalparentheses
def pr():         print  matlab(  matrixToTuple(zmp.rightSupport()) ).resstr

@optionalparentheses
def dump():
    history.dumpToOpenHRP('openhrp/screenwash')

attime.addPing( VisualPinger(robot.viewer) )

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

dyn.lowerJl.recompute(0)
dyn.upperJl.recompute(0)
llimit = matrix(dyn.lowerJl.value)
ulimit = matrix(dyn.upperJl.value)
dlimit = ulimit-llimit
mlimit = (ulimit+llimit)/2
llimit[6:18] = mlimit[6:12] - dlimit[6:12]*0.48
dyn.upperJl.value = vectorToTuple(ulimit)

dyn.inertiaRotor.value = inertiaRotor[robotName]
dyn.gearRatio.value    = gearRatio[robotName]

plug(robot.state,dyn.position)
plug(robot.velocity,dyn.velocity)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)---------------------------------------------
#-----------------------------------------------------------------------------

# --- Op task for the waist ------------------------------
taskWaist = MetaTaskDyn6d('taskWaist',dyn,'waist','waist')
taskChest = MetaTaskDyn6d('taskChest',dyn,'chest','chest')
taskHead = MetaTaskDyn6d('taskHead',dyn,'head','gaze')
taskrh = MetaTaskDyn6d('rh',dyn,'rh','right-wrist')
tasklh = MetaTaskDyn6d('lh',dyn,'lh','left-wrist')
taskrf = MetaTaskDyn6d('rf',dyn,'rf','right-ankle')

for task in [ taskWaist, taskChest, taskHead, taskrh, tasklh, taskrf ]:
    task.feature.frame('current')
    task.gain.setConstant(50)
    task.task.dt.value = dt

# --- TASK COM ------------------------------------------------------
taskCom = MetaTaskDynCom(dyn,dt)

# --- TASK POSTURE --------------------------------------------------
taskPosture = MetaTaskDynPosture(dyn,dt)

# --- Task lim ---------------------------------------------------
taskLim = TaskDynLimits('taskLim')
plug(dyn.position,taskLim.position)
plug(dyn.velocity,taskLim.velocity)
taskLim.dt.value = dt

dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskLim.referencePosInf.value = dyn.lowerJl.value
taskLim.referencePosSup.value = dyn.upperJl.value

#dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240, 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
dqup = (1000,)*robotDim
taskLim.referenceVelInf.value = tuple([-val*pi/180 for val in dqup])
taskLim.referenceVelSup.value = tuple([val*pi/180 for val in dqup])



taskSupport = TaskDynInequality('taskSupport')
featureSupport = FeatureGeneric('featureSupport')
plug(dyn.com,featureSupport.errorIN)
plug(dyn.Jcom,featureSupport.jacobianIN)

taskSupport.add(featureSupport.name)
taskSupport.selec.value = '011'
taskSupport.referenceInf.value = (-0.08,-0.045,0)    # Xmin, Ymin
taskSupport.referenceSup.value = (0.11,0.335,0)  # Xmax, Ymax
taskSupport.dt.value=dt
plug(dyn.velocity,taskSupport.qdot)





#-----------------------------------------------------------------------------
# --- SOT Dyn OpSpaceH: SOT controller  --------------------------------------
#-----------------------------------------------------------------------------

sot = SolverDynReduced('sot')
contact = AddContactHelper(sot)

sot.setSize(robotDim-6)
#sot.damping.value = 1e-2
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

plug(sot.solution, robot.control)

#For the integration of q = int(int(qddot)).
plug(sot.acceleration,robot.acceleration)

#-----------------------------------------------------------------------------
# ---- CONTACT: Contact definition -------------------------------------------
#-----------------------------------------------------------------------------

# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.feature.frame('desired')
contactLF.gain.setConstant(1000)
contactLF.name = "LF"

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')
contactRF.name = "RF"

contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactLF.support =  ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))

#--- ZMP ---------------------------------------------------------------------
zmp = ZmpEstimator('zmp')
zmp.declare(sot,dyn)

#-----------------------------------------------------------------------------
# --- TRACE ------------------------------------------------------------------
#-----------------------------------------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','screenwash_','.dat')

tr.add('dyn.com','com')
tr.add(taskCom.feature.name+'.error','ecom')
tr.add('dyn.waist','waist')
tr.add('dyn.rh','rh')
tr.add('zmp.zmp','')
tr.add('dyn.position','')
tr.add('dyn.velocity','')
tr.add('robot.acceleration','robot_acceleration')
tr.add('robot.control','')
tr.add(taskCom.gain.name+'.gain','com_gain')
tr.add(taskrf.gain.name+'.gain','rf_gain')

tr.add('dyn.lf','lf')
tr.add('dyn.rf','rf')

tr.start()
robot.after.addSignal('tr.triger')
robot.after.addSignal(contactLF.task.name+'.error')
robot.after.addSignal('dyn.rf')
robot.after.addSignal('dyn.lf')
robot.after.addSignal('dyn.com')
robot.after.addSignal('sot.forcesNormal')
robot.after.addSignal('dyn.waist')

robot.after.addSignal('taskLim.normalizedPosition')
tr.add('taskLim.normalizedPosition','qn')

tr.add('taskSupport.task','icom')
tr.add('taskSupport.errorDot','comdot')

history = History(dyn,1,zmp.zmp)

#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

DEG = 180.0/pi

sot.clear()
contact(contactLF)
contact(contactRF)

taskCom.feature.selec.value = "11"
taskCom.gain.setByPoint(100,10,0.005,0.8)

taskSupport.referenceInf.value = (-0.015,-0.09,0)    # Xmin, Ymin
taskSupport.referenceSup.value = (0.035,0.09,0)  # Xmax, Ymax
taskSupport.controlGain.value = 10

mrh=eye(4)
mrh[0:3,3] = (0,0,-0.18)
taskrh.opmodif = matrixToTuple(mrh)
taskrh.gain.setByPoint(120,5,0.03,0.8)
drh=array(matrix(rotate('x',-45/DEG))*matrix(rotate('y',180/DEG)))
taskrh.ref = matrixToTuple(drh)
taskrh.feature.selec.value = '111000'

tasklh.opmodif = matrixToTuple(mrh)
tasklh.gain.setByPoint(120,5,0.03,0.8)
dlh=array(matrix(rotate('x',+45/DEG))*matrix(rotate('y',180/DEG)))
tasklh.ref = matrixToTuple(dlh)
tasklh.feature.selec.value = '111000'

taskWaist.gain.setByPoint(60,5,0.01,0.8)
taskWaist.feature.frame('desired')

#sot.push(taskLim.name)
plug(robot.state,sot.position)

[s.recompute(0) for s in robot.state, dyn.com, dyn.rh, dyn.lh ]

q0 = robot.state.value
com0 = dyn.com.value
rh0 = dyn.rh.value

# --- Events ---------------------------------------------
sigset = ( lambda s,v : s.__class__.value.__set__(s,v) )
refset = ( lambda mt,v : mt.__class__.ref.__set__(mt,v) )

attime(2
       ,(lambda : sot.push(taskSupport.name),"Add Support")
#       ,(lambda : sot.push(taskCom.task.name),"Add COM")
#       ,(lambda : refset(taskCom, com0), "Com to rest")
       ,(lambda: sot.push(taskrh.task.name), "Add RH")
       ,(lambda: sot.push(tasklh.task.name), "Add LH")
       )

attime(400
       ,(lambda: gotoNd(taskrh,(),'110111'),'RH to 6D')
       ,(lambda: taskrh.feature.keep(),'keep RH ')
       ,(lambda: gotoNd(tasklh,(),'110111'),'LH to 6D')
       ,(lambda: tasklh.feature.keep(),'keep LH ')
       ,(lambda: sot.push(taskWaist.task.name),'Add waist')
       ,(lambda: gotoNd(taskWaist,(0,0.0,0.65),'110'),'Waist to up')
)

attime(500,lambda: gotoNd(taskWaist,(0,0.05,0.62),'110'),'Waist to left')
attime(600,lambda: gotoNd(taskWaist,(0,0.05,0.42),'110'),'Waist to bottom left')
attime(800,lambda: gotoNd(taskWaist,(0,-0.05,0.42),'110'),'Waist to bottom right')
attime(1000,lambda: gotoNd(taskWaist,(0,-0.05,0.6),'110'),'Waist to up right')

attime(1200,lambda: gotoNd(taskWaist,(0,0.05,0.62),'110'),'Waist to left')
attime(1300,lambda: gotoNd(taskWaist,(0,0.05,0.42),'110'),'Waist to bottom left')
attime(1500,lambda: gotoNd(taskWaist,(0,-0.05,0.42),'110'),'Waist to bottom right')
attime(1700,lambda: gotoNd(taskWaist,(0,-0.05,0.6),'110'),'Waist to up right')

attime(1900,lambda: gotoNd(taskWaist,(0,0.05,0.62),'110'),'Waist to left')
attime(2000,lambda: gotoNd(taskWaist,(0,0.05,0.42),'110'),'Waist to bottom left')
attime(2200,lambda: gotoNd(taskWaist,(0,-0.05,0.42),'110'),'Waist to bottom right')
attime(2400,lambda: gotoNd(taskWaist,(0,-0.05,0.6),'110'),'Waist to up right')
attime(2500,lambda: gotoNd(taskWaist,(0,0,0.6),'110'),'Waist to up center')

attime(2650
       ,(lambda: sigset(sot.posture,q0), "Robot to initial pose")
       ,(lambda: sot.rm(taskrh.task.name), 'rm rh')
       ,(lambda: sot.rm(tasklh.task.name), 'rm lh')
       ,(lambda: sot.rm(taskWaist.task.name), 'rm waist')
)
attime(3200,stop)

'''
robot.set( (-0.0013393589475475609, -0.0018793714660677973, 0.61652696640204518, 4.077168349517344e-06, -0.079368382946045368, -0.0014814482541992021, 0.0011854695519697305, 0.003776172460719502, -0.49196116106248722, 1.0999658010722786, -0.52863577122262018, -0.0036743472676854902, 0.0011855230223719129, 0.0037755056968030633, -0.49079316286050761, 1.0983457361725071, -0.52818370472537191, -0.0036736775362441851, -0.0029092387929590996, -0.087265999992735224, -0.00060561022632208046, -0.04935484475912507, -0.095740823457753019, -0.52701828684129937, 0.22402988804334981, -1.6948942299007437, 0.2352332739763284, -1.2009080463253117, 0.17478139949422683, -0.09942591769293331, 0.530365237032702, -0.21874120188824195, -1.690756503538019, -0.23210164421705265, -1.197643906783777, 0.17497037278714084) )
robot.setVelocity( (0.0023459312192029794, 1.9324966075519278e-05, -0.00037965857165365497, 3.1087772088106599e-05, -0.0073616238519256267, -0.00020406983753833795, 0.00018165236673074378, -5.204782726283328e-05, 0.010528166504417158, 0.0027557570165100344, -0.0059222856601468107, 3.5072230322404451e-05, 0.00018165007897321355, -5.2003973276289372e-05, 0.01058113012464732, 0.002741687094005327, -0.0059611793537690284, 3.5020815320320569e-05, -0.0003792468157678192, 3.4842735753219725e-10, -2.0781622603772377e-06, 0.00010493125566758389, -0.021172740376247282, -0.006921516001430616, 0.018677271130027481, -0.031784214644352986, 0.012085900823845322, -0.035707836241261129, -1.2513673688466769e-07, -0.021638776823933382, 0.007167064224579126, -0.018628508289441001, -0.032435947772568627, -0.012368040323395753, -0.036291042542633656, -3.0717889727652116e-07) )
T0 =  400
robot.state.time = T0
[ t.feature.position.recompute(T0) for t in taskrh,tasklh]
attime.fastForward(T0)
'''

#attime(70, sot.debugOnce)
attime(75,stop)

def check():
    J = matrix(taskSupport.jacobian.value)
    Jdot = matrix(taskSupport.Jdot.value)
    dq = matrix(dyn.velocity.value).T
    ddq = matrix(sot.acceleration.value).T
    print J*ddq+Jdot*dq
    print taskSupport.task.value


#sot.rm(taskSupport.name)
#sot.rm(taskrh.task.name)
#sot.rm(tasklh.task.name)

#print sot
#inc()

#check()
#sot.debugOnce()
#inc()
#go()



