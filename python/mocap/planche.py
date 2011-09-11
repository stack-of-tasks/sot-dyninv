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

#-----------------------------------------------------------------------------
# --- ROBOT SIMU -------------------------------------------------------------
#-----------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim  = robotDimension[robotName]
RobotClass = RobotDynSimu
robot      = RobotClass("robot")
robot.resize(robotDim)
dt = 5e-3



#half sitting
#qhs=matrix((0,0,0,0,0,0,  0,0,-26,50,-24,0,0,0,-26,50,-24,0,0,0,0,0,15,10,0,-30,0,0,10,15,-10,0,-30,0,0,10))
#robot.set((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4537856055185257, 0.87266462599716477, -0.41887902047863906, 0.0, 0.0, 0.0, -0.4537856055185257, 0.87266462599716477, -0.41887902047863906, 0.0, 0.0, 0.0, 0.0, 0.0, 0.26179938779914941, 0.17453292519943295, 0.0, -0.52359877559829882, 0.0, 0.0, 0.17453292519943295, 0.26179938779914941, -0.17453292519943295, 0.0, -0.52359877559829882, 0.0, 0.0, 0.17453292519943295))

# Similar initial position with hand forward
robot.set((-0.033328803958899381, -0.0019839923040723341, 0.62176527349722499, 2.379901541270165e-05, 0.037719492175904465, 0.00043085147714449579, -0.00028574496353126724, 0.0038294370786961648, -0.64798319906979551, 1.0552418879542016, -0.44497846451873851, -0.0038397195926379991, -0.00028578259876671871, 0.0038284398205732629, -0.64712828871069394, 1.0534202525984278, -0.4440117393779604, -0.0038387216246160054, 0.00014352031102944824, 0.013151503268540811, -0.00057411504064861592, -0.050871000025766742, 0.21782780288481224, -0.37965640592672439, -0.14072647716213352, -1.1942332339530364, 0.0055454863752273523, -0.66956710808008013, 0.1747981826611808, 0.21400703176352612, 0.38370527720078107, 0.14620204468509851, -1.1873407322935838, -0.0038746980026940735, -0.66430172366423146, 0.17500428384087438))

# ------------------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
# ------------------------------------------------------------------------------

try:
    import robotviewer

    def stateFullSize(robot):
        return [float(val) for val in robot.state.value]+10*[0.0]
    RobotClass.stateFullSize = stateFullSize
    robot.viewer = robotviewer.client('XML-RPC')
    #robot.viewer = robotviewer.client('CORBA')
    # Check the connection
    robot.viewer.updateElementConfig('hrp',robot.stateFullSize())

    def refreshView( robot ):
       robot.viewer.updateElementConfig('hrp',robot.stateFullSize())
    RobotClass.refresh = refreshView

    def incrementView(robot,dt):
        robot.incrementNoView(dt)
        robot.refresh()
        #if zmp.zmp.time > 0:
        #    robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
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
    class RobotViewerFaked:
        def update(*args): void
        def updateElementConfig(*args): void
    robot.viewer = RobotViewerFaked()


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
    if 'dyn' in globals(): print dyn.ffposition.__repr__()
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
def pr():         print  matlab(  matrixToTuple(zmp.righttSupport()) ).resstr

@optionalparentheses
def dump():
    history.dumpToOpenHRP('openhrp/planche')


# Add a visual output when an event is called.
class Ping:
    def __init__(self):
        self.pos = 1
        self.refresh()
    def refresh(self):
        robot.viewer.updateElementConfig('pong',  [ 0, 0.9, self.pos*0.1, 0, 0, 0 ])  
    def __call__(self):
        self.pos += 1
        self.refresh()
ping=Ping()
attime.addPing( ping )


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
tr.open('/tmp/','planche_','.dat')

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

history = History(dyn,1,zmp.zmp)

#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

DEG = 180.0/pi

# Angles for both knee, plus angle for the chest wrt the world.
MAX_EXT = 35/DEG
MAX_SUPPORT_EXT = 45/DEG
CHEST = 60/DEG # 80 ... 90?

sot.clear()
contact(contactLF)
contact(contactRF)

taskCom.feature.selec.value = "11"
taskCom.gain.setByPoint(100,10,0.005,0.8)

taskrf.feature.frame('desired')
taskrf.gain.setByPoint(40,2,0.002,0.8)

taskChest.feature.selec.value='111000'
taskChest.ref = rotate('y',CHEST)
taskChest.gain.setByPoint(30,3,1/DEG,0.8)

taskPosture.gain.setByPoint(30,3,1/DEG,0.8)

ql = matrix(dyn.lowerJl.value)
ql[0,15] = MAX_SUPPORT_EXT
taskLim.referencePosInf.value = vectorToTuple(ql)

sot.push(taskLim.name)
plug(robot.state,sot.position)

q0 = robot.state.value
rf0 = matrix(taskrf.feature.position.value)[0:3,3]

# --- Events ---------------------------------------------
sigset = ( lambda s,v : s.__class__.value.__set__(s,v) )
refset = ( lambda mt,v : mt.__class__.ref.__set__(mt,v) )

attime(2
       ,(lambda : sot.push(taskCom.task.name),"Add COM")
       ,(lambda : refset(taskCom, ( 0.01, 0.09,  0.7 )), "Com to left foot")
       )

attime(140
       ,(lambda: sot.rmContact("RF"),"Remove RF contact" )
       ,(lambda: sot.push(taskrf.task.name), "Add RF")
       ,(lambda: gotoNd(taskrf, (0,0,0.65),"000110" ), "Up foot RF")
       )

# Was -.8,0,.8 with full extension
attime(500,lambda: gotoNd(taskrf, (-0.75,0,0.7),"000101" ), "Extend RF")

attime(800,lambda: sot.push(taskChest.task.name), "add chest")

attime(1000
       ,(lambda: sot.rm(taskrf.task.name), "rm RF")
       ,(lambda: sot.push(taskPosture.task.name), "add posture")
       ,(lambda: taskPosture.gotoq(chest=(0,0),rleg=(0,0,0,MAX_EXT,0,0),head=(0,0)), "extend body")
       )

with_full_extention=False
if with_full_extention:
    attime(1300
           ,lambda: taskPosture.gotoq(chest=(0,0),rleg=(0,0,0,MAX_EXT,0,0),head=(0,0),rarm=(0,-pi/2,0,0,0,0),larm=(0,pi/2,0,0,0,0)), "extend arm")

    attime(2000
           ,lambda: taskPosture.gotoq(30,chest=(0,0),rleg=(0,0,0,MAX_EXT,0.73,0),head=(0,0),rarm=(0,-pi/2,0,0,0,0),larm=(0,pi/2,0,0,0,0)), "extend foot")
    tex=1000
else: tex=0


attime(1500 + tex
        ,(lambda: sot.rm(taskChest.task.name),"rm chest")
        ,(lambda: taskPosture.gotoq((30,3,1/DEG,0.8),chest=(0,0),rleg=q0[6:12],head=(0,0),rarm=q0[22:28],larm=q0[29:35]),"fold arms&leg")
        )

attime(1800+tex
       ,(lambda: sot.rm(taskPosture.task.name),"Remove posture" )
       ,(lambda: sot.push(taskrf.task.name), "Add RF")
       ,(lambda: gotoNd(taskrf, (-0.1,-0.1,0.25),"111" ), "Back RF")
       )

attime(2200+tex  ,lambda: goto6d(taskrf,vectorToTuple(rf0.T+(0,0,0.07)),(80,15,0.005,0.8))  , "RF to upper ground"       )
attime(2400+tex  ,lambda: goto6d(taskrf,vectorToTuple(rf0),(100,15,0.005,0.8))  , "RF to ground"       )
attime(2550+tex  
       ,(lambda: refset(taskCom,(0.01,0,0.797))  , "COM to zero"       )
       ,(lambda: taskCom.gain.setConstant(3)  , "lower com gain"       )
)
attime(2650+tex, lambda: sigset(sot.posture,q0), "Robot to initial pose")
attime(2650+tex  ,(lambda: contact(contactRF)  , "RF to contact"       )
       ,(lambda: sigset(taskCom.feature.selec,"111")  , "COM XYZ"       )
       ,(lambda: taskCom.gain.setByPoint(100,10,0.005,0.8)  , "upper com gain"       )
       )

attime(3200+tex,stop)

go()



