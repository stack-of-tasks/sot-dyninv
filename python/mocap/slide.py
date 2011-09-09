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
from attime import attime
from numpy import *
from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
from matrix_util import matrixToTuple, vectorToTuple
from history import History

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
    attime.run(robot.control.time)
    zmpup()
    robot.viewer.updateElementConfig('zmp',[zmp.zmp.value[0],zmp.zmp.value[1],0,0,0,0])
    if dyn.com.time >0:
        robot.viewer.updateElementConfig('com',[dyn.com.value[0],dyn.com.value[1],0,0,0,0])
    history.record()

from ThreadInterruptibleLoop import *
@loopInThread
def loop():
    try:
        inc()
    except:
        tr.dump()
        print robot.state.time,': -- Robot has stopped --'
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
def dump():
    history.dumpToOpenHRP('planche')

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

def goto6d(task,position,gain=None):
    M=eye(4)
    if( len(position)==3 ): M[0:3,3] = position
    else: print "Position 6D with rotation ... todo"
    task.feature.selec.value = "111111"
    if gain!=None: task.gain.setConstant(gain)
    task.featureDes.position.value = matrixToTuple(M)

def contact(contact,task=None):
    sot.addContactFromTask(contact.task.name,contact.name)
    sot.signal("_"+contact.name+"_p").value = contact.support
    if task!= None: sot.rm(task.task.name)
    contact.task.resetJacobianDerivative()


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
    taskWaist.feature.frame('current')
    taskWaist.gain.setConstant(50)
    taskWaist.task.dt.value = dt

#-----------------------------------------------------------------------------
# --- OTHER TASKS ------------------------------------------------------------
#-----------------------------------------------------------------------------

# --- TASK COM ------------------------------------------------------

class MetaTaskDynCom(object):
    def __init__(self,dyn,dt,name="com"):
        self.dyn=dyn
        self.name=name
        dyn.setProperty('ComputeCoM','true')

        self.feature    = FeatureGeneric('feature'+name)
        self.featureDes = FeatureGeneric('featureDes'+name)
        self.task = TaskDynPD('task'+name)
        self.gain = GainAdaptive('gain'+name)

        plug(dyn.com,self.feature.errorIN)
        plug(dyn.Jcom,self.feature.jacobianIN)
        self.feature.sdes.value = self.featureDes.name

        self.task.add(self.feature.name)
        plug(dyn.velocity,self.task.qdot)
        self.task.dt.value = dt

        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)

    @property
    def ref(self):
        return self.featureDes.errorIN.value

    @ref.setter
    def ref(self,v):
        self.featureDes.errorIN.value = v

taskCom = MetaTaskDynCom(dyn,dt)


# --- TASK POSTURE --------------------------------------------------
featurePosture    = FeatureGeneric('featurePosture')
featurePostureDes = FeatureGeneric('featurePostureDes')
plug(dyn.position,featurePosture.errorIN)
featurePosture.sdes.value = featurePostureDes.name
featurePosture.jacobianIN.value = matrixToTuple( identity(robotDim) )

taskPosture = TaskDynPD('taskPosture')
taskPosture.add('featurePosture')
plug(dyn.velocity,taskPosture.qdot)
taskPosture.dt.value = dt

taskPosture.controlGain.value = 2

def gotoq( qref,selec=None ):
    featurePostureDes.errorIN.value = vectorToTuple( qref.transpose() )
    if selec!=None: featurePosture.selec.value = toFlags( selec )

qref = zeros((robotDim,1))
qref[29] = -0.95
gotoq(qref,[29])

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
sot.setSize(robotDim-6)
sot.damping.value = 1e-2
sot.breakFactor.value = 20

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

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')

# ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))
# ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))
contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
#contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactLF.support =  ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))
contactLF.name = "LF"
contactRF.name = "RF"

#-----------------------------------------------------------------------------
#--- ZMP ---------------------------------------------------------------------
#-----------------------------------------------------------------------------
zmp = ZmpEstimator('zmp')
def computeZmp():
    p=zeros((4,0))
    f=zeros((0,1))
    if '_RF_p' in [s.name for s in sot.signals()]:
        Mr=matrix(dyn.rf.value)
        fr=matrix(sot._RF_fn.value).transpose()
        pr=matrix(sot._RF_p.value+((1,1,1,1),))
        p=hstack((p,Mr*pr))
        f=vstack((f,fr))
    if '_LF_p' in [s.name for s in sot.signals()]:
        Ml=matrix(dyn.lf.value)
        fl=matrix(sot._LF_fn.value).transpose()
        pl=matrix(sot._LF_p.value+((1,1,1,1),))
        p=hstack((p,Ml*pl))
        f=vstack((f,fl))
    zmp=p*f/sum(f)
    return zmp
def zmpup():
    zmp.zmp.value = tuple(computeZmp()[0:3].transpose().tolist()[0])
    zmp.zmp.time = sot.solution.time

@optionalparentheses
def pl():
    if '_LF_p' in [s.name for s in sot.signals()]:
        print 'checkin'
        Ml=matrix(dyn.lf.value)
        pl=matrix(sot._LF_p.value+((1,1,1,1),))
        return  matlab(  matrixToTuple((Ml*pl)[0:3,:]) ).resstr
@optionalparentheses
def pr():
    if '_RF_p' in [s.name for s in sot.signals()]:
        Mr=matrix(dyn.rf.value)
        pr=matrix(sot._RF_p.value+((1,1,1,1),))
        return matlab(  matrixToTuple(  (Mr*pr)[0:3,:] ))


#-----------------------------------------------------------------------------
# --- TRACE ------------------------------------------------------------------
#-----------------------------------------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','yoga_','.dat')

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
# --- FUNCTIONS TO PUSH/PULL/UP/DOWN TASKS -----------------------------------

#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------


sot.clear()

contact(contactLF)
contact(contactRF)

#taskCom.feature.selec.value = "111"
taskCom.gain.setByPoint(100,10,0.005,0.8)
#taskCom.gain.setConstant(2)

#taskrh.feature.selec.value = '111'
#taskrh.gain.setConstant(100)

tasklh.feature.selec.value = '111'
tasklh.gain.setConstant(100)
#tasklh.opmodif = matrixToTuple(mrh)


mrf=eye(4)
mrf[0:3,3] = (0,0,-0.105)
taskrf.opmodif = matrixToTuple(mrf)
taskrf.feature.frame('desired')
taskrf.feature.selec.value = '111111'
drh=eye(4)
drh[0,3]=0.1
drh[0,3]=0.1
taskrf.ref = matrixToTuple(drh)

#taskHead.gain.setConstant(500)
taskHead.feature.selec.value = '111000'

#sot.push(taskLim.name)
#sot.push(taskCom.task.name)
#sot.push(taskrh.task.name)
#sot.push(tasklh.task.name)
#sot.push(taskHead.task.name)
#sot.push(taskWaist.task.name)
#sot.push(taskPosture.name)



# --- Events ---------------------------------------------
sigset = ( lambda s,v : s.__class__.value.__set__(s,v) )
refset = ( lambda mt,v : mt.__class__.ref.__set__(mt,v) )

'''
def upfoot():
    drh=matrix(eye(4))
    taskrf.feature.position.recompute( robot.state.time )
    crh = matrix(taskrf.feature.position.value)
    drh[0:3,3]=crh[0:3,3]
    drh[2,3] += 0.005
    taskrf.ref = matrixToTuple(drh)

def slidefoot(x,y):
    taskrf.feature.position.recompute( robot.state.time )
    drh = matrix(taskrf.feature.position.value)
    drh[0:3,0:3] = eye(3)
    drh[0:2,3] = (x,y)
    drh[0:2,3] = 0.005
    print drh
    taskrf.ref = matrixToTuple(drh)
'''
rf0=matrix((0.0095,-0.095,0.02))

attime(2
       ,(lambda : sot.push(taskCom.task.name),"Add COM")
       ,(lambda : sigset(taskCom.feature.selec, "111"),"COM XYZ")
       ,(lambda : refset(taskCom, ( 0.01, 0.09,  0.7 )), "Com to left foot")
       )

attime(300
       ,(lambda : sigset(taskCom.feature.selec, "11"),"COM XY")
       ,(lambda: sot.rmContact("RF"),"Remove RF contact" )
       ,(lambda: sot.push(taskrf.task.name), "Add RF")
       ,(lambda: goto6d(taskrf,vectorToTuple(rf0) ), "Up foot RF")
       )

attime(400  ,(lambda: goto6d(taskrf, vectorToTuple(rf0+( 0.1, 0.0,0)))  , "RF to front")       )
attime(500  ,(lambda: goto6d(taskrf, vectorToTuple(rf0+( 0.1,-0.1,0)))  , "RF to front-right")       )
attime(600  ,(lambda: goto6d(taskrf, vectorToTuple(rf0+(-0.1,-0.1,0)))  , "RF to back-right")       )
attime(700  ,(lambda: goto6d(taskrf, vectorToTuple(rf0+(-0.1, 0.0,0)))  , "RF to back")       )
attime(800  ,(lambda: goto6d(taskrf, vectorToTuple(rf0+( 0.0, 0.0,0)))  , "RF to center")       )




attime(3000,stop)



#inc()
go()


