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
from matrix_util import matrixToTuple, vectorToTuple,rotate
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
    zmpup()
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

def setGain(gain,val):
    if val!=None:
        if len(val)==1:  gain.setConstant(val)
        elif len(val)==3: gain.set( val[0],val[1],val[2])
        elif len(val)==4: gain.setByPoint( val[0],val[1],val[2],val[3])

def goto6d(task,position,gain=None):
    M=eye(4)
    if( len(position)==3 ): M[0:3,3] = position
    else: print "Position 6D with rotation ... todo"
    task.feature.selec.value = "111111"
    setGain(task.gain,gain)
    task.featureDes.position.value = matrixToTuple(M)

def gotoNd(task,position,selec,gain=None):
    M=eye(4)
    if isinstance(position,matrix): position = vectorToTuple(position)
    if( len(position)==3 ): M[0:3,3] = position
    else: print "Position 6D with rotation ... todo"
    if isinstance(selec,str):   task.feature.selec.value = selec
    else: task.feature.selec.value = toFlags(selec)
    task.featureDes.position.value = matrixToTuple(M)
    setGain(task.gain,gain)

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
    task.feature.frame('current')
    task.gain.setConstant(50)
    task.task.dt.value = dt



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

class MetaTaskDynPosture(object):
    postureRange = { \
        "rleg": range(6,12), \
        "lleg": range(12,18), \
        "chest": range(18,20), \
        "head": range(20,22), \
        "rarm": range(22,28), \
        "rhand": [28], \
        "larm": range(29,35), \
        "lhand": [35], \
            }
    def __init__(self,dyn,dt,name="posture"):
        self.dyn=dyn
        self.name=name

        self.feature    = FeatureGeneric('feature'+name)
        self.featureDes = FeatureGeneric('featureDes'+name)
        self.task = TaskDynPD('task'+name)
        self.gain = GainAdaptive('gain'+name)

        plug(dyn.position,self.feature.errorIN)
        self.feature.jacobianIN.value = matrixToTuple( identity(robotDim) )
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

    def gotoq(self,gain=None,**kwargs):
        act=list()
        qdes = zeros((36,1))
        for n,v in kwargs.items():
            r = self.postureRange[n]
            act += r
            if isinstance(v,matrix): qdes[r,0] = vectorToTuple(v)
            else: qdes[r,0] = v
        self.ref = vectorToTuple(qdes)
        self.feature.selec.value = toFlags(act)
        setGain(self.gain,gain)

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
# --- FUNCTIONS TO PUSH/PULL/UP/DOWN TASKS -----------------------------------

#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------
DEG = 180.0/pi
sot.clear()

contact(contactLF)
contact(contactRF)

taskCom.feature.selec.value = "11"
taskCom.gain.setByPoint(100,10,0.005,0.8)

mrf=eye(4)
mrf[0:3,3] = (0,0,-0.105)
#taskrf.opmodif = matrixToTuple(mrf)
taskrf.feature.frame('desired')
#taskrf.gain.setByPoint(60,5,0.01,0.8)
taskrf.gain.setByPoint(40,2,0.002,0.8)

taskChest.feature.selec.value='111000'
taskChest.ref = rotate('y',80/DEG)
#taskChest.ref = rotate('y',80/DEG)
taskChest.gain.setByPoint(30,3,1/DEG,0.8)

#taskPosture.gain.setByPoint(60,5,5/DEG,0.9)
taskPosture.gain.setByPoint(30,3,1/DEG,0.8)

ql = matrix(dyn.lowerJl.value)
ql[0,15] = 25/DEG
dyn.lowerJl.value
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

attime(500,lambda: gotoNd(taskrf, (-0.8,0,0.8),"000101" ), "Extend RF")

attime(800,lambda: sot.push(taskChest.task.name), "add chest")

attime(1000
       ,(lambda: sot.rm(taskrf.task.name), "rm RF")
       ,(lambda: sot.push(taskPosture.task.name), "add posture")
       ,(lambda: taskPosture.gotoq(chest=(0,0),rleg=(0,0,0,0,0,0),head=(0,0)), "extend body")
       )

with_full_extention=False
if with_full_extention:
    attime(1300
           ,lambda: taskPosture.gotoq(chest=(0,0),rleg=(0,0,0,0,0,0),head=(0,0),rarm=(0,-pi/2,0,0,0,0),larm=(0,pi/2,0,0,0,0)), "extend arm")

    attime(2000
           ,lambda: taskPosture.gotoq(30,chest=(0,0),rleg=(0,0,0,0,0.73,0),head=(0,0),rarm=(0,-pi/2,0,0,0,0),larm=(0,pi/2,0,0,0,0)), "extend foot")
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

'''robot.set( (-0.22338017682364666, 0.094679696008257069, 0.6406354063194315, -0.046149502155439906, 1.3834043086232479, -0.045751564595492393, -0.00043528996402237706, 0.0028744648411845957, -0.47567845262453851, 0.7731765985092065, -0.32594537966076575, -0.0029603563556822449, -0.16456900265664706, -0.031430863650961184, -2.0148121310236262, 0.43929960908288562, 0.18928315181673525, 0.17611103058143049, -0.00037370245565424205, -0.00014143571164947397, 8.7559046906180755e-07, 7.1211147920686882e-05, 0.20950155287705138, -0.38261191572903752, -0.1409579892969334, -1.2006478349239718, 0.0055192144589002994, -0.67056461724265271, 0.17476330298560117, 0.20672021910771882, 0.39182703814168718, 0.14945548959841934, -1.1920338277316804, -0.003453520219234802, -0.66498751012318535, 0.17474628650307528) )
robot.setVelocity( (-0.0034980508073499746, -7.8546901386487115e-05, -0.0016849885158409757, -0.00042323578018366197, -0.0080054937692844511, 0.00086344279117106829, -0.00015755559034456312, 0.0013060411790611658, -0.21786102712256575, 0.35440517206749117, -0.14941610627508575, -0.0013290768265727577, 0.00011113538937583873, -0.00054127148646175311, 8.5548931282521993e-05, 0.0024653080400074217, 0.0054855827172477344, 0.00041334449428695003, -0.00010747410962737952, -3.8505135328234213e-05, 2.3358739214336891e-07, 1.9098782957311844e-05, 0.010521707268533676, 0.0037563490996982444, 0.00030433568575777238, 0.0081039468043348356, 3.4077369449514742e-05, 0.0012601869472091919, 8.3190734016776364e-07, 0.0092086079693020056, -0.010246626556435254, -0.0041025198155700817, 0.0059363185419371552, -0.00053151985927071734, 0.00086774348235294993, 6.7138299807645551e-07) )
robot.state.time =  1800-1
attime.fastForward(1800)
'''
go()



