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
from meta_task_dyn_6d import MetaTaskDyn6d
from attime import attime
from numpy import *
from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
from mocap_parser import MocapParser,MocapParserTimed
from matrix_util import matrixToTuple

#-----------------------------------------------------------------------------
# --- ROBOT SIMU -------------------------------------------------------------
#-----------------------------------------------------------------------------

robotName = 'hrp14small'
robotDim  = robotDimension[robotName]
RobotClass = RobotDynSimu
robot      = RobotClass("robot")
robot.resize(robotDim)
dt = 1e-3

# Initial configuration
robot.set( ( 0,0,0.6487,0,0,0,0,0,-0.453786,0.872665,-0.418879,0,0,0,-0.453786,0.872665,-0.418879,0,0,0,0,0,0.261799,-0.174533,0,-0.523599,0,0,0.174533,0.261799,0.174533,0,-0.523599,0,0,0.174533 ) )

# Init config, both hands in place, com in the center of the polygon.
#robot.set((0.047605478333881616, 0.0069670567778655663, 0.70217819698128325, 0.033271671746594733, -0.14633177925107063, -0.0043665416399416524, 0.0060395105648764135, -0.044264063566775931, 0.10843367835692867, 0.20411254610434074, -0.16600602326644109, 0.011755310912801515, 0.0060134908716506048, -0.044087946756041933, 0.21121791020744213, -0.0023194171097636559, -0.06235944098204433, 0.011577286094383481, -0.031867567519300713, 0.0034122542878611364, -0.00018994268250045398, 0.017828376138565111, 0.57116020077409424, -0.20030039713333259, -0.035022771905715559, -1.4854699434782164, 0.033323186696575108, -0.98274234563896556, 0.1745446596832258, 0.67308873547891679, 0.22219955983304274, 0.048809070665574876, -1.412474883412113, -0.083661057118875753, -0.78534673513887354, 0.17410999723730466))

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
    updateMocap()
    robot.increment(dt)
    # Execute a function at time t, if specified with t.add(...)
    attime.run(robot.control.time)
    zmpup()

from ThreadInterruptibleLoop import *
@loopInThread
def loop():
    try:
        inc()
    except:
        tr.dump()
        print ' -- Robot has stopped --'
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
def r():
    mp.refresh()
@optionalparentheses
def m():
    mp.hlp_mocapCoord()
    mp.refresh()
@optionalparentheses
def n(): mp.hlp_showNumber()
@optionalparentheses
def t():
    mp.hlp_toggle()
    mp.refresh()


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
#--- MOCAP TRACKER -----------------------------------------------------------
#-----------------------------------------------------------------------------


class MocapTracker(MocapParserTimed):
    class TaskAssociation:
        def __init__(self,task,mocap):
            self.task = task
            self.mocap = mocap
    def __init__(self,*args):
        MocapParserTimed.__init__(self,*args)
        self.jointMap = dict()
    def addJointMap( self,jointName,metaTask ): 
        joint=None
        for i,j in enumerate(self.joints):
            if j.name==jointName: joint=i
        if joint==None:
            print "Error, joint name ",jointName," does not correspond to a mocap node."
        self.jointMap[jointName] = MocapTracker.TaskAssociation(metaTask,joint)
    def rmJointMap( self,jointName):
        if jointName in jointMap.keys():
            del jointMap[jointName]
        else: print "Error, joint name ",jointName," is not stored yet."
    def update(self):
        MocapParserTimed.update(self)
        for n,p in self.jointMap.iteritems():
            p.task.ref = self.jointPosition(p.mocap)

#-----------------------------------------------------------------------------
mp = MocapTracker('yoga/','yoga/outputJointsYogaTR')
mp.setLinksMap()
mp.assignDisplayObjects()
mp.setRobotViewer( robot.viewer )
mp.with_dispRobot = False
mp.with_dispJoints= True
mp.with_dispLinks = False
mp.hideLinks()
mp.delayTime = 0



#-----------------------------------------------------------------------------
# --- OPERATIONAL TASKS (For HRP2-14)---------------------------------------------
#-----------------------------------------------------------------------------

# --- Op task for the waist ------------------------------
taskWaist = MetaTaskDyn6d('taskWaist',dyn,'waist','waist')
taskChest = MetaTaskDyn6d('taskChest',dyn,'chest','chest')
taskHead = MetaTaskDyn6d('taskHead',dyn,'head','gaze')
taskrh = MetaTaskDyn6d('rh',dyn,'rh','right-wrist')
tasklh = MetaTaskDyn6d('lh',dyn,'lh','left-wrist')

for task in [ taskWaist, taskChest, taskHead, taskrh, tasklh ]:
    taskWaist.feature.frame('current')
    taskWaist.gain.setConstant(100)
    taskWaist.task.dt.value = dt

#-----------------------------------------------------------------------------
# --- OTHER TASKS ------------------------------------------------------------
#-----------------------------------------------------------------------------

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

#-----------------------------------------------------------------------------
# --- SOT Dyn OpSpaceH: SOT controller  --------------------------------------
#-----------------------------------------------------------------------------

sot = SolverDynReduced('sot')
sot.setSize(robotDim-6)
#sot.damping.value = 2e-2
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

#supportLF = ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))
#supportRF = ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))
supportLF = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
supportRF = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))

# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.feature.frame('desired')

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')

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
        Ml=matrix(dyn.lf.value)
        pl=matrix(sot._LF_p.value+((1,1,1,1),))
        return (Ml*pl)[0:3,:]
@optionalparentheses
def pr():
    if '_RF_p' in [s.name for s in sot.signals()]:
        Mr=matrix(dyn.rf.value)
        pr=matrix(sot._RF_p.value+((1,1,1,1),))
        return (Mr*pr)[0:3,:]



#-----------------------------------------------------------------------------
# --- TRACE ------------------------------------------------------------------
#-----------------------------------------------------------------------------

from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','step_','.dat')

tr.add('dyn.com','com')
tr.add('dyn.waist','waist')
tr.add('dyn.rh','rh')
tr.add('zmp.zmp','')
tr.add('dyn.position','')
tr.add('dyn.velocity','')
tr.add('robot.acceleration','robot_acceleration')
tr.add('robot.control','')
tr.add('gCom.gain','com_gain')
tr.add(contactLF.task.name+'.error','lf')
tr.add(contactRF.task.name+'.error','rf')

tr.start()
robot.after.addSignal('tr.triger')
robot.after.addSignal(contactLF.task.name+'.error')
robot.after.addSignal('dyn.rf')
robot.after.addSignal('dyn.lf')
robot.after.addSignal('sot.forcesNormal')

#-----------------------------------------------------------------------------
# --- FUNCTIONS TO PUSH/PULL/UP/DOWN TASKS -----------------------------------

#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

mp.setPositionMethod("KM")
mp.refresh()
mp.pause()
mp.timeScale = 2

dyn.rf.recompute(0)
dyn.lf.recompute(0)
Mrfr = matrix(dyn.rf.value)
Mrfm = matrix(mp.jointPosition_M(8,0))
Mlfr = matrix(dyn.lf.value)
Mlfm = matrix(mp.jointPosition_M(8,0))
#mp.Kw = Mrfr*Mrfm.I
#mp.Kw[0:3,0:3] = eye(3)
mp.Kw[0:3,0:3] = eye(3)
mp.Kw[0:3,3] = Mrfr[0:3,3]-Mrfm[0:3,3]


def updateMocap():
    mp.update()

sot.clear()

sot.addContactFromTask(contactLF.task.name,'LF')
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = supportRF
sot._LF_p.value = supportLF

featureComDes.errorIN.value = ( 0.01, 0.,  0.8077 )
featureCom.selec.value = "11"
gCom.setConstant( 500.0 )
sot.push('taskCom')

sot.push(taskrh.task.name)
sot.push(tasklh.task.name)
taskrh.feature.selec.value = '111'
taskrh.gain.setConstant(500)
tasklh.feature.selec.value = '111'
tasklh.gain.setConstant(500)

mp.addJointMap("Rhand",taskrh)
mp.addJointMap("Lhand",tasklh)


inc()
#go()

#attime(2, lambda: mp.forward())
#attime(2, lambda: taskrh.gain.setConstant(2/dt))
#attime(2, lambda: tasklh.gain.setConstant(2/dt))

mrh=eye(4)
mrh[0:3,3] = (0,0,-0.2)
taskrh.opmodif = matrixToTuple(mrh)
