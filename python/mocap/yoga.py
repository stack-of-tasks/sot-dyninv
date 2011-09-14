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
from meta_tasks_dyn import *
from meta_task_dyn_6d import MetaTaskDyn6d
from attime import attime
from numpy import *
from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
from mocap_parser import MocapParser,MocapParserScaled
from matrix_util import matrixToTuple, vectorToTuple
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

# Initial configuration
robot.set( ( 0,0,0.6487,0,0,0,0,0,-0.453786,0.872665,-0.418879,0,0,0,-0.453786,0.872665,-0.418879,0,0,0,0,0,0.261799,-0.174533,0,-0.523599,0,0,0.174533,0.261799,0.174533,0,-0.523599,0,0,0.174533 ) )

# Init config, both hands in place, com in the center of the polygon.
#robot.set((0.047605478333881616, 0.0069670567778655663, 0.70217819698128325, 0.033271671746594733, -0.14633177925107063, -0.0043665416399416524, 0.0060395105648764135, -0.044264063566775931, 0.10843367835692867, 0.20411254610434074, -0.16600602326644109, 0.011755310912801515, 0.0060134908716506048, -0.044087946756041933, 0.21121791020744213, -0.0023194171097636559, -0.06235944098204433, 0.011577286094383481, -0.031867567519300713, 0.0034122542878611364, -0.00018994268250045398, 0.017828376138565111, 0.57116020077409424, -0.20030039713333259, -0.035022771905715559, -1.4854699434782164, 0.033323186696575108, -0.98274234563896556, 0.1745446596832258, 0.67308873547891679, 0.22219955983304274, 0.048809070665574876, -1.412474883412113, -0.083661057118875753, -0.78534673513887354, 0.17410999723730466))

# Similar initial position
robot.set((0.029999999999999954, -9.1828441355437758e-29, 0.63000000000000145, 1.4360295221127874e-28, -8.7850988124233508e-28, -3.5290660360498378e-28, 3.5290667992075291e-28, 4.3262691761853982e-19, -0.46492814154439754, 1.0079478744182508, -0.54301973287385275, -4.326269177621427e-19, 3.5290664066979039e-28, 6.5287166994574407e-20, -0.46492814154439754, 1.0079478744182508, -0.54301973287385275, -6.5287167138177328e-20, 1.2924296127032732e-27, 0.15167281550539663, -9.5043422474809308e-28, -0.15167281550539663, 0.5642849052442489, -0.35455784714971755, -0.24227191641099344, -1.3414948262245163, -0.02295302606649734, -0.88270884805420247, 0.17453299999999999, 0.56428357448356325, 0.35455662057462251, 0.24227231972524182, -1.3414929885300777, 0.022954869149019692, -0.88271337036701469, 0.17453299999999999))
'''
References for the initial position
taskrh.ref = ((0, 0, -1, 0.18), (0, 1, 0, -0.41), (1, 0, 0, 0.8), (0.0, 0.0, 0.0, 1.0))
tasklh.ref = ((0, 0, -1, 0.18), (0, 0.1, 0, 0.41), (1, 0, 0, 0.8), (0.0, 0.0, 0.0, 1.0))
taskHead.ref = ((1,0,0,0.02),(0,1,0,0),(0,0,1,0),(0,0,0,1))
taskWaist.ref = ((1,0,0,0.03),(0,1,0,0),(0,0,1,0.63),(0,0,0,1))
'''

# Similar initial position with hand forward
robot.set((-0.033328803958899381, -0.0019839923040723341, 0.62176527349722499, 2.379901541270165e-05, 0.037719492175904465, 0.00043085147714449579, -0.00028574496353126724, 0.0038294370786961648, -0.64798319906979551, 1.0552418879542016, -0.44497846451873851, -0.0038397195926379991, -0.00028578259876671871, 0.0038284398205732629, -0.64712828871069394, 1.0534202525984278, -0.4440117393779604, -0.0038387216246160054, 0.00014352031102944824, 0.013151503268540811, -0.00057411504064861592, -0.050871000025766742, 0.21782780288481224, -0.37965640592672439, -0.14072647716213352, -1.1942332339530364, 0.0055454863752273523, -0.66956710808008013, 0.1747981826611808, 0.21400703176352612, 0.38370527720078107, 0.14620204468509851, -1.1873407322935838, -0.0038746980026940735, -0.66430172366423146, 0.17500428384087438))
'''
taskrh.ref = ((0, 0, -1, 0.25), (0, 1, 0, -0.41), (1, 0, 0, 0.8), (0.0, 0.0, 0.0, 1.0))
tasklh.ref = ((0, 0, -1, 0.25), (0, 0.1, 0, 0.41), (1, 0, 0, 0.8), (0.0, 0.0, 0.0, 1.0))
taskHead.ref = ((1,0,0,0.02),(0,1,0,0),(0,0,1,0),(0,0,0,1))
taskWaist.ref = ((1,0,0,0.),(0,1,0,0),(0,0,1,0.63),(0,0,0,1))
featureComDes.errorIN.value = ( 0.01, 0.,  0.8 )
gCom.setConstant( 50.0 )
'''

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

def inc():
    updateMocap()
    robot.increment(dt)
    if 'refresh' in ZmpEstimator.__dict__: zmp.refresh()
    # Execute a function at time t, if specified with t.add(...)
    attime.run(robot.control.time)
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

@optionalparentheses
def dump():
    history.dumpToOpenHRP('openhrp/yoga')

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
llimit[6:12] = mlimit[6:12] - dlimit[6:12]*0.49
ulimit[0,10] = mlimit[0,10] + dlimit[0,10]*0.49
ulimit[0,7] = mlimit[0,7] + dlimit[0,7]*0.49
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
#--- MOCAP TRACKER -----------------------------------------------------------
#-----------------------------------------------------------------------------

class MocapTracker(MocapParserScaled):
    class TaskAssociation:
        def __init__(self,task,mocap):
            self.task = task
            self.mocap = mocap
    def __init__(self,*args):
        MocapParserScaled.__init__(self,*args)
        self.jointMap = dict()
        self.posture = None
    def addJointMap( self,jointName,metaTask ): 
        joint=None
        if not jointName in self.joints:
            print "Error, joint name ",jointName," does not correspond to a mocap node."
            return
        self.jointMap[jointName] = MocapTracker.TaskAssociation(metaTask,jointName)
    def rmJointMap( self,jointName):
        if jointName in self.jointMap.keys():
            del self.jointMap[jointName]
        else: print "Error, joint name ",jointName," is not stored yet."
    def update(self):
        MocapParserScaled.update(self)
        for n,p in self.jointMap.iteritems():
            p.task.ref = self.jointPosition(p.mocap)
        if self.posture != None:
            self.posture.value = self.jointAngles(self.iter)
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

mp.alignHrpFrames()
for joint in mp.joints.values():
    joint.Ki[0:3,3] *= 0


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

# --- TASK COM ------------------------------------------------------
taskCom = MetaTaskDynCom(dyn,dt)

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
sot.damping.value = 8e-2
sot.breakFactor.value = 10
contact = AddContactHelper(sot)

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
#contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
contactLF.support =  ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))

#--- ZMP ---------------------------------------------------------------------
zmp = ZmpEstimator('zmp')
zmp.declare(sot,dyn)

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
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

mp.setPositionMethod("KMK")
mp.refresh()
mp.pause()
mp.timeScale = 2
mp.spaceScale = 0.9

# Initial position of the mocap reference
dyn.rf.recompute(0)
dyn.lf.recompute(0)
Mrfr = matrix(dyn.rf.value)
Mrfm = matrix(mp.jointPosition_M("Rfoot",0))
Mlfr = matrix(dyn.lf.value)
Mlfm = matrix(mp.jointPosition_M("Lfoot",0))
mp.Kw[0:3,0:3] = eye(3)
mp.Kw[0:3,3] = (Mrfr[0:3,3]-Mrfm[0:3,3] + Mlfr[0:3,3]-Mlfm[0:3,3])/2

for mocap,task in ( ("Lhand",tasklh),("Rhand",taskrh),("head",taskHead)):
    wmMm = matrix(mp.jointPosition_KM(mocap,0))
    task.feature.position.recompute(0)
    wrMr = matrix(task.feature.position.value)
    mp.joints[mocap].Ki = wmMm.I*wrMr
    mp.joints[mocap].Ki[0:3,3] *= 0

    wmMm = matrix(mp.jointPosition_KMK(mocap,0))
    mp.joints[mocap].Kw = wrMr*wmMm.I*mp.joints[mocap].Kw

def updateMocap():
    mp.update()

sot.clear()

contact(contactLF)
contact(contactRF)

taskCom.feature.selec.value = "11"
taskCom.ref = ( 0.01, 0.09,  0.8077 )
taskCom.gain.setByPoint(200,10,0.005,0.8)

taskrh.feature.selec.value = '111'
taskrh.gain.setConstant(100)
mrh=eye(4)
mrh[0:3,3] = (0,0,-0.08)
taskrh.opmodif = matrixToTuple(mrh)

tasklh.feature.selec.value = '111'
tasklh.gain.setConstant(100)
tasklh.opmodif = matrixToTuple(mrh)

#taskHead.gain.setConstant(500)
taskHead.feature.selec.value = '111000'

taskrf.feature.selec.value = '111'
mrf=eye(4)
mrf[0:3,3] = (0,0,-0.05)
taskrf.opmodif = matrixToTuple(mrf)
taskrf.feature.frame('desired')

sot.push(taskLim.name)
sot.push(taskCom.task.name)
sot.push(taskrh.task.name)

mp.addJointMap("Rhand",taskrh)
mp.addJointMap("Lhand",tasklh)
mp.addJointMap("head",taskHead)
mp.addJointMap("Rfoot",taskrf)
mp.posture = sot.posture
plug(robot.state,sot.position)
sot.breakFactor.value = 10

mp.forward()

sigset = ( lambda s,v : s.__class__.value.__set__(s,v) )
refset = ( lambda mt,v : mt.__class__.ref.__set__(mt,v) )

attime(80*mp.timeScale
       ,(lambda: sot.rmContact("RF"),"Remove RF contact" )
       ,(lambda: sot.push(taskrf.task.name), "Start to track RF mocap")
       )

attime(1000*mp.timeScale, 
       (lambda: taskCom.gain.setConstant(1), "Lower Com gains"),
       (lambda: refset(taskCom,(0.01,0,0.8077)), "Com back to the center")  )

attime(1050*mp.timeScale, 
       (lambda: goto6d(taskrf,(0.02,-0.12,0.055)) , "RF to final position"),
       (lambda: mp.rmJointMap("Rfoot"),"Stop tracking RF mocap")  )

attime(1100*mp.timeScale,
       lambda: contact(contactRF,taskrf),"Add the RF contact")

attime(3000,stop)

m()
inc()
go()

maxQ10=0.4
maxQ10=0.65
for i,q in enumerate(mp.qs):
    if q[10]>=maxQ10:
        qrep=matrix(q)
        qrep[0,10] = maxQ10
        mp.qs[i] = vectorToTuple(qrep)
