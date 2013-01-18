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
from meta_task_dyn_6d import MetaTaskDyn6d
from attime import attime
from numpy import *

from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor

from numpy import *
def totuple(a):
    al = a.tolist()
    res = []
    for i in range( a.shape[0] ):
        res.append( tuple(al[i]) )
    return tuple(res)


#-----------------------------------------------------------------------------
#    INPUT ARGUMENTS, IF POSSIBLE
#-----------------------------------------------------------------------------

usage = "usage: py postureOpTask.py [options] args"
parser = OptionParser(usage=usage)
parser.add_option("-p", dest="opti_q_file", help="Specification of qs from mocap optimization")
parser.add_option("-o", dest="opFolder", help="Folder containing the operational points (WRM,Ki,KW)")
parser.add_option("-r", dest="robot_state", help="Specification of the robot state")
parser.add_option("-v", dest="robot_velocity", help="Specification of the robot velocity")
parser.add_option("-a", dest="robot_acceleration", help="Specification of the robot acceleration")
parser.add_option("-t", dest="robot_time", help="Specification of the robot time",
                        type=int,default=0)
(options, args) = parser.parse_args()
parser.print_help()
print '\n -------------------------------------------------------------------------'

# Open the file containing robot_state
if options.robot_state == None:
    print "Please, specify the states of the robot"
    #sys.exit(0)
else:
    fileRobotState = open(options.robot_state,'r')
    linesRobotState = fileRobotState.readlines()

# Open the file containing robot_velocity
if options.robot_velocity == None:
    print "Please, specify the velocity of the robot"
    #sys.exit(0)
else:
    fileRobotVelocity = open(options.robot_velocity,'r')
    linesRobotVelocity = fileRobotVelocity.readlines()

# Open the file containing robot_acceleration
if options.robot_acceleration == None:
    print "Please, specify the acceleration of the robot"
    #sys.exit(0)
else:
    fileRobotAcceleration = open(options.robot_acceleration,'r')
    linesRobotAcceleration = fileRobotAcceleration.readlines()

# Read the delay time, if provided
robot_init_time = options.robot_time
print " Initial Robot Time set to " + str(robot_init_time)


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
    robot.viewer = None


#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

class TimeControl:
    ''' Class to store a variable number of functions to be executed at a
    a certain time
    '''
    def __init__(self):
        self.time = []
        self.task = dict()
    def add(self,value,*functions):
        self.time.append(value)
        self.task[value] = functions

showNumber  = False
flag_finish = 0      # Indicate when reading of the file finished
READ_TIME = 5        # Every how many iterations the new 'desired' position is read (1000/200)
cnt = robot_init_time % READ_TIME  # Counter to read file every READ_TIME iterations

zmp_waist = (0.,0.,0.)

def inc():
    global showNumber, cnt, READ_TIME, flag_finish

    robot.increment(dt)
    # Execute a function at time t, if specified with t.add(...)
    attime.run(robot.control.time)
    if showNumber: print robot.control.time
    zmpup()

def hlp_showNumber():
    global showNumber
    if showNumber: showNumber=False
    else:          showNumber=True

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
def n(): hlp_showNumber()
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
# --- OPERATIONAL TASKS (For HRP2-14)---------------------------------------------
#-----------------------------------------------------------------------------

# --- Op task for the waist ------------------------------
taskWaist = MetaTaskDyn6d('taskWaist',dyn,'waist','waist')
taskWaist.feature.position.recompute(0)
tempW = taskWaist.feature.position.value
taskWaist.ref = ((1,0,0,0),(0,1,0,tempW[1][3]),(0,0,1,0),(0,0,0,1))
taskWaist.feature.selec.value = '111000'
taskWaist.feature.frame('current')
taskWaist.gain.setConstant(50)
taskWaist.task.dt.value = dt

# --- Op task for the chest ------------------------------
taskChest = MetaTaskDyn6d('taskChest',dyn,'chest','chest')
taskChest.feature.position.recompute(0)
tempC = taskChest.feature.position.value
taskChest.ref = ((1,0,0,0),(0,1,0,tempC[1][3]),(0,0,1,0),(0,0,0,1))
taskChest.feature.selec.value = '111000'
taskChest.feature.frame('current')
taskChest.gain.setConstant(50)
taskChest.task.dt.value = dt

# --- Op task for the head-------------------------------
taskHead = MetaTaskDyn6d('taskHead',dyn,'head','gaze')
tempH = taskHead.feature.position.value
taskHead.ref = ((1,0,0,0),(0,1,0,tempH[1][3]),(0,0,1,0),(0,0,0,1))
taskHead.feature.selec.value = '111000'  #Rot
taskHead.feature.frame('current')
taskHead.gain.setConstant(10)
taskHead.task.dt.value = dt

# --- Op task for right hand ----------------------------
taskrh = MetaTaskDyn6d('rhand',dyn,'rh','right-wrist')
taskrh.feature.selec.value = '111111'
taskrh.feature.frame('current')
taskrh.ref = ((0,0,1,-0.1),(0,1,0,0.45),(-1,0,0,0.6),(0,0,0,1))
taskrh.gain.setConstant(10)
taskrh.task.dt.value = dt

# --- Op task for left hand -----------------------------
tasklh = MetaTaskDyn6d('lhand',dyn,'lh','left-wrist')
tasklh.ref = ((0,0,1,-0.3),(0,1,0,0.4),(-1,0,0,0.72),(0,0,0,1))
tasklh.feature.selec.value = '111111'
tasklh.feature.frame('current')
tasklh.gain.setConstant(10)
tasklh.task.dt.value = dt

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

# Two feet com initial position
featureComDes.errorIN.value = (0.06,0.145,0.80481744155)


# --- Task Joint Limits -----------------------------------------
taskjl = TaskDynJointLimits('taskjl')
plug(dyn.position,taskjl.position)
plug(dyn.velocity,taskjl.velocity)
taskjl.dt.value = dt
taskjl.controlGain.value = 10

dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskjl.referenceInf.value = dyn.lowerJl.value
taskjl.referenceSup.value = dyn.upperJl.value

# --- Task For Position and Velocity limits ---------------------
'''
taskLim = TaskDynLimits('taskLim')
plug(dyn.position,taskLim.position)
plug(dyn.velocity,taskLim.velocity)
taskLim.dt.value = 1e-3
taskLim.controlGain.value = 10

dyn.upperJl.recompute(0)
dyn.lowerJl.recompute(0)
taskLim.referencePosInf.value = dyn.lowerJl.value
taskLim.referencePosSup.value = dyn.upperJl.value

dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240, 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
dqlow = [-val*pi/180 for val in dqup]
dqup  = [val*pi/180 for val in dqup]
taskLim.referenceVelInf.value = tuple(dqlow)
taskLim.referenceVelSup.value = tuple(dqup)
'''

#-----------------------------------------------------------------------------
# --- SOT Dyn OpSpaceH: SOT controller  --------------------------------------
#-----------------------------------------------------------------------------

#sot = SolverOpSpaceModif('sot')     # Does not include ddq = -kv dq
#sot = SolverOpSpace('sot')         # Includes  ddq = -kv dq
sot = SolverDynReduced('sot')
sot.setSize(robotDim-6)
#sot.damping.value = 2e-2
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

#plug(sot.control,robot.control)
plug(sot.torque, robot.control)

#For the integration of q = int(int(qddot)).
plug(sot.acceleration,robot.acceleration)

#-----------------------------------------------------------------------------
# ---- CONTACT: Contact definition -------------------------------------------
#-----------------------------------------------------------------------------

# == New support polygons ==

supportLF = ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))
supportRF = ((0.03,-0.03,-0.03,0.03),(-0.015,-0.015,0.015,0.015),(-0.105,-0.105,-0.105,-0.105))

#supportLF = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
#supportRF = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))



# Left foot contact
contactLF = MetaTaskDyn6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.feature.frame('desired')
sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = supportLF

# Right foot contact
contactRF = MetaTaskDyn6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = supportRF

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

# --- Operational Space tasks ------------------------------------------------

def pushRightH():sot.push(taskrh.task.name);    print ' -- Pushed: Right hand task (6d)'
def pushLeftH(): sot.push(tasklh.task.name);    print ' -- Pushed: Left hand task (6d)'
def pushWaist(): sot.push(taskWaist.task.name); print ' -- Pushed: waist task (6d)'
def pushChest(): sot.push(taskChest.task.name); print ' -- Pushed: chest task (6d)'
def pushHead(): sot.push(taskHead.task.name);   print ' -- Pushed: head task (6d)'

def rmRightH():  sot.rm(taskrh.task.name);      print ' -- Removed: right hand task (6d)'
def rmLeftH():   sot.rm(tasklh.task.name);      print ' -- Removed: left hand task (6d)'
def rmWaist():   sot.rm(taskWaist.task.name);   print ' -- Removed: waist task (6d)'
def rmChest():   sot.rm(taskChest.task.name);   print ' -- Removed: chest task (6d)'
def rmHead():    sot.rm(taskHead.task.name);    print ' -- Removed: head task (6d)'

def upRightH():  sot.up(taskrh.task.name);      print ' -- Up: right hand task (6d)'
def upLeftH():   sot.up(tasklh.task.name);      print ' -- Up: left hand task (6d)'
def upWaist():   sot.up(taskWaist.task.name);   print ' -- Up: waist task (6d)'
def upChest():   sot.up(taskChest.task.name);   print ' -- Up: chest task (6d)'
def upHead():   sot.up(taskHead.task.name);     print ' -- Up: head task (6d)'

def downRightH():sot.down(taskrh.task.name);    print ' -- Down: right hand task (6d)'
def downLeftH(): sot.down(tasklh.task.name);    print ' -- Down: left hand task (6d)'
def downWaist(): sot.down(taskWaist.task.name); print ' -- Down: waist task (6d)'
def downChest(): sot.down(taskChest.task.name); print ' -- Down: chest task (6d)'
def downHead():  sot.down(taskHead.task.name);  print ' -- Down: head task (6d)'


# --- Other tasks -------------------------------------------------------------

def pushTaskJL():     sot.push('taskjl');       print ' -- Pushed: joint limits (only position) task'
def pushTaskLimits(): sot.push('taskLim');      print ' -- Pushed: joint limits (position and velocity) task'
def pushCom(): sot.push('taskCom'); print ' -- COM task pushed '
def rmCom():   sot.rm('taskCom');   print ' -- COM task removed'

def moveCom2Left():
    featureComDes.errorIN.value = ( 0.00949, 0.095,  0.8077 )

def raiseRightFoot():
    sot.rmContact('RF')
    sot.push(contactRF.task.name)
    contactRF.ref = ((1,0,0,0.00949),(0,1,0,-0.095),(0,0,1,0.185),(0,0,0,1))
    contactRF.gain.setByPoint( 5.0, 35.0, 0.02, 0.5)
    contactRF.feature.selec.value = '111111'
    contactRF.feature.frame('desired')
    contactRF.task.resetJacobianDerivative()
    print ' -- Move Right leg up'

def lowerRightFoot():
    contactRF.ref = ((1,0,0,0.00949),(0,1,0,-0.095),(0,0,1,0.105),(0,0,0,1))
    contactRF.feature.selec.value = '111111'
    contactRF.task.resetJacobianDerivative()
    print ' -- Move Right leg down'

def addRightContact():
    global supportRF
    sot.addContactFromTask(contactRF.task.name,'RF')
    sot._RF_p.value = supportRF
    sot.rm(contactRF.task.name)
    contactRF.task.resetJacobianDerivative()
    print ' -- Added support for the right foot'

def chestStraight():
    taskChest.ref = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
    taskChest.feature.selec.value = '111000'
    sot.push(taskChest.task.name)

def moveCom2Center():
    featureComDes.errorIN.value = (0.01356,  0.001535,  0.8077 )


#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

t = TimeControl()
sot.clear()
#sot.push('taskLim')

gCom.setByPoint( 35.0, 5.0, 0.02, 0.5)
#gCom.setByPoint( 150.0, 50.0, 0.02, 0.5)

attime(     1, pushCom, moveCom2Left)
attime(  2000, raiseRightFoot)
attime(  6000, lowerRightFoot)
attime(  9000, addRightContact)
attime(  9001, moveCom2Center)
attime( 11500, stop)


#inc()
go()

'''
matlab.prec=9
matlab.fullPrec=0


print sot.velocity.m
print sot.dyndrift.m
print sot.matrixInertia.m
print sot.acceleration.m
print "dq=velocity; b=dyndrift; A=matrixInertia; ddq=acceleration;"

print sot.sizeForceSpatial.m
print sot.velocity.m
print sot.dyndrift.m
print sot.forceGenerator.m
print sot.Jc.m
print sot.freeMotionBase.m
print sot.freeForceBase.m
print sot.inertiaSqroot.m
print sot.inertiaSqrootInv.m
print sot.driftContact.m
print taskCom.jacobian.m
print taskCom.task.m
print taskCom.Jdot.m
print "X=forceGenerator; V=freeMotionBase; K = freeForceBase; B=inertiaSqrootInv; Bi=inertiaSqroot; Jcom=jacobian; Ab=Bi'*Bi; dc = driftContact; J=X*Jc; G=J*B; Gc=Jc*B; Sb=[eye(6) zeros(6,30)];"

print sot.solution.m
print sot.acceleration.m
sot.forces.recompute(sot.acceleration.time)
print sot.forces.m
print "x=solution; ddq=acceleration; f=forces; phi = X'*f;"
print "u=x(1:24); psi=x(25:end); "

print "ddq2= ddq; phi2=phi; f2=f; fn2=f(3:3:end);"
'''
