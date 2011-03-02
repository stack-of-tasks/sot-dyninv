from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.matlab import matlab

# --- Dynamic parameters ---
hrp2_14_pkgdatarootdir = "/home/nmansard/compil/devgiri/hpp2/share/hrp2_14"
modelDir = hrp2_14_pkgdatarootdir
xmlDir = hrp2_14_pkgdatarootdir
modelName = 'HRP2JRLmainsmall.wrl'
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath = xmlDir + '/HRP2LinkJointRankSmall.xml'
robotDim=36


# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

robot = RobotSimu("robot")
robot.resize(robotDim)
#robot.set( (0,0,0,0,0,0, -0.0091163033083694940822,-0.091409963793829082657,-0.47197874328281280709,0.84038019261713603481,-0.47023279905304871118,0.089662407859063653071,0.0095078180225693850747,0.091110286804129178573,-0.4694503518480188653,0.83530799538565336793,-0.46768619090392338222,-0.093802946636709280681,-8.7564658296357018321e-05,0.0032643187737393364843,-7.8338082008638037324e-06,0.00019474257801330629915,0.25837025731361307201,-0.17509910214200960499,-6.1173425555032122825e-05,-0.52495354876847799552,3.1825099712999219606e-06,-0.00025760004742291479777,-3.4121048192112107608e-06,0.25836727162795458668,0.17432209754621175168,-8.8902395499734237117e-05,-0.52498369084585783106,-3.4610294148795152394e-07,-0.00026540143977199129972,1.004984814529256132e-06) )
robot.set( ( 0.0274106623863, 0.143843868989, 0.646921914726, 0.00221064938462, 0.101393756965, 1.36729741242e-05, -0.00911630330837, -0.0914099637938, -0.471978743283, 0.840380192617, -0.470232799053, 0.0896624078591, 0.00950781802257, 0.0911102868041, -0.469450351848, 0.835307995386, -0.467686190904, -0.0938029466367, -8.75646582964e-05, 0.00326431877374, -7.83380820086e-06, 0.000194742578013, 0.258370257314, -0.175099102142, -6.1173425555e-05, -0.524953548768, 3.1825099713e-06, -0.000257600047423, -3.41210481921e-06, 0.258367271628, 0.174322097546, -8.89023954997e-05, -0.524983690846, -3.46102941488e-07, -0.000265401439772, 1.00498481453e-06 ) )


robot_wrapper=robot
robot = PseudoRobotDynamic("dynint")
robot.replace(robot_wrapper.name,True)

dt=1e-3
robot.dt.value = dt

#robot.boundNewCommand("increment")


# --- DYN ----------------------------------------------------------------------
# --- DYN ----------------------------------------------------------------------
# --- DYN ----------------------------------------------------------------------

dyn = Dynamic("dyn")
dyn.setFiles(modelDir, modelName,specificitiesPath,jointRankPath)
dyn.parse()

dyn.inertiaRotor.value = (0,0,0,0,0,0,1.01e-4,6.96e-4,1.34e-4,1.34e-4,6.96e-4,6.96e-4,1.01e-4,6.96e-4,1.34e-4,1.34e-4,6.96e-4,6.96e-4,6.96e-4,6.96e-4,1.10e-4,1.10e-4,6.96e-4,6.60e-4,1.00e-4,6.60e-4,1.10e-4,1.00e-4,1.00e-4,6.96e-4,6.60e-4,1.00e-4,6.60e-4,1.10e-4,1.00e-4,1.00e-4)
dyn.gearRatio.value = (0,0,0,0,0,0,384.0,240.0,180.0,200.0,180.0,100.0,384.0,240.0,180.0,200.0,180.0,100.0,207.69,381.54,100.0,100.0,219.23,231.25,266.67,250.0,145.45,350.0,200.0,219.23,231.25,266.67,250.0,145.45,350.0,200.0)


plug(robot.state,dyn.position)
plug(robot.velocity,dyn.velocity)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()

dyn.setProperty('ComputeBackwardDynamics','true')
dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# --- FOOT on the ground
# The integrator is not working properly. So added some trick
# to integrate properly, based on the knowledge that one foot
# stay still on the ground ... bbbaaaaaaaaadddd! (but working)

dyn2 = Dynamic('dyn2')
dyn2.setFiles(modelDir, modelName,specificitiesPath,jointRankPath)
dyn2.parse()
plug(robot.state,dyn2.position)
dyn2.velocity.value = robotDim*(0.,)
dyn2.acceleration.value = robotDim*(0.,)
dyn2.ffposition.value = 6*(0,)
# TODO: was J0 instead of Jn in previous version
dyn2.createOpPoint('contact','right-ankle')

waistMsole = OpPointModifier('waistMsole')
plug(dyn2.contact,waistMsole.positionIN)
wMs=((1,0,0,0),(0,1,0,0),(0,0,1,-0.105),(0,0,0,1))
waistMsole.setTransformation( wMs )

flex=AngleEstimator('flex')
flex.setFromSensor(False)
plug(dyn2.contact,flex.contactEmbeddedPosition)
plug(waistMsole.position,flex.contactEmbeddedPosition)
plug(flex.waistWorldPoseRPY,dyn.ffposition)
flex.contactWorldPosition.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))

# --- FF VELOCITY ----------
dyn3=Dynamic('dyn3')
dyn3.setFiles(modelDir, modelName,specificitiesPath,jointRankPath)
dyn3.parse()
plug(robot.state,dyn3.position)
plug(flex.waistWorldPoseRPY,dyn3.ffposition)
dyn3.velocity.value = robotDim*(0.,)
dyn3.acceleration.value = robotDim*(0.,)
dyn3.createPosition('contact','right-ankle')
dyn3.createJacobian('Jcontact','right-ankle')

plug(dyn3.Jcontact,flex.jacobian)
plug(robot.velocity,flex.qdot)
plug(flex.qdotOUT,dyn.velocity)

# --- CONTACT MODIFICATIONS --- ---------------------------------------------
# Put the robot on the ground
# These computations are done only for debug, to be use by the programmer
# to compute (e.g. with matlab) the root position at start.

sole = OpPointModifier('sole')
dyn.createOpPoint('rf','right-ankle')
dyn.createPosition('root','waist')

plug(dyn.rf,sole.positionIN)
sole.setTransformation(((1,0,0,0),(0,1,0,0),(0,0,1,-0.105),(0,0,0,1)))

# 45deg
# In AF: next; dyn.root ; sole.position ... copy/paste from AF to matlab
# In Matlab:
# Mref=[ s 0 s 0 ; 0 1 0 0 ; -s 0 s 0 ; 0 0 0 1]
# Mref*inv(position)*root
# robot.root [4,4]((0.99772,0.06735,0.00309,0.20444),(-0.06477,0.97027,-0.23318,0.06157),(-0.01870,0.23245,0.97243,0.39644),(0.00000,0.00000,0.00000,1.00000))

# 10deg
# a=10/180*pi; c=cos(a); s=sin(a); Mref=[ c 0 s 0 ; 0 1 0 0 ; -s 0 c 0 ; 0 0 0 1];
# robot.root [4,4]((0.99841,0.03015,-0.04757,0.05129),(-0.04181,0.96266,-0.26745,0.02754),(0.03773,0.26901,0.96240,0.47705),(0.00000,0.00000,0.00000,1.00000))

# 0deg robot.root [4,4]((0.99500,0.09983,-0.00000,0.01710),(-0.09977,0.99441,0.03462,-0.09466),(0.00346,-0.03445,0.99940,0.60432),(0.00000,0.00000,0.00000,1.00000))

# two feet
robot.setRoot(((0.994864055284,0.000210089058006,0.101219896104,0.0274106623863),(1.36027504855e-05,0.999997559482,-0.00220926360724,0.143843868989),(-0.101220113217,0.00219929382048,0.994861624442, 0.646921914726),(0,0,0,1)))

# 0deg
# robot.root [4,4]((0.994862921513, 0.000210047741424, 0.101231039102, 0.0274148975459),(1.3690802568e-05, 0.999997559008, -0.00220947747375, 0.143849747317),(-0.101231256093, 0.00219951314872, 0.994860490185, 0.646921274968),(0, 0, 0, 1))
# 10deg
# robot.root [4,4]((0.962169712502,0.000588801121243,0.272450174631,0.13933634465),(1.369076794e-05,0.999997559008,-0.00220947748355,0.143849747317),(-0.272450810525,0.00212962236724,0.962167355792,0.632333943842),(0,0,0,1))

# --- Task Dyn -----------------------------------------

class MetaTask6d(object):
    name=''
    opPoint=''
    dyn=0
    derivator=0
    task=0
    feature=0
    featureDes=0

    def opPointExist(self,opPoint):
        sigsP = filter(lambda x: x.getName().split(':')[-1] == opPoint, self.dyn.signals())
        sigsJ = filter(lambda x: x.getName().split(':')[-1] == 'J'+opPoint, self.dyn.signals())
        return len(sigsP)==1 & len(sigsJ)==1

    def defineDynEntities(self,dyn):
        self.dyn=dyn

    def createOpPoint(self,opPoint,opPointRef = 'right-wrist'):
        self.opPoint=opPoint
        if self.opPointExist(opPoint): return
        self.dyn.createOpPoint(opPoint,opPointRef)

    def createFeatures(self):
        self.feature    = FeaturePoint6d('feature'+self.name)
        self.featureDes = FeaturePoint6d('feature'+self.name+'_ref')
        self.feature.selec.value = '111111'
        self.feature.frame('current')

    def createTask(self):
        self.task = TaskDynPD('task'+self.name)
        self.task.dt.value = 1e-3

    def createGain(self):
        self.gain = GainAdaptive('gain'+self.name)
        self.gain.set(1050,45,125e3)

    def plugEverything(self):
        self.feature.sdes.value = self.featureDes.name
        plug(self.dyn.signal(self.opPoint),self.feature.signal('position'))
        plug(self.dyn.signal('J'+self.opPoint),self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.dyn.velocity,self.task.qdot)
        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)

    def __init__(self,name,dyn,opPoint,opPointRef='right-wrist'):
        self.name=name
        self.defineDynEntities(dyn)
        self.createOpPoint(opPoint,opPointRef)
        self.createFeatures()
        self.createTask()
        self.createGain()
        self.plugEverything()

    @property
    def ref(self):
        return self.featureDes.position.value

    @ref.setter
    def ref(self,m):
        self.featureDes.position.value = m

# Task right hand
task=MetaTask6d('rh',dyn,'task')
task.ref = ((0,0,-1,0.22),(0,1,0,-0.37),(1,0,0,.74),(0,0,0,1))

# Task LFoot: Move the right foot up.
taskLF=MetaTask6d('lf',dyn,'lf','left-ankle')
taskLF.ref = ((1,0,0,0.0),(0,1,0,+0.29),(0,0,1,.15),(0,0,0,1))

# --- TASK COM ------------------------------------------------------
dyn.setProperty('ComputeCoM','true')

featureCom    = FeatureGeneric('featureCom')
featureComDes = FeatureGeneric('featureComDes')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureCom.sdes.value = 'featureComDes'
featureComDes.errorIN.value = (0.0478408688115,-0.0620357207995,0.684865189311)

taskCom = TaskDynPD('taskCom')
taskCom.add('featureCom')
plug(dyn.velocity,taskCom.qdot)
taskCom.dt.value = 1e-3

gCom = GainAdaptive('gCom')
plug(taskCom.error,gCom.error)
plug(gCom.gain,taskCom.controlGain)
# Work from .04 to .09 gCom.set 1050 45 125e3
# Good behavior gCom.set 1080 15 125e3
# Low gains gCom.set 400 1 39e3
# Current choice
gCom.set(1050,45,125e3)

# --- SOT Dyn OpSpaceH --------------------------------------
# SOT controller.
sot = SolverOpSpace('sot')
sot.setSize(30)
#sot.damping.value = 2e-2
sot.breakFactor.value = 10

plug(dyn.inertiaReal,sot.matrixInertia)
plug(dyn.dynamicDrift,sot.dyndrift)
plug(dyn.velocity,sot.velocity)

plug(sot.control,robot.control)

# For the integration of q = int(int(qddot)).
plug(sot.acceleration,robot.acceleration)

# ---- CONTACT -----------------------------------------
# Contact definition

supportLF = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
supportRF = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
# supportRF = ((0.12,-0.09,-0.09,0.12),(-0.08,-0.08,0.05,0.05),(-0.105,-0.105,-0.105,-0.105))

# Left foot contact
contactLF = MetaTask6d('contact_lleg',dyn,'lf','left-ankle')
contactLF.feature.frame('desired')
sot.addContactFromTask(contactLF.task.name,'LF')
sot._LF_p.value = supportLF

# Right foot contact
contactRF = MetaTask6d('contact_rleg',dyn,'rf','right-ankle')
contactRF.feature.frame('desired')
sot.addContactFromTask(contactRF.task.name,'RF')
sot._RF_p.value = supportRF

# constraint order
#                               19  16
#     ^ Front +X                +---+
#     |                 9   6   | R |
#     +--> Right -Y     +---+   |   |
#                       |   |   +---+
#       ^X              | L |  18   17(-8,-45)
#    Y  |               +---+   (-8,7)
#    <--+              8    7(-8,-7)
#                      (-8,45)

# --- TRACE ----------------------------------------------

from dynamic_graph.tracer import *
from dynamic_graph.tracer_real_time import *
tr = TracerRealTime('tr')
tr.setBufferSize(10485760)
tr.open('/tmp/','dyn_','.dat')
tr.add('dyn.position','')
tr.start()

#robot.periodicCall addSignal tr.triger

#tr.add('p6.error','position')
tr.add('featureCom.error','comerror')
tr.add('dyn.com','com')
tr.add('sot.zmp','')
#tr.add('sot.qdot','')
tr.add('dyn.position','state')
tr.add('dyn.ffposition','ffposition')
# tr.add('gCom.gain','')
# tr.add('gCom.error','gerror')

tr.add('sot.control','')

# --- RUN ------------------------------------------------
def inc():
    robot_wrapper.increment(dt)
    tr.triger.recompute( robot_wrapper.control.time )

featureComDes.errorIN.value = (0.06,0,0.8)
sot.push('taskCom')


#for i in range(2100):
#    inc()

#tr.dump()


