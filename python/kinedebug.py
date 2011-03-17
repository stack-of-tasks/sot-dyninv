from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.matlab import matlab
from MetaTask6d import MetaTask6d,toFlags
from attime import attime

from robotSpecific import pkgDataRootDir,modelName,robotDimension,initialConfig,gearRatio,inertiaRotor
robotName = 'hrp14small'

from numpy import *
def totuple( a ):
    al=a.tolist()
    res=[]
    for i in range(a.shape[0]):
        res.append( tuple(al[i]) )
    return tuple(res)


# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------
# --- ROBOT SIMU ---------------------------------------------------------------

robotDim=robotDimension[robotName]
robot = RobotSimu("robot")
robot.resize(robotDim)

robot.set( initialConfig[robotName] )
dt=5e-3

# --- VIEWER -------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
# --- VIEWER -------------------------------------------------------------------
try:
    import robotviewer

    def stateFullSize(robot):
        return [float(val) for val in robot.state.value]+10*[0.0]
    RobotSimu.stateFullSize = stateFullSize

    robot.viewer = robotviewer.client('XML-RPC')
    robot.viewer.updateElementConfig('hrp',robot.stateFullSize())

    def incrementView( robot,dt ):
        robot.incrementNoView(dt)
        robot.viewer.updateElementConfig('hrp',robot.stateFullSize())
    RobotSimu.incrementNoView = RobotSimu.increment
    RobotSimu.increment = incrementView
except:
    print "No robot viewer, sorry."
    robot.viewer = None

# --- MAIN LOOP ------------------------------------------

qs=[]
def inc():
    attime.run(robot.control.time+1)
    robot.increment(dt)
    tr.triger.recompute( robot.control.time )
    qs.append(robot.state.value)

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
def next(): runner.once()

# --- DYN ----------------------------------------------------------------------
# --- DYN ----------------------------------------------------------------------
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

#dyn.setProperty('ComputeBackwardDynamics','true')
#dyn.setProperty('ComputeAccelerationCoM','true')

robot.control.unplug()

# --- Task Dyn -----------------------------------------
class MetaTaskKine6d( MetaTask6d ):
    def createTask(self):
        self.task = Task('task'+self.name)

    def createGain(self):
        self.gain = GainAdaptive('gain'+self.name)
        self.gain.set(0.1,0.1,125e3)
    def plugEverything(self):
        self.feature.sdes.value = self.featureDes.name
        plug(self.dyn.signal(self.opPoint),self.feature.signal('position'))
        plug(self.dyn.signal('J'+self.opPoint),self.feature.signal('Jq'))
        self.task.add(self.feature.name)
        plug(self.task.error,self.gain.error)
        plug(self.gain.gain,self.task.controlGain)
    def keep(self):
        self.feature.position.recompute(self.dyn.position.time)
        self.feature.keep()

# Task right hand
taskRH=MetaTaskKine6d('rh',dyn,'rh','right-wrist')
taskRH.ref = ((0,0,-1,0.22),(0,1,0,-0.37),(1,0,0,.74),(0,0,0,1))
taskLH=MetaTaskKine6d('lh',dyn,'lh','left-wrist')
#TODO taskLH.ref = ((0,0,-1,0.22),(0,1,0,0.37),(1,0,0,.74),(0,0,0,1))

# Task LFoot: Move the right foot up.
taskRF=MetaTaskKine6d('rf',dyn,'rf','right-ankle')
taskLF=MetaTaskKine6d('lf',dyn,'lf','left-ankle')

# --- TASK COM ------------------------------------------------------
dyn.setProperty('ComputeCoM','true')

featureCom    = FeatureGeneric('featureCom')
featureComDes = FeatureGeneric('featureComDes')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureCom.sdes.value = 'featureComDes'
featureComDes.errorIN.value = (0.0478408688115,-0.0620357207995,0.684865189311)

taskCom = Task('taskCom')
taskCom.add('featureCom')

gCom = GainAdaptive('gCom')
plug(taskCom.error,gCom.error)
plug(gCom.gain,taskCom.controlGain)
gCom.set(1,1,1)

# --- TASK SUPPORT --------------------------------------------------
featureSupport    = FeatureGeneric('featureSupport')
plug(dyn.com,featureSupport.errorIN)
plug(dyn.Jcom,featureSupport.jacobianIN)

taskSupport=TaskInequality('taskSupport')
taskSupport.add(featureSupport.name)
taskSupport.selec.value = '011'
taskSupport.referenceInf.value = (-0.08,-0.045,0)    # Xmin, Ymin
taskSupport.referenceSup.value = (0.11,0.335,0)  # Xmax, Ymax
taskSupport.dt.value=dt

# --- TASK POSTURE --------------------------------------------------
featurePosture    = FeatureGeneric('featurePosture')
featurePostureDes = FeatureGeneric('featurePostureDes')
plug(dyn.position,featurePosture.errorIN)
featurePosture.sdes.value = 'featurePostureDes'
featurePosture.jacobianIN.value = totuple( identity(robotDim) )
featurePostureDes.errorIN.value = dyn.position.value

taskPosture = Task('taskPosture')
taskPosture.add('featurePosture')

gPosture = GainAdaptive('gPosture')
plug(taskPosture.error,gPosture.error)
plug(gPosture.gain,taskPosture.controlGain)
gPosture.set(1,1,1)

postureSelec = range(0,3)      # right leg
postureSelec += range(6,9)     # left leg
postureSelec += range(12,16)   # chest+head
#postureSelec += range(16,19)   # right arm
#postureSelec += range(23,26)   # left arm
featurePosture.selec.value = toFlags(postureSelec)

# --- TASK JL ------------------------------------------------------
taskJL = TaskJointLimits('taskJL')
plug(dyn.position,taskJL.position)
plug(dyn.lowerJl,taskJL.referenceInf)
plug(dyn.upperJl,taskJL.referenceSup)
taskJL.dt.value = dt
taskJL.selec.value = toFlags(range(6,robotDim))

# --- SOT Dyn OpSpaceH --------------------------------------
# SOT controller.
sot = SolverKine('sot')
sot.setSize(robotDim)
#sot.damping.value = 2e-2

plug(sot.control,robot.control)

def sot_addContact( sot,metaTask ):
    metaTask.keep()
    sot.push(metaTask.task.name)
import new
sot.addContact = new.instancemethod(sot_addContact, sot, sot.__class__)


# --- TRACE ----------------------------------------------

from dynamic_graph.tracer import *
from dynamic_graph.tracer_real_time import *
tr = TracerRealTime('tr')
tr.setBufferSize(10485760)
tr.open('/tmp/','dyn_','.dat')
tr.start()

#robot.periodicCall addSignal tr.triger

#tr.add('p6.error','position')
tr.add('featureCom.error','comerror')
tr.add('dyn.com','com')
tr.add('dyn.position','state')
# tr.add('gCom.gain','')
# tr.add('gCom.error','gerror')

tr.add('sot.control','')

# --- shortcuts -------------------------------------------------
qn=taskJL.normalizedPosition
q=taskJL.position
comref=featureComDes.errorIN

@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def dump():         tr.dump()
@optionalparentheses
def status():       print runner.isPlay

# --- A FIRST MOTION ---------------------------------------------

if 0:
  sot.addContact(taskLF)
  sot.addContact(taskRF)
  sot.push(taskCom.name)
  sot.push(taskRH.task.name)

  taskRH.ref = ((0,0,-1,0.22),(0,1,0,-0.5),(1,0,0,1.24),(0,0,0,1))
  taskRH.gain.setConstant(1)
  comref.value=( 0.059949815729,  0.2098857010921,  0.750396505072 )

# --- RUN ------------------------------------------------

#sot.damping.value=.1
sot.addContact(taskLF)
sot.addContact(taskRF)
#sot.push(taskCom.name)
sot.push(taskJL.name)
sot.push(taskSupport.name)
sot.push(taskRH.task.name)

taskRH.ref = ((0,0,-1,0.22),(0,1,0,-0.5),(1,0,0,1.24),(0,0,0,1))
taskRH.gain.setConstant(1)
comref.value=( 0.059949815729,  0.2098857010921,  0.750396505072 )

tr.add('taskJL.normalizedPosition','qn')
robot.after.addSignal('taskJL.normalizedPosition')
robot.after.addSignal('taskJL.task')

# impossible position (ok with damping):
taskRH.ref = ((0,0,-1,0.32),(0,1,0,-0.75),(1,0,0,1.24),(0,0,0,1))
# feasible
taskRH.ref = ((0,0,-1,0.32),(0,1,0,-0.63),(1,0,0,1.24),(0,0,0,1))

# --- UMBRELLA ----------------------------------------------------
# --- UMBRELLA ----------------------------------------------------
# --- UMBRELLA ----------------------------------------------------
import copy

# M is the position of the top of the umbrella.
M=identity(4)
M[0][3]=.595 # Stick lenght
M[0][3]=.45  # without unbrella lenght
M[2][3]=-.19 # Lenght of the wrist
widthUmbrella=.41 # Width of the umbrella
# List of the polygon defining the umbrella hat.
umbrellaPolygon = [ [ +1,+1 ],[+1,-1],[-1,-1],[-1,+1] ]
umbrellaPointNames = [ 'a','b','c','d' ]

# Creation of the Op Point modifiers for each vertex of the polygon.
dyn.createJacobian('Jrh0','right-wrist')
umbrellaModif = dict()
for i in range(len(umbrellaPolygon)):
    point=umbrellaPolygon[i]
    name=umbrellaPointNames[i]
    modif=OpPointModifier('modif'+name)
    plug(dyn.rh,modif.positionIN)
    plug(dyn.Jrh0,modif.jacobianIN)
    Mi=copy.copy(M)
    Mi[1][3]+=widthUmbrella*point[0]
    Mi[2][3]+=widthUmbrella*point[1]
    modif.setTransformation(totuple(Mi))
    umbrellaModif[name]=modif

# Creation of the features for the edges of the polygon.
umbrellaFeature=dict()
for i in range(len(umbrellaPointNames)):
    nameA=umbrellaPointNames[i]
    if i+1<len(umbrellaPointNames):
        nameB=umbrellaPointNames[i+1]
    else:
        nameB=umbrellaPointNames[0]
    A=umbrellaModif[nameA]
    B=umbrellaModif[nameB]
    feature=FeatureProjectedLine('feature'+nameA+nameB)
    plug(A.position,feature.xa)
    plug(A.jacobian,feature.Ja)
    plug(B.position,feature.xb)
    plug(B.jacobian,feature.Jb)
    feature.xc.value = (0,0)
    umbrellaFeature[nameA+nameB]=feature

# Adding everything to the tracer
for name in umbrellaPointNames:
    tr.add(umbrellaModif[name].name+'.position',name)
    robot.after.addSignal(umbrellaModif[name].name+'.position')
for name,feature in umbrellaFeature.items():
    tr.add(feature.name+'.error','e'+name)
    robot.after.addSignal(feature.name+'.error')

# And finally, creating the task.
taskUmbrella=Task('taskUmbrella')
taskUmbrella.add(umbrellaFeature['ab'].name)
taskUmbrella.add(umbrellaFeature['bc'].name)
taskUmbrella.add(umbrellaFeature['cd'].name)
taskUmbrella.add(umbrellaFeature['da'].name)
taskUmbrella.controlGain.value = 10
tr.add('taskUmbrella.error','error')

taskAB=Task('taskAB')
taskAB.add(umbrellaFeature['bc'].name)
taskAB.controlGain.value = 10

A=umbrellaModif['a'].position
B=umbrellaModif['b'].position
C=umbrellaModif['c'].position
D=umbrellaModif['d'].position

AB=umbrellaFeature['ab']
BC=umbrellaFeature['bc']
CD=umbrellaFeature['cd']
DA=umbrellaFeature['da']

# Position at starting point: error is null
CD.xc.value=(-0.001012500000,-0.609998000000)
DA.xc.value=CD.xc.value
AB.xc.value=(0.818988000000,0.210001000000)
BC.xc.value=AB.xc.value

CD.xc.value=(0.0,-0.6)
DA.xc.value=CD.xc.value
AB.xc.value=(0.8,0.2)
BC.xc.value=AB.xc.value

robot.set((0.15787508826861599, 0.045511651751362708, 0.58971500036966162, -0.0040107700215134432, -0.45564238401733737, -0.52686538178073516, 0.44716913166621519, 0.30513254004984031, 0.080434789095732442, 1.1705171307901581, -0.84322874602000764, -0.085337729992559314, 0.34519179463400057, 0.51096319874334073, 0.39729314896741563, 0.79637015282207479, -0.74524853791029155, -0.3113477017777998, 0.59528786833733205, -0.0060236125952612008, 0.042089394399565112, 0.028985249939047025, -0.37671405575009481, -0.29002038141976838, -0.43761993851863368, -2.0077185039386283, -0.50346194284379353, 1.3345805041453564, 0.030948677237969232, 0.13701711567791786, 0.43501758221426229, 0.11316541844882388, -0.17894103295334104, 0.00080661744280727005, 0.054276071429870718, 0.0050874263605387498))
sot.clear()
#sot.push(taskUmbrella.name)
sot.push(taskAB.name)
inc()

@attime(600)
def m1():
    "adding COM"
#    comref.value = ( -0.05,0.06,0.7 )
    comref.value = ( 0.05,0.06,0.5 )
    sot.addContact(taskLF)
    sot.addContact(taskRF)
    sot.push(taskCom.name)

attime(1000,stop,'pause')
attime(1000,dump,'dump')
