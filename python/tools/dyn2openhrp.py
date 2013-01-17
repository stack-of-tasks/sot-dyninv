# This is for HRP2-14 only. Define the variable q as the
# current pose, and display it under the shape of a project OpenHRP.


#import random
#q=[random.random() for i in range(36)]
#q=dyn.position.value
q=[0, 0, 0.6487, 0, 0, 0, 0, 0, -0.453786, 0.872665, -0.418879, 0, 0, 0, -0.453786, 0.872665, -0.418879, 0, 0, 0, 0, 0, 0.261799, -0.174533, 0, -1.3, 0, -0.5, 0.174533, 0.261799, 0.174533, 0, -1.3, 0, -0.5, 0.174533 ]




from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
import dynamic_graph.script_shortcuts
import math

# --- Conversion

jointNames = ("RLEG_JOINT0",
"RLEG_JOINT1",
"RLEG_JOINT2",
"RLEG_JOINT3",
"RLEG_JOINT4",
"RLEG_JOINT5",
"LLEG_JOINT0",
"LLEG_JOINT1",
"LLEG_JOINT2",
"LLEG_JOINT3",
"LLEG_JOINT4",
"LLEG_JOINT5",
"CHEST_JOINT0",
"CHEST_JOINT1",
"HEAD_JOINT0",
"HEAD_JOINT1",
"RARM_JOINT0",
"RARM_JOINT1",
"RARM_JOINT2",
"RARM_JOINT3",
"RARM_JOINT4",
"RARM_JOINT5",
"RARM_JOINT6",
"LARM_JOINT0",
"LARM_JOINT1",
"LARM_JOINT2",
"LARM_JOINT3",
"LARM_JOINT4",
"LARM_JOINT5",
"LARM_JOINT6")


pattern = "<property name=\"%s.angle\" value=\"%f\"/>"

for i in range(len(q)-6):
    print pattern % (jointNames[i],q[i+6])


toUT=PoseRollPitchYawToPoseUTheta('toUT')
#plug(dyn.position,toUT.sin)
toUT.sin.value = tuple(q[0:6])
toUT.sout.recompute(1)
u=toUT.sout.value[3:6]
theta=math.sqrt(sum(v*v for v in u))
if theta>1e-4:
    (ux,uy,uz)=[v/theta for v in u]
else:
    theta=0
    (ux,uy,uz)=[0,0,0]

print "<property name=\"WAIST.translation\" value=\"%f %f %f\"/>" % (q[0],q[1],q[2])
print "<property name=\"WAIST.angle\" value=\"0\"/>"
print "<property name=\"WAIST.rotation\" value=\"%f %f %f %f\"/>" % (ux, uy, uz, theta)

bushJointNames = ("RLEG_BUSH_ROLL","RLEG_BUSH_PITCH","RLEG_BUSH_Z",
"LLEG_BUSH_ROLL","LLEG_BUSH_PITCH","LLEG_BUSH_Z")
for i in range(len(bushJointNames)):
    print pattern % (bushJointNames[i],0)




print " <property name=\"RHAND_JOINT4.angle\" value=\"0.0 \"/>"
print " <property name=\"RHAND_JOINT1.angle\" value=\"0.0 \"/>"
print " <property name=\"RHAND_JOINT3.angle\" value=\"0.0 \"/>"
print " <property name=\"RHAND_JOINT0.angle\" value=\"0.0 \"/>"
print " <property name=\"RHAND_JOINT2.angle\" value=\"0.0 \"/>"
print " <property name=\"LHAND_JOINT2.angle\" value=\"0.0 \"/>"
print " <property name=\"LHAND_JOINT1.angle\" value=\"0.0 \"/>"
print " <property name=\"LHAND_JOINT4.angle\" value=\"0.0 \"/>"
print " <property name=\"LHAND_JOINT0.angle\" value=\"0.0 \"/>"
print " <property name=\"LHAND_JOINT3.angle\" value=\"0.0 \"/>"
print   ""
print " <property name=\"setupDirectory\" value=\"/opt/grx3.0/HRP2LAAS/bin\"/>"
print " <property name=\"imageProcessTime\" value=\"0.033\"/>"
print " <property name=\"imageProcessor\" value=\"\"/>"
print " <property name=\"controller\" value=\"HRP2LAASControllerFactory\"/>"
print " <property name=\"isRobot\" value=\"true\"/>"
print " <property name=\"controlTime\" value=\"0.0010\"/>"
print " <property name=\"setupCommand\" value=\"HRP2Controller$(BIN_SFX)\"/>"



#import robotviewer
#viewer = robotviewer.client('XML-RPC')
#viewer.updateElementConfig('hrp',[float(val) for val in q]+10*[0.0])



posinit=""
for v in q[6:]:
    posinit+=str(v*180/math.pi)+" "
print "seq.sendMsg(\":joint-angles \"+\""   + posinit + "\"+\"   0 0 0 0 0 0 0 0 0 0  2.5\")"
