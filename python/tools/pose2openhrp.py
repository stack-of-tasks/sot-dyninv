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


pattern = "<property name=\"%s\" value=\"%f\"/>"

import random
q=[random.random() for i in range(36)]

import math
for i in range(len(q)-6):
    print pattern % (jointNames[i],q[i+6])

u=[random.random() for i in range(3)]
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



