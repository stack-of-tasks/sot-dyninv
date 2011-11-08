'''
 This version does not use the free floating position for the 'base' of the robot,
 A thread is used to display, then, 'go', 'stop', 'next' are available
 it relies completely on the outputs of dyn_position
     Use it as:
           py replay.py dyn_position.dat
           and then: 'go'
Author: O. Ramos - LAAS/CNRS
'''

from numpy import *
from dynamic_graph.script_shortcuts import optionalparentheses
import sys
from optparse import OptionParser
import time



usage = "usage: py replay.py [options] args"
parser = OptionParser(usage=usage)
parser.add_option("-p", dest="prefix", help="prefix of the file to read, under the shape /path/prefix_ or prefix only (no / no _) to implicit /tmp/prefix_ .", default="dyn")
parser.add_option("-N", dest="frequency", help="Nominal frequency of the robot play, %default Hz by default.", default='200', type=int)
parser.add_option("-d", dest="delayTime", help="Delay between to images sent to robot viewer, default %default. Give auto to automatic estimation.", default="0")
parser.add_option("-s", dest="server", help="Name of server (CORBA or XML-RPC). Default is %default", default="XML-RPC")
parser.add_option("-r", dest="size", help="Robot size if file verification is needed.", default="-1", type=int)
parser.add_option("-n", dest="iter", help="Number of iteration to play.", default="-1", type=int)
parser.add_option("-z", dest="decimate", help="percentage of frame to use, should be <1", default="1", type=float)
parser.add_option("-R", dest="robotName", help="Robot on which the simulation is replayed. Default is %default", default="hrp2")
(options, args) = parser.parse_args()
parser.print_help()
print '\n -------------------------------------------------------------------------'


prefix = options.prefix
if len(prefix.split('/'))==1 and len(prefix.split('_'))==1:
    prefix='/tmp/'+prefix+'_'
robotSize = options.size
nbIter=options.iter
decimate=options.decimate
if( options.delayTime=="auto"): delayTime=0
else: delayTime=float(options.delayTime)

#------------------------------------------------------
# Try to read the input file, if possible
try:
    fileName = prefix+'position.dat'
    file_dyn_position = open(fileName,'r')       # Open the file
except:
    print "Non valid input file ",fileName
    sys.exit(0)


#------------------------------------------------------
# Initialize robotviewer, if possible
try:
    import robotviewer
    clt = robotviewer.client(options.server)
except:
    print "No robot viewer, sorry."
    sys.exit(0)


#-----------------------------------------------------
nbImage=0
lastImage=0
robotTime = 2
hq=[]


def inc():

    global robotTime, q, hq, robotSize, nbImage, lastImage, decimate
    while True:
        pos_angles = [float(val) for val in  file_dyn_position.readline().split()[1:] ]
        if len(pos_angles)==0:
            print " -- File reading completed --"
            return False
        if robotSize==-1:
            robotSize=len(pos_angles)
            print "Robot size estimated to ",robotSize
        if len(pos_angles)==robotSize: break

    robotTime += 1
    if robotTime%1000==0: print "\nTime: " + str(robotTime)

    # Add 10 values for the hand DOFs for HRP2
    if options.robotName=="hrp2": pos_angles += 10*[0.0]

    # Plot only if the values are not 'NaN'
    nbImage+=decimate
    if floor(nbImage)>lastImage:
        lastImage=floor(nbImage)
        if reduce(lambda x,y: x and y,[isnan(q) for q in pos_angles]):
            print ' -- Not displaying: nan positions --'
        else:
            clt.updateElementConfig('hrp',pos_angles)
            q = pos_angles
            hq.append( q )

    return True

def hlp_showNumber():
    global showNumber
    if showNumber: showNumber=False
    else:          showNumber=True
showNumber=False

#-----------------------------------------------------
@optionalparentheses
def n():
    inc()
def incIter(dummy,nb):
    for i in range(nb): n()
setattr(n.__class__,'__add__',incIter)




t0=time.time()
iterNum=0
while inc():
    if nbIter>0 and iterNum>nbIter: break
    iterNum+=1
    time.sleep( delayTime )

t1=time.time()
dt=t1-t0
dtn=(iterNum+0.0)/options.frequency

if options.delayTime=="auto":
    print "Duration: ",dt," s"
    print "Nominal time :",dtn
    if( abs(dt-dtn)/iterNum < 1e-3 ):
        print "Non pertinent delay"
    else:
        if dt>dtn:
            print "use decimate -z ",dtn/dt
        else:
            print "use delay time -d ",(dtn-dt)/iterNum


