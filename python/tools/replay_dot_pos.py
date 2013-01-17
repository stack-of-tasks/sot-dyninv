'''
 This version does not use the free floating position for the 'base' of the robot,
 A thread is used to display, then, 'go', 'stop', 'next' are available
 it relies completely on the outputs of dyn_position
     Use it as:
           py replay.py dyn_position.dat
           and then: 'go'
Author: O. Ramos - LAAS/CNRS
'''

#sys.path.append('/home/nmansard/src/hpp2/sot-dyninv/python')
from numpy import *
from dynamic_graph.script_shortcuts import optionalparentheses
import sys


#------------------------------------------------------
# Try to read the input file, if possible
try:
    #fileName = sys.argv[1]                       # Input File ([2] if using 'py', [1] if 'python')
    prefix='mocap/'
    fileName = prefix+'yoganmsd.pos'
    file_dyn_position = open(fileName,'r')       # Open the file
#    fileffName = prefix+'ffposition.dat'
#    file_dyn_ffposition = open(fileffName,'r')       # Open the file
except:
    print "Please, specify the input file"
    sys.exit(0)


#------------------------------------------------------
# Initialize robotviewer, if possible
try:
    import robotviewer
    clt = robotviewer.client('XML-RPC')
except:
    print "No robot viewer, sorry."
    sys.exit(0)


#-----------------------------------------------------
# Main Loop
robotTime = 1
hq=[]
def inc():

    global robotTime, q, hq
    while True:
        pos_angles = file_dyn_position.readline()
        if len(pos_angles)==0: break
        print float(pos_angles.split()[0])
        if int(round(float(pos_angles.split()[0])/0.005))==robotTime:
            print "OK ", int(float(pos_angles.split()[0])/0.005),robotTime
            pos_angles = pos_angles.split()[1:]
            pos_angles = 6*[0,] + [float(val) for val in pos_angles]
            break
        else:
            print "nonono ",int(round(float(pos_angles.split()[0])/0.005)),robotTime
            sys.exit(0)
#    while True:
#        pos_ffangles = file_dyn_ffposition.readline()
#        if len(pos_ffangles)==0: break
#        if float(pos_ffangles.split()[0])==robotTime:
#            pos_angles[0:6] = pos_ffangles.split()[1:]
#            break

    if len(pos_angles)==0:
        stop()
        print " -- File reading completed --"
        return
    else:
        robotTime += 1

        if robotTime%1000==0: print "\nTime: " + str(robotTime)

        # Add 10 values for the hand DOFs
        #pos_angles += 10*[0.0]

        # Plot only if the values are not 'NaN'
        plot=1
        for qi in pos_angles:
            if isnan(qi):
                plot=0
                print ' -- Not displaying: nan positions --'
        if plot==1:
            clt.updateElementConfig('hrp',pos_angles)
            q = pos_angles
            hq.append( q )

    global showNumber
    if showNumber:
        print 't = ', robotTime
    time.sleep(.005)


def hlp_showNumber():
    global showNumber
    if showNumber: showNumber=False
    else:          showNumber=True
showNumber=False

#-----------------------------------------------------
# Thread

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
@optionalparentheses
def n(): hlp_showNumber()


go()
