'''
============================================================
  Replay qs and WRM      -- author Oscar Ramos (rewr. NMsd)
 ---------------------------------

 The program uses the Mocap Parser to replay the mocap on robot viewer.
 Object are expected on robot viewer config file:
    -- hrp: the robot vrml
    -- axis1->axis15: the center (with orientation) of the joints.
    -- one object per link, named from the link-map.

 Use it as:
       py playJointsFile.py [options] args
 Options:
     - go:       start the simulation
     - stop:     stop the simulation
     - next:     single step of simulation
     - forward:  play forward (by default)
     - backward: play backward
     - replay:   start from the beginning
     - s:        show/hide the skeleton from the mocap
     - n:        show/hide frame numbers (display on screen)
     - t:        toggle between WRM, WRM*Ki, Kw*WRM*Ki, Kw*WRM

 Some variables can be changed iteratively:
     - mp.delayTime: float, to change the time between frames (default=0.0 seconds)
     - mp.iter: int, to go to a certain frame number

=================================================================
'''

from optparse import OptionParser
from mocap_parser import MocapParser
sys.path.append('/home/nmansard/src/hpp2/sot-dyninv/python')
from dynamic_graph.script_shortcuts import optionalparentheses
from random import random


# --- PARSER INIT ------------------------------------------------------------------
# --- PARSER INIT ------------------------------------------------------------------
# --- PARSER INIT ------------------------------------------------------------------

usage = "usage: py postureOpTask.py [options] args"
parser = OptionParser(usage=usage)
parser.add_option("-p", dest="opti_q_file", help="Specification of qs from mocap optimization (outputJoints)")
parser.add_option("-o", dest="opFolder", help="Folder containing the operational points (WRM,Ki,KW)")
parser.add_option("-d", dest="delayTime", help="Delay time between iterations (in sec). Default is %default", \
                      type=float, default="0")
parser.add_option("-s", dest="server", help="Name of server (CORBA or XML-RPC). Default is %default", \
                      metavar="NAME_SERVER", default="CORBA")
(options, args) = parser.parse_args()
parser.print_help()
print '\n -------------------------------------------------------------------------'


# --- DISPLAY INIT --------------------------------------------------------------------
# --- DISPLAY INIT --------------------------------------------------------------------
# --- DISPLAY INIT --------------------------------------------------------------------

# Initialize robotviewer, if possible
try:
    import robotviewer
    clt = robotviewer.client(options.server) # default is CORBA
except:
    print " -- WARNING!: No robot viewer (or not correct server)."
    class RobotViewerFaked:
        def update(*args): void
        def updateElementConfig(*args): void
    clt =RobotViewerFaked()


# --- PARSER INIT ------------------------------------------------------------------
# --- PARSER INIT ------------------------------------------------------------------
# --- PARSER INIT ------------------------------------------------------------------

mp = MocapParser(options.opFolder,options.opti_q_file)
mp.setLinksMap()
mp.assignDisplayObjects()
mp.setRobotViewer( clt )
mp.delayTime = options.delayTime
#mp.dispLinks = True

#=======================================================================================
#                                    THREAD
#=======================================================================================

from ThreadInterruptibleLoop import *
@loopInThread
def loop():
    mp.update()
runner=loop()

@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): mp.update()
@optionalparentheses
def forward():hlp_forward()
@optionalparentheses
def backward():hlp_backward()
@optionalparentheses
def replay():
    mp.iter=0
    mp.forward()
    runner.play()
@optionalparentheses
def s(): mp.hlp_skeleton()
@optionalparentheses
def m(): mp.hlp_mocapCoord()
@optionalparentheses
def n(): mp.hlp_showNumber()
@optionalparentheses
def t(): mp.hlp_toggle()
