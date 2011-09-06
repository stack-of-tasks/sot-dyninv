'''
==========================================================================================
  Mocap position files replay -- author O. Ramos -- rewrite NMsd
  ---------------------------------

  Read the positions files coming from testHuma, and make them available for 
  control or display.

  The files are read and stored at the beginning, so that any position of the file
  can be accessed at any time, that is, 'iter' can be changed to see the robot
  at a different moment of time

=========================================================================================
'''

import sys
import time
from numpy import *
sys.path.append('..')
from matrix_util import matrixToTuple,rpy2tr,distVector,generateOrthonormalM,tr2rpy,matrixToRPY


#--------------------------------------------------------------------------
#--------------------------------------------------------------------------
#--------------------------------------------------------------------------

class JointData:
    def __init__(self):
        self.name = ""
        self.positions = []
        self.Ki = None
        self.displayObject = ""


class LinkData:
    def __init__(self):
        self.body1 = ""
        self.body2 = ""
        self.displayObject = ""

class MocapParser:
    # Mocap variables
    M = 1
    KM = 2
    MK = 3
    KMK = 4
    classicalLinkMap = \
        {'lleg0':['waist','Lleg1'],     'lleg1':['Lleg1','Lleg2'],  'lleg2':['Lleg2','Lfoot'], \
             'rleg0':['waist','Rleg1'],     'rleg1':['Rleg1','Rleg2'],  'rleg2':['Rleg2','Rfoot'],\
             'lshoulder':['chest','Larm1'], 'larm1':['Larm1','Larm2'],  'larm2':['Larm2','Lhand'],\
             'rshoulder':['chest','Rarm1'], 'rarm1':['Rarm1','Rarm2'],  'rarm2':['Rarm2','Rhand'],\
             'head':['head','chest'],  'chest':['chest','waist'] }
    # Constructer:
    def __init__(self, mocapDir=None, skeletumFileName=None):
        self.qs = []
        self.Kw = None
        self.jointsNumber = 0
        self.joints = None
        self.links = {}
        self.method = MocapParser.M
        self.iter = 0
        self.increment = +1
        self.delayTime = 0.0
        self.with_dispJoints = True
        self.with_dispRobot = True
        self.with_dispLinks = False
        self.with_dispNumber = False
        self.maxTime = -1

        if skeletumFileName!=None: self.openQFile( skeletumFileName)
        if mocapDir!=None:
            self.openKwFile( mocapDir+'Kw.dat')
            self.openKiFile( mocapDir+'Ki.dat')
            self. openMocapPositionFile( mocapDir )

    # --- read files ----
    def openKwFile(self,KwFile):
        '''
        Try to open the Transformation Matrices WRM (corresponding to the coordinate
        system of each joint).
        '''
        fileKw  = open(KwFile,'r')
        linesKw = fileKw.readlines()
        self.Kw = matrix(eval(linesKw[0].split()[2][5:]))

    def openKiFile(self,KiFile):
        '''
        Ki contains one line for each joint, with its name and the offset matrix Ki.
        Try to open and read Ki and set the names for the joints (using Ki.dat).
        '''
        self.jointsNumber = len( open(KiFile,'r').readlines() )
        self.joints = [ JointData() for i in range(self.jointsNumber) ]
        try:
            KiFile = open(KiFile,'r')
            for val in range(self.jointsNumber):
                KiLine = KiFile.readline()[5:].split(' ')
                self.joints[val].Ki = matrix( eval(KiLine[1][5:]) )
                self.joints[val].name = KiLine[0]
        except:
            print " -- There is no Ki --"
            sys.exit(0)


    def checkMaxTime( self,l,msg=None ):
        if self.maxTime<0: self.maxTime = l
        elif l<self.maxTime:
            if msg==None: msg = ''
            else: msg = 'in '+msg
            print 'Time length is not concordant',msg,'. Reducing it.'
            self.maxTime = len(self.qs)


    def openQFile(self,skeletumFileName):
        self.qs = []
        for l in open(skeletumFileName,'r').readlines():
            self.qs.append( list( eval(l.split()[0][4:]) ) )  # + 10*[0.0]
        self.checkMaxTime( len(self.qs) )

    def openMocapPositionFile(self,dataFolder):
        for i,j in enumerate(self.joints):
            for l in open(dataFolder+'WRM_'+j.name,'r').readlines():
                j.positions.append( eval(l[5:]) )
            self.checkMaxTime( len(j.positions),'joint '+str(i) )

    # --- Other init ---
    def setLinksMap( self, linkDef = classicalLinkMap ):
        self.links = dict()
        for name,(b1,b2) in linkDef.iteritems():
            self.links[ name ] = LinkData()
            for i,j in enumerate(self.joints):
                if j.name==b1: self.links[name].body1 = i
                if j.name==b2: self.links[name].body2 = i

    def assignDisplayObjects(self):
        for i,j in enumerate(self.joints):
            j.displayObject = 'axis'+str(i+1)
        for n,l in self.links.iteritems():
            l.displayObject = n

    # --- data access
    def jointPosition_M(self,joint,iter):
        return self.joints[joint].positions[iter]
    def jointPosition_MK(self,joint,iter):
        M = matrix(self.jointPosition_M(joint,iter))
        return matrixToTuple( M*self.joints[joint].Ki )
    def jointPosition_KMK(self,joint,iter):
        M = matrix(self.jointPosition_M(joint,iter))
        return matrixToTuple( self.Kw*M*self.joints[joint].Ki )
    def jointPosition_KM(self,joint,iter):
        M = matrix(self.jointPosition_M(joint,iter))
        return matrixToTuple( self.Kw*M )
   
    def setPositionMethod(self,method): # M KM KMK
        if( method=='M'): self.method = self.M 
        if( method=='KM'): self.method = self.KM 
        if( method=='MK'): self.method = self.MK 
        if( method=='KMK'): self.method = self.KMK 
    def jointPosition(self,j,t=None):
        if t==None: t=self.iter
        if self.method == self.M:
            return self.jointPosition_M(j,t)
        elif self.method == self.KM:
            return self.jointPosition_KM(j,t)
        elif self.method == self.MK:
            return self.jointPosition_MK(j,t)
        elif self.method == self.KMK:
            return self.jointPosition_KMK(j,t)
    def jointRPY( self,j,t): return matrixToRPY( self.jointPosition(j,t) )

    def linkPosition(self,l,t):
        # Get the 'vector' joining the positions of two consecutive transformation matrices
        # and use it to generate a orthonormal matrix (rotation matrix), then get roll, pitch, yaw
        M1 = self.jointPosition( self.links[l].body1,t )
        M2 = self.jointPosition( self.links[l].body2,t )
        return [ M1[0][3],M1[1][3],M1[2][3] ]+ tr2rpy( generateOrthonormalM( distVector(M1,M2) ) )

    # --- iter update
    def forward( self ) : self.increment = +1
    def backward( self ) : self.increment = -1
    def pause( self ) : self.increment = 0
    def incTime( self ):
        self.iter += self.increment
        if self.iter<0:
            print " -- Start of file reached (when going backwards). --"
            self.increment = 0
            self.iter=0
        elif self.iter >= self.maxTime:
            print " -- End of file containing the positions. --"
            self.increment = 0
            self.iter=self.maxTime-1

    # --- display update
    def setRobotViewer( self, rview, robotName = None ):
        self.rview = rview
        if robotName==None: self.robotName = "hrp"
        else: self.robotName = robotName
        self.hideLinks()
        self.hideJoints()

    def dispJoints( self ):
        if self.with_dispJoints:
            for j in range(self.jointsNumber):
                self.rview.updateElementConfig( self.joints[j].displayObject,self.jointRPY(j,self.iter) )
    def dispLinks( self ):
        if self.with_dispLinks:
            for n,l in self.links.iteritems():
                self.rview.updateElementConfig( l.displayObject,self.linkPosition(n,self.iter) )
    def dispRobot(self):
        if self.with_dispRobot:
            self.rview.updateElementConfig( self.robotName, list(self.qs[self.iter])+10*[0,] )
    def hideJoints( self ):
        for j in self.joints:
            self.rview.updateElementConfig( j.displayObject,[ 0, 0, -1, 0, 0, 0 ])
    def hideLinks( self ):
        for j in self.links.values():
            self.rview.updateElementConfig( j.displayObject,[ 0, 0, -1, 0, 0, 0 ])
    def alignHrpFrames( self ):
        namesT1 = ['Larm2','Lhand','Lleg1','Lleg2','Rarm2','Rhand','Rleg1','Rleg2','head','chest']
        namesT2 = ['Larm1','Rarm1']
        transf1 = matrix(rotate('x',-pi))*matrix(rotate('z',-pi/2))
        transf2 = matrix(rotate('y',pi/2))*matrix(rotate('z',pi/2))
        for joint in self.joints:
            if joint.name in namesT1: joint.Ki = joints.Ki*transf1
            if joint.name in namesT2: joint.Ki = joints.Ki*transf2
            
    def update( self ):
        self.incTime()
        if self.delayTime>0: time.sleep( self.delayTime )
        if self.with_dispNumber: print self.iter
        self.refresh()
    def refresh( self ):
        self.dispJoints()
        self.dispLinks()
        self.dispRobot()


    #=======================================================================================
    #                        Helper functions
    #=======================================================================================


    def hlp_mocapCoord(self):
        self.with_dispJoints = not self.with_dispJoints
        self.hideJoints()

    def hlp_skeleton(self):
        self.with_dispLinks = not self.with_dispLinks
        self.hideLinks()

    def hlp_showNumber(self):
        self.with_dispNumber = not self.with_dispNumber

    def hlp_toggle(self):
        if self.method == self.M:
            self.method = self.MK
        elif self.method == self.MK:
            self.method = self.KMK
        elif self.method == self.KMK:
            self.method = self.KM
        elif self.method == self.KM:
            self.method = self.M
        print self.method
        self.disp_toggle()

    def disp_toggle(self):
        if self.method == self.MK:
            print " --- wRm*Ki --- "
        elif self.method == self.KMK:
            print " --- Kw*wRm*Ki --- "
        elif self.method == self.KM:
            print " --- Kw*wRm --- "
        elif self.method == self.M:
            print " --- wRm --- "




class MocapParserTimed(MocapParser):
    def __init__(self,*args):
        MocapParser.__init__(self,*args)
        self.timeScale = 1.0
        self.timeDecimal = 0.0
    def incTime(self):
        if self.iter != floor(self.timeDecimal): self.timeDecimal = self.iter
        self.timeDecimal += float(self.increment)/self.timeScale
        self.iter = floor(self.timeDecimal)
    def jointPosition(self,joint,t=None):
        if t==None: t=self.timeDecimal
        p1=matrix(MocapParser.jointPosition(self,joint,int(floor(t))))
        p2=matrix(MocapParser.jointPosition(self,joint,int(ceil(t))))
        dt=self.timeDecimal%1
        p1[0:3,3]*=dt
        p1[0:3,3]+=(1-dt)*p2[0:3,3]
        return matrixToTuple(p1)
