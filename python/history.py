class History:
    def __init__(self,robot,freq=100):
        self.robot = robot
        self.q = list()
        self.qdot = list()
        self.zmp = list()
        self.freq=freq
    def record(self):
        i=self.robot.state.time
        if i%self.freq == 0:
            self.q.append(robot.state.value)
            self.qdot.append(robot.state.value)
            self.zmp.append(list(vectorToTuple(matrix(dyn.waist.value).I*matrix(zmp.zmp.value+(1,)).T)))
    def restore(self,t):
        if not t in self.q.keys():
            print "Time ",t," has not been stored (freq is ",self.freq,")."
            return
        print "robot.set(",self.q[t],")"
        print "robot.setVelocity(",self.qdot[t],")"
        print "robot.state.time = ",t
    def dumpToOpenHRP(self,baseName = "dyninv",sample = 1):
        filePos = open(baseName+'.pos','w')
        fileRPY = open(baseName+'.hip','w')
        fileZMP = open(baseName+'.zmp','w')
        sampleT = 0.005
        for nT,(q,z) in enumerate(zip(self.q,self.zmp)):
            fileZMP.write(str(sampleT*nT)+' '+str(z[0])+' '+str(z[1])+' '+str(z[2])+'\n')
            fileRPY.write(str(sampleT*nT)+' '+str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n')
            filePos.write(str(sampleT*nT)+' ')
            for j in range(6,36):
                filePos.write(str(q[j])+' ')
            filePos.write(10*' 0'+'\n')
