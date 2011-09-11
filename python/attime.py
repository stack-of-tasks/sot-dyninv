class Calendar:
    def __init__(self):
        self.events=dict()
        self.ping = list()
    def __repr__(self):
        res=''
#        for iter in sort(self.events.keys()):
        for iter,funpairs in sorted( self.events.iteritems() ):
            res += str(iter)+": \n"
#            funpairs=self.events[iter]
            for funpair in funpairs:
                if funpair[1]=='': res+=funpair[0]+'\n'
                else: res += str(funpair[1])+'\n'
        return res

    def registerEvent( self,iter,pairfundoc ):
        if not iter in self.events.keys(): self.events[iter] = list()
        self.events[iter].append(pairfundoc)

    def registerEvents( self,iter,*funs ):
        '''
        3 entry types are possible: 1. only the functor. 2. a pair
        (functor,doc). 3. a list of pairs (functor,doc).
        '''
        if len(funs)==2 and callable(funs[0]) and isinstance( funs[1],str ): self.registerEvent(iter, ( funs[0],funs[1] ) )
        else:
            for fun in funs:
                if isinstance( fun,tuple ):
                    self.registerEvent(iter,fun)
                else: # assert iscallable(fun)
                    if 'functor' in fun.__dict__:
                        self.registerEvent(iter, (fun.functor,fun.functor.__doc__) )
                    else:
                        self.registerEvent(iter, (fun,fun.__doc__) )
    def addPing(self,f): self.ping.append(f)
    def callPing(self):
        for f in self.ping: f()
    def run(self,iter,*args):
        if iter in self.events.keys():
            self.callPing()
            for fun,doc in self.events[iter]:
                intro = "At time "+str(iter)+": "
                if doc!=None:               print intro, doc
                else:
                    if fun.__doc__ != None: print intro, fun.__doc__
                    else:                   print intro,"Runing ", fun
                fun(*args)

    def __call__(self,iterarg,*funs):
        if len(funs)==0: return self.generatorDecorator(iterarg)
        else: self.registerEvents(iterarg,*funs)

    def generatorDecorator(self,iterarg):
        """
        This next calling pattern is a little bit strange. Use it to decorate
        a function definition: @attime(30) def run30(): ...
        """
        class calendarDeco:
            iterRef=iterarg
            calendarRef=self
            fun=None
            def __init__(selfdeco,functer):
                if functer.__doc__!=None:
                    if  len(functer.__doc__)>0:
                        selfdeco.__doc__  = functer.__doc__
                        selfdeco.__doc__ += " (will be run at time "+str(selfdeco.iterRef)+")"
                selfdeco.fun=functer
                selfdeco.calendarRef.registerEvents(selfdeco.iterRef,functer,functer.__doc__)
            def __call__(selfdeco,*args):
                selfdeco.fun(*args)
        return calendarDeco

    def fastForward(self,t):
        for i in range(t): self.run(i)

attime=Calendar()
