class Calendar:
    events=dict()
    def registerEvent( self,iter,fun,doc ):
        if not iter in self.events.keys(): self.events[iter] = list()
        self.events[iter] += [ (fun,doc) ]
    def run(self,iter,*args):
        if iter in self.events.keys():
            for fun,doc in self.events[iter]:
                intro = "At time "+str(iter)+": "
                if doc!=None:               print intro, doc
                else:
                    if fun.__doc__ != None: print intro, fun.__doc__
                    else:                   print intro,"Runing ", fun
                fun(*args)

    def __call__(self,iterarg,functer=None,doc=None):
        if functer==None: return self.generatorDecorator(iterarg)
        else:
            self.registerEvent(iterarg,functer,doc)

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
                selfdeco.calendarRef.registerEvent(selfdeco.iterRef,functer,functer.__doc__)
            def __call__(selfdeco,*args):
                selfdeco.fun(*args)
        return calendarDeco

attime=Calendar()
