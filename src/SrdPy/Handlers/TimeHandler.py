class TimeHandler:
    def __init__(self,timeLog=[],continous=False, timeStep = 1./500.):
        self.currentTime = 0
        self.currentIndex = 0
        self.timeLog = timeLog
        self.continous = continous
        self.timeStep = timeStep
        if self.continous:
            self.dt = timeStep
        else:
            self.dt = timeLog[1]-timeLog[0]
    
    def update(self):
        if self.continous:
            self.currentTime = self.currentTime+self.timeStep
        else:   
            self.currentIndex = min(self.timeLog.shape[0]-2,self.currentIndex+1)
            self.currentTime = self.timeLog[self.currentIndex]
            if self.currentIndex<len(self.timeLog):
                self.dt = self.timeLog[self.currentIndex+1]-self.timeLog[self.currentIndex]
