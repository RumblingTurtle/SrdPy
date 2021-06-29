class TimeHandler:
    def __init__(self,timeLog=[],continous=False, timeStep = 1./500.):
        self.currentTime = 0
        self.currentIndex = 0
        self.timeLog = timeLog
        self.continous = continous
    
    def update(self):
        if self.continous:
            self.currentTime = self.currentTime+self.timeStep
        else:   
            self.currentIndex = min(self.timeLog.shape[0]-2,self.currentIndex+1)
            self.currentTime = self.timeLog[self.currentIndex]
