class SimulationTickDisplay:
    def __init__(self,timeHandler,customMessage1="Current step: ",customMessage2=" out of ",displayFreq=1):
        self.timeHandler = timeHandler
        self.customMessage1 = customMessage1
        self.customMessage2 = customMessage2
        self.displayFreq = displayFreq

    def update(self):
        if self.timeHandler.currentIndex%self.displayFreq==0:
            print(self.customMessage1+str(self.timeHandler.currentIndex)+self.customMessage2+str(len(self.timeHandler.timeLog)))
        