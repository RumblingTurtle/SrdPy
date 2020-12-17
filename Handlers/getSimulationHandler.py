class SimulationHandler():
    
    def __init__(self,timeLog,currentTime):
        self.timeLog = timeLog
        self.currentTime = currentTime
        self.currentIndex = None
        self.preprocessingHandlersArray = []
        self.controllerArray = []
        self.solverArray = []
        self.loggerArray = []

    def simulate(self):
        for i in range(len(self.timeLog)-1):
            self.currentTime = self.timeLog[i]
            self.currentIndex = i

            for preprocessor in self.preprocessingHandlersArray:
                preprocessor.update()

            for controller in self.controllerArray:
                controller.update()

            for solver in self.solverArray:
                solver.update()

            for logger in self.loggerArray:
                logger.update()

def getSimulationHandler(timeLog):
    return SimulationHandler(timeLog,timeLog[0])