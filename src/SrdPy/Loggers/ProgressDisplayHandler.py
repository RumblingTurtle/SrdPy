class ProgressDisplayHandler():
    def __init__(self,simulationHandler):
        self.simulationHandler = simulationHandler

    def update(self):
        currentTick = self.simulationHandler.currentIndex
        totalTicks = self.simulationHandler.timeLog.shape[0]
        print("Simulated {} out of {} steps".format(currentTick,totalTicks))