class ProgressDisplayHandler():
    def __init__(self,timeHandler):
        self.timeHandler = timeHandler

    def update(self):
        currentTick = self.timeHandler.currentIndex
        totalTicks = self.timeHandler.timeLog.shape[0]
        print("Simulated {} out of {} steps".format(currentTick,totalTicks))