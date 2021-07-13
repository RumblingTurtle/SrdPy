class ConstrainedObserver:
    def __init__(self,CLQE_Handler,observerStateHandler,measuredOutput,mainController,
        inverseDynamicsHandler,controlInputStateSpaceHandler,timeHandler,tol):
        self.CLQE_Handler = CLQE_Handler
        self.observerStateHandler = observerStateHandler
        self.measuredOutput = measuredOutput
        self.mainController = mainController
        self.inverseDynamicsHandler = inverseDynamicsHandler
        self.controlInputStateSpaceHandler = controlInputStateSpaceHandler
        self.timeHandler = timeHandler
        self.tol = tol

    def update(self):
        E = self.CLQE_Handler.solution.observer.map
            
        u = self.mainController.u
        y = self.measuredOutput.y
        chi = E.T @ self.observerStateHandler.x
        x0 = self.observerStateHandler.x - E@chi
        
        dt = self.timeHandler.dt
        t = self.timeHandler.currentTime  
        desired = self.controlInputStateSpaceHandler.getX_dx(t)
        desired_x =  desired[0]
        desired_u = self.inverseDynamicsHandler.u 
        

        dchi = self.CLQE_Handler.solution.observer.matrix_chi @ (chi - E.T@desired_x) 
        + self.CLQE_Handler.solution.observer.matrix_u @ (u- desired_u) 
        + self.CLQE_Handler.solution.observer.matrix_y @ (y- self.measuredOutput.C@desired_x) 
        + self.CLQE_Handler.solution.observer.vector
        
        chi = chi + dchi@dt
        
        x = x0 + E@chi
        
        self.observerStateHandler.x = x