import numpy as np
from casadi import *

class OdeFunctionHandler:
    def __init__(self, gcModelHandler, linearizedModelHandler, K_table, x_table, u_table, time_table):
        self.gcModelHandler = gcModelHandler
        self.linearizedModelHandler = linearizedModelHandler
        self.K_table = K_table
        self.x_table = x_table
        self.u_table = u_table
        self.time_table = time_table
    
    def eval(self,t,x):
        closest_index = np.argwhere(np.array(self.time_table)<=t)[-1]
        n = self.gcModelHandler.dofConfigurationSpaceRobot

        K = self.K_tablep[closest_index]
        x_desired = self.x_table[closest_index]
        u_desired = self.u_table[closest_index]
        
        u = -K@(x - x_desired) + u_desired
        
        q = x[:n]
        v = x[n:2*n]
        
        q_desired = x_desired[:n]
        v_desired = x_desired[n:2*n]
        
        iH = self.gcModelHandler.getJointSpaceInertiaMatrixInverse(q)
        
        iH_desired = self.gcModelHandler.getJointSpaceInertiaMatrixInverse(q_desired)
        c_desired = self.gcModelHandler.getBiasVector(q_desired, v_desired)
        T_desired = self.gcModelHandler.getControlMap(q_desired)
        a_desired = iH_desired@(T_desired@(u_desired) - c_desired)
        
        A =  self.linearizedModelHandler.getA(q, v, u, iH)
        B =  self.linearizedModelHandler.getB(q, u,    iH)
        
        f0 = vertcat(v_desired,a_desired)
        dx = f0 + A@(x - x_desired) + B@(u - u_desired)
        return dx

def getLinearizedModelOdeFunctionHandler(gcModelHandler, linearizedModelHandler, K_table, x_table, u_table, time_table):
    return OdeFunctionHandler(gcModelHandler, linearizedModelHandler, K_table, x_table, u_table, time_table)