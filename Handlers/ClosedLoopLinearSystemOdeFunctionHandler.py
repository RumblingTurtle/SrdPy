import numpy as np

def func(time_table,AA_table,cc_table,t,x):
    closest_index = np.argmin(np.abs(time_table - t))
    return AA_table[closest_index]@x + cc_table[closest_index]

class ClosedLoopLinearSystemOdeFunctionHandler:
    def __init__(self,AA_table, cc_table, time_table):
        self.AA_table = AA_table
        self.cc_table = cc_table
        self.time_table = time_table
    def __call__(self,t,x):
        return func(self.time_table,self.AA_table,self.cc_table,t,x)

