import numpy as np

class ClosedLoopLinearSystemOdeFunctionHandler:
    def __init__(self,AA_table, cc_table, time_table):
        self.AA_table = AA_table
        self.cc_table = cc_table
        self.time_table = time_table

    def __call__(self,t,x):
        closest_index = np.argwhere(np.array(self.time_table)<=t)[-1]
        if closest_index>=self.AA_table.shape[0]:
            return np.zeros(self.cc_table.shape[1])

        AA = self.AA_table[closest_index]
        cc = self.cc_table[closest_index]
        
        result = np.squeeze(AA)@np.squeeze(x) + cc
        return result