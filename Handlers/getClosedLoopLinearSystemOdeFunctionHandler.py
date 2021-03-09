import numpy as np

class ClosedLoopLinearSystemOdeFunctionHandler:
    def __init__(self,AA_table, cc_table, time_table):
        self.AA_table = AA_table
        self.cc_table = cc_table
        self.time_table = time_table

    def eval(self,t,x):
        closest_index = np.argwhere(np.array(self.time_table)<=t)[-1]
        AA = self.AA_table[closest_index]
        cc = self.cc_table[closest_index]
        
        return AA@x + cc


def getClosedLoopLinearSystemOdeFunctionHandler(AA_table, cc_table, time_table):
    return ClosedLoopLinearSystemOdeFunctionHandler(AA_table, cc_table, time_table)