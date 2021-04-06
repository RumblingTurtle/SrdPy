import numpy as np

class ClosedLoopConstrainedLinearSystemOdeFunctionHandler:
    def __init__(self,AA_table, cc_table,G_table,F_table, time_table):
        self.AA_table = AA_table
        self.cc_table = cc_table
        self.G_table = G_table
        self.F_table = F_table
        self.time_table = time_table

        self.n = self.AA_table.shape[1]
        self.k = self.F_table.shape[2]

    
    def __call__(self,t,x):
        closest_index = np.argmin(np.abs(self.time_table - t))

        AA = self.AA_table[closest_index]
        cc = self.cc_table[closest_index]
        G  = self.G_table[closest_index]
        F  = self.F_table[closest_index]

        M = np.vstack(
             (
             np.hstack((np.eye(self.n), -F)),
             np.hstack(( G,       np.zeros((2*self.k, self.k)) ))
             )
            )

        var = np.linalg.pinv(M) @ np.hstack( ((AA@x + cc), np.zeros(2*self.k) ) )

        return var[:self.n]

