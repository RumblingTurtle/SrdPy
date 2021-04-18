from casadi import *
import numpy as np
from numpy.core.numeric import indices

#Simple integer list comparison with preserved order
def compare_lists(list1,list2):
    return ''.join(map(str, list1)) == ''.join(map(str, list2))

#Python list string to integer list conversion
def csv_to_list(value):
    values = value.replace('[','').replace(']','').split(",")
    if len(values)==1:
        return int(values[0])
    else:
        return [int(val) for val in values]

#Substitutes @symbols in SX matrix for printing
def str_sx(matrix):
    import re
    matrix_str = str(matrix)
    p = re.split('\[',matrix_str,1)[0]
    out = '['+re.split('\[',matrix_str,1)[1]
    subs = p.split(',')
    for sub in subs[::-1]:
        if len(sub.split('='))==2:
            at, val = sub.split('=')
            out = out.replace(at.strip(),val.strip())
    return out

class SparseMatrix():

    #Overrides numpy's @ operator
    __array_priority__ = 10000

    def __init__(self,values=None,indices=None,shape=None,derivative_idx=None):

        if values==None:
            self.values = []
        else:
            self.values = values

        if indices==None:
            self.indices = []
        else:
            self.indices = indices

        if shape == None:
            self.shape = []
        else:
            self.shape = shape
        
        if derivative_idx==None:
            self.derivative_idx = []
        else:
            self.derivative_idx = derivative_idx
    
    def __str__(self):
        output =  "Sparse tensor of order shape "+str(self.shape)+"\n"
        output = output+ "Values:\n"
        output = output+ str('\n'.join([str_sx(val)+'\n' for val in self.values]))+"\n"
        output = output+ "Indices:\n"
        output = output+ str(self.indices)+"\n"
        return output

    #Multiplies sparse tensor by matrix
    def mulTensorXMat(self,matrix,mulLeft=False):
            result = SparseMatrix()
            result.shape = self.shape.copy()
            result.indices = self.indices.copy()
            result.derivative_idx = self.derivative_idx.copy()

            if not mulLeft:
                result.values = [simplify(value@matrix) for value in self.values]
                result.shape[1] = matrix.shape[1]
            else:
                result.values = [simplify(matrix@value) for value in self.values]
                result.shape[0] = matrix.shape[0]

            return result

    # NxA tensor by AxM tensor multiplication
    def mulTensorXTensorNxM(self,tensor,mulLeft=False):
        result = SparseMatrix()
        result.shape = self.shape[:2].copy()

        if not mulLeft:
            result.shape[1] = tensor.shape[1]
        else:
            result.shape[0] = tensor.shape[0]
        
        ordered_idxs = np.argsort(self.derivative_idx+tensor.derivative_idx)

        shapeTail = np.array(self.shape[2:]+tensor.shape[2:])[ordered_idxs].tolist()

        result.shape+=shapeTail

        avalues = self.values
        bvalues = tensor.values

        aidxs = self.indices
        bidxs = tensor.indices
        
        for aidx,aval in zip(aidxs,avalues):
            if len(self.shape)==3:
                aidx = [aidx]
            for bidx,bval in zip(bidxs,bvalues):

                if len(tensor.shape)==3:
                    bidx = [bidx]

                indices = np.array(aidx+bidx)[ordered_idxs].tolist()

                result.indices.append(indices)

                if not mulLeft:  
                    value = aval@bval
                else:
                    value = bval@aval
                result.values.append(simplify(value))
        return result

    #Nx1x.... by Nx1x... tensor multiplication
    def mulTensorXTensorNxN(self,tensor,mulLeft=False):
        if len(self.shape)==3 and len(tensor.shape)==3:
            result = SX.zeros(tensor.shape[2],tensor.shape[2])
        else:
            result = SX.zeros(np.prod(self.shape[2:]+tensor.shape[2:]))

        avalues = self.values
        bvalues = tensor.values

        aidxs = self.indices
        bidxs = tensor.indices
        
        ordered_idxs = np.argsort(self.derivative_idx+tensor.derivative_idx)

        shape = self.shape[2:]+tensor.shape[2:]
        for aidx,aval in zip(aidxs,avalues):
            if len(self.shape)==3:
                aidx = [aidx]
            for bidx,bval in zip(bidxs,bvalues):
                if len(tensor.shape)==3:
                    bidx = [bidx]

                
                if len(self.shape)>3 or len(tensor.shape)>3:
                    index = np.array(aidx+bidx)
                    
                    if len(self.shape)==len(tensor.shape) and len(self.shape)==4:
                        index = np.array(aidx+bidx)[ordered_idxs].tolist()
                    #indexing magic for proper reshape result
                    index = index[::-1]
                    if len(index)==3:
                        index[1],index[2] = index[2],index[1] 
                    if len(index)==4:
                        index[2],index[3] = index[3],index[2] 
                        index[2],index[1] = index[1],index[2] 
                    
                    idx = 0
                    for i in range(len(index)):
                        shape_val = shape[i]**i
                        idx = idx+shape_val*index[i]
                    index = idx
                else:
                    index = tuple(aidx+bidx)

                if not mulLeft:  
                    result[index]=simplify(aval.T@bval)
                else:
                    result[index]=simplify(bval.T@aval)
        return result

    #Tensor addition operator
    def __add__(self,tensor):
        if not compare_lists(self.shape,tensor.shape):
            raise RuntimeError("Passed array is not of the right shape: lhs is "+str(self.shape)+" rhs is " +str(tensor.shape))

        result = SparseMatrix()
        result.shape = self.shape.copy()
        result.derivative_idx = self.derivative_idx.copy()
        
        sum_dict = {}

        values = self.values+tensor.values
        indices = self.indices+tensor.indices

        for val,idx in zip(values,indices):
            if len(self.shape)==3:
                idx = [idx]

            str_idx = ','.join(map(str, idx))
            if str_idx in sum_dict.keys():
                sum_dict[str_idx] = sum_dict[str_idx]+val
            else:
                sum_dict[str_idx] = val
        
        result.values = list(sum_dict.values())
        result.indices = list(map(csv_to_list,list(sum_dict.keys())))

        return result

    #Right side multiplication
    def __matmul__(self,tensor):
        if type(tensor)==list:
            tensor = np.array(tensor)

        if tensor.shape[0]==self.shape[0] and self.shape[1]==1 and tensor.shape[1]==1:
            return self.mulTensorXTensorNxN(tensor)

        if tensor.shape[0]!=self.shape[1]:
            raise RuntimeError("Passed array is not of the right shape: lhs is "+str(self.shape)+" rhs is " +str(tensor.shape))

        if len(tensor.shape)==2:
            return self.mulTensorXMat(tensor)
            
        return self.mulTensorXTensorNxM(tensor)

    #Left side multiplication
    def __rmatmul__(self,tensor):
        if type(tensor)==list:
            tensor = np.array(tensor)

        if tensor.shape[0]==self.shape[0] and self.shape[1]==1 and tensor.shape[1]==1:
            return self.mulTensorXTensorNxN(tensor,mulLeft=True)

        if tensor.shape[1]!=self.shape[0]:
            raise RuntimeError("Passed array is not of the right shape: lhs is "+str(self.shape)+" rhs is " +str(tensor.shape))

        if len(tensor.shape)==2:
            return self.mulTensorXMat(tensor,mulLeft=True)


        return self.mulTensorXTensorNxM(tensor,mulLeft=True)
