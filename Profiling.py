from functools import wraps
from time import time
from cProfile import Profile
import matplotlib.pyplot as plt
import numpy as np

_srd_profiler_dict = {}

class Profiler:
    def __init__(self):
        pass

    def getReport(self):
        global _srd_profiler_dict
        fig, ax = plt.subplots()

        func_names = _srd_profiler_dict.keys()
        values = _srd_profiler_dict.values()

        y_pos = np.arange(len(func_names))

        ax.barh(y_pos, values,0.1, align='center')
        ax.set_yticks(y_pos)
        ax.set_yticklabels(func_names)
        ax.invert_yaxis() 
        ax.set_xlabel('Time s')
        ax.set_title('Profiler report')
        plt.show()
        return _srd_profiler_dict

def profile(func):
    @wraps(func)
    def wrap(*args, **kw):
        ts = time()
        result = func(*args, **kw)
        te = time()
        name = func.__name__
        global _srd_profiler_dict
        if name in _srd_profiler_dict.keys():
            _srd_profiler_dict[name] = _srd_profiler_dict[name]+te-ts
        else:
            _srd_profiler_dict[name] = te-ts
        return result
    return wrap

def timer(func,showArgs=False):
    @wraps(func)
    def wrap(*args, **kw):
        pr = Profile()
        pr.enable()
        result = func(*args, **kw)
        pr.disable()
        pr.print_stats()
        return result
    return wrap