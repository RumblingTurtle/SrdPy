from SrdPy.Chain import Chain
from SrdPy.Spline import Spline
from SrdPy.SplineConstructor import SplineConstructor
from SrdPy.SymbolicEngine import SymbolicEngine
from SrdPy.plotGeneric import plotGeneric
from SrdPy.Profiling import *
import pickle

def save(variable,name):
    pickle.dump(variable,open(name+".p", "wb" ))

def get(name):
    return pickle.load(open(name+".p", "rb" ))