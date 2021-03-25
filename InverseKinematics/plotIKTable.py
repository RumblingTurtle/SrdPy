import numpy as np
import matplotlib.pyplot as plt
from SrdPy import plotGeneric
def plotIKTable(IKModelHandler,timeTable,IKTable,tol=10**(-5)):
    count = len(timeTable)

    condition_number_tape = np.zeros((count, 1))
    rank_tape = np.zeros((count, 1))

    for i in range(count):

        q = IKTable[i]
        J = IKModelHandler.getJacobian(q)

        condition_number_tape[i] = np.linalg.cond(J)
        rank_tape[i] = np.linalg.matrix_rank(J, tol)

    ax = plotGeneric(timeTable,IKTable,figureTitle='IK Solution',ylabel="q")
    
    fig, ax = plt.subplots(2)
    ax[0].plot(timeTable,condition_number_tape,'r',label=r'$cond_1$')
    ax[0].legend(loc='upper right')
    ax[0].set_xlabel(r'$t, s$', fontsize=15)
    ax[0].set_ylabel(r'$cond_i$', fontsize=15)
    ax[0].set_title(r'condition number J', fontsize=18)
    ax[0].grid(color='k', linestyle='-', linewidth=0.15)

    ax[1].plot(timeTable,rank_tape,'r',label=r'$cond_1$')
    ax[1].legend(loc='upper right')
    ax[1].set_xlabel(r'$t, s$', fontsize=15)
    ax[1].set_ylabel(r'$rank_i$', fontsize=15)
    ax[1].set_title(r'rank J', fontsize=18)
    ax[1].grid(color='k', linestyle='-', linewidth=0.15)

    plt.show(block=True)