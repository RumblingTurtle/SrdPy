import numpy as np
import matplotlib.pyplot as plt
def plotIKTable(IKModelHandler,timeTable,IKTable,tol=10**(-5)):
    count = len(timeTable)

    condition_number_tape = np.zeros((count, 1))
    rank_tape = np.zeros((count, 1))

    for i in range(count):

        q = IKTable[i]
        J = IKModelHandler.getJacobian(q)

        condition_number_tape[i] = np.linalg.cond(J)
        rank_tape[i] = np.linalg.matrix_rank(J, tol)

    fig, ax = plt.subplots(1, 1)
    ax.plot(timeTable,IKTable[:,0],'r',label=r'$q_1$')
    ax.plot(timeTable,IKTable[:,1],'b',label=r'$q_2$')
    ax.plot(timeTable,IKTable[:,2],'g--',label=r'$q_3$')
    ax.legend(loc='upper right')

    ax.set_xlabel(r'$t, s$', fontsize=15)
    ax.set_ylabel(r'$q_i$', fontsize=15)
    ax.set_title(r'IK Solution', fontsize=18)

    ax.grid(color='k', linestyle='-', linewidth=0.15)
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