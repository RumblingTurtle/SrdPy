import matplotlib.pyplot as plt
import numpy as np
def plotGeneric(x,y,figureTitle="title",ylabel="y",plot=False, old_ax = None):
    line_styles=["-","-.","--",":"]
    colors = ["r","g","b","c","m","k","y"]
    y = np.array(y)

    def getStyle(i):
        colIdx = i%len(colors)
        lineStyleIdx = i%len(line_styles)

        return colors[colIdx]+line_styles[lineStyleIdx]
    def getRandomStyle():
        return colors[np.random.randint(len(colors))]+line_styles[np.random.randint(len(line_styles))]
    
    if old_ax==None:
        fig, ax = plt.subplots(1, 1)

        ax.set_xlabel(r'$t, s$', fontsize=15)
        ax.set_ylabel(ylabel, fontsize=15)
        ax.set_title(figureTitle, fontsize=18)

        for i in range(y.shape[1]):
            ax.plot(x, y[:,i].T, getRandomStyle(), label=r'$'+ylabel+r'_'+str(i)+r'$')

        ax.grid(color='k', linestyle='-', linewidth=0.15)
        ax.legend(loc='upper right')
    else:
        for i in range(y.shape[1]):
            old_ax.plot(x, y[:,i].T, getRandomStyle(), label=r'$'+ylabel+r'_'+str(i)+r'$')
        ax = old_ax
        ax.legend(loc='upper right')

    if plot:
        plt.show()

    return ax