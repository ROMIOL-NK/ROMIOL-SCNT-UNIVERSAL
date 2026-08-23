import matplotlib.pyplot as plt


def drawTraj(t, pos, ref):
    plt.plot(t, pos)
    plt.plot(t, ref)
    plt.xlabel('Time (s)',fontdict={'family' : 'Times New Roman', 'size'   : 16})
    plt.ylabel('Position',fontdict={'family' : 'Times New Roman', 'size'   : 16})
    plt.xticks(fontproperties = 'Times New Roman', size = 14)
    plt.yticks(fontproperties = 'Times New Roman', size = 14)
    plt.show()


def drawU(t, u):
    plt.plot(t, u)
    plt.xlabel('Time (s)',fontdict={'family' : 'Times New Roman', 'size'   : 16})
    plt.ylabel('Input',fontdict={'family' : 'Times New Roman', 'size'   : 16})
    plt.xticks(fontproperties = 'Times New Roman', size = 14)
    plt.yticks(fontproperties = 'Times New Roman', size = 14)
    plt.show()

