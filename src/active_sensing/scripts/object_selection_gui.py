import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree


class ObjectSelectionGUI:
  
  def __init__(self):
    pass


  def onClick(self, event):  
    self.x = event.xdata
    self.y = event.ydata
    tree = cKDTree(self.cloud[:,0:2])
    ballCloudIndices = tree.query_ball_point([self.x, self.y], 0.03)
    self.z = np.max(self.cloud[ballCloudIndices,2])
    print "x:", self.x, "y:", self.y, "z:", self.z
    plt.close()

  
  def requestPointInCloud(self, cloud, cloudColors):
    self.cloud = cloud
    fig, ax = plt.subplots(1,1)
    x = cloud[:,0]; y = cloud[:,1]; z = cloud[:,2]
    
    ax.scatter(x,y,c=cloudColors,s=5,edgecolor='none')
    plt.axis("equal")
    #plt.xlim(np.min(x), np.max(x)) # doesn't work
    plt.ylim(np.min(y), np.max(y))
    
    mng = plt.get_current_fig_manager()
    #mng.window.showMaximized()
    mng.window.raise_()
    fig.canvas.mpl_connect('button_press_event', self.onClick)
    plt.show()
    
    return self.x, self.y, self.z
