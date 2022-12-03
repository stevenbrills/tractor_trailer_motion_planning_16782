import numpy as np
import matplotlib.pyplot as plt
from icecream import ic
from collision_check import collision_check
from matplotlib import animation
from matplotlib.animation import FuncAnimation, PillowWriter

with open('src/TestForwardTrajectory.txt', 'r') as f:
    data = f.readlines()
    #save every 100th line into an numpy array
    data = np.array([line.split() for line in data[::100]], dtype=np.float)

cc=collision_check()

fig = plt.figure(figsize=(9,9))
ax2 = plt.axes(xlim=(-10, 10), ylim=(-10, 10))
rect1,=ax2.plot([],[], 'rs')
rect2,=ax2.plot([], [], 'bs')

def init():
    rect1.set_data([], [])
    rect2.set_data([], [])
    return rect1, rect2

def animate(i):
    ic(i)
    cc.computeTransformMatrices(data[i,0],data[i,1],data[i,2],data[i,3])
    a , b , c ,d =cc.computeCoords()
    rect1.set_data(a, b)
    rect2.set_data(c, d)
    return rect1, rect2

anim = animation.FuncAnimation(fig, animate, init_func=init, blit=True, interval=100, frames=len(data), repeat=False)

writergif = animation.PillowWriter(fps=30)
anim.save('filename.gif',writer=writergif)

    
