from mpl_toolkits.mplot3d import Axes3D  
import matplotlib.pyplot as plt
import math
import numpy as np
import copy


qDeg = np.array([0,30,-70,45,0,0])
qRad = math.pi*qDeg/180.0



#a[i] represents the frame matrix between i and i-1
#d1 = 10
#a2 = 10 
#d4 = 10
a = []
#a0 base frame
a.append(np.array(
    [[1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1]]))
#a01
a.append(np.array(
[[math.cos(qRad[0]),0,math.sin(qRad[0]),0],
[math.sin(qRad[0]),0,-math.cos(qRad[0]),0],
[0,1,0,5],
[0,0,0,1]]))

#a12
a.append(np.array(
[[math.cos(qRad[1]),-math.sin(qRad[1]),0,5*math.cos(qRad[1])],
[math.sin(qRad[1]),math.cos(qRad[1]),0,5*math.sin(qRad[1])],
[0,0,1,0],
[0,0,0,1]]))


#a23
a.append(np.array(
[[math.sin(qRad[2]),0,math.cos(qRad[2]),-1*math.sin(qRad[2])],
[-math.cos(qRad[2]),0,math.sin(qRad[2]),1*math.cos(qRad[2])],
[0,-1,0,0],
[0,0,0,1]]))



#a34
a.append(np.array(
[[math.cos(qRad[3]),0,math.sin(qRad[3]),0],
[math.sin(qRad[3]),0,-math.cos(qRad[3]),0],
[0,1,0,5],
[0,0,0,1]]))

#a45
a.append(np.array(
[[math.cos(qRad[4]),0,-math.sin(qRad[4]),0],
[math.sin(qRad[4]),0,math.cos(qRad[4]),0],
[0,-1,0,0],
[0,0,0,1]]))

#a56
a.append(np.array(
[[math.cos(qRad[5]),-math.sin(qRad[5]),0,0],
[math.sin(qRad[5]),math.cos(qRad[5]),0,0],
[0,0,1,2],
[0,0,0,1]]))



#implement inverse kinematics and return several plausible configurations
def inverseKinematics():
    pass

#for adding arrows to each frame x,y,z red, green, blue / yellow magenta cyan
#input: ax is frame for rendering, a is of current result of multiplication, and num is number of terms

def addFrame(ax, a, num):
    colors = []
    #even a[]
    if num%2 == 0:
        colors = ['r','g','b']
        print("odd axis")
    #odd a[]
    else:
        colors = ['y','m','c']
        print("even axis")
    #origin to each frame draw three rgb/ymc
    ax.quiver(a[0][3],a[1][3],a[2][3],a[0][0],a[1][0],a[2][0], color=colors[0], length = 3)
    ax.quiver(a[0][3],a[1][3],a[2][3],a[0][1],a[1][1],a[2][1], color=colors[1], length = 3)
    ax.quiver(a[0][3],a[1][3],a[2][3],a[0][2],a[1][2],a[2][2], color=colors[2], length = 3)


def drawBody(ax,a1,a2):
    absSum = abs(a1[0][3]-a2[0][3])+abs(a1[1][3]-a2[1][3])+abs(a1[2][3]-a2[2][3])
    if absSum>1:
        ax.quiver(a1[0][3],a1[1][3],a1[2][3],a2[0][3]-a1[0][3],a2[1][3]-a1[1][3],a2[2][3]-a1[2][3], color='orange')
        print(a1)
        print(a2)


#loop through A until
def forwardKinematics(ax):
    aCul = np.array(
    [[1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1]])

    aPrev = np.array(
    [[1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1]])
   
    addFrame(ax,a[0],0)
    for index in range(1,len(a)):
        aPrev = copy.deepcopy(aCul)
        aCul = np.dot(aCul,a[index])
       # addFrame(ax,aCul,index)
        drawBody(ax,aPrev,aCul)
    addFrame(ax, aCul, 1)

       


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_aspect('equal')
plt.ylim(-10,10)
plt.xlim(-10,10)
ax.set_zlim(0,20)


forwardKinematics(ax)

plt.show()


#TODO write function to convert Deg to Rad




