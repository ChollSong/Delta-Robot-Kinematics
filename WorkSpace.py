from mpl_toolkits.mplot3d import Axes3D  
import matplotlib.pyplot as plt
import math
import numpy as np


#frame relative to base


sb = 500
sp = 110
#lower arm
lowerArm = 300
#upper arm
upperArm = 150
#derived from sb and sp
wb = math.sqrt(3)*sb/6.0
ub = math.sqrt(3)*sb/3.0

wp = math.sqrt(3)*sp/6.0
up = math.sqrt(3)*sp/3.0

#generate combo -90 to 90
q1 = np.linspace(-30, 85, 15)
q2 = np.linspace(-30, 85, 15)
q3 = np.linspace(-30, 85, 15)



#find point of line after cross product
def findPointofLine(varMatrix):
    normalVector = np.cross(varMatrix[0][0:3],varMatrix[1][0:3])
    detMatrix = [varMatrix[0][0:3],varMatrix[1][0:3], normalVector]
    xMatrix = [[varMatrix[0][3],varMatrix[0][1],varMatrix[0][2]], [varMatrix[1][3],varMatrix[1][1],varMatrix[1][2]], [0,normalVector[1],normalVector[2]]]
    yMatrix = [[varMatrix[0][0],varMatrix[0][3],varMatrix[0][2]], [varMatrix[1][0],varMatrix[1][3],varMatrix[1][2]], [normalVector[0],0 ,normalVector[2]]]
    zMatrix = [[varMatrix[0][0],varMatrix[0][1],varMatrix[0][3]], [varMatrix[1][0],varMatrix[1][1],varMatrix[1][3]], [normalVector[0],normalVector[1],0]]
    det = np.linalg.det(detMatrix)
    #check if determinant is zero aka no sol
    if (abs(det) < 0.0001):
        print("zero det")
        return None
    detX = np.linalg.det(xMatrix)
    detY = np.linalg.det(yMatrix)
    detZ = np.linalg.det(zMatrix)
    #no solution
    return [detX/det, detY/det, detZ/det]

#return theta1, theta2, theta3
#p1 p2 p3 are xyz of point
#for function to use in inverse Kinematics
def getVariableMatricesIK(pX, pY, pZ):
    cA = [0, -wb+up, 0]
    cB = [0.5*(math.sqrt(3)*wb - sp), 0.5*wb - wp, 0]
    cC = [-0.5*(math.sqrt(3)*wb - sp), 0.5*wb - wp, 0]
    centers = [cA,cB,cC]

    varMatrix1 = [[0.0,0.0,0.0,0.0],[1.0,0.0,0.0,0.0]]
    varMatrix2 = [[0.0,0.0,0.0,0.0],[1.0,-math.sqrt(3),0.0,0.0]]
    varMatrix3 = [[0.0,0.0,0.0,0.0],[1.0,math.sqrt(3),0.0,0.0]]
    varMatrices = [varMatrix1, varMatrix2, varMatrix3]
    for i in range(3):
        varMatrices[i][0][0] = 2*(pX-centers[i][0])
        varMatrices[i][0][1] = 2*(pY-centers[i][1])
        varMatrices[i][0][2] = 2*(pZ-centers[i][2])
        varMatrices[i][0][3] = upperArm*upperArm - lowerArm*lowerArm - centers[i][0]*centers[i][0] - centers[i][1]*centers[i][1] - centers[i][2]*centers[i][2] + pX*pX + pY*pY + pZ * pZ
    
    
    return varMatrices

'''
    problem at -90 degree and 90 degree on deciding which result to use. But in actual model would not
    be concerned at that point since actual angle range lesser than from 90 degrees to - 90 degrees
'''
def inverseKinematic(pX,pY,pZ):
    varMatrices = getVariableMatricesIK(pX, pY, pZ)
    normalVectors = [np.cross(varMatrices[0][0][0:3],varMatrices[0][1][0:3]), np.cross(varMatrices[1][0][0:3],varMatrices[1][1][0:3]), np.cross(varMatrices[2][0][0:3],varMatrices[2][1][0:3])]
    resultCenters = []

    for k in range(3):
        if abs(normalVectors[k][0]) < 0.0001 and abs(normalVectors[k][1]) < 0.0001 and abs(normalVectors[k][2]) < 0.0001:
            print("cannot find forward kinematics")
            return None
        
        
        mPoint = findPointofLine(varMatrices[k])
        if(mPoint == None):
            return None
        nPoint = np.subtract(mPoint, [pX,pY,pZ]).tolist()
        a = normalVectors[k][0]*normalVectors[k][0]+normalVectors[k][1]*normalVectors[k][1]+normalVectors[k][2]*normalVectors[k][2]
        b = 2*(normalVectors[k][0]*nPoint[0]+normalVectors[k][1]*nPoint[1]+normalVectors[k][2]*nPoint[2])
        c = nPoint[0]*nPoint[0]+nPoint[1]*nPoint[1]+nPoint[2]*nPoint[2] - lowerArm*lowerArm

        if(b*b - 4*a*c < 0 or abs(a) < 0.0001):
            print("problem here with b^2-4ac")
            return None
        tPlus = (-b+math.sqrt(b*b-4*a*c))/(2*a)
        tMinus = (-b-math.sqrt(b*b-4*a*c))/(2*a)

        result = np.array([])
        result1 = tPlus*normalVectors[k] + np.array(mPoint)
        result2 = tMinus*normalVectors[k] + np.array(mPoint)

        if k == 0:
            if result1[1] < result2[1]:
                result = result1
            else:
                result = result2
        elif k == 1:
            if result1[1] > result2[1] and result1[0] > result2[0]:
                result = result1
            else:
                result = result2
        else:
            if result1[1] > result2[1] and result1[0] < result2[0]:
                result = result1
            else:
                result = result2
        resultCenters.append(result)

    cA = [0, -wb+up, 0]
    cB = [0.5*(math.sqrt(3)*wb - sp), 0.5*wb - wp, 0]
    cC = [-0.5*(math.sqrt(3)*wb - sp), 0.5*wb - wp, 0]



    #return resultCenters

    return [math.asin(-resultCenters[0][2]/upperArm), math.asin(-resultCenters[1][2]/upperArm), math.asin(-resultCenters[2][2]/upperArm) ]         


def findXYZ(angle1,angle2,angle3):

    #checks if it is Y based or not
    c1 = [0.0, -wb-upperArm*math.cos(angle1)+up, -upperArm*math.sin(angle1)]
    c2 = [(math.sqrt(3)*(wb + upperArm*math.cos(angle2))-sp)/2.0, 0.5*(wb+upperArm*math.cos(angle2))-wp, -upperArm*math.sin(angle2)]
    c3 = [-(math.sqrt(3)*(wb + upperArm*math.cos(angle3))-sp)/2.0, 0.5*(wb+upperArm*math.cos(angle3))-wp, -upperArm*math.sin(angle3)]


    varMatrix = [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    varMatrix[0][0] = 2*(c3[0]-c1[0])
    varMatrix[0][1] = 2*(c3[1]-c1[1])
    varMatrix[0][2] = 2*(c3[2]-c1[2])

    varMatrix[1][0] = 2*(c3[0]-c2[0])
    varMatrix[1][1] = 2*(c3[1]-c2[1])
    varMatrix[1][2] = 2*(c3[2]-c2[2])

    varMatrix[0][3] = -c1[0]*c1[0]-c1[1]*c1[1]-c1[2]*c1[2]+c3[0]*c3[0]+c3[1]*c3[1]+c3[2]*c3[2]
    varMatrix[1][3] = -c2[0]*c2[0]-c2[1]*c2[1]-c2[2]*c2[2]+c3[0]*c3[0]+c3[1]*c3[1]+c3[2]*c3[2]

    normalVector = np.cross(varMatrix[0][0:3],varMatrix[1][0:3])
    
  
    if abs(normalVector[0]) < 0.0001 and abs(normalVector[1]) < 0.0001 and abs(normalVector[2]) < 0.0001:
        print("parallel planes intersection planes here")
        print(c1,c2,c3)
        print(varMatrix)
        return [0,0,0]
    #point containing the line resulting from intersection of both planes in varMatrix
    mPoint = findPointofLine(varMatrix)
    if(mPoint == None):
        return [0,0,0]

    nPoint = np.subtract(mPoint, c1).tolist()


    a = normalVector[0]*normalVector[0]+normalVector[1]*normalVector[1]+normalVector[2]*normalVector[2]
    b = 2*(normalVector[0]*nPoint[0]+normalVector[1]*nPoint[1]+normalVector[2]*nPoint[2])
    c = nPoint[0]*nPoint[0]+nPoint[1]*nPoint[1]+nPoint[2]*nPoint[2] - lowerArm*lowerArm

    tPlus = (-b+math.sqrt(b*b-4*a*c))/(2*a)
    tMinus = (-b-math.sqrt(b*b-4*a*c))/(2*a)

    if(b*b - 4*a*c < 0 or abs(a) < 0.0001):
        print("problem here with b^2-4ac")
        return [0,0,0]

    result1 = tPlus*normalVector + np.array(mPoint)
    result2 = tMinus*normalVector + np.array(mPoint)

    if result1[2] < result2[2]:
        return result1.tolist()
    else:
        return result2.tolist()



result = findXYZ(0.5236,0.5236,0.5236)

print(result[0],result[1],result[2])

#result of inverse kinematic
print inverseKinematic(-100,100,-500)




#part that run script
angles = []
for i in q1:
    for l in q2:
        for k in q3:
            angles.append((math.radians(i),math.radians(l),math.radians(k)))
            
            
#forwared kinematics to find graph

x=[]
y=[]
z=[]
#convert angle to radian
for angle in angles:
    #three centers

    result = findXYZ( angle[0] ,angle[1] ,angle[2])

    if (result==[0,0,0]):
        continue
    
    x.append(result[0])
    y.append(result[1])
    z.append(result[2])

        

#logic from calculated point


# Cylinder
x1=np.linspace(-100, 100, 50)
z1=np.linspace(-500, -200, 100)
Xc, Zc=np.meshgrid(x1, z1)
Yc = np.sqrt(10000-Xc**2)
rstride = 50
cstride = 10

    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect('equal')
ax.scatter(x, y, z, c=z, marker='x', cmap='hsv', alpha = 1)
rstride = 10
cstride = 10
ax.plot_surface(Xc, Yc, Zc, alpha=1, rstride=rstride, cstride=cstride)
ax.plot_surface(Xc, -Yc, Zc, alpha=1, rstride=rstride, cstride=cstride)




ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')



plt.show()



#return a pair of possible solutions

