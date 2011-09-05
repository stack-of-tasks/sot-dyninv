
'''
Tiny matrix functions, taken from Oscar source code.
'''

#--------------------------------------------------------------------------
#--------------------------------------------------------------------------
#--------------------------------------------------------------------------
from random import random
from math import sqrt,atan2,pi

# Convert matrix to tuple
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

# Convert from Roll, Pitch, Yaw to transformation Matrix
def rpy2tr(r,p,y):
    mat = matrix(rotate('z',r))*matrix(rotate('y',p))*matrix(rotate('x',y))
    return matrixToTuple(mat)

# Get the distance btw the position components of 2 transf matrices
def distVector(M2,M1):
    px = M1[0][3] - M2[0][3]
    py = M1[1][3] - M2[1][3]
    pz = M1[2][3] - M2[2][3]
    return [px,py,pz]

# Obtain an orthonormal matrix SO3 using the given vector as its first column
# (it computes Gram Schmidt to obtain an orthonormal basis using the
#  first vector and 2 other 'random' vectors)

def generateOrthonormalM(v1):
    v2 = matrix([random(),random(),random()])
    v3 = matrix([random(),random(),random()])

    v1 = matrix(v1)
    e1 = v1/linalg.norm(v1)

    u2 = v2-inner(v2,e1)*e1
    e2 = u2/linalg.norm(u2)

    u3 = v3-inner(v3,e1)*e1-inner(v3,e2)*e2
    e3 = u3/linalg.norm(u3)

    e1=e1.tolist(); e2=e2.tolist(); e3=e3.tolist()
    M = ( (e1[0][0],e2[0][0],e3[0][0]), (e1[0][1],e2[0][1],e3[0][1]), (e1[0][2],e2[0][2],e3[0][2]) )
    return M

# Convert from Transformation Matrix to Roll,Pitch,Yaw
def tr2rpy(M):
    m = sqrt(M[2][1]**2+M[2][2]**2)
    p = atan2(-M[2][0],m)

    if abs( p-pi/2 ) < 0.001:
        r = 0
        y = atan2(M[0][1],M[1][1])
    elif abs( p+pi/2 ) < 0.001:
        r = 0
        y = -atan2(M[0][1],M[1][1])
    else:
        r = atan2(M[1][0],M[0][0])
        y = atan2(M[2][1],M[2][2])

    return [float(r),float(p),float(y)]

def matrixToRPY( M ):
    '''
    Convert a 4x4 homogeneous matrix to a 6x1 rpy pose vector.
    '''
    rot = tr2rpy(M)
    return [ M[0][3], M[1][3], M[2][3], rot[2],rot[1],rot[0]]
