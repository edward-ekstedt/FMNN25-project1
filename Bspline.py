# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 09:35:16 2015

@author: Edward Ekstedt, Johannes Olsson, Margus Ågren
"""
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt



class spline(object):
    
    def __init__(self,control,knots=None):
        
#==============================================================================
#         Initializes the spline object with parameters control and knots. 
#          control contains the de Boor points in two dimensions while knots
#       contains an array of knots. If either of the parameters are not numpy
#       arrays of floats, errors are raised. At least four control points are
#       needed. If knots are not given, an array of uniformly distributed points
#       on [0,1] are generated.
#==============================================================================
        if not isinstance(control,np.ndarray):
            raise TypeError('Control points must be a numpy array')
        if control.dtype != 'float64':
            raise TypeError('Control point array must concist of floats')
        if len(control) != 2:
            raise ValueError('Control point array must have dimension 2.')
        if len(control[0]) <= 3:
            raise ValueError('At least 4 control points are needed')
        self.control = control
        if knots != None:
            if not isinstance(knots,np.ndarray):
                raise TypeError('knots must be a numpy array')
            if knots.dtype != 'float64':
                raise TypeError('knot array must concist of floats.')
            knots.sort()
            if knots[0] != knots[1]:
                knots = np.hstack(([knots[0],knots]))
            if knots[1] != knots[2]:
                knots = np.hstack(([knots[0],knots]))
            if knots[-1] != knots[-2]:
                knots = np.hstack(([knots,knots[-1]]))
            if knots[-2] != knots[-3]:
                knots = np.hstack(([knots,knots[-1]]))
            if len(knots) != len(control[0]) +2:
                raise ValueError('Knot array is of wrong size')
            self.knots = knots
        else:
            knots = np.linspace(0.,1.,len(control[0])-2)
            self.knots = np.hstack(([0,0],knots,[1,1]))
        
    def __call__(self,u):
#==============================================================================
#        When the function is called, the B-splines at the points defined in array
#       u is evaluated. It returns the values of s(u)
#==============================================================================
        self.u = u        
        if not isinstance(u,np.ndarray):
            raise TypeError('u must be a numpy array')
        if u.dtype != 'float64':
            raise TypeError('u array must concist of floats.')
        u.sort()
        if u[np.argmax(u)] >= self.knots[-1] or u[np.argmin(u)] < self.knots[0]:
            raise ValueError('u out of bounds')
        self.s = self.computeS(u)
        return self.s

    

    @classmethod    
    def interpolate(cls, d):
#==============================================================================
#     returns a spline which can be used to interpolate, to create a graph 
#       which intersects the points defined in d.
#==============================================================================
        if not isinstance(d,np.ndarray):
            raise TypeError('Interpolation points must be a numpy array')
        if d.dtype != 'float64':
            raise TypeError('Interpolation point array must concist of floats')
        if len(d) != 2:
            raise ValueError('Interpolation point array must have dimension 2.')
        if len(d[0]) <= 3:
            raise ValueError('At least 4 interpolation points are needed')
        knots = np.linspace(0.,1.,len(d[0])-2)
        knots = np.hstack(([0,0],knots,[1,1]))
        cls.knots = knots
        return cls(cls.findD(cls, d, knots))
        
        

    def computeS(self,u):
        #Sets up arrays for the values of s(u) and loops through the blossom
        #recursion, returns the value of s(u).
        sX = np.zeros(len(u))
        sY = np.zeros(len(u))
        for i in range(0,len(u)):
            index = self.findHot(u[i])-1
            sX[i] = self.sU(u[i],index,index+1,0)
            sY[i] = self.sU(u[i],index,index+1,1)
        return np.array([sX,sY])
        
    def sU(self,u,rightMost,leftMost,dim):
        #Calculates the blossoms with recursive algorithm
        if rightMost - leftMost == 2:
            return self.control[dim,leftMost]
        elif self.knots[rightMost+1]-self.knots[leftMost-1] == 0:
            alpha = 0
            return alpha*self.sU(u,rightMost,leftMost-1,dim)+(1-alpha)*self.sU(u,rightMost+1,leftMost,dim)
        else:
            alpha = (self.knots[rightMost+1]-u)/(self.knots[rightMost+1]-self.knots[leftMost-1])
            return alpha*self.sU(u,rightMost,leftMost-1,dim)+(1-alpha)*self.sU(u,rightMost+1,leftMost,dim)
    def findHot(self,u):
        #Finds the interval in which u is.
        return (self.knots > u).argmax()
    def makeBasis(self,knots,j):
        #Returns a spline that can be used to evaluate the j:th B-spline basis
        # function, using the blossom recursion.
        controlBase = np.zeros([2,len(knots)-2])
        controlBase[:,j] = [1,1.]
        return  spline(controlBase)    
    def findD(self, d, knots):
        #Creates xi, the Greville abscissae, to be used in the Vermonde like
        # system. Then does a matrix division to receive the control points
        # in arrays dy and dx
        xi = (knots[:-2]+knots[1:-1]+knots[2:])/3.
        xi[-1] -= 0.0001 #to avoid errors
        NMatrix = np.zeros((len(xi),len(xi)))
        for i in range(len(xi)):
            for j in range(len(xi)):
                NMatrix[i,j] = self.makeBasis(self,knots,j)(np.array([xi[i]]))[0]       
        dx = sp.linalg.solve(NMatrix,d[0])
        dy = sp.linalg.solve(NMatrix,d[1])
        return np.vstack([dx,dy])

    def plot(self,plotControl = False):
        #Plots the B-spline, if plotControl is set to be true, the control
        #points and the control polygon is plotted as well.
        x = self.s[0]
        y = self.s[1]
        plt.figure()
        plt.plot(x,y,'b')
        if plotControl:
            plt.plot(self.control[0],self.control[1],'ro')
            plt.plot(self.control[0],self.control[1],'r--')
        plt.show()

def demoInterp():
    plt.close('all')
    control = np.array([[-8.18548387, -7.13709677, -2.82258065, -2.37903226,  1.00806452,
         2.41935484,  4.87903226,  5.88709677,  6.93548387,  7.41935484],
       [ 4.18410042, -3.45188285,  5.75313808, -2.71966527,  8.21129707,
        -3.66108787,  4.55020921, -0.31380753,  7.4790795 , -3.9748954 ]])

    u = np.linspace(0.,0.9999,1000)
    Sp = spline.interpolate(control)
    knots = np.linspace(0,1.,10)
    knots = np.hstack([0,0, knots, 1,1])
    Sp(u)
    Sp.plot()
    plt.figure(1)
    plt.plot(control[0],control[1],'ro')
def demoControl():
    control = np.array([[-6.53225806, -7.45967742, -8.62903226, -8.5483871 , -7.98387097,
        -7.74193548, -6.97580645, -6.12903226, -5.68548387, -5.12096774,
        -5.12096774, -5.04032258, -5.2016129 , -5.44354839, -5.44354839,
        -5.40322581, -4.7983871 , -4.39516129, -3.91129032, -3.30645161,
        -2.86290323, -2.37903226, -2.78225806, -2.98387097, -2.54032258,
        -1.97580645, -1.77419355, -1.77419355, -1.81451613, -2.09677419,
        -1.81451613, -1.16935484, -0.48387097, -0.04032258,  0.16129032,
         0.16129032,  0.16129032,  0.64516129,  0.88709677,  1.49193548,
         1.89516129,  2.13709677,  2.13709677,  1.97580645,  1.89516129,
         1.85483871,  2.58064516,  3.14516129,  3.75      ,  4.31451613,
         4.83870968,  4.59677419,  3.9516129 ,  4.67741935,  5.2016129 ,
         5.2016129 ,  4.63709677,  4.39516129,  3.87096774,  3.66935484,
        -6.93548387, -7.25806452, -5.80645161, -4.51612903, -3.62903226,
        -2.45967742, -3.06451613, -3.50806452, -3.58870968, -2.78225806,
        -1.85483871, -1.61290323, -1.65322581, -1.57258065, -0.84677419,
         0.04032258,  0.60483871,  0.48387097, -0.44354839, -1.41129032,
        -2.13709677],
       [ 6.85146444,  6.32845188,  4.44560669,  3.76569038,  2.66736402,
         2.51046025,  2.19665272,  2.03974895,  2.03974895,  2.77196653,
         3.45188285,  4.96861925,  5.59623431,  5.43933054,  4.44560669,
         3.92259414,  2.66736402,  2.14435146,  2.87656904,  3.66108787,
         3.92259414,  4.07949791,  3.5041841 ,  2.71966527,  2.09205021,
         2.19665272,  2.61506276,  3.08577406,  3.55648536,  3.9748954 ,
         3.76569038,  2.98117155,  2.30125523,  2.35355649,  3.08577406,
         4.44560669,  3.92259414,  2.98117155,  2.51046025,  2.66736402,
         2.77196653,  3.5041841 ,  4.13179916,  4.60251046,  4.96861925,
         5.07322176,  5.12552301,  5.12552301,  5.28242678,  5.4916318 ,
         5.4916318 ,  5.4916318 ,  4.60251046,  3.76569038,  3.66108787,
         2.87656904,  2.09205021,  1.88284519,  1.93514644,  1.93514644,
        -0.26150628, -1.5167364 , -4.13179916, -5.33472803, -5.28242678,
        -5.59623431, -4.55020921, -3.39958159, -1.62133891, -0.62761506,
        -0.94142259, -1.62133891, -2.77196653, -1.5167364 , -1.20292887,
        -1.41213389, -2.09205021, -3.5041841 , -4.49790795, -5.0209205 ,
        -6.01464435]])
    knots = np.linspace(0.,1.,len(control[0])-2)
    knots = np.hstack(([0, 1],knots))
    u = np.linspace(0.,0.999,1000)
    Sp = spline(control)
    Sp(u)
    Sp.plot()
def main():
    demoInterp()
    demoControl()
    
main()
