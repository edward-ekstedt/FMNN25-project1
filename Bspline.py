# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 09:35:16 2015

@author: Edward
"""
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt


class spline(object):
    
    def __init__(self,control,knots=None):
        self.control = control
        if knots != None:
            self.knots = knots
        
        
    def __call__(self,u):
        self.s = self.computeS(u)


    def computeS(self,u):
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
            alpha = 0#self.knots[rightMost]-u
            return alpha*self.sU(u,rightMost,leftMost-1,dim)+(1-alpha)*self.sU(u,rightMost+1,leftMost,dim)
        else:
            alpha = (self.knots[rightMost+1]-u)/(self.knots[rightMost+1]-self.knots[leftMost-1])
            return alpha*self.sU(u,rightMost,leftMost-1,dim)+(1-alpha)*self.sU(u,rightMost+1,leftMost,dim)
    def findHot(self,u):
        #jomp
        #Finds the interval in which u is.
        return (self.knots > u).argmax()
        
    def findD(self,u):
        k=3
        u = self.u
        xi = (u[:-2]+u[1:-1]+u[2:])/3.
        NMatrix = np.zeros((len(xi),len(xi)))
        for i in range(len(xi)):
            for j in range(len(xi)):
                NMatrix[[i],[j]]=self.computeNXi(u,k,i,xi[j])
        dx = sp.linalg.solve(NMatrix,x)
        dy = sp.linalg.solve(NMatrix,y)
        
    def computeNXi(self, u, k, i, xi):
        if k==0:
            if u[i-1] == u[i]:
                return 0
            elif u[i]>xi>=u[i-1]:
                return 1
            else:
                return 0
        else:
            if u[i-1]==u[i+k-1]:
                coef1=0
            else:
                coef1=(xi-u[i-1])/(u[i+k-1]-u[i-1])
            if u[i+k]==u[i]:
                coef2=0
            else:
                coef2=(u[i+k]-xi)/(u[i+k]-u[i])
        NXi = coef1*self.computeNXi(u, k-1, i, xi)+coef2*self.computeNXi(u, k-1, i+1, xi)
        return NXi


    def plot(self):
        
        x = self.s[0]
        y = self.s[1]
        plt.figure(2)
        plt.plot(x,y,'b')
        plt.plot(self.control[0],self.control[1],'ro')
        plt.plot(self.control[0],self.control[1],'r--')
        plt.show()
        

    
def main():
    plt.close('all')
    control = np.load('CLAUS.npy')
    knots = np.linspace(0,1,len(control[0])-2)
    knots = np.hstack(([0,0],knots,[1, 1]))
    u = np.linspace(0.,0.999,1000)
    Sp = spline(control,knots)
    Sp(u)
    Sp.plot()
main()

#knots = np.linspace(0,1,12)
#u = np.linspace(0,1,100)
#knots = np.hstack(([0,0],knots,[1, 1]))
#control = np.array([[5.,1,2,3,2,1,-1,2,-3,-4,-5,-5,-7],[4.,-3,2,1,-2,-3,-2,-1,2,3,4,3,6]])
#Sp = spline(knots,control)
#Sp(u)
#Sp.plot()
        
