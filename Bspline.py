# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 09:35:16 2015

@author: Edward
"""
import numpy as np
import scipy as sp


class spline(object):
    
    def __init__(self,knots,control):
        
        self.knots = knots
        self.control = control
        
    def __call__(self,u):
        s = self.computeS(u)
        


    def computeS(self,u):
        index = self.findHot(u)
        dx = self.sU(u,index,index+1,0)
        dy = self.sU(u,index,index+1,1)
        return np.array[[dx],[dy]]
        
    def sU(self,u,rightMost,leftMost,dim):
        #Calculates the blossoms with recursive algorithm
        if rightMost - leftMost == 2:
            return self.control[dim,leftMost]
        else:
            alpha = (self.knots[rightMost]-u)/(self.knots[rightMost]-self.knots[leftMost])
            return alpha*self.sU(u,rightMost,leftMost-1,dim)+(1-alpha)*self.sU(u,rightMost+1,leftMost,dim)
    def findHot(self,u):
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


        
        
        
        
