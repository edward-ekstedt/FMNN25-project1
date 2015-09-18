# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 09:35:16 2015

@author: Edward
"""
import numpy as np
import scipy as sp


class spline(object):
    def ma_matrix(self):
        u = self.u
        ux = u[0]
        uy = u[1]
        xix = (ux[:-2]+ux[1:-1]+ux[2:])/3.
        xiy = (uy[:-2]+uy[1:-1]+uy[2:])/3.
        self.xi = np.vstack((xix,xiy))  
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
        NXi = coef1*computeNXi(u, k-1, i, xi)+coef2*computeNXi(u, k-1, i+1, xi)
        return NXi
        
    

u = np.array([[0,0,0,1,2,2,2],[1,1,1,2,3,3,3]])
ux = u[0]
uy = u[1]
xix = (ux[:-2]+ux[1:-1]+ux[2:])/3.
xiy = (uy[:-2]+uy[1:-1]+uy[2:])/3.
xi = np.vstack((xix,xiy))       
print(xi)

