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
        
    

u = np.array([[0,0,0,1,2,2,2],[1,1,1,2,3,3,3]])
ux = u[0]
uy = u[1]
xix = (ux[:-2]+ux[1:-1]+ux[2:])/3.
xiy = (uy[:-2]+uy[1:-1]+uy[2:])/3.
xi = np.vstack((xix,xiy))       
print(xi)

