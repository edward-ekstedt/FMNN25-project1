# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 09:35:16 2015

@author: Edward
"""
import numpy as np
import scipy as sp


class spline(object):
    def ma_matrix(self):
        # Computes the moving average of a 2-dimensional matrix over 3 elements
        # Creates the vector xi_i
        u = self.u
        ux = u[0]
        uy = u[1]
        xix = (ux[:-2]+ux[1:-1]+ux[2:])/3.
        xiy = (uy[:-2]+uy[1:-1]+uy[2:])/3.
        self.xi = np.vstack((xix,xiy))        
    def ma_array(self):
        u = self.u
        self.xi = (u[:-2]+u[1:-1]+u[2:])/3.
        
        
        
        