# -*- coding: utf-8 -*-
"""
Created on Tue Sep 22 09:56:46 2015

@author: marcusagren
"""

from Bspline import spline
import unittest
import numpy as np
import matplotlib.pyplot as plt

class Test_Bspline(unittest.TestCase):

# Tests concerning __INIT__
    
    def testControlArray(self):
        control = 12.
        self.assertRaises(TypeError,spline,control)   
    def testControlDim(self):
        control = np.array([[1., 2, 3, 4]])
        self.assertRaises(ValueError,spline,control)
    def testControlType(self):
        control = np.array([['splines','are','cool','!'],
                            [1, 2, 3, 4]])
        self.assertRaises(TypeError,spline,control)
        
    def testControlLen(self):
        control = np.array([[1,2,3],
                            [2,3,4.]])
        self.assertRaises(ValueError,spline,control)
        
    def testKnotType(self):
        control = np.array([[1,2,3,4],
                            [2,3,4,5.]])
        knots = [1,2,3,4.]        
        self.assertRaises(TypeError,spline,control,knots)
        
    def testKnotIndex(self):
        control = np.array([[1, 2, 3,4,5],
                          [1, 2, 3,4,5.]])
        knots = np.array([2,3,3.])                  
        self.assertRaises(ValueError,spline,control,knots)
        
    def testKnotControlSize(self):
        control = np.array([[1, 2, 3, 4, 5],
                            [1, 2, 3, 4, 5.]])
        knots = np.array([2,3,3,5.])  
        self.assertRaises(ValueError,spline,control,knots)
        
# Tests concerning __CALL__        
        
    def testCallType(self):
        control = np.array([[1, 2, 3, 4, 5],
                            [3, 2, 1, 2, 3.]])
        sp = spline(control)
        u = [0.1,0.2,0.3,0.4,0.5]
        self.assertRaises(TypeError,sp,u)

    def testCallFloat(self):
        control = np.array([[1, 2, 3, 4, 5],
                            [3, 2, 1, 2, 3.]])
        sp = spline(control)
        u = np.linspace(complex(0,1),complex(1,1),10)
        self.assertRaises(TypeError,sp,u)
        
    def testCallBounds(self):
        control = np.array([[1, 2, 3, 4, 5],
                            [3, 2, 1, 2, 3.]])
        sp = spline(control)
        u = np.linspace(0,2,10)
        self.assertRaises(ValueError,sp,u)
        
        
    def testSumBaseFunctions(self):
        knots = np.linspace(0,1.,10)
        knots = np.hstack(([0,0,knots,1,1]))
        control = np.array([[1, 2, 3, 4, 5],
                            [3, 2, 1, 2, 3.]])
        sp = spline(control)
        u = np.linspace(0,0.9999,100)
        plt.figure(1)
        x = np.zeros([12,len(u)])
        for j in range(12):
            basfn = sp.makeBasis(knots,j)
            x[j,:]=basfn(u)[0]
            plt.plot(x[j])
        
        plt.show()
        su = np.sum(x,axis=0)
        one = np.ones([1,len(su)])
        self.assertAlmostEqual(su.all(),one.all())
        
    def testSEqual(self):

        knots = np.linspace(0,1.,np.random.randint(5,high=10))
        knots = np.hstack(([0,0,knots,1,1]))
        control = np.random.rand(2,len(knots)-2)

        sp = spline(control)
        
        u = np.linspace(0,0.9999,100)
        s = sp(u)
        #plt.figure(1)
        x = np.zeros([len(control[0]),len(u)])
        for j in range(len(control[0])):
            basfn = sp.makeBasis(knots,j)
            x[j,:]=basfn(u)[0]
        print(len(s[0]))
        print(len(x[0]))
        shat = np.zeros([2,100])
        
        for j in range(100):
            shatx = 0
            shaty = 0
            for i in range(len(control[0])):
                shatx += x[i,j]*control[0,i]
                shaty += x[i,j]*control[1,i]
            shat[:,j] = [shatx,shaty]
            
        self.assertAlmostEqual(shat.all(),s.all())
        
unittest.main()        

