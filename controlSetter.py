# -*- coding: utf-8 -*-
"""
Created on Mon Sep 21 20:12:29 2015

@author: Edward
"""
import numpy as np
import matplotlib.pyplot as plt


class controlSetter(object):
    def __init__(self):
        self.xa = np.zeros(100)
        self.ya = np.zeros(100)
        self.i = 0
        self.fig = plt.figure()
        ax = self.fig.add_subplot(111)
        ax.autoscale(False)
        plt.axis([-10, 10,-10,10])
        plt.show()
    #def __call__(self):
        def onclick(event):
            button=event.button
            x=event.xdata
            y=event.ydata
            if button==1:
                plt.plot(x,y,'ro')
                self.xa[self.i] = x
                self.ya[self.i] = y
                plt.plot(self.xa[:self.i+1],self.ya[:self.i+1],'r--')
                print(self.i)
                self.i+=1
            if button!=1: 
                self.fig.canvas.mpl_disconnect(self.cid)
                self.xa=self.xa[:self.i]
                self.ya=self.ya[:self.i]
                controlPoint = np.vstack((self.xa,self.ya))
                np.save('controlPoints',controlPoint)
                plt.close('all')
        self.cid = self.fig.canvas.mpl_connect('button_press_event',onclick)
        return

cs = controlSetter()