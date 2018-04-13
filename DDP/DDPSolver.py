# DDPSolver inspired by flforget https://github.com/flforget/DDP

import numpy as np
import numpy.linalg
import matplotlib.pyplot as plt
from costFunction import *
from dynamicModel import *
import time

class DDPSolver:
    def __init__(self, model, costFunction):
        self.model = model
        self.costfucntion = costFunction.named_func # TODO

        # These init values are set by solveTrajectory -- how does this work?
        self.xinit = np.zeros((model.stateNumber, 1))
        self.Xdes = np.zeros((model.stateNumber, 1)) # What does this do?
        self.T = 10
        self.dt = 1e-4
        self.iterMax = 20
        self.stopCrit = 1e-3

        self.changeAmount = 0.0
        self.completeBackwardFlag = 0

        self.X = np.zeros((self.model.stateNumber, 1))
        self.nextX = np.zeros((self.model.stateNumber, 1))
        self.U = np.zeros((self.model.stateNumber, 1))
        self.nextXList = []
        self.nextUList = []
        self.Qx = np.zeros((self.model.stateNumber, 1))
        self.Qu = np.zeros((self.model.stateNumber, 1))
        self.Qxx = np.zeros((self.model.stateNumber, 1, self.model.stateNumber, 1))
        self.Quu = np.zeros((self.model.stateNumber, 1, self.model.stateNumber, 1))

        self.alphaList = [1.0, 0.8, 0.6, 0.4, 0.2] # Are these enough?
        self.alpha = 1.0
        self.mu = 0.0
        self.muEye = self.mu * np.eye(self.model.stateNumber, dtype=float)

        self.zerosCommand = np.zeros((self.model.stateNumber, 1))

        # What do these do?
        self.kList = []
        self.KList = []
        self.XList = []
        self.UList = []

        # What do these do?
        self.k = np.zeros((self.model.stateNumber, 1))
        self.K = np.zeros((self.model.stateNumber, 1, self.model.stateNumber, 1))
        self.Qx = np.zeros((self.model.stateNumber, 1))
        self.Qu = np.zeros((self.model.stateNumber, 1))
        self.K = np.zeros((self.model.stateNumber, 1, self.model.stateNumber, 1))

    def solveTrajectory(self, Xinit, Xdes, T, dt, iterMax=20, stopCrit=1e-3):
        '''
        Intialize variables, backward pass, forward pass,
        stop when converge or iterMax
        '''

        self.Xinit = Xinit
        self.Xdes = Xdes
        self.T = T
        self.dt = dt
        self.iterMax = iterMax
        self.stopCrit = stopCrit

        self.initTrajectory()
        for i in range(self.iterMax):
            self.backwardLoop()
            self.forwardLoop()
            self.XList = self.nextXList
            self.UList = self.nextUList
            if(self.changeAmount < self.stopCrit):
                break
        return self.XList, self.UList

    def initTrajectory(self):
        self.XList = self.Xinit
        self.UList = [self.zerosCommand for i in range(self.T)]
        for in range(self.T):
            self.model.computeNextState(self.dt, self.XList[i], self.UList[i])
            self.XList.append(self.model.nextX)
        return 0

    def backwardLoop(self):
        self.kList = []
        self.KList = []
        self.costfunction.computeFinalCostDeriv(self.XList[self.T], self.Xdes)
        self.nextVx = self.costfunction.lx
        self.nextVxx = self.costfunction.lxx
        self.mu = 0.0
        self.completeBackwardFlag = 0
        while(self.completeBackwardFlag==0):
            self.completeBackwardFlag = 1
            self.myEye = self.mu*np.eye(self.nextVxx.shape[0], dtype=float)
            for i in range(self.T -1, -1, -1): #backward pass
                self.X = self.XList[i]
                self.U = self.UList[i]

                self.model.computeAllModelDeriv(self.dt, self.X, self.U)
                self.costfunction.compute







