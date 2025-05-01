





import math
import cv2
import numpy as np




class RunningStats:

    '''This class is used to retirive statistics about a module
        as its running. Also enables a change of basis for multi-coordinate fusion.'''

    def __init__(self,name,  toLow, toHigh):


        #Inicital conditions
        self.fromLow=     500
        self.fromHigh =  -500

        # Boundary for basis we are changing to
        self.toLow =  toLow
        self.toHigh = toHigh
        self.name = name


    def update_stats(self, value):

        if value > self.fromHigh: self.fromHigh = value
        if value < self.fromLow: self.fromLow = value

        #print(self.name, "UPDATED")


    def changeBasis(self,value):
        try:
            return (value - self.fromLow) * (self.toHigh - self.toLow) / (self.fromHigh - self.fromLow) + self.toLow
        except ZeroDivisionError as ZERO:
            print(ZERO)
            return 0
        except FloatingPointError as FPE:
            print("fpe", FPE)


class MultiCoordinateFusion:

    def __init__(self, to_limits):

        # limits = [ [low, high],  [--], ... [--] ]
        self.to_limits = np.array(to_limits)
        #init a running stats module for each thing we want to track
        self.runningStatsModel = self.init_runningModels()
        self.from_limits = [[0.1408463940024376, 1.885925829410553],
                            [0.533228725194931, 1.893748939037323],
                            [-0.21015746146440506, 0.036600396037101746],
                            [-0.20434769988059998, 0.15319591760635376],
                            [-0.5627008080482483, 0.4764138162136078],
                            [-0.2630888298153877, 0.023174556903541088]]


    def get_from_limits(self):
        self.from_limits = [[model.fromLow, model.fromHigh] for model in self.runningStatsModel]

        return self.from_limits


    def init_runningModels(self):
        runningModels = [RunningStats(range(len(self.to_limits)),toLow, toHigh) for toLow, toHigh in self.to_limits]
        return np.array(runningModels)


    def fuse_coordinates(self, new_values, scalar = 2):

        if len(new_values) != len(self.to_limits):
            [print("INHOMOGENEOUS VALUES ACROSS STATS") for _ in range(10)]

        #instanciate memory for fused values
        fused_values = np.zeros(self.runningStatsModel.shape)
        #print(f"Fused Values Shape: {fused_values.shape}")
        # For each stats model we have, update stats, fuse coordinates
        for i, (statsModel, value) in enumerate(zip(self.runningStatsModel, new_values)):

            #Update stats
            statsModel.update_stats(value*scalar)

            #Fuse coordinate to new basis
            fused = statsModel.changeBasis(value)
            fused_values[i] = fused
        #print("Fused Values", fused_values)

        return fused_values




