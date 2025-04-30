



from DynamicsSolver import StatsModule
from OpenArmV1 import InverseKinematicsModule as Kinematics_module, ComputerVisonModulev1 as Vision_module
from DynamicsSolver import MultivariableCalculus as MultiVarCalc_module

import cv2
import math
import numpy as np
import matplotlib.pyplot as plt



class Person:

    def __init__(self):

        #boundary limits for urdf robot
        self.ik_limits = [[-200, 200], [-200, 200], [-200, 200], [-6, 6], [-6, 6], [-6, 6]]

        self.do_kinematics = False
        # Init kinematics module and ComputerVison Module
        if self.do_kinematics:
            self.IK = Kinematics_module.InverseKinematics(self.ik_limits)
            self.FK = Kinematics_module.ForwardKinematics(self.ik_limits[0])


        # other semi-needed clasees
        self.eye = Vision_module.ComputerVision(threaded=True)
        self.PathFusor = StatsModule.MultiCoordinateFusion(self.ik_limits[0:3])
        self.mCalc = MultiVarCalc_module.MultiVar_Calc()




        self.img = None
        #person coords
        self.body_coords = []
        self.hand_coords = []

        #last path coords
        self.path_coords = [[],[],[]]
        self.orient_path_coords = [[],[],[]]

        #plots
        self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='3d'))

        self.product_mode = False


    def get_destination(self):
        #Get Coords
        self.img, self.body_coords, self.hand_coords = self.eye.imageProcessing()

        # Create vector for hand
        hand_vec = [self.hand_coords[0][8].x - self.hand_coords[0][0].x,
                    self.hand_coords[0][8].y - self.hand_coords[0][0].y,
                    self.hand_coords[0][8].z - self.hand_coords[0][0].z]
        #compute orientation
        roll, pitch, yaw = self.vector_to_roll_pitch_yaw(hand_vec)
        #TODO: INIT ANOTHER FUSOR MODULE AND FUSE COORDS TO ROBOT URDF
        coords = [

            0.5 * (self.hand_coords[0][8].x + self.body_coords[0][19]),
            0.5 * (self.hand_coords[0][8].y + self.body_coords[1][19]),
            0.5 * (self.hand_coords[0][8].z + self.body_coords[2][19])

        ]

        destination = np.array([
            coords,
            [roll, pitch, yaw]

        ]).flatten()
        return destination

    def vector_to_roll_pitch_yaw(self,vector):
        x, y, z = vector
        roll = math.atan2(y, x)
        pitch = math.atan2(-z, math.sqrt(x ** 2 + y ** 2))
        yaw = math.atan2(math.sin(roll) * z - math.cos(roll) * y,
                         math.cos(roll) * x + math.sin(roll) * math.sin(pitch) * y + math.sin(roll) * math.cos(
                             pitch) * z)
        return (np.rad2deg(roll),
                np.rad2deg(pitch),
                np.rad2deg(yaw))

    def get_uvw(self,euler):
        roll, pitch, yaw = euler
        # Convert roll, pitch, and yaw angles to radians
        r = np.deg2rad(roll)
        p = np.deg2rad(pitch)
        y = np.deg2rad(yaw)

        # Define the unit vector in the local coordinate system
        u = np.array([1, 0, 0])

        # Define the rotation matrix
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(r), -np.sin(r)],
                        [0, np.sin(r), np.cos(r)]])
        R_y = np.array([[np.cos(p), 0, np.sin(p)],
                        [0, 1, 0],
                        [-np.sin(p), 0, np.cos(p)]])
        R_z = np.array([[np.cos(y), -np.sin(y), 0],
                        [np.sin(y), np.cos(y), 0],
                        [0, 0, 1]])
        R = R_z @ R_y @ R_x

        # Rotate the unit vector to the global coordinate system
        u_global = R @ u
        return 10*u_global

    def plot_hand_path(self, destination, path_length):
        coords, orients = destination[0:3], destination[3:]

        #clear plots
        self.ax.clear()


        #get list of path coordinates plotted
        self.path_coords = self.prep_stem_plot(coords, self.path_coords, path_length)
        xList, yList, zList = self.path_coords

        self.orient_path_coords = self.prep_stem_plot(orients, self.orient_path_coords, path_length)
        orient_xList, orient_yList, orient_zList = self.orient_path_coords


        # get running stats
        limits = self.PathFusor.get_from_limits()
        # update plot limits according to the path already drawn
        self.ax.set_xlim(*limits[0])
        self.ax.set_ylim(*limits[1])
        self.ax.set_zlim(*limits[2])


        #plot
        #self.ax.stem(xList, yList, zList)
        self.ax.quiver(xList, yList, zList, orient_xList, orient_yList, orient_zList,
                       length=0.1,
                       pivot = 'tip',
                       normalize = True
                       )
        plt.draw()
        plt.pause(0.1)

    def prep_stem_plot(self, new_coords, myList, path_length):
        def prep_stem_plot_sub(value, coord_set, path_length):
            #Make a meaned average value
            coord_set = coord_set + [value]
            mean_value = mean
            #But append the standard value that is passed into the function
            coord_set.append(np.mean([value] + coord_set))
            if len(coord_set)>= path_length:
                coord_set.pop(0)

            return coord_set

        path_coords = [prep_stem_plot_sub(new_coords[i], coord_set, path_length)
                       for i,coord_set in enumerate(myList)]
        return path_coords
        #function returns a list of 10 newest values

    def main(self):

        #updates cv and coords
        destination = self.get_destination()

        #fuse coordinates, return value is unimportant, used for plot limits
        self.PathFusor.fuse_coordinates(destination[0:3])

        #COmpute the forward kinemtaics acutator angle sof rhte robot to tocuht that x,y,z
        # coordinate at the orientation defined by the euler angles derived form quaterniosn
        if self.do_kinematics:
            requestList = self.IK.main(destination)
            #ssend this request list of angle sfo rhte 6dof robot to the forward kinetmatic solver
            self.FK.main(requestList) # this does the plotting and disly

        #PLot sthe last 5 coordinates choses along with the orientaiton
        self.plot_hand_path(destination, 20)







if __name__ == "__main__":

    Pacheck = Person()



    while True:

        try:
            Pacheck.main()


            img = Pacheck.img
            cv2.imshow("me", img)
            cv2.waitKey(1)
        #except TypeError as TE:
            #print(f"SOMETHINGS MISSING: {TE}")
            #pass
        except IndexError as IE:
            print("No hands", IE)


        finally:
            pass

            '''print(f"From Limits"
                  f"{Pacheck.PathFusor.get_from_limits()}"
                  f""
                  f"")'''''




