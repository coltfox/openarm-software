

from DynamicsSolvers import StatsModule
import InverseKinematicsModule as Kinematics_module
import ComputerVisonModulev1 as Vision_module
from DynamicsSolvers import MultivariableCalculus as MultiVarCalc_module

import math
import numpy as np
import matplotlib.pyplot as plt



# SAME AS ARDUINO MAP() FUNCTION
def changeBasis(value, fromLow, fromHigh, toLow, toHigh):
    try:
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
    except ZeroDivisionError as ZERO:
        print(ZERO)
        return 0



class DepthLocalizer:

    def __init__(self, dataset = []):

        self.max_area = -1000000000000
        self.min_area = 10000000000000

        #The area read when the user is at their origin
        self.area_origin = 0

        self.dataset = dataset

    def generate_zCoords(self, coords):

        #compute area
        area = self.compute_bodyArea(coords)
        #Normalize data point
        z_offset = 100/(np.sqrt(area))

        return z_offset


    def compute_bodyArea(self, coords):

        #Unpack coordinates
        cx, cy, cz = coords[0], coords[1], coords[2]
        try:
            #Compute boundary conditions
            max_cx, max_cy, max_cz = max(cx), max(cy), max(cz)
            min_cx, min_cy, min_cz = min(cx), min(cy), min(cz)
            #Compute area of boundary conditions
            bbox_len, bbox_heigh = (max_cx - min_cx), (max_cy - min_cy)
            bbox = bbox_len*bbox_heigh
        except ValueError as VE:
            return 0

        return bbox




    def set_zeroOrigin(self, bbox_area):
        self.area_origin = bbox_area



class Person:

    def __init__(self, do_kin=True):

        #boundary limits for urdf robot
        self.ik_limits = [[-200, 200], [-200, 200], [-200, 200], [-6, 6], [-6, 6], [-6, 6]]

        self.do_kinematics = do_kin
        # Init kinematics module and ComputerVison Module
        if self.do_kinematics:
            self.IK = Kinematics_module.InverseKinematics(self.ik_limits)
            self.FK = Kinematics_module.ForwardKinematics(self.ik_limits[0])


        # other semi-needed clasees
        self.eye = Vision_module.ComputerVision(threaded=True)
        self.PathFusor = StatsModule.MultiCoordinateFusion(self.ik_limits)
        self.mCalc = MultiVarCalc_module.MultiVar_Calc()
        self.depthLocalizer = DepthLocalizer()



        self.img = None
        #person coords
        self.body_coords = []
        self.hand_coords = []

        #last path coords
        self.path_coords = [[],[],[]]
        self.orient_path_coords = [[],[],[]]

        #plots
        if not self.do_kinematics:
            self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='3d'))

        else:
            self.fig, self.ax = self.FK.fig, self.FK.ax
        self.prior_dest = [0,0,0,0,0,0]
        self.prior_error = [self.prior_dest]
        self.product_mode = False


    def get_destination(self):
        #Get Coords
        self.img, self.body_coords, self.hand_coords = self.eye.imageProcessing()

        # Create vector for hand
        hand_vec = [self.hand_coords[0][8].x - self.hand_coords[0][5].x,
                    self.hand_coords[0][8].y - self.hand_coords[0][5].y,
                    self.hand_coords[0][8].z - self.hand_coords[0][5].z]
        #compute orientation
        roll, pitch, yaw = self.vector_to_roll_pitch_yaw(hand_vec)
        #u,v,w = self.get_uvw([roll, pitch, yaw])
        #TODO: INIT ANOTHER FUSOR MODULE AND FUSE COORDS TO ROBOT URDF
        # coords = [
        #
        #     0.5 * (self.hand_coords[0][8].x + self.body_coords[0][19]),
        #     0.5 * (self.hand_coords[0][8].y + self.body_coords[1][19]),
        #     0.5 * (self.hand_coords[0][8].z + self.body_coords[2][19])
        #
        # ]
        # define origin
        coords = [self.hand_coords[0][5].x,
                  self.hand_coords[0][5].y,
                  self.hand_coords[0][5].z ]


        destination = np.array([
            coords,
            hand_vec

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
        u = np.array([1, 1, 1])

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
        limits = self.PathFusor.to_limits#get_from_limits()
        # update plot limits according to the path already drawn
        self.ax.set_xlim(*limits[0])
        self.ax.set_ylim(*limits[1])
        self.ax.set_zlim(*limits[2])



        #plot
        #self.ax.stem(xList, yList, zList)
        self.ax.quiver(xList, yList, zList, orient_xList, orient_yList, orient_zList,
                       length=10,
                       pivot = 'tail',
                       normalize = False
                       )
        plt.draw()
        plt.pause(0.01)

    def prep_stem_plot(self, new_coords, myList, path_length):
        def prep_stem_plot_sub(value, coord_set, path_length):
            #Make a meaned average value
            coord_set = coord_set + [value]
            #But append the standard value that is passed into the function
            #coord_set.append(np.mean([value] + coord_set))
            if len(coord_set)>= path_length:
                coord_set.pop(0)

            return coord_set

        path_coords = [prep_stem_plot_sub(new_coords[i], coord_set, path_length)
                       for i,coord_set in enumerate(myList)]
        return path_coords
        #function returns a list of 10 newest values

    def pid_loop(self, destination):

        Kp, Ki, Kd = [.1,0,1]
        # compute the error
        error = [(coord-prior_coord)
                 for coord, prior_coord in zip(destination, self.prior_dest)]
        value = np.mean([e*e for e in error[:3]])

        if value>10:
            print("Movement", value)

            P = Kp*error
            #I = Ki*[now+before
                   # for now, before in zip(error, self.prior_error)]
            D = Kd*[now-before
                    for now, before in zip(error, self.prior_error)]

            self.prior_error = error
            self.prior_dest = destination

            PID = [P[i]+D[i] for i in range(len(P))]

            return destination
        else:
            self.prior_error = error

            return self.prior_dest
    def average_coords(self, destination):
        mean = [(now+prior)/2 for now, prior in zip(destination, self.prior_dest)]
        self.prior_dest = destination
        return mean
    def main(self):

        #updates cv and coords
        destination = self.get_destination()

        destination = self.average_coords(destination)


        #fuse coordinates, return value is unimportant, used for plot limits
        fused = self.PathFusor.fuse_coordinates(destination, scalar=2)
        destination = fused

        destination = [destination[2], destination[0], -destination[1],
                       destination[3], destination[4],destination[5]]

        #COmpute the forward kinemtaics acutator angle sof rhte robot to tocuht that x,y,z
        # coordinate at the orientation defined by the euler angles derived form quaterniosn
        if self.do_kinematics and None not in destination:
            # change the basis of the destincation for xyz
            # destination = [changeBasis(destination[0], limits[0], limits[1],
            #                            self.ik_limits[0][0], self.ik_limits[0][1]),
            #                changeBasis(destination[1], limits[0], limits[1],
            #                            self.ik_limits[0][0], self.ik_limits[0][1]),
            #                changeBasis(destination[2], limits[0], limits[1],
            #                            self.ik_limits[0][0], self.ik_limits[0][1]),
            #                destination[3], destination[4], destination[5]]
            # print(destination)
            #destination = 200*destination




            # comptue the fk angles needed to get to destination
            requestList = self.IK.main(destination)
            if requestList is not None:
                #requestList = self.pid_loop(requestList)
                #ssend this request list of angle sfo rhte 6dof robot to the forward kinetmatic solver
                self.FK.main(requestList) # this does the plotting and disly
        else:


            #PLot sthe last 5 coordinates choses along with the orientaiton
            self.plot_hand_path(destination, 20)







def test1():

    Pacheck = Person(True)



    while True:

        try:
            Pacheck.main()
            print(Pacheck.PathFusor.get_from_limits())



            #img = Pacheck.img
            #cv2.imshow("me", img)
            #cv2.waitKey(1)
        #except TypeError as TE:
            #print(f"SOMETHINGS MISSING: {TE}")
            #pass
        except IndexError as IE:
            print("No hands", IE)


        finally:
            print(Pacheck.PathFusor.get_from_limits())
            pass

            '''print(f"From Limits"
                  f"{Pacheck.PathFusor.get_from_limits()}"
                  f""
                  f"")'''''


test1()