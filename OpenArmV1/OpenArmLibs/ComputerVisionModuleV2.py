import InverseKinematicsModule as Kinematics_module
import ComputerVisonModulev1 as Vision_module
from OpenArmV1.OpenArmLibs.DynamicsSolvers import MultivariableCalculus as MultiVarCalc_module, StatsModule

import queue

import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
import mediapipe as mp
import math
import threading


# SAME AS ARDUINO MAP() FUNCTION
def changeBasis(value, fromLow, fromHigh, toLow, toHigh):
    try:
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
    except ZeroDivisionError as ZERO:
        print(ZERO)
        return 0

class ComputerVision:
    '''This class is to be used as module for image processing and computer vision landmark detection.
    Init the module in the constructor to be used as a submodule within the class it will operate in.
    If you arent using OOP then just make sure you init the class not within the while loop.

    call image processing as main'''

    def __init__(self, threaded = False,proc=False, cap = cv2.VideoCapture(0)):

        #An attribute that contains a path to methods for drawing on image
        self.mpDraw = mp.solutions.drawing_utils

        # Inicialized the Machine Learning for the pose detection. Is constructor from solutions
        #This attribute contains the solutions from the Machine Learning (ML) Library "Mediapipe"
        self.mpPose = mp.solutions.pose

        # Defines a pose through method beloinging to mpPose constructor under solutions class
        #Contains information about what the results of the ML where
        self.pose = self.mpPose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

        # Starts up the camera but doesnt take any pictures, just decarles camera.
        # For robotic arm we are going toconfigure the camer here
        self.cap = cap

        #Coordinate minimas and maximas that define the boundary conditions tha thte user canoperate in
        self.coord_min, self.coord_max = -200, 200
        self.verbose = False

        self.USE_MULTITHREADING = threaded
        self.USE_MULTIPROCESSING = proc
        if self.USE_MULTITHREADING:
            #Used for threaded image processing to return asnwers from each thread
            self.thread_answers = queue.Queue()

    #Constrctor, puts together the dependecnies of thre mediapipe and oper cv into the same scope

    def get_landmarks(self, img):

        # Pose detection algorith works in (red,green,bue) while cv library works in BGR, thus we need to convert.
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  #

        # Now we Process the pose in the new color formatting. Is a class that contains the pose solutions
        results = self.pose.process(imgRGB)


        #RESET THE COORDINATE LISTS
        cx_list, cy_list, cz_list =  [], [] ,[]
        ogimg = img

        #IF A PERSON IS IN FRAME
        if results.pose_landmarks:

            if self.verbose: print("human found")
            #DRAW LANDMARKS ON HUMAN ON IMAGE THAT OUTLINES THE BODY
            self.mpDraw.draw_landmarks(img, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS,
                                       self.mpDraw.DrawingSpec(color=(55,175,212), thickness=3, circle_radius=2),
                                       self.mpDraw.DrawingSpec(color=(1,0,139), thickness=4, circle_radius=2))

            #LOOPS OVER A ENUMERATED SET OF POSE LANDMARKS
                #THE landmark, lm, has x and y values which define the landmarks ratio wrt the shape of img
            for id, lm in enumerate(results.pose_world_landmarks.landmark):
                #GET SHAPE OF IMAGE MATRIX
                h, w, c = img.shape
                #GET COORDINATE FOR LANDMARK ID
                cx, cy, cz = float(lm.x * w), float(lm.y * h), float(lm.z * c)

                #APPEND THE COORDINATE TO COORDINATE LISTS
                cx_list.append(float(cx))
                cy_list.append(float(cy))
                cz_list.append(float(cz))

            if self.USE_MULTITHREADING:
                results = {"body":[cx_list, cy_list, cz_list]}
                self.thread_answers.put(results)

            return cx_list, cy_list, cz_list, ogimg

        else:
            print("No human")
            time.sleep(1)
            return [],[],[], ogimg
    # returns a list of points for which cx, cy, and cZ for body landmarks

    def get_hand_landmarks(self, og_image, image):


        returnThis = [0,0,0,0,0,0,0,0]


        #Hands should probrally only be used for camera at end effector
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_tracking_confidence=.6)
        results = hands.process(cv2.cvtColor(og_image, cv2.COLOR_BGR2RGB))
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            returnThis =  [results.multi_hand_landmarks[i].landmark for i in range(len(
                                                results.multi_hand_landmarks
            ))]
        if self.USE_MULTITHREADING:

            thread_answer = dict({"hand": returnThis})
            self.thread_answers.put(thread_answer)

        return returnThis

    #returns the landmark cx, cy, cz coordinates from the usershand
    def imageProcessing(self, threaded = None):

        if threaded is not None: self.USE_MULTITHREADING = threaded

        if self.USE_MULTITHREADING:
            img, body_coords, hand_coords = self.threaded_imageProcessing()
        elif self.USE_MULTIPROCESSING:
            pass
        else: # dont use multithreading
            img, body_coords, hand_coords = self.unthreaded_imageProcessing()
        return img, body_coords, hand_coords



    def unthreaded_imageProcessing(self): #somewhat depracated, the threading works much better

        #READ IMAGE
        success, img = self.cap.read()
        #resize image for useable basis
        cv2.resize(img, (800, 600))
        #store image such that both processes can use seperate image
        self.ogimg = img


        # Find coordinates of user (X,Y,Z)
        cx_list, cy_list, cz_list, ogimg = self.get_landmarks(img)

        # Hand landmark preditions
        hand_coords = np.array(self.get_hand_landmarks(self.ogimg, img))

        return ogimg, np.array([cx_list, cy_list, cz_list]), hand_coords
    #returns the original image taken, body coordinates, and hand corodinares

    def threaded_imageProcessing(self):

        # READ IMAGE
        success, img = self.cap.read()
        # resize image for useable basis
        #cv2.resize(img, (800, 600))
        # store image such that both processes can use seperate image
        self.ogimg = img

        #Make a thread for each detection type
        body_thread = threading.Thread(target = self.get_landmarks,
                                        args=(img, ))
        hand_thread = threading.Thread(target=self.get_hand_landmarks,
                                       args=(self.ogimg, img))
        threads = [body_thread, hand_thread]
        #start and join threads back to main thread
        [thread.start() for thread in threads]
        [thread.join() for thread in threads]

        result1 = self.thread_answers.get()
        result2 = self.thread_answers.get()
        results = {**result1, **result2}

        #print("HAND: ",results['hand'])
        #print("BODY: ", results['body'])

        body_coords = results['body']
        hand_coords = results['hand']


        return img, np.array(body_coords), np.array(hand_coords)



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

#DEEFINE HAND LANDAMRKS
WRIST = 0
INDEX = 8
INDEX_BOTTOM = 5
class Person:


    def __init__(self, do_kin=True):



        #boundary limits for urdf robot
        self.ik_limits = [[-200, 200], [-200, 200], [-200, 200], [-6, 6], [-6, 6], [-6, 6]]

        # bool
        self.do_kinematics = do_kin
        # Init kinematics module only if in test and trying to show graphics
        if self.do_kinematics:
            self.IK = Kinematics_module.InverseKinematics(self.ik_limits)
            self.FK = Kinematics_module.ForwardKinematics(self.ik_limits[0])


        # inicialize the computer vision module
        self.eye = Vision_module.ComputerVision(threaded=True)

        # maps user vector space to robot vector space
        self.PathFusor = StatsModule.MultiCoordinateFusion(self.ik_limits)
        # used for some basic calculations
        self.mCalc = MultiVarCalc_module.MultiVar_Calc()
        # used to give an additonal z depth based on hand/body area
        self.depthLocalizer = DepthLocalizer()



        self.img = None
        #person coords
        self.body_coords = []
        self.cx_list, self.cy_list, self.cz_list = [[0], [0], [0]]
        self.hand_coords = []

        #last path coords
        self.path_coords = [[0],[0],[0]]
        #orientations (pitch roll yaw)
        self.orient_path_coords = [[],[],[]]

        #plots
        if not self.do_kinematics:
            self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='3d'))

        else:
            self.fig, self.ax = self.FK.fig, self.FK.ax
        self.prior_dest = [0,0,0,0,0,0] #update before real tst

        # used for pid
        self.prior_coord = self.prior_dest
        self.prior_error = [self.prior_dest]
        self.product_mode = False


    # inverse kinematics
    def get_destination(self):
        #Get Coords
        self.img, self.body_coords, self.hand_coords = self.eye.imageProcessing()

        # Create vector for hand
        hand_vec = [self.hand_coords[WRIST][INDEX].x - self.hand_coords[WRIST][INDEX_BOTTOM].x,
                    self.hand_coords[WRIST][INDEX].y - self.hand_coords[WRIST][INDEX_BOTTOM].y,
                    self.hand_coords[WRIST][INDEX].z - self.hand_coords[WRIST][INDEX_BOTTOM].z]
        #compute orientation
        orients = self.vector_to_roll_pitch_yaw(hand_vec)
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
            coords,#xyz
            hand_vec# oretnation vector

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

    def average_coords(self, destination):
        mean = [(now+prior)/2 for now, prior in zip(destination, self.prior_dest)]
        self.prior_dest = destination
        return mean
    def compute_ikHandControlLimbAngles(self):

        #Compute the (x,y,z) and <vx, vy,vz> of the oreintation of finger
        destination = self.get_destination()

        # compute the moving average
        destination = self.average_coords(destination)


        #fuse coordinates, return value is unimportant, used for plot limits
        fused = self.PathFusor.fuse_coordinates(destination, scalar=2)
        destination = fused

        destination = [destination[2], destination[0], -destination[1],
                       destination[3], destination[4],destination[5]]

        #COmpute the forward kinemtaics acutator angle sof rhte robot to tocuht that x,y,z
        # coordinate at the orientation defined by the euler angles derived form quaterniosn
        requestList = self.IK.main(destination)
        if requestList is not None and self.do_kinematics and None not in destination:
            #requestList = self.pid_loop(requestList)
            #ssend this request list of angle sfo rhte 6dof robot to the forward kinetmatic solver
            self.FK.main(requestList) # this does the plotting and disly
            return requestList

    #forward kinematics
    def compute_fkLimbAngles(self):

        #Compute fk angle between given 3 points for all sets of 3 points given
        shoudler_elbow = [self.compute_cv_angle(jointIndex) for jointIndex in self.fk_joints]

        #Compute the roll of the user s arm wrt to the robotic arm for DiffA and DiffB
        differTheta = self.compute_diffAngles()

        #Compute the base angle
        baseTheta = self.compute_baseAngle()

        #concatenate
        thetaListFK = np.array([baseTheta, shoudler_elbow[0], shoudler_elbow[1], differTheta[0], differTheta[1]])
        if len(thetaListFK) ==6:
            self.SIX_DOF_MODE = True
        thetaListFK = [ int(angle) for angle in thetaListFK]


        return np.array(thetaListFK)
    #returns the joint angles to each accturator to match user arm angles

    def compute_baseAngle(self):

        #unpack hip coordinates (left, right) s.t side:= xyz
        #hip_coords = np.array([[self.cx_list[24], self.cy_list[24], self.cz_list[24]],
        #                      [[self.cx_list[23], self.cy_list[23], self.cz_list[23]]]])




        #calculate the center of body
        centerCoord = np.array([abs(self.cx_list[23] + self.cx_list[24])/2,
                                abs(self.cy_list[23] + self.cy_list[24])/2,
                                abs(self.cz_list[23] + self.cz_list[24])/2])

        #calculate hip coordinates wrt to the center
        hip_coordsWRT_center = ([

            [centerCoord - np.array([self.cx_list[24], self.cy_list[24], self.cz_list[24]])],
            [centerCoord - np.array([self.cx_list[23], self.cy_list[23], self.cz_list[23]])]

        ])
        #include z offset on the center calcualtion but dont include for base calculation

        coords_hipLeft = centerCoord - np.array([self.cx_list[24], self.cy_list[24], self.cz_list[24]])
        x_hipLeft = coords_hipLeft[0]
        z_hipLeft = coords_hipLeft[2]

        arg = x_hipLeft/(np.sqrt(x_hipLeft**2 + z_hipLeft**2))
        baseTheta = np.rad2deg(np.arccos(arg))

        # compute z coordiante prozimation
        dist = self.depthLocalizer.generate_zCoords(self.body_coords)
        # Add z coord offset to the predicted  coords
        #self.cz_list = [z + dist for z in self.cz_list]



        return baseTheta



    def compute_diffAngles(self):
        return [0,0]
    def update_coordinates(self, body_coords, hand_coords):
        self.cx_list, self.cy_list, self.cz_list = body_coords
        self.body_coords = body_coords
        self.hand_coords = hand_coords
    def compute_cv_angle(self, jointIndexes):
        '''THIS FUNCTION COMPUTES THE ANGLE BETWEEN 3 POINTS P1, P2, AND P3
            WHICH ARE ENUMERATED ACCORDING TO LANDMARK IDs.

            COMPUTES IN 3D SPACE, should be called after image proccessing

            INPUTS:
                p1, p2, p3 where p2 is the rotation point
                self.(cx_list, cy_list, cz_list)

            OUTPUTS:
                Angle '''

        try:
            #Unpack joint indexes
            p1, p2, p3 = jointIndexes

            # Calculate the points defined by the paramters p1, p2, p3
            point1 = (self.cx_list[p1], self.cy_list[p1], self.cz_list[p1])  # make a coordiante (x, y, z) for p1
            point2 = (self.cx_list[p2], self.cy_list[p2], self.cz_list[p2]) # make a coordiante (x, y, z) for p2
            point3 = (self.cx_list[p3], self.cy_list[p3], self.cz_list[p3])  # make a coordiante (x, y, z) for p3


            #GIVEN 3 POINTS IN 3 DIM COORDINATE SPACE, DEFINE 2 VECOTRS
            v1 = np.array([point2[0] - point1[0],
                           point2[1] - point1[1],
                           point2[2] - point1[2]])  # V1 == (X_initcial - X_final, Y_init - Y_final)

            v2 = np.array([point3[0] - point2[0],
                           point3[1] - point2[1],
                           point3[2] - point2[2]])  # V2 == (X_initcial - X_final, Y_init - Y_final)

            #COMPUTE DOT PRODUCT
            v1dotv2 = np.dot(v1, v2)
            #COMPUTE MAGNITUDES OF EACH VECTOR
            v1_mag = np.linalg.norm(v1)
            v2_mag = np.linalg.norm(v2)

            # Calculate the angle using the dot product and magnitudes: solving for theta
            angle = math.acos(v1dotv2 / (v1_mag * v2_mag))

            # Convert angle to degrees
            angle = math.degrees(angle)

            return angle

        except IndexError as IE:
            print("INDEX ERROR IN CALCULATING CV ANGLE", IE)
            return 0
    #takes join indexes and returns the angles between the joint indexes from image




def test1():

    Pacheck = Person(True)



    while True:

        try:
            Pacheck.compute_ikHandControlLimbAngles()
            print(Pacheck.PathFusor.get_from_limits())


            if Pacheck.img is not None:
                img = Pacheck.img
                cv2.imshow("me", img)
                cv2.waitKey(1)
        #except TypeError as TE:
            #print(f"SOMETHINGS MISSING: {TE}")
            #pass
        except IndexError as IE:
            print("No hands", IE)



if __name__ == "__main__":
    test1()
