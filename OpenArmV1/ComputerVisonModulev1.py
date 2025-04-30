
'''
=============================================================

/ Engineer:   Pacheck Nicholas
// Date Created: 11/16/23

// Module Description:

    This module handles all the image processing and landmark estimation required for OpenVIMA's
    MIMIC Controller. Applicable/used in both Forward Kinematics and Inverse Kinematics Version.
    While the mediapipe library provides landmark solutions for body, and landmark solutions for hands
    the goal of this module is to combine those solutions together.

    Forward Kinematics:
        Provides set of angles determined by joint angles of user.

    Inverse Kinematics:
        Provides (x,y,z) coordinate describing the end effector ( finger ) position

    Challenges:
        Z-coordinate estimation in addtion to localization have been challenging




// TODO:

1. Add baud rate, zfill, and port communications to insure compatibility across devices
2. embed the self.conenciton into the status bits



// Modifications:
    Date:    Engineer:     Modification Description

=============================================================
'''


import queue

import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
import mediapipe as mp
import pickle as pkl
import math
import threading


#SAME AS ARDUINO MAP() FUNCTION
def changeBasis(value, fromLow, fromHigh, toLow, toHigh):
    try:return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
    except ZeroDivisionError as ZERO:
        print(ZERO)
        return 0


    #TODO, GPU ACCELERATION





class ComputerVision:
    '''This class is to be used as module for image processing and computer vision landmark detection.
    Init the module in the constructor to be used as a submodule within the class it will operate in.
    If you arent using OOP then just make sure you init the class not within the while loop.

    call image processing as main'''

    def __init__(self, threaded = False, cap = cv2.VideoCapture(0)):

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

    def __init__(self):
        #inicnial conditions for the running statisics
        self.max_area = -1000000000000
        self.min_area = 10000000000000

        #The area read when the user is at their origin
        self.area_origin = 0
    #constructor

    def generate_zCoords(self, coords):

        #compute area
        area = self.compute_bodyArea(coords)
        #Normalize data point
        z_offset = 100/(np.sqrt(area))

        return z_offset
    #returns a z coordinate offset related to the distance away from the camera

    def compute_bodyArea(self, coords):

        #Unpack coordinates
        try:
            cx, cy, cz = coords
        except ValueError:
            cx, cy, cz = coords[0], coords[1], coords[2]
        #Compute boundary conditions
        max_cx, max_cy, max_cz = max(cx), max(cy), max(cz)
        min_cx, min_cy, min_cz = min(cx), min(cy), min(cz)
        #Compute area of boundary conditions
        bbox_len, bbox_heigh = (max_cx - min_cx), (max_cy - min_cy)
        bbox = bbox_len*bbox_heigh
        #update extrema
        if bbox > self.max_area: self.max_area = bbox
        if bbox < self.min_area: self.min_area = bbox

        return bbox
    #return the area in pixels the the max and min on the body do

    def set_zeroOrigin(self, bbox_area):
        self.area_origin = bbox_area
    #sets the zere axis inicialiation poitnt








#main
class MimicController:

    '''This class should work in harmony withe multivariable calculaus solver, the depth localizer, ]
    and the computer vision modle to provdie smooth and reliable coordiantes to the serial bus.

    The base fk angle will be derived by approxinamating the z- location of the users hips which
    coorespond to the coordinates 23 and 24.
    The shoulder and elbow angles are dervifed from the dotproduct of the two vecotrs in 3 dimentiosn to find angle
    DiffA and DiffB wiill be derived from the inverse kinematics sections to select the orientation of the robto'''


    def __init__(self):


        #Boolean that defines if the program is for 5 or 6 dof robot
        self.SIX_DOF_MODE = False
        self.verbose = False

        #list which holds the cv landmark indexes between limbs
        #Indexed shoulder, elbow,
        self.fk_joints = [[23, 11, 13], [11, 13, 21]] #BASE NOT INCLUDED

        #Updates each loop so that we dont have to pass the coords into every funciton but instead just reference the attribute
        self.cx_list, self.cy_list, self.cz_list = [], [], []
        self.hand_coords = []
        self.body_coords = []

        #boolean element lis that tells if two landmarks are touching or not
        self.click_flag = []


        #Init computer vision module
        self.Vision = ComputerVision(threaded=True)

        #todo Init multivariable calculus module

        #Init depth localizer
        self.Depth_Sovler = DepthLocalizer()


    def top_main(self):


        #call computer vision process
        img, body_coords, hand_coords = self.Vision.imageProcessing()

        #make attribute updates
        self.update_coordinates(body_coords, hand_coords)

        #Forward kinematics
        thetaListFK = self.compute_fkLimbAngles()

        #inverse kinematics
        thetaListIK = self.compute_ikHandControlLimbAngles()

        thetaListFK = [ int(angle) for angle in thetaListFK]


        if self.verbose: print(thetaListFK, thetaListIK.shape)
        return img, thetaListFK, thetaListIK

    #main1
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

        return np.array(thetaListFK)
    #returns the joint angles to each accturator to match user arm angles



    def compute_ikHandControlLimbAngles(self):


        #Compute the (x,y,z) coordinate of the finger end effector


        #Compute the vector given from wrist to end effector tip

        #Compute 3d quaternion orientation in space


        #Compute inverse kinematics with infomration above, returns fk joint angles



        return np.array([])
    #returns the joint angles to each accturator to match (x,y,z) coordiante in space and quaternionic orientiation

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
        dist = self.Depth_Sovler.generate_zCoords(self.body_coords)
        # Add z coord offset to the predicted  coords
        #self.cz_list = [z + dist for z in self.cz_list]



        return baseTheta



    def compute_diffAngles(self):
        return [0,0]

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


    def update_coordinates(self, body_coords, hand_coords):
        self.cx_list, self.cy_list, self.cz_list = body_coords
        self.body_coords = body_coords
        self.hand_coords = hand_coords


def plot_topView(coords, recent_coords, ax):
    cx_list, cy_list, cz_list = coords

    # Get hip coordinates of interest
    xcoords = [cx_list[24], cx_list[23]]
    zcoords = [cz_list[24], cz_list[23]]

    # Append them to a most recent list
    recent_coords = np.append(recent_coords, [xcoords, zcoords], axis=1)

    if recent_coords.shape[1] >= 60:
        # Remove the oldest appended coordinate
        recent_coords = np.delete(recent_coords, 0, axis=1)
        recent_coords = np.delete(recent_coords, 0, axis=1)


    # Clear plot
    ax.clear()

    # Plot all coordinates in recent_coords
    ax.scatter(recent_coords[0], recent_coords[1])
    unit = .2
    ax.set_ylim([-unit, 1.2])
    ax.set_xlim([150, 700])



    plt.pause(0.01)

    return recent_coords




#todo======
# Make 3D computer vision system
def main_threaded(count = 2000):
    # init mimic controler
    simon = MimicController()
    simon.Vision.USE_MULTITHREADING = True

    i = 0
    start_time = time.time()
    try:
        while i < count or count is None:
            # everything
            img, fk, ik  = simon.top_main()

            # ============= cv disp
            # Show the image
            cv2.imshow("This is the name of the image box", img)
            # Interface timing such that it looks like a video
            cv2.waitKey(1)
            i = i + 1

    finally:
        end_time = time.time()
        total_time = start_time - end_time
        print("THREADED TIME", total_time)
        return total_time


def main_no_threads(count):
    # init mimic controler
    simon = MimicController()
    simon.Vision.USE_MULTITHREADING = False
    i=0
    start_time = time.time()
    try:
        while i<count:

            # everything
            img, thetaListFK, thetaListIK = simon.top_main()

            # ============= cv disp
            # Show the image
            cv2.imshow("This is the name of the image box", img)
            # Interface timing such that it looks like a video
            cv2.waitKey(1)
            i = i + 1

    finally:
        end_time = time.time()
        total_time = start_time - end_time
        print("UNTHREADED TIME", total_time)
        return total_time



#Funciton unpuckles the data put ther en urinf ghte rest and proeforms data anlytics on the data
def analyzer_test():

    # Open the file for binary reading
    with open('data.pickle', 'rb') as f:
        # Unpickle the data
        bbox_area_list = pkl.load(f)

    loggedbbox_area_list = [np.log(x)+3*np.sqrt(x) for x,i in enumerate(bbox_area_list)]

    fig, ax = plt.subplots()
    ax.plot(np.linspace(0, 4, len(loggedbbox_area_list)), loggedbbox_area_list)

    plt.show()



def thread_comparision_testbench():

    count = 500000000000000
    threaded_time = main_threaded(count)
    standard_time = main_no_threads(count)

    print(f"threaded average cycle: {threaded_time/count}"
          f"standard average cycle: {standard_time/count} "
          f"")

if __name__ == "__main__":
    main_threaded()