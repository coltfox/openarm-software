import math
import random
import mediapipe as mp
import cv2
import numpy as np
import time
import numpy
import matplotlib.pyplot as plt

# STEP 1: Import the necessary modules.
import mediapipe as mp
import pickle as pkl






'''
THIS SCRIPT CONTAINS THE REVISED COMPUTER VISION MODULE FOR GORA:

RUNING THIS SCRIPT WILL INICIATE A DEMO FOR THE COMPUTER VISION FOR GORA, 
BUT WILL NOT CONNECT TO GORA ALONE, SEE MAIN.

'''





#SAME AS ARDUINO MAP() FUNCTION
def changeBasis(value, fromLow, fromHigh, toLow, toHigh):
    try:return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
    except ZeroDivisionError as ZERO:
        print(ZERO)
        return 0


class ComputerVision:
    #This is called the constructor: They give the object different "attributes"
    # "self" is referencing the object, or the "self". More specifically it is in reference to the instance of the object
    #I will talk more about the "instance" of the object later

    '''TLDR: This function gets called when you iniciate the instance of the object'''

    def __init__(self, ACTIVE_KALMAN, full_world_plot_bool = True, cap = cv2.VideoCapture(0)):

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

        self.coord_min, self.coord_max = -200, 200

        self.verbose = False
        self.full_world_plot_bool = full_world_plot_bool


        self.eigVal, self.eigVec = [], []
        self.prior_img = np.zeros((480, 480))




        if self.full_world_plot_bool:
            plt.figure()
            self.ax = plt.axes(projection='3d')
            #self.ax.view_init(elev=10, azim=10)

            self.connections = [(0, 1), (1, 2), (2, 3), (3, 7), (0, 4),
                                (4, 5), (5, 6), (6, 8), (9, 10), (11, 12),
                                (11, 13), (13, 15), (15, 17), (17, 19),
                                (19, 15), (15, 21), (12, 14), (14, 16),
                                (16, 18), (18, 20), (20, 16), (16, 22),
                                (11, 23), (12, 24), (23, 24), (23, 25),
                                (24, 26), (25, 27), (26, 28), (27, 29),
                                (28, 30), (29, 31), (30, 32)]




    def get_landmarks(self, results, img):  # returns a list of points for which cx and cy for certain landmarks

        #RESET THE COORDINATE LISTS
        cx_list, cy_list, cz_list =  [], [] ,[]
        ogimg = img

        #IF A PERSON IS IN FRAME
        if results.pose_landmarks:

            # print only in th edegbuiuging mod ecaleld erbose
            if self.verbose: print("human found")

            #DRAW LANDMARKS ON HUMAN ON IMAGE THAT OUTLINES THE BODY
            self.mpDraw.draw_landmarks(img, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS,
                                       self.mpDraw.DrawingSpec(color=(55,175,212), thickness=3, circle_radius=2),
                                       self.mpDraw.DrawingSpec(color=(1,0,139), thickness=4, circle_radius=2))

            '''
                            DEPRACATED
            
            if self.full_world_plot_bool:
                #coinfigure plot
                self.ax.clear()
                unit = 2
                self.ax.set_xlim([0, unit])
                self.ax.set_ylim([0, 1])
                self.ax.set_zlim([0, unit])



                #Make 3d plot of the world landmarks
                self.mpDraw.plot_landmarks(self.ax,results.pose_world_landmarks, connections= self.connections)
                
                            DEPRACATED

                
                '''

            #LOOPS OVER A ENUMERATED SET OF POSE LANDMARKS
                #THE landmark, lm, has x and y values which define the landmarks ratio wrt the shape of img
            for id, lm in enumerate(results.pose_landmarks.landmark):
                #GET SHAPE OF IMAGE MATRIX
                h, w, c = img.shape
                #GET COORDINATE FOR LANDMARK ID
                cx, cy, cz = float(lm.x * w), float(lm.y * h), float(lm.z * c)

                #APPEND THE COORDINATE TO COORDINATE LISTS
                cx_list.append(float(cx))
                cy_list.append(float(cy))
                cz_list.append(float(cz))
            return cx_list, cy_list, cz_list, ogimg

        else:
            print("No human")
            return [],[],[], ogimg

    def get_hand_landmarks(self, og_image, image):

        #Hands should probrally only be used for camera at end effector
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(static_image_mode=False,
                               max_num_hands=2,
                               min_tracking_confidence=0.3,
                               min_detection_confidence=0.7)
        results = hands.process(cv2.cvtColor(og_image, cv2.COLOR_BGR2RGB))
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                hand_coord_xyz = [results.multi_hand_landmarks[i].landmark for i in range(len(
                                                results.multi_hand_landmarks))]

            return hand_coord_xyz

        else:
            return None



    def get_full_world_landmarks(self, results):
        landmarks = []
        for landmark in results.pose_world_landmarks.landmark:
            landmarks.append([landmark.x, landmark.y, landmark.z])

            return landmarks


    def plot_full_world_coordinates(self, coords):

        cx_list, cy_list, cz_list = coords
        index_combinations = self.connections

        self.ax.clear()
        self.ax.scatter(cx_list, cy_list, cz_list)

        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')
        self.ax.set_zlabel('Z Label')
        elev, azim = self.get_elev_azim([11, 19], coords)
        self.ax.view_init(elev=-elev, azim=azim)

        unit = 600
        self.ax.set_xlim([-unit, unit])

        for combination in index_combinations:
            x = [cx_list[combination[0]], cx_list[combination[1]]]
            y = [cy_list[combination[0]], cy_list[combination[1]]]
            z = [cz_list[combination[0]], cz_list[combination[1]]]
            self.ax.plot(x, y, z, 'r' )
        plt.draw()
        plt.pause(0.001)


    def get_elev_azim(self, indexes, coords):

        p1, p2 = indexes
        cx_list, cy_list, cz_list = coords
        x1, y1, z1 = cx_list[p1], cy_list[p1], cz_list[p1]
        x2, y2, z2 = cx_list[p2], cy_list[p2], cz_list[p2]
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        r = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        elev = math.asin(dz / r)
        azim = math.atan2(dy, dx)
        return math.degrees(elev), math.degrees(azim)

    #RETURNS IMAGE WITH SKELOTON DRAWN AND MAKES COORDINATE LISTS UPDATED BY ATTRIB REF
    def imageProcessing(self):

        #READ IMAGE
        success, img = self.cap.read()

        # Pose detection algorith works in (red,green,bue) while cv library works in BGR, thus we need to convert.
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  #


        print(imgRGB.shape)

        if self.verbose:
            square_rgb = self.rgb_to_matrix(imgRGB)
            print(square_rgb.shape)
            self.eigVal, self.eigVec = np.linalg.eig(square_rgb)

        # Now we Process the pose in the new color formatting. Is a class that contains the pose solutions
        results = self.pose.process(imgRGB)

        # Find coordinates of user (X,Y,Z)
        cx_list, cy_list, cz_list, ogimg = self.get_landmarks(results, img)

        # Hand landmark preditions
        hand_coords = np.array(self.get_hand_landmarks(ogimg, img))


        return ogimg, success, [cx_list, cy_list, cz_list], hand_coords

    def rgb_to_matrix(self, img):

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


        square = cv2.resize(gray, (480, 480))
        self.prior_img = square

        square =  (square**3  + self.prior_img**2)

        cv2.imshow("w", square)

        return square



class MultiVariableCalculusSolver:

    def __init__(self, conf_x = 50, conf_y = 50):

        # TOD0: MAKE VARIABLE CONFIDENCE FOR CLICK FLAG RELATED TO DISTANCE FORM CAMERA
        #coordinate variance confidences for clickflag
        self.conf_x = conf_x
        self.conf_y = conf_y

    def vector_mag(self, vector):

        # give the sum an initical condition
        sum = 0

        # for each vector component, square it and addd sum to the total sum
        for vector_component in vector:
            # compute the squares
            squared = vector_component**2
            # compute the sum
            sum = sum + squared

        # take squart root of the sum
        magn = np.sqrt(sum)
        return magn
    # this function returns the magutide of any vector given vector components

    def get_elev_azim(self, indexes, coords):

        #unpacking
        p1, p2, = indexes
        cx_list, cy_list, cz_list = coords
        x1, y1, z1 = cx_list[p1], cy_list[p1], cz_list[p1]
        x2, y2, z2 = cx_list[p2], cy_list[p2], cz_list[p2]


        #compute deltas
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        #compute vector
        vec = np.array([dx, dy, dz])
        #convert vector to polar coordinates
        r = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        elev = np.acos(dz / r)
        azim = np.atan2(dy, dx)

        return math.degrees(elev), math.degrees(azim)
    #Given body coordinates, generates vector between 2 indexed points, and computes orietnations


    def compute_userRotation(self, coords):

        ''' This is a fucntion tha tcomputes the cusers orentation with
        respect to their body when facing the vcamer. the treturned angle is usperspositon ed with the
        anle form the arm with respect tto its oewm body and sent over the serial bus to
        the arudino in th efirst index of the string unpacked and parsed. after it does so the
        arudino will rigjt he angle to the servo motor with a s curv esmoothing.

        If i recall correctly the hip index coorfiantes in the coordinates
        list are 23,24, double check this'''

        #UNpak coordinates
        cx_list, cy_list, cz_list = coords

        #Calculate center coordinates for hips, this is done by comuptig th eaverage coordinate for ach dimaention
        center_coords = np.array([
            (cx_list[23] + cx_list[24])/2,# center x
            (cy_list[23] + cy_list[24])/2,# center y
            (cz_list[23] + cz_list[24])/2 # center z
        ])

        # Now make a triangle witht the average coordinate for all coorinates
        # Goal is to calcuate the pangle bewtween the ohoriz
        # define coordates for the horizontal z- axis
        left_hip =  np.array([cx_list[23], cy_list[23], cz_list[23]])

        # define the vector from the center coordinate to the hip coordiante
        # as if the person was facing the camera directly
        left_horizontal = [cx_list[23], cy_list[23], 0]

        # compute horizontal vector
        horizontal_vector = center_coords - left_horizontal

        # now define a vector dform the center to your actual left hip
        hip_vector = center_coords - left_hip

        # define ht e the dot product between the hip vveoer and th ehorizontal vector
        hip_horizontal_dot_product = np.dot(hip_vector, horizontal_vector)

        # form the formula  A dot B = mag(A) * mag(B) * cos( theta)
        # we can derive theta = arccos( (A dot B ) / ( mag(A) * mag(B) )
        # this will giv eus the angle between the two vecotr defines
        # this will do the computation in radtians
        theta_rad = np.arccos(hip_horizontal_dot_product/(
                          self.vector_mag(hip_vector)*self.vector_mag(horizontal_vector)))
        # but we want to return degrees
        theta_deg = np.rad2deg(theta_rad)

        print(f" the theta between the vectors is {theta_deg}")

        return theta_deg


    def clickFlag(self, coords, p1=12, p2=22):

        '''Function determies a boolena value for if 2 landmarks
        are considered to be touching eachother by vector coordinate
        confidences definesd in attributes'''

        #unpacking
        cx_list, cy_list, cz_list = coords
        xList = cx_list
        yList = cy_list
        try:
            if abs(xList[p2] - xList[p1]) < self.conf_x and abs(
                    yList[p2] - yList[p1]) < self.conf_y:  # if click defined by p1 and p2 nearness
                click_flag = True  # Click is true
            else:
                click_flag = False  # Click is false

            return click_flag
        except IndexError as IE:
            print(IE, "IN CLICK FLAG METHOD")
            return False







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
        cx, cy, cz = coords
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
    #ax.set_ylim([-unit, 1.2])
    #ax.set_xlim([150, 700])



    plt.pause(0.01)

    return recent_coords







#todo======
# Make 3D computer vision system
def main():

    ''' This is a function for  finding the coordinates form body and
    computing a z offset, this is only a test function. and isnt implemented
    into any modeules yet.'''

    #INIT COMPUTER VISION CLASS
    visionc = ComputerVision(False, False)

    # inti the multivaribale clauclus solver
    multi_variableCalculus = MultiVariableCalculusSolver(50, 50)

    #INIT DepthLocalizer class
    dl = DepthLocalizer()

    #Plot configurations
    fig, ax = plt.subplots(ncols =2, nrows=1)

    #Inicial conditions
    recent_coords = np.empty((2, 0))
    start = time.time()
    i = 0

    while True:
        i = i + 1

        #return body coordinates and image
        img, success, coords, hand_coords = visionc.imageProcessing()
        cx_list, cy_list, cz_list = coords

        #compute z coordiante approzimation
        dist = dl.generate_zCoords(coords) *600
        #Add z coord offset to the predicted  coords
        cz_list = [ z + dist for z in cz_list]

        # remake the cooridnate vector but use the ne  generated z coordinates
        depth_coords = [cx_list, cy_list, cz_list]

        # return the angle of the use rwith repsec to the camera
        user_theta = multi_variableCalculus.compute_userRotation(depth_coords)


        #TODO: Compute quaternionic oreitnation

        #TODO: Add click flags


        #analyzer_test()
        #plot the hip coordinates from a top view
        recent_coords = plot_topView(depth_coords, recent_coords, ax[0])

        #=============
        # Show the image
        cv2.imshow("disp", img)
        # Resize the window
        cv2.resizeWindow('disp', 600, 600)
        # Interface timing such that it looks like a video
        cv2.waitKey(1)


        end = time.time()
        loopTime = abs(start - end)
        if loopTime >= 30:
            pass





def analyzer_test():

    # Open the file for binary reading
    with open('data.pickle', 'rb') as f:
        # Unpickle the data
        bbox_area_list = pkl.load(f)

    loggedbbox_area_list = [np.log(x)+3*np.sqrt(x) for x,i in enumerate(bbox_area_list)]

    fig, ax = plt.subplots()
    ax.plot(np.linspace(0, 4, len(loggedbbox_area_list)), loggedbbox_area_list)

    plt.show()



if __name__ == "__main__":



    main()













