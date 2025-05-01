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



    #returns the landmark cx, cy, cz coordinates from the usershand

    # does either threaded or non threaded image processing, returning the img, hand and body coordinates of the user
    def imageProcessing(self, threaded = None):


        if threaded is not None: self.USE_MULTITHREADING = threaded

        if self.USE_MULTITHREADING:
            img, body_coords, hand_coords = self.threaded_imageProcessing()
        else: # dont use multithreading
            img, body_coords, hand_coords = self.unthreaded_imageProcessing()
        return img, body_coords, hand_coords

    #returns the original image taken, body coordinates, and hand corodinares
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

    # Starts two threads and performs hand image processing in parrallel to body image processing
    def threaded_imageProcessing(self):
        # seperartes the hands and body threads to seperate threads

        # READ IMAGE
        success, img = self.cap.read()
        # resize image for useable basis
        cv2.resize(img, (800, 600))
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

    # returns a list of points for which cx, cy, and cZ for body landmarks
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
            return [],[],[], ogimg

    def get_hand_landmarks(self, og_image, image):


        returnThis = [0,0,0,0,0,0,0,0]


        #Hands should probrally only be used for camera at end effector
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_tracking_confidence=.3)
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





#todo======
# Make 3D computer vision system
def main_threaded(count):
    # init mimic controler
    eye = ComputerVision(threaded=True)

    i = 0
    start_time = time.time()
    try:
        while i < count:
            # everything
            img, fk, ik  = eye.imageProcessing()


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
    eye = ComputerVision(threaded=False)
    i=0
    start_time = time.time()
    try:
        while i<count:

            # everything
            img, thetaListFK, thetaListIK = eye.imageProcessing()

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



if __name__ == "__main__":


    count = 50
    threaded_time = main_threaded(count)
    standard_time = main_no_threads(count)

    print(f"threaded average cycle: {threaded_time/count}"
          f"standard average cycle: {standard_time/count} "
          f"")
