
'''
=============================================================

/ Engineer:   Pacheck Nicholas
// Date Created: 11/16/23

// Module Description:

// TODO:

1. Add baud rate, zfill, and port communications to insure compatibility across devices
2. embed the self.conenciton into the status bits



// Modifications:
    Date:    Engineer:     Modification Description

=============================================================
'''


import SerialSenderNOV as SerialModule
import ComputerVisonModulev1 as eye
import InverseKinematicsModule as kin
import cv2



if __name__ == "__main__":

    #init computer vision
    simon = eye.MimicController()
    simon.Vision.USE_MULTITHREADING = True






    while True:


        # CV returns forward and inverse kinematics
        img, fk, ik = simon.top_main()
        print("FK: ", fk, "IK", ik)

        # Show the image
        cv2.imshow("CV", img)
        # Interface timing such that it looks like a video
        cv2.waitKey(1)



