from DynamicsSolvers import StatsModule
from OpenVIMAv1dot0 import InverseKinematicsModule as Kinematics_module
from OpenVIMAv1dot0 import ComputerVisonModulev1 as Vision_module

import cv2


#SAME AS ARDUINO MAP() FUNCTION
def changeBasis(value, fromLow, fromHigh, toLow, toHigh):
    try:return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
    except ZeroDivisionError as ZERO:
        print(ZERO)
        return 0




def handTracker():



    class process:

        def __init__(self):



            self.ik_limits = [[-200, 200], [-200, 200], [-200, 200], [-6, 6], [-6, 6], [-6, 6]]

            #Init kinematics module and ComputerVison Module
            self.IK = Kinematics_module.InverseKinematics(self.ik_limits)
            self.FK = Kinematics_module.ForwardKinematics(self.ik_limits[0])
            self.eye = Vision_module.ComputerVision()
            self.Fusor = StatsModule.MultiCoordinateFusion(self.ik_limits)

            #Run main
            self.main()



        def main(self):


            while True:

                try:

                    # call computer vision process
                    img, body_coords, hand_coords = self.eye.imageProcessing()


                    # Create vector for hand
                    hand_vec = [hand_coords[0][12].x - hand_coords[0][0].x,
                                hand_coords[0][12].y - hand_coords[0][0].y,
                                hand_coords[0][12].z - hand_coords[0][0].z]


                    #Compute Euler Angles
                    roll, pitch, yaw = self.IK.vector_to_roll_pitch_yaw(hand_vec)


                    #Compute end effector destination
                    x = body_coords[0][19]
                    y = body_coords[1][19]
                    z = body_coords[2][19]

                    # Define a destination in (x,y,z, pitch, roll, yaw)
                    destination = [x,
                                   y,
                                   z,
                                   roll, pitch, yaw]


                    #Running Stats
                    fused_destination = self.Fusor.fuse_coordinates(destination)










                    #Send to kinematics
                    requestList = self.IK.main(destination)
                    self.FK.main(requestList)

                    cv2.imshow("frame", img)
                    cv2.waitKey(1)



                except ValueError as VE:
                    print("NO HANDS", VE)
                except IndexError as IE:
                    print("NO HANDS", IE)
                except TypeError as TE:
                    print("Probs no biggy ", TE)

    idfk_anymore = process()

#todo make a running stats funcitn with this archtechture
handTracker()