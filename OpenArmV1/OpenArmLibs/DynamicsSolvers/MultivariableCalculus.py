




import math
import numpy as np
import matplotlib.pyplot as plt


class MultiVar_Calc:

    '''Goal of this module is to easily return end effector points and the orientation to reach them in space'''

    def __init__(self):


        self.hand_coord = []
        self.body_coord = []


        self.centerOfMass = [ 11, 12, 23, 24]

        pass

    def update_coords(self, hand_coords, body_coords):
        self.hand_coord = hand_coords
        self.body_coord = body_coords

    def compute_controlVector(self, hand_coord, body_coords):

        self.update_coords(hand_coord, body_coords)


        bodyZ, handZ = self.generate_zCoords()

        #Vector between pointer finger and wrist
        pointer_vector = np.array(
            [self.hand_coord[0][9].x - self.hand_coord[0][0].x,
             self.hand_coord[0][9].y - self.hand_coord[0][0].y,
             self.hand_coord[0][9].z - self.hand_coord[0][0].z ]
        )

        #Compute Euler angles for pointer finger
        eulerBasis = np.array(self.vector_to_roll_pitch_yaw(pointer_vector))

        # compute center of mass
        cx, cy, cz = self.get_bodyOrigin()


        #compute end effector
        end_effector = np.array(
            [self.hand_coord[0][9].x + self.body_coord[0][19] - cx,
             self.hand_coord[0][9].y + self.body_coord[1][19] - cy,
             self.hand_coord[0][9].z + self.body_coord[2][19] - cz + handZ]
        )

        dest = np.array([end_effector, eulerBasis]).flatten()



        return dest



    #Used to localize vector space origin
    def get_bodyOrigin(self):

        return [np.mean([self.body_coord[ax][i] for i in self.centerOfMass]) for ax in range(3)]
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

    def generate_zCoords(self):

        # compute area for body
        body_area = self.compute_bodyArea(self.body_coord)
        # Normalize data point
        body_z_offset = 100 / (np.sqrt(body_area))


        #compute area for hand
        hand_area = self.compute_handArea()

        #normalize hand area
        hand_z_offset = 100/ np.sqrt(hand_area)


        return body_z_offset, hand_z_offset

    def compute_bodyArea(self, coords):
        # Unpack coordinates
        cx, cy, cz = coords
        # Compute boundary conditions
        max_cx, max_cy, max_cz = max(cx), max(cy), max(cz)
        min_cx, min_cy, min_cz = min(cx), min(cy), min(cz)
        # Compute area of boundary conditions
        bbox_len, bbox_heigh = [(self.body_coord[0][11] - self.body_coord[0][24]),
                               (self.body_coord[1][11] - self.body_coord[1][24])]
        bbox = abs(bbox_len * bbox_heigh)

        print(bbox)

        return bbox

    def seperate_coords(self):

        cx = [self.hand_coord[0][i].x for i in range(len(self.hand_coord[0]))]
        cy = [self.hand_coord[0][i].y for i in range(len(self.hand_coord[0]))]
        cz = [self.hand_coord[0][i].z for i in range(len(self.hand_coord[0]))]

        hand_coords = [cx, cy, cz]
        return hand_coords

    def compute_handArea(self):

        new_shape_handCoords = self.seperate_coords()
        # Unpack coordinates
        cx, cy, cz = new_shape_handCoords
        # Compute boundary conditions
        max_cx, max_cy, max_cz = max(cx), max(cy), max(cz)
        min_cx, min_cy, min_cz = min(cx), min(cy), min(cz)

        # Compute area of boundary conditions
        hbox_len, hbox_heigh = (max_cx - min_cx), (max_cy - min_cy)
        hbox = hbox_len * hbox_heigh

        return hbox





    # Avoids gimble lock, needs new Ik solver to be implelemtned
    def convert_toQuaternions(self):
        raise NotImplementedError