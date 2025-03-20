
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


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from random import randint
import math
#import KEYBOARD_CONTROLLER2

# make numpy raise errors instead of warn so can be caught by try except blocks
np.seterr(all="raise")





class InverseKinematics:
    def __init__(self, ik_limits):
        pass





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

        #self.controller = KEYBOARD_CONTROLLER2.Keyboard_Controller(ik_limits, self.step_size)




    # plot axis origins onto point with rotation
    def plotOrigin(self, tx, ty, tz, rx, ry, rz, length):
        # transform origin
        rotall = np.matmul(np.matmul(self.Rz(rz), self.Ry(rx)), self.Rz(rx))
        xplots = np.matmul(rotall, ((0, length), (0, 0), (0, 0)))
        yplots = np.matmul(rotall, ((0, 0), (0, length), (0, 0)))
        zplots = np.matmul(rotall, ((0, 0), (0, 0), (0, length)))
        xplots[0] += tx
        xplots[1] += ty
        xplots[2] += tz
        yplots[0] += tx
        yplots[1] += ty
        yplots[2] += tz
        zplots[0] += tx
        zplots[1] += ty
        zplots[2] += tz

        # plot origin
        #self.ax.plot(*xplots, c="#ff0000")
        #self.ax.plot(*yplots, c="#00ff00")
        #self.ax.plot(*zplots, c="#0000ff")


    def findEndEffector(self,tx, ty, tz, rx, ry, rz, length):
        # transform line
        rotall = np.matmul(np.matmul(self.Rz(rz), self.Ry(ry)), self.Rx(rx))
        plots = list(np.matmul(rotall, (0, 0, length)))
        plots[0] += tx
        plots[1] += ty
        plots[2] += tz
        return plots


    # define rotation matrices
    def Rx(self,a):
        return (
            (1, 0, 0),
            (0, np.cos(a), -np.sin(a)),
            (0, np.sin(a), np.cos(a))
        )


    def Ry(self,a):
        return (
            (np.cos(a), 0, np.sin(a)),
            (0, 1, 0),
            (-np.sin(a), 0, np.cos(a))
        )


    def Rz(self,a):
        return (
            (np.cos(a), -np.sin(a), 0),
            (np.sin(a), np.cos(a), 0),
            (0, 0, 1)
        )


    # functions to find angle between two vectors
    def getAngle(self,vect1, vect2):
        unitvect1 = vect1 / np.linalg.norm(vect1)
        unitvect2 = vect2 / np.linalg.norm(vect2)
        theta = np.arccos(
            np.clip(
                np.dot(unitvect1, unitvect2),
                -1.0,
                1.0
            )
        )
        return theta


    # project a 3d point onto a 3d plane
    def pointToPlane(self, point, origin, normal):
        unitnormal = normal / np.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2)
        vector = point - origin
        distance = np.dot(vector, unitnormal)
        projectedpoint = point - distance * unitnormal
        return projectedpoint


    # linear interpolation
    def lerp(self,a, b, t):
        return (a * (1 - t)) + (b * t)


    def invKin(self,lengths, destination):
        # define parameters
        length1 = lengths[0]
        length2 = lengths[1]
        length3 = lengths[2]
        length4 = lengths[3]
        length5 = lengths[4]
        length6 = lengths[5]

        # define other
        error = False

        xplots = [0, 0]
        yplots = [0, 0]
        zplots = [0, length1]

        length2 = length2
        length34 = length3 + length4
        length56 = length5 + length6

        desttx = destination[0]
        destty = destination[1]
        desttz = destination[2]
        destrx = destination[3]
        destry = destination[4]
        destrz = destination[5]

        # create one large rotation matrix
        rall = np.matmul(
            np.matmul(
                self.Rz(destrz),
                self.Ry(destry)
            ),
            self.Rx(destrx)
        )

        # define points to be transformed
        act5pos = [[0.0, float(-length56)], [0.0, 0.0], [0.0, 0.0]]

        # rotate position
        act5newpos = np.matmul(rall, act5pos)

        # translate potitions
        act5pos[0][0] += desttx
        act5pos[0][1] += desttx
        act5pos[1][0] += destty
        act5pos[1][1] += destty
        act5pos[2][0] += desttz
        act5pos[2][1] += desttz
        act5newpos[0][0] += desttx
        act5newpos[0][1] += desttx
        act5newpos[1][0] += destty
        act5newpos[1][1] += destty
        act5newpos[2][0] += desttz
        act5newpos[2][1] += desttz

        # find angle between act5newpos and the xz plane
        joint5toorigin = (
            (0.0, act5newpos[0][1]),
            (0.0, act5newpos[1][1]),
            (act5newpos[2][1], act5newpos[2][1]),
        )

        # cannot divide by zero so catch and replace with real value
        try:
            angle = np.arctan(
                (joint5toorigin[1][1] - joint5toorigin[1][0])
                / (joint5toorigin[0][1] - joint5toorigin[0][0])
            )
        except FloatingPointError:
            angle = np.pi / 2

        # rotate act5newpos to the xz plane
        act5xzpos = list(map(list, np.matmul(self.Rz(-angle), joint5toorigin)))

        # define target position for 2D inverse kinematics (xz (z-up) plane now referred as xy (y-up) plane)
        targetpos = [act5xzpos[0][1], act5xzpos[2][1]]

        # perform inverse kinematics on the target position
        if targetpos[0] < 0:
            targetpos[0] *= -1
            angle += np.pi

        # inverse kinematics for angles 1, 2, and 3
        try:
            theta1 = np.arccos(
                (
                    length2 ** 2
                    + targetpos[0] ** 2
                    + (targetpos[1] - length1) ** 2
                    - length34 ** 2
                )
                / (2 * length2 * np.sqrt(targetpos[0] ** 2 + (targetpos[1] - length1) ** 2))
            )
            theta2 = np.pi - np.arccos(
                (
                    length2 ** 2
                    + length34 ** 2
                    - (targetpos[0] ** 2 + (targetpos[1] - length1) ** 2)
                )
                / (2 * length2 * length34)
            )
            theta3 = np.arctan((targetpos[1] - length1) / targetpos[0])
        except FloatingPointError:
            return None

        # angles saved to sensible variables
        angle1 = -angle if angle <= np.pi else (np.pi * 2) - angle
        angle2 = theta1 + theta3 - np.pi / 2
        angle3 = theta2

        # get plot positions from calculated angles
        xplots.append(-np.sin(angle2) * length2)
        yplots.append(0)
        zplots.append((np.cos(angle2) * length2) + length1)
        xplots.append(self.lerp(xplots[-1], targetpos[0], length3 / length34))
        yplots.append(0)
        zplots.append(self.lerp(zplots[-1], targetpos[1], length3 / length34))
        xplots.append(targetpos[0])
        yplots.append(0)
        zplots.append(targetpos[1])

        allplots = list(map(list, np.matmul(self.Rz(angle), [xplots, yplots, zplots])))
        xplots = allplots[0]
        yplots = allplots[1]
        zplots = allplots[2]
        xplots.append(self.lerp(act5newpos[0][1], desttx, length5 / length56))
        yplots.append(self.lerp(act5newpos[1][1], destty, length5 / length56))
        zplots.append(self.lerp(act5newpos[2][1], desttz, length5 / length56))
        xplots.append(desttx)
        yplots.append(destty)
        zplots.append(desttz)

        # angle 4 calculated by projecting plots between arm 2 and arm 5-6 onto plane with origin and normal of the forearm
        origin4 = np.array([xplots[2], yplots[2], zplots[2]])
        normal4 = np.array([xplots[4], yplots[4], zplots[4]]) - origin4
        point41 = self.pointToPlane(
            np.array([xplots[1], yplots[1], zplots[1]]), origin4, normal4
        )
        point42 = self.pointToPlane(
            np.array([xplots[6], yplots[6], zplots[6]]), origin4, normal4
        )
        vector41 = point41 - origin4
        vector42 = point42 - origin4
        point42direction = -1 if np.matmul(self.Rz(angle1), point42)[1] >= 0 else 1
        try:
            angle4 = (np.pi - self.getAngle(vector41, vector42)) * point42direction
        except:
            angle4 = 0

        # angle 5 calculated by taking plots 2, 4, and 6
        vector51 = (xplots[2] - xplots[4], yplots[2] - yplots[4], zplots[2] - zplots[4])
        vector52 = (xplots[6] - xplots[4], yplots[6] - yplots[4], zplots[6] - zplots[4])
        angle5 = np.pi - self.getAngle(vector51, vector52)

        # angle 6 calculated by projecting plots between arm 3 and end effector onto plane with origin and normal of the wrist
        origin6 = np.array([xplots[4], yplots[4], zplots[4]])
        normal6 = np.array([xplots[6], yplots[6], zplots[6]]) - origin6
        point61 = self.pointToPlane(
            np.array([xplots[2], yplots[2], zplots[2]]), origin6, normal6
        )
        point62 = self.pointToPlane(
            self.findEndEffector(desttx, destty, desttz, destrx, destry, destrz, 50),
            origin6,
            normal6,
        )
        vector61 = point61 - origin6
        vector62 = point62 - origin6
        point62direction = 1 if np.matmul(self.Rz(angle1), point42)[1] >= 0 else -1
        try:
            angle6 = self.getAngle(vector61, vector62) * point62direction
        except:
            angle6 = 0

        return (
            (-angle1, -angle2, angle3, angle4, angle5, angle6),
            tuple(xplots),
            tuple(yplots),
            tuple(zplots),
        )


    def main(self, destination):


        lengths = [50, 200, 100, 100, 25, 25]
        #self.ax.clear()

        raw = self.invKin(lengths, destination)
        if raw is not None:
            angle1 = raw[0][0]
            angle2 = raw[0][1]
            angle3 = raw[0][2]
            angle4 = raw[0][3]
            angle5 = raw[0][4]
            angle6 = raw[0][5]
            requestList = [angle1, angle2, angle3, angle4, angle5, angle6]
            xplots = raw[1]
            yplots = raw[2]
            zplots = raw[3]

            requestList = np.rad2deg(requestList)



            return requestList
        else: return None

ik_limits = [[-200, 200], [-200, 200], [-200, 200], [-6, 6], [-6, 6], [-6, 6]]





# make numpy raise errors instead of warn so can be caught by try except blocks
np.seterr(all="raise")
hover = "#CF4420"

plt.ion()

class ForwardKinematics:

    def __init__(self, limits):
        #self.vision = pacheckCV.ComputerVisionBody(False)


        # Create a new figure and add a 3D subplot
        self.fig = plt.figure(figsize=(10,8)) #stable

        plt.style.use('dark_background')

        plt.subplots_adjust(left=0,
                            bottom=-0.7,
                            right=0.917,
                            top=1,
                            wspace=0.706,
                            hspace=0.5)
        #DEV
        #self.fig = Figure(figsize=(20,16))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.limits = limits

        pass

    # plot axis origins onto point with rotation
    def plotOrigin(self, tx, ty, tz, rx, ry, rz, length):
        # transform origin
        rotall = np.matmul(np.matmul(self.Rz(rz), self.Ry(ry)), self.Rx(rx))
        xplots = np.matmul(rotall, ((0, length), (0, 0), (0, 0)))
        yplots = np.matmul(rotall, ((0, 0), (0, length), (0, 0)))
        zplots = np.matmul(rotall, ((0, 0), (0, 0), (0, length)))
        xplots[0] += tx
        xplots[1] += ty
        xplots[2] += tz
        yplots[0] += tx
        yplots[1] += ty
        yplots[2] += tz
        zplots[0] += tx
        zplots[1] += ty
        zplots[2] += tz

        # plot origin
        self.ax.plot(*xplots, c="#ff0000")
        self.ax.plot(*yplots, c="#00ff00")
        self.ax.plot(*zplots, c="#0000ff")

    # find the position of point given a vector
    def findPoint(self, start, length, theta1, theta2):
        end = [0, 0, 0]
        end[0] = start[0] + length * np.cos(theta1) * np.cos(theta2)
        end[1] = start[1] + length * np.cos(theta1) * np.sin(theta2)
        end[2] = start[2] + length * np.sin(theta1)
        return end

    # define rotation matrices
    def Rx(self, a):
        return ((1, 0, 0), (0, np.cos(a), -np.sin(a)), (0, np.sin(a), np.cos(a)))

    def Ry(self, a):
        return ((np.cos(a), 0, np.sin(a)), (0, 1, 0), (-np.sin(a), 0, np.cos(a)))

    def Rz(self, a):
        return ((np.cos(a), -np.sin(a), 0), (np.sin(a), np.cos(a), 0), (0, 0, 1))

    # function to find angle between two vectors
    def getAngle(self, vect1, vect2):
        try:
            temp1 = np.linalg.norm(vect1)
            temp2 = np.linalg.norm(vect2)
            unitvect1 = vect1 / temp1
            unitvect2 = vect2 / temp2
            theta = np.arccos(np.clip(np.dot(unitvect1, unitvect2), -1.0, 1.0))
            return theta
        except Exception:
            return np.pi / 2 if temp2 > 0 else -np.pi / 2

    # linear interpolation
    def lerp(self, a, b, t):
        return (a * (1 - t)) + (b * t)

    def fwdKin(self, lengths, angles):

        try:
            # trace the path to find plot points
            xplots = [0, 0]
            yplots = [0, 0]
            zplots = [0, lengths[0]]
            mainlength1 = lengths[1]
            mainlength2 = lengths[2] + lengths[3]
            mainlength3 = lengths[4] + lengths[5]
            theta1 = (np.pi / 2) - angles[1]
            theta2 = angles[0]
            point = self.findPoint([xplots[1], yplots[1], zplots[1]], mainlength1, theta1, theta2)
            xplots.append(point[0])
            yplots.append(point[1])
            zplots.append(point[2])
            theta1 = (np.pi / 2) - angles[1] - angles[2]
            theat2 = angles[0]
            point = self.findPoint([xplots[2], yplots[2], zplots[2]], mainlength2, theta1, theta2)
            savedpoint = point  # save the point for later derivation
            xplots.append(self.lerp(xplots[2], point[0], lengths[2] / mainlength2))
            yplots.append(self.lerp(yplots[2], point[1], lengths[2] / mainlength2))
            zplots.append(self.lerp(zplots[2], point[2], lengths[2] / mainlength2))
            xplots.append(point[0])
            yplots.append(point[1])
            zplots.append(point[2])
            transformed = np.array([mainlength3, 0, 0])
            transformed = np.matmul(self.Ry(angles[4]), transformed)
            transformed = np.matmul(self.Rx(angles[3]), transformed)
            transformed = np.matmul(self.Ry(np.pi + theta1), transformed)
            transformed = np.matmul(self.Rz(np.pi + theta2), transformed)
            point = transformed + point
            xplots.append(self.lerp(xplots[4], point[0], lengths[4] / mainlength3))
            yplots.append(self.lerp(yplots[4], point[1], lengths[4] / mainlength3))
            zplots.append(self.lerp(zplots[4], point[2], lengths[4] / mainlength3))
            xplots.append(point[0])
            yplots.append(point[1])
            zplots.append(point[2])

            # convert the data type
            xplots = np.array(xplots)
            yplots = np.array(yplots)
            zplots = np.array(zplots)

            # we now know the end position
            tx = xplots[-1]
            ty = yplots[-1]
            tz = zplots[-1]

            # derive theta1 and theta2 of wrist vector
            # define root vectors
            wrstvect = point - savedpoint

            txdelta = wrstvect[0]
            tydelta = wrstvect[1]
            tzdelta = wrstvect[2]

            elevvect = np.array([txdelta, tydelta, 0])
            azimvect = np.array([txdelta, 0, tzdelta])

            elevangle = self.getAngle(elevvect, wrstvect)
            azimangle = self.getAngle(azimvect, wrstvect)

            if tzdelta < 0:
                elevangle *= -1

            if tydelta < 0:
                azimangle *= -1

            rx = angles[5]
            ry = -elevangle
            rz = azimangle

            return ((tx, ty, tz, rx, ry, rz), xplots, yplots, zplots)
        except IndexError as IE:
            print(IE)
            pass

    def main(self, angles):
        angles = np.array(angles)

        self.ax.clear()

        # CONFIGS
        self.ax.xaxis.line.set_color(hover)
        self.ax.yaxis.line.set_color(hover)
        self.ax.zaxis.line.set_color(hover)

        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")

        # SET AXIXS LIMITS SUCH THAT BASIS DOESNT CHANGE FROM FRAME2FRAME
        # self.limits = [-200, 200]
        self.ax.set_xlim(self.limits)
        self.ax.set_ylim(self.limits)
        self.ax.set_zlim(self.limits)

        # self.ax.view_init(elev=30, azim=-135)
        # self.ax.view_init(elev=30, azim=135)

        lengths = np.array([50.0, 200.0, 100.0, 100.0, 25.0, 25.0])

        raw = self.fwdKin(lengths, np.radians(angles))
        tx = raw[0][0]
        ty = raw[0][1]
        tz = raw[0][2]
        rx = raw[0][3]
        ry = raw[0][4]
        rz = raw[0][5]
        xplots = raw[1]
        yplots = raw[2]
        zplots = raw[3]

        # plot data
        self.ax.plot(xplots, yplots, zplots, c= "#630031")


        # update line width of all lines in the figure
        for line in self.ax.get_lines():
            line.set_linewidth(12)

        self.ax.scatter(xplots, yplots, zplots, c=hover)
        self.plotOrigin(xplots[0], yplots[0], zplots[0], 0, 0, 0, 50)
        self.plotOrigin(xplots[6], yplots[6], zplots[6], rx, ry, rz, 50)

        # show plotted data
        plt.draw()
        plt.pause(0.05)


    def main1(self, angles):

        angles = np.array(angles)


        self.ax.clear()

        #CONFIGS
        self.ax.w_xaxis.line.set_color(hover)
        self.ax.w_yaxis.line.set_color("#00ff00")
        self.ax.w_zaxis.line.set_color("#0000ff")



        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")

        #SET AXIXS LIMITS SUCH THAT BASIS DOESNT CHANGE FROM FRAME2FRAME
        #self.limits = [-200, 200]
        self.ax.set_xlim(self.limits)
        self.ax.set_ylim(self.limits)
        self.ax.set_zlim(self.limits)

        # self.ax.view_init(elev=30, azim=-135)
        #self.ax.view_init(elev=30, azim=135)

        lengths = np.array([70.0, 120.0, 100.0, 100.0, 25.0, 25.0])

        raw = self.fwdKin(lengths, np.radians(angles))
        tx = raw[0][0]
        ty = raw[0][1]
        tz = raw[0][2]
        rx = raw[0][3]
        ry = raw[0][4]
        rz = raw[0][5]
        xplots = raw[1]
        yplots = raw[2]
        zplots = raw[3]

        # plot data
        self.ax.plot(xplots, yplots, zplots, c=hover)
        self.ax.scatter(xplots, yplots, zplots, c="#00ffff")
        self.plotOrigin(xplots[0], yplots[0], zplots[0], 0, 0, 0, 50)
        self.plotOrigin(xplots[6], yplots[6], zplots[6], rx, ry, rz, 50)
        #self.fig.canvas.draw()



        # show plotted data
        plt.draw()
        plt.pause(0.5)

    def main2(self, angles):
        angles = np.array(angles)

        # get current axis object and configure plot
        self.ax = plt.gca(projection="3d")
        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")
        self.ax.view_init(elev=30, azim=135)
        self.ax.w_xaxis.line.set_color("#ff0000")
        self.ax.w_yaxis.line.set_color("#00ff00")
        self.ax.w_zaxis.line.set_color("#0000ff")

        lengths = np.array([50.0, 200.0, 100.0, 100.0, 25.0, 25.0])

        raw = self.fwdKin(lengths, np.radians(angles))
        tx = raw[0][0]
        ty = raw[0][1]
        tz = raw[0][2]
        rx = raw[0][3]
        ry = raw[0][4]
        rz = raw[0][5]
        xplots = raw[1]
        yplots = raw[2]
        zplots = raw[3]


        # plot data with updated styles
        self.ax.plot(xplots, yplots, zplots, c="#000000", linestyle='--')

        for line in self.ax.get_lines():
            line.set_linewidth(30)


        self.ax.scatter(xplots, yplots, zplots, c="#00ffff", marker='o')
        self.plotOrigin(xplots[0], yplots[0], zplots[0], 0, 0, 0, 50)
        self.plotOrigin(xplots[6], yplots[6], zplots[6], rx, ry, rz, 50)



        # show updated plot without blocking
        plt.draw()
        #plt.show(block=False)


def foo(i):

    class myclass:
        def __init__(self, i):
            self.i = i
            self.min, self.max = ik_limits[i]
            self.init_pose = self.get_rand_dest()
            self.dtheta = 0.5

        def get_rand_dest(self):
            random_destination = [randint(x, y) for x, y in ik_limits]
            return random_destination

        def actual_foo(self):

            self.init_pose[i] = self.min + self.dtheta

            prev = self.prev + .1
            if prev == ik_limits[i][1]:
                prev = ik_limits[i][0]



    hi = myclass(i)


if __name__ == "__main__":
    IK = InverseKinematics(ik_limits)
    FK = ForwardKinematics([-200, 200])

    i = 5

    while True:





        requestList= IK.main(random_destination)


        print(f" requestList: {requestList}, .. random_destination: { random_destination}")
        FK.main(requestList)
