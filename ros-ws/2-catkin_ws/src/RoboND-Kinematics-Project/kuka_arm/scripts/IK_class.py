#!/sr/bin/env python

import numpy as np
from sympy import atan2, cos, latex, pi, pprint, simplify, sin, sqrt, symbols, acos
from sympy.matrices import Matrix
import tf.transformations as tf
import mpmath as mp
from FK import rot_m, trsl_m, Tmatrix, invT, rpy_m, getWC
from FK_class import FK, DH


def cosLaw_cosC(A, B, C):
    '''Return de cosine of angle(C) of a ABC triangle by Cosine Law.'''
    return ((A ** 2 + B ** 2 - C ** 2) / (2. * A * B))


def hypotenusePita(A, B):
    '''Returns the hypotenuse of a rectangle triangle of legs A and B'''
    return sqrt(A ** 2 + B ** 2)


def rad2deg(x):
    '''Return the degree value of x in radians.'''
    return x * 180 / np.pi


# %%
class IK(FK):
    roll = symbols('roll')
    pitch = symbols('pitch')
    yaw = symbols('yaw')

    def __init__(self):
        self.wc = []
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        self.Rcor = self.rpy_m(pi, -pi / 2, 0)
        self.Rrpy = self.rpy_m(self.roll, self.pitch, self.yaw) * self.Rcor

    def get_Rrpy(self, r, p, y):
        '''Return the numerical Rotation matrix substituting the roll, pitch, yaw
        angles.

        Parameters:

        - *r*: roll angle, angle of rotation measured along axis X.

        - *p*: pitch angle, angle of rotation measured along axis Y.

        - *y*: yaw angle, angle of rotation measured along axis Z.
        '''
        self.Rrpy = self.Rrpy.subs({self.roll: r, self.pitch: p, self.yaw: y})
        return self.Rrpy

    def get_wc(self, px, py, pz, Rrpy=None, l=0.303):
        '''Return the wrist center position given the end-effector pose.

        Parameters:
        - *px*: X component of end-effector position.

        - *py*: Y component of end-effector position.

        - *pz*: Z component of end-effector position.
        -
        - *Rrpy*: Rotation Matrix that define the end-effector orientation.
        -
        - *l*: Distance of the end-effector from the WC.
        '''
        if Rrpy is None:
            Rrpy = self.Rrpy
        self.wc = np.array([[px], [py], [pz]]) - l * np.array(Rrpy[:3, 2])
        self.wc = self.wc.T[0]
        return self.wc

    def get_Theta1(self, wc=None):
        '''Return the two possible values of Theta 1 given the wrist center (wc)
        '''
        if wc is None:
            wc = self.wc

        theta = float(atan2(wc[1], wc[0]).evalf())
        if theta >= 0:
            return [theta, theta - np.pi]
        else:
            return [theta, theta + np.pi]

    def get_Theta2_3(self, wc=None, wk=0, cfg=1):
        '''Return the two possible values of Theta 2 and Theta 3 given the wrist
         center (wc).

        Parameters:

         - *wc*: wrist center position.

         - *wk*: working position (0:frontware-1:backward).

         - *cfg*: elbow configuration (0:down-1:up).
        '''
        if wc is None:
            wc = self.wc

        # Get x, y proyected coords
        Q = []
        projected = hypotenusePita(wc[0], wc[1])
        x = (projected - 0.35,  # worikng a head
             projected + 0.35)  # workin backward
        x = x[wk]
        y = wc[2] - 0.75
        # Define sides of triangle
        A = 1.501
        C = 1.25
        # get the Cos of the angles
        B = hypotenusePita(x, y)
        cosA = cosLaw_cosC(B, C, A)
        cosB = cosLaw_cosC(C, A, B)
        # get the ineers angles
        if cosA >= 1:
            cosA = 1 * mp.sign(cosA)
        if abs(cosB) >= 1:
            cosB = 1 * mp.sign(cosB)

        a = atan2(mp.sqrt(1 - cosA ** 2), cosA)
        b = atan2(mp.sqrt(1 - cosB ** 2), cosB)

        # get the complementary angles
        alpha = atan2(y, x)
        beta = 0.036

        # Solve Theta2 and Theta3
        if wk:
            theta2 = -(np.pi / 2 - alpha - a)
            theta3 = -(np.pi / 2 - b - beta)
            theta3_ = -(np.pi - theta3 + 2 * beta)
            theta2_ = -(np.pi / 2 - alpha + a)
            if cfg:
                # cfg 1: elbow up
                Q = [theta2.evalf(), theta3_.evalf()]
            else:
                # cfg 2: elbow down
                Q = [theta2_.evalf(), theta3.evalf()]
        else:
            theta2 = np.pi / 2 - alpha - a
            theta3 = np.pi / 2 - b - beta
            theta2_ = np.pi / 2 - alpha + a
            theta3_ = -(np.pi + theta3 + 2 * beta)
            if cfg:
                # cfg 1: elbow up
                Q = [theta2.evalf(), theta3.evalf()]
            else:
                # cfg-2: elbow down
                Q = [theta2_.evalf(), theta3_.evalf()]
        return Q

    def get_orientation(self, Qwc, T3_0, Rrpy=None):
        if Rrpy is None:
            Rrpy = self.Rrpy
        q1, q2, q3, q4, q5, q6 = symbols('q1:7')
        joints = {}
        joints = {q1: Qwc[0], q2: Qwc[1], q3: Qwc[2]}
        R3_6 = T3_0.evalf(subs=joints) * Rrpy
        R3_6 = np.round(np.float64(np.array(R3_6)), 5)
    #    print R3_6
        theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
        theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
        theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
    #        print theta4,theta5,theta6
        joints.update({q4: theta4, q5: theta5, q6: theta6})
#        Qf.append(joints)
        return (theta4, theta5, theta6)

# %%


if __name__ == "__main__":
    # Symbolics variables definition
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    # Calculate de sides length

    test_cases = {1: [[[2.16135, -1.42635, 1.55109],
                       [0.708611, 0.186356, -0.157931, 0.661967]],
                      [1.89451, -1.44302, 1.69366],
                      [-0.65, 0.45, -0.36, 0.95, 0.79, 0.49]],
                  2: [[[-0.56754, 0.93663, 3.0038],
                       [0.62073, 0.48318, 0.38759, 0.480629]],
                      [-0.638, 0.64198, 2.9988],
                      [-0.79, -0.11, -2.33, 1.94, 1.14, -3.68]],
                  3: [[[-1.3863, 0.02074, 0.90986],
                       [0.01735, -0.2179, 0.9025, 0.371016]],
                      [-1.1669, -0.17989, 0.85137],
                      [-2.99, -0.12, 0.94, 4.06, 1.29, -4.12]],
                  4: [[[-0.47958, 0.69824, 3.3771],
                       [0.67747, 0.4606, 0.56252, 0.11163]],
                      [-0.46227, 0.47109, 3.1773],
                      [2.35, -0.4, -0.65, -1.23, 0.54, 2.72]],
                  5: [[[0.020265, -0.2279, 3.2806],
                       [0.43272, -0.71555, -0.035909, 0.54722]],
                      [0.028327, -0.028356, 3.0527],
                      [-0.79, -0.76, -0.47, -1.23, 0.54, 2.72]],
                  6: [[[-0.71977, 0.71957, 3.3389],
                       [0.35118, 0.848, 0.15188, -0.36675]],
                      [-0.57302, 0.57286, 3.1181],
                      [-0.79, -0.14, -2.19, -0, 0, 0]]}
    dh_kuka = DH()
    fk_kuka = FK(dh_kuka)
    ik_kuka = IK()
    n = 1
    pt = test_cases[n][0][0]
    rpy = tf.euler_from_quaternion(test_cases[n][0][1])
    ik_kuka.get_Rrpy(*rpy)
    wc = ik_kuka.get_wc(pt[0], pt[1], pt[2])
#    wc = ik_kuka.wc = [-0.513038549472886, 2.49994199750433, 1.59970494816823]
    tt1 = ik_kuka.get_Theta1()
    tt2_3 = ik_kuka.get_Theta2_3(wk=0)
    print 'Wrist center: '
    print ik_kuka.wc
    print test_cases[n][1]
    print 'Thetas'
    print 'q1: ', tt1, 'q2: ', tt2_3[0], 'q3: ', tt2_3[1]
    print test_cases[n][2]

    T3_0 = fk_kuka.invT(fk_kuka.T0_3)
    print ik_kuka.get_orientation((tt1[0], tt2_3[0], tt2_3[1]), T3_0)
#    fk_kuka.verify_rich_wc(min(tt1),tt2_3[0],tt2_3[1],wc)
    print fk_kuka.verify_rich_wc(-0.65, 0.45, -0.36, wc)
    # TODO: Multiple solution orientation
