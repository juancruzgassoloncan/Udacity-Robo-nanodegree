#!/usr/bin/env python

import numpy as np
from sympy import atan2, cos, latex, pi, pprint, simplify, sin, sqrt, symbols
from sympy.matrices import Matrix
import tf.transformations as tf


class DH:
    '''Denavit-Hartenberg paramenters.'''

    def __init__(self):
        self.a0 = 0
        self.a1 = 0.35
        self.a2 = 1.25
        self.a3 = -0.054
        self.a4 = 0
        self.a5 = 0
        self.a6 = 0

        self.d1 = 0.75
        self.d2 = 0
        self.d3 = 0
        self.d4 = 1.5
        self.d5 = 0
        self.d6 = 0
        self.d7 = 0.303

        self.alpha0 = 0
        self.alpha1 = -pi / 2
        self.alpha2 = 0
        self.alpha3 = -pi / 2
        self.alpha4 = pi / 2
        self.alpha5 = -pi / 2
        self.alpha6 = 0


class FK(object):

    def __init__(self, DH):
        # Symbolics variables definition
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, \
            alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Symbolics variables definition
        self.DH = {alpha0: DH.alpha0, a0: DH.a0, d1: DH.d1, q1: q1,
                   alpha1: DH.alpha1, a1: DH.a1, d2: DH.d2, q2: q2 - pi / 2,
                   alpha2: DH.alpha2, a2: DH.a2, d3: DH.d3, q3: q3,
                   alpha3: DH.alpha3, a3: DH.a3, d4: DH.d4, q4: q4,
                   alpha4: DH.alpha4, a4: DH.a4, d5: DH.d5, q5: q5,
                   alpha5: DH.alpha5, a5: DH.a5, d6: DH.d6, q6: q6,
                   alpha6: DH.alpha6, a6: DH.a6, d7: DH.d7, q7: 0}

#        # Homogeneous Transforms
        self.T0_1 = self.Tmatrix(alpha0, a0, d1, q1).subs(self.DH)
        self.T1_2 = self.Tmatrix(alpha1, a1, d2, q2).subs(self.DH)
        self.T2_3 = self.Tmatrix(alpha2, a2, d3, q3).subs(self.DH)
        self.T3_4 = self.Tmatrix(alpha3, a3, d4, q4).subs(self.DH)
        self.T4_5 = self.Tmatrix(alpha4, a4, d5, q5).subs(self.DH)
        self.T5_6 = self.Tmatrix(alpha5, a5, d6, q6).subs(self.DH)
        self.T6_EE = self.Tmatrix(alpha6, a6, d7, q7).subs(self.DH)

        # Composition of Homogeneous Transforms
        self.T0_2 = self.T0_1 * self.T1_2  # base to link2
        self.T0_3 = self.T0_2 * self.T2_3  # base to link3
        self.T0_4 = self.T0_3 * self.T3_4  # base to link4
        self.T0_5 = self.T0_4 * self.T4_5  # base to link5
        self.T0_6 = self.T0_5 * self.T5_6  # base to link6
        self.T0_EE = self.T0_6 * self.T6_EE  # base to End-Effector

    def rot_m(self, axis, q):
        '''Rotation Matrix.

        This function returns the general form of a pure rotation in a
        homogeneous transformation matrix around a Cartesian axis.

        Parameters:

         - *axis*: option between the x, y and z axes.

         - *q*: the angle of rotation in radians.'''

        if axis == 'x':
            R = Matrix([[1, 0, 0, 0],
                        [0, cos(q), -sin(q), 0],
                        [0, sin(q), cos(q), 0],
                        [0, 0, 0, 1]])
        elif axis == 'y':
            R = Matrix([[cos(q), 0, sin(q), 0],
                        [0, 1, 0, 0],
                        [-sin(q), 0, cos(q), 0],
                        [0, 0, 0, 1]])
        elif axis == 'z':
            R = Matrix([[cos(q), -sin(q), 0, 0],
                        [sin(q), cos(q), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        else:
            # print 'Wrong axis: x,y,z'
            R = None
        return R

    def trsl_m(self, x, y, z):
        '''Translation Matrix.

        This function returns the general form of a pure translation
        in a homogeneous transformation matrix in the Cartesian space.

        Parameters:

         - *x* : X component.

         - *y* : Y component.

         - *z* : Z component.'''

        T = Matrix([[1, 0, 0, x],
                    [0, 1, 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1]])
        return T

    def Tmatrix(self, alpha, a, d, q):
        '''Return the general form of an homogeneous transformation T by
            definition following the modified DH parameters.

        Paramenters:

         - *alpha*: twist angle, angle between axis Z(i-1) and Z(i)
                    measured about axis X(i-1).

         - *a*: link length, distance from axis Z(i-1) to Z(i)
                measured along axis X(i-1).

         - *d*: link offset, distance from axis X(i-1) to X(i)
                measured along axis Z(i).

         - *q*: joint angle, angle between axis X(i-1) and X(i)
                measured about axis Z(i).
        '''
        T = self.rot_m('x', alpha) * self.trsl_m(a, 0, 0) *\
            self.rot_m('z', q) * self.trsl_m(0, 0, d)
        return T

    def invT(self, T):
        '''Return the inverse form of an homogeneous transformation.
        '''
        R = T[:3, :3]  # Extract the Rotation Matrix.
        D = T[:3, 3]  # Extract the Traslation Vector.
        bottom_row = Matrix([[0, 0, 0, 1]])
        iR = R.T  # Traspose R to get the inverse as it is  orthogonal matrix.
        iD = iR * D  # Apply the the inverse rotation to the traslation vector.
        iT = iR.row_join(iD)
        iT = iT.col_join(bottom_row)
        return iT

    def rpy_m(self, roll, pitch, yaw):
        '''Return a Rotation matrix by the roll, pitch, yaw angles.

        Paramenters:

         - *roll*: roll angle, angle of rotation measured along axis X.

         - *pitch*: pitch angle, angle of rotation measured along axis Y.

         - *yaw*: yaw angle, angle of rotation measured along axis Z.
        '''
        return self.rot_m('z', yaw) *\
            self.rot_m('y', pitch) *\
            self.rot_m('x', roll)

    def verify_rich_wc(self, j1, j2, j3, wc, tol=0.1):

        q1, q2, q3 = symbols('q1:4')

        Q = {q1: j1, q2: j2, q3: j3}
        wc_d = self.T0_5.evalf(subs=Q)[:3, 3]
        e_dist = sqrt(((wc[0] - wc_d[0]) ** 2 +
                       (wc[1] - wc_d[1]) ** 2 +
                       (wc[2] - wc_d[2]) ** 2))
        if round(e_dist, 1) <= tol:
            # print 'Reached, distance: ', e_dist
            # print 'wc_d: ',wc_d
            return True
        else:
            # print 'Not Reached wc, distance: ',e_dist
            # print 'wc_d: ',wc_d
            return False


# %%
if __name__ == '__main__':

    # Symbolics variables definition
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    dh_kuka = DH()

    kuka_fk = FK(dh_kuka)

#    print simplify(kuka_fk.T0_3)
#    print simplify(kuka_fk.invT(kuka_fk.T0_3))
    T3_0 = kuka_fk.invT(kuka_fk.T0_3)
    T3_6 = T3_0 * kuka_fk.T0_6
#    print simplify(T3_6)
    # Correction needed to account of correlation between URDF and DH convention
#    Rcorr = simplify(kuka_fk.rot_m('z', pi) * kuka_fk.rot_m('y', -pi / 2))
#    T0_G = simplify(kuka_fk.T0_EE * Rcorr)
