#!/usr/bin/env python

import numpy as np
from sympy import atan2, cos, latex, pi, pprint, simplify, sin, sqrt, symbols
from sympy.matrices import Matrix
import tf.transformations as tf




def rot_m(axis, q):
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


def trsl_m(x, y, z):
    T = Matrix([[1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]])
    return T


def Tmatrix(alpha, a, d, q):
    T = rot_m('x', alpha) * trsl_m(a, 0, 0) * rot_m('z', q) * trsl_m(0, 0, d)
    T = simplify(T)
    return T


def invT(T):
    R = T[:3, :3]
    D = T[:3, 3]
    bottom_row = Matrix([[0, 0, 0, 1]])
    iR = R.T
    iD = iR * D
    iT = iR.row_join(iD)
    iT = iT.col_join(bottom_row)
    return iT

def getWC(px,py,pz,l,T):
    wc = Matrix([[px],[py],[pz]]) - (l * T[:3,:3])[:,2]
    return (wc[0], wc[1], wc[2])

def rpy_m(roll, pitch, yaw):
    return rot_m('z',yaw) * rot_m('y',pitch) * rot_m('x',roll)


if __name__ == '__main__':

    # Symbolics variables definition
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols(
        'alpha0:7')
    # KUKA KR2140 DH parameters
    s = {alpha0: 0, a0: 0, d1: 0.75,
         alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
         alpha2: 0, a2: 1.25, d3: 0,
         alpha3: -pi / 2, a3: -0.054, d4: 1.50,
         alpha4: pi / 2, a4: 0, d5: 0,
         alpha5: -pi / 2, a5: 0, d6: 0,
         alpha6: 0, a6: 0, d7: 0.303, q7: 0}

    # Homogeneous Transforms
    T0_1 = Tmatrix(alpha0, a0, d1, q1)
    T1_2 = Tmatrix(alpha1, a1, d2, q2)
    T2_3 = Tmatrix(alpha2, a2, d3, q3)
    T3_4 = Tmatrix(alpha3, a3, d4, q4)
    T4_5 = Tmatrix(alpha4, a4, d5, q5)
    T5_6 = Tmatrix(alpha5, a5, d6, q6)
    T6_EE = Tmatrix(alpha6, a6, d7, q7)
    # Subtitution
    T_n = [T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_EE]
    for each in range(len(T_n)):
        T_n[each] = T_n[each].subs(s)

    # Composition of Homogeneous Transforms
    T0_2 = simplify(T_n[0] * T_n[1])  # base to link2
    T0_3 = simplify(T0_2 * T_n[2])  # link2 to link3
    T0_4 = simplify(T0_3 * T_n[3])  # link3 to link4
    T0_5 = simplify(T0_4 * T_n[4])  # link4 to link5
    T0_6 = simplify(T0_5 * T_n[5])  # link5 to link6
    T0_EE = simplify(T0_6 * T_n[6])  # link5 to link6

    # Correction needed to account of correlation between URDF and DH convention
    Rcorr = simplify(rot_m('z', pi) * rot_m('y', -pi / 2))
    T0_G = simplify(T0_EE * Rcorr)

    # joint angles vector
    q_0 = {q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}
    q_1 = {q1: 0, q2: 0, q3: -pi/2, q4: pi/2, q5: pi/3, q6: 0}
    q = {q1: -1.33, q2: 1.22, q3: -2.53, q4: -5.90, q5: -1.26, q6: -1.31}
    # Evaluation for comparing with output of tf_echo!
#    print("T0_1 = ", T_n[0].evalf(2,subs=q))
#    print("T0_2 = ", T0_2.evalf(2,subs=q))
#    print("T0_3 = ", T0_3.evalf(subs=q))
#    print("T0_4 = ", T0_4.evalf(subs=q))
#    print("T0_5 = ", T0_5.evalf(subs=q))
#    print("T0_6 = ", T0_6.evalf(subs=q))
    M =  np.array(T0_G.evalf(5,subs=q_0))
    print("T0_G = ", M)
    print(tf.euler_from_matrix(M))
    T0_G = T0_G.evalf(subs=q_0)
    Rrpy = rpy_m(0,0.379,0) * Rcorr
#    wc_x,wc_y,wc_z = getWC(2.153, 0.000, 1.947,Rrpy) # Home pos
