#!/sr/bin/env python

import numpy as np
from sympy import atan2, cos, latex, pi, pprint, simplify, sin, sqrt, symbols
from sympy.matrices import Matrix
import tf.transformations as tf
import mpmath as mp
from FK import  rot_m, trsl_m, Tmatrix, invT, rpy_m, getWC

def cosLaw_cosC(A,B,C):
    return ((A ** 2 + B ** 2 - C ** 2 ) / (2. * A * B))

def hypotenusePita(A,B):
    return sqrt(A ** 2 + B ** 2)

def rad2deg(x):
    return x * 180 /np.pi

def get_Theta1(wc):
    theta = round(float(mp.atan2(wc[1],wc[0])),3)
    if theta > 0 :
        return [theta, round(theta - np.pi,3)]
    else:
        return [theta, round(theta + np.pi,3)]


def get_Theta2_3(s,wc,theta1):
    # Get x, y proyected coords
    Q = []
    for q in theta1:
#        print q
#        j2 = T0_2.subs({q1:q})[:3,3].T
        projected = round(mp.sqrt(wc[0] ** 2 + wc[1] ** 2),5)
#        x = (projected - s[a1], projected + s[a1])
        x = (projected - 0.35, projected + 0.35)

#        y = round(wc[2] - s[d1],5)
        y = round(wc[2] - 0.75,5)
        # Define sides of triangle
#        A = hypotenusePita(s[d4], s[a3])
        A = 1.501
#        C = s[a2]
        C = 1.25
        # get the Cos of the angles
#        alt = 1
        for alt in range(len(x)):
            B = hypotenusePita(x[alt], y)
            cosA = cosLaw_cosC(B,C,A)
            cosB = cosLaw_cosC(C,A,B)
            # get the ineers angles
            if cosA < 1 and cosB < 1:
                a = mp.atan2(mp.sqrt(1-cosA **2) ,cosA)
                #        a_ = mp.atan2(-mp.sqrt(1-cosA **2) ,cosA)
                b = mp.atan2(mp.sqrt(1-cosB **2) ,cosB)
                #    g = mp.atan2(mp.sqrt(1-cosC **2) ,cosC)
                # get the complementary angles
                alpha = round(mp.atan2(y,x[alt]),5)
#                beta = round(mp.atan2(-s[a3],s[d4]),5)
                beta = 0.036

            # Solve Theta2 and Theta3
                if alt:
                    # Alternative 1:
#                    print 'modo 1'
                    theta2 = round(-float(np.pi/2 - alpha - a),3)
                    theta3 = round(float(np.pi/2 - (b - beta)),3)
                    theta3_ = round(-float(np.pi + theta3 + beta ),3)
                    theta2_ = round(-float(np.pi / 2 + theta2),3)
#                    Q.update({str(alt)+'par3': [q,theta2, theta3_]})
                    Q.append([q,theta2, theta3_])
                    # Alternative 2:
#                    Q.update({str(alt)+'par4': [q,theta2_,theta3]})
                    Q.append([q,theta2_,theta3])
                else:
#                    print 'modo 0'
                    theta2 = round(float(np.pi/2 - alpha - a),3)
                    theta3 = round(float(np.pi/2 - b - beta),3)
#                    Q.update({str(alt)+'par1': [q,theta2, theta3]})
                    Q.append([q,theta2, theta3])
                    # Alternative 2:
                    theta2_ = round(float(np.pi/2 - (alpha - a)),3)
                    theta3_ = round(-float(np.pi + theta3 + 2 * beta ),3)
#                    Q.update({str(alt)+'par2': [q,theta2_,theta3_]})
                    Q.append([q,theta2_,theta3_])
    return Q


def get_Theta1_2_3(s,wc):
     theta1 = get_Theta1(wc)
     Q = get_Theta2_3(s,wc,theta1)
     return Q


def get_wc(px,py,pz,Rrpy,l=0.303):
    wc = np.array([[px],[py],[pz]]) - l * np.array(Rrpy[:3,2])
    return np.round(np.float64(wc.T[0]),5)


def verify_joint_range(q1,q2,q3, rq1=(-3.23,3.23),rq2=(-0.79,1.48),rq3=(-3.67,1.13)):
    if not (rq1[0] <= q1 <= rq1[1]):
        return False
    elif not (-0.79 <= q2 <= 1.48):
        return False
    elif not(-3.67 <= q3 <= 1.13):
        return False
#    elif not (-6.11 <= q4 <= 6.11):
#        return False
#    elif not (-2.18 <= q5 <= 2.18):
#        return False
#    elif not (-6.11 <= q6 <= 6.11):
#        return False
    else:
        return True

def verify_rich_wc(wc,T0_3,j1,j2,j3):
    q1,q2,q3 = symbols('q1:4')

    Q = {q1:j1,q2:j2,q3:j3}
    wc_d = np.float64(np.array(T0_3.evalf(subs=Q)[:3,3]))
    e_dist = np.sqrt(((wc[0]-wc_d[0]) ** 2 +\
                     (wc[1]-wc_d[1]) ** 2 +\
                     (wc[2]-wc_d[2]) ** 2 ))
    if round(e_dist,1) <= 0.1:
#        print 'Riched, distance: ', e_dist
#        print 'wc_d: ',wc_d
        return True
    else:
#        print 'Not Riched wc, distance: ',e_dist
#        print 'wc_d: ',wc_d
        return False


def clean_joint_solutions(wc,T0_5,Q):
    rm = []
    for idx in range(len(Q)):
        j1,j2,j3 = Q[idx]
        if verify_joint_range(j1,j2,j3,rq1=(-2,2),rq2=(-0.79,0.8),rq3=(-1.1,1.1)):
#            print 'range ok'
            if verify_rich_wc(wc, T0_5,j1,j2,j3):
#                print ' rich point'
                pass
            else:
#                print 'not riched'
                rm.append(idx)
        else:
#            print 'out of range'
            rm.append(idx)
    rm.sort(reverse=True)
    print rm
    for idx in rm:
        Q.pop(idx)
    return Q

def get_orientation(Q,T0_3,Rrpy):
    Qf = []
    q1,q2,q3,q4,q5,q6 = symbols('q1:7')
    for each in Q:

    #   Rrpy = np.round(np.float64(np.array(Rrpy[:3,:3])),10)
            joints = {}
            joints = {q1:each[0],q2:each[1],q3:each[2]}
            R3_6 = invT(T0_3).evalf(subs=joints) * Rrpy
            R3_6 = np.round(np.float64(np.array(R3_6)),5)
    #    print R3_6
            theta4 = round(float(mp.atan2(R3_6[2,2],-R3_6[0,2])),2)
#            theta5 = round(float(mp.atan2(mp.sqrt(1-R3_6[1,2]**2), R3_6[1,2])),2)
            theta5 = round(float(mp.atan2(mp.sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])),2)
            theta6 = round(float(mp.atan2(-R3_6[1,1],R3_6[1,0])),2)

            if abs(theta4 + theta6) < 0.01:
                theta4 = 0
                theta6 = 0
    #        print theta4,theta5,theta6
            joints.update({q4:theta4,q5:theta5,q6:theta6})
            Qf.append(joints)
    return Qf


# %%
if __name__ == "__main__":
    # Symbolics variables definition
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols(
        'alpha0:7')
    roll,pitch,yaw = symbols(('roll','pitch','yaw'))
    Rcorr = simplify(rot_m('z', pi) * rot_m('y', -pi / 2))

    # KUKA KR2140 DH parameters
    s = {alpha0: 0, a0: 0, d1: 0.75,
         alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
         alpha2: 0, a2: 1.25, d3: 0,
         alpha3: -pi / 2, a3: -0.054, d4: 1.50,
         alpha4: pi / 2, a4: 0, d5: 0,
         alpha5: -pi / 2, a5: 0, d6: 0,
         alpha6: 0, a6: 0, d7: 0.303, q7: 0}

    # Calculate de sides length

    test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[-0.47958, 0.69824, 3.3771],
                  [0.67747, 0.4606, 0.56252, 0.11163]],
                  [-0.46227, 0.47109, 3.1773],
                  [2.35,-0.4,-0.65,-1.23,0.54,2.72]],
              5:[[[0.020265, -0.2279, 3.2806],
                  [0.43272, -0.71555, -0.035909, 0.54722]],
                  [0.028327, -0.028356, 3.0527],
                  [-0.79,-0.76,-0.47,-1.23,0.54,2.72]]}

    T0_1 = Tmatrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = Tmatrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = Tmatrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = Tmatrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = Tmatrix(alpha4, a4, d5, q5).subs(s)
    T5_6 = Tmatrix(alpha5, a5, d6, q6).subs(s)
    T6_EE = Tmatrix(alpha6, a6, d7, q7).subs(s)

    T0_2 = T0_1 * T1_2
    T0_3 = T0_2 * T2_3
    T0_5 = T0_3 * T3_4 * T4_5
    R0_3 = T0_3[:3,:3]
    T3_6 = T3_4 * T4_5 * T5_6
    T0_6 = T0_3 * T3_6 * T6_EE
    n = 1
    pt = test_cases[n][0][0]
    rpy = tf.euler_from_quaternion(test_cases[n][0][1])
    Rrpy = rpy_m(*rpy) * Rcorr
    wc = get_wc(pt[0],pt[1],pt[2], Rrpy)
#    theta1 = get_Theta1(wc)
#    q = get_Theta2_3(wc,theta1)
#
#    for qi in q:
#        print qi
#        print verify_joint_range(*qi)
#        print verify_rich_wc(wc,T0_5,*qi)
#
#    clean_q = clean_joint_solutions(wc,T0_5,q)
#    print clean_q
# %%
    # TODO: funcion que verifique los rangos de los thetas
#    wc = (wc_x[n],wc_y[n],wc_z[n])
#    x = (wc_x[4] - s[a1])
#    y = (wc_z[4] - s[d1])
#
#
#    A = hypotenusePita(s[d4], s[a3])
#    B = hypotenusePita(x, y)
#    C = s[a2]
#
#    B = hypotenusePita(x, y)
#    cosC = cosLaw_cosC(A,B,C)
#    cosA = cosLaw_cosC(B,C,A)
#    cosB = cosLaw_cosC(C,A,B)
#
#    a = mp.acos(cosA)
#    b = mp.acos(cosB)
#    g = mp.acos(cosC)
#
#    a = mp.atan2(mp.sqrt(1-cosA **2) ,cosA)
#    a_ = mp.atan2(-mp.sqrt(1-cosA **2) ,cosA)
#    b = mp.atan2(mp.sqrt(1-cosB **2) ,cosB)
#    g = mp.atan2(mp.sqrt(1-cosC **2) ,cosC)
#
#    alpha = mp.atan2(y,x)
#    beta = mp.atan2(-s[a3],s[d4])
#    theta2 = np.pi/2 - alpha - a
#    theta2_ = np.pi/2 - alpha - a_
#    theta3 = np.pi/2 - b - beta
#    theta3_ = -(np.pi + theta3 + 2 * beta )
#    print 'Theta2: deg: ',np.round_(float(rad2deg(theta2)),3),'rad: ',np.round_(float(theta2),3)
#    print 'Theta3: deg: ',np.round_(float(rad2deg(theta3)),3),'rad: ',np.round_(float(theta3),3)
#    print 'Alternativa: '
#    print 'Theta2: deg: ',np.round_(float(rad2deg(theta2_)),3),'rad: ',np.round_(float(theta2_),3)
#    print 'Theta3: deg: ',np.round_(float(rad2deg(theta3_)),3),'rad: ',np.round_(float(theta3_),3)
#

    T0_1 = Tmatrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = Tmatrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = Tmatrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = Tmatrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = Tmatrix(alpha4, a4, d5, q5).subs(s)
    T5_6 = Tmatrix(alpha5, a5, d6, q6).subs(s)
    T6_EE = Tmatrix(alpha6, a6, d7, q7).subs(s)

    T0_2 = T0_1 * T1_2
    T0_3 = T0_2 * T2_3
    R0_3 = T0_3[:3,:3]
    T3_6 = T3_4 * T4_5 * T5_6
    T0_6 = T0_3 * T3_6 * T6_EE

#    Q = {q1:0,q2:0,q3:0,q4:-1.371,q5:0,q6:0}

#[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
#[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
#[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]

#[0, 1, 0],
#[0, 0, 1],
#[1, 0, 0]]

#    j4 = atan2(-T3_6[0,2],T3_6[2,2])
#    j5 = atan2(sqrt(1-T3_6[2,1]**2), T3_6[2,1])
#    j6 = atan2(T3_6[2,1],T3_6[2,0])
    #%%
    theta1_2_3 = get_Theta1_2_3(s,wc)

#    Qf = theta1_2_3['sol1']
#    print theta1_2_3
    Q = clean_joint_solutions(wc,T0_5,theta1_2_3)
#    Qf = []
#    each = Q[2]
#    for each in Q:
#
##   Rrpy = np.round(np.float64(np.array(Rrpy[:3,:3])),10)
#        joints = {}
#        joints = {q1:each[0],q2:each[1],q3:each[2]}
#        R3_6 = invT(T0_3).evalf(subs=joints) * Rrpy
#        R3_6 = np.round(np.float64(np.array(R3_6)),5)
##    print R3_6
#        theta4 = round(float(mp.atan2(R3_6[2,2],-R3_6[0,2])),2)
#        theta5 = round(float(mp.atan2(mp.sqrt(1-R3_6[1,2]**2), R3_6[1,2])),2)
#        theta6 = round(float(mp.atan2(-R3_6[1,1],R3_6[1,0])),2)
#
#        if abs(theta4 + theta6) < 0.01:
#            theta4 = 0
#            theta6 = 0
##        print theta4,theta5,theta6
#        joints.update({q4:theta4,q5:theta5,q6:theta6})
#        Qf.append(joints)
#        Qf[each_s].update({q4:theta4,q5:theta5,q6:theta6})
#    Tf = np.float64(np.array(T0_6.subs(Qf)))
#    print Tf
    Qf = get_orientation(Q,T0_3,Rrpy)

    print 'Vector de juntas: ',Qf
    print 'obj: ', test_cases[1][2]
#    for k in range(len(Qf)):
#        for q in sorted(Qf[k].iterkeys()):
#        print "%s: %s" % (k, Qf[k])
#            print Qf[k][q]