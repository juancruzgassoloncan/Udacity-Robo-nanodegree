#!/sr/bin/env python

import numpy as np
from sympy import atan2, cos, latex, pi, pprint, simplify, sin, sqrt, symbols, acos
from sympy.matrices import Matrix
import tf.transformations as tf
import mpmath as mp
from FK import  rot_m, trsl_m, Tmatrix, invT, rpy_m, getWC
from FK_class import FK,DH






def cosLaw_cosC(A,B,C):
    return ((A ** 2 + B ** 2 - C ** 2 ) / (2. * A * B))

def hypotenusePita(A,B):
    return sqrt(A ** 2 + B ** 2)

def rad2deg(x):
    return x * 180 /np.pi


# %%
class IK(FK):
    roll = simplify('roll')
    pitch = simplify('pitch')
    yaw = simplify('yaw')

    def __init__(self):
        self.wc = []
        self.Rcor = self.rpy_m(pi,-pi/2,0)
        self.Rrpy = self.rpy_m(self.roll, self.pitch, self.yaw) * self.Rcor

    def get_Rrpy(self,r,p,y):
        self.Rrpy = self.Rrpy.subs({self.roll:r, self.pitch:p, self.yaw:y})
        return self.Rrpy

    def get_wc(self,px,py,pz,Rrpy=None,l=0.303):
        if Rrpy is None: Rrpy = self.Rrpy
        self.wc = np.array([[px],[py],[pz]]) - l * np.array(Rrpy[:3,2])
        self.wc = self.wc.T[0]
        return self.wc

    def get_Theta1(self,wc=None):
        if wc is None: wc=self.wc

        theta = float(atan2(wc[1],wc[0]).evalf())
        if theta >= 0 :
            return [theta, theta - np.pi]
        else:
            return [theta, theta + np.pi]

    def get_Theta2_3(self,wc=None,wk=0, cfg=1):
        if wc is None: wc=self.wc

        # Get x, y proyected coords
        Q = []
        projected = hypotenusePita(wc[0],wc[1])
        x = (projected - 0.35, # worikng a head
             projected + 0.35) # workin backward
        x = x[wk]
        y = wc[2] - 0.75
        # Define sides of triangle
        A = 1.501
        C = 1.25
            # get the Cos of the angles
        B = hypotenusePita(x, y)
        cosA = cosLaw_cosC(B,C,A)
        cosB = cosLaw_cosC(C,A,B)
#        print 'cosA ',cosA
#        print'cosB ',cosB
        # get the ineers angles
        if cosA >= 1:
#            print 'a out',cosA
#            print "NO REACHEABLE POINT"
            cosA = 1 * mp.sign(cosA)
        if abs(cosB) >= 1:
#            print 'b out',cosB
            cosB = 1 * mp.sign(cosB)
#        else:
        a = atan2(mp.sqrt(1-cosA **2), cosA)
        b = atan2(mp.sqrt(1-cosB **2), cosB)

        # get the complementary angles
        alpha = atan2(y,x)
        beta = 0.036

                # Solve Theta2 and Theta3
        if wk:
            theta2 = -(np.pi/2 - alpha - a)
            theta3 = -(np.pi/2 - b - beta)
            theta3_ = -(np.pi - theta3 + 2 * beta)
            theta2_ = -(np.pi/2 - alpha + a)
            if cfg:
            # cfg 1: elbow up
                Q = [theta2.evalf(), theta3_.evalf()]
            else:
            # cfg 2: elbow down
                Q = [theta2_.evalf(),theta3.evalf()]
        else:
            theta2 = np.pi/2 - alpha - a
            theta3 = np.pi/2 - b - beta
            theta2_ = np.pi/2 - alpha + a
            theta3_ = -(np.pi + theta3 + 2 * beta )
            if cfg:
            # cfg 1: elbow up
                Q = [theta2.evalf(), theta3.evalf()]
            else:
            # cfg-2: elbow down
                Q = [theta2_.evalf(),theta3_.evalf()]
        return Q

    def get_orientation(self,Qwc,T3_0,Rrpy=None):
        if Rrpy is None: Rrpy=self.Rrpy
        q1,q2,q3,q4,q5,q6 = symbols('q1:7')
        joints = {}
        joints = {q1:Qwc[0],q2:Qwc[1],q3:Qwc[2]}
        R3_6 = T3_0.evalf(subs=joints) * Rrpy
        R3_6 = np.round(np.float64(np.array(R3_6)),5)
    #    print R3_6
        theta4 = atan2(R3_6[2,2],-R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
        theta6 = atan2(-R3_6[1,1],R3_6[1,0])
    #        print theta4,theta5,theta6
        joints.update({q4:theta4,q5:theta5,q6:theta6})
#        Qf.append(joints)
        return (theta4,theta5,theta6)



# %%
def get_Theta1_2_3(s,wc):
     theta1 = get_Theta1(wc)
     Q = get_Theta2_3(s,wc,theta1)
     return Q


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


#[0.850979590356099 2.16703764381894 1.38605713827320]
#q=[0.862681150861269 ,0.461817448667319, -0.170204147609226]

#[0.736235619812743 2.23262538887108 1.40856583001886]
#[0.533137745611759 2.32834675646592 1.44525693223893]
#[0.319195070728320 2.40449274786955 1.48056617584669]
#[0.101365555534477 2.45890086944120 1.51392454329762]
#[-0.123047933145421 2.49253488408743 1.54632460265737]
#[-0.246102926117589 2.50176697954842 1.56347399583520]
#[-0.370140021709296 2.50462290621726 1.58044171557705]
#[-0.441518268789593 2.50335008680461 1.59009380374323]
#[-0.513038549472886 2.49994199750433 1.59970494816823]

# %%
if __name__ == "__main__":
    # Symbolics variables definition
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
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
                  [-0.79,-0.76,-0.47,-1.23,0.54,2.72]],
              6:[[[-0.71977, 0.71957, 3.3389],
                  [0.35118, 0.848, 0.15188, -0.36675]],
                  [-0.57302, 0.57286, 3.1181],
                  [-0.79,-0.14,-2.19,-0,0,0]]}
    dh_kuka = DH()
    fk_kuka = FK(dh_kuka)
    ik_kuka = IK()
    n = 1
    pt = test_cases[n][0][0]
    rpy = tf.euler_from_quaternion(test_cases[n][0][1])
    ik_kuka.get_Rrpy(*rpy)
#    wc = ik_kuka.get_wc(pt[0],pt[1],pt[2])
    wc = ik_kuka.wc = [-0.513038549472886, 2.49994199750433, 1.59970494816823]
    tt1 = ik_kuka.get_Theta1()
    tt2_3 =ik_kuka.get_Theta2_3(wk=0)
    print 'Wrist center: '
    print ik_kuka.wc
    print test_cases[n][1]
    print 'Thetas'
    print 'q1: ', tt1, 'q2: ',tt2_3[0], 'q3: ',tt2_3[1]
    print test_cases[n][2]


    T3_0 = fk_kuka.invT(fk_kuka.T0_3)
    print ik_kuka.get_orientation((tt1[0],tt2_3[0],tt2_3[1]),T3_0)
#    fk_kuka.verify_rich_wc(min(tt1),tt2_3[0],tt2_3[1],wc)
    fk_kuka.verify_rich_wc(-0.65,0.45,-0.36,wc)
    # TODO: Multiple solution orientation