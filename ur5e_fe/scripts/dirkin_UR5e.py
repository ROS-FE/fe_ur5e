#!/usr/bin/env python

# direktna kinematika za robota UR5e
# A matrike
# T6 matrika
# J matrika

import math
import numpy as np
import rospy
import tf

class dirkinUR5e:
    def __init__(self):

        # dh parametri
        self.a = [0.00000, -0.42500, -0.39225, 0.00000, 0.00000, 0.0000]
        self.d = [0.1625, 0.138, -0.131, 0.127, 0.0997, 0.0996]
        self.alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]
        self.theta_offset = [0, 0, 0, 0, 0, 0]

        '''
        # motoman mh5 DH parametri
        self.a = [0.088, -0.310, -0.040, 0.00000, 0.00000, 0.0000]
        self.d = [0.131, 0.0, 0.0, 0.305, 0.0, 0.0865]
        self.alpha = [-math.pi/2, 0, math.pi/2, -math.pi/2, math.pi/2, 0]
        self.theta_offset = [0, math.pi/2, 0, 0, 0, 0]
        '''

    def hdh(self, dh_param):
        theta = dh_param[0]
        d = dh_param[1]
        a = dh_param[2]
        alpha = dh_param[3]

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        cos_alpha = math.cos(alpha)
        sin_alpha = math.sin(alpha)

        A = np.array([  [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
                        [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],    
                        [0, sin_alpha, cos_alpha, d],
                        [0, 0, 0, 1]])
        
        return A
    
    def dirkinT6(self, qq):
        A01 = self.hdh([qq[0]+self.theta_offset[0], self.d[0], self.a[0], self.alpha[0]])
        A12 = self.hdh([qq[1]+self.theta_offset[1], self.d[1], self.a[1], self.alpha[1]])
        A23 = self.hdh([qq[2]+self.theta_offset[2], self.d[2], self.a[2], self.alpha[2]])
        A34 = self.hdh([qq[3]+self.theta_offset[3], self.d[3], self.a[3], self.alpha[3]])
        A45 = self.hdh([qq[4]+self.theta_offset[4], self.d[4], self.a[4], self.alpha[4]])
        A56 = self.hdh([qq[5]+self.theta_offset[5], self.d[5], self.a[5], self.alpha[5]])

        self.A01 = A01
        self.A02 = np.matmul(self.A01,A12)
        self.A03 = np.matmul(self.A02,A23)
        self.A04 = np.matmul(self.A03,A34)
        self.A05 = np.matmul(self.A04,A45)
        self.T6 = np.matmul(self.A05,A56)

        return self.T6

    def jacobi_G(self, qq):
    
        Jg = np.zeros((6, 6))
        # izracunaj direktno kinematiko
        self.dirkinT6(qq)

        z0 = np.array([0, 0, 1])
        Tn = np.eye(4)
        
        ## 1. sklep
        p6 = self.T6[0:3,3]
        Jg[0:3, 0] = np.cross(z0,p6)                        # Jp 1. stolpec
        Jg[3:6, 0] = z0                                     # Jo 1. stolpec

        ## 2. sklep
        Tn = self.A01 
        z_tmp = np.matmul(Tn[0:3,0:3],np.transpose(z0))
        Jg[0:3, 1] = np.cross(z_tmp,(p6-Tn[0:3,3]))         # Jp 2. stolpec
        Jg[3:6, 1] = z_tmp                                  # Jo 2. stolpec

        ## 3. sklep
        Tn = self.A02 
        z_tmp = np.matmul(Tn[0:3,0:3],np.transpose(z0))
        Jg[0:3, 2] = np.cross(z_tmp,(p6-Tn[0:3,3]))         # Jp 3. stolpec
        Jg[3:6, 2] = z_tmp                                  # Jo 3. stolpec

        ## 4. sklep
        Tn = self.A03 
        z_tmp = np.matmul(Tn[0:3,0:3],np.transpose(z0))
        Jg[0:3, 3] = np.cross(z_tmp,(p6-Tn[0:3,3]))         # Jp 4. stolpec
        Jg[3:6, 3] = z_tmp                                  # Jo 4. stolpec

        ## 5. sklep
        Tn = self.A04 
        z_tmp = np.matmul(Tn[0:3,0:3],np.transpose(z0))
        Jg[0:3, 4] = np.cross(z_tmp,(p6-Tn[0:3,3]))         # Jp 5. stolpec
        Jg[3:6, 4] = z_tmp                                  # Jo 5. stolpec

        ## 6. sklep
        Tn = self.A05 
        z_tmp = np.matmul(Tn[0:3,0:3],np.transpose(z0))
        Jg[0:3, 5] = np.cross(z_tmp,(p6-Tn[0:3,3]))         # Jp 6. stolpec
        Jg[3:6, 5] = z_tmp                                  # Jo 6. stolpec

        self.Jg = Jg

        return Jg

    def rot_x(self, angle):

        R = np.array([[1, 0, 0],[0, math.cos(angle), -math.sin(angle)], [0, math.sin(angle), math.cos(angle)]])

        return R

    def rot_y(self, angle):
        
        R = np.array([[math.cos(angle), 0, math.sin(angle)],[0, 1, 0],[-math.sin(angle), 0, math.cos(angle)]])

        return R
    
    def rot_z(self, angle):
        
        R = np.array([[math.cos(angle), -math.sin(angle), 0],[math.sin(angle), math.cos(angle), 0],[0, 0, 1]])

        return R

    def Ta_matrix(self, angles, axes):
        
        T = np.zeros((3,3)) 
        
        if axes[0] == 'x':
            T[0:3,0] = np.transpose([1, 0, 0])
            T1 = self.rot_x(angles[0])
        elif axes[0] == 'y':
            T[0:3,0] = np.transpose([0, 1, 0])
            T1 = self.rot_y(angles[0])
        elif axes[0] == 'z':
            T[0:3,0] = np.transpose([0, 0, 1])
            T1 = self.rot_z(angles[0])
        else:
            rospy.errorlog("Wrong Euler axes [0] for calculating Ta_matrix")

        if axes[1] == 'x':
            T2 = np.matmul(T1,self.rot_x(angles[1])) 
            T[0:3,1] = T1[0:3,0] 
        elif axes[1] == 'y':
            T2 = np.matmul(T1,self.rot_y(angles[1]))
            T[0:3,1] = T1[0:3,1]
        elif axes[1] == 'z':
            T2 = np.matmul(T1,self.rot_z(angles[1]))
            T[0:3,1] = T1[0:3,2]
        else:
            rospy.errorlog("Wrong Euler axes [1] for calculating Ta_matrix")

        if axes[2] == 'x':
            T[0:3,2] = T2[0:3,0] 
        elif axes[2] == 'y':
            T[0:3,2] = T2[0:3,1]
        elif axes[2] == 'z':
            T[0:3,2] = T2[0:3,2]
        else:
            rospy.errorlog("Wrong Euler axes [2] for calculating Ta_matrix")

        Ta = np.identity(6)
        Ta[3:6,3:6] = T

        self.Ta = Ta

        return Ta

    def jacobi_A(self,qq,axes):

        # geometrijski jacobi
        Jg = self.jacobi_G(qq)

        # eulerjevi koti
        eul = tf.transformations.euler_from_matrix(self.T6, axes='r'+axes)

        # Ta matrika
        Ta = self.Ta_matrix(eul, axes)

        # Ja = Ta^-1 Jg
        Ja = np.matmul(np.linalg.inv(Ta),Jg)

        iJa = np.linalg.inv(Ja)

        return Ja, iJa


'''
dk_ur =dirkinUR5e()

hh = dk_ur.hdh([1,2,3,4])

H6 = dk_ur.dirkinT6([1,2,3,4,5,6])

jg = dk_ur.jacobi_G([1,2,3,4,5,6])

ja, ija, eul = dk_ur.jacobi_A([1,2,3,4,5,6],'zyx')

print(ija)

print(jg)

#print(np.linalg.inv(jg))

dk_ur.Ta_matrix([0,0,0],'zyx')

print(ija)
'''