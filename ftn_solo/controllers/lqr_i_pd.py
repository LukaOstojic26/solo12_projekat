import numpy as np
from datetime import datetime
import pybullet
import matplotlib.pyplot as plt
from matplotlib import style
from matplotlib.animation import FuncAnimation
import cvxpy as cp

x_val, y_val = [], []
plt.ion()
fig = plt.figure()
plt.axis([0,1000, -2*np.pi, 2*np.pi])

def float_or_list(value, num_joints):
    return np.array(value if type(value) is list else [float(value)]*num_joints, dtype=np.float64)

class LQR():
    def __init__(self, num_joints, yaml_config) -> None:
        self.Kp = float_or_list(yaml_config["Kp"], num_joints)
        self.Kd = float_or_list(yaml_config["Kd"], num_joints)
        self.B = float_or_list(yaml_config["B"], num_joints)
        self.K = float_or_list(yaml_config["K"], num_joints)
        self.alfa = float_or_list(yaml_config["alfa"], num_joints)
        self.beta = float_or_list(yaml_config["beta"], num_joints)
        self.max_control = float_or_list(
            yaml_config["max_control"], num_joints)
        self.i = 0 #za plotovanje uglova
        self.beta3 = 0
        self.pocetno_tetadd1 = 0
        

    def init_pose(self, position, velocity):
        pass

    def calculate_angle(self, attitude):
        #racuna ugao izmedju tela robota i vertikale

        qw = attitude[0]
        qx = attitude[1]
        qy = attitude[2]
        qz = attitude[3]

        x = np.array([1 - 2*(qy*qy + qz*qz), 2*(qx*qy + qw*qz), 2*(qx*qz - qw*qy)]) # prva kolona iz matrice 3.53 (str. 40) Industrijska robotika

        proj_x = np.array([1 - 2*(qy*qy + qz*qz), 2*(qx*qy + qw*qz), 0]) # projekcija vektora x na xy ravan

        x_len = np.linalg.norm(x)  #duzine vektora
        proj_x_len = np.linalg.norm(proj_x)

        norm_x = (1/x_len)*x
        norm_proj_x = (1/proj_x_len)*proj_x

        x_dot_proj_x = np.dot(norm_x, norm_proj_x)

        x_dot_i = np.dot(norm_x, np.array([1, 0, 0])) # da bi odredili na koju stranu pada robot (x_dot_i je <0 ako pada na desnu stranu)

        beta = np.arccos(x_dot_proj_x) # ugao izmedju vektora x i proj_x

        if(x_dot_i < 0):
            beta = beta - np.pi/2
        else:          
            beta = np.pi/2 - beta

        return beta
    
    def calculate_angle2(self, imu):

        ang_vel = imu[0] 

        beta = ang_vel[1]

        return beta

    def matrice_A_B_C_D(self, param, ac1x, ac1y, ac2x, ac2y, q1_1, q1_2, q1_1_d, q1_2_d, tau):
        I1, I2, m1, m2, l1, l2, lc1, lc2 = param

        A = 1/(2*np.cos(q1_1))
        B = (1/(2*np.cos(q1_1)))*(-2*m2*ac2x*np.sin(q1_1) - I1*q1_1_d + m1*9.81*np.cos(q1_1) + m1*ac1y*np.cos(q1_1) + m1*ac1x*np.sin(q1_1))
        C = -1/(np.sin(q1_1 + q1_2))
        D = m1*ac1x - (1/(np.sin(q1_1 + q1_2)))*(m2*ac2y*np.cos(q1_1 + q1_2) + m2*9.81*np.cos(q1_1 + q1_2) + I2*(q1_1_d + q1_2_d))

        epsilon = 1
        mi = 1

        v1 = (epsilon - B)/A
        v2 = (-mi*B-D)/(C + mi*A)
        v3 = (mi*B - D)/(C - mi*A)

        min, mid, max = sorted((v1, v2, v3))

        if A > 0:
            if min == v1 or mid == v1:
                if tau <= max and tau >= mid:
                    return tau
                elif tau > max:
                    return max
                elif tau < mid:
                    return mid
            else:
                return max
        elif A <= 0:
            if min == v1:
                return min
            if mid == v1 or max == v1:
                if tau <= mid and tau >= min:
                    return tau
                elif tau > mid:
                    return mid
                elif tau < min:
                    return min




    def compute_control(self, ref_position,  ref_velocity, position, velocity, sensors):
        controlPD = self.Kp * (ref_position - position) + self.Kd * (ref_velocity -
                                                                   velocity) + self.B * ref_velocity 

        attitude = sensors['attitude']
        imu = sensors['imu']
        touch = sensors['touch']

        beta = self.calculate_angle(attitude)
        beta2 = self.calculate_angle2(imu)

        self.beta3 -= beta2*0.001

        q1_1 = self.beta3 - position[1]

        q1_2 = position[1] + (np.pi/2)

        q2_1 = self.beta3 - position[4]

        q2_2 = position[4] + (np.pi/2)

        q1_1_d = -imu[0][1] - velocity[1]
        q1_2_d = velocity[1]
        q2_1_d = -imu[0][1] - velocity[4]
        q2_2_d = velocity[4]

        controlLQR1 = np.dot(-self.K, np.array([q1_1, q1_2, -imu[0][1] - velocity[1], velocity[1]]) - np.array([np.pi/2, 0, 0, 0]))
        controlLQR2 = np.dot(-self.K, np.array([q2_1, q2_2, -imu[0][1] - velocity[4], velocity[4]]) - np.array([np.pi/2, 0, 0, 0]))

        controlPD_sym1 = 8 * (position[4] - position[1]) + 0.05 * (velocity[4] - velocity[1])
                                                                   
        controlPD_sym2 = 8 * (position[1] - position[4]) + 0.05 * (velocity[1] - velocity[4])

        controlPD[1] = 0   
        controlPD[4] = 0

        ###### QP problem ######

        #epsilon_x, epsilon_y = 0.1, 0.1
#
        #tau = cp.Variable(1)
#
        #A_Fx = np.array([abs(np.sin(q1_1))/0.3375])
        #A_Fy = np.array([abs(np.cos(q1_1))/0.3375])
#
        #constraints = [
            #A_Fx @ tau >= epsilon_x,
            #A_Fy @ tau >= epsilon_y - (0.34111236 + 2.1601915299999996)*9.81  
        #]
#
        #objective = cp.Minimize(0.5*cp.square(tau - controlLQR1))
#
        #problem = cp.Problem(objective, constraints)
        #problem.solve()
#
        #tau_optimal = tau.value
        #tau_optimal = tau_optimal[0]


        ########################

        epsilon_x, epsilon_y = 0.1, 0.001
        m1, m2, l1, lc1, lc2, I1 = 0.34111236, 2.1601915299999996, 0.3375, 0.240403710, 0.211247879, 0.00192579089
        param = (0.00192579089, 0.0281182992, 0.34111236,  2.1601915299999996,  0.3375,  0.4087,  0.240403710,  0.211247879) # I1, I2, m1, m2, l1, l2, lc1, lc2

        tetadd1 = ((-imu[0][1] - velocity[1]) - self.pocetno_tetadd1)/0.001 #racunam teta1 sa dve tacke, odnosno ugaono ubrzanje prvog segmenta
        self.pocetno_tetadd1 = -imu[0][1] - velocity[1]

        #ac1x = -lc1*(tetadd1*np.sin(q1_1) + ((-imu[0][1] - velocity[1])**2)*np.cos(q1_1)) ovo sam pre nesto racunao
        #ac1y = lc1*(tetadd1*np.cos(q1_1) - ((-imu[0][1] - velocity[1])**2)*np.sin(q1_1))

        ac1x = -lc1*tetadd1*tetadd1*np.cos(q1_1)
        ac1y = -lc1*tetadd1*tetadd1*np.sin(q1_1)

        ac2x = imu[1][0]
        ac2y = imu[1][2]

        upravljanje1 = self.matrice_A_B_C_D(param, ac1x, ac1y, ac2x, ac2y, q1_1, q1_2, q1_1_d, q1_2_d, controlLQR1)
        upravljanje2 = self.matrice_A_B_C_D(param, ac1x, ac1y, ac2x, ac2y, q2_1, q2_2, q2_1_d, q2_2_d, controlLQR2)

        ########################

        controlLQR = np.array([0, 0 + controlPD_sym1, 0, 0, 0 + controlPD_sym2, 0, 0, 0, 0, 0, 0, 0])

        ########################################################
        self.i += 1
#
        x_val.append(self.i)
        y_val.append((q1_1_d, q1_2_d))
        plt.xlim(0, 3000)
        plt.ylim(-2, 2)
        plt.plot(x_val, y_val)
        plt.show()
        plt.pause(0.00001)
#
        ########################################################

        control = controlPD + controlLQR

        return np.clip(control, -self.max_control, self.max_control)

