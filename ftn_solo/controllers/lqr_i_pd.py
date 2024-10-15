import numpy as np
from datetime import datetime
import pybullet
import matplotlib.pyplot as plt
from matplotlib import style
from matplotlib.animation import FuncAnimation

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
        

    def init_pose(self, position, velocity):
        pass

    def calculate_angle(self, attitude):
        #racuna ugao izmedju tela robota i xy ravni

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
            side = -1        
        else:
            side = 1

        return beta, side

    def compute_control(self, ref_position,  ref_velocity, position, velocity, sensors):
        controlPD = self.Kp * (ref_position - position) + self.Kd * (ref_velocity -
                                                                   velocity) + self.B * ref_velocity 

        attitude = sensors['attitude']
        angular_velocity = sensors['imu']

        beta, side = self.calculate_angle(attitude)


        if(side >= 0):
            q1_1 = np.pi - beta - position[1]
        else:
            q1_1 = beta - position[1]


        q1_2 = position[1] + (np.pi/2)

        if(side >= 0):
            q2_1 = np.pi - beta - position[1]
        else:
            q2_1 = beta - position[1]

        q2_2 = position[4] + (np.pi/2)

        ########################################################
       # self.i += 1

       # x_val.append(self.i)
       # y_val.append(q1_1)
       # plt.xlim(0, 2000)
       # plt.ylim(-10, 10)
       # plt.plot(x_val, y_val)
       # plt.show()
       # plt.pause(0.001)

        ########################################################

        # do ovde je valjda dobro ##################################################################################
#        -angular_velocity[0][1] - velocity[1]
        # angular_velocity[0][1] ugaona brzina tela robota oko y ose

        #controlLQR1 = np.dot(-self.K, np.array([q1_1, q1_2, velocity[1], angular_velocity[0][0]]) - np.array([np.pi, 0, 0, 0]))
        #controlLQR2 = np.dot(-self.K, np.array([q2_1, q2_2, velocity[4], angular_velocity[0][0]]) - np.array([np.pi, 0, 0, 0]))

        controlLQR1 = np.dot(-self.K, np.array([q1_1, q1_2, -angular_velocity[0][1] - velocity[1], velocity[1]]) - np.array([np.pi, 0, 0, 0]))
        controlLQR2 = np.dot(-self.K, np.array([q2_1, q2_2, -angular_velocity[0][1] - velocity[4], velocity[4]]) - np.array([np.pi, 0, 0, 0]))

        controlPD[1] = 0   
        controlPD[4] = 0
                                                           
        controlLQR = np.array([0, controlLQR1, 0, 0, controlLQR2, 0, 0, 0, 0, 0, 0, 0])

        control = controlPD + controlLQR

        return np.clip(control, -self.max_control, self.max_control)

