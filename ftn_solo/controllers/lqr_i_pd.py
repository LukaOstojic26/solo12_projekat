import numpy as np
import pybullet

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

        x_dot_proj_x = np.dot(x, proj_x)

        beta = np.arccos(x_dot_proj_x/(x_len*proj_x_len)) # ugao izmedju vektora x i proj_x

        return beta

    def compute_control(self, ref_position,  ref_velocity, position, velocity, sensors):
        controlPD = self.Kp * (ref_position - position) + self.Kd * (ref_velocity -
                                                                   velocity) + self.B * ref_velocity 

        attitude = sensors['attitude']
        angular_velocity = sensors['imu']

        beta = self.calculate_angle(attitude)

        q1_1 = np.pi - beta - position[1]
        q1_2 = position[1] + (np.pi/2)

        q2_1 = np.pi - beta - position[4]
        q2_2 = position[4] + (np.pi/2)

        # do ovde je valjda dobro ##################################################################################

        #controlLQR1 = np.dot(-self.K, np.array([q1_1, q1_2, velocity[1], angular_velocity[0][0]]) - np.array([np.pi, 0, 0, 0]))
        #controlLQR2 = np.dot(-self.K, np.array([q2_1, q2_2, velocity[4], angular_velocity[0][0]]) - np.array([np.pi, 0, 0, 0]))

        controlLQR1 = np.dot(-self.K, np.array([q1_1, q1_2, angular_velocity[0][1], angular_velocity[0][1] - velocity[1]]) - np.array([np.pi, 0, 0, 0]))
        controlLQR2 = np.dot(-self.K, np.array([q2_1, q2_2, angular_velocity[0][1], angular_velocity[0][1] - velocity[4]]) - np.array([np.pi, 0, 0, 0]))

        controlPD[1] = 0   
        controlPD[4] = 0
                                                           
        controlLQR = np.array([0, controlLQR1, 0, 0, controlLQR2, 0, 0, 0, 0, 0, 0, 0])

        control = controlPD + controlLQR

        return np.clip(control, -self.max_control, self.max_control)
