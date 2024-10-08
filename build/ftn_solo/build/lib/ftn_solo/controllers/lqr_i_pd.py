import numpy as np

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

    def compute_control(self, ref_position,  ref_velocity, position, velocity):
        controlPD = self.Kp * (ref_position - position) + self.Kd * (ref_velocity -
                                                                   velocity) + self.B * ref_velocity 
        controlLQR_temp = np.dot(self.K, np.array(position[1], position[4]))

        controlPD[1] = 0   
        controlPD[4] = 0
                                                           
        controlLQR = np.array([0, controlLQR_temp[0], 0, 0, controlLQR_temp[1], 0, 0, 0, 0, 0, 0, 0])

        control = controlPD + controlLQR
        return np.clip(control, -self.max_control, self.max_control)
