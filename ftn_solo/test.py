import pinocchio as pin
from pinocchio.utils import *
import numpy as np

urdf_filename = '/media/luka/HDD/solo12/src/robot_properties_solo/src/robot_properties_solo/resources/urdf/solo12.urdf'

model = pin.buildModelFromUrdf(urdf_filename, pin.JointModelFreeFlyer())
data = model.createData()

q = pin.randomConfiguration(model)

q[:] = 0

q[2] = 0.534
q[4] = 0.7071068
q[6] = 0.7071068 

q[8:18:3] = -1.57

print("q: %s" % q.T)

pin.framesForwardKinematics(model, data, q)

print(model.inertias[model.getJointId("root_joint")])

