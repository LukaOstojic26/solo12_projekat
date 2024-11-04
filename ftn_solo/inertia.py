import pinocchio as pin
from pinocchio.utils import *
import numpy as np

urdf_filename = '/media/luka/HDD/solo12/src/robot_properties_solo/src/robot_properties_solo/resources/urdf/solo12.urdf'

model = pin.buildModelFromUrdf(urdf_filename, pin.JointModelFreeFlyer())
data = model.createData()

joint_id = {

    "root_joint" : 1,
    "FL_HAA" : 2,
    "FL_HFE" : 3,
    "FL_KFE" : 4,
    "FR_HAA" : 5,
    "FR_HFE" : 6,
    "FR_KFE" : 7,
    "HL_HAA" : 8,
    "HL_HFE" : 9,
    "HL_KFE" : 10,
    "HR_HAA" : 11,
    "HR_HFE" : 12,
    "HR_KFE" : 13

}

q = np.zeros(model.nq)

q[2] = 0.534
q[4] = 0.7071068
q[6] = 0.7071068 

q[8:18:3] = -1.57

print("q: %s" % q.T)

pin.framesForwardKinematics(model, data, q)

grupa1 = {"FR_HFE", "FR_KFE", "FL_HFE", "FL_KFE"}
grupa2 = {"root_joint", "FL_HAA", "FR_HAA", "HL_HAA", "HL_HFE", "HL_KFE", "HR_HAA", "HR_HFE", "HR_KFE"}

I1 = 0
I2 = 0

for i in grupa1:
   I1 = I1 + data.oMf[joint_id[i]].rotation*model.inertias[joint_id[i]].inertia*np.transpose(data.oMf[joint_id[i]].rotation) + np.dot(data.oMf[joint_id[i]].translation, data.oMf[joint_id[i]].translation)*model.inertias[joint_id[i]].mass

for i in grupa2:
   I2 = I2 + data.oMf[joint_id[i]].rotation*model.inertias[joint_id[i]].inertia*np.transpose(data.oMf[joint_id[i]].rotation) + np.dot(data.oMf[joint_id[i]].translation - np.array([0, 0, 0.534]), data.oMf[joint_id[i]].translation - np.array([0, 0, 0.534]))*model.inertias[joint_id[i]].mass


print("I1 = {}\nI2 = {}".format(I1, I2))