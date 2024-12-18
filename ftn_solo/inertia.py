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

grupa1 = {"FR_HFE", "FR_KFE", "FL_HFE", "FL_KFE"}
grupa2 = {"root_joint", "FL_HAA", "FR_HAA", "HL_HAA", "HL_HFE", "HL_KFE", "HR_HAA", "HR_HFE", "HR_KFE"}

I1 = 0
m1 = 0
x1 = 0
y1 = 0
z1 = 0

I2 = 0
m2 = 0
x2 = 0
y2 = 0
z2 = 0

for i in grupa1:
   R = data.oMf[model.getFrameId(i)].rotation
   R_t = np.transpose(R)
   Is = model.inertias[model.getJointId(i)].inertia
   pos_t = data.oMf[model.getFrameId(i)].translation + R.dot(model.inertias[model.getJointId(i)].lever)
   pos_t = np.array([pos_t])
   pos = np.array(pos_t)
   pos = pos.T
   mass = model.inertias[model.getJointId(i)].mass

   I1 = I1 + np.matmul(R, np.matmul(Is, R_t)) + np.matmul(pos, pos_t)*mass

   m1 = m1 + model.inertias[model.getJointId(i)].mass

for i in grupa1:
   x1 = x1 + model.inertias[model.getJointId(i)].mass*(data.oMf[model.getFrameId(i)].translation[0] + model.inertias[model.getJointId(i)].lever[0])
   y1 = y1 + model.inertias[model.getJointId(i)].mass*(data.oMf[model.getFrameId(i)].translation[1] + model.inertias[model.getJointId(i)].lever[1])
   z1 = z1 + model.inertias[model.getJointId(i)].mass*(data.oMf[model.getFrameId(i)].translation[2] + model.inertias[model.getJointId(i)].lever[2])

xg1 = x1/m1
yg1 = y1/m1
zg1 = z1/m1

rc1 = np.array([xg1, yg1, zg1])

for i in grupa2:
   R = data.oMf[model.getFrameId(i)].rotation
   R_t = np.transpose(R)
   Is = model.inertias[model.getJointId(i)].inertia
   pos_t = data.oMf[model.getFrameId(i)].translation + R.dot(model.inertias[model.getJointId(i)].lever)
   pos_t = np.array([pos_t])
   pos = np.array(pos_t)
   pos = pos.T
   mass = model.inertias[model.getJointId(i)].mass

   I2 = I2 + np.matmul(R, np.matmul(Is, R_t)) + np.matmul(pos - np.array([[0.0], [0.0], [0.3395]]), pos_t - np.array([[0.0, 0.0, 0.3395]]))*mass

   m2 = m2 + model.inertias[model.getJointId(i)].mass

for i in grupa2:
   x2 = x2 + model.inertias[model.getJointId(i)].mass*(data.oMf[model.getFrameId(i)].translation[0] + model.inertias[model.getJointId(i)].lever[0])
   y2 = y2 + model.inertias[model.getJointId(i)].mass*(data.oMf[model.getFrameId(i)].translation[1] + model.inertias[model.getJointId(i)].lever[1])
   z2 = z2 + model.inertias[model.getJointId(i)].mass*(data.oMf[model.getFrameId(i)].translation[2] + model.inertias[model.getJointId(i)].lever[2])

xg2 = x2/m2
yg2 = y2/m2
zg2 = z2/m2

rc2 = np.array([xg2, yg2, zg2])

print("I1 = {}\nm1 = {}\nrc1 = {}\n\nI2 = {}\nm2 = {}\nrc2 - rc1 = {}".format(I1, m1, rc1, I2, m2, rc2 - rc1))