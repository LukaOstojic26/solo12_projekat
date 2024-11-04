import pinocchio as pin
from pinocchio.utils import *
import numpy as np

urdf_filename = '/media/luka/HDD/solo12/src/robot_properties_solo/src/robot_properties_solo/resources/urdf/solo12.urdf'

model = pin.buildModelFromUrdf(urdf_filename, pin.JointModelFreeFlyer())
data = model.createData()

#################################

joint_Is = {
    "root_joint" : np.array([
        [0.0026620, -0.00003684, -0.00001716],
        [-0.00003684, 0.01388510, -0.00000009],
        [-0.00001716, -0.00000009, 0.01605370]
    ]),

    "FL_HAA" : np.array([
        [0.00002802, 0.00003687, -0.00000009],
        [0.00003687, 0.00038264, 0.00000000],
        [-0.00000009, 0.00000000, 0.00038050]
    ]),

    "FL_HFE" : np.array([
        [0.00041540, 0.00000000, 0.00000010],
        [0.00000000, 0.00041637, -0.00004589],
        [0.00000010, -0.0004589, 0.00002982]
    ]),

    "FL_KFE" : np.array([
        [0.00008508, 0.00000000, 0.00000000],
        [0.00000000, 0.00008580, -0.00000200],
        [0.00000000, -0.00000200, 0.00000139]
    ]),

    "FR_HAA" : np.array([
        [0.00002802, -0.00003687, 0.00000009],
        [-0.00003687, 0.00038264, 0.00000000],
        [0.00000009, 0.00000000, 0.00038050]
    ]),

    "FR_HFE" : np.array([
        [0.00041540, 0.00000000, -0.00000010],
        [0.00000000, 0.00041637, 0.00004589],
        [-0.00000010, 0.0004589, 0.00002982]
    ]),

    "FR_KFE" : np.array([
        [0.00008508, 0.00000000, 0.00000000],
        [0.00000000, 0.00008580, 0.00000200],
        [0.00000000, 0.00000200, 0.00000139]
    ]),

    "HL_HAA" : np.array([
        [0.00002802, -0.00003687, -0.00000009],
        [-0.00003687, 0.00038264, 0.00000000],
        [-0.00000009, 0.00000000, 0.00038050]
    ]),

    "HL_HFE" : np.array([
        [0.00041540, 0.00000000, 0.00000010],
        [0.00000000, 0.00041637, -0.00004589],
        [0.00000010, -0.0004589, 0.00002982]
    ]),

    "HL_KFE" : np.array([
        [0.00008508, 0.00000000, 0.00000000],
        [0.00000000, 0.00008580, -0.00000200],
        [0.00000000, -0.00000200, 0.00000139]
    ]),

    "HR_HAA" : np.array([
        [0.00002802, 0.00003687, 0.00000009],
        [0.00003687, 0.00038264, 0.00000000],
        [0.00000009, 0.00000000, 0.00038050]
    ]),

    "HR_HFE" : np.array([
        [0.00041540, 0.00000000, -0.00000010],
        [0.00000000, 0.00041637, 0.00004589],
        [-0.00000010, 0.0004589, 0.00002982]
    ]),

    "HR_KFE" : np.array([
        [0.00008508, 0.00000000, 0.00000000],
        [0.00000000, 0.00008580, 0.00000200],
        [0.00000000, 0.00000200, 0.00000139]
    ]),

}

joint_m = {

    "root_joint" : 1.25123725,
    "FL_HAA" : 0.14196048,
    "FL_HFE" : 0.14737324,
    "FL_KFE" : 0.02318294,
    "FR_HAA" : 0.14196048,
    "FR_HFE" : 0.14737324,
    "FR_KFE" : 0.02318294,
    "HL_HAA" : 0.14196048,
    "HL_HFE" : 0.14737324,
    "HL_KFE" : 0.02318294,
    "HR_HAA" : 0.14196048,
    "HR_HFE" : 0.14737324,
    "HR_KFE" : 0.02318294

}

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

#################################

#print(model)

q = np.zeros(model.nq)

q[2] = 0.534
q[4] = 0.7068252
q[6] = 0.7073883 
q[8:18:3] = -1.57

#print("q: %s" % q.T)

pin.framesForwardKinematics(model, data, q)


#for i, name in zip(range(14), model.names):
    #print("---------------------------\n")
    #print("{}\n\n{}\n\n{}".format(name, data.oMi[i], data.Ycrb[i]))
    #print("---------------------------\n")


grupa1 = {"FR_HFE", "FR_KFE", "FL_HFE", "FL_KFE"}
grupa2 = {"root_joint", "FL_HAA", "FR_HAA", "HL_HAA", "HL_HFE", "HL_KFE", "HR_HAA", "HR_HFE", "HR_KFE"}

I1 = 0
I2 = 0

for i in grupa1:
    I1 = I1 + data.oMi[joint_id[i]].rotation*joint_Is[i]*np.transpose(data.oMi[joint_id[i]].rotation) + np.dot(data.oMi[joint_id[i]].translation, data.oMi[joint_id[i]].translation)*joint_m[i]


for i in grupa2:
    I2 = I2 + data.oMi[joint_id[i]].rotation*joint_Is[i]*np.transpose(data.oMi[joint_id[i]].rotation) + np.dot(data.oMi[joint_id[i]].translation - np.array([0, 0, 0.534]), data.oMi[joint_id[i]].translation - np.array([0, 0, 0.534]))*joint_m[i]

print("I1 = {}\nI2 = {}".format(I1, I2))