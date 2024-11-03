import pinocchio as pin
import numpy as np

urdf_filename = '/media/luka/HDD/solo12/src/robot_properties_solo/src/robot_properties_solo/resources/urdf/solo12.urdf'

model = pin.buildModelFromUrdf(urdf_filename, pin.JointModelFreeFlyer())
data = model.createData()

print(model)

q = np.zeros(model.nq)

q[2] = 0.534
q[4] = 0.7068252
q[6] = 0.7073883 
q[8:18:3] = -1.57

print("q: %s" % q.T)

pin.framesForwardKinematics(model, data, q)


for i, name in zip(range(12), model.names):
    print("---------------------------\n")
    print("{}\n\n{}".format(name, data.oMf[i]))
    print("---------------------------\n")
