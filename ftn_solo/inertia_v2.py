import pinocchio as pin
from pinocchio.utils import *
import numpy as np


def compute_equivalent_inertia_with_rotation(selected_segments, equivalent_com, model, data):
    equivalent_inertia = np.zeros((3, 3))

    for frame_idx in selected_segments:
        frame = model.frames[model.getFrameId(frame_idx)]
        body = model.inertias[frame.parentJoint]
        frame_placement = data.oMf[model.getFrameId(frame_idx)]

        # Transform inertia to the world frame
        rotation = frame_placement.rotation
        body_inertia_world = rotation @ body.inertia @ rotation.T

        # Transform CoM to world coordinates
        body_com_world = frame_placement.translation + rotation @ body.lever

        # Offset vector from the segment CoM to the equivalent CoM
        offset = body_com_world - equivalent_com

        # Parallel axis theorem
        inertia_offset = body.mass * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
        equivalent_inertia += body_inertia_world + inertia_offset

    return equivalent_inertia

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

total_mass = 0.0
weighted_com_sum = np.zeros(3)

for frame_idx in grupa1:
    # Get frame inertia and placement
    frame = model.frames[model.getFrameId(frame_idx)]
    body = model.inertias[frame.parentJoint]
    frame_placement = data.oMf[model.getFrameId(frame_idx)]  # Transformation from frame to world

    # Extract rotation and translation
    rotation = frame_placement.rotation
    translation = frame_placement.translation

    # Transform CoM to world coordinates
    body_mass = body.mass
    local_com = body.lever
    world_com = translation + rotation @ local_com

    # Accumulate weighted sum
    weighted_com_sum += body_mass * world_com
    total_mass += body_mass

# Compute the equivalent CoM
equivalent_com1 = weighted_com_sum / total_mass
print("Equivalent CoM1 (with rotations):", equivalent_com1)


I1 = compute_equivalent_inertia_with_rotation(grupa1, equivalent_com1, model, data)
print(I1)


total_mass2 = 0.0
weighted_com_sum2 = np.zeros(3)

for frame_idx in grupa2:
    # Get frame inertia and placement
    frame2 = model.frames[model.getFrameId(frame_idx)]
    body2 = model.inertias[frame.parentJoint]
    frame_placement2 = data.oMf[model.getFrameId(frame_idx)]  # Transformation from frame to world

    # Extract rotation and translation
    rotation2 = frame_placement2.rotation
    translation2 = frame_placement2.translation

    # Transform CoM to world coordinates
    body_mass2 = body2.mass
    local_com2 = body2.lever
    world_com2 = translation2 + rotation2 @ local_com2

    # Accumulate weighted sum
    weighted_com_sum2 += body_mass2 * world_com2
    total_mass2 += body_mass2

# Compute the equivalent CoM
equivalent_com2 = weighted_com_sum2 / total_mass2
print("Equivalent CoM2 (with rotations):", equivalent_com2 - np.array([0,0,0.3375]))

I2 = compute_equivalent_inertia_with_rotation(grupa2, equivalent_com2, model, data)
print(I2)