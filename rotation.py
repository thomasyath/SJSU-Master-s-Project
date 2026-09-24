
import numpy as np

# Rotation matrix
rotation_matrix = np.array([[np.cos(2.593),-np.sin(2.593),0],
                                    [np.sin(2.593), np.cos(2.593),0],
                                    [0,0,1]])

def ft_rotate(force_torque):
    corrected_data = np.reshape(force_torque, (6, 1))
    force_torque_rotated = np.vstack((
        np.dot(rotation_matrix[0], corrected_data[:3]),
        -np.dot(rotation_matrix[1], corrected_data[:3]),
        np.dot(rotation_matrix[2], corrected_data[:3]),
        np.dot(rotation_matrix[0], corrected_data[3:]),
        -np.dot(rotation_matrix[1], corrected_data[3:]),
        np.dot(rotation_matrix[2], corrected_data[3:]),
    ))
    force_torque_rotated = np.round(force_torque_rotated.ravel())
    return force_torque_rotated