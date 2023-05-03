import numpy as np

d = [0.1575, 0.2025, 0.2045, 0.2155, 0.1845, 0.2155, 0.081, 0.02] ## link lengths in mm

vectors_to_com = (np.matrix([-0.1, 0, 0.07]).T,
                  np.matrix([0, -0.03, 0.12]).T,
                  np.matrix([0.0003, 0.059, 0.042]).T,
                  np.matrix([0, 0.03, 0.13]).T,
                  np.matrix([0, 0.067, 0.034]).T,
                  np.matrix([.0001, .021, 0.076]).T,
                  np.matrix([0, .0006, .0004]).T,
                  np.matrix([0, 0, .02]).T)

link_mass = (5, 4, 4, 3, 2.7, 1.7, 1.8, 1.0)
inertia_tensors = (np.diag([.05, .06, .03]),
                   np.diag([.1, .09, .02]),
                   np.diag([.05, .018, .044]),
                   np.diag([.08, .075, .01]),
                   np.diag([.03, .01, .029]),
                   np.diag([.02, .018, .005]),
                   np.diag([.005, .0036, .0047]),
                   np.diag([.001, .001, .001]))

def get_frame_translation_vectors():
    translation_vectors = []

    translation_vector = np.matrix([0, 0, d[0]]).T
    translation_vectors.append(translation_vector)

    translation_vector = np.matrix([0, 0, d[1]]).T
    translation_vectors.append(translation_vector)

    translation_vector = np.matrix([0, d[2], 0]).T
    translation_vectors.append(translation_vector)

    translation_vector = np.matrix([0, 0, d[3]]).T
    translation_vectors.append(translation_vector)

    translation_vector = np.matrix([0, d[4], 0]).T
    translation_vectors.append(translation_vector)

    translation_vector = np.matrix([0, 0, d[5]]).T
    translation_vectors.append(translation_vector)

    translation_vector = np.matrix([0, d[6], 0]).T
    translation_vectors.append(translation_vector)

    translation_vector = np.matrix([0, 0, d[7]]).T
    translation_vectors.append(translation_vector)

    return translation_vectors
    
def get_frame_rotation_matrices(joint_angles):
    rotation_matrices = []

    rotation_matrix = np.matrix([[np.cos(joint_angles[0]), -np.sin(joint_angles[0]), 0],
                                 [np.sin(joint_angles[0]), np.cos(joint_angles[0]), 0],
                                 [0, 0, 1]])
    rotation_matrices.append(rotation_matrix)
    
    rotation_matrix = np.matrix([[-np.cos(joint_angles[1]), np.sin(joint_angles[1]), 0],
                                 [0, 0, 1],
                                 [np.sin(joint_angles[1]), np.cos(joint_angles[1]), 0]])
    rotation_matrices.append(rotation_matrix)

    rotation_matrix = np.matrix([[-np.cos(joint_angles[2]), np.sin(joint_angles[2]), 0],
                                 [0, 0, 1],
                                 [np.sin(joint_angles[2]), np.cos(joint_angles[2]), 0]])
    rotation_matrices.append(rotation_matrix)

    rotation_matrix = np.matrix([[np.cos(joint_angles[3]), -np.sin(joint_angles[3]), 0],
                                 [0, 0, -1],
                                 [np.sin(joint_angles[3]), np.cos(joint_angles[3]), 0]])
    rotation_matrices.append(rotation_matrix)

    rotation_matrix = np.matrix([[np.cos(joint_angles[4]), -np.sin(joint_angles[4]), 0],
                                 [0, 0, 1],
                                 [-np.sin(joint_angles[4]), -np.cos(joint_angles[4]), 0]])
    rotation_matrices.append(rotation_matrix)

    rotation_matrix = np.matrix([[-np.cos(joint_angles[5]), np.sin(joint_angles[5]), 0],
                                 [0, 0, 1],
                                 [np.sin(joint_angles[5]), np.cos(joint_angles[5]), 0]])
    rotation_matrices.append(rotation_matrix)

    rotation_matrix = np.matrix([[-np.cos(joint_angles[6]), np.sin(joint_angles[6]), 0],
                                 [0, 0, 1],
                                 [np.sin(joint_angles[6]), np.cos(joint_angles[6]), 0]])
    rotation_matrices.append(rotation_matrix)

    rotation_matrix = np.matrix([[1, 0, 0],
                                 [0, 1, 0],
                                 [0, 0, 1]])
    rotation_matrices.append(rotation_matrix)

    return rotation_matrices
        
def outward_iterations(number_of_joints, angles, velocities, accelerations, gravity=10):
    previous_linear_acceleration = np.matrix([0,0,gravity]).T
    previous_angular_velocity = np.matrix([0,0,0]).T
    previous_angular_acceleration = np.matrix([0,0,0]).T
        
    translation_vectors = get_frame_translation_vectors()
    rotation_matrices = get_frame_rotation_matrices(angles)

    link_forces = []
    link_torques = []
    for joint_index in range(number_of_joints):
        joint_angle = angles[joint_index]
        joint_velocity = velocities[joint_index]
        joint_acceleration = accelerations[joint_index]

        translation_vector = translation_vectors[joint_index]
        rotation_matrix = rotation_matrices[joint_index].T
        vector_to_com = vectors_to_com[joint_index+1]

        ## compute angular velocity and acceleration
        angular_velocity = rotation_matrix*previous_angular_velocity \
            + np.matrix([0,0,joint_velocity]).T
        angular_acceleration = rotation_matrix*previous_angular_acceleration \
            + np.cross(rotation_matrix*previous_angular_velocity, np.matrix([0,0,joint_velocity]).T, axis=0) \
            + np.matrix([0,0,joint_acceleration]).T

        ## compute linear acceleration
        linear_acceleration = \
            rotation_matrix*(np.cross(previous_angular_acceleration, translation_vector, axis=0) \
                             + np.cross(previous_angular_velocity, np.cross(previous_angular_velocity, translation_vector, axis=0), axis=0) \
                             + previous_linear_acceleration)

        linear_com_acceleration = np.cross(angular_acceleration, vector_to_com, axis=0) \
            + np.cross(angular_velocity, np.cross(angular_velocity, vector_to_com, axis=0), axis=0) \
            + linear_acceleration

        ## compute link force and torque
        link_force = link_mass[joint_index+1]*linear_com_acceleration
        link_torque = inertia_tensors[joint_index+1]*angular_acceleration \
            + np.cross(angular_velocity, inertia_tensors[joint_index+1]*angular_velocity, axis=0)

        link_forces.append(link_force)
        link_torques.append(link_torque)

        previous_linear_acceleration = linear_acceleration
        previous_angular_velocity = angular_velocity
        previous_angular_acceleration = angular_acceleration

    return link_forces, link_torques

def inward_iterations(number_of_joints, angles, link_forces, link_torques):
    previous_force = np.matrix([0,0,0]).T
    previous_torque = np.matrix([0,0,0]).T

    translation_vectors = get_frame_translation_vectors()
    rotation_matrices = get_frame_rotation_matrices(angles)

    joint_actuator_torques = np.zeros(number_of_joints)
    for joint_index in reversed(range(number_of_joints)):
        translation_vector = translation_vectors[joint_index+1]
        rotation_matrix = rotation_matrices[joint_index+1]
        vector_to_com = vectors_to_com[joint_index+1]

        link_force = link_forces[joint_index]
        link_torque = link_torques[joint_index]
        
        force = rotation_matrix*previous_force + link_force
        torque = link_torque \
            + rotation_matrix*previous_torque \
            + np.cross(vector_to_com, link_force, axis=0) \
            + np.cross(translation_vector, rotation_matrix*previous_force, axis=0)

        joint_actuator_torques[joint_index] = (float(torque[-1]))

        previous_force = force
        previous_torque = torque

    return joint_actuator_torques

def iterative_ne_algorithm(number_of_joints, angles, velocities, accelerations, gravity=10):
    link_forces, link_torques = outward_iterations(number_of_joints, angles, velocities, accelerations, gravity)

    joint_actuator_torques = inward_iterations(number_of_joints, angles, link_forces, link_torques)

    return joint_actuator_torques
    

# def compute_torques(pb_client, robot_id, joint_angles, joint_velocities, accelerations):
#     joint_torques = pb_client.calculateInverseDynamics(bodyUniqueId=robot_id,
#                                                        objPositions=joint_angles,
#                                                        objVelocities=joint_velocities,
#                                                        objAccelerations=accelerations)

#     return joint_torques

# def compute_control_gain():
    
    
