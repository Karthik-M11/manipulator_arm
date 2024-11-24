# File contains forward and inverse kinematics functions for a 5 dof arm
# Forward kinematics finds the end-effector position based on the input link lengths and angles
# Inverse kinematics finds the joint angles from the input link lengths and end effector position
# arduino_comm enables communication with arduino


# import necessary functions
import math
import numpy as np
from scipy.optimize import minimize
import serial


PI = np.pi


class Kinematics:
    def __init__(self, link_lengths, offset, t_angles):
        self.t_end_matrix = np.zeros((4,4))
        self.link_lengths = link_lengths
        self.offset = offset
        self.t_angles = t_angles


    def dh_transform(self, theta, a, d, alpha):
        '''
        Function definition
        '''
        t_matrix = np.array(
            [
                [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1],
            ]
        )
        return t_matrix
    

    def forward_kinematics(self, j_angles):
        '''
        Definition
        '''
        theta1, theta2, theta3, theta4, theta5 = j_angles
        a1, a2, a3, a4, a5 = self.link_lengths
        d1, d2, d3, d4, d5 = self.offset
        alpha1, alpha2, alpha3, alpha4, alpha5 = self.t_angles

        t_matrix_1 = self.dh_transform(theta1, a1, d1, alpha1)
        t_matrix_2 = self.dh_transform(theta2, a2, d2, alpha2)
        t_matrix_3 = self.dh_transform(theta3, a3, d3, alpha3)
        t_matrix_4 = self.dh_transform(theta4, a4, d4, alpha4)
        t_matrix_5 = self.dh_transform(theta5, a5, d5, alpha5)

        temp_array = [t_matrix_2, t_matrix_3, t_matrix_4, t_matrix_5]
        t_end = t_matrix_1

        for i in range(4):
            t_end = np.dot(t_end, temp_array[i])

        self.t_end_matrix = t_end
        x, y, z = t_end[0][-1], t_end[1][-1], t_end[2][-1]
        roll, pitch = np.arccos(t_end[0][0]), np.arcsin(t_end[1][0])        

        return [x, y, z, roll, pitch]
    

    def inverse_kinematics(self, coordinates):
        '''
        Description
        '''
        coordinates = np.array(coordinates)

        def objective_function(joint_angles):
            actual = np.array(self.forward_kinematics(joint_angles))
            error = np.linalg.norm(actual - coordinates)
            return error

        initial_guess = np.zeros(5)
        result = minimize(objective_function, initial_guess, method='BFGS') 

        return result


    def arduino_comm(self):
        pass


kinematics = Kinematics(
    [0, 1, 1, 0, 0], 
    [1, 0, 0, 0, 1],
    [PI/2, 0, 0, -PI/2, 0]
    )

result = kinematics.forward_kinematics([0,0,0,0,0])
print(result)