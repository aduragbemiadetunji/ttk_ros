#!/usr/bin/env python3
# import numpy as np

# # Constants
# g = 9.81  # Gravity, m/s^2
# dx = 0.0  # Drag in the x-direction, can be set accordingly
# dy = 0.0  # Drag in the y-direction, can be set accordingly

# # Example values for acceleration 'a' and reference heading 'psi'
# # These values should be determined by your specific application
# a = np.array([0, 0, 0])  # Acceleration vector, m/s^2
# psi = np.radians(30)  # Reference heading in degrees, converted to radians

# # Calculate the reference direction vectors in the world frame
# xC = np.array([np.cos(psi), np.sin(psi), 0])
# yC = np.array([-np.sin(psi), np.cos(psi), 0])

# # Gravity vector
# gzW = np.array([0, 0, -g])

# # Drag components (assuming v is the velocity vector)
# v = np.array([0, 0, 0])  # This should be the current velocity of the quadrotor
# dxv = dx * v
# dyv = dy * v

# # Calculate alpha and beta as per equations (38) and (39)
# alpha = a + gzW + dxv
# beta = a + gzW + dyv

# # Construct the body-frame axes
# xB = np.cross(yC, alpha)
# xB /= np.linalg.norm(xB)  # Normalize the vector

# yB = np.cross(beta, xB)
# yB /= np.linalg.norm(yB)  # Normalize the vector

# zB = np.cross(xB, yB)
# # zB is already normalized since it's the cross product of two normalized, perpendicular vectors

# # Construct the rotation matrix R from the body-frame axes
# R = np.vstack((xB, yB, zB)).T

# # Now R is the rotation matrix representing the quadrotor orientation
# print(R)



import numpy as np

# Constants and flat outputs (example values, these should be replaced with your actual values)
a = np.array([0, 0, 0])  # Acceleration vector
g = 9.81  # Gravitational acceleration
zW = np.array([0, 0, 1])
v = np.array([0, 0, 0])  # Desired velocity vector
dx, dy, dz = 0, 0, 0  # Drag coefficients
psi = 0  # Desired heading
kh = 0  # Some constant from thrust model

# Compute xC and yC vectors
xC = np.array([np.cos(psi), np.sin(psi), 0])
yC = np.array([-np.sin(psi), np.cos(psi), 0])

# Equations (38) and (39)
alpha = a + g * zW + dx * v
beta = a + g * zW + dy * v

# Compute the body-fixed frame vectors (xB, yB, zB)
xB = np.cross(yC, alpha)
xB /= np.linalg.norm(xB)

yB = np.cross(beta, xB)
yB /= np.linalg.norm(yB)

zB = np.cross(xB, yB)

# Assemble the rotation matrix R
R = np.column_stack((xB, yB, zB))

# Collective thrust (equation 41)
c = np.dot(zB, a + g * zW + dz * v)

# Adjusted collective thrust command
ccmd = c - kh * (np.dot(v, xB + yB))**2

print(R, ccmd)

# At this point, you would publish 'R' and 'ccmd' to the respective ROS topics.
