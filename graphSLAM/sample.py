import math
import copy
import itertools
import numpy as np 
import cvxpy as cp
import matplotlib.pyplot as plt


np.random.seed(10)
R = 0.2
Q = 0.5
N = 2
graphics_radius = 0.1

odom = np.empty((N,1))
obs = np.empty((N,1))

landmark = 3
odom[0] = 0.2
odom[1] = 0.8
obs[0] = 3.3
obs[1] = 1.9

plot_robot_1d(odom)
plt.plot(landmark,0, '*k', markersize=30)
plt.plot(obs+odom, np.zeros(len(obs)), '.', markersize=30, color='brown')
plt.text(odom[0], 0.02, "X_0", fontsize=12)
plt.text(odom[1], 0.02, "X_1", fontsize=12)
plt.text(obs[0]+odom[0], 0.02, "z_0", fontsize=12)
plt.text(obs[1]+odom[1], 0.02, "z_1", fontsize=12)
plt.show()


constraints = {}
omegas = {}

zids = list(itertools.combinations(range(N),2))
for (t1, t2) in zids:
    print("Node combination: ", (t1, t2))
    x1 = odom[t1]
    x2 = odom[t2]
    z1 = obs[t1]
    z2 = obs[t2]
    constraints[(t1,t2)] = (x2-x1-z1+z2)
    omegas[(t1,t2)] = (1 / (2*Q))

H = np.zeros((N,N))
b = np.zeros((N,1))


for (t1, t2) in zids:
    H[t1,t1] += omegas[(t1,t2)]
    H[t2,t2] += omegas[(t1,t2)]
    H[t2,t1] -= omegas[(t1,t2)]
    H[t1][t2] -= omegas[(t1,t2)]

    b[t1]   -= constraints[(t1,t2)]
    b[t2]   += constraints[(t1,t2)]

print(H)
print("The determinant of H: ", np.linalg.det(H))
H[(0,0)] +=1
print(H)
print("The determinant of H after adding initial contraints: ", np.linalg.det(H))

print(b)
dx = np.linalg.inv(H) @ b
