import numpy as np
import matplotlib.pyplot as plt
import itertools
np.random.seed(10)
R = 0.2
Q = 0.2
N = 3
graphics_radius = 0.1

odom = np.empty((N,1))
obs = np.empty((N,1))
x_true = np.empty((N,1))

landmark = 3
x_true[0], odom[0], obs[0] = 0.0, 0.0, 2.9
x_true[1], odom[1], obs[1] = 1.0, 1.5, 2.0
x_true[2], odom[2], obs[2] = 2.0, 2.4, 1.0

plt.plot(landmark,0, '*k', markersize=30)

for i in range(N):
    plt.plot(odom[i], 0, '.', markersize=50, alpha=0.8, color='steelblue')
    plt.plot([odom[i], odom[i] + graphics_radius],
             [0,0], 'r')
    plt.text(odom[i], 0.02, "X_{}".format(i), fontsize=12)
    plt.plot(obs[i]+odom[i],0,'.', markersize=25, color='brown')
    plt.plot(x_true[i],0,'.g', markersize=20)
plt.show()


def get_H_b(odom, obs):
    """
    Create the information matrix and information vector. This implementation is 
    based on the concept of virtual measurement i.e. the observations of the
    landmarks are converted into constraints (edges) between the nodes that
    have observed this landmark.
    """
    constraints = {}
    omegas = {}
    zids = list(itertools.combinations(range(N),2))
    for (t1, t2) in zids:
#         print("Node combination: ", (t1, t2))
        x1 = odom[t1]
        x2 = odom[t2]
        z1 = obs[t1]
        z2 = obs[t2]
        
        # Adding virtual measurement constraint
        constraints[(t1,t2)] = (x2-x1-z1+z2)
        omegas[(t1,t2)] = (1 / (2*Q))

    H = np.zeros((N,N))
    b = np.zeros((N,1))


    for (t1, t2) in zids:
        H[t1,t1] += omegas[(t1,t2)]
        H[t2,t2] += omegas[(t1,t2)]
        H[t2,t1] -= omegas[(t1,t2)]
        H[t1,t2] -= omegas[(t1,t2)]

        b[t1] += constraints[(t1,t2)]
        b[t2] -= constraints[(t1,t2)]

    return H, b


H, b = get_H_b(odom, obs)
print("The determinant of H: ", np.linalg.det(H))
H[0,0] += 1 # np.inf ?
print("The determinant of H after anchoring constraint: ", np.linalg.det(H))

for i in range(5):
    H, b = get_H_b(odom, obs)
    H[(0,0)] += 1
    
    # Recover the posterior over the path
    dx = np.linalg.inv(H) @ b
    odom += dx
    # repeat till convergence
    
print("Odometry values after running optimzation: ", odom)

plt.figure()
plt.plot(x_true, np.zeros(x_true.shape), '.', markersize=20, label='Ground truth')
plt.plot(odom, np.zeros(x_true.shape), '.', markersize=20, label='Estimation')
plt.legend()
plt.show()
