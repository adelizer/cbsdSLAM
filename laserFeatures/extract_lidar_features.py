"""
Provided with a LiDAR data, extract discernible features in the environment
"""

import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import os

data_path = "data/"
filename = "scan_nparray.npy"


def visualize_input_data(data):
    plt.figure()
    idx = np.random.randint(1,data.shape[0],4)
    for i in range(len(idx)):
        plt.subplot(2,2,i+1)
        plt.plot(data[i,0,:], data[i,1,:], '.r', markersize=5)
        plt.grid()


def remove_nans(data):
    data[np.isneginf(data)] = 0.0
    data[np.isinf(data)] = 0.0
    data[np.isnan(data)] = 0.0
    return data


def main():
    scan_reader = os.path.join(data_path, filename)
    scans = np.load(scan_reader)
    scans[np.isinf(scans)] = 0.0
    sample = remove_nans(scans[0,:,:]).transpose()
    print(sample.shape, np.max(sample), np.min(sample))
    visualize_input_data(scans)
    kmeans = KMeans(n_clusters=6)
    y_pred = kmeans.fit_predict(sample)
    plt.figure()
    plt.grid()
    plt.scatter(sample[:, 0], sample[:, 1], c=y_pred)
    plt.show()



if __name__ == "__main__":
    main()