from __future__ import division, print_function

from scipy.linalg import eigh

import numpy as np
from scipy.spatial.distance import pdist
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sklearn import neighbors

from math import exp


class KPCA(object):
    def __init__(self, trajectories, gamma):
        self.trajectories = trajectories
        self.gamma = gamma
        self.computeKPCA()

    @staticmethod
    def RBF(x1, x2, gamma):
        X = np.array([x1, x2])
        return exp(pdist(X, "sqeuclidean") / -gamma)

    @staticmethod
    def MKR(t1, t2, gamma):
        return (KPCA.RBF(t1[:, 0], t2[:, 0], gamma) + KPCA.RBF(t1[:, 1], t2[:, 1], gamma)) / 2

    def computeKPCA(self):
        K = np.empty((len(trajectories), len(trajectories)))
        for i in range(len(trajectories)):
            for j in range(len(trajectories)):
                K[i, j] = KPCA.MKR(trajectories[i], trajectories[j], self.gamma)

        # Centering the symmetric NxN kernel matrix.
        N = K.shape[0]
        one_n = np.ones((N, N)) / N
        K = K - one_n.dot(K) - K.dot(one_n) + one_n.dot(K).dot(one_n)

        # Obtaining eigenvalues in descending order with corresponding
        # eigenvectors from the symmetric matrix.
        eigvals, eigvecs = eigh(K)

        # Obtaining the i eigenvectors that corresponds to the i highest eigenvalues.
        alphas = np.column_stack((eigvecs[:, -i] for i in range(1, n_components + 1)))
        lambdas = [eigvals[-i] for i in range(1, n_components + 1)]

        self.eigvecs = alphas
        self.eigvals = lambdas
        self.K = K

    def project(self, trajectory_new):
        pair_dist = np.array([KPCA.MKR(trajectory_new, trajectory, self.gamma) for trajectory in trajectories])
        return pair_dist.dot(self.eigvecs / self.eigvals)


if __name__ == '__main__':
    # Read in saved trajectories
    trajectories_L = []
    for i in range(1, 21):
        trajectories_L.append(np.load("data/L/{0}.npy".format(i)))
    trajectories_O = []
    for i in range(1, 21):
        trajectories_O.append(np.load("data/O/{0}.npy".format(i)))
    trajectories_R = []
    for i in range(1, 21):
        trajectories_R.append(np.load("data/R/{0}.npy".format(i)))
    trajectories_S = []
    for i in range(1, 21):
        trajectories_S.append(np.load("data/S/{0}.npy".format(i)))
    trajectories_W = []
    for i in range(1, 21):
        trajectories_W.append(np.load("data/W/{0}.npy".format(i)))

    trajectories = []
    trajectories.extend(trajectories_L)
    trajectories.extend(trajectories_O)
    trajectories.extend(trajectories_R)
    trajectories.extend(trajectories_S)
    trajectories.extend(trajectories_W)
    n_components = 5

    kpca = KPCA(trajectories, 18)

    # Guess a new point
    test = np.load("out.npy")
    test_projected = kpca.project(test)

    classifier = neighbors.KNeighborsClassifier(n_neighbors=3)
    classifier.fit(kpca.eigvecs,
                   ["L"] * len(trajectories_L) + ["O"] * len(trajectories_O) + ["R"] * len(trajectories_R) +
                   ["S"] * len(trajectories_S) + ["W"] * len(trajectories_W))
    print(classifier.predict(test_projected.reshape(1, -1)))

    # Show plots
    plt.imshow(kpca.K, interpolation="nearest")
    plt.colorbar()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(kpca.eigvecs[:, 0], kpca.eigvecs[:, 1], kpca.eigvecs[:, 2],
               c=["r"] * len(trajectories_L) +
                 ["b"] * len(trajectories_O) +
                 ["k"] * len(trajectories_R) +
                 ["g"] * len(trajectories_S) +
                 ["c"] * len(trajectories_W),
               marker="o")
    ax.scatter(test_projected[0], test_projected[1], test_projected[2], c="y", marker="^")
    plt.show()
