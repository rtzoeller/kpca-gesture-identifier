from __future__ import division, print_function

from scipy.linalg import eigh

import numpy as np
from scipy.spatial.distance import pdist
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sklearn import neighbors

from math import exp

from Interpolater import normalizeNumpyArray


class KPCA(object):
    def __init__(self, trajectories, n_components, gamma):
        self.trajectories = trajectories
        self.n_components = n_components
        self.gamma = gamma
        self.computeKPCA()
        # Call computeKPCA() to populate these values
        self.eigvecs = None
        self.eigvals = None
        self.K = None

    @staticmethod
    def RBF(x1, x2, gamma):
        X = np.array([x1, x2])
        return exp(pdist(X, "sqeuclidean") / -gamma)

    @staticmethod
    def MKR(t1, t2, gamma):
        return (KPCA.RBF(t1[:, 0], t2[:, 0], gamma) + KPCA.RBF(t1[:, 1], t2[:, 1], gamma)) / 2

    def computeKPCA(self):
        """
        Makes use of an algorithm listed by Sebastian Raschka
        (http://sebastianraschka.com/Articles/2014_kernel_pca.html), modified to use the multiple kernel representation
        proposed by Ramirez-Giraldo et al.
        """
        K = np.empty((len(self.trajectories), len(self.trajectories)))
        for i in range(len(self.trajectories)):
            for j in range(len(self.trajectories)):
                K[i, j] = KPCA.MKR(self.trajectories[i], self.trajectories[j], self.gamma)

        # Centering the symmetric NxN kernel matrix.
        N = K.shape[0]
        one_n = np.ones((N, N)) / N
        K = K - one_n.dot(K) - K.dot(one_n) + one_n.dot(K).dot(one_n)

        # Obtaining eigenvalues in descending order with corresponding
        # eigenvectors from the symmetric matrix.
        eigvals, eigvecs = eigh(K)

        # Obtaining the i eigenvectors that corresponds to the i highest eigenvalues.
        alphas = np.column_stack((eigvecs[:, -i] for i in range(1, self.n_components + 1)))
        lambdas = [eigvals[-i] for i in range(1, self.n_components + 1)]

        self.eigvecs = alphas
        self.eigvals = lambdas
        self.K = K

    def project(self, trajectory_new):
        pair_dist = np.array([KPCA.MKR(trajectory_new, trajectory, self.gamma) for trajectory in self.trajectories])
        return pair_dist.dot(self.eigvecs / self.eigvals)


class Predictor(object):
    def __init__(self, trajectories=None, labels=None, classifier=None, gamma=18):
        if (trajectories is None and labels is not None) or \
                (trajectories is not None and labels is None) or \
                (trajectories is not None and labels is not None and (trajectories) != len(labels)):
            raise ValueError("trajectories and labels must have the same size")

        self.trajectories = [] if trajectories is None else trajectories
        self.labels = [] if labels is None else labels
        self.classifier = neighbors.KNeighborsClassifier(n_neighbors=3) if classifier is None else classifier
        self.gamma = gamma
        self.kpca = None

    def addTrajectory(self, trajectory, label):
        if trajectory is None or label is None:
            raise ValueError("trajectory and label must not be None")

        self.trajectories.append(trajectory)
        self.labels.append(label)
        self.kpca = None

    def classify(self, trajectory):
        if self.kpca is None:
            unique_labels = set(self.labels)
            self.kpca = KPCA(self.trajectories, len(unique_labels), self.gamma)
            self.classifier.fit(self.kpca.eigvecs, self.labels)

        projected = self.kpca.project(trajectory)
        return self.classifier.predict(projected.reshape(1, -1))[0], projected


if __name__ == '__main__':
    import os

    # Read in saved trajectories
    strategy = "linear_time_invariant"
    predictor = Predictor()
    path = "."
    gestures = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    counts = [0] * len(gestures)
    middleCounts = [0] * len(counts)
    for i, gesture in enumerate(gestures):
        for directory, subdirectories, filePaths in os.walk(os.path.join("degenerate_data", gesture)):
            for filePath in filePaths:
                fullPath = os.path.join(directory, filePath)
                normalizedTrajectory = normalizeNumpyArray(np.load(fullPath), strategy)
                predictor.addTrajectory(normalizedTrajectory, gesture)
                counts[i] += 1

    for i in range(len(gestures)):
        if i == 0:
            middleCounts[i] = counts[i] / 2
        else:
            middleCounts[i] = middleCounts[i - 1] + counts[i - 1]

    # Guess a new point
    normalizedNumpyTrajectory = normalizeNumpyArray(np.load("out.npy"), strategy)
    prediction, projection = predictor.classify(normalizedNumpyTrajectory)
    print(prediction)

    # Show plots
    plt.imshow(predictor.kpca.K, interpolation="nearest")
    ax = plt.gca()
    ax.set_xticks(middleCounts)
    ax.set_xticklabels(gestures)
    ax.set_yticks(middleCounts)
    ax.set_yticklabels(gestures)
    plt.colorbar()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(predictor.kpca.eigvecs[:, 0], predictor.kpca.eigvecs[:, 1], predictor.kpca.eigvecs[:, 2],
            c=map(lambda l: {"N": "r", "NE": "m", "E": "b", "SE": "k", "S": "g", "SW": "c", "W": "y", "NW": "orange"}[l], predictor.labels),
               marker="o")
    ax.scatter(projection[0], projection[1], projection[2], c="y", marker="^")
    plt.show()
