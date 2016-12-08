import os
from random import shuffle

import numpy as np

from kpca_gesture_identifier.Interpolater import normalizeNumpyArray
from kpca_gesture_identifier.KPCA import Predictor

numBuckets = 10

strategy = "default"
labels = []
paths = []
path = "."
gestures = ["L", "N", "O", "R", "S", "W"]
count = 0
for gesture in gestures:
    label = gesture
    gesture = os.path.join(gesture, "Mouse")
    for directory, subdirectories, filePaths in os.walk(os.path.join("data", gesture)):
        for filePath in filePaths:
            fullPath = os.path.join(directory, filePath)
            labels.append(label)
            paths.append(fullPath)
            count += 1

print("{0} files found across {1} labels".format(count, len(gestures)))

# Randomly shuffle them and divide them into buckets
buckets = zip(paths, labels)
shuffle(buckets)
buckets = np.array_split(buckets, numBuckets)

for i in range(len(buckets)):
    predictor = Predictor()
    for j in range(len(buckets)):
        if i == j:
            # The ith bucket is not included in training
            continue

        for path, label in buckets[j]:
            normalizedTrajectory = normalizeNumpyArray(np.load(path), strategy)
            predictor.addTrajectory(normalizedTrajectory, label)

    for path, label in buckets[i]:
        normalizedTrajectory = normalizeNumpyArray(np.load(path), strategy)
        prediction, projected = predictor.classify(normalizedTrajectory)
        if prediction != label:
            print("Expected {0}, got {1}".format(label, prediction))

    print("Finished " + str(i))
