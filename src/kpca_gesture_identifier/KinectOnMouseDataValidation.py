import os

import numpy as np

from kpca_gesture_identifier.Interpolater import normalizeNumpyArray
from kpca_gesture_identifier.KPCA import Predictor

strategy = "default"
path = "."
gestures = ["L", "N", "O", "R", "S", "W"]
count = 0
predictor = Predictor()
for gesture in gestures:
    label = gesture
    gesture = os.path.join(gesture, "Mouse")
    for directory, subdirectories, filePaths in os.walk(os.path.join("data", gesture)):
        for filePath in filePaths:
            fullPath = os.path.join(directory, filePath)
            normalizedTrajectory = normalizeNumpyArray(np.load(fullPath), strategy)
            predictor.addTrajectory(normalizedTrajectory, label)
            count += 1

print("{0} training files found across {1} labels".format(count, len(gestures)))

count = 0
for gesture in gestures:
    label = gesture
    gesture = os.path.join(gesture, "Kinect")
    for directory, subdirectories, filePaths in os.walk(os.path.join("data", gesture)):
        for filePath in filePaths:
            fullPath = os.path.join(directory, filePath)
            normalizedTrajectory = normalizeNumpyArray(np.load(fullPath), strategy)
            prediction, projected = predictor.classify(normalizedTrajectory)
            if prediction != label:
                print("Expected {0}, got {1}".format(label, prediction))
            count += 1

print("{0} validation files found across {1} labels".format(count, len(gestures)))
