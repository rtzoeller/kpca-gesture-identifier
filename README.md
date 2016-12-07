# KPCA Gesture Identifier

## What is it?

KPCA Gesture Identifier is a ROS package for recognizing and matching
gestures and trajectories with training data. It uses Kernel Principal
Component Analysis to separate the training data, and a simple K-nearest
neighbors algorithm to perform classification on this space.

## Usage

The package is comprised of a number of generic source files that do not
depend on ROS, located in `src/`. ROS nodes live in `nodes/`.

Support is included for recognizing mouse movement with in a dedicated
window, although this serves primarily as a demo. It is likely desirable
to replace this front end with a custom one suited for your application.

| Front End/Utility Nodes | Back End Nodes |
| --- | --- |
| MouseTrackerNode | TrajectoryNormalizerNode |
| TrajectorySaverNode | NormalizedTrajectoryMatcherNode |

As part of matching trajectories, both the training data as well as the
test data must be normalized. Built in normalization methods include:

| Name | Description |
| --- | --- |
| `default` | Alias for `linear_time_invariant`, may change. |
| `linear` | Points are resampled uniformly by time &mdash; linear interpolation is used between points. Each axis of the input data is scaled independently. |
| `linear_time_invariant` | Points are resampled uniformly by distance &mdash; linear interpolation is used between points. Each axis of the input data is scaled independently. |
| `linear_scale_uniform` | Like `linear`, except axes are not scaled independently. |
| `linear_time_invariant_scale_uniform` | Like `linear_time_invariant`, except axes are not scaled independently. |
| `none` | Indicates you will handle interpolation yourself. See below. |

If a built in normalization method is sufficient, launching a
`TrajectoryNormalizerNode` and specifying an `interpolation_strategy` on
the parameter server will automatically normalize and convert
`TrajectoryStamped` messages into the `Trajectory` message accepted by
`NormalizedTrajectoryMatcherNode`.

In the case that you want to handle normalization yourself, you must
specify `none` as the `interpolation_strategy` and additionally ensure
all training data has been normalized prior to being saved to disk.