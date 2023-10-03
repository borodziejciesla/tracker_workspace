# Tracker Workspace
Tracker workspace provides nodes and messages for multi target tracking based on automotive tracker detections.
## Messages
There are available few sets of messages:
### radar_msgs
* Detection
* SensorOrigin
* RadarScan
### radar_processor_msgs
* Pose
* Velocity
* Size
* MovingObject
* ScanObjects
### tracker_msgs
* Pose
* Velocity
* Shape
* Covariance
* Object
* TrackerScan

## Packages
There are few packages:
### radar_preprocessor
This package provides node for preprocessing of radar detections. Which provides one scan based estimation of objects.

### sensors_static_tf_broadcaster
This package provides to other static sensors position in vehicle coordinate system (VCS).

### tracker
This package provides multiple object tracking functionalit.

### visualizations
This package provides nodes for viualization of other packages oututs:
* radar_visualization_node: visualize radar detections
* radar_preprocessor_visualization_node: visualize position and size of prprocessed targets.