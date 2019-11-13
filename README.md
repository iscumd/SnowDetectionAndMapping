# SnowDetectionAndMapping
Detect Snow on the ground and create occupancy grid
## Running Node
To run snow detection make sure to have a test image in your home directory "test.jpg".
```bash
cd <catkin_ws>
roscore
rosrun SnowDetectionAndMapping snow_detector_test_pub_node
rosrun SnowDetectionAndMapping snow_detector_node
```
