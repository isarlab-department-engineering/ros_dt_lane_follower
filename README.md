# ros-lane-detection
ROS nodes for lane detection based on computer vision algorithms

- lane_detection.py: scans the image captured by the camera looking for white, yellow and red lines and calculates the error in 
  pixel from the middle of the lane. It subscribes the "image_topic" topic and publishes the error value on the "lane_detection" 
  topic.
- lane_controller.py: contains the PID controller and sets motors' speed. It subscribes the "lane_detection" topic and publishes 
  linear and angular velocities of the robot on the "follow_topic" topic.
