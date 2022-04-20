# EE346-Lab4
Racetrack Lane Following

# Part I: Driving the robot in the racetrack
In the first part, the car will run out of the track in the middle of the picture because the lighter gray line next to it was also judged to be a white line when judging the white line. So in the first part, you only need to turn up the parameters that determine the minimum brightness of the white line, and the robot will not run out of the track again。

Modify line 29 of the code to lower_white = numpy.array([0, 0, 220])（The value of 220 can be changed appropriately）

# Part II: Driving with BEV with Perspective Distortion Correction
Refer to the information linked below to supplement the code in The First Part. 

https://learnopencv.com/homography-examples-using-opencv-python-c/.

Locate the four points on the original image and map the four points to another image. Manually select these four points.
# Part III: Stop Sign with Aruco Marker
Not done yet，to be continued
# Usage
1、Clone the source code

  cd ~/catkin_ws/src


2、Catkin make the lane following package

  cd ..

  catkin_make

3、 Add course models

  export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models

4、Launch the gazebo map

  source ~/catkin_ws/devel/setup.bash

  roslaunch lane_following race_track.launch

5、Run lane following python node

  cd ~/catkin_ws/src/lane_following/scripts/

  chmod +x lane_following_part1.py

  (chmod +x lane_following_part2.py)

  cd ~/catkin_ws

  source devel/setup.bash

  rosrun lane_following lane_following_part1.py

  (rosrun lane_following lane_following_part2.py)
