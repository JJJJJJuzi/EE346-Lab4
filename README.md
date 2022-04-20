# EE346-Lab4
Racetrack Lane Following

Part I: Driving the robot in the racetrack
In the first part, the car will run out of the track in the middle of the picture because the lighter gray line next to it was also judged to be a white line when judging the white line. So in the first part, you only need to turn up the parameters that determine the minimum brightness of the white line, and the robot will not run out of the track again。
Modify line 29 of the code to lower_white = numpy.array([0, 0, 220])（The value of 220 can be changed appropriately）
