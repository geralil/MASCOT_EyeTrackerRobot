# MASCOT_EyeTrackerRobot
C/C++ based Arduino code to control robotic eye tracker testing rig. 

***Background***

The purpose of the project was to develop a robot that can be used to automate the testing of eye trackers. These eye trackers are a great resource for people living through advanced stages of Amyotrophic Lateral Sclerosis(ALS), as it enables them to speak and write using their computers.
However, little work has been done to standardize the testing or benchmark eye trackers from different manufacturers for precision or accuracy. This sort of benchmarking would require automation of testing, which was the objective of building this robot. 

The robot consists of 3 sets of steppers that control the X,Y,Z axes of the robot simulating the shoulder. Linear translation along these axes moves the face of the robot that consists of the eye subsystem of the robot. The eye subsystem plays the critical role of controlling the position of the cursor on the screen using the eye tracker. The 2 eyes of the robot are allowed to oscillate on pitch and yaw axis using 2 pairs of servos. A PC fitted with an eye tracker is mounted on the end of the rig, roughly 60cm away from the center of stepper axes origin. 

In order to control the cursor on the PC, it is vital for both eyes of the robot to converge or parallax on a single point on the screen. This is attained by feeding angles to the servos that are trigonometrically obtained from the coordinates of the point on the screen, and the distance between the face and the screen. Alpha and beta angles are obtained for calculating the pitch and yaw angles of the servos. For more details on the algorithm for finding these angles, refer to Servo.h section of this document.

An arduino microcontroller is used to control the different modes of the robot and to process information for the translation of steppers and servos. The code that controls the microcontroller operates on the principle of state machine. The robot can be set in 7 different states or modes: Menu, Calibration, Stepper Control, Servo Control, Homing, FindCoordinate and SetCoordinate state. For details on the states, refer to State Machine section of this document. 

The robot's mechanical design and the initial code base is based off of work done by Harsft et al. (https://github.com/Leaf-Harvest/Eye-Tracker-Test-Rig).
Modifications to the code have been implemented to achieve the following functions on a high level: 
	- One-Time Calibration of the steppers and eye mount servos by writing information to EEPROM
	- Improve accuracy of parallax function to reduce noise in cursor position on screen
	- Modify the logic of the program to adapt to changes in electrical components and wiring schematics
	- Implement different calibration sequence for servos for new parallax logic
	- Implement a virtual DotPosition matrix the size of the computer screen to accurately rotate servos to desired coordinates on screen

***Code Notes***



