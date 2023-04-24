# Steel-Beam-Climbing-Robot
This was a Senior Design Project in Spring 2023 at Oakland University, Rochester Hills, MI. The goal of this project was to develop a Autonomous Steel Climbing Robot.

The goal of this project was to develop a Autonomous Steel Climbing Robot that could perform
these tasks by itself:

1. Find Beam
2. Climb Beam
3. Ring Bell
4. Come Down
5. Dismount
6 Home from start

In general, the system relies on Jetson Nano that runs the vision python script to detect the beam and
the bell. The models used for the object detection are going to be unique to your situation, a good place
to learn Object Detection and Machine Learning on the Jetson Nano is the Nvidia Jetson Nano Inference 
Github found at https://github.com/dusty-nv/jetson-inference. 

Within the visions script, the Jetson Nano runs ROS nodes to publish and subscribe the topics of data.
To learn more about ROS, I suggest the wiki page found at http://wiki.ros.org/Documentation. 

Moreover, the system also runs a Arduino Mega that is the primary controller and state holder of the robot.
It also runs ROS using the ros.h library, and communicates with the Jetson Nano by using ROSserial. ROSserial
is a package that is ran on the Jetson Nano, to initiate serial communication. That can also be found on the Wiki.

The robot uses a variety of DC Motors with encoders, servos, distance sensors, magnetic wheels, and general
power system components.

Pictures and Videos will also be provided in this repository

Keep in mind that this project was developed by students, so there may be inefficiencies, bugs, 
or non-standard code practices. So take everything with a grain of salt and adapt it as needed 
to your work.

There is no plan to continue developing the project, the code will be provided AS IS.  
