# aruco-headpose-tracker

This is an aruco pose estimator that will be used for head pose estimation.

An aruco marker, or several, will be placed on a VR headset and used to track pose
of the head.  

The VR headset is part of a larger autonomous vehicle user research
project to understand how people behave with new and autonomous vehicle technologies.

Requirements:

Numpy
OpenCV 3.4+

To use:

You will need to get the calibrations for the camera you will be using by taking
photos of a checkerboard (camera-calibration-checker-board_9x7.pdf).  Use ~10 or so
and place them in the checker-board-pics directory.  Alternatively, you can use the 
default ones but it won't be as accurate.  The camera used was a Logitech C920.

Print out the marker (example) and track the pose.

The pose will be sent to a UDP server, and can be received with the socket_receive.py
script as well.