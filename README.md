A set of ROS nodes to gather landmarks bearings from an omnicam while moving a robot.

installation:
git-clone it to your ros workspace
rospack profile
rosmake bearings_collector

You will need OpenCV and Eigen libraries installed.

[BearingCollector]
The actual gathering node.
Reads images from the /cam_stream topic and robotposes from the /pose topic.
When the robot has moved (or rotated) enough, the gathering action is triggered.
Gathering consists of:
    • look for landmarks in the image (colored blobs)
    • detect landmarks centroids
    • store the angles that those blobs form with respect to the center of the image (that's because we assume we are using an omnicam)
The output is a directory containing:
    • the captured images
    • the corresponding filtered images with a cross on each detected landmark
    • images showing the blobs associated to each captured image
    • a text file associating each image with the corresponding robot pose and the list of the bearings detected from that pose
Configurations for the HSV parametres and the triggering distance/angle are read from the "collector.conf" file.

[CameraStream]
Captures images from a camera and publishes them to the /cam_stream topic
The user can specify the number of the camera to be used.

[ImageReceiver]
Reads images from the /cam_stream topic and displays them on the screen

[Joy2cmd_vel]
User interface that allows to drive the robot by the use of a joypad. This publishes on the /cmd_vel topic.
Its peculiarity is that it can be customized (I.E. select which axes/button does what) by editing a simple text file.
This is very nice feature if your joypad has a broken lever :D
If you want to get infos about your joypad's axes/buttons numeration, you may find useful a tool named "jstest".
Commands:
	• The ACCEL axes works for moving the robot forward(default behaviour) or backward(if the REAR button is pressed).
	• The REAR button, if pressed, makes the ACCEL axes work in the other direction.
	• the ROTATION axes makes the robot turn both clockwise and counterclockwise.
	• The RECALIB button stores the current position of the axes as the rest position.
	• The HANDBREAK button prevents any movements.
	• The QUIT button quits the program.
Axes/button configurations are read from the "joy.conf" file in the directory where you run the program.

[ConvertOutput]
This is not a ROS node, but it is here because it is used for transforming the output of this program in a format that is readable for my other project "BearingSlam".
A transformation matrix is applied to the robot poses to take into account the relative position of the sensor with respect to the center of the robot (the recorded poses are referred to the robot, not to the camera).