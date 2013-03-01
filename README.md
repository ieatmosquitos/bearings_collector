A set of ROS nodes to gather landmarks bearings from an omnicam while moving a robot.

[BearingCollector]
The actual gathering node.
Reads images from the /cam_stream topic and robotposes from the /pose topic.
When the robot has moved (or rotated) enough, the gathering action is triggered.
Gathering consists of:
    look for landmarks in the image (colored blobs)
    detect landmarks centroids
    store the angles that those blobs form with respect to the center of the image (that's because we assume we are using an omnicam)
The output is a directory containing:
    the captured images
    the corresponding filtered images with a cross on each detected landmark
    images showing the blobs associated to each captured image
    a text file associating each image with the corresponding robot pose and the list of the bearings detected from that pose

[CameraStream]
Captures images from a camera and publishes them to the /cam_stream topic

[ImageReceiver]
Reads images from the /cam_stream topic and displays them on the screen

[Joy2cmd_vel]
User interface that allows to drive the robot by the use of a joypad. This publishes on the /cmd_vel topic.
Its peculiarity is that it can be customized (I.E. select which axes/button does what) by editing a simple text file.
This is very nice feature if your joypad has a broken lever :D
If you want to get infos about your joypad's axes/buttons numeration, you may find useful a tool named "jstest".

[ConvertOutput]
This is not a ROS node, but is here because it is used for transforming the output of this program in a format that is readable for my other project "BearingSlam".
A transformation matrix is applied to the robot poses to take into account the relative position of the sensor with respect to the center of the robot (the recorded poses are referred to the robot, not to the camera).