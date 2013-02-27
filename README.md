A set of ROS nodes to gather landmarks bearings from an omnicam while moving a robot.

[BearingCollector]
The actual gathering node.
Reads images from the /cam_stream topic and robotposes from the /pose topic.
When the robot has moved (or rotated) enough, the gathering action is triggered.
Gathering consists in:
	  look for landmarks in the image (colored blobs)
	  detect landmarks centroids
	  store the angles that those blobs form with respect to the center of the image (that's because we assume we are using an omnicam)

[CameraStream]
Captures images from a camera and publishes them to the /cam_stream topic

[ImageReceiver]
Reads images from the /cam_stream topic and displays them on the screen

[Joy2cmd_vel]
User interface that allows to drive the robot by the use of a joypad. This publishes on the /cmd_vel topic.
Its peculiarity is that it can be customized (I.E. select which axes/button does what) by editing a simple text file.
This is very nice feature if your joypad has a broken lever :D
If you want to get infos about your joypad's axes/buttons numeration, you may find useful a tool named "jstest".