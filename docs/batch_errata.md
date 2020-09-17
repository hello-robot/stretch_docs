![](images/hello_robot_large_rs.png)

# Batch Errata
The Stretch RE1 is manufactured fixed size batches. There may be minor changes, issues, and improvements between batches. These errata are captured here.

Each batch is named alphabetically after a musician. 'Guthrie' is the first publicly available version of Stretch.

## Guthrie

Released May 2020

Known issues

* The sound localization of Guthrie can be inconsistent due to the acoustic properties of the enclosure around the ReSpeaker microphone array. The voice recognition accuracy is not affected. Workaround: remove the head shell for experiments requiring accurate localization. Contact support@hello-robot.com for instructions.
* A version incompatibility between the ROS D435i packages and the Intel RealSense packages can cause Realsense Viewer to stop finding the camera on the bus. This happens after the first time the ROS package utilizes the camera. Prior to running ROS, the viewer will work as expected.
* The provided EasySMX XBox controller can be set into an incorrect interface mode (when holding down the illuminated center button for a few seconds.) The correct mode is with the upper half of ring LEDs illuminated top two LED quarter circle arcs).  To change it back to the correct mode, hold the center button down for 5s and release.  Repeat until top half of ring is illuminated.

Changes from prior version

* N/A
* 



------
.<div align="center"> All materials are Copyright 2020 by Hello Robot Inc. The Stretch RE1 robot has patents pending</div>
