# EC463/464 - Team 2
## Autonomous Legged Robot
This repository is for the senior design class team 2. PuppyPi, our autonomous-legged robot is an AI guide dog, providing visually-impaired users with timely information about obstacle positions for safer navigation. The robot's first function is to reach a pre-determined destination using an audio-based user interface. It will have the ability to understand the building's map based on lines on the floor and gather information about the building by scanning QR codes placed on the floor. Decoding each QR code provides information about the location, helping the robot dog decide where to go and guiding the user to the intended destination.

#

**Documentation**

* [Hardware documentation](https://github.com/PicassoEEA/legged_robot/tree/main/Hardware_Info)
 This hardware documentation will give a brief introduction of all hardware we have used in this project including Oak-d camera, Lidar, RGB camera, embedded battery, raspberry pi and so on. All dimensions like the angle and fps of camera, weight of Oak-d, scope of lidar should be found in this documentation. 

* [Sofware documentation](https://github.com/PicassoEEA/legged_robot/tree/main/Software_Info)
 This software documentation mainly describe the connection between raspberry pi and another control end(PC or app control). The tutorial of using VNC viewer and hotspot wifi to make that connection should be found here.
* [Hand Gesture Action](https://github.com/PicassoEEA/legged_robot/tree/main/Hand_Gesture_Action)
 This is an extra function run on Oak-d camera, which allows users to display certain kinds of gestures and make the robot perform specific movement. The Oak-d camera has to be connected to the raspberry pi of puppypi.
* [463 Files](https://github.com/PicassoEEA/legged_robot/tree/main/463Files)
  This is a folder of edited code which may help to understand different modules. Most edited scripts are stored under 463Files/src/test_files/scripts. AllInOne.py is the final integrated code includes the priority of different modules and runs all the functions in this single python script. apriltag_detect_demo.py is the python file for QR code scanning. lidar_edited.py is the file which includes both obstacle detection by lidar and obstacle avoidance algorithm using puppypi movement control. This direfctory is the first one that a new user wants to look at, and we recommend to try run AllInOne.py.
* [AllInOne.py](https://github.com/PicassoEEA/legged_robot/blob/main/AllInOne.py)
  This python script is the latest version of our robot dog, which will first scann a QR code for destination, then follow the line on the floor to the destination. When obstacle appears on the line, lidar will detect that and switch to the lidar module to do obstacle avoidance; after avoiding the obstacle, the robot dog will turn to see the line and continue to follow that line. If a "Y" intersection is appearing, the robot dog will look for a another QR code which tells it to turn left or right, and then follow the line as usual. Eventually when it reaches the destination, the final QR code will be scanned and tell the visually blined people that the destination is arrived.
#

**Demo Video**

Here are some demo videos of the performance of the robot dog(https://drive.google.com/drive/folders/1-3u4MFsINuggVaRiVoGgDMhkyEvuZgEe?usp=sharing)

#
**Reports**

In this directory, all of our progress reports and testing plans are uploaded. It should be useful for new users to understand what we are doing and how to start this project. 

#

**Attempts to start this project**

1.
2.

#

## Team member
Bowen Ma, Shun Zhang, Xiteng Yao, Yichen Wang, Yihe Bi
