# Hardware Infomation

The robot we develope upon is called "Puppy Pi", developed by [Hiwonder](https://www.hiwonder.com/) and the central processor of it is a Raspberry Pi 4B (4GB). We also utilize some additional sensors like TOF Lidar and OAK-D Camera. This Readme will show how to utilize them and their spectations. 

# 

**Puppy Pi**

The robot we use has eight motors on it as the movenment system, with four motor on each. It use the ROS system to operate and support Python for programming. It uses a 7.4V 2200mAh Lipo battery and since it can only last for about 25 minuetes, we add two additional ones at the bottom of it to make it last over an hour.

<img src="https://github.com/PicassoEEA/legged_robot/blob/main/Hardware_Info/Puppy-Pi.png" width=50% height=50%>

Specs: 

<img src="https://github.com/PicassoEEA/legged_robot/blob/main/Hardware_Info/Puppy-Pi_spec.png" width=70% height=50%>

#

**Raspeberry Pi 4B**

The robot uses raspberry pi 4B (4GB) as its contral processor. We have two USB port for the connection of outer sensors like OAK-D camera. The Pi itself also have pins for connections of smaller sensors like the TOF lidar we utilize. This Pi also has ROS system pre-installed with it. 

<img src="https://github.com/PicassoEEA/legged_robot/blob/main/Hardware_Info/Pi-4B.png" width=50% height=50%>

Specs:

<img src="https://github.com/PicassoEEA/legged_robot/blob/main/Hardware_Info/Pi_spec.png" width=70% height=50%>

#

**TOF Lidar**

A Tof (Time of Flight) Lidar refers to using the speed of light (or even sound) to determine distance. It measures the time it takes for light (or sound) to leave the device, bounce off an object or plane, and return to the device, all divided by two reveals the distance from the device to the object or plane. We use this TOF in our project for detecting the obstacles in front of the dog to determine when and how to avoid it. 

<img src="https://github.com/PicassoEEA/legged_robot/blob/main/Hardware_Info/TOF.png" width=50% height=50%>

Specs:

<img src="https://github.com/PicassoEEA/legged_robot/blob/main/Hardware_Info/TOF_spec.png" width=70% height=50%>

#

**OAK-D**

OAK-D is the ultimate camera for robotic vision developed by [Luxonis](https://www.luxonis.com). It perceives the world like a human by combining stereo depth camera and high-resolution color camera with an on-device Neural Network inferencing and Computer Vision capabilities. It uses USB-C for both power and USB3 connectivity. 

<img src="https://github.com/PicassoEEA/legged_robot/blob/main/Hardware_Info/OAK-D.png" width=50% height=50%>

Specs:

<img src="https://github.com/PicassoEEA/legged_robot/blob/main/Hardware_Info/OAK-D_spec.png" width=70% height=50%>


