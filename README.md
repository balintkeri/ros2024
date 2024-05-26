# Pick and place robot arm

A repository for the Pick and Place robot arm project.

##  Administrative informations

Semester: 2023/24/2\
Subject: Robotic Systems Laboratory \
Subject code: BMEGEMINMRL\
Project: Pick and place robot arm\
Members: 
- Viktor Csáji
- Bálint Kéri

## Requirements

We wrote some basic system requirement for our project

| Requirement ID          | Description |
| ----------- | ----------- |
| 1           | The System shall be able to see cubes |
| 2           | The System shall be able to sort the cubes by color        |
| 3           | The System shall be able to pick up cubes        |


To cover this requirements, we designed the system like on the image below, and we wrote some requiremnt for our implementation

![kép](https://github.com/balintkeri/ros2024/assets/52506432/3d7f6460-bd0f-40b3-bc73-f12e7ce1f0d5)


| Requirement ID          | Description | Which System Requirement will this cover ? |
| ----------- | ----------- | -------------|
| 4          | The Camera Controller shall detect cubes | 1 |
| 5           | The Camera Controller shall detech the colour of the given cube |  2  |
| 6           | The Camera Controller shall calculate the coordinates of cubes|   2    |
| 7           | The Camera Controller shall send the coordinate and the colour of the cubes to the Robot Arm Controller |      |
| 12           | The Camera Controller shall receive the camera's signal |      |
| 8           | The Robot Arm Controller shall caculate the position of the unmoved cubes  |   3    |
| 9           | The Robot Arm Controller shall caculate the position of the sorted cubes  |   2    |
| 10           | The Robot Arm Controller shall control the robot arm in the simulation  |   3    |
| 11           | The Robot Arm Controller shall calculate the track of the cubes  |   2    |

## Video

We made a video about the operation of the robot arm.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/l6i5513F12M/0.jpg)](https://www.youtube.com/watch?v=l6i5513F12M)

## The Camera

We used a basic 320x240 pixel camera mounted over the the robot arm and it's work region. We painted the table black to have a better image quality, handle light reflection. To simulate this, we added a ROS plug in into the Gazeboo simulation.

## The Camera Controller 

The Camera Controller is implemented in the image_processer.py. The most used python library in this modul is the OpenCV (cv2). The Camera Controller subscribes to the simulated Camera. If we get a trigger, then we take a picture from the raw camera signal. First, we mask out the robot arm, the enviroment of the table and the robot arm's built in camera.

![image](https://github.com/balintkeri/ros2024/assets/52506432/77b65317-4dd6-4fe1-ad51-015d7f518845)

After this we applyed a threshold for the RGB colors to get the objects to move as BLOBs. To get the coordinate of the object, i used to OpenCV's contour finder algorith. The centre of the blob is the mean value of the contour's points.

![image](https://github.com/balintkeri/ros2024/assets/52506432/30fdae48-0f10-4132-963c-f7c2fa28dd24)

The camera controller transforms the centre point of the BLOBS (in pixels) to coordinates (in the Gazeboo world) with a linear interpolation. The parameters is determined by empiric methods. The camera controllers sends those coordinates to the robot arm controller
