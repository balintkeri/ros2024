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
| 3           | The Camera Controller shall detect cubes | 1 |
| 4           | The Camera Controller shall detech the colour of the given cube |  2  |
| 5           | The Camera Controller shall calculate the coordinates of cubes|   2    |
| 6           | The Camera Controller shall send the coordinate and the colour of the cubes to the Robot Arm Controller |      |
| 7           | The Robot Arm Controller shall sort the cubes by colour  |   2    |
| 8           | The Robot Arm Controller shall control the robot arm in the simulation  |   3    |
| 9           | The Robot Arm Controller shall move the cubes to the sorted position  |   2    |
