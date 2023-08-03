# webotsMultiAgentEnv
This is a Webots environment with omnidirectional-robots for testing algorithms.

## Software Requirement
Mac OS/Windows/Ubuntu
Webots 2023a (Download: https://cyberbotics.com/#download)

## Robot Design
There are in total of four omnidirectional robots in the simulation world. Each robot runs its own controller (**controllers/drive_my_robot/drive_my_robot.py**, let's call this a local controller), and it contains a GPS, transmitter, and receiver. The local controller takes in velocity commands through the receiver (in *String* format) and sends out its own position, velocity through the transmitter. Note the channel for the receiver and transmitter are unique to each robot. The local controller also has a function that computes the inverse kinematics, such that the received velocity commands can be computed to generate speed for each wheel. This part of the code should not be changed. 

The supervisor controller (**controllers/supervisor_controller/supervisor_controller.py**) is in charge of performing centralized control among all robots. It listens to all robots' state information through receivers (with different channels) and broadcasts velocity commands through transmitters (also with different channels). You can change agent_cmd_vel in **supervisor_controller.py** to control the robots. 

## How to run environment
1. Navigate to **worlds**
2. Double click **empty.wbt**
