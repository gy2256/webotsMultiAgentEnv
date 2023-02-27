# webotsMultiAgentEnv
This is an Webots environment with omnidirectional-robots for testing algorithms

## Software Requirement
Mac OS/Windows/Ubuntu
Webots 2023a (Download: https://cyberbotics.com/#download)

## Robot Design
There are in total of four omni-directional robots in the simulaiton world. Each robot runs its own controller (**controllers/drive_my_robot/drive_my_robot.py**, let's call this local controller) and it contains a GPS, transmitter and a receiver. The local controller takes in velocity commands through receiver (in *String* format) and send out its own position, velocity through transmitter. Note, the channel for receiver and transmitter are unqiue to each robot. The local controller also has a function that computes the inverse kinematics, such that the recevied velocity commands can be computed to generate speed for each wheel. This part of the code should not be changed. 

The supervisor controller (**controllers/supervisor_controller/supervisor_controller.py**) is in charged of perform centralized control among all robots. It listens to all robots' state information through receivers (with different channels) and brodcast velocity commands through transmitters (also with different channels). You can change agent_cmd_vel in **supervisor_controller.py** to see the affect. 

## How to run environment
1. Navigate to **worlds**
2. Double click **empty.wbt**
