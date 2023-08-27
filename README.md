# webotsMultiAgentEnv
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.8287825.svg)](https://doi.org/10.5281/zenodo.8287825)

Please consider citing this work if you find it helpful. All support is appreciated.
```
@software{Yang_Webots_Multi-agent_Environment_2023,
author = {Yang, Guang},
doi = {10.5281/zenodo.8287825},
month = aug,
title = {{Webots Multi-agent Environment}},
url = {https://github.com/gy2256/webotsMultiAgentEnv},
version = {0.0.1},
year = {2023}
}
```

This is a Webots environment with omnidirectional robots for testing algorithms. The environment contains four robots that each can take into a list of waypoints in x and y coordinates. The program then takes into the desired speed (`target_speed`), calculates the corresponding velocity profile, and generates a state reference for each MPC controller to track. The MPC implementation is in **controllers/supervisor_controller/MPC_controller.py**. 

## Dependencies (tested)
Webots 2023a (Download: https://cyberbotics.com/#download)
```
python: "3.10.12"
cvxpy: "1.3.2"
numpy: "1.24.4"
```

## Robot Design
There are in total of four omnidirectional robots in the simulation world. Each robot runs its own controller (**controllers/drive_my_robot/drive_my_robot.py**, let's call this a local controller), and it contains a GPS, transmitter, and receiver. The local controller takes in velocity commands through the receiver (in *String* format) and sends out its own position, velocity through the transmitter. Note the channel for the receiver and transmitter are unique to each robot. The local controller also has a function that computes the inverse kinematics, such that the received velocity commands can be computed to generate speed for each wheel. This part of the code should not be changed. 

The supervisor controller (**controllers/supervisor_controller/supervisor_controller.py**) is in charge of performing centralized control among all robots. It listens to all robots' state information through receivers (with different channels) and broadcasts velocity commands through transmitters (also with different channels). You can change agent_cmd_vel in **supervisor_controller.py** to control the robots. 

## How to run environment
1. Navigate to **worlds**
2. Double click **empty.wbt**
3. Press play button in Webots
