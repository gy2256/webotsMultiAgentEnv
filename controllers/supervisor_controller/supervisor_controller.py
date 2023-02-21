"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor,Receiver,Emitter
import ast
# create the Robot instance.
supervisor_robot = Supervisor()


# get the time step of the current world.
timestep = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
agent0_state_receiver = supervisor_robot.getDevice("supervisor_receiver")
agent0_state_receiver.enable(timestep)
agent0_state_receiver.setChannel(0)


control_agent0_emitter = supervisor_robot.getDevice("control_agent0_emitter")
control_agent0_emitter.setChannel(0)

control_agent1_emitter = supervisor_robot.getDevice("control_agent1_emitter")
control_agent1_emitter.setChannel(1)

control_agent2_emitter = supervisor_robot.getDevice("control_agent2_emitter")
control_agent2_emitter.setChannel(2)

control_agent3_emitter = supervisor_robot.getDevice("control_agent3_emitter")
control_agent3_emitter.setChannel(3)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor_robot.step(timestep) != -1:

    if agent0_state_receiver.getQueueLength()>0:
        received_data = agent0_state_receiver.getString()
        agent0_state_receiver.nextPacket()
    
    # Agent 0 Control 
    agent0_cmd_vel = "[1.0, 0.0, 0]"
    control_agent0_emitter.send(agent0_cmd_vel)
    
    # Agent 1 Control
    agent1_cmd_vel = "[-1.0, 0.0, 0]"
    control_agent1_emitter.send(agent1_cmd_vel)
    
    # Agent 2 Control
    agent2_cmd_vel = "[0.0, 1.0, 0]"
    control_agent2_emitter.send(agent2_cmd_vel)
    
    # Agent 3 Control
    agent3_cmd_vel = "[0.0, -1.0, 0]"
    control_agent3_emitter.send(agent3_cmd_vel)
    
    pass

# Enter here exit cleanup code.
