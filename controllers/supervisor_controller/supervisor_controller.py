"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import ast
import numpy as np

from controller import Supervisor,Receiver,Emitter
from MPC_controller import MPC_controller

class Agent:
    def __init__(self, pos, goal=None):
        self.x = pos[0]
        self.y = pos[1]
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.goal = goal

    def update_state(self, current_state):
        self.x = current_state[0]
        self.y = current_state[1]
        self.vx = current_state[3]
        self.vy = current_state[4]

    def return_pos(self):
        return [self.x, self.y]

    def return_vel(self):
        return [self.vx, self.vy]

    def return_state(self):
        return [self.x, self.vx, self.y, self.vy]

    def at_goal(self):
        if np.linalg.norm(np.array([self.x, self.y]) - np.array(self.goal)) <= 0.25:
            return True
        else:
            return False

# Define Mission parameters
agent_init_pos_list = [[1.0,1.0,0.0],[7.0, 1.0, 0.0], [5.5, 5.5, 0.0], [0.8, 5.5, 0.0]]
agent_goal_pos_list = [[7.0, 1.0, 0.0], [1.0, 1.0, 0.0], [0.8, 5.5, 0.0], [5.5, 5.5, 0.0]]

# MPC controller parameters
Q = np.diag([1.0, 0.15, 1.0, 0.15])
R = np.diag([0.01, 0.01])
MPC_horizon = 8

# create the Robot instance.
supervisor_robot = Supervisor()

# get the time step of the current world.
timestep = 32
DT = timestep * 0.001 # This value should be multiple of the basictimestep in Webots

# Setup sensors for receving control commands and brodcasting state information
agent0_state_receiver = supervisor_robot.getDevice("agent0_state_receiver")
agent0_state_receiver.enable(timestep)
agent0_state_receiver.setChannel(0)

agent1_state_receiver = supervisor_robot.getDevice("agent1_state_receiver")
agent1_state_receiver.enable(timestep)
agent1_state_receiver.setChannel(1)

agent2_state_receiver = supervisor_robot.getDevice("agent2_state_receiver")
agent2_state_receiver.enable(timestep)
agent2_state_receiver.setChannel(2)

agent3_state_receiver = supervisor_robot.getDevice("agent3_state_receiver")
agent3_state_receiver.enable(timestep)
agent3_state_receiver.setChannel(3)

agent_receiver_list = [agent0_state_receiver, agent1_state_receiver, agent2_state_receiver, agent3_state_receiver]


control_agent0_emitter = supervisor_robot.getDevice("control_agent0_emitter")
control_agent0_emitter.setChannel(0)

control_agent1_emitter = supervisor_robot.getDevice("control_agent1_emitter")
control_agent1_emitter.setChannel(1)

control_agent2_emitter = supervisor_robot.getDevice("control_agent2_emitter")
control_agent2_emitter.setChannel(2)

control_agent3_emitter = supervisor_robot.getDevice("control_agent3_emitter")
control_agent3_emitter.setChannel(3)

agent_emitter_list = [control_agent0_emitter, control_agent1_emitter, control_agent2_emitter, control_agent3_emitter]

# Place agents at initial positions
for idx, agent_init_pos in enumerate(agent_init_pos_list):
    agent_translation_field = supervisor_robot.getFromDef('agent_'+str(idx)).getField('translation')
    agent_translation_field.setSFVec3f(agent_init_pos)

# Initiate agent objects
agent_0 = Agent(agent_init_pos_list[0])
agent_1 = Agent(agent_init_pos_list[1])
agent_2 = Agent(agent_init_pos_list[2])
agent_3 = Agent(agent_init_pos_list[3])
agent_list = [agent_0, agent_1, agent_2, agent_3]

# MPC initialization
agent_MPC_list = []
agent_x_ref_list = []

for i in range(len(agent_list)):
    '''
    
    agent_x_ref_list.append(np.array([pos_waypoint_list[i][:,0], 
                                      vel_waypoint_list[i][:,0],
                                      pos_waypoint_list[i][:,1],
                                      vel_waypoint_list[i][:,1]]))
    '''
    agent_MPC_list.append(MPC_controller(
        MPC_horizon=MPC_horizon,
        dt=DT,  # MPC dt needs to match simulation dt
        state_weight=Q,  # Q matrix
        control_weight=R,  # R matrix
        # Initialize initial state with 0 velocities in x, y direction
        x_init=agent_list[i].return_state()))

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor_robot.step(timestep) != -1:

    if agent0_state_receiver.getQueueLength()>0:
        #Receive state information from the environment
        for agent, agent_receiver in zip(agent_list, agent_receiver_list):
            agent.update_state(ast.literal_eval(agent_receiver.getString()))
            agent_receiver.nextPacket()
    

    # Agent 0 Control 
    agent0_cmd_vel = "[1.0, 0.0, 0]"
    control_agent0_emitter.send(agent0_cmd_vel)
    
    # Agent 1 Control
    agent1_cmd_vel = "[1.0, 0.0, 0]"
    control_agent1_emitter.send(agent1_cmd_vel)
    
    # Agent 2 Control
    agent2_cmd_vel = "[0.0, 1.0, 0]"
    control_agent2_emitter.send(agent2_cmd_vel)
    
    # Agent 3 Control
    agent3_cmd_vel = "[0.0, -1.0, 0]"
    control_agent3_emitter.send(agent3_cmd_vel)
    
    pass

# Enter here exit cleanup code.
