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
agent0_waypoints = np.array([[-10.16,10.00],[-10.16, 5.0], [-10.16, 0.0], [-5.0, 5.0]]) #[[x1,y1],[x2,y2],...]
agent1_waypoints = np.array([[-5.5, 10.15],[-11.92, 10.15], [-11.92, 3.63], [-15.26, 7.92]])
agent2_waypoints = np.array([[-0.65, 10.1],[-0.65, 3.21], [3.58, -2.58], [-2.43, -8.48]])
agent3_waypoints = np.array([[4.5, 10.06],[8.72, 10.06], [3.03, 4.31], [10.74, -3.65]])
agent_waypoint_list = [agent0_waypoints, agent1_waypoints, agent2_waypoints, agent3_waypoints]
agent_init_pos_list = [agent0_waypoints[0].tolist(),agent1_waypoints[0].tolist(),agent2_waypoints[0].tolist(),agent3_waypoints[0].tolist()]

for i in range(len(agent_init_pos_list)):
    agent_init_pos_list[i].append(0.0) # Add z coordinate to the initial position

target_speed = 2.5 # m/s

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
Agent0_target_idx, Agent1_target_idx, Agent2_target_idx, Agent3_target_idx = 0, 0, 0, 0
agent_target_idx_list = [Agent0_target_idx, Agent1_target_idx, Agent2_target_idx, Agent3_target_idx]
agent_x_local_ref_list = [None, None, None, None]
agent_command_list = [None, None, None, None]

for i in range(len(agent_list)):
    agent_MPC_list.append(MPC_controller(
        MPC_horizon=MPC_horizon,
        dt=DT,  # MPC dt needs to match simulation dt
        state_weight=Q,  # Q matrix
        control_weight=R,  # R matrix
        # Initialize initial state with 0 velocities in x, y direction
        x_init=agent_list[i].return_state()))
    
    agent_x_ref_list.append(agent_MPC_list[i].waypoints_to_x_ref(agent_waypoint_list[i], interpolated_dist=0.25,
                                                                 target_speed=target_speed, interpolation_type="linear"))



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor_robot.step(timestep) != -1:

    if agent0_state_receiver.getQueueLength()>0:
        #Receive state information from the environment
        for agent, agent_receiver in zip(agent_list, agent_receiver_list):
            agent.update_state(ast.literal_eval(agent_receiver.getString()))
            agent_receiver.nextPacket()
    
    agent_current_state_list = [agent.return_state() for agent in agent_list]

    for i in range(len(agent_list)):
        agent_x_local_ref_list[i], agent_target_idx_list[i] = agent_MPC_list[i].calculate_local_reference(
            agent_x_ref_list[i], agent_list[i].return_state(), agent_target_idx_list[i]
        )
        _, _, _, _, u1_traj, u2_traj = agent_MPC_list[i].mpc_control(
            agent_x_local_ref_list[i], agent_list[i].return_state()
        )

        agent_command_list[i] = "[{}, {}, 0]".format(
            agent_current_state_list[i][1]+u1_traj[0] * DT,
            agent_current_state_list[i][3]+u2_traj[0] * DT,
            0,
        )
    
    for emitter, agent_command in zip(agent_emitter_list, agent_command_list):
        emitter.send(agent_command)
    
    pass

