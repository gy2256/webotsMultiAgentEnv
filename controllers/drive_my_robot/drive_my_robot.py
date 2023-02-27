"""drive_my_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS, Emitter, Receiver
from typing import List
import struct
import ast

def inverse_kinematics_velocity(vy:float,vx:float,omega:float)->List[float]:
    """
           [2] -------- [1]    +y (green)
              |       |      ^
              |       |      |  
              |       |      ---->  +x (red)
           [4] --------[3]
                  
    """
    w1,w2,w3,w4 = 0, 0, 0, 0
    l1, l2 = 0.56 , 0.25
    r = 0.15 #radius of wheel
     
    w1 = (1/r)*(vy-vx-omega*(l1+l2))
    w2 = (1/r)*(vy+vx+omega*(l1+l2))
    w3 = (1/r)*(vy+vx-omega*(l1+l2))
    w4 = (1/r)*(vy-vx+omega*(l1+l2))
     
    return [-w1,-w2,-w3,-w4]

if __name__ =="__main__":
    # create the Robot instance.
    robot = Robot()
    
    
    timestep = 32
    
    gps = robot.getDevice("gps")
    control_receiver = robot.getDevice("agent_receiver")
    state_emitter = robot.getDevice("agent_emitter")
    gps.enable(timestep)
    control_receiver.enable(timestep)
    #max_speed = 20
    motor1 = robot.getDevice("motor1")
    motor2 = robot.getDevice("motor2")
    motor3 = robot.getDevice("motor3")
    motor4 = robot.getDevice("motor4")
    
    motor1.setPosition(float("inf"))
    motor1.setVelocity(0.0)
    motor2.setPosition(float("inf"))
    motor2.setVelocity(0.0)
    motor3.setPosition(float("inf"))
    motor3.setVelocity(0.0)
    motor4.setPosition(float("inf"))
    motor4.setVelocity(0.0)
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        
        #vy, vx, omega = 0, 0, 0
        # Get GPS Vlaues
        
        
        if control_receiver.getQueueLength()>0:
            received_data_string = control_receiver.getString()
            cmd_vel = ast.literal_eval(received_data_string)
            velocities = inverse_kinematics_velocity(cmd_vel[1],cmd_vel[0],0.0)
            
            motor1.setVelocity(velocities[0])
            motor2.setVelocity(velocities[1])
            motor3.setVelocity(velocities[2])
            motor4.setVelocity(velocities[3])
            
            control_receiver.nextPacket()
        #velocities = inverse_kinematics_velocity(vy,vx,omega)
        
        gps_value = gps.getValues() #[x,y,z]
        gps_speed_vec = gps.getSpeedVector()  #[vx,vy,vz]
        state_vec = str(gps_value+gps_speed_vec)
        message = state_vec.encode("utf-8")
        state_emitter.send(message)
        
        
        
        pass
    
    # Enter here exit cleanup code.
    