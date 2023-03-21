# Import the needed libraries
import copy
import time
import socket
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

# The methodology is the same as the rectangular trajectory practice in the previous lab
positions = {}
rotations = {}
Targets = [[-1.93, -0.02], [-1.64, 0.45], [-0.52, 0.627], [-0.52, -0.8], [-1.56, -0.77], [-1.93, -0.02], [-1.93, -0.02]]


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


# Connect to the robot
IP_ADDRESS = "192.168.0.204"
# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')
# There are two things get done in this loop
# Find the first target and find the rest of the targets
if __name__ == "__main__":
    clientAddress = "192.168.0.27"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 204
    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    # In this loop we force the robot to find the first target point
    # The robot will sleep for a while when arrive at the first target point
    The_First_Target = [-1.93, -0.02]
    try:
        while is_running:
            if robot_id in positions:
                X_Position = positions[robot_id][0]
                Y_Position = positions[robot_id][1]
                Desired_Position_X = The_First_Target[0]
                Desired_Position_Y = The_First_Target[1]
                Difference_X = Desired_Position_X - X_Position
                Difference_Y = Desired_Position_Y - Y_Position
                Distance = math.sqrt(Difference_X ** 2 + Difference_Y ** 2)
                Rotation = rotations[robot_id]
                # Calculate the ideal rotation and convert it to degrees
                Prefer_Rotation = np.arctan(Difference_Y / Difference_X)
                Prefer_Rotation = np.rad2deg(Prefer_Rotation)
                if Difference_X > 0:
                    Rotation_Difference = Prefer_Rotation - Rotation
                else:
                    Rotation_Difference = Prefer_Rotation - Rotation + 180
                if Rotation_Difference > 180:
                    Rotation_Difference = Rotation_Difference - 360
                if Rotation_Difference < -180:
                    Rotation_Difference = Rotation_Difference + 360
                # Calculate the difference between the desired rotation and the current rotation
                Gain_v = 500
                v = 500 + Distance * Gain_v
                Gain_omega = 20
                omega = Rotation_Difference * Gain_omega
                # The function used to get linear and angular speed
                u = np.array([v - omega, v + omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                time.sleep(0.5)
                # Threshold used to judge if the robot arrived at desired point
                if Distance < 0.2:
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (0, 0, 0, 0)
                    s.send(command.encode('utf-8'))
                    time.sleep(2)
                    break
                time.sleep(0.5)
    # Keyboard Interrupt
    except KeyboardInterrupt:
        streaming_client.shutdown()
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()
    # In this loop the robot will track the rest of the targets.
    j = 1
    Start = copy.copy(Targets[j])
    try:
        while is_running:
            if robot_id in positions:
                X_Position = positions[robot_id][0]
                Y_Position = positions[robot_id][1]
                Desired_Position_X = Start[0]
                Desired_Position_Y = Start[1]
                Difference_X = Desired_Position_X - X_Position
                Difference_Y = Desired_Position_Y - Y_Position
                Distance = math.sqrt(Difference_X ** 2 + Difference_Y ** 2)
                Rotation = rotations[robot_id]
                Prefer_Rotation = np.arctan(Difference_Y / Difference_X)
                Prefer_Rotation = np.rad2deg(Prefer_Rotation)
                if Difference_X > 0:
                    Rotation_Difference = Prefer_Rotation - Rotation
                else:
                    Rotation_Difference = Prefer_Rotation - Rotation + 180
                if Rotation_Difference > 180:
                    Rotation_Difference = Rotation_Difference - 360
                if Rotation_Difference < -180:
                    Rotation_Difference = Rotation_Difference + 360
                if j == 4:
                    v = 500 + Distance * 1200
                    omega = Rotation_Difference * 120
                    u = np.array([v - omega, v + omega])
                    u[u > 5000] = 5000
                    u[u < -5000] = -5000
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (u[0], u[0], u[1], u[1])
                else:
                    v = 500 + Distance * 700
                    omega = Rotation_Difference * 40
                    u = np.array([v - omega, v + omega])
                    u[u > 2000] = 2000
                    u[u < -2000] = -2000
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))
                if Distance < 0.1:
                    j = j + 1
                    if j >= 7:
                        break
                    Start = copy.copy(Targets[j - 1])
                time.sleep(0.1)
    # Keyboard Interrupt
    except KeyboardInterrupt:
        streaming_client.shutdown()
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        s.shutdown(2)
        s.close()
# STOP all motors and shutdown the server
streaming_client.shutdown()
command = 'CMD_MOTOR#00#00#00#00\n'
s.send(command.encode('utf-8'))
s.shutdown(2)
s.close()
