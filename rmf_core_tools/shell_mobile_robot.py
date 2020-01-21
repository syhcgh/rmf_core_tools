from __future__ import print_function
import yaml
import os
import rclpy
import numpy as np
import time
from threading import Thread

def normalize_angle(self, yaw):
    angle = yaw % (2 * np.pi)
    angle = ( angle + 2 * np.pi ) % (2 * np.pi)
    if (angle > np.pi):
        angle -= 2 * np.pi
    return angle

class ShellMobileRobot:
    def __init__(self, robot_name):
        self.node = rclpy.create_node("shell_" + robot_name)
        dir_path = os.path.dirname(os.path.realpath(__file__))
        yaml_path = dir_path + '/../robots/' + robot_name + '.yaml' 

        self.node.get_logger().info("Loading yaml from " + yaml_path + ".")

        with open(yaml_path) as f:
            try:
                self.details = yaml.load(f, Loader=yaml.FullLoader)
            except Exception as e:
                self.node.get_logger().error(e)
                rclpy.shutdown()
    
        # TODO: Write module to verify the contents of yaml

    def on(self):
        # "Turns on" the robot motors. The robot will start publishing fleet states, and will respond to Requests. 
        self.details["state"]["mode_on"] = True

    def off(self):
        # "Turns off" the robot motors. The robot will stop publishing fleet states, and will not respond to Requests
        self.details["state"]["mode_on"] = False

    def pause(self):
        # Robot does not move, although it is on. Simulates scenarios like temporary obstructions
        raise NotImplementedError

    def resume(self):
        # Robot resumes movement, simulating the removal of temporary obstructions
        raise NotImplementedError
     
    def rotate(self, yaw):
        # Spins robot to the specified angle
        yaw_now = self.details["state"]["yaw"]
        hz = self.details["config"]["update_rate"]
        thread = Thread(target = self._rotate_thread, args = (yaw, hz))
        thread.start()

def _rotate_thread(self, yaw, hz):
        # Thread that spins robot asynchronously 

    def move(self, pos):
        # Translates the robot to the specified position
        # x_now = self.details["state"]["x"]
        # y_now = self.details["state"]["y"]
        # pos_now = np.array([x_now, y_now])
        # d_vec = pos - pos_now
        raise NotImplementedError

    
    def get_state(self):
        # Get the current status of the robot in a human-readable printout
        fleet_name = self.details["config"]["fleet_name"]
        x = self.details["state"]["x"]
        y = self.details["state"]["y"]
        yaw = self.details["state"]["yaw"]
        mode = self.details["state"]["mode_on"]
        motor = self.details["state"]["motor_on"]
        self.node.get_logger().info("\n" + fleet_name + " state:" + "\n" + 
                "X: " + str(x) + "\n" +
                "Y: " + str(y) + "\n" + 
                "Yaw: " + str(yaw) + "\n" + 
                "Mode On: " + str(mode) + "\n" + 
                "Motor On: " + str(motor))

    def publish_fleet_state(self):
        # Publish a ROS2 of the current robot fleet state
        raise NotImplementedError

    def handle_request_callback(self, msg):
        # Handles the message received on from the fleet adapter, Depending on the type of control.
        raise NotImplementedError

def main(args=None):
    rclpy.init(args=args)
    robot_name = "magni" # TODO: Replace with parameter from ROS2
    robot = ShellMobileRobot(robot_name)
    # robot.rotate(2)

if __name__ == '__main__':
    main()
