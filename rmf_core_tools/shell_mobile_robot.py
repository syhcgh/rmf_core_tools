from __future__ import print_function
import yaml
import os
import rclpy
import numpy as np
import time
from threading import Thread

def normalize_angle(yaw):
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
        self.dt = 1.0 / float(self.details["config"]["update_rate"])

    def on(self):
        # "Turns on" the robot. The robot will start publishing fleet states, and will respond to Requests. 
        self.details["state"]["mode_on"] = True

    def off(self):
        # "Turns off" the robot. The robot will stop publishing fleet states, and will not respond to Requests
        self.details["state"]["mode_on"] = False

    def pause(self):
        # Robot does not move, although it is on. Simulates scenarios like temporary obstructions
        self.details["state"]["motor_on"] = True

    def resume(self):
        # Robot resumes movement, simulating the removal of temporary obstructions
        self.details["state"]["motor_on"] = False
     
    def rotate(self, yaw):
        # Spins robot to the specified angle
        thread = Thread(target = self._rotate_thread, args = (normalize_angle(yaw),))
        thread.start()

    def _rotate_thread(self, yaw):
            # Thread that spins robot asynchronously 
            yaw_now = self.details["state"]["yaw"]
            angular_vel = self.details["config"]["angular_velocity"]

            direction = 1
            if yaw - yaw_now < np.pi: # spin anticlockwise
                pass
            else:
                direction = -1  # spin clockwise, flip direction of spin

            time_required = (yaw - yaw_now) / angular_vel
            time_passed = 0

            self.node.get_logger().info("Rotating to yaw of " + str(yaw) + " from a yaw of " + str(yaw_now) + ".")
            time.sleep(1.5)

            while True:
                if not self.details["state"]["motor_on"]:
                    continue

                if time_passed < time_required:
                    time.sleep(self.dt)
                    yaw_now += angular_vel * self.dt * direction
                    yaw_now = normalize_angle(yaw_now)
                    self.details["state"]["yaw"] = yaw_now
                    time_passed += self.dt
                    self.get_state()
                else:
                    break

            self.details["state"]["yaw"] = float(yaw)

    def move(self, pos):
        thread = Thread(target = self._move_thread, args = (pos,))
        thread.start()

    def _move_thread(self, pos):
        # Thread that moves the robot synchronously
        pos_now = np.array([self.details["state"]["x"], self.details["state"]["y"]])
        linear_vel = self.details["config"]["linear_velocity"]

        x_direction = 1
        if pos[0] - pos_now[0] < 0:
            x_drection = -1
        y_direction = 1
        if pos[0] - pos_now[0] < 0:
            y_drection = -1

        time_required = np.linalg.norm(pos - pos_now) / linear_vel
        time_passed = 0
        x_delta = (pos[0] - pos_now[0]) / time_required
        y_delta = (pos[1] - pos_now[1]) / time_required

        self.node.get_logger().info("Moving from position of " + str(pos_now) + " from a position of  " + str(pos) + ".")
        time.sleep(1.5)

        while True:
            if not self.details["state"]["motor_on"]:
                continue

            if time_passed < time_required:
                time.sleep(self.dt)
                pos_now += np.array([x_delta, y_delta]) * self.dt
                self.details["state"]["x"] = pos_now[0]
                self.details["state"]["y"] = pos_now[1]
                time_passed += self.dt
                self.get_state()
            else:
                break
        self.details["state"]["x"] = float(pos[0])
        self.details["state"]["y"] = float(pos[1])

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
    robot.move(np.array([1,1]))

if __name__ == '__main__':
    main()
