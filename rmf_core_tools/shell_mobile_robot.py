from __future__ import print_function
import yaml
import os
import rclpy
import numpy as np
import time
from threading import Thread
from rmf_fleet_msgs.msg import *

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
        self.path = []

        # Initialize publishers and subscribers
        self.fleet_state_pub = self.node.create_publisher(FleetState, self.details["topics"]["fleet_state_topic"], 10)
        self.path_request_sub = self.node.create_subscription(PathRequest, self.details["topics"]["path_request_topic"], 
                self.path_request_callback, 10)
        self.destination_request_sub = self.node.create_subscription(DestinationRequest,
                self.details["topics"]["destination_request_topic"], 
                self.destination_request_callback, 10)
        self.mode_request_sub = self.node.create_subscription(ModeRequest,
                self.details["topics"]["mode_request_topic"], 
                self.mode_request_callback, 10)

    def on(self):
        # "Turns on" the robot. The robot will start publishing fleet states, and will respond to Requests. 
        self.node.get_logger().info("Turning on robot. Robot will start publishing fleet states.")
        self.details["state"]["mode_on"] = True

    def off(self):
        # "Turns off" the robot. The robot will stop publishing fleet states, and will not respond to Requests
        self.node.get_logger().info("Turning off robot. Robot will stop publishing fleet states.")
        self.details["state"]["mode_on"] = False

    def pause(self):
        # Robot does not move, although it is on. Simulates scenarios like temporary obstructions
        self.node.get_logger().info("Robot paused. Robot will not move until resume is called.")
        self.details["state"]["motor_on"] = False

    def resume(self):
        # Robot resumes movement, simulating the removal of temporary obstructions
        self.node.get_logger().info("Robot resumed. Robot will start moving again.")
        self.details["state"]["motor_on"] = True
     
    def rotate(self, yaw):
        # Spins robot to the specified angle
        thread = Thread(target = self._rotate_thread, args = (normalize_angle(yaw),))
        thread.start()

    def _rotate_thread(self, yaw):
            # Thread that spins robot asynchronously 
            yaw_now = self.details["state"]["yaw"]
            angular_vel = self.details["config"]["angular_velocity"]

            direction = 1
            if yaw < yaw_now:
                direction *= -1

            if yaw - yaw_now < np.pi:
                pass
            else:
                direction *= -1 

            time_required = np.abs((yaw - yaw_now) / angular_vel)
            time_passed = 0

            self.node.get_logger().info("Rotating to yaw of " + str(yaw) + " from a yaw of " + str(yaw_now) + ".")
            self.node.get_logger().info("Time Required: " + str(time_required))
            time.sleep(1.5)

            while True:
                if not self.details["state"]["motor_on"]:
                    self.node.get_logger().info("Robot is currently paused, waiting to resume...")
                    time.sleep(3)
                    continue

                if time_passed < time_required:
                    time.sleep(self.dt)
                    yaw_now += angular_vel * self.dt * direction
                    yaw_now = normalize_angle(yaw_now)
                    self.details["state"]["yaw"] = yaw_now
                    time_passed += self.dt
                    self.publish_fleet_state()
                else:
                    break

            self.details["state"]["yaw"] = float(yaw)
            self.node.get_logger().info("Rotation complete.")

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

        time_required = np.abs(np.linalg.norm(pos - pos_now) / linear_vel)
        time_passed = 0
        x_delta = (pos[0] - pos_now[0]) / time_required
        y_delta = (pos[1] - pos_now[1]) / time_required

        self.node.get_logger().info("Moving from position of " + str(pos_now) + " from a position of  " + str(pos) + ".")
        self.node.get_logger().info("Time Required: " + str(time_required))
        time.sleep(1.5)

        while True:
            if not self.details["state"]["motor_on"]:
                self.node.get_logger().info("Robot is currently paused, waiting to resume...")
                time.sleep(3)
                continue

            if time_passed < time_required:
                time.sleep(self.dt)
                pos_now += np.array([x_delta, y_delta]) * self.dt
                self.details["state"]["x"] = pos_now[0]
                self.details["state"]["y"] = pos_now[1]
                time_passed += self.dt
                self.publish_fleet_state()
            else:
                break
        self.details["state"]["x"] = float(pos[0])
        self.details["state"]["y"] = float(pos[1])
        self.node.get_logger().info("Movement complete.")

    def get_state(self):
        # Get the current status of the robot in a human-readable printout
        name = self.details["config"]["name"]
        x = self.details["state"]["x"]
        y = self.details["state"]["y"]
        yaw = self.details["state"]["yaw"]
        mode = self.details["state"]["mode_on"]
        motor = self.details["state"]["motor_on"]
        self.node.get_logger().info("\n" + name + " state:" + "\n" + 
                "X: " + str(x) + "\n" +
                "Y: " + str(y) + "\n" + 
                "Yaw: " + str(yaw) + "\n" + 
                "Mode On: " + str(mode) + "\n" + 
                "Motor On: " + str(motor))

    def publish_fleet_state(self):
        # Publish a ROS2 of the current robot fleet state
        if not self.details["state"]["mode_on"]:
            self.node.get_logger().error("Robot is currently switched off!")
            return

        robot_mode_msg = RobotMode()
        if self.details["state"]["task_id"] == "":
            robot_mode_msg.mode = 0 # IDLE
        else:
            if not self.details["motor_on"]:
                robot_mode_msg.mode = 3 # PAUSED
            else:
                robot_mode_msg.mode = 2 # MOVING

        location_msg = Location()
        location_msg.t = self.node.get_clock().now().to_msg()
        location_msg.x = self.details["state"]["x"]
        location_msg.y = self.details["state"]["y"]
        location_msg.yaw = self.details["state"]["yaw"]
        location_msg.level_name = self.details["state"]["level_name"]

        robot_state_msg = RobotState()
        robot_state_msg.name = self.details["config"]["name"]
        robot_state_msg.model = self.details["config"]["fleet_name"]
        robot_state_msg.task_id = self.details["state"]["task_id"]
        robot_state_msg.mode = robot_mode_msg
        robot_state_msg.battery_percent = self.details["state"]["battery_percent"]
        robot_state_msg.location = location_msg
        robot_state_msg.path = self.path

        fleet_state_msg = FleetState()
        fleet_state_msg.name = self.details["config"]["fleet_name"]
        fleet_state_msg.robots = [robot_state_msg]
        self.fleet_state_pub.publish(fleet_state_msg)

    def destination_request_callback(self, msg):
        # Handles the message received on from the fleet adapter, Depending on the type of control.
        raise NotImplementedError

    def path_request_callback(self, msg):
        # Handles the message received on from the fleet adapter, Depending on the type of control.
        raise NotImplementedError

    def mode_request_callback(self, msg):
        # Handles the message received on from the fleet adapter, Depending on the type of control.
        raise NotImplementedError

def main(args=None):
    rclpy.init(args=args)
    robot_name = "magni" # TODO: Replace with parameter from ROS2
    robot = ShellMobileRobot(robot_name)
    robot.rotate(np.pi-0.01)
    time.sleep(3)
    robot.rotate(0.0)

if __name__ == '__main__':
    main()
