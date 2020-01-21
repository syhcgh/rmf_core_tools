import rclpy
from shell_mobile_robot import *
from rmf_fleet_msgs.msg import *
import numpy as np

# Commands in here are meant to be run manually line by line
# Use vim-slime or vscode "Send to terminal"
rclpy.init()
node = rclpy.create_node("location_msg_generator")
robot = ShellMobileRobot("magni")

robot.on()
robot.off()
robot.pause()
robot.resume()
robot.rotate(2.0)
robot.rotate(2.0, block=True)
robot.rotate(0.0)
robot.move(np.array([3.0, 4.0]))
robot.move(np.array([3.0, 4.0]), block=True)
robot.move(np.array([0.0, 0.0]))

# Create PathRequest
def generate_location_msg(x, y, yaw):
    msg = Location()
    msg.t = node.get_clock().now().to_msg() 
    msg.x = x
    msg.y = y
    msg.yaw = yaw
    msg.level_name = "demo-level"
    return msg

path_pub = node.create_publisher(PathRequest, "path_requests", 10)

msg = PathRequest()
msg.fleet_name = "magni"
msg.robot_name = "magni1"
msg.task_id = str(np.random.randint(0, 10000))
msg.path = [generate_location_msg(1.0, 1.0, np.pi / 2),
        generate_location_msg(2.0, 2.0, -np.pi / 2),
        generate_location_msg(0.0, 0.0, 0.0)]

robot.listen()
path_pub.publish(msg)
