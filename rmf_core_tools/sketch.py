import rclpy
from shell_mobile_robot import *
import numpy as np

# Commands in here are meant to be run manually line by line
# Use vim-slime or vscode "Send to terminal"
rclpy.init()
robot = ShellMobileRobot("magni")
robot.on()
robot.off()
robot.pause()
robot.resume()
robot.rotate(2.0)
robot.rotate(0.0)
robot.move(np.array([3.0, 4.0]))
robot.move(np.array([0.0, 0.0]))
