# rmf_core_tools

Attempt to add tools to make work and testing multi-robot systems easier with `rmf_core`

## Setup
```
sudo apt install python3-vcstool -y
cd $HOME
mkdir -p core_ws/src && cd core_ws/src
git clone git@github.com:cnboonhan94/rmf_core_tools.git
cp rmf_core_tools/deps.repos .
vcs import < deps.repos
cd ..
# Source your ROS2
colcon build
```
