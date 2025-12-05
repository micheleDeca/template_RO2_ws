ROS 2 Thesis Template (Python)

This repository provides a minimal template for thesis projects based on ROS 2 and Python.

Structure

Collection of ROS 2 packages (each one independent, without a dedicated workspace)

For example:

<package_name_1>/   – ROS 2 Python package
<package_name_2>/   – another ROS 2 Python package
…

docs/               – project documentation


general documentation (overview, setup, architecture, usage)

package-specific documentation in docs/packages/<package_name>/

Getting Started

Since the packages are no longer inside a workspace, you need to create or use an external workspace to import them, or manually add them to your development environment.
Typical example using a personal workspace:

# create a workspace (if it does not exist)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# copy or symlink the packages from the template
# e.g.: ln -s /path/to/repo/<package_name> .

cd ~/ros2_ws
colcon build
source install/setup.bash

Running a node or a launch file
ros2 run <package_name> <node_name>
# or
ros2 launch <package_name> <launch_file>.py
