# Untitled

# **ROS 2 Thesis Template (Python)**

This repository provides a minimal and modular template for thesis projects based on **ROS 2** and **Python**.

---

## **Project Structure**

The repository contains a **collection of independent ROS 2 packages**, without an internal workspace.

Example layout:

```
<package_name_1>/    # ROS 2 Python package
<package_name_2>/    # another ROS 2 Python package
...
docs/                # project documentation

```

Documentation includes:

- General project documentation
    
    *(overview, setup, architecture, usage)*
    
- Package-specific documentation
    
    located in:
    
    `docs/packages/<package_name>/`
    

---

## **Getting Started**

Because the packages are **not** placed inside a workspace, you must use or create an **external ROS 2 workspace** and import the packages manually.

Example using a personal workspace:

```bash
# Create a workspace (if it does not exist)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy or symlink the packages from this repository
# Example:
ln -s /path/to/repo/<package_name> .

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the environment
source install/setup.bash

```

---

## **Running Nodes and Launch Files**

Run a node:

```bash
ros2 run <package_name> <node_name>

```

Run a launch file:

```bash
ros2 launch <package_name> <launch_file>.py

```
