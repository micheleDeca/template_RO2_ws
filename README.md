# ROS 2 Thesis Template (Python)

This repository provides a minimal template for thesis projects based on ROS 2 and Python.

## Structure

- `ros2_ws/` – ROS 2 workspace
  - `src/hello_world/` – example Python package with a simple node `hello_node`
- `docs/` – documentation
  - global project docs (overview, setup, architecture, usage)
  - per-package docs under `docs/packages/<package_name>/`

## Getting Started

```bash
cd ros2_ws
colcon build
source install/setup.bash

ros2 run hello_world hello_node
# or
ros2 launch hello_world hello_launch.py

