
# ROS 2 Thesis Template (Python)

Template for Bachelor's and Master's thesis projects based on ROS 2 and Python

---

## Overview

This template provides a modular and well-organized structure for developing thesis projects using ROS 2 and Python.

It is designed to help students produce high-quality code that is testable, maintainable, and well documented.

The repository contains a collection of independent ROS 2 packages, not included inside an internal workspace.

This architecture allows students to create their own external ROS 2 workspace and integrate the thesis packages into it, ensuring a clear separation between:

- the student's personal workspace
- the project’s official codebase

---

## Repository Structure

```
.
├── README.md                           # This file
├── .gitignore                          # Git ignore rules
├── hello_world/                        # Example package
│   ├── hello_world/                    # Python module
│   │   ├── __init__.py
│   │   └── hello_node.py               # Node implementation
│   ├── launch/
│   │   └── hello_world.launch.py       # Launch file
│   ├── test/                           # Automated tests
│   │   ├── test_copyright.py
│   │   ├── test_flake8.py
│   │   └── test_pep257.py
│   ├── resource/
│   │   └── hello_world                 # Entry point
│   ├── package.xml                     # ROS 2 metadata
│   ├── setup.py                        # Python package configuration
│   └── setup.cfg                       # Additional configuration
└── docs/                               # Project documentation
    ├── README.md                       # General thesis overview (optional)
    ├── ARCHITECTURE.md                 # Architecture description (optional)
    ├── SETUP.md                        # Setup instructions
    └── packages/
        └── hello_world/                # Package-specific documentation
            ├── design.md               # Design notes
            └── api.md                  # API documentation

```

---

## Technical Requirements

Before using this template, ensure you have:

- **Ubuntu 22.04** or a compatible version
- **ROS 2 Humble** or newer
- **Python 3.10** or newer
- **colcon** (ROS 2 build tool)
- **Git**

If you are working inside a Docker environment provided by the technical supervisors, these requirements are already installed and configured.

---

## **Quick Start Guide**

### 1. Create the ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

```

### 2. Clone the Template Directly into `src`

Move into the `src` directory and clone the template.

Using a dot (`.`) clones the repository **directly into `src`** without creating a subfolder:

```bash
cd ~/ros2_ws/src
git clone <TEMPLATE_URL> .

```

### 3. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

```

Resulting structure:

```
~/ros2_ws/
├── src/
│   ├── hello_world/
│   ├── docs/
│   ├── README.md
│   ├── .gitignore
│   └── .git/
├── build/
├── install/
└── log/

```

### 4. Run the Example

```bash
ros2 launch hello_world hello_world.launch.py

```

If everything works, you should see `"Hello world"` printed in the terminal.

---

## Documentation Organization

### General Documentation (`docs/`)

The `docs/` directory contains all project documentation:

- **README.md** – Overview of the thesis, goals, and basic instructions
- **ARCHITECTURE.md** – System architecture description, diagrams, components (optional)
- **SETUP.md** – Detailed setup and configuration instructions (optional)

### Per-Package Documentation (`docs/packages/`)

Each package should have its own subdirectory with three files:

- **design.md** – Architectural decisions, trade-offs, purpose of the package
- **api.md** – Public interfaces (nodes, topics, services, parameters)
- **usage.md** – How to use the package, examples, troubleshooting

This structure helps supervisors quickly understand your project, evaluate its technical quality, and supports reuse in future thesis work.

---

## Creating New Packages

To add a new package, you may copy the example package:

```bash
cp -r hello_world my_new_package
cd my_new_package

# Update the configuration files:
# - package.xml → change <name> to my_new_package
# - setup.py → update the package_name

```

Then create documentation for your package inside:

```
docs/packages/my_new_package/

```

Alternatively, generate the package using ROS 2:

```bash
ros2 pkg create --build-type ament_python my_new_package --dependencies rclpy

```

---

## Version Control with Git

Configure your Git profile once on your system:

```bash
git config user.name "Your Name"
git config user.email "your.email@uniba.it"

```

During development, follow this workflow to synchronize your work with the remote repository.

### 1. Check the current status

```bash
git status

```

### 2. Add changes to the staging area

```bash
git add .

```

### 3. Commit with a meaningful message

```bash
git commit -m "feat: description of the update"

```

Commit messages should follow the format:

```
<type>: <short description>

```

Allowed types include:

`feat`, `fix`, `docs`, `refactor`, `test`

### 4. Pull updates, build, test, and push

```bash
git pull origin main
colcon build
colcon test
git push origin main

```

If the push succeeds, your commits will appear in the remote repository.

For more detailed Git instructions, refer to the official Collab Thesis Guidelines.

---

## Thesis Guidelines

This template follows the development recommendations of the Collab Laboratory:

[https://collab.di.uniba.it/tesi-di-laurea/come-sviluppare-un-progetto-di-tesi/](https://collab.di.uniba.it/tesi-di-laurea/come-sviluppare-un-progetto-di-tesi/)

Refer to the guidelines for:

- structure and formatting of the written thesis
- development standards
- presentation requirements

The repository should support the written thesis by providing complete, documented, and tested source code.

---

## Support and Assistance

For development questions, contact the technical supervisors assigned by your thesis advisor.

For ROS 2-related issues, refer to the official documentation:

[https://docs.ros.org/](https://docs.ros.org/)

---

## License

If this project is made public, **it must use the MIT License**, which provides:

- freedom to use, modify, and distribute the code;
- the requirement to retain copyright and license notice.

Include the following header in every Python file:

```python
# Copyright (c) 2025 <Your Name>
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

```

Ensure that the repository root contains a `LICENSE` file with the complete MIT license text.
