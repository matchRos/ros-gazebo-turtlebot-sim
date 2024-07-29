# TurtleBot3 Simulation with ROS "noetic" and Gazebo

This repo demonstrates how to set up and run **TurtleBot3** simulations
using **ROS "noetic" LTS** and **Gazebo** with **Github Actions**.
It includes simulation tests to verify the robot's movement.

`ros-gazebo-turtlebot-sim` is a fork of
[MechanicalRock/ros-github-actions-tutorial](https://github.com/MechanicalRock/ros-github-actions-tutorial),
with the following updates:

- Add `test_bot_velocity.py` and `my_turtlebot_velocity.test` to the base `my_turtlebot_sim` package
  - Used to simulate a virtual environment with Gazebo, then spawn a ROS robot within it
- Update ROS 1 version to "noetic" LTS
  - Target Ubuntu 20.04 (Focal) with GitHub Actions

## Prerequisites

- ROS (Robot Operating System) "Noetic" LTS
- Gazebo
- TurtleBot3 packages

## Setup

1. **Clone the repository:**

   ```bash
   git clone https://github.com/jcassady/ros-gazebo-turtlebot-sim.git
   cd catkin_ws/
   ```

2. **Install dependencies:**

   ```bash
   sudo apt-get update
   sudo apt-get install -y \
       ros-noetic-turtlebot3 \
       ros-noetic-turtlebot3-simulations \
       ros-noetic-gazebo-ros-pkgs \
       ros-noetic-gazebo-ros-control
   ```

3. **Build the workspace:**

   ```bash
   source /opt/ros/noetic/setup.bash
   catkin_make
   ```

4. **Sourcing the build's setup script:**

   ```bash
   source catkin_ws/devel/setup.bash
   ```

## Running Simulation Tests Locally

### Base Movement Test

This test checks if the TurtleBot3 moves a specified distance from
its spawn point.

**Run the test:**

```bash
TURTLEBOT3_MODEL=burger rostest my_turtlebot_sim my_turtlebot_sim.test --text
```

### Added Velocity Test

As part of the fork, a velocity test has been added to verify the
speed and direction of TurtleBot3.

**Run the test:**

```bash
TURTLEBOT3_MODEL=waffle rostest my_turtlebot_sim velocity.test --text
```

## Notes

- Ensure that the ROS environment is properly sourced before running the tests.
- Modify the test files as needed to suit your specific requirements.

## Running Simulation Tests with GitHub Actions

This repository contains workflows for running simulations and tests on TurtleBot3
using ROS and Gazebo. The workflows are defined in YAML files located in the
`.github/workflows` directory.

## GitHub Actions

### 1. Simulation Workflow (`sim.yml`)

This is the original workflow included in the fork's source code, but updated for ROS Noetic.
When triggered on pull requests, it runs a simulation of the TurtleBot3 in a
Gazebo environment to ensure that the changes do not break the simulation.

**Key Steps:**

- **Setup ROS and Gazebo:** Installs the necessary ROS and Gazebo packages
- **Launch Simulation:** Starts the TurtleBot3 simulation in Gazebo
- **Run Tests:** Executes predefined tests to verify the behavior of the robot in the simulated environment

### 2. Velocity Test Workflow (`velocity.yml`)

This workflow is designed to test the velocity commands of the TurtleBot3
It ensures that the robot responds correctly to velocity commands

**Key Steps:**

- **Setup ROS and Gazebo:** Installs the necessary ROS and Gazebo packages
- **Launch Simulation:** Starts the TurtleBot3 simulation in Gazebo
- **Send Velocity Commands:** Sends a series of velocity commands to the robot
- **Verify Responses:** Checks the robot's responses to the velocity commands to ensure they are as expected

## References

- [ROS GitHub Actions Tutorial by MechanicalRock](https://github.com/MechanicalRock/ros-github-actions-tutorial)
- [Creating a Robotics Simulation Pipeline with GitHub Actions and ROS](https://formant.io/blog/creating-a-robotics-simulation-pipeline-with-github-actions-and-ros/)
