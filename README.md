# x3plus_bot

The packages for the hardware of the robot

## x3plus_wrapper Package

The `x3plus_wrapper` package maps the driver functionality into the ROS2 world. It includes launch files, RViz configuration files, and parameter files. The main driver code for the robot is contained in the `x3plus_wrapper/x3plus_wrapper/Mecanum_driver_X3Plus.py` file.

### Installation

To install the `x3plus_wrapper` package, follow these steps:

1. Clone the repository:

   ```sh
   git clone https://github.com/cord-burmeister/x3plus_bot.git
   ```

2. Navigate to the `x3plus_wrapper` directory:

   ```sh
   cd x3plus_bot/x3plus_wrapper
   ```

3. Install the package using `colcon`:

   ```sh
   colcon build
   ```

### Usage

To use the `x3plus_wrapper` package, follow these steps:

1. Source the workspace:

   ```sh
   source install/setup.bash
   ```

2. Run the launch file:

   ```sh
   ros2 launch x3plus_wrapper drive_bringup_X3Plus_launch.py
   ```

### Running the Launch File

The `drive_bringup_X3Plus_launch.py` file is used to launch the robot's nodes and configurations. It includes the following nodes:

- `robot_state_publisher`: Publishes the robot's state.
- `joint_state_publisher`: Publishes the joint states.
- `joint_state_publisher_gui`: GUI for publishing joint states.
- `rviz2`: RViz visualization tool.
- `Mecanum_driver_X3Plus`: Main driver node for the robot.

## Github Pages

The web site of the project can be found under

[Github Pages](https://cord-burmeister.github.io/x3plus_docs/)
