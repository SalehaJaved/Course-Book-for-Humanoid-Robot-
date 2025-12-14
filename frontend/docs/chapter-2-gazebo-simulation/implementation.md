# Gazebo Simulation Implementation: Creating Your First Virtual Robot

## Prerequisites

Before starting this implementation, make sure you have:
- ROS 2 Humble installed (from Chapter 1)
- Basic understanding of ROS 2 concepts (nodes, topics, packages)
- A working ROS 2 workspace

## Installing Gazebo

First, let's install Gazebo and the ROS 2 integration packages:

```bash
# Update package lists
sudo apt update

# Install Gazebo (Fortress version for ROS 2 Humble)
sudo apt install ros-humble-gazebo-*

# Install ROS 2 Gazebo integration packages
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Install robot description packages
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

## Creating a Simple Robot Model

Let's create a simple wheeled robot model that we can simulate in Gazebo.

### Step 1: Create a Robot Description Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Create a new package for robot descriptions
ros2 pkg create --build-type ament_python my_robot_description --dependencies rclpy std_msgs geometry_msgs sensor_msgs

# Navigate to the package directory
cd ~/ros2_ws/src/my_robot_description
```

### Step 2: Create the URDF Robot Model

Create a directory for your robot models:

```bash
mkdir -p my_robot_description/urdf
```

Create a file called `simple_robot.urdf` in the urdf directory:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Robot base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints connecting wheels to base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Add a camera to the robot -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.05" rpy="0 0 0"/>
  </joint>
</robot>
```

### Step 3: Create a Launch File

Create a launch directory and file:

```bash
mkdir -p my_robot_description/launch
```

Create a file called `simple_robot_gazebo.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_robot_description = FindPackageShare('my_robot_description')

    # World file
    world_file = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'worlds',
        'empty.world'
    ])

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(
                os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'urdf',
                    'simple_robot.urdf'
                )
            ).read()
        }]
    )

    # Spawn entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

### Step 4: Create a Control Node

Let's create a simple control node to move our robot:

```bash
mkdir -p my_robot_description/my_robot_description
```

Create a file called `robot_controller.py`:

```python
#!/usr/bin/env python3
"""
Simple robot controller that moves the robot forward and turns
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create timer to send commands periodically
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Control state
        self.state = 'forward'  # 'forward' or 'turn'
        self.state_start_time = self.get_clock().now()
        self.forward_duration = 5.0  # seconds to move forward
        self.turn_duration = 3.0     # seconds to turn

    def timer_callback(self):
        msg = Twist()

        current_time = self.get_clock().now()
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9

        if self.state == 'forward':
            msg.linear.x = 0.5  # Move forward at 0.5 m/s
            msg.angular.z = 0.0  # No turning

            if elapsed > self.forward_duration:
                self.state = 'turn'
                self.state_start_time = current_time
        else:  # turning state
            msg.linear.x = 0.0   # Stop moving forward
            msg.angular.z = 0.5  # Turn at 0.5 rad/s

            if elapsed > self.turn_duration:
                self.state = 'forward'
                self.state_start_time = current_time

        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Update setup.py

Update the setup.py file to make the Python script executable:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple robot description package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_description.robot_controller:main',
        ],
    },
)
```

## Building and Running the Simulation

### Step 1: Build the Package

```bash
# Navigate to your workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select my_robot_description

# Source the workspace
source install/setup.bash
```

### Step 2: Launch the Simulation

```bash
# Launch Gazebo with your robot
ros2 launch my_robot_description simple_robot_gazebo.launch.py
```

This will open Gazebo with your simple robot in the empty world.

### Step 3: Control the Robot

Open a new terminal and run:

```bash
# Source the workspace
cd ~/ros2_ws
source install/setup.bash

# Run the robot controller
ros2 run my_robot_description robot_controller
```

You should now see your robot moving forward for 5 seconds, then turning for 3 seconds, repeating this pattern.

## Alternative: Manual Robot Control

Instead of using the automated controller, you can manually control the robot using ROS 2 commands:

```bash
# Move the robot forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Turn the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

## Adding a Camera to Your Robot

To add camera functionality, you'll need to modify your URDF file to include a camera plugin. Add this to your URDF inside the robot tag:

```xml
<!-- Camera plugin -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/simple_robot</namespace>
        <remapping>~/out/image_raw:=/camera/image_raw</remapping>
        <remapping>~/out/camera_info:=/camera/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
      <update_rate>30</update_rate>
    </plugin>
  </sensor>
</gazebo>
```

## Viewing Camera Data

To view the camera feed from your simulated robot:

```bash
# In a new terminal, after launching the simulation
cd ~/ros2_ws
source install/setup.bash

# Echo camera images (they will be in binary format)
ros2 topic echo /camera/image_raw

# Or use image_view to visualize the camera feed
sudo apt install ros-humble-image-view
ros2 run image_view image_view /image:=/camera/image_raw
```

## Practical Exercises

### Exercise 1: Modify Robot Appearance
Change the colors and sizes of your robot parts in the URDF file to create a robot that looks different.

### Exercise 2: Create a Square Path
Modify the robot controller to make the robot drive in a square pattern instead of forward and turn.

### Exercise 3: Add LIDAR Sensor
Add a LIDAR sensor to your robot by adding a ray plugin to your URDF file.

### Exercise 4: Create Custom World
Create your own world file with obstacles and test how your robot navigates around them.

## Troubleshooting Common Issues

1. **Robot doesn't appear in Gazebo**: Check that the URDF file is valid and that the spawn_entity node is working properly.

2. **Robot doesn't move**: Verify that the differential drive plugin is correctly configured and that you're publishing to the correct topic.

3. **Gazebo crashes**: This can happen with complex models; try simplifying your robot or reducing physics complexity.

4. **No response to commands**: Check that the topic names match between your controller and the Gazebo plugin.

## Next Steps

Now that you've created your first simulated robot, you can:
- Add more complex sensors (LIDAR, IMU, etc.)
- Create more interesting environments
- Implement more sophisticated control algorithms
- Explore ROS 2 navigation stack for autonomous navigation

This implementation guide gives you the foundation to create and control simulated robots in Gazebo. The concepts learned here will be essential as we move forward to more advanced robotics topics!