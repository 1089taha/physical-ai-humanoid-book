---
sidebar_position: 6
title: "Chapter 5: Robot Description with URDF"
description: "Creating and visualizing robot models using Unified Robot Description Format"
tags: [urdf, robot-modeling, visualization, kinematics]
---

# Chapter 5: Robot Description with URDF

## Introduction

In the previous chapters, you've learned how to create distributed robotic systems using ROS 2's communication patterns. Now, we'll explore URDF (Unified Robot Description Format)—the standard XML-based language for describing robot geometry, kinematics, and dynamics. URDF is essential for robotics because it provides a unified way to represent your robot's physical structure, enabling simulation, visualization, motion planning, and control.

Think of URDF as your robot's digital blueprint: it defines the shape, size, and relationships of all your robot's components. Without this description, ROS 2 wouldn't know how your robot is constructed, making tasks like collision detection, path planning, and visualization impossible. URDF bridges the gap between abstract robot concepts and concrete physical implementations.

By the end of this chapter, you will:
- Understand URDF's XML structure and core elements
- Create complete robot descriptions with links, joints, and materials
- Visualize robots in RViz and Gazebo simulation environments
- Model complex kinematic chains and robot mechanisms
- Integrate sensors and actuators into robot descriptions
- Generate and validate URDF files for real robotic applications

## Understanding URDF Structure

URDF (Unified Robot Description Format) is an XML-based format that describes a robot's physical and kinematic properties. It defines the robot's structure as a tree of rigid bodies (links) connected by joints, with additional information about geometry, materials, and dynamics.

### Core URDF Elements

**Robot Element**: The root element that contains the entire robot description:
```xml
<robot name="my_robot">
  <!-- Robot description goes here -->
</robot>
```

**Link Elements**: Represent rigid bodies of the robot:
```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

**Joint Elements**: Define connections between links:
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.1 0.2 0.0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

### URDF Components Breakdown

**Visual Elements**: Define how the robot appears in visualization tools:
- Geometry: Shape (box, cylinder, sphere, mesh)
- Material: Color and appearance properties
- Origin: Position and orientation relative to parent

**Collision Elements**: Define collision properties for physics simulation:
- Same geometry specification as visual elements
- Used for collision detection and physics simulation
- Can be simplified for performance

**Inertial Elements**: Define mass properties for dynamics simulation:
- Mass: Total mass of the link
- Inertia: 3x3 inertia matrix (ixx, ixy, ixz, iyy, iyz, izz)

## Creating Your First URDF Robot

Let's create a simple mobile robot with a base and two wheels. Create the following URDF file:

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/urdf/simple_robot.urdf`**

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link - the main body of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 0.8 0.0 1.0"/>
      </material>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting left wheel to base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Joint connecting right wheel to base -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Add a caster wheel for stability -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint connecting caster wheel to base -->
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.15 0 0.05" rpy="0 0 0"/>
  </joint>
</robot>
```

## Visualizing URDF in RViz

To visualize your robot model in RViz, you need to create a launch file and use the robot_state_publisher to publish the robot description and joint states.

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/launch/robot_visualization.launch.py`**

```python
#!/usr/bin/env python3
"""
Launch file for visualizing URDF robot in RViz
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('my_robot_package')

    # Declare launch arguments
    urdf_model_path = LaunchConfiguration('model')
    declare_urdf_model_path = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(pkg_share, 'urdf/simple_robot.urdf'),
        description='Absolute path to robot urdf file'
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_model_path).read()}]
    )

    # Joint state publisher (for visualization)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': True,
            'source_list': ['joint_states']
        }]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz/robot_visualization.rviz')],
        output='screen'
    )

    return LaunchDescription([
        declare_urdf_model_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
```

You'll also need an RViz configuration file:

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/rviz/robot_visualization.rviz`**

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Topic:
        Name: /robot_description
      Update Interval: 0
      Alpha: 1
      Robot Description: robot_description
      TF Prefix: ""
      Show Names: false
      Show Axes: false
      Show Trail: false
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        caster_wheel:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        left_wheel:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        right_wheel:
          Alpha: 1
          Show Axes: false
          Show Trail: false
      Tree:
        root:
          base_link:
            caster_wheel:
              {}
            left_wheel:
              {}
            right_wheel:
              {}
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Frame Timeout: 15
      Marker Scale: 0.5
      Alpha: 1
  Global Options:
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Target Frame: <Fixed Frame>
      Distance: 1.5
      Pitch: 0.3
      Yaw: 0.3
      Roll: 0
      Optical Rotation: false
      Invert Z Axis: false
      Near Clip Distance: 0.01
      Far Clip Distance: 100
      Field of View: 0.75
    Saved: ~
Window Geometry:
  Height: 800
  Width: 1200
```

## Working with Xacro for Complex Robots

Xacro (XML Macros) extends URDF by providing macros, properties, and mathematical expressions, making it easier to create complex robot models without repetition.

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/urdf/robot_with_xacro.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_length" value="0.4" />
  <xacro:property name="base_height" value="0.1" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />

  <!-- Define a wheel macro -->
  <xacro:macro name="wheel" params="prefix parent xyz *origin">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </collision>

      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Use the wheel macro to create left and right wheels -->
  <xacro:wheel prefix="left" parent="base_link" xyz="0 0 0">
    <origin xyz="${base_length/2} ${base_width/2} 0" rpy="0 0 0"/>
  </xacro:wheel>

  <xacro:wheel prefix="right" parent="base_link" xyz="0 0 0">
    <origin xyz="${base_length/2} ${-base_width/2} 0" rpy="0 0 0"/>
  </xacro:wheel>

  <!-- Add a sensor to the robot -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${base_length/2} 0 ${base_height/2}" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

</robot>
```

## Adding Sensors to Your Robot Model

Robots need sensors to perceive their environment. In URDF, sensors are typically represented as additional links connected to the main robot body through fixed joints.

**Example: Adding a 3D camera/LiDAR sensor**

```xml
<!-- Joint connecting sensor to robot -->
<joint name="sensor_mount_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>  <!-- Position on top front of robot -->
</joint>

<!-- Sensor link -->
<link name="sensor_link">
  <visual>
    <geometry>
      <cylinder length="0.05" radius="0.03"/>
    </geometry>
    <material name="silver">
      <color rgba="0.7 0.7 0.7 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <cylinder length="0.05" radius="0.03"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>

<!-- Add Gazebo plugin for sensor simulation -->
<gazebo reference="sensor_link">
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
      <frame_name>sensor_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Kinematic Chains and Robot Arms

For more complex robots like robotic arms, URDF describes the kinematic chain - a series of links connected by joints that can move relative to each other.

**Example: Simple 3-DOF robotic arm**

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base of the arm -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="base_color">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- First joint: rotation around Z axis -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI}" upper="${M_PI}" effort="100" velocity="1.0"/>
  </joint>

  <!-- First link -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
      <material name="link_color">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
      <origin xyz="0 0 0.1" rpy="1.5708 0 0"/>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="1.5708 0 0"/>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Second joint: rotation around Y axis -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1.0"/>
  </joint>

  <!-- Second link -->
  <link name="link2">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
      <material name="link_color">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Third joint: rotation around Z axis -->
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI}" upper="${M_PI}" effort="100" velocity="1.0"/>
  </joint>

  <!-- End effector -->
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="end_color">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

</robot>
```

## Validating and Testing URDF Files

Before using your URDF in simulation or with real robots, it's important to validate it and check for common issues.

### Using check_urdf Tool

```bash
# Install the tool if not already installed
sudo apt install ros-humble-urdfdom-tools

# Check your URDF file
check_urdf ~/ros2_ws/src/my_robot_package/my_robot_package/urdf/simple_robot.urdf
```

### Using urdf_to_graphiz for Visualization

```bash
# Generate a visual representation of the kinematic tree
urdf_to_graphiz ~/ros2_ws/src/my_robot_package/my_robot_package/urdf/simple_robot.urdf

# This creates .dot and .pdf files showing the robot structure
```

### Common URDF Issues and Solutions

1. **Missing inertial properties**: All links should have inertial elements for physics simulation
2. **Incorrect joint limits**: Ensure joint limits are appropriate for your robot
3. **Non-unique names**: All links and joints must have unique names
4. **Invalid parent-child relationships**: Joints must connect existing links
5. **Geometry specification errors**: Check that geometry elements are properly formatted

## Integrating URDF with ROS 2 Nodes

To use your URDF in ROS 2 applications, you need to load it and publish it using the robot_state_publisher.

**Example Python node to load and publish URDF:**

```python
#!/usr/bin/env python3
"""
URDF loader and publisher node
Loads URDF from file and publishes robot description
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import os


class URDFPublisher(Node):
    """Node that loads and publishes URDF description"""

    def __init__(self):
        super().__init__('urdf_publisher')

        # Declare parameters
        self.declare_parameter('urdf_file', 'urdf/simple_robot.urdf')

        # Get URDF file path
        urdf_file = self.get_parameter('urdf_file').value
        pkg_share = get_package_share_directory('my_robot_package')
        urdf_path = os.path.join(pkg_share, urdf_file)

        # Read URDF file
        try:
            with open(urdf_path, 'r') as infp:
                robot_desc = infp.read()

            # Publish robot description
            self.get_logger().info(f'URDF loaded from: {urdf_path}')

            # Create publisher for robot description
            self.robot_desc_publisher = self.create_publisher(
                String, 'robot_description', qos_profile_system_default)

            # Publish the URDF
            msg = String()
            msg.data = robot_desc
            self.robot_desc_publisher.publish(msg)

            self.get_logger().info('Robot description published')

        except FileNotFoundError:
            self.get_logger().error(f'URDF file not found: {urdf_path}')
            return

        # Create joint state publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', qos_profile_system_default)

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Initialize joint positions
        self.joint_positions = {
            'left_wheel_joint': 0.0,
            'right_wheel_joint': 0.0
        }

    def publish_joint_states(self):
        """Publish joint states for visualization"""
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_state_publisher.publish(msg)

        # Update joint positions (simulate movement)
        self.joint_positions['left_wheel_joint'] += 0.1
        self.joint_positions['right_wheel_joint'] += 0.1


def main(args=None):
    import rclpy
    from std_msgs.msg import String

    rclpy.init(args=args)
    node = URDFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Hands-On Exercise: Create a Complete Robot Model

**Title:** Build a Differential Drive Robot with Sensors

**Objective:** Create a complete URDF model of a differential drive robot with sensors, visualize it in RViz, and prepare it for simulation

**Time:** 90 minutes

### Exercise Implementation

Create a comprehensive URDF for a differential drive robot:

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/urdf/differential_robot.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="differential_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Robot dimensions -->
  <xacro:property name="base_width" value="0.4" />
  <xacro:property name="base_length" value="0.6" />
  <xacro:property name="base_height" value="0.2" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="wheel_base" value="0.4" />
  <xacro:property name="caster_radius" value="0.05" />

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue"/>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
    </visual>

    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="0.916" iyz="0.0" izz="1.25"/>
    </inertial>
  </link>

  <!-- Macro for wheels -->
  <xacro:macro name="wheel" params="prefix *origin">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black"/>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </collision>

      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Wheels -->
  <xacro:wheel prefix="left">
    <origin xyz="0 ${wheel_base/2} ${wheel_radius}" rpy="0 0 0"/>
  </xacro:wheel>

  <xacro:wheel prefix="right">
    <origin xyz="0 ${-wheel_base/2} ${wheel_radius}" rpy="0 0 0"/>
  </xacro:wheel>

  <!-- Caster wheels for stability -->
  <joint name="front_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_caster"/>
    <origin xyz="${base_length/2 - caster_radius} 0 ${caster_radius}" rpy="0 0 0"/>
  </joint>

  <link name="front_caster">
    <visual>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
      <material name="black"/>
    </visual>

    <collision>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Camera sensor -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${base_length/2 - 0.05} 0 ${base_height - 0.05}" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- IMU sensor -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
      <material name="black"/>
    </visual>

    <collision>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-06" ixy="0.0" ixz="0.0" iyy="1e-06" iyz="0.0" izz="1e-06"/>
    </inertial>
  </link>

  <!-- Gazebo plugins for simulation -->
  <gazebo>
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <update_rate>30</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>${wheel_base}</wheel_separation>
      <wheel_diameter>${2 * wheel_radius}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

### Launch File for Visualization

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/launch/differential_robot.launch.py`**

```python
#!/usr/bin/env python3
"""
Launch file for differential robot visualization
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ'
    )

    # Robot description parameter
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot_package'),
        'urdf',
        'differential_robot.urdf.xacro'
    ])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': [
                '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="differential_robot">',
                '  <xacro:include filename="', urdf_path, '" />',
                '</robot>'
            ]
        }]
    )

    # Joint state publisher with GUI
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz node
    rviz_node = Node(
        condition=LaunchConfiguration('use_rviz'),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('my_robot_package'),
            'rviz',
            'robot_visualization.rviz'
        ])]
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
```

### Testing the Robot Model

To visualize your robot model:

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash

# Launch the visualization
ros2 launch my_robot_package differential_robot.launch.py
```

## Key Takeaways

URDF provides a comprehensive framework for describing robot models:

- **XML structure** defines links (rigid bodies) and joints (connections) in a tree structure
- **Visual, collision, and inertial elements** describe appearance, physics, and dynamics properties
- **Xacro macros** simplify complex robot models and reduce redundancy
- **Sensors and plugins** can be integrated for simulation and real-world use
- **Proper validation** ensures models work correctly in simulation and with real robots
- **Integration with ROS 2** enables visualization, simulation, and control applications

URDF is fundamental to robotics development as it enables simulation, visualization, motion planning, and control. A well-designed URDF model is essential for creating effective robotic applications.

## Summary

Module 1 has provided you with a comprehensive foundation in Physical AI and ROS 2. You've learned:

1. **Physical AI Concepts**: How AI moves from digital spaces into the physical world, understanding the unique challenges and opportunities of embodied intelligence.

2. **Foundations**: The sensor and actuator systems that enable robots to perceive and interact with their environment, including the critical perception-action loop.

3. **ROS 2 Architecture**: The distributed middleware that connects all robot components, including nodes, packages, workspaces, and Quality of Service settings.

4. **Communication Patterns**: The three fundamental ROS 2 communication mechanisms - topics for streaming data, services for request-response interactions, and actions for goal-oriented behaviors.

5. **Robot Description**: How to create detailed robot models using URDF, enabling visualization, simulation, and control.

This foundation prepares you for advanced robotics development and sets the stage for Modules 2-5, where you'll explore simulation environments, advanced control systems, and specialized robotic applications.

## Further Reading

- URDF tutorials in the ROS 2 documentation
- Xacro documentation and advanced usage examples
- "Robotics, Vision and Control" by Peter Corke for kinematics and dynamics
- Gazebo simulation tutorials for integrating URDF with physics simulation
- ROS 2 Navigation and MoveIt! tutorials for advanced robot applications