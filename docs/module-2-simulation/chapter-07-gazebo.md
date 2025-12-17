---
sidebar_position: 3
title: "Chapter 7: Mastering Gazebo"
description: "Spawn robots, simulate sensors, and integrate with ROS 2"
tags: [gazebo, ros2, urdf, sensor-simulation]
---

# Chapter 7: Mastering Gazebo

## 🎬 The NASA Mars Rover Story

Before Perseverance landed on Mars in 2021, it drove thousands of virtual kilometers in Gazebo. Engineers simulated Mars terrain, tested wheel traction on sand, validated sensor performance in dust storms—all from Earth. When it landed? It worked perfectly because Gazebo predicted every challenge.

**This chapter: you'll spawn robots, add sensors, and control them with ROS 2—just like NASA.**

---

## 💡 Core Concepts

Gazebo serves as the bridge between your robot's digital design (URDF) and its virtual behavior. When you create a robot model in URDF, Gazebo brings it to life by applying physics, simulating sensors, and connecting to ROS 2 for real-time control. The magic happens through Gazebo plugins that translate ROS 2 messages into simulated robot behavior and vice versa.

Think of Gazebo as a virtual physics laboratory where you can test your robot's responses to forces, collisions, and environmental conditions without risking expensive hardware. The beauty lies in its seamless integration with ROS 2—messages you send to your real robot work identically in simulation, making the transition from virtual to physical remarkably smooth.

[DIAGRAM: ROS 2 ↔ Gazebo bridge showing topic flow: /cmd_vel → Gazebo → /odom back to ROS 2]

The core architecture involves three key components working together: your robot's URDF description defines the physical structure, Gazebo plugins handle the simulation logic, and ROS 2 topics provide the communication interface. When you send a velocity command to `/cmd_vel`, Gazebo's differential drive plugin calculates wheel forces, applies physics, and publishes odometry data back to your ROS 2 system—exactly as a real robot would behave.

---

## ⚙️ Technical Details

### URDF Structure for Gazebo Integration

For Gazebo to properly simulate your robot, you need to extend your URDF with Gazebo-specific tags. These include physics properties, visual materials, and plugin configurations:

```xml
<?xml version="1.0"?>
<robot name="simulation_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Robot properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.05" />
  <xacro:property name="wheel_width" value="0.02" />
  <xacro:property name="wheel_base" value="0.2" />

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
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
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
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
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 ${wheel_base/2} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 ${-wheel_base/2} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Gazebo plugins for simulation -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Differential drive plugin -->
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
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
    </plugin>
  </gazebo>

  <!-- Camera sensor plugin -->
  <gazebo reference="base_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>base_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Launch File for Gazebo Integration

To spawn your robot in Gazebo with ROS 2 integration, create a launch file that coordinates the robot_state_publisher, Gazebo, and your robot model:

```python
#!/usr/bin/env python3
"""
Launch file for Gazebo robot simulation
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty')

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
            'world': world,
            'verbose': 'false',
        }.items()
    )

    # Robot description parameter
    robot_description = PathJoinSubstitution([
        FindPackageShare('my_robot_package'),
        'urdf',
        'simulation_robot.urdf.xacro'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': [
                '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simulation_robot">',
                '  <xacro:include filename="', robot_description, '" />',
                '</robot>'
            ]
        }]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simulation_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=use_sim_time),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])
```

---

## 🛠️ Practical Application

### Exercise: Spawn a Wheeled Robot with Camera in Gazebo, Control via ROS 2

**Step 1: Create the robot URDF file**
Save the URDF code above as `simulation_robot.urdf.xacro` in your package's urdf directory.

**Step 2: Create the launch file**
Save the launch file code as `gazebo_simulation.launch.py` in your package's launch directory.

**Step 3: Build and launch**
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash

# Launch Gazebo with your robot
ros2 launch my_robot_package gazebo_simulation.launch.py
```

**Step 4: Test robot control**
In a new terminal, test the robot's movement:
```bash
# Make sure you source the workspace
source ~/ros2_ws/install/setup.bash

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

**Step 5: Monitor sensor data**
Watch the camera feed and odometry:
```bash
# Check available topics
ros2 topic list | grep -E "(camera|odom)"

# View camera images (if you have image_view installed)
ros2 run image_view image_view_raw image:=/camera_sensor/image_raw
```

**Step 6: Experiment**
- Try different velocity commands to see how the robot responds
- Add obstacles in Gazebo and test navigation
- Modify the URDF to change robot properties and see effects
- Test sensor data accuracy under different conditions

This exercise demonstrates the complete pipeline: URDF → Gazebo → ROS 2 topics, which is fundamental to robotics simulation.

---

## 🎯 Wrap-Up

**Key insights:**
- **Gazebo plugins** bridge URDF models with ROS 2 communication
- **Differential drive plugin** simulates wheeled robot motion accurately
- **Sensor plugins** provide realistic camera, LiDAR, and IMU data
- **Launch files** coordinate all components for seamless simulation
- **Simulation enables rapid testing** without hardware risk

**Next chapter:** Unity robotics—photorealistic visualization and human-robot interaction for advanced simulation scenarios.

**You now have the foundation to simulate any robot in Gazebo with full ROS 2 integration.**