---
sidebar_position: 4
title: "Chapter 11: Navigation for Humanoids"
description: "Path planning and obstacle avoidance for bipedal robots"
tags: [nav2, path-planning, humanoid-navigation, obstacle-avoidance]
---

# Chapter 11: Navigation for Humanoids

## 🎬 The Bipedal Challenge

When Tesla's Optimus needs to walk across a factory floor, it can't just follow a line like a warehouse robot. It must:
- Plan footsteps dynamically
- Maintain balance while turning
- Step over/around obstacles
- Adjust for uneven terrain

**This chapter: adapt Nav2 (ROS 2's navigation framework) for humanoid locomotion.**

---

## 💡 Core Concepts

Navigation for wheeled robots and bipedal robots differ fundamentally in their kinematic constraints and stability requirements. While a differential drive robot can rotate in place or follow smooth curves, a humanoid must plan each step carefully to maintain balance and avoid collisions.

**Why Wheeled Navigation Differs from Bipedal:**
Wheeled robots operate with continuous motion—they can move forward, backward, and rotate smoothly. Humanoids, however, operate with discrete stepping motions. Each foot placement must be planned for both geometric positioning and dynamic stability.

Key differences:
- **Motion Continuity:** Wheeled robots move continuously; humanoids step discretely
- **Stability Constraints:** Humanoids must maintain balance during movement
- **Terrain Adaptation:** Humanoids can step over obstacles; wheeled robots must go around
- **Foot Placement:** Humanoids need precise foot placement for stability

**Nav2 Architecture for Humanoids:**
Nav2 traditionally consists of three main components:
- **Global Planner:** Creates a high-level path from start to goal
- **Local Planner:** Handles real-time obstacle avoidance and path following
- **Controller:** Converts path commands to motor commands

For humanoids, we enhance this architecture with:
- **Footstep Planner:** Translates continuous paths to discrete steps
- **Balance Controller:** Ensures center of mass stays within support polygon
- **Terrain Analyzer:** Assesses walkability of surfaces

**Footstep Planning vs. Differential Drive:**
Instead of publishing velocity commands, humanoid navigation systems generate a sequence of foot placements. Each footstep must satisfy:
- Reachability (within leg workspace)
- Stability (maintains center of mass)
- Collision-free (avoids obstacles)
- Kinematically feasible (achievable by inverse kinematics)

**Balance Constraints During Navigation:**
Humanoid stability relies on maintaining the center of mass within the support polygon defined by feet contact points. This creates additional constraints:
- Minimum step width to prevent tipping
- Maximum step length to maintain reachability
- Timing coordination between steps
- Upper body posture adjustments

**Zero-Moment Point (ZMP) Stability:**
The ZMP represents the point where the net moment of ground reaction forces equals zero. For stable walking, the ZMP must remain within the support polygon formed by the feet. Navigation systems must consider ZMP constraints when planning paths and footstep sequences.

[DIAGRAM: Nav2 pipeline showing: Goal → Global Planner → Local Planner → Footstep Generator → Balance Controller → Motors]

---

## ⚙️ Technical Details

### Installing and Configuring Nav2 for Humanoids

Nav2 installation remains standard, but configuration requires humanoid-specific parameters:

```bash
# Install Nav2 packages
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-isaac-ros-navigation

# Install humanoid-specific packages
sudo apt install -y ros-humble-footstep-planners
sudo apt install -y ros-humble-humanoid-navigation-msgs
```

### Creating Costmaps for Humanoids

Humanoid costmaps must account for:
- Leg workspace limitations
- Step height capabilities
- Balance constraints
- Terrain roughness

```yaml
# humanoid_costmap_params.yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: true
  resolution: 0.05  # Higher resolution for precise foot placement
  inflation_radius: 0.6  # Account for leg swing space

  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 5.0
  static_map: false
  rolling_window: true
  width: 5.0
  height: 5.0
  resolution: 0.025  # Very fine resolution for footstep planning

  plugins:
    - {name: voxel_layer, type: "nav2_costmap_2d::VoxelLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
```

### Custom Footstep Planner Implementation

```python
# footstep_planner.py
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration

class FootstepPlanner:
    def __init__(self, step_length=0.3, step_width=0.2, max_step_height=0.1):
        self.step_length = step_length
        self.step_width = step_width
        self.max_step_height = max_step_height

        # Humanoid kinematic constraints
        self.min_step_length = 0.1
        self.max_step_length = 0.4
        self.max_step_yaw = np.radians(20)  # Max turn per step

    def plan_path(self, start_pose, goal_pose, costmap):
        """
        Generate footstep sequence from start to goal
        Returns: List of PoseStamped for each footstep
        """
        footsteps = []
        current_pose = start_pose
        current_foot = 'left'  # Start with left foot

        # Calculate required steps using RRT or A* for initial path
        path = self.global_plan(current_pose, goal_pose, costmap)

        # Convert path to footsteps
        for i in range(len(path.poses)):
            # Calculate foot placement based on path direction
            step_pose = self.calculate_foot_placement(
                current_pose,
                path.poses[i],
                current_foot
            )

            # Validate step feasibility
            if self.is_step_feasible(step_pose, costmap, current_foot):
                footsteps.append(step_pose)
                current_pose = step_pose
                current_foot = 'right' if current_foot == 'left' else 'left'
            else:
                # Plan alternative step
                alt_step = self.find_alternative_step(
                    current_pose,
                    path.poses[i],
                    costmap,
                    current_foot
                )
                if alt_step:
                    footsteps.append(alt_step)
                    current_pose = alt_step
                    current_foot = 'right' if current_foot == 'left' else 'left'

        return footsteps

    def calculate_foot_placement(self, current_pose, target_pose, foot_side):
        """Calculate optimal foot placement for given target"""
        # Calculate desired step vector
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        distance = np.sqrt(dx*dx + dy*dy)

        # Normalize and scale to step length
        if distance > self.min_step_length:
            scale = min(self.step_length, distance)
            step_x = current_pose.pose.position.x + (dx/distance) * scale
            step_y = current_pose.pose.position.y + (dy/distance) * scale
        else:
            step_x = target_pose.pose.position.x
            step_y = target_pose.pose.position.y

        # Adjust for foot side (left/right offset)
        target_yaw = self.quaternion_to_yaw(target_pose.pose.orientation)
        if foot_side == 'left':
            step_x += self.step_width * np.cos(target_yaw + np.pi/2)
            step_y += self.step_width * np.sin(target_yaw + np.pi/2)
        else:  # right foot
            step_x += self.step_width * np.cos(target_yaw - np.pi/2)
            step_y += self.step_width * np.sin(target_yaw - np.pi/2)

        # Create step pose
        step_pose = PoseStamped()
        step_pose.header.frame_id = 'map'
        step_pose.header.stamp.sec = 0
        step_pose.header.stamp.nanosec = 0
        step_pose.pose.position.x = step_x
        step_pose.pose.position.y = step_y
        step_pose.pose.position.z = 0.0  # Ground level
        step_pose.pose.orientation = target_pose.pose.orientation

        return step_pose

    def is_step_feasible(self, step_pose, costmap, foot_side):
        """Check if step is kinematically and collision-free"""
        # Check collision at step location
        if self.is_collision_at(step_pose, costmap):
            return False

        # Check reachability (within leg workspace)
        if self.is_out_of_workspace(step_pose):
            return False

        # Check step height constraint
        if self.is_step_too_high(step_pose, costmap):
            return False

        return True

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)

# Integration with Nav2
class HumanoidNavigator:
    def __init__(self):
        # Initialize Nav2 components
        self.nav2_client = ActionClient('navigate_to_pose', NavigateToPose)
        self.footstep_planner = FootstepPlanner()

    def navigate_with_footsteps(self, goal_pose):
        """Navigate using footstep planning"""
        # Plan footsteps to goal
        footsteps = self.footstep_planner.plan_path(
            self.get_current_pose(),
            goal_pose,
            self.get_local_costmap()
        )

        # Execute footsteps with balance control
        for step in footsteps:
            self.execute_footstep(step)
            self.wait_for_balance()
```

### Integrating with Isaac Sim Humanoid Model

To test navigation in Isaac Sim, we need to connect Nav2 to the simulated humanoid:

```yaml
# isaac_sim_nav2_bridge.yaml
/**:
  ros__parameters:
    # Humanoid model parameters
    model_name: 'humanoid'
    joint_names: [
      'left_hip_roll', 'left_hip_yaw', 'left_hip_pitch',
      'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
      'right_hip_roll', 'right_hip_yaw', 'right_hip_pitch',
      'right_knee', 'right_ankle_pitch', 'right_ankle_roll'
    ]

    # Navigation parameters
    controller_frequency: 50.0
    max_linear_speed: 0.5  # m/s
    max_angular_speed: 0.5  # rad/s
    step_duration: 0.8  # seconds per step

    # Balance parameters
    com_height: 0.8  # Center of mass height
    support_polygon_margin: 0.05  # Safety margin for ZMP
```

### Real-time Obstacle Avoidance While Walking

Humanoid navigation must handle dynamic obstacle avoidance differently than wheeled robots:

```python
class HumanoidLocalPlanner:
    def __init__(self):
        self.current_footsteps = []
        self.step_index = 0
        self.balance_controller = BalanceController()

    def update_local_plan(self, sensor_data):
        """Update plan based on new sensor data"""
        if self.is_path_blocked(sensor_data):
            # Stop current step execution
            self.halt_movement()

            # Recalculate footsteps from current position
            new_footsteps = self.replan_from_current_position(
                self.get_current_position(),
                self.goal_position
            )

            # Validate new plan with balance constraints
            if self.validate_balance_sequence(new_footsteps):
                self.current_footsteps = new_footsteps
                self.step_index = 0
                return True
            else:
                # Emergency stop
                self.activate_emergency_balance()
                return False

        return True

    def execute_step_with_obstacle_avoidance(self, step_pose):
        """Execute single step while monitoring for obstacles"""
        # Pre-check step safety
        if not self.is_step_safe(step_pose):
            return False

        # Begin step execution
        self.start_step_execution(step_pose)

        # Monitor during execution
        while not self.step_complete():
            # Check for new obstacles in path
            if self.detect_impending_collision():
                self.emergency_stop_step()
                return False

            # Maintain balance during step
            self.balance_controller.update()

        return True
```

---

## 🛠️ Practical Application

### Exercise: Make a humanoid navigate to a goal in Isaac Sim using Nav2

**Goal:** Implement complete humanoid navigation pipeline in Isaac Sim.

**Step 1: Launch Isaac Sim with humanoid model**
```bash
# Start Isaac Sim with humanoid scene
cd ~/.local/share/ov/pkg/isaac-sim-4.0.0/
./isaac-sim.bat --exec "from omni.isaac.examples.simple_robots import HUMANOID_ENV"
```

**Step 2: Launch Nav2 stack with humanoid parameters**
```xml
<!-- launch/humanoid_nav2.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    nav2_bringup_launch_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # Navigation lifecycle manager
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']

    nav_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'diff_drive_controller/cmd_vel_unstamped')]
    )

    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Footstep planner node (custom)
    footstep_planner = Node(
        package='humanoid_navigation',
        executable='footstep_planner_node',
        name='footstep_planner',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        nav_lifecycle_manager,
        controller_server,
        planner_server,
        footstep_planner
    ])
```

**Step 3: Configure navigation parameters**
```yaml
# config/humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.0
      wz_std: 0.3
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.1
      wz_max: 0.5
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      rot_stopped_velocity: 0.25
      simulate_with_random_deviation: false
      publish_cost_grid_pc: false
      transform_tolerance: 0.1
      backup_restore_timeout: 0.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

footstep_planner:
  ros__parameters:
    use_sim_time: True
    step_length: 0.3
    step_width: 0.2
    max_step_height: 0.1
    step_duration: 0.8
    com_height: 0.8
    support_polygon_margin: 0.05
```

**Step 4: Send navigation goal**
```bash
# In a new terminal
source ~/nav2_ws/install/setup.bash
ros2 run nav2_example_nav2_examples nav2_humanoid_demo

# Or manually send goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

**Step 5: Monitor navigation**
Watch the humanoid:
- Plan footsteps to reach goal
- Navigate around obstacles
- Maintain balance during movement
- Achieve goal position safely

**Expected behavior:**
- Smooth footstep planning
- Dynamic obstacle avoidance
- Stable walking gait
- Accurate goal reaching

---

## 🎯 Wrap-Up

**Key insights:**
- **Humanoid navigation requires discrete footstep planning** instead of continuous velocity control
- **Balance constraints** fundamentally change path planning requirements
- **Integration with Isaac Sim** enables safe testing of navigation algorithms
- **Real-time obstacle avoidance** must account for ongoing step execution
- **ZMP stability** ensures safe bipedal locomotion

**Module 4 preview:** Vision-Language-Action (VLA) models that combine perception, language understanding, and robotic control for human-like interaction.

**You've mastered the complete pipeline from simulation to real-world humanoid navigation.**