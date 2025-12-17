---
sidebar_position: 5
title: "Chapter 4: ROS 2 Communication Patterns"
description: "Mastering topics, services, and actions for robot communication"
tags: [topics, services, actions, communication, messaging]
---

# Chapter 4: ROS 2 Communication Patterns

## Introduction

In the previous chapter, you created your first ROS 2 nodes and began exploring the distributed architecture of robotic systems. Now, we'll dive deep into the three fundamental communication patterns that enable sophisticated robot behavior: topics, services, and actions. These patterns form the backbone of all ROS 2 communication, allowing different parts of your robot to share information, request specific operations, and coordinate complex tasks.

Understanding these communication patterns is crucial because they determine how efficiently your robot can process sensor data, execute commands, and respond to its environment. Topics enable continuous data streaming like sensor feeds, services provide synchronous request-response interactions for specific operations, and actions coordinate long-running tasks with feedback and cancellation capabilities.

By the end of this chapter, you will:
- Master the three primary ROS 2 communication patterns and their appropriate use cases
- Implement publisher-subscriber systems for continuous data streaming
- Create client-server systems for synchronous operations
- Build action-based systems for goal-oriented behaviors
- Design effective communication architectures for complex robotic applications
- Understand Quality of Service (QoS) settings for different communication needs

## Topics: Asynchronous Data Streaming

Topics are the primary mechanism for asynchronous, one-to-many communication in ROS 2. They work on a publish-subscribe model where publishers send messages to named topics and subscribers receive messages from those topics. This pattern is ideal for continuous data streams like sensor readings, robot state information, or processed data.

### How Topics Work

In the topic communication model:
- **Publishers** send messages to specific topic names
- **Subscribers** receive messages from specific topic names
- The ROS 2 middleware handles message delivery between publishers and subscribers
- Multiple publishers can send to the same topic, and multiple subscribers can receive from the same topic
- Communication is asynchronous—publishers don't wait for responses from subscribers

### Topic Characteristics

Topics are perfect for:
- Sensor data streams (camera images, LiDAR scans, IMU readings)
- Robot state broadcasts (joint positions, battery status, current pose)
- Processed information (detected objects, navigation goals)
- System status updates (CPU usage, memory, diagnostics)

### Quality of Service for Topics

Topics support various QoS settings that affect their behavior:
- **Reliability**: Reliable (all messages delivered) or Best Effort (some messages may be dropped)
- **Durability**: Volatile (only new messages) or Transient Local (late joiners receive recent messages)
- **History**: Keep All or Keep Last N messages
- **Depth**: Number of messages to keep in publisher/subscriber queues

### Example: Sensor Data Publisher

```python
#!/usr/bin/env python3
"""
Sensor data publisher example
Demonstrates continuous data streaming using topics
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import random
import math


class SensorPublisher(Node):
    """Publishes simulated sensor data using topics"""

    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publisher for laser scan data
        self.publisher = self.create_publisher(LaserScan, 'laser_scan', 10)

        # Timer for periodic publishing
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_scan)

        # Laser scan parameters
        self.angle_min = -math.pi / 2  # -90 degrees
        self.angle_max = math.pi / 2   # 90 degrees
        self.angle_increment = math.pi / 180  # 1 degree
        self.scan_time = 0.1
        self.time_increment = 0.0

        self.get_logger().info('Sensor publisher started')

    def publish_scan(self):
        """Publish simulated laser scan data"""
        msg = LaserScan()

        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Set scan parameters
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.scan_time = self.scan_time
        msg.time_increment = self.time_increment

        # Calculate number of ranges
        num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        msg.ranges = []

        # Generate simulated distance readings
        for i in range(num_ranges):
            # Simulate distances with some obstacles
            angle = self.angle_min + i * self.angle_increment
            distance = 3.0  # Default distance

            # Add some obstacles
            if 0.2 < abs(angle) < 0.3:
                distance = 1.0  # Close obstacle
            elif 0.5 < abs(angle) < 0.6:
                distance = 1.5  # Medium obstacle
            else:
                # Add some random variation
                distance += random.uniform(-0.1, 0.1)

            msg.ranges.append(distance)

        # Set infinite range for invalid readings
        msg.range_min = 0.1
        msg.range_max = 10.0

        self.publisher.publish(msg)
        self.get_logger().info(f'Published laser scan with {len(msg.ranges)} ranges')


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()

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

### Example: Multiple Subscribers

```python
#!/usr/bin/env python3
"""
Multiple subscribers example
Shows how multiple nodes can subscribe to the same topic
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ObstacleDetector(Node):
    """Detects obstacles from laser scan data"""

    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg):
        """Process laser scan for obstacles"""
        # Find minimum distance in scan
        valid_ranges = [r for r in msg.ranges if 0.1 < r < 10.0]  # Filter valid ranges
        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < 1.0:  # Obstacle within 1 meter
                self.get_logger().warn(f'OBSTACLE DETECTED: {min_distance:.2f}m away')
            else:
                self.get_logger().info(f'Clear path: {min_distance:.2f}m to nearest obstacle')


class SafetyMonitor(Node):
    """Monitors safety from laser scan data"""

    def __init__(self):
        super().__init__('safety_monitor')
        self.subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg):
        """Monitor safety based on scan data"""
        # Count points within safety threshold
        dangerous_points = [r for r in msg.ranges if 0.1 < r < 0.5]  # Very close obstacles
        if len(dangerous_points) > 3:  # More than 3 very close points
            self.get_logger().fatal('DANGER: Multiple close obstacles detected!')
        elif len(dangerous_points) > 0:
            self.get_logger().error(f'WARNING: {len(dangerous_points)} close obstacles')


def main(args=None):
    rclpy.init(args=args)

    # Create both subscriber nodes
    detector = ObstacleDetector()
    monitor = SafetyMonitor()

    # Use MultiThreadedExecutor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(monitor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Services: Synchronous Request-Response

Services provide synchronous, request-response communication between nodes. When a client sends a request to a service, it waits for the server to process the request and return a response. This pattern is ideal for operations that have a clear input and output, like saving data, performing calculations, or controlling hardware.

### How Services Work

In the service communication model:
- **Service Servers** provide specific functionality and wait for requests
- **Service Clients** send requests and wait for responses
- Communication is synchronous—clients block until they receive a response
- Only one server typically handles requests for a given service name
- Perfect for operations that require a definitive result

### Service Characteristics

Services are ideal for:
- Saving calibration data or maps
- Performing computational tasks (path planning, image processing)
- Hardware control operations (enable/disable sensors, set parameters)
- Database operations (query, update, delete)
- System configuration changes

### Example: Distance Calculation Service

First, we need to define a custom service message. Create the service definition file:

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/srv/CalculateDistance.srv`**

```
# Request: two points in 2D space
float64 x1
float64 y1
float64 x2
float64 y2
---
# Response: distance between points
float64 distance
```

Now create the service server:

```python
#!/usr/bin/env python3
"""
Distance calculation service server
Demonstrates service-based synchronous communication
"""
import rclpy
from rclpy.node import Node
from my_robot_package.srv import CalculateDistance  # Custom service
import math


class DistanceService(Node):
    """Service server that calculates distance between two points"""

    def __init__(self):
        super().__init__('distance_service')

        # Create service
        self.srv = self.create_service(
            CalculateDistance,
            'calculate_distance',
            self.calculate_distance_callback)

        self.get_logger().info('Distance calculation service started')

    def calculate_distance_callback(self, request, response):
        """Calculate distance between two points"""
        dx = request.x2 - request.x1
        dy = request.y2 - request.y1
        distance = math.sqrt(dx*dx + dy*dy)

        response.distance = distance

        self.get_logger().info(
            f'Calculated distance between ({request.x1}, {request.y1}) '
            f'and ({request.x2}, {request.y2}): {distance:.2f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DistanceService()

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

And the corresponding service client:

```python
#!/usr/bin/env python3
"""
Distance calculation service client
Demonstrates service-based synchronous communication
"""
import rclpy
from rclpy.node import Node
from my_robot_package.srv import CalculateDistance  # Custom service
import sys


class DistanceClient(Node):
    """Service client that requests distance calculations"""

    def __init__(self):
        super().__init__('distance_client')

        # Create client
        self.cli = self.create_client(CalculateDistance, 'calculate_distance')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Distance client ready')

    def send_request(self, x1, y1, x2, y2):
        """Send request to calculate distance"""
        request = CalculateDistance.Request()
        request.x1 = x1
        request.y1 = y1
        request.x2 = x2
        request.y2 = y2

        # Call service asynchronously
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    client = DistanceClient()

    # Example: Calculate distance between (0,0) and (3,4)
    x1, y1 = 0.0, 0.0
    x2, y2 = 3.0, 4.0

    try:
        response = client.send_request(x1, y1, x2, y2)
        if response:
            print(f'Distance between ({x1},{y1}) and ({x2},{y2}): {response.distance:.2f}')
            print(f'Expected: 5.00 (3-4-5 triangle)')
        else:
            print('Service call failed')
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions: Goal-Oriented Communication

Actions are designed for long-running tasks that provide feedback during execution and can be canceled. They're perfect for operations like navigation, manipulation, or any task that takes time to complete and may need to report progress or be interrupted.

### How Actions Work

Actions involve three message types:
- **Goal**: Specifies what the action should do
- **Feedback**: Provides status updates during execution
- **Result**: Contains the final outcome when the action completes

The action flow:
1. Client sends a goal to the action server
2. Server processes the goal and sends feedback periodically
3. Client can monitor progress and cancel if needed
4. Server sends final result when complete

### Action Characteristics

Actions are perfect for:
- Navigation to a goal location
- Manipulation tasks that take time
- Calibration procedures
- Long computational processes
- Any task that needs progress feedback or cancellation

### Example: Navigation Action

First, define the action message:

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/action/NavigateToPose.action`**

```
# Goal: target pose
geometry_msgs/Pose target_pose
---
# Result: success status
bool reached
string message
---
# Feedback: current progress
geometry_msgs/Pose current_pose
float64 distance_remaining
string status
```

Now create the action server:

```python
#!/usr/bin/env python3
"""
Navigation action server
Demonstrates goal-oriented communication with feedback
"""
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from my_robot_package.action import NavigateToPose
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time
import math


class NavigationActionServer(Node):
    """Action server for navigation tasks"""

    def __init__(self):
        super().__init__('navigation_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Current robot position (simulated)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.get_logger().info('Navigation action server started')

    def goal_callback(self, goal_request):
        """Accept or reject a goal"""
        # Check if goal is valid (in bounds, reachable, etc.)
        if self.is_goal_valid(goal_request):
            self.get_logger().info('Accepting navigation goal')
            return GoalResponse.ACCEPT
        else:
            self.get_logger().info('Rejecting navigation goal')
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('Received request to cancel navigation goal')
        return CancelResponse.ACCEPT

    def is_goal_valid(self, goal_request):
        """Check if the goal is valid"""
        # Simple validation: check if goal is not too far
        dx = goal_request.target_pose.position.x - self.current_x
        dy = goal_request.target_pose.position.y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)

        return distance < 100.0  # Max 100m navigation

    def execute_callback(self, goal_handle):
        """Execute the navigation goal"""
        self.get_logger().info('Executing navigation goal')

        # Get target position
        target_x = goal_handle.request.target_pose.position.x
        target_y = goal_handle.request.target_pose.position.y

        # Simulate navigation with feedback
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        # Calculate initial distance
        start_x, start_y = self.current_x, self.current_y
        total_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)
        current_distance = total_distance

        # Navigation loop
        step_size = 0.1  # Move 0.1m per step
        steps = int(total_distance / step_size)

        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.reached = False
                result.message = 'Goal canceled'
                self.get_logger().info('Navigation goal canceled')
                return result

            # Update current position (simulated movement)
            progress = i / steps
            self.current_x = start_x + (target_x - start_x) * progress
            self.current_y = start_y + (target_y - start_y) * progress

            # Calculate remaining distance
            remaining_distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

            # Update feedback
            feedback_msg.current_pose.position.x = self.current_x
            feedback_msg.current_pose.position.y = self.current_y
            feedback_msg.distance_remaining = remaining_distance
            feedback_msg.status = f'Navigating... {progress*100:.1f}% complete'

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Log progress
            self.get_logger().info(f'Navigation progress: {progress*100:.1f}% - {remaining_distance:.2f}m remaining')

            # Simulate movement time
            time.sleep(0.1)

            # Check if we're close enough to target
            if remaining_distance < 0.1:  # Within 0.1m of target
                break

        # Check final result
        final_distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

        if final_distance <= 0.1:
            goal_handle.succeed()
            result.reached = True
            result.message = f'Reached destination at ({target_x:.2f}, {target_y:.2f})'
            self.get_logger().info(f'Navigation successful: reached ({self.current_x:.2f}, {self.current_y:.2f})')
        else:
            goal_handle.abort()
            result.reached = False
            result.message = f'Failed to reach destination, {final_distance:.2f}m away'
            self.get_logger().info(f'Navigation failed: {final_distance:.2f}m from target')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()

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

And the action client:

```python
#!/usr/bin/env python3
"""
Navigation action client
Demonstrates goal-oriented communication with feedback
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_package.action import NavigateToPose
from geometry_msgs.msg import Pose, Point, Quaternion


class NavigationActionClient(Node):
    """Action client for navigation tasks"""

    def __init__(self):
        super().__init__('navigation_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')

    def send_goal(self, x, y):
        """Send navigation goal to action server"""
        # Wait for action server
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.target_pose = Pose()
        goal_msg.target_pose.position = Point(x=x, y=y, z=0.0)
        goal_msg.target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Send goal and get future
        self.get_logger().info(f'Sending navigation goal to ({x}, {y})')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Add done callback
        send_goal_future.add_done_callback(self.goal_response_callback)

        return send_goal_future

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result future
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.status}, '
            f'Distance remaining: {feedback.distance_remaining:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationActionClient()

    # Send a navigation goal
    future = action_client.send_goal(5.0, 3.0)  # Navigate to (5, 3)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Comparing Communication Patterns

| Pattern | Communication Type | Use Case | Example |
|---------|-------------------|----------|---------|
| Topics | Asynchronous, many-to-many | Continuous data streaming | Sensor data, robot state |
| Services | Synchronous, request-response | Specific operations with results | Map saving, path planning |
| Actions | Asynchronous with feedback | Long-running tasks | Navigation, manipulation |

## Designing Communication Architectures

When designing your robot's communication system, consider these guidelines:

### Topic Design
- Use topics for data that changes continuously
- Choose appropriate QoS settings based on data importance
- Name topics descriptively (e.g., `/sensors/lidar_scan` vs `/data`)
- Consider message frequency and network bandwidth

### Service Design
- Use services for operations with clear input/output
- Keep service operations relatively fast (under a few seconds if possible)
- Design services to be idempotent when possible
- Consider using services for system configuration

### Action Design
- Use actions for tasks that take significant time
- Provide meaningful feedback messages
- Implement proper cancellation handling
- Design actions to be interruptible when appropriate

## Hands-On Exercise: Multi-Pattern Communication System

**Title:** Build an Integrated Robot Control System

**Objective:** Create a system that uses all three communication patterns to control a simulated robot

**Time:** 90 minutes

### Implementation

```python
#!/usr/bin/env python3
"""
Integrated robot control system
Combines topics, services, and actions for comprehensive robot control
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool, Int16
from my_robot_package.srv import CalculateDistance
from my_robot_package.action import NavigateToPose
from geometry_msgs.msg import Pose, Point
import time
import math
import random


class RobotController(Node):
    """Main robot controller that integrates all communication patterns"""

    def __init__(self):
        super().__init__('robot_controller')

        # Publishers for status and sensor simulation
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)
        self.sensor_publisher = self.create_publisher(String, 'sensor_data', 10)

        # Subscribers for commands
        self.command_subscription = self.create_subscription(
            String, 'robot_commands', self.command_callback, 10)

        # Service server for robot operations
        self.service = self.create_service(
            CalculateDistance, 'robot_operation', self.operation_callback)

        # Action server for navigation
        self.nav_action_server = ActionServer(
            self, NavigateToPose, 'robot_navigate',
            execute_callback=self.nav_execute_callback,
            goal_callback=self.nav_goal_callback)

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.battery_level = 100.0
        self.is_moving = False

        # Start sensor simulation timer
        self.timer = self.create_timer(1.0, self.publish_sensor_data)

        self.get_logger().info('Robot controller initialized')

    def command_callback(self, msg):
        """Handle robot commands via topics"""
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')

        if command == 'status':
            status_msg = String()
            status_msg.data = f'Position: ({self.current_x:.2f}, {self.current_y:.2f}), Battery: {self.battery_level:.1f}%'
            self.status_publisher.publish(status_msg)
        elif command == 'charge':
            if self.battery_level < 100.0:
                self.battery_level = min(100.0, self.battery_level + 20.0)
                self.get_logger().info(f'Battery charged to {self.battery_level:.1f}%')
        elif command == 'move_random':
            # Generate random target and start navigation
            target_x = random.uniform(-10.0, 10.0)
            target_y = random.uniform(-10.0, 10.0)

            # This would normally use an action client, but we'll simulate here
            self.get_logger().info(f'Moving to random position ({target_x:.2f}, {target_y:.2f})')

    def operation_callback(self, request, response):
        """Handle robot operations via services"""
        operation = request.x1  # Use x1 to determine operation type

        if operation == 1:  # Calculate distance
            dx = request.x2 - self.current_x
            dy = request.y2 - self.current_y
            distance = math.sqrt(dx*dx + dy*dy)
            response.distance = distance
            self.get_logger().info(f'Calculated distance to ({request.x2}, {request.y2}): {distance:.2f}')
        elif operation == 2:  # Check battery
            response.distance = self.battery_level
            self.get_logger().info(f'Battery level: {self.battery_level:.1f}%')
        else:
            response.distance = -1.0
            self.get_logger().warn(f'Unknown operation: {operation}')

        return response

    def nav_goal_callback(self, goal_request):
        """Handle navigation goal"""
        if self.battery_level > 10.0:  # Need at least 10% battery
            return self.GoalResponse.ACCEPT
        else:
            return self.GoalResponse.REJECT

    def nav_execute_callback(self, goal_handle):
        """Execute navigation goal"""
        self.get_logger().info('Starting navigation task')

        target_x = goal_handle.request.target_pose.position.x
        target_y = goal_handle.request.target_pose.position.y

        # Calculate path
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)

        # Simulate navigation
        steps = int(distance / 0.1)  # 0.1m steps
        result = NavigateToPose.Result()

        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.reached = False
                result.message = 'Navigation canceled'
                return result

            # Update position
            progress = i / steps
            self.current_x = self.current_x + dx * 0.1/distance
            self.current_y = self.current_y + dy * 0.1/distance

            # Consume battery
            self.battery_level = max(0.0, self.battery_level - 0.01)

            # Send feedback
            feedback = NavigateToPose.Feedback()
            feedback.current_pose.position.x = self.current_x
            feedback.current_pose.position.y = self.current_y
            feedback.distance_remaining = distance - (progress * distance)
            feedback.status = f'Navigating... {progress*100:.1f}% complete'
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        # Check if successful
        final_dx = target_x - self.current_x
        final_dy = target_y - self.current_y
        final_distance = math.sqrt(final_dx*final_dx + final_dy*final_dy)

        if final_distance <= 0.2:  # Within 0.2m
            goal_handle.succeed()
            result.reached = True
            result.message = f'Navigation successful, reached ({self.current_x:.2f}, {self.current_y:.2f})'
        else:
            goal_handle.abort()
            result.reached = False
            result.message = f'Navigation failed, {final_distance:.2f}m from target'

        return result

    def publish_sensor_data(self):
        """Simulate and publish sensor data"""
        sensor_msg = String()
        sensor_msg.data = f'Pos:({self.current_x:.2f},{self.current_y:.2f}) Bat:{self.battery_level:.1f}%'
        self.sensor_publisher.publish(sensor_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Testing the System

To test the integrated system, you can use ROS 2 command-line tools:

```bash
# Send commands via topics
ros2 topic pub /robot_commands std_msgs/String "data: 'status'"

# Call services
ros2 service call /robot_operation my_robot_package/srv/CalculateDistance "{x1: 1.0, y1: 0.0, x2: 5.0, y2: 0.0}"

# Send action goals (using action client)
ros2 action send_goal /robot_navigate my_robot_package/action/NavigateToPose "{target_pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## Key Takeaways

ROS 2 communication patterns provide flexible ways to connect different parts of your robot system:

- **Topics** enable asynchronous, continuous data streaming perfect for sensor data and state broadcasts
- **Services** provide synchronous request-response communication for specific operations with clear results
- **Actions** coordinate long-running tasks with progress feedback and cancellation capabilities
- **Quality of Service settings** allow fine-tuning of communication behavior for different requirements
- **Proper pattern selection** is crucial for system performance and maintainability

Understanding when to use each communication pattern is essential for designing efficient and robust robotic systems. The choice depends on factors like timing requirements, data importance, and the nature of the interaction between components.

**Next Chapter Preview:** In Chapter 5, we'll explore URDF (Unified Robot Description Format)—the standard for describing robot geometry, kinematics, and dynamics. You'll learn to create detailed robot models and visualize them in simulation environments.

## Further Reading

- ROS 2 Communication Patterns: docs.ros.org/en/humble/Tutorials/Topics/Understanding-ROS2-Topics.html
- Service and Action tutorials in the ROS 2 documentation
- "Effective Robotics Programming with ROS" by Anis Koubaa
- URDF tutorials for robot description and visualization