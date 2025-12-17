---
sidebar_position: 4
title: "Chapter 3: ROS 2 Architecture and Your First Node"
description: "Understanding ROS 2 architecture and creating your first distributed robot application"
tags: [ros2-architecture, nodes, packages, workspaces]
---

# Chapter 3: ROS 2 Architecture and Your First Node

## Introduction

Welcome to the heart of modern robotics development! In this chapter, you'll dive deep into ROS 2 (Robot Operating System 2) architecture—the middleware that connects all the hardware and software components you learned about in previous chapters. ROS 2 isn't actually an operating system, but rather a flexible framework that enables different parts of a robot's software to communicate seamlessly, whether those parts run on the same computer or distributed across multiple machines.

Think of ROS 2 as the robot's nervous system: it carries sensory information from sensors to processors, transmits control commands to actuators, and enables different software components to work together harmoniously. Understanding ROS 2 architecture is crucial because it provides the communication backbone that connects all the concepts you've learned and will continue to learn throughout this course.

By the end of this chapter, you will:
- Understand the DDS-based architecture that makes ROS 2 robust and scalable
- Install and configure ROS 2 Humble Hawksbill on your system
- Create your first ROS 2 workspace and package
- Develop and run your first ROS 2 nodes
- Learn about the package management system and workspace organization
- Implement a simple distributed system with multiple communicating nodes

## Understanding ROS 2 Architecture

ROS 2 is built on a modern, distributed architecture that addresses many limitations of the original ROS while maintaining its core philosophy of modularity and reusability. The architecture is designed around the Data Distribution Service (DDS) standard, which provides reliable, real-time communication between different parts of a robotic system.

### The DDS Foundation

DDS (Data Distribution Service) is an industry-standard middleware that provides a publisher-subscriber communication model. In ROS 2, DDS handles the low-level networking details, allowing developers to focus on robot behavior rather than communication protocols. DDS ensures that messages are delivered reliably between nodes, even when they're running on different machines or when network conditions change.

Key DDS features that benefit robotics include:
- **Quality of Service (QoS) policies**: Allow fine-tuning of communication behavior (reliability, durability, deadline, etc.)
- **Discovery mechanism**: Nodes automatically find each other on the network
- **Data modeling**: Clear definitions of message types and structures
- **Platform independence**: Works across different operating systems and hardware architectures

### Core Architecture Components

**Nodes** are the fundamental building blocks of ROS 2 applications. Each node is a process that performs a specific function within the robot system. A typical robot might have nodes for:
- Sensor data processing
- Motion control
- Path planning
- User interface
- Data logging

Nodes are designed to be modular and reusable, allowing developers to create complex systems by combining simple, well-defined components.

**Packages** are containers for related ROS 2 functionality. A package typically contains:
- Source code (nodes, libraries, interfaces)
- Configuration files
- Launch files
- Documentation
- Tests

Packages promote code reuse and make it easier to share functionality across different robot projects.

**Workspaces** are directories where you organize and build your ROS 2 packages. A typical workspace contains:
- Source directory (where you develop packages)
- Build directory (where compiled code goes)
- Install directory (where final executables are placed)
- Log directory (where runtime logs are stored)

### Quality of Service (QoS) Profiles

QoS profiles allow you to specify communication requirements for different types of data:

- **Reliability**: Choose between "reliable" (all messages delivered) or "best effort" (messages may be dropped)
- **Durability**: Decide whether late-joining subscribers receive old messages ("transient local") or only new ones ("volatile")
- **History**: Control how many messages to keep in memory
- **Deadline**: Specify maximum time between consecutive messages

## Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the current Long-Term Support (LTS) release, making it ideal for learning and production applications. Here's how to install it on Ubuntu 22.04:

### Step 1: Set up locale
```bash
locale  # Check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Set up sources
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### Step 4: Environment setup
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Install additional tools
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

## Creating Your First ROS 2 Workspace

A ROS 2 workspace is where you'll develop and organize your robot applications. Let's create your first workspace:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Creating Your First ROS 2 Package

Now let's create your first ROS 2 package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_package --dependencies rclpy std_msgs
```

This creates a new Python package named `my_robot_package` with dependencies on `rclpy` (ROS 2 Python client library) and `std_msgs` (standard message types).

## Your First ROS 2 Node

Let's create your first ROS 2 node. Navigate to your package directory and create a simple publisher node:

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/simple_publisher.py`**

```python
#!/usr/bin/env python3
"""
Simple publisher node example
Demonstrates basic ROS 2 node creation and topic publishing
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """Simple publisher node that sends messages to a topic"""

    def __init__(self):
        super().__init__('simple_publisher')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'robot_messages', 10)

        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track message sequence
        self.i = 0

        self.get_logger().info('Simple publisher node initialized')

    def timer_callback(self):
        """Callback function called by the timer"""
        msg = String()
        msg.data = f'Hello from robot! Message #{self.i}'

        # Publish the message
        self.publisher.publish(msg)

        # Log the message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1


def main(args=None):
    """Main function to run the node"""
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Now let's create a corresponding subscriber node:

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/simple_subscriber.py`**

```python
#!/usr/bin/env python3
"""
Simple subscriber node example
Demonstrates basic ROS 2 node creation and topic subscription
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """Simple subscriber node that receives messages from a topic"""

    def __init__(self):
        super().__init__('simple_subscriber')

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'robot_messages',
            self.listener_callback,
            10)  # Queue size

        # Make sure the subscription is properly configured
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Simple subscriber node initialized')

    def listener_callback(self, msg):
        """Callback function called when a message is received"""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function to run the node"""
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Making Nodes Executable and Updating Setup Files

Make the Python files executable:

```bash
chmod +x ~/ros2_ws/src/my_robot_package/my_robot_package/simple_publisher.py
chmod +x ~/ros2_ws/src/my_robot_package/my_robot_package/simple_subscriber.py
```

Update the `setup.py` file in your package to make the nodes available as executables:

**File: `~/ros2_ws/src/my_robot_package/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simple robot package for learning ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_package.simple_publisher:main',
            'simple_subscriber = my_robot_package.simple_subscriber:main',
        ],
    },
)
```

## Building and Running Your Nodes

First, rebuild your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
```

Source the workspace:

```bash
source install/setup.bash
```

Now you can run your nodes! Open two terminal windows:

**Terminal 1 (Publisher):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_package simple_publisher
```

**Terminal 2 (Subscriber):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_package simple_subscriber
```

You should see the publisher sending messages and the subscriber receiving them.

## Understanding the Node Lifecycle

ROS 2 nodes follow a specific lifecycle that ensures proper resource management:

1. **Initialization**: The node is created and its components are initialized
2. **Activation**: The node becomes active and can start processing
3. **Execution**: The node runs and performs its functions
4. **Shutdown**: The node cleans up resources and exits gracefully

Proper lifecycle management is crucial for robust robot applications. The example nodes above demonstrate the proper pattern:
- Initialize ROS 2 with `rclpy.init()`
- Create and configure the node
- Enter the spin loop to process callbacks
- Handle cleanup in a finally block

## Advanced Node Features

### Parameters

ROS 2 nodes can accept parameters that can be configured at runtime:

```python
#!/usr/bin/env python3
"""
Node with parameters example
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParameterNode(Node):
    """Node that demonstrates parameter usage"""

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('message_prefix', 'Robot says: ')
        self.declare_parameter('publish_rate', 1.0)

        # Get parameter values
        self.message_prefix = self.get_parameter('message_prefix').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'parameter_messages', 10)

        # Create timer with parameter-controlled rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.message_prefix}Message #{self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

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

You can run this node with custom parameters:

```bash
ros2 run my_robot_package parameter_node --ros-args -p message_prefix:="Custom prefix: " -p publish_rate:=2.0
```

## Hands-On Exercise: Create a Distributed Sensor Node System

**Title:** Build a Distributed Temperature Monitoring System

**Objective:** Create multiple ROS 2 nodes that simulate a temperature monitoring system with publishers and subscribers

**Time:** 60 minutes

### Setup
Ensure you're in your workspace:
```bash
cd ~/ros2_ws
source install/setup.bash
```

### Implementation

Create a temperature publisher node:

**File: `~/ros2_ws/src/my_robot_package/my_robot_package/temperature_publisher.py`**

```python
#!/usr/bin/env python3
"""
Temperature sensor publisher node
Simulates a temperature sensor publishing readings
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperaturePublisher(Node):
    """Publishes simulated temperature readings"""

    def __init__(self):
        super().__init__('temperature_publisher')

        # Create publisher for temperature topic
        self.publisher = self.create_publisher(Float32, 'temperature', 10)

        # Timer for periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_temperature)

        # Simulate base temperature around 22°C with variation
        self.base_temp = 22.0
        self.temp_variance = 5.0

        self.get_logger().info('Temperature publisher started')

    def publish_temperature(self):
        # Simulate temperature reading with some variation
        temperature = self.base_temp + random.uniform(-self.temp_variance, self.temp_variance)

        msg = Float32()
        msg.data = temperature

        self.publisher.publish(msg)
        self.get_logger().info(f'Published temperature: {temperature:.2f}°C')


class TemperatureSubscriber(Node):
    """Subscribes to temperature readings and processes them"""

    def __init__(self):
        super().__init__('temperature_subscriber')

        # Create subscription to temperature topic
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10)

        self.subscription  # prevent unused variable warning
        self.get_logger().info('Temperature subscriber started')

    def temperature_callback(self, msg):
        temp = msg.data

        # Determine temperature status
        if temp < 18.0:
            status = "COLD"
        elif temp > 26.0:
            status = "HOT"
        else:
            status = "COMFORTABLE"

        self.get_logger().info(f'Temperature: {temp:.2f}°C - Status: {status}')


def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    publisher = TemperaturePublisher()
    subscriber = TemperatureSubscriber()

    # Create an executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Update the setup.py file to include the new executable:

```python
entry_points={
    'console_scripts': [
        'simple_publisher = my_robot_package.simple_publisher:main',
        'simple_subscriber = my_robot_package.simple_subscriber:main',
        'temperature_monitor = my_robot_package.temperature_publisher:main',
    ],
},
```

Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash
```

Run the temperature monitoring system:
```bash
ros2 run my_robot_package temperature_monitor
```

### Expected Output
```
[INFO] [1697482345.123456]: Temperature publisher started
[INFO] [1697482345.123457]: Temperature subscriber started
[INFO] [1697482345.623456]: Published temperature: 23.45°C
[INFO] [1697482345.623457]: Temperature: 23.45°C - Status: COMFORTABLE
[INFO] [1697482346.123456]: Published temperature: 17.89°C
[INFO] [1697482346.123457]: Temperature: 17.89°C - Status: COLD
```

### Extension Challenge
Modify the system to:
1. Add multiple temperature sensors at different locations
2. Create a node that aggregates readings from multiple sensors
3. Implement temperature trend analysis (rising, falling, stable)

## Key Takeaways

ROS 2 architecture provides a robust foundation for building distributed robotic systems:

- **DDS-based communication** ensures reliable, real-time message passing between nodes
- **Nodes** are modular, reusable components that perform specific functions
- **Packages** organize related functionality and promote code reuse
- **Workspaces** provide a structured environment for development and building
- **Quality of Service (QoS) profiles** allow fine-tuning of communication behavior
- **Parameters** enable runtime configuration of node behavior

Understanding these architectural concepts is essential for developing scalable and maintainable robotic applications. The distributed nature of ROS 2 allows you to build complex systems by combining simple, well-defined components.

**Next Chapter Preview:** In Chapter 4, we'll explore ROS 2's communication patterns in detail—topics for streaming data, services for request-response interactions, and actions for goal-oriented behaviors. You'll learn how to design effective communication architectures for your robotic systems.

## Further Reading

- ROS 2 Documentation: docs.ros.org
- "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart
- DDS specification and its applications in robotics
- ROS 2 Design: docs.ros.org/en/rolling/Concepts/About-Data-Distribution-Service-DDS.html