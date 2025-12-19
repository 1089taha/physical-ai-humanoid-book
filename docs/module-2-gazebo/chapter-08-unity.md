---
sidebar_position: 4
title: "Chapter 8: Unity for Robotics"
description: "Photorealistic robot visualization and human-robot interaction"
tags: [unity, visualization, hri, unity-robotics-hub]
---

# Chapter 8: Unity for Robotics

## 🎬 The Future of Robot Training

Tesla's Optimus reveal in 2022 wasn't just real footage—much of the promotional material used Unity to showcase robots in photorealistic homes, offices, and factories. Why? Because Unity lets you visualize robots in environments that don't exist yet.

**This chapter: build stunning robot visualizations for demos, training, and human-interaction studies.**

---

## 💡 Core Concepts

Unity Robotics Hub transforms Unity from a game engine into a powerful robotics visualization platform. Unlike Gazebo's focus on physics accuracy, Unity excels at photorealistic rendering and complex environment design. The Unity Robotics Hub provides essential tools: the URDF Importer for bringing ROS 2 robots into Unity, the ROS TCP Connector for real-time communication, and sample environments for testing.

Think of Unity as your robot's "movie studio"—where you create compelling visualizations for stakeholders, generate synthetic training data for AI, and prototype human-robot interactions in photorealistic settings. While Gazebo handles algorithm validation, Unity handles presentation and perception training where visual fidelity matters.

The key advantage is Unity's ability to create complex, photorealistic environments that would be expensive or impossible to build physically. You can simulate a robot navigating a cluttered home, working in a factory, or interacting with humans under various lighting conditions—all while maintaining ROS 2 connectivity for real-time control and data exchange.

---

## ⚙️ Technical Details

### Unity Installation and Robotics Hub Setup

First, install Unity Hub and Unity 2022.3 LTS (Long Term Support version). Then add the Unity Robotics Hub packages:

1. **Unity Robotics Hub**: Core integration package
2. **URDF Importer**: Converts ROS 2 robot descriptions
3. **ROS TCP Connector**: Enables ROS 2 communication
4. **Visual Design Toolkit**: UI and visualization tools

### URDF Import Process

The URDF Importer in Unity converts your robot description into a Unity GameObject hierarchy:

```csharp
// Example C# script for Unity to handle imported robot
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RobotController : MonoBehaviour
{
    [SerializeField]
    private string robotName = "simulation_robot";

    private ROSConnection ros;
    private string jointStateTopic = "/joint_states";

    void Start()
    {
        // Connect to ROS 2
        ros = ROSConnection.instance;

        // Subscribe to joint states
        ros.Subscribe<sensor_msgs.JointStateMsg>(jointStateTopic, OnJointStateReceived);
    }

    void OnJointStateReceived(sensor_msgs.JointStateMsg jointState)
    {
        // Update robot joint positions in Unity
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = (float)jointState.position[i];

            // Find corresponding joint in Unity hierarchy
            Transform jointTransform = transform.Find(jointName);
            if (jointTransform != null)
            {
                // Update joint rotation (example for revolute joints)
                jointTransform.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
            }
        }
    }
}
```

### ROS 2 TCP Endpoint Configuration

Unity communicates with ROS 2 through TCP/IP connections. Here's how to set up the bridge:

**Python script to send robot joint states to Unity:**
```python
#!/usr/bin/env python3
"""
Python script to send robot joint states to Unity
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import socket
import json
import time


class UnityBridge(Node):
    """Bridge between ROS 2 and Unity via TCP"""

    def __init__(self):
        super().__init__('unity_bridge')

        # Create subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Unity TCP connection
        self.unity_host = '127.0.0.1'
        self.unity_port = 10000  # Default Unity port
        self.socket = None

        self.connect_to_unity()

        self.get_logger().info('Unity bridge initialized')

    def connect_to_unity(self):
        """Connect to Unity TCP endpoint"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.unity_host, self.unity_port))
            self.get_logger().info(f'Connected to Unity at {self.unity_host}:{self.unity_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Unity: {e}')

    def joint_state_callback(self, msg):
        """Send joint states to Unity"""
        if self.socket:
            # Create Unity-compatible message
            unity_msg = {
                'robot_name': 'simulation_robot',
                'joint_names': list(msg.name),
                'joint_positions': [float(pos) for pos in msg.position],
                'joint_velocities': [float(vel) for vel in msg.velocity],
                'timestamp': time.time()
            }

            try:
                # Send as JSON string
                json_msg = json.dumps(unity_msg) + '\n'
                self.socket.send(json_msg.encode())
            except Exception as e:
                self.get_logger().error(f'Failed to send to Unity: {e}')

    def destroy_node(self):
        """Clean up TCP connection"""
        if self.socket:
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    bridge = UnityBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Unity Scene Setup

In Unity, create a scene with:
1. **Robot GameObject**: Imported via URDF Importer
2. **ROS TCP Connector**: Handles communication
3. **Camera Rig**: For visualization
4. **Lighting System**: For photorealistic rendering
5. **Environment Objects**: Furniture, obstacles, etc.

The scene should be configured to:
- Run at 60 FPS for smooth visualization
- Use physically-based rendering (PBR) materials
- Include realistic lighting and shadows
- Support real-time ray tracing (if using RTX GPU)

---

## 🛠️ Practical Application

### Exercise: Import Your Robot into Unity and Visualize It Receiving ROS 2 Commands

**Step 1: Install Unity and Robotics Hub**
1. Download Unity Hub from unity3d.com
2. Install Unity 2022.3 LTS
3. Create a new 3D project
4. In Package Manager, install Unity Robotics packages

**Step 2: Import your robot**
1. Go to GameObject → Import Robot from URDF
2. Select your robot's URDF file (the same one from Gazebo)
3. Configure import settings:
   - Base coordinate system: Z-up, Y-forward
   - Import collision objects: Yes (for debugging)
   - Import visual meshes: Yes
   - Import joint limits: Yes

**Step 3: Set up ROS TCP connection**
1. Add ROS TCP Connector to your scene
2. Configure IP address (usually 127.0.0.1 for local)
3. Set appropriate port (default 10000)

**Step 4: Create visualization scripts**
Add the C# script above to handle joint state updates from ROS 2.

**Step 5: Build and run**
1. Build your Unity scene as a standalone application
2. Launch the Unity application
3. Run your ROS 2 nodes that publish joint states
4. Watch your robot move in Unity in real-time

**Step 6: Create a demonstration environment**
1. Add photorealistic environment objects
2. Set up lighting conditions
3. Create camera animations showing robot capabilities
4. Record video for presentations

**What you'll see:** Your robot moving in a photorealistic environment, responding to ROS 2 commands with smooth animations and realistic physics interactions. This visualization is perfect for stakeholder presentations, training materials, and synthetic data generation.

---

## 🎯 Wrap-Up

**Key insights:**
- **Unity excels at photorealistic visualization** while Gazebo focuses on physics accuracy
- **URDF Importer** seamlessly brings ROS 2 robots into Unity
- **ROS TCP Connector** maintains real-time communication
- **Synthetic data generation** is possible with Unity's high-fidelity rendering
- **Human-robot interaction studies** benefit from realistic environments

**Module 2 recap:** You've learned simulation fundamentals, mastered Gazebo for physics-based testing, and explored Unity for photorealistic visualization. These tools form the foundation of modern robotics development—simulating before building, testing algorithms safely, and creating compelling visualizations.

**Next module preview:** Module 3 covers NVIDIA Isaac Sim—advanced simulation for AI training with photorealistic graphics and accurate physics combined.

**You're now equipped to build digital twins that accelerate robot development and reduce hardware risk.**