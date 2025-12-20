---
sidebar_position: 3
title: "Core Concepts: Nodes, Topics, Services"
---

# Core Concepts: Nodes, Topics, Services

ROS 2 communication is built on three fundamental concepts that form the backbone of robotic systems: nodes, topics, and services. Understanding these concepts is essential for developing distributed robotic applications that can scale from simple single-robot systems to complex multi-robot environments. These concepts provide the foundation for building modular, maintainable, and scalable robotic systems that can handle the complexity of humanoid robots with their multiple sensors, actuators, and control systems.

The publish-subscribe model of topics, the request-response pattern of services, and the action-based communication for long-running tasks create a comprehensive communication framework that addresses different types of interactions required in robotic systems. Each communication pattern has specific use cases and advantages that make them suitable for different scenarios in robotic applications.

## Nodes

In ROS 2, a node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically encapsulates a specific functionality such as sensor processing, motion planning, or control algorithms. Nodes can be written in different programming languages (C++, Python, etc.) and communicate seamlessly through ROS 2's middleware.

Nodes follow the Single Responsibility Principle, where each node should have a clear, well-defined purpose. This modularity allows for easier testing, debugging, and maintenance of robotic systems. Nodes can be launched independently or as part of a larger system using launch files, which coordinate the startup and configuration of multiple nodes.

Node lifecycle management is an important concept in ROS 2, allowing nodes to transition through different states (unconfigured, inactive, active, finalized) based on system requirements. This enables sophisticated system management where nodes can be configured, activated, deactivated, and cleaned up in a controlled manner.

### Creating a Node

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics

Topics enable asynchronous message passing between nodes. Multiple nodes can publish and subscribe to the same topic, creating a publish-subscribe pattern that supports one-to-many and many-to-one communication. Topics are ideal for streaming data like sensor readings, robot states, or control commands.

Topic-based communication is characterized by loose coupling between publishers and subscribers. Publishers don't need to know about subscribers, and subscribers don't need to know about publishers. This allows for flexible system architectures where components can be added or removed without affecting other parts of the system.

Quality of Service (QoS) profiles allow fine-tuning of topic behavior, including reliability settings (reliable vs. best-effort), durability (transient-local vs. volatile), and history depth. These settings are crucial for humanoid robots where some data streams (like safety-critical sensor data) require guaranteed delivery while others (like debugging information) can tolerate occasional losses.

## Services

Services provide synchronous request/response communication between nodes. This pattern is suitable for operations that require immediate responses, such as requesting robot calibration, querying system status, or executing specific actions with guaranteed completion.

Service communication is blocking, meaning the client waits for a response from the server before continuing. This is appropriate for operations that need guaranteed completion before proceeding. Services are typically used for state-changing operations or queries that need immediate answers.

Service definitions are defined using Interface Definition Language (IDL) files that specify the request and response message formats. This ensures type safety and enables automatic code generation for both client and server implementations.

## Actions

Actions provide a communication pattern for long-running tasks that require feedback and goal management. Unlike services, actions don't block the client and provide continuous feedback during execution. Actions are ideal for navigation goals, manipulation tasks, or any operation that takes significant time to complete.

Action communication involves three message types: goal (requesting an action), feedback (intermediate status updates), and result (final outcome). This enables sophisticated interaction patterns where clients can monitor progress, cancel operations, or handle partial results.

## Quality of Service (QoS) Settings

ROS 2 provides Quality of Service settings that allow fine-tuning of communication behavior. QoS profiles include settings for reliability, durability, history, and depth. These settings are crucial for humanoid robots where different data streams have different requirements for reliability and timing.

For example, sensor data might use a best-effort reliability setting to avoid blocking on lost packets, while control commands might use reliable delivery to ensure they reach their destination. Durability settings determine whether messages persist for late-joining subscribers, which is important for state information that new nodes need to know.

## Next Steps

Continue to learn about rclpy implementation examples. The concepts covered in this section form the foundation for all ROS 2 development, and understanding them thoroughly will be essential as you progress through the course and work with more complex robotic systems.