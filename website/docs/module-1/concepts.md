---
sidebar_position: 3
title: "Core Concepts: Nodes, Topics, Services"
---

# Core Concepts: Nodes, Topics, Services

ROS 2 communication is built on three fundamental concepts that form the backbone of robotic systems: nodes, topics, and services. Understanding these concepts is essential for developing distributed robotic applications that can scale from simple single-robot systems to complex multi-robot environments.

## Nodes

In ROS 2, a node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically encapsulates a specific functionality such as sensor processing, motion planning, or control algorithms. Nodes can be written in different programming languages (C++, Python, etc.) and communicate seamlessly through ROS 2's middleware.

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

## Services

Services provide synchronous request/response communication between nodes. This pattern is suitable for operations that require immediate responses, such as requesting robot calibration, querying system status, or executing specific actions with guaranteed completion.

## Next Steps

Continue to learn about rclpy implementation examples.