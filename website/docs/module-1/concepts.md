---
sidebar_position: 3
title: "Core Concepts: Nodes, Topics, Services"
---

# Core Concepts: Nodes, Topics, Services

## Nodes

In ROS 2, a node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system.

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

Topics enable asynchronous message passing between nodes. Multiple nodes can publish and subscribe to the same topic.

## Services

Services provide synchronous request/response communication between nodes.

## Next Steps

Continue to learn about rclpy implementation examples.