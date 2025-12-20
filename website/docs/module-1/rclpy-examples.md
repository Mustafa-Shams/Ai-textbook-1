---
sidebar_position: 4
title: "rclpy Implementation Examples"
---

# rclpy Implementation Examples

This section provides practical implementation examples using rclpy, the Python client library for ROS 2. These examples demonstrate the fundamental patterns of publisher-subscriber communication that form the backbone of robotic systems. Understanding these patterns is crucial for developing real-world robotic applications where multiple components must coordinate seamlessly.

The examples provided here serve as templates for more complex implementations you'll encounter in humanoid robotics applications. Each pattern can be extended with additional features such as parameter management, custom message types, and sophisticated data processing pipelines.

## Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publisher Node with Advanced Features

A more sophisticated publisher node might include error handling, parameter management, and custom message types:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math

class AdvancedPublisher(Node):
    def __init__(self):
        super().__init__('advanced_publisher')

        # Create publisher with custom QoS settings
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)

        # Publisher for joint states (common in humanoid robots)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('robot_name', 'humanoid_robot')

        # Get parameter values
        publish_rate = self.get_parameter('publish_rate').value
        self.robot_name = self.get_parameter('robot_name').value

        # Create timer based on parameter
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.i = 0

        self.get_logger().info(f'Advanced publisher started for {self.robot_name}')

    def timer_callback(self):
        # Publish status message
        status_msg = String()
        status_msg.data = f'{self.robot_name} status: operational - cycle {self.i}'
        self.publisher_.publish(status_msg)

        # Publish joint states (simulated for humanoid robot)
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        joint_msg.position = [
            math.sin(self.i * 0.1),      # Hip
            math.cos(self.i * 0.15),     # Knee
            math.sin(self.i * 0.2)       # Ankle
        ]
        self.joint_publisher.publish(joint_msg)

        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    advanced_publisher = AdvancedPublisher()
    rclpy.spin(advanced_publisher)
    advanced_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Subscriber Node

An advanced subscriber might include multiple subscriptions, message filtering, and complex processing:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class AdvancedSubscriber(Node):
    def __init__(self):
        super().__init__('advanced_subscriber')

        # Multiple subscriptions for different message types
        self.status_sub = self.create_subscription(
            String, 'robot_status', self.status_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.velocity_callback, 10)

        # Publisher for processed data
        self.result_pub = self.create_publisher(String, 'processed_data', 10)

        self.joint_data = None
        self.velocity_data = None

    def status_callback(self, msg):
        self.get_logger().info(f'Status received: {msg.data}')

        # Process status and potentially publish result
        result_msg = String()
        result_msg.data = f'Processed: {msg.data}'
        self.result_pub.publish(result_msg)

    def joint_callback(self, msg):
        self.joint_data = msg
        self.get_logger().info(f'Joint positions: {msg.position[:3]}...')

    def velocity_callback(self, msg):
        self.velocity_data = msg
        self.get_logger().info(f'Velocity command: {msg.linear.x}, {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    advanced_subscriber = AdvancedSubscriber()
    rclpy.spin(advanced_subscriber)
    advanced_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

These examples demonstrate the fundamental publish-subscribe pattern in ROS 2. In real robotic applications, publishers might represent sensor data (camera images, LIDAR scans, IMU readings) while subscribers might represent control systems that react to this data. The publisher-subscriber pattern enables loose coupling between components, allowing for more robust and maintainable robotic systems.

## Service Client and Server Examples

In addition to publishers and subscribers, ROS 2 nodes often implement service clients and servers for synchronous communication:

**Service Server:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

**Service Client:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
```

## Next Steps

Explore URDF structures for humanoid joints and links. The programming patterns demonstrated here will be essential as you work with more complex robotic systems and integrate various components of your humanoid robot.