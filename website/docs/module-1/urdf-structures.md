---
sidebar_position: 5
title: "URDF Structures for Humanoid Joints/Links"
---

# URDF Structures for Humanoid Joints/Links

URDF (Unified Robot Description Format) is a fundamental component in robotics that enables the precise description of robot models in ROS. For humanoid robots, URDF serves as the blueprint that defines the physical structure, kinematic relationships, and visual representation of the robot. This format is essential for both simulation and real-world robot control, as it provides the necessary information for motion planning, collision detection, and visualization.

## Understanding URDF

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and their relationships. For humanoid robots, URDF is particularly important as it captures the complex kinematic chains that enable human-like movement patterns.

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
  </link>

  <link name="upper_leg">
    <visual>
      <geometry>
        <box size="0.1 0.5 0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="hip" type="revolute">
    <parent link="base_link"/>
    <child link="upper_leg"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Humanoid Robot URDF Example

A humanoid robot typically includes:

- **Links**: Represent rigid bodies (head, torso, arms, legs)
- **Joints**: Define how links connect (revolute, prismatic, fixed)
- **Materials**: Define visual appearance
- **Gazebo plugins**: For simulation integration

## Key Components for Humanoid Joints

1. **Revolute joints**: For rotational movement (knees, elbows)
2. **Fixed joints**: For permanent connections
3. **Continuous joints**: For unlimited rotation (waist)
4. **Prismatic joints**: For linear movement (if needed)

The URDF description is critical for humanoid robots as it enables accurate forward and inverse kinematics calculations, which are essential for complex movements like walking, reaching, and manipulation tasks.

## Next Steps

Continue to Module 2: The Digital Twin to learn about simulation environments.