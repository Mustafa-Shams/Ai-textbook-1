---
sidebar_position: 5
title: "URDF Structures for Humanoid Joints/Links"
---

# URDF Structures for Humanoid Joints/Links

URDF (Unified Robot Description Format) is a fundamental component in robotics that enables the precise description of robot models in ROS. For humanoid robots, URDF serves as the blueprint that defines the physical structure, kinematic relationships, and visual representation of the robot. This format is essential for both simulation and real-world robot control, as it provides the necessary information for motion planning, collision detection, and visualization.

URDF files are XML-based documents that describe the complete kinematic and geometric structure of a robot. They define the physical properties, visual appearance, and collision geometry of each component. For humanoid robots with their complex multi-link structures and numerous degrees of freedom, URDF becomes even more critical as it must accurately represent the intricate relationships between all body parts.

## Understanding URDF

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and their relationships. For humanoid robots, URDF is particularly important as it captures the complex kinematic chains that enable human-like movement patterns.

A URDF file consists of two main elements:
- **Links**: Represent rigid bodies with associated properties like mass, inertia, visual geometry, and collision geometry
- **Joints**: Define the connections between links, specifying the type of movement allowed and constraints

The kinematic chain in URDF is represented as a tree structure where each joint connects a parent link to a child link. This hierarchical structure allows for complex robot morphologies while maintaining computational efficiency for kinematic calculations.

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

## Advanced URDF for Humanoid Robots

Humanoid robots require much more complex URDF descriptions due to their numerous joints and links. A typical humanoid URDF might include:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.1 0 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </joint>

  <!-- Additional links and joints for complete humanoid structure -->
  <!-- Legs, additional arm segments, hands, feet, etc. -->
</robot>
```

## Humanoid Robot URDF Example

A humanoid robot typically includes:

- **Links**: Represent rigid bodies (head, torso, arms, legs, hands, feet)
- **Joints**: Define how links connect (revolute, prismatic, fixed, continuous)
- **Materials**: Define visual appearance and colors
- **Gazebo plugins**: For simulation integration and physics properties
- **Transmission elements**: For actuator control
- **Inertial properties**: For physics simulation

## Key Components for Humanoid Joints

1. **Revolute joints**: For rotational movement (knees, elbows, shoulders)
2. **Fixed joints**: For permanent connections (sensor mounting, rigid attachments)
3. **Continuous joints**: For unlimited rotation (waist, neck)
4. **Prismatic joints**: For linear movement (if needed for specific mechanisms)

## URDF Best Practices for Humanoid Robots

When creating URDF files for humanoid robots, several best practices should be followed:

- **Accurate inertial properties**: Properly calculated mass, center of mass, and inertia tensors are crucial for realistic physics simulation
- **Collision geometry**: Use simplified geometries for collision detection to maintain simulation performance
- **Joint limits**: Define appropriate position, velocity, and effort limits based on physical constraints
- **Visual and collision separation**: Use detailed visual meshes for rendering and simplified collision meshes for physics
- **Kinematic chain consistency**: Ensure the joint hierarchy is logically organized and kinematically valid

## Xacro for Complex URDFs

For complex humanoid robots, Xacro (XML Macros) is often used to simplify URDF creation:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Define a macro for creating a simple link -->
  <xacro:macro name="simple_link" params="name mass x y z">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="${x} ${y} ${z}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="${x} ${y} ${z}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create links -->
  <xacro:simple_link name="base_link" mass="1.0" x="0.1" y="0.1" z="0.1"/>
</robot>
```

## Validation and Testing

URDF files should be validated using ROS tools like `check_urdf` to ensure they are properly formatted and kinematically valid. The robot state publisher can be used to visualize the robot in RViz, and Gazebo can be used to test the physics simulation.

The URDF description is critical for humanoid robots as it enables accurate forward and inverse kinematics calculations, which are essential for complex movements like walking, reaching, and manipulation tasks. Proper URDF configuration is fundamental to the success of any humanoid robot project.

## Next Steps

Continue to Module 2: The Digital Twin to learn about simulation environments.