# Data Model: Physical AI & Humanoid Robotics Documentation

## Overview
This document defines the conceptual data model for the Physical AI & Humanoid Robotics documentation site. Since this is primarily a content-based documentation site, the "data model" refers to the structure and relationships of content modules and their components.

## Core Entities

### Learning Module
- **Properties**:
  - moduleId: Unique identifier for the module
  - title: Display title of the module
  - description: Brief description of the module
  - position: Order in the curriculum sequence
  - prerequisites: List of prerequisite modules or concepts
  - learningObjectives: List of learning objectives
  - contentPages: List of pages within the module
  - duration: Estimated time to complete the module

### Documentation Page
- **Properties**:
  - pageId: Unique identifier for the page
  - title: Page title
  - content: Markdown content of the page
  - module: Reference to the parent module
  - sidebarPosition: Position in the sidebar navigation
  - next: Reference to the next page in sequence
  - previous: Reference to the previous page in sequence
  - tags: List of tags for search and categorization
  - difficulty: Difficulty level (beginner, intermediate, advanced)

### Code Example
- **Properties**:
  - exampleId: Unique identifier for the example
  - title: Brief description of the example
  - language: Programming language (python, xml, etc.)
  - code: The actual code content
  - explanation: Text explaining the code
  - associatedModule: Reference to the module it belongs to
  - useCase: Description of when/why this code is used

### Technical Diagram (Mermaid)
- **Properties**:
  - diagramId: Unique identifier for the diagram
  - title: Brief description of the diagram
  - type: Type of diagram (flowchart, sequence, state, etc.)
  - code: Mermaid code defining the diagram
  - description: Explanation of the diagram
  - associatedModule: Reference to the module it belongs to

### Navigation Item
- **Properties**:
  - navId: Unique identifier for the navigation item
  - title: Display text for the navigation
  - path: URL path to the page
  - parent: Reference to parent navigation item (for hierarchy)
  - children: List of child navigation items
  - position: Order in the navigation list

## Content Module Definitions

### Introduction Module
- **moduleId**: "intro"
- **title**: "Introduction: The Shift from Digital Brain to Physical Body"
- **Description**: Overview of embodied intelligence concepts
- **Learning Objectives**:
  - Understand the concept of embodied intelligence
  - Recognize the importance of physical interaction in AI
  - Identify key challenges in bridging digital and physical systems

### Module 1: The Robotic Nervous System (ROS 2)
- **moduleId**: "module-1"
- **title**: "The Robotic Nervous System (ROS 2)"
- **Description**: Core concepts of ROS 2 including nodes, topics, and services
- **Learning Objectives**:
  - Implement basic ROS 2 nodes using rclpy
  - Design message passing systems using topics
  - Create service-based communication
  - Understand URDF structures for humanoid joints and links

### Module 2: The Digital Twin
- **moduleId**: "module-2"
- **title**: "The Digital Twin"
- **Description**: Simulation tools and sensor modeling
- **Learning Objectives**:
  - Configure Gazebo for physics simulation
  - Set up Unity for rendering (where applicable)
  - Simulate LiDAR and depth cameras
  - Connect simulation to real-world robot behaviors

### Module 3: The AI-Robot Brain
- **moduleId**: "module-3"
- **title**: "The AI-Robot Brain"
- **Description**: Advanced AI integration with robotics
- **Learning Objectives**:
  - Implement NVIDIA Isaac Sim and Isaac ROS
  - Configure Nav2 stack for bipedal movement
  - Integrate AI decision-making with robot control

### Module 4: Vision-Language-Action (VLA)
- **moduleId**: "module-4"
- **title**: "Vision-Language-Action (VLA)"
- **Description**: Integration of perception, language, and action
- **Learning Objectives**:
  - Integrate OpenAI Whisper for speech processing
  - Connect LLMs to robot actions
  - Implement ROS 2 actions for complex behaviors

### Capstone Project: The Autonomous Humanoid
- **moduleId**: "capstone"
- **title**: "Capstone Project: The Autonomous Humanoid"
- **Description**: Comprehensive integration of all concepts
- **Learning Objectives**:
  - Synthesize all previous module concepts
  - Implement a complete autonomous humanoid system
  - Demonstrate embodied intelligence principles

## Relationships

### Module-Page Relationship
- One Learning Module contains many Documentation Pages
- Each Documentation Page belongs to exactly one Learning Module

### Module-Code Example Relationship
- One Learning Module may contain many Code Examples
- Each Code Example belongs to one or more Learning Modules

### Module-Diagram Relationship
- One Learning Module may contain many Technical Diagrams
- Each Technical Diagram belongs to one Learning Module

### Navigation Hierarchy
- Navigation Items form a tree structure
- Each Navigation Item may have zero or more children
- Each Navigation Item (except the root) has exactly one parent

## Content Attributes

### Common Page Attributes
- **title**: Required string, max 100 characters
- **description**: Required string, max 300 characters
- **sidebar_position**: Required integer for ordering
- **tags**: Optional array of strings for categorization
- **authors**: Optional array of author names
- **date**: Optional publication date
- **draft**: Boolean indicating if page is ready for publication

### Code Example Attributes
- **language**: Required string (python, xml, cpp, bash, etc.)
- **title**: Required string describing the example
- **line_numbers**: Boolean for showing line numbers
- **highlight**: Optional string for highlighting specific lines
- **filename**: Optional string for showing as filename header

### Diagram Attributes
- **type**: Required string (graph, flowchart, sequence, gantt, etc.)
- **theme**: Optional string for styling (default, dark, neutral)
- **title**: Optional string for diagram title
- **width**: Optional integer for diagram width