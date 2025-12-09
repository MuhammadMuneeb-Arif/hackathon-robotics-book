# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview
This guide provides a quick start for contributors and users of the Physical AI & Humanoid Robotics Textbook project. This textbook covers ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems for humanoid robotics education.

## Prerequisites

### System Requirements
- Node.js 20+ with npm
- Python 3.8+
- Git
- Basic familiarity with robotics concepts

### Software Dependencies
- ROS 2 Humble Hawksbill (for development and testing)
- Gazebo simulation environment
- NVIDIA Isaac Sim (optional, for advanced content)
- Docusaurus-compatible environment

## Setting Up the Development Environment

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/hackathon-robotics-book.git
cd hackathon-robotics-book
```

### 2. Navigate to the Docusaurus Directory
The textbook is built using Docusaurus in the `my-website` directory:

```bash
cd my-website
```

### 3. Install Docusaurus Dependencies
```bash
npm install
```

### 4. Start the Development Server
```bash
npm start
```
This command starts a local development server and opens the documentation in your browser. Most changes are reflected live without restarting the server.

## Textbook Structure

The textbook is organized into 4 main modules:

1. **Module 1 — Robotic Nervous System (ROS 2)**
   - ROS 2 Architecture: Nodes, Topics, Services, Actions, DDS
   - Python Agents & Controllers — rclpy, Behavior Trees
   - Humanoid URDF — Links, Joints, Sensors

2. **Module 2 — Digital Twin (Gazebo & Unity)**
   - Gazebo Physics Engine & Sensors
   - Unity Robotics Hub for HRI
   - Simulated Cameras, RGB-D, and Sensor Fusion

3. **Module 3 — AI-Robot Brain (NVIDIA Isaac)**
   - Isaac Sim Basics — RTX, Articulations, USD
   - Isaac ROS: Perception & VSLAM
   - Nav2: Path Planning for Humanoids

4. **Module 4 — Vision-Language-Action (VLA)**
   - Whisper Speech-to-Command
   - LLM Cognitive Planning → Action Graphs
   - Capstone: Full Digital Humanoid Agent

## Contributing Content

### Adding a New Lesson
1. Navigate to the appropriate module directory in `/docs/`
2. Create a new Markdown file with proper frontmatter
3. Follow the lesson template structure

### Example Lesson Structure
```markdown
---
title: Lesson Title
sidebar_label: Lesson Title
description: Brief description of the lesson
---

import DocCardList from '@theme/DocCardList';

# Lesson Title

## Learning Objectives
- Objective 1
- Objective 2
- Objective 3

## Content
[Main lesson content goes here - 1000-2000 words]

## Hands-on Exercise
[Practical exercise for students]

## Key Takeaways
- Key point 1
- Key point 2
- Key point 3

## Reflection Questions
1. Question 1?
2. Question 2?

## References
[APA-formatted citations]
```

### Adding Code Examples
When adding code examples, use Docusaurus code blocks with appropriate language tags:

```python
# Python example for ROS 2
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

### Adding Citations
All technical claims must be supported by credible sources. Use APA 7th edition format:

> According to Smith et al. (2023), ROS 2 provides significant improvements in real-time performance compared to ROS 1.

**Reference:**
Smith, J., Johnson, M., & Williams, K. (2023). Real-time performance in ROS 2. *Journal of Robotics*, 15(3), 45-62. https://doi.org/10.1234/example.doi

## Building the Textbook

### Local Build
To build the static HTML files for the textbook:

```bash
npm run build
```

### Serving the Build Locally
To serve the built files locally for review:

```bash
npm run serve
```

### Deployment
The textbook is designed to be deployed to GitHub Pages. The build artifacts will be in the `build` directory.

## Quality Standards

### Academic Requirements
- Maintain Flesch-Kincaid grade level 10-12 readability
- Include minimum 50% peer-reviewed citations
- Ensure 0% plagiarism
- Follow APA 7th edition citation format

### Technical Requirements
- All code examples must be tested and verified
- Hands-on exercises must be reproducible
- All claims must be traceable to credible sources
- Content must align with the project constitution

## Simulation Environment Setup

### ROS 2 Environment
For lessons involving ROS 2, ensure your environment is properly configured:

```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash  # If you have a workspace built

# Verify ROS 2 is working
ros2 topic list
```

### Gazebo Simulation
For simulation-based lessons:

```bash
# Launch Gazebo
gz sim -r shapes.sdf
```

## Getting Help

### Documentation
- Docusaurus documentation: https://docusaurus.io/
- ROS 2 documentation: https://docs.ros.org/
- Isaac Sim documentation: https://docs.omniverse.nvidia.com/

### Community
- Join our development Discord/Slack
- Open issues for bugs or suggestions
- Submit pull requests for content contributions

## Next Steps

1. Review the [project constitution](../.specify/memory/constitution.md) for detailed principles
2. Check the [feature specification](../spec.md) for detailed requirements
3. Look at existing lessons as examples
4. Start contributing content following the established patterns

## Troubleshooting

### Common Issues
- **Build errors**: Ensure all dependencies are installed and Node.js version is correct
- **Links not working**: Check that all internal links use Docusaurus link syntax
- **Images not displaying**: Verify image paths are correct relative to the document

### Getting Support
If you encounter issues not covered in this guide, please open an issue in the repository with:
- Detailed description of the problem
- Steps to reproduce
- Expected vs. actual behavior
- Environment information (OS, Node.js version, etc.)