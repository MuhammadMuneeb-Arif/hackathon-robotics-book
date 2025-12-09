---
title: Module 1 - The Robotic Nervous System (ROS 2)
sidebar_label: Overview
---

# Module 1: The Robotic Nervous System (ROS 2)

## Introduction to ROS 2 for Humanoid Robots

Welcome to Module 1, where we establish the foundational nervous system for your humanoid robot using ROS 2 (Robot Operating System 2). Just as the nervous system coordinates the various parts of the human body, ROS 2 provides the communication and control infrastructure that coordinates the various components of a humanoid robot.

## Module Overview

This module focuses on the middleware that enables distributed robot control, providing a framework for developing robot applications with support for multiple programming languages and platforms. We'll cover the core concepts that form the backbone of all the subsequent modules.

### Focus: Middleware for humanoid robot control

In this module, you'll learn:
- ROS 2 architecture including nodes, topics, services, and actions
- Building robot controllers using rclpy (ROS Client Library for Python)
- URDF (Unified Robot Description Format) for humanoid structure modeling
- Creating ROS 2 packages and launch files

### Module Structure

This module contains three lessons that progressively build your understanding:

1. **Lesson 1**: ROS 2 Architecture - Nodes, Topics, Services, Actions, DDS
2. **Lesson 2**: Python Agents & Controllers - rclpy, Behavior Trees
3. **Lesson 3**: Humanoid URDF - Links, Joints, Sensors

## Learning Objectives

By the end of this module, you will be able to:
- Understand distributed robot control frameworks
- Build ROS 2 nodes for motors, sensors, and behaviors
- Design URDF-based humanoid robot models

## Prerequisites

Before starting this module, ensure you have:
- Basic Python programming skills
- Ubuntu 22.04 LTS installed
- ROS 2 Humble installed and configured
- Basic understanding of robotics concepts (covered in prerequisites section)

## What You'll Build

Throughout this module, you'll incrementally build components of a humanoid robot control system:

- A ROS 2 workspace with custom packages
- Publisher and subscriber nodes for communication
- A basic humanoid robot URDF model
- Controller nodes to move robot joints

## Key Concepts

### Nodes, Topics, Services, and Actions

- **Nodes**: Processes that perform computation - the building blocks of ROS 2 applications
- **Topics**: Named buses for asynchronous message passing using publish/subscribe pattern
- **Services**: Synchronous request/response communication for direct interaction
- **Actions**: Goal-oriented communication with feedback, ideal for long-running tasks

### DDS (Data Distribution Service)

ROS 2 uses DDS as its communication middleware, providing:
- Real-time performance capabilities
- Deterministic behavior for safety-critical applications
- Support for distributed systems
- Quality of Service (QoS) settings for different communication needs

### URDF (Unified Robot Description Format)

URDF provides an XML format to describe robots, including:
- Kinematic structure (links and joints)
- Visual and collision properties
- Inertial properties
- Sensor and actuator mounting points

## Hands-on Exercises

Each lesson in this module includes practical exercises to reinforce concepts:

- **Exercise 1.1**: Create a ROS 2 publisher/subscriber package for basic communication
- **Exercise 1.2**: Create a URDF of a simple humanoid torso with proper joint definitions
- **Exercise 1.3**: Implement a basic controller node to move a robot joint

## Connecting to the Big Picture

The concepts in this module form the foundation for all subsequent modules:
- The communication patterns you learn here will be used in simulation (Module 2)
- The URDF models will be loaded into Gazebo and Isaac Sim (Modules 2 & 3)
- The control architecture will enable the AI and navigation systems (Modules 3 & 4)

## Technical Requirements

For this module, you'll need:
- ROS 2 Humble installed on Ubuntu 22.04
- Basic development tools (Python 3.8+, Git, etc.)
- Adequate system resources (4GB+ RAM recommended)

## Getting Started

Begin with Lesson 1 to understand the fundamental architecture of ROS 2. Each lesson builds upon the previous one, so we recommend following them in sequence. The hands-on exercises are designed to be practical and directly applicable to humanoid robot development.

Let's begin by exploring the ROS 2 architecture that will serve as the nervous system for your humanoid robot!