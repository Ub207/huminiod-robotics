---
sidebar_position: 1
---

# Module 1: The Robotic Nervous System (ROS 2)

## Purpose

This module introduces **ROS 2 (Robotic Operating System)** as the central nervous system of a humanoid robot, responsible for communication, coordination, sensing, and motor commands.

## Learning Outcomes

After completing this module, students will be able to:
- Understand ROS 2 architecture and its role in humanoid robotics
- Build nodes and communication systems
- Create a complete neural messaging system for a humanoid robot
- Connect AI agents to robotic sensors and actuators

## Core Concepts

- **Nodes**: Individual processes that communicate with each other
- **Topics**: Publish/subscribe communication channels
- **Publishers/Subscribers**: Unidirectional communication pattern
- **Services**: Request/response communication pattern
- **Actions**: Goal-oriented communication with feedback
- **URDF (Unified Robot Description Format)**: Robot modeling and description

## Module Overview

ROS 2 serves as the communication backbone for humanoid robots, enabling different software components to coordinate their activities. Think of it as the nervous system that allows the "brain" (AI agent) to communicate with the "body" (sensors and actuators).

In this module, we'll build a complete ROS 2 ecosystem for a humanoid robot, connecting various subsystems:
- Sensory systems (cameras, IMUs, force sensors)
- Motor control systems (joint controllers, actuators)
- AI decision-making systems
- Simulation interfaces

## Prerequisites

Before starting this module, ensure you have:
1. ROS 2 Humble Hawksbill installed
2. Basic Python programming skills
3. Understanding of robotics concepts (optional but helpful)

## Module Structure

1. [ROS 2 Basics](./ros-basics.md) - Core concepts and architecture
2. [Nodes and Topics](./nodes-topics.md) - Publish/subscribe communication
3. [Services and Actions](./services-actions.md) - Request/response and goal-oriented communication
4. [URDF Modeling](./urdf.md) - Robot description and modeling
5. [AI-ROS Integration](./ai-ros-integration.md) - Connecting AI agents to ROS
6. [Module 1 Project](./project.md) - Building a complete humanoid nervous system

Let's begin by exploring the fundamentals of ROS 2.