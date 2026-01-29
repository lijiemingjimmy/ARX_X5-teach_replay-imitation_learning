# Copilot Instructions for ARX X5 Teach/Replay Imitation Learning

## Project Overview

This repository contains an end-to-end pipeline for ARX X5 robotic arm:
- Environment setup
- CAN/ROS2 bring-up
- Teleoperation & teach/replay functionality
- Data collection
- Simulation training practice

## Development Guidelines

### ROS2 Specific Guidelines
- Follow ROS2 naming conventions (snake_case for nodes, topics, services)
- Use appropriate ROS2 message types and interfaces
- Ensure proper node lifecycle management
- Follow the ROS2 rclpy/rclcpp patterns for node implementation

### CAN Communication
- Handle CAN bus communication with proper error checking
- Ensure thread-safe CAN message handling
- Follow the ARX X5 CAN protocol specifications

### Code Style
- Use clear, descriptive variable and function names
- Add comments for complex robotics algorithms or control logic
- Document hardware-specific configurations
- Include safety checks for motor control and hardware interfaces

### Testing
- Test hardware interactions with appropriate mocking when hardware is unavailable
- Validate data collection pipelines before training
- Ensure teach/replay sequences are properly synchronized

### Safety Considerations
- Always include safety limits for motor commands
- Implement emergency stop functionality
- Validate input ranges for joint positions and velocities
- Add timeouts for hardware communication

### Documentation
- Document hardware setup requirements
- Provide clear instructions for environment configuration
- Include examples for common use cases (teach, replay, data collection)
- Maintain up-to-date installation and dependency information

## Common Tasks

### Adding New ROS2 Nodes
- Create proper package structure
- Define interfaces in separate package if needed
- Use launch files for multi-node systems

### Data Collection
- Ensure consistent data formats
- Include proper timestamps
- Validate data integrity during collection

### Simulation
- Clearly separate simulation and real robot code paths
- Use appropriate physics parameters for simulation
- Ensure simulation environment matches real hardware constraints
