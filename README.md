# README.md

# DroneOps

DroneOps is an advanced drone control and mission planning library designed for autonomous drone operations. It provides tools for creating and managing drone missions, integrating with QGroundControl, and performing intelligent landing capabilities.

## Features

- Connect to drones using the DroneKit library.
- Create and manage mission plans with the MissionPlanner class.
- Convert QGroundControl mission plans to DroneOps format.
- Generate survey grids and circular flight patterns.
- Calculate mission statistics, including total distance and estimated flight time.

## Installation

To install the DroneOps library, you can use pip:

```
pip install droneops
```

## Usage

Here is a simple example of how to use the DroneOps library:

```python
from droneops.droneops import DroneOps
from droneops.mission_planner import MissionPlanner

# Initialize the DroneOps class
drone = DroneOps(connection_string='udp:127.0.0.1:14550')

# Arm and take off to a specified altitude
drone.arm_and_takeoff(altitude=10)

# Create a mission plan
waypoints = [(37.7749, -122.4194, 10), (37.7750, -122.4195, 10)]
mission_file = MissionPlanner.create_mission_file(waypoints)

# Upload the mission to the drone
drone.upload_mission(waypoints)

# Execute the mission
drone.run_route(waypoints)

# Land the drone
drone.land()
```

## Contribution

Contributions are welcome! Please feel free to submit a pull request or open an issue for any bugs or feature requests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.