# Walkie-Isaac: Standalone Isaac Sim Package for Walkie Bot

This package contains the Isaac Sim simulation assets, configuration, and automated runner script for **Walkie Bot** for **RoboCup 2027**.

## Directory Layout

```
walkie-isaac/
├── config/
│   └── sim_config.yaml              # Topic names, LiDAR specs, physics rates
├── assets/
│   └── Collected_gz_walkie_roller/  # Robot USD model, meshes, and physics payloads
├── launch_sim.py                    # Standalone Isaac Sim Python runner
└── README.md
```

## Features

- **Automated ROS 2 Action Graph**: Programmatically sets up `/clock`, `/cmd_vel`, and `/scan` at runtime without touching the Omniverse GUI.
- **Clean Stage Referencing**: Never opens raw converted stages directly; creates a clean stage and references the robot prim (`/World/walkie_bot`) to prevent viewport metadata corruption.
- **Config-Driven**: All parameters (topics, LiDAR specs, physics step) are defined in `config/sim_config.yaml`.
- **Environment Isolation**: Isaac Sim executes with its own Python runtime while cleanly interfacing with ROS 2 over DDS.

## Usage

### Run via Pixi (from workspace root)
```bash
pixi run sim
```

### Run standalone with Isaac Sim Python
```bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/kay/IsaacLab/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/humble/lib

python3 launch_sim.py [--headless] [--test] [--steps 120]
```
