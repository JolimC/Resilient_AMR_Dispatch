# Resilient AMR Dispatch

A ROS 2 Jazzy and MQTT warehouse AMR simulation. The current Phase 1 demo
launches a central dispatcher and eight simulated robots. The dispatcher sends
missions over MQTT, while the robots publish their positions and mission states
over ROS 2.

## Prerequisites

- Docker Desktop running with the WSL 2 engine enabled
- Docker Desktop integration enabled for the WSL distribution
- A WSL terminal with `docker` and `docker compose` available

Verify the environment with:

```bash
docker version
docker compose version
```

## First-Time Setup

Before running the demo for the first time, build the ROS 2 image and start the
Compose services:

```bash
docker compose up -d --build
docker compose ps
```

This constructs the custom ROS 2 image, starts both the `mqtt` and `ros2`
containers, and builds the mounted ROS workspace. Both services should report
as running, with the MQTT service reporting as healthy.

You only need `--build` again after changing the `Dockerfile` or its system
dependencies.

## Run the Demo

From this repository directory in WSL, run:

```bash
./run_demo.sh
```

The script performs the complete development run sequence:

1. Starts the Mosquitto and ROS 2 Compose services if they are not running.
2. Builds the mounted ROS 2 workspace with `colcon build --symlink-install`.
3. Sources the built workspace.
4. Launches the centralized dispatch demo.

The default scenario starts eight AMRs. To select any supported count from 6
through 12, pass a ROS launch argument:

```bash
./run_demo.sh robot_count:=6
```

Phase 1 is terminal-based and does not open a graphical window. Each robot logs
the `assigned`, `executing`, and `completed` mission states. A successful run
ends with a dispatcher message containing:

```json
{"event":"baseline_complete","missions_assigned":8,"missions_completed":8}
```

Press `Ctrl+C` to stop the launched ROS nodes. The Docker services remain
running so the demo can be launched again without recreating the containers.

## Stop the Environment

When finished working, stop and remove the Compose containers and network:

```bash
docker compose down
```

The project source remains on the host and is not removed.

## Manual Run Sequence

The equivalent commands, useful for debugging, are:

```bash
docker compose up -d
docker compose exec ros2 bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch resilient_amr_dispatch demo.launch.py
```

Type `exit` to leave the container shell. Rebuild the Docker image only after
changing the `Dockerfile` or its system dependencies.
