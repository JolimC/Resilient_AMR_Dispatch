# Resilient AMR Dispatch

A ROS 2 Jazzy and MQTT warehouse AMR simulation. The current Phase 4 demo
launches a central dispatcher and eight simulated robots. The dispatcher sends
missions over MQTT, the robots publish their positions and mission states over
ROS 2, and a live Matplotlib window visualizes warehouse activity and injected
disruptions.

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

If you previously built Phase 0 or Phase 1, rebuild once for Phase 2 because the
image now includes the Tk graphical backend:

```bash
docker compose up -d --build
```

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

The default scenario injects `spill_001` after 2.5 seconds. Its delay can be
changed with another ROS launch argument:

```bash
./run_demo.sh hazard_delay:=4.0
```

The demo opens a live warehouse window through WSLg. It shows shelves, staging
and dock areas, robot positions, goals, planned paths, traveled paths, and a
fleet status panel. Each robot also logs the `assigned`, `executing`, and
`completed` mission states. A successful run ends with a dispatcher message
containing:

```json
{"event":"baseline_complete","missions_assigned":8,"missions_completed":8}
```

The visualization includes DVR-style playback controls:

- **Pause** freezes the displayed frame while incoming telemetry continues to
  be recorded.
- **Timeline** can be dragged to inspect an earlier or later recorded frame.
- **Play** resumes playback from the selected frame and returns to live display
  after catching up.

During the demo, a red hatched hazmat spill appears across the active paths of
several robots. The injector publishes it on `/warehouse/hazards` and
`vda5050/fleet/events/hazard`. Affected agents log `active_path_affected`, and
the status panel reports the affected count.

Affected robots run a bounded local A* recovery against the warehouse grid and
known blocked cells. Successful recovery paths bend around the spill and appear
yellow while `recovery_state` is `rerouting`. After clearing the spill, robots
return to blue normal execution and continue to their original goals. Each
recovery publishes an exception report on
`vda5050/fleet/events/exception`. If no route exists, the robot stops in the red
`blocked` state and escalates to dispatch.

Each AMR also consumes peer telemetry. Other robots' current positions and
assigned goals are reserved with a one-cell safety radius during local
planning. If a peer moves into a planned route, the affected AMR replans or
waits until the temporary conflict clears. Completed robots therefore remain
physical obstacles and recovery paths do not pass through another AMR's final
position.

Close the visualization window and press `Ctrl+C` to stop the launched ROS
nodes. The Docker services remain running so the demo can be launched again
without recreating the containers.

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
