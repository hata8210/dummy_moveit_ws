# API Server Startup Guide

This guide explains how to start the Dummy Arm MoveIt 2 HTTP API server.

## Prerequisites

1.  **ROS 2 Humble Environment**: Ensure ROS 2 is installed and sourced.
2.  **MoveIt 2 Core**: The robot's MoveIt 2 configuration and controllers must be running.
3.  **Python Dependencies**:
    *   `fastapi`
    *   `uvicorn`
    *   `pydantic`
    *   `rclpy` (part of ROS 2)

---

## Startup Steps

### 1. Launch the Robot and MoveIt 2
Before starting the API server, you must have the robot's MoveIt 2 environment active. In a new terminal:

```bash
# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the MoveIt 2 demo (Simulated or Real)
ros2 launch dummy_moveit_config demo.launch.py
```

### 2. Start the API Server
Once MoveIt 2 is ready (RViz is open and robot state is loaded), open another terminal:

#### Option A: Using `ros2 run` (Recommended)
If you have built the workspace using `colcon build`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run pymoveit2 api_server.py
```

#### Option B: Direct Python Execution
If you are developing and want to run the script directly:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/dummy_server/server/api_server.py
```

---

## Applying Code Changes (Rebuilding)

If you modify `api_server.py`, the core library in `pymoveit2/`, or any configuration, you must ensure the changes are reflected in the running system.

### If using `ros2 run`:
You must rebuild the package to update the installed files:

```bash
# From the workspace root
colcon build --packages-select pymoveit2

# Source the workspace again
source install/setup.bash

# Restart the API server
ros2 run pymoveit2 api_server.py
```

> **Pro Tip**: Use `colcon build --symlink-install` during your initial build. This creates symbolic links instead of copying files, so changes to Python scripts will take effect immediately upon restarting the process without needing another `colcon build`.

### If using Direct Python Execution:
Simply restart the process (Ctrl+C and run the command again). No build step is required.

---

## Verification

Once started, the server will be available at `http://localhost:8000`. You can verify it is running by checking the status endpoint:

```bash
curl http://localhost:8000/api/status
```

**Expected Response**:
```json
{"status": "ok", "robot_state": "IDLE", "current_joints": [...]}
```

---

## Troubleshooting

*   **MoveIt 2 Not Found**: Ensure you have sourced `install/setup.bash` in the terminal where you start the API server.
*   **Port 8000 Conflict**: If port 8000 is already in use, you can modify the port in `api_server.py` at the bottom of the file:
    ```python
    uvicorn.run(app, host="0.0.0.0", port=8000)
    ```
*   **Joint State Missing**: If the API returns an error about joint states, make sure the robot's state publishers are running (usually part of the `demo.launch.py`).
