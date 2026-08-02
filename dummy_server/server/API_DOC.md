# Dummy Arm MoveIt 2 HTTP API Documentation

This API provides a RESTful interface to control the Dummy robot arm via MoveIt 2. It allows external systems (web dashboards, mobile apps, or other servers) to command the arm without requiring a local ROS 2 environment.

**Base URL**: `http://<ROBOT_IP>:8000`

---

## 1. Endpoints Overview

| Method | Endpoint | Description |
| :--- | :--- | :--- |
| **GET** | `/api/status` | Get current robot execution state and joint positions. |
| **GET** | `/api/pose` | Get the current Cartesian pose (X, Y, Z, Quat) of the end-effector. |
| **POST** | `/api/move_to_pose` | Standard MoveIt planning to a Cartesian (X, Y, Z) goal. |
| **POST** | `/api/move_optimal` | Optimized planning using current/custom joints as a seed for the nearest IK solution. |

---

## 2. API Details

### GET `/api/status`
Returns the current state of the MoveIt interface and the real-time joint positions (in radians).

**Response Example**:
```json
{
  "status": "ok",
  "robot_state": "IDLE", 
  "current_joints": [0.0, -1.30, 1.57, 0.0, 0.0, 0.0]
}
```
*Note: `robot_state` can be `IDLE`, `REQUESTING`, or `EXECUTING`.*

---

### GET `/api/pose`
Computes forward kinematics to return the current Cartesian pose of the end-effector.

**Response Example**:
```json
{
  "status": "ok",
  "frame_id": "dummy_base_link",
  "position": {
    "x": 0.25,
    "y": 0.0,
    "z": 0.20
  },
  "orientation": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "w": 1.0
  }
}
```

---

### POST `/api/move_to_pose`
Standard Cartesian planning. MoveIt will find a path to the target pose.

**Request Body**:
```json
{
  "x": 0.25,
  "y": 0.0,
  "z": 0.3,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.0,
  "qw": 1.0,
  "cartesian": false
}
```

---

### POST `/api/move_optimal`
Finds the joint configuration nearest to the `start_joints` (or current real joints) that satisfies the target pose, then executes a joint-space trajectory. **Recommended for minimizing unnecessary movement.**

**Request Body**:
```json
{
  "x": 0.20,
  "y": -0.1,
  "z": 0.25,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.0,
  "qw": 1.0,
  "start_joints": [0.0, -1.0, 1.2, 0.0, 0.5, 0.0] 
}
```
*`start_joints` is optional. If omitted, the server uses the robot's actual current joints.*

---

## 3. Usage Demos

### Using `curl` (Command Line)
**Move to a point using optimal planning:**
```bash
curl -X POST "http://localhost:8000/api/move_optimal" \
     -H "Content-Type: application/json" \
     -d '{
           "x": 0.22,
           "y": 0.05,
           "z": 0.3,
           "qw": 1.0
         }'
```

---

### Using Python (`requests`)
```python
import requests
import time

BASE_URL = "http://192.168.1.100:8000" # Replace with your robot IP

def move_robot(x, y, z):
    payload = {
        "x": x, "y": y, "z": z,
        "qw": 1.0
    }
    response = requests.post(f"{BASE_URL}/api/move_optimal", json=payload)
    return response.json()

def wait_for_idle():
    while True:
        status = requests.get(f"{BASE_URL}/api/status").json()
        if status["robot_state"] == "IDLE":
            print("Robot is ready.")
            break
        print("Robot is moving...")
        time.sleep(0.5)

# Example Workflow
if __name__ == "__main__":
    print(move_robot(0.2, 0.0, 0.25))
    wait_for_idle()
    print("Task Complete!")
```

---

## 4. Development Notes
- **Coordinate System**: Values are in Meters (m) for position and Radians (rad) for joints.
- **Orientation**: Default orientation `[0,0,0,1]` (qx,qy,qz,qw) typically points the end-effector downwards for the Dummy arm configuration.
- **Safety**: The API will reject new goals with a `400` error if the robot is currently `EXECUTING`.
