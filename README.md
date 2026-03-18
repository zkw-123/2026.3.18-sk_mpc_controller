# sk_mpc_controller

`sk_mpc_controller` is a **stability-index-aware MPC package** for robotic tissue manipulation.

It sits between perception and robot execution:

```text
Perception (target, stability, pose)
        ↓
sk_mpc_controller
        ↓
Robot command
```

The package supports both:

- **legacy 1D MPC pipeline**
- **new 2D lateral MPC pipeline**

No existing node has been removed.

---

## 1. Main Functions

This package is responsible for:

- converting perception outputs into control-relevant errors
- solving MPC problems (1D or 2D)
- generating bounded robot motion commands
- blocking motion under unsafe conditions
- logging important topics for debugging and analysis

---

## 2. Executable Nodes

### Legacy pipeline

- `perception_to_mpc_inputs`  
  computes **scalar** lateral error

- `mpc_node`  
  1D MPC controller

- `mpc_to_robot_command_bridge`  
  converts scalar MPC output into robot commands

### 2D pipeline

- `perception_to_mpc_inputs_2d`
- `mpc_node_2d`
- `mpc_to_robot_command_bridge_2d`

### Utility

- `csv_topic_logger`  
  logs multiple topics into one CSV file

⚠️ Do **not** run the legacy and 2D pipelines at the same time.

---

## 3. Main Topics

### Inputs

- `/perception/target_point` (`geometry_msgs/PointStamped`)
- `/perception/sk` (`std_msgs/Float32`)
- `/robot_current_pose` (`geometry_msgs/PoseStamped`)
- `/ft_sensor` (`geometry_msgs/WrenchStamped`, optional for safety)

### Perception → MPC

#### Legacy
- `/mpc/lateral_error` (`std_msgs/Float64`)
- `/mpc/normal_step` (`std_msgs/Float64`)
- `/mpc/stability_index` (`std_msgs/Float64`)

#### 2D
- `/mpc/lateral_error_vec` (`geometry_msgs/Vector3`)
- `/mpc/lateral_error` (`std_msgs/Float64`, compatibility output)
- `/mpc/normal_step` (`std_msgs/Float64`)
- `/mpc/stability_index` (`std_msgs/Float64`)

For the 2D pipeline, the target error is expressed in **tool frame**:

```text
delta_base = p_target - p_ee
delta_tool = R_ee^T * delta_base
```

### MPC outputs

#### Legacy
- `/mpc/dk_cmd` (`std_msgs/Float64`)
- `/mpc/safety_mode` (`std_msgs/Bool`)

#### 2D
- `/mpc/dxy_cmd` (`geometry_msgs/Vector3`)
- `/mpc/safety_mode` (`std_msgs/Bool`)
- `/mpc/dk_cmd` (`std_msgs/Float64`, compatibility output)

### Robot command

- `/robot_command` (`std_msgs/String`)

Command format depends on the bridge:

- legacy bridge: typically `move X Y Z`
- current uploaded 2D bridge: `down X Y Z 0`

---

## 4. Control Models

### Legacy 1D MPC

```text
x_k = e_lat
u_k = Δk
x_{k+1} = x_k - u_k
```

### 2D lateral MPC

```text
x_k = [ e_x , e_y ]^T
u_k = [ Δx , Δy ]^T
x_{k+1} = x_k - u_k
```

Cost:

```text
Σ ( x_k^T Q x_k + λ(S_k) u_k^T R u_k ) + x_N^T Q_f x_N
```

As stability decreases:

- motion becomes more conservative
- admissible control step becomes smaller

---

## 5. Safety Logic

The controller publishes:

- `/mpc/safety_mode` (`std_msgs/Bool`)

Safety mode is triggered when:

- stability falls below a threshold
- force exceeds a threshold
- torque exceeds a threshold

When safety mode is active:

- MPC output is suppressed
- robot motion is blocked by the bridge

⚠️ In the current implementation, `safety_mode = true` means **block motion**.

---

## 6. CSV Logging

The package also provides:

- `csv_topic_logger`

This node records multiple ROS2 topics into a single CSV file for:

- debugging
- offline analysis
- plotting
- experiment replay

Typical logged topics include:

- `/mpc/lateral_error_vec`
- `/mpc/lateral_error`
- `/robot_command`
- `/robot_current_pose`
- `/perception/sk`
- `/perception/target_point`
- `/mpc/dk_cmd`
- `/mpc/dxy_cmd`

---

## 7. Build

```bash
colcon build --packages-select sk_mpc_controller
source install/setup.bash
```

---

## 8. Run

### Recommended: launch the 2D pipeline

```bash
ros2 launch sk_mpc_controller mpc_three_nodes.launch.py
```

This launch file starts:

- `csv_topic_logger`
- `perception_to_mpc_inputs_2d`
- `mpc_node_2d`
- `mpc_to_robot_command_bridge_2d`

### Run 2D nodes manually

```bash
ros2 run sk_mpc_controller perception_to_mpc_inputs_2d
ros2 run sk_mpc_controller mpc_node_2d
ros2 run sk_mpc_controller mpc_to_robot_command_bridge_2d
```

### Run legacy nodes manually

```bash
ros2 run sk_mpc_controller perception_to_mpc_inputs
ros2 run sk_mpc_controller mpc_node
ros2 run sk_mpc_controller mpc_to_robot_command_bridge
```

---

## 9. Important Notes

- legacy nodes are **not removed**
- only one pipeline should run at a time
- target and robot pose should share the same frame
- units should be meters
- the 2D bridge currently locks z motion
- the 2D bridge publishes `down X Y Z 0`
- safety mode acts as a motion-blocking signal

---

## 10. Design Intent

- **legacy MPC**: backward compatibility
- **2D MPC**: physically meaningful lateral control
- **tool-frame error formulation**: geometrically meaningful control state
- **stability-aware weighting**: safer behavior under poor imaging
- **safety gating**: prevent motion under unreliable or unsafe conditions
- **CSV logging**: support debugging, replay, and evaluation

---

## 11. Summary

`sk_mpc_controller` is a perception-to-control bridge for robotic tissue manipulation.

It converts perception outputs into control errors, solves either legacy 1D or new 2D MPC problems, applies safety gating, generates robot command strings, and logs the closed-loop behavior for analysis.

In short, this package implements:

```text
Perception -> MPC -> Robot Command -> Logging
```