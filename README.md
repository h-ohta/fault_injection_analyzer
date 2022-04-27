# fault_injection_analyzer

## How to use

### 1. Launch autoware

```bash
# for each terminals
$ ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=[your_vehicle] sensor_model:=[your_sensor] map_path:=[/your/map/path]
$ ros2 launch fault_injection fault_injection.launch.xml log-level:=debug
$ ros2 run rqt_robot_monitor rqt_robot_monitor
```

### 2. In Rviz

Set start position with 2D Pose Estimate and goal position with 2D Nav Goal.

### 3. In rqt_robot_monitor

Confirm all diags are OK

### 4. Launch fault_injection_analyzer

```bash
ros2 run fault_injection_analyzer fault_injection_analyzer
```

After launching it, `analyzed_[GateMode].csv` is generated.

### Remarks

You can change current gate mode with following commands.

```bash
ros2 topic pub /control/gate_mode_cmd tier4_control_msgs/msg/GateMode data:\ 0\
```

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                              | Type                                                | Description                            |
| --------------------------------- | --------------------------------------------------- | -------------------------------------- |
| `/system/emergency/hazard_status` | `autoware_auto_system_msgs/msg/HazardStatusStamped` | hazard status                          |
| `/control/current_gate_mode`      | `tier4_control_msgs/msg/GateMode`                   | current gate mode: Auto(0)/External(1) |

### Output

Output `analyzed_{GateMode}.csv` like following formats

```csv
diagnostics,collecting,hardware_id_0,hardware_id_1
/autoware/control/autonomous_driving/node_alive_monitoring/topic_status,control_topic_status,ad_service_state_monitor,fault_injection
...
```

## Parameters

None.

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

TBD.
