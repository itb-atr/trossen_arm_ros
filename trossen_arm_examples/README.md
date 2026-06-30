# Trossen Arm Examples

This package contains small ROS 2 examples for the Cartesian controllers added to
`trossen_arm_controllers`.

## Start the Cartesian controller manager

Use this launch file when you want a standalone Cartesian controller setup for
the examples:

```bash
ros2 launch trossen_arm_examples cartesian_control.launch.py
```

It starts `controller_manager/ros2_control_node` and spawns:

- `joint_state_broadcaster`
- `cartesian_position_controller`
- `cartesian_external_effort_controller`

Only `joint_state_broadcaster` is activated at startup. The two Cartesian
controllers are spawned inactive because they claim mutually exclusive command
interfaces in the hardware interface.

The launch arguments mirror the upstream Trossen bringup defaults:

```bash
ros2 launch trossen_arm_examples cartesian_control.launch.py \
  robot_model:=wxai \
  arm_variant:=base \
  arm_side:=none \
  ip_address:=192.168.1.2 \
  ros2_control_hardware_type:=real
```

For a non-hardware smoke test:

```bash
ros2 launch trossen_arm_examples cartesian_control.launch.py \
  ros2_control_hardware_type:=mock_components
```

## Run the position example with bringup

```bash
ros2 launch trossen_arm_examples cartesian_position_demo.launch.py
```

The demo first moves the tool point `ee_gripper_link` to a known start pose,
then sends five absolute base-frame Cartesian targets. Each target adds 2 cm in
base-frame +X on top of the start pose.

Default sequence:

```text
1. Move to [0.25, 0.0, 0.35, 0.0, 0.0, 0.0]
2. Print "Reached start position"
3. Publish [0.27, 0.0, 0.35, 0.0, 0.0, 0.0]
4. Publish [0.29, 0.0, 0.35, 0.0, 0.0, 0.0]
5. Publish [0.31, 0.0, 0.35, 0.0, 0.0, 0.0]
6. Publish [0.33, 0.0, 0.35, 0.0, 0.0, 0.0]
7. Publish [0.35, 0.0, 0.35, 0.0, 0.0, 0.0]
8. Wait 5 seconds
9. Move back to [0.25, 0.0, 0.35, 0.0, 0.0, 0.0]
```

Override the start pose or base-frame step:

```bash
ros2 launch trossen_arm_examples cartesian_position_demo.launch.py \
  start_pose:="[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]" \
  base_frame_step:="[0.02, 0.0, 0.0, 0.0, 0.0, 0.0]" \
  step_count:=5
```

The controller command topic now uses `geometry_msgs/msg/PoseStamped`. The example still keeps the launch/config pose as a compact six-value list:

```text
[x, y, z, roll, pitch, yaw]
```

The example converts that list to `PoseStamped` before publishing.

## Run the button press example with bringup

```bash
ros2 launch trossen_arm_examples cartesian_button_press_demo.launch.py
```

The external-effort demo starts from the same clear pose as the position demo,
then captures that Cartesian hold pose. While the external-effort controller is
active, it publishes an impedance-stabilized wrench:

```text
effort = stiffness * (hold_pose - current_pose)
       - damping * current_velocity
       + ramped_press_force
```

This mirrors the native careful press demo and keeps the end effector from
collapsing while applying the press force. It stops the press when either the
Cartesian state reports 0.1 m of travel along the press direction, or when the
10 second duration limit is reached. After zeroing the wrench, it switches back
to the position controller and publishes interpolated position targets to slowly
return to the start pose.

Default sequence:

```text
1. Move to [0.25, 0.0, 0.35, 0.0, 0.0, 0.0]
2. Switch to cartesian_external_effort_controller
3. Publish impedance-stabilized external efforts at 100 Hz
4. Stop at 0.1 m travel or after 10 seconds
5. Publish zero wrench several times
6. Switch to cartesian_position_controller
7. Return to [0.25, 0.0, 0.35, 0.0, 0.0, 0.0] over 5 seconds
```

Override the press force, direction, or travel limit:

```bash
ros2 launch trossen_arm_examples cartesian_button_press_demo.launch.py \
  press_direction:="[1.0, 0.0, 0.0]" \
  press_force_n:=4.0 \
  max_press_travel_m:=0.1 \
  press_duration_sec:=10.0
```

The external-effort controller command topic now uses `geometry_msgs/msg/WrenchStamped`. The example computes the six effort values and publishes them as:

```text
wrench.force:  [fx, fy, fz]
wrench.torque: [tx, ty, tz]
```

The direction and gain parameter formats remain:

```text
press_direction: [x, y, z]
torque_xyz_nm:  [tx, ty, tz]
stiffness:      [kx, ky, kz, krx, kry, krz]
damping:        [dx, dy, dz, drx, dry, drz]
```

Use conservative force values first and verify the tool frame direction before
testing against a real button or touch screen.

The impedance loop depends on `joint_state_broadcaster` publishing Cartesian
position and velocity on `/dynamic_joint_states` under the `cartesian` state
name. If that state is unavailable, the demo zeros the wrench, switches back to
position control, attempts the return, and reports the error.

The button press demo switches controllers in this order:

```text
cartesian_position_controller active
cartesian_external_effort_controller active
cartesian_position_controller active
```

That sequence is required because the hardware interface accepts only one
Cartesian command mode at a time.
