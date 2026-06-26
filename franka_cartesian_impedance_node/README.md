# franka_cartesian_impedance_node

Direct-libfranka Cartesian controllers for the Franka FR3 (`cartesian_impedance_node`,
`admittance_node`, and the experimental `cartesian_impedance_torque_node`), launch files,
config, and scripts. Depends on the interface package `franka_cartesian_impedance_msgs`
(`srv/GoToPose.srv`).

See the **repo root [`README.md`](../README.md)** for the full documentation: architecture,
the Franka robot data used, the ROS topic/service interface, parameters, gripper, build, and
run instructions.

Build (robot side, resolves the interface-package dependency automatically):
```bash
colcon build --packages-up-to franka_cartesian_impedance_node
```
