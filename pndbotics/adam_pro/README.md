# PNDbotics Adam Pro driver

Directly uses the official `pnd_adam` ROS2 contract. It bridges LowState and ZED images to Agent Core and exposes 31-motor commands, 400 Hz trajectories, dual dexterous hands, named joint groups and arbitrary choreography.

The driver intentionally does not invent a walking topic. Vendor high-level gait and teleoperation packages can be added once the exact robot software image is known; all joints remain available through the official low-level contract.
