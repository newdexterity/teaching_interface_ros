string reference                              # Reference frame of the twist, can be 'base' or 'ee'
geometry_msgs/Twist twist                     # Unitless translational and rotational velocity commands for the robot end-effector
control_msgs/GripperCommand gripper_command   # Unitless gripper command [0:1]
bool waypoint                                 # Task waypoint trigger
