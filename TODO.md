# TODO
1. Send trajectory message from web front-end to ROS controller
 - probably start with something that inches a single joint forward
 - subscribe to /joint_states, publish to /arm_controller/follow_joint_trajectory/goal with message that slightly increments joint