# Sim vs Real tracking error sequence (pybullet_bridge publishes at publish_frequency)
std_msgs/Header header
string[] joint_names
float64[] q_error
float64[] dq_error
# Sim reference timestamp used for alignment (M4 input)
builtin_interfaces/Time timestamp
# When Real instance actually applied the delayed command (sim-time seconds)
float64 real_execution_time_sec
