# Grasp status and slip detection estimation
std_msgs/Header header

# Whether the object has been successfully grasped and contact is established
bool grasp_established

# Whether relative slip or movement of the object in the gripper is detected
bool object_slipped

# Estimated net contact force and torque wrench exerted on the object (重力补偿后的末端净力矩)
geometry_msgs/Wrench net_wrench

# Confidence level of the estimator [0.0, 1.0]
float64 confidence
