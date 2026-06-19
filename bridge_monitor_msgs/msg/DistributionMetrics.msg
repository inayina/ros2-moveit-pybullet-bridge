std_msgs/Header header

string[] joint_names
float64[] kl_divergence_per_joint
float64 kl_divergence_mean

float64[] wasserstein_per_joint
float64 wasserstein_mean
float64 w1_threshold
bool shift_detected_w1

float64 mmd_statistic
float64 mmd_p_value
float64 mmd_threshold

float64 window_duration_sec
uint32 sample_count_sim
uint32 sample_count_real

# Window position distribution per joint (for HOC boxplot; empty until min_samples met)
float64[] sim_position_min_per_joint
float64[] sim_position_q1_per_joint
float64[] sim_position_median_per_joint
float64[] sim_position_q3_per_joint
float64[] sim_position_max_per_joint
float64[] real_position_min_per_joint
float64[] real_position_q1_per_joint
float64[] real_position_median_per_joint
float64[] real_position_q3_per_joint
float64[] real_position_max_per_joint

bool shift_detected
string detection_method
