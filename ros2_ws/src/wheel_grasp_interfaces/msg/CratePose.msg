# Header stamp is the original observation time, not inference completion time.
std_msgs/Header header
string camera_id
string pair_id
string target_id
string model_id
geometry_msgs/Pose pose
float64 valid_depth_ratio
float64 surface_residual_m
bool quality_valid
bool diagnostic_only
string[] reject_reasons
