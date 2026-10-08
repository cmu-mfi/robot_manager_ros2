# A Pose with reference & target frame
string frame_id                 # Leave emtpy to use base_link, otherwise any link is accepted
string target_id                # Leave empty to use tool0, otherwise any child frame of tool0 is accepted
geometry_msgs/Pose target_pose
