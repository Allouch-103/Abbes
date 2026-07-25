# BaseState.msg — estimated floating-base (torso) state from the InEKF.
std_msgs/Header header
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 position
geometry_msgs/Vector3 linear_velocity     # WORLD frame (m/s) — the missing signal
geometry_msgs/Vector3 angular_velocity    # BASE frame (rad/s)
bool left_contact
bool right_contact
bool valid                                # false while filter rejects bad init
