# Cartesian controller control-mode status (ADVISORY / observability only).
# The controller's internal gate is the source of truth -- do NOT use this to decide whether
# to publish commands (read-vs-act is racy). It is for humans / GUI display.
string state              # "TOPIC"  : following ~/target_pose
                          # "HOMING" : running a go_home/go_pose service (ignores target_pose)
                          # "GUARD"  : ignoring target_pose until it comes near the current pose
float64 position_error    # [m]   distance from the latest ~/target_pose to the measured pose
float64 orientation_error # [deg] angle between the latest ~/target_pose and the measured pose
