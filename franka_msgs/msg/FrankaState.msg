std_msgs/Header header
float64[6] cartesian_collision
float64[6] cartesian_contact
float64[7] q
float64[7] q_d
float64[7] dq
float64[7] dq_d
float64[7] ddq_d
float64[7] theta
float64[7] dtheta
float64[7] tau_j
float64[7] dtau_j
float64[7] tau_j_d
float64[6] k_f_ext_hat_k
float64[2] elbow
float64[2] elbow_d
float64[2] elbow_c
float64[2] delbow_c
float64[2] ddelbow_c
float64[7] joint_collision
float64[7] joint_contact
float64[6] o_f_ext_hat_k
float64[6] o_dp_ee_d
float64[3] o_ddp_o
float64[6] o_dp_ee_c
float64[6] o_ddp_ee_c
float64[7] tau_ext_hat_filtered
float64 m_ee
float64[3] f_x_cee
float64[9] i_ee
float64 m_load
float64[3] f_x_cload
float64[9] i_load
float64 m_total
float64[3] f_x_ctotal
float64[9] i_total
float64[16] o_t_ee
float64[16] o_t_ee_d
float64[16] o_t_ee_c
float64[16] f_t_ee
float64[16] f_t_ne
float64[16] ne_t_ee
float64[16] ee_t_k
float64 time
float64 control_command_success_rate
uint8 ROBOT_MODE_OTHER=0
uint8 ROBOT_MODE_IDLE=1
uint8 ROBOT_MODE_MOVE=2
uint8 ROBOT_MODE_GUIDING=3
uint8 ROBOT_MODE_REFLEX=4
uint8 ROBOT_MODE_USER_STOPPED=5
uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
uint8 robot_mode
franka_msgs/Errors current_errors
franka_msgs/Errors last_motion_errors
