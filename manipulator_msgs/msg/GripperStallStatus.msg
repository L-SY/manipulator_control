Header header
bool is_stalled                # 是否处于堵转状态
float64 current_velocity       # 当前速度绝对值
float64 current_effort         # 当前力矩绝对值
float64 velocity_threshold     # 速度阈值
float64 effort_threshold       # 力矩阈值
float64 stall_duration         # 堵转持续时间(秒)
float64 timeout_threshold      # 堵转判定时间阈值
string joint_name              # 关节名称
