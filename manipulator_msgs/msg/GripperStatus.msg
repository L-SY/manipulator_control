# 标准消息头
Header header

# 基本状态
string state           # 当前状态 (IDLE, MOVING, HOLDING, ERROR, SELF_TEST)
int8 current_state
int8 previous_state
float64 position       # 当前位置
float64 velocity       # 当前速度
float64 effort         # 当前力矩

# 目标信息
float64 target_position  # 目标位置
float64 target_effort    # 目标力矩

# 堵转信息
bool is_stalled          # 是否处于堵转状态
bool is_instant_stalled  # 是否瞬时堵转

string command           # 当前执行的命令
