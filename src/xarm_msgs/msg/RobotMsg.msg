std_msgs/Header header
# 受控机器人的反馈信息

# 机器人的状态:
#   1：RUNNING（运行中），正在执行运动指令。
#   2：SLEEPING（待机），未执行指令，但已准备好运动。
#   3：PAUSED（暂停），在未完成的运动中暂停。
#   4：STOPPED（停止），未准备好接受任何运动指令。
#   5：CONFIG_CHANGED（配置已更改），系统配置或模式已更改，未准备好运动指令。
int16 state

# 0：POSITION模式（位置模式）—— 通过xarm控制器盒进行位置控制，执行标准API指令。
# 1：SERVOJ模式（伺服模式）—— 立即执行接收到的关节空间目标，类似于步进响应。
# 2：TEACHING_JOINT模式（关节教学模式）—— 重力补偿模式，便于进行教学。
int16 mode

# cmdnum：缓冲区中等待的命令数量。
int16 cmdnum

# mt_brake：转换为二进制数时，每个位表示一个轴，1表示刹车启用，0表示刹车禁用。
int16 mt_brake

# mt_able：转换为二进制数时，每个位表示一个轴，1表示伺服控制启用，0表示伺服禁用。
int16 mt_able

# 错误代码（非零时表示存在错误）。
int16 err

# 警告代码（非零时表示存在警告）。
int16 warn

# 当前关节角度，以弧度表示。
float32[] angle

# 当前TCP笛卡尔位置，以毫米（位置）和弧度（方向）表示。
float32[6] pose

# 相对于工具框架，从法兰中心偏移的TCP偏移量。
float32[6] offset
