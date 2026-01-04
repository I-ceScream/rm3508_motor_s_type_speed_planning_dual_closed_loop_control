# rm3508_motor_s_type_speed_planning_dual_closed_loop_control
RM3508电机双闭环控制，并使用S型速度规划，平滑速度变化
## API介绍
void RM3508_Init(void)：初始化模块

void RM3508_Control_Loop(void)：核心控制函数，定时计数器中1ms中断调用

void RM3508_Set_Speed(uint8_t id, float speed)：设置电机速度

void RM3508_Set_Angle_Abs(uint8_t id, float angle)：设置绝对位置，可以是多圈角度，也可到达

void RM3508_Set_Angle_Rel(uint8_t id, float delta_angle)：设置相对于当前的相对位置

void RM3508_Set_Enable(uint8_t id, uint8_t enable)：使能/失能

void RM3508_Rx_Callback(CAN_HandleTypeDef *hcan)：这是是模块的输入，由RM3508的CAN接收中断触发调用，不用担心效率问题，这函数执行速度很快的

void RM3508_Print_Status(uint8_t id)：打印电机状态

uint8_t RM3508_Is_Position_Arrived(uint8_t motor_idx)：获得电机的位置控制模式到位信号

float RM3508_Get_Absolute_Position(uint8_t id)：获得历史累计移动角度，这里需要注意一个问题，3508电机是存在机械零点的，所以这角度可能因为不同的初始角度，需要减去一定的偏移。这里并没有处理。

## 移植
这个模块需要CAN和TIM的支持才可以使用
1. CAN要求：开启接收中断，并在接收中断中调用RM3508_Rx_Callback即可
2. TIM要求：要求1ms中断，中断中调用RM3508_Control_Loop即可
