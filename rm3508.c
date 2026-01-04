#include "rm3508.h"
#include "log.h"
#include "pid.h"
#include "s_curve.h"
#include <math.h>
#include <stdint.h>

typedef struct {
  // 基础反馈数据 (来自 CAN 接收)
  RM3508_Raw_t raw;
  // 控制器
  PID_t pos_pid;     // 位置环PID
  PID_t speed_pid;   // 速度环PID
  S_Curve_t s_curve; // 速度S型规划器
  // 目标值
  float target_angle; // 目标角度 (单位: °)
  float target_speed; // 最终期望达到的速度
  // 速度/位置控制模式
  uint8_t mode; // 0=速度模式, 1=相对位置模式 2=绝对位置模式
  // 使能标志
  uint8_t is_enable;
  // 稳定计数器 (用于位置到达判定)
  uint16_t stable_count;
  // 位置控制到达信号 0=未完成, 1=完成
  uint8_t position_arrived_signal;
  // 稳定信号是否已经处理过了 防止重复触发
  uint8_t stable_signal_processed; // 0=未处理, 1=已处理
} RM3508_Control_t;
static RM3508_Control_t motor_ctrl[RM3508_IN_USE_NUM];
// 定义发送缓冲区
static int16_t g_can8_tx_buf[8] = {0};
// 对应的 CAN ID 宏
#define CAN_ID_GROUP_1 0x200 // 控制 ID 1-4
#define CAN_ID_GROUP_2 0x1FF // 控制 ID 5-8
#define ANGLE_CONVERT_FACTOR 0.0439453125f

/**
 * @brief 获取电机当前总角度 (考虑圈数)
 * @param m 电机控制结构体指针
 * @return float 总角度 (单位: °)
 */
static float get_motor_total_angle(RM3508_Control_t *m) {
  // 先计算总脉冲差值
  int32_t total_ticks = (m->raw.round_cnt << 13) +
                        (int32_t)m->raw.rotor_angle -
                        (int32_t)m->raw.init_offset_angle;
  // 转换为物理角度
  return (float)total_ticks * ANGLE_CONVERT_FACTOR;
}
/**
 * @brief 更新电机控制缓冲区
 * @param id 电机ID (1-8)
 * @param enable 使能标志 (1: 正常输出, 0: 强制输出0电流)
 * @param current 计算出的控制电流
 */
static void Update_Motor_Control(uint8_t id, uint8_t enable, int16_t current) {
  if (id < 1 || id > RM3508_IN_USE_NUM)
    return;

  uint8_t idx = id - 1;
  if (enable) {
    // 电流限幅保护
    if (current > 16384)
      current = 16384;
    if (current < -16384)
      current = -16384;
    g_can8_tx_buf[idx] = current;
  } else {
    g_can8_tx_buf[idx] = 0;
  }
}
/**
 * @brief 硬件抽象层发送接口 (可移植修改)
 */
static void CAN_Transmit_Interface(uint32_t std_id, uint8_t *data) {
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;

  tx_header.StdId = std_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  // 发送消息，如果邮箱满则跳过（避免阻塞中断）
  if (HAL_CAN_GetTxMailboxesFreeLevel(&RM3508_CAN_HANDLE) > 0) {
    HAL_CAN_AddTxMessage(&RM3508_CAN_HANDLE, &tx_header, data, &tx_mailbox);
  }
}

/**
 * @brief 统一发送函数，根据 IN_USE_NUM 自动决定发送几帧
 */
static void RM3508_Send_Control(void) {
  uint8_t tx_data[8];

  // --- 处理 0x200 (电机 1-4) ---
  // 即使 RM3508_IN_USE_NUM 小于4，通常也建议发送全8字节以维持电调心跳
  tx_data[0] = (uint8_t)(g_can8_tx_buf[0] >> 8);
  tx_data[1] = (uint8_t)(g_can8_tx_buf[0] & 0xFF);
  tx_data[2] = (uint8_t)(g_can8_tx_buf[1] >> 8);
  tx_data[3] = (uint8_t)(g_can8_tx_buf[1] & 0xFF);
  tx_data[4] = (uint8_t)(g_can8_tx_buf[2] >> 8);
  tx_data[5] = (uint8_t)(g_can8_tx_buf[2] & 0xFF);
  tx_data[6] = (uint8_t)(g_can8_tx_buf[3] >> 8);
  tx_data[7] = (uint8_t)(g_can8_tx_buf[3] & 0xFF);
  CAN_Transmit_Interface(CAN_ID_GROUP_1, tx_data);
  // --- 处理 0x1FF (电机 5-8) ---
#if RM3508_IN_USE_NUM > 4
  tx_data[0] = (uint8_t)(g_can8_tx_buf[4] >> 8);
  tx_data[1] = (uint8_t)(g_can8_tx_buf[4] & 0xFF);
  tx_data[2] = (uint8_t)(g_can8_tx_buf[5] >> 8);
  tx_data[3] = (uint8_t)(g_can8_tx_buf[5] & 0xFF);
  tx_data[4] = (uint8_t)(g_can8_tx_buf[6] >> 8);
  tx_data[5] = (uint8_t)(g_can8_tx_buf[6] & 0xFF);
  tx_data[6] = (uint8_t)(g_can8_tx_buf[7] >> 8);
  tx_data[7] = (uint8_t)(g_can8_tx_buf[7] & 0xFF);
  CAN_Transmit_Interface(CAN_ID_GROUP_2, tx_data);
#endif
}

/**
 * @brief 电机控制任务
 * @note  必须在定时器中断中每 1ms 调用一次！
 */
void RM3508_Control_Loop(void) {
  static uint8_t loop_cnt = 0;
  loop_cnt++;

  // 确定本轮是否需要运行位置环 (100Hz = 每 10ms 运行一次)
  uint8_t run_pos_loop = (loop_cnt % 10 == 0);

  for (int i = 0; i < RM3508_IN_USE_NUM; i++) {
    RM3508_Control_t *m = &motor_ctrl[i];
    // --- 0. 安全检查 ---
    // 如果电机未使能或未完成初始化，直接输出0电流并重置信号
    if (!m->is_enable || m->raw.init_offset_angle == 0xFFFF) {
      Update_Motor_Control(i + 1, 0, 0);
      m->stable_count = 0;
      m->position_arrived_signal = 0;
      m->stable_signal_processed = 0;
      continue;
    }
    float current_angle = get_motor_total_angle(m);

    // --- 1. 位置环 (100Hz) ---
    // 只有在角度模式下 (mode 1或2) 且轮次到达时才计算
    if (run_pos_loop && (m->mode != 0)) {
      // 计算出的目标速度 (RPM) 存入 target_speed 供速度环使用
      m->target_speed =
          PID_Calculate(&m->pos_pid, m->target_angle, current_angle);
    }

    // --- 2. 速度环 (1000Hz) ---
    // 通过 S 曲线规划器平滑速度目标
    float planned_v = S_Curve_Calculate(&m->s_curve, m->target_speed);
    float actual_v = (float)m->raw.rotor_speed;
    float current_cmd = PID_Calculate(&m->speed_pid, planned_v, actual_v);

    // --- 3. 位置到达判定逻辑 (1000Hz) ---
    if (m->mode != 0) {
      float angle_error = fabsf(m->target_angle - current_angle);
      float abs_actual_v = fabsf(actual_v);

      // 判定条件：误差 < 10° 且 实际转速 < 100 RPM
      if (angle_error < 50.0f && abs_actual_v < 500.0f) {
        if (m->stable_count < 500) {
          m->stable_count++;
        } else {
          m->position_arrived_signal = 1;
        }
      } else {
        // 只要不满足条件，立即重置计数和信号
        m->stable_count = 0;
        m->position_arrived_signal = 0;
        m->stable_signal_processed = 0; // 如果需要重新触发逻辑，重置此标志
      }
    }
    // --- 4. 更新输出 ---
    Update_Motor_Control(i + 1, 1, (int16_t)current_cmd);
  }
  // --- 5. 硬件发送 (1000Hz) ---
  RM3508_Send_Control();
  // 计数器归零 (100ms 一个大周期)
  if (loop_cnt >= 10)
    loop_cnt = 0;
}

/**
 * @brief 设置速度模式
 * @param id 电机ID (1-N)
 * @param speed 目标速度 (RPM)
 */
void RM3508_Set_Speed(uint8_t id, float speed) {
  uint8_t idx = id - 1;
  if (idx >= RM3508_IN_USE_NUM)
    return;
  motor_ctrl[idx].position_arrived_signal = 0;
  motor_ctrl[idx].mode = 0;
  motor_ctrl[idx].target_speed = speed;
}

/**
 * @brief 设置绝对角度
 * @param angle 目标角度 (°), 例如 90.0f, 720.0f(转两圈)
 */
void RM3508_Set_Angle_Abs(uint8_t id, float angle) {
  uint8_t idx = id - 1;
  if (idx >= RM3508_IN_USE_NUM)
    return;
  motor_ctrl[idx].position_arrived_signal = 0;
  motor_ctrl[idx].mode = 2;
  motor_ctrl[idx].target_angle = angle;
}

/**
 * @brief 设置相对角度增量
 * @param delta_angle 增加的角度 (°), 例如 -45.0f
 */
void RM3508_Set_Angle_Rel(uint8_t id, float delta_angle) {
  uint8_t idx = id - 1;
  if (idx >= RM3508_IN_USE_NUM)
    return;
  motor_ctrl[idx].position_arrived_signal = 0;
  motor_ctrl[idx].mode = 1;
  motor_ctrl[idx].target_angle =
      get_motor_total_angle(&motor_ctrl[idx]) + delta_angle;
}

/**
 * @brief 停止/使能电机
 */
void RM3508_Set_Enable(uint8_t id, uint8_t enable) {
  uint8_t idx = id - 1;
  if (idx >= RM3508_IN_USE_NUM)
    return;

  motor_ctrl[idx].is_enable = enable;
  if (!enable) {
    Update_Motor_Control(id, 0, 0); // 立即下发0电流
    PID_Reset(&motor_ctrl[idx].speed_pid);
    PID_Reset(&motor_ctrl[idx].pos_pid);
    S_Curve_Reset(&motor_ctrl[idx].s_curve, 0);
  }
}

void RM3508_Init(void) {
  for (int i = 0; i < RM3508_IN_USE_NUM; i++) {
    motor_ctrl[i].raw.init_offset_angle = 0xFFFF; // 标记未初始化
    motor_ctrl[i].is_enable = 1;                  // 默认使能
    motor_ctrl[i].mode = 0;                       // 默认速度模式

    /* --- 1. 位置环 PID 初始化 --- */
    float pos_params[3] = {0.7f, 0.00003f, 0.00f};
    PID_Init(&motor_ctrl[i].pos_pid, pos_params, 3000.0f,
             3000.0f, 3600.0f);

    /* --- 2. 速度环 PID 初始化 --- */
    float speed_params[3] = {5.0f, 0.28f, 0.00f};
    // 映射到 C620 电调的最大控制电流值 16384
    PID_Init(&motor_ctrl[i].speed_pid, speed_params, 16384.0f,
             16384.0f, 500);

    /* --- 3. S 曲线规划器初始化 --- */
    S_Curve_Init(&motor_ctrl[i].s_curve, 10000.0f, 50000.0f, 0.001f);
    motor_ctrl[i].stable_count = 0;
    motor_ctrl[i].position_arrived_signal = 0;
    motor_ctrl[i].target_angle = 0;
    motor_ctrl[i].target_speed = 0;
  }
  // --- 启动硬件定时器中断 ---
  HAL_TIM_Base_Start_IT(&RM3508_TIM_HANDLE);
}

/**
 * @brief CAN接收中断回调函数
 * @note  该函数应在 HAL_CAN_RxFifo0MsgPendingCallback 中被调用
 */
/**
 * @brief CAN接收中断回调函数 (适配 RM3508_Control_t)
 */
void RM3508_Rx_Callback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // 1. 校验硬件接口 (确保是控制电机的那个CAN接口)
  if (hcan->Instance != RM3508_CAN_HANDLE.Instance) {
    return;
  }

  // 2. 尝试获取 CAN 消息 (从 FIFO0 提取)
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
    return;
  }

  // 3. 校验 ID 范围 (0x201 - 0x208)
  if (rx_header.StdId >= 0x201 &&
      rx_header.StdId < (0x201 + RM3508_IN_USE_NUM)) {
    uint8_t idx = rx_header.StdId - 0x201;

    // --- 核心改动：指向控制结构体中的 raw 成员 ---
    RM3508_Raw_t *motor_raw = &motor_ctrl[idx].raw;

    // --- 数据解析 (大端模式组合) ---
    motor_raw->last_angle = motor_raw->rotor_angle; // 记录上一时刻角度
    motor_raw->rotor_angle = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
    motor_raw->rotor_speed = (int16_t)(rx_data[2] << 8 | rx_data[3]);
    motor_raw->torque_current = (int16_t)(rx_data[4] << 8 | rx_data[5]);
    motor_raw->temp = (int8_t)rx_data[6];

    // --- 逻辑处理：初始化偏移 ---
    // 使用 0xFFFF 判断是否为上电后第一次接收数据
    if (motor_raw->init_offset_angle == 0xFFFF) {
      motor_raw->init_offset_angle = motor_raw->rotor_angle;
      motor_raw->last_angle = motor_raw->rotor_angle;
      motor_raw->round_cnt = 0;
      return;
    }

    // --- 逻辑处理：累计圈数 (过零检测算法) ---
    // 判定准则：单次跳变超过 4096 (半圈) 则认为发生了机械角度回环
    if (motor_raw->rotor_angle - motor_raw->last_angle > 4096) {
      motor_raw->round_cnt--; // 逆时针跳变：0 -> 8191
    } else if (motor_raw->rotor_angle - motor_raw->last_angle < -4096) {
      motor_raw->round_cnt++; // 顺时针跳变：8191 -> 0
    }
  }
}

/**
 * @brief  获取指定电机的位置到达信号 (单次触发逻辑)
 * @param  id: 电机ID (1 到 RM3508_IN_USE_NUM)
 * @retval 1: 刚刚到达并稳定, 0: 仍在运动、未使能或已经处理过该到达信号
 */
uint8_t RM3508_Is_Position_Arrived(uint8_t id) {
  uint8_t idx = id - 1;
  if (idx > RM3508_IN_USE_NUM)
    return 0;
  RM3508_Control_t *m = &motor_ctrl[idx];
  // 判定条件：1. 硬件判定到位  2. 用户层尚未处理过这个“到位”事件
  if (m->position_arrived_signal == 1 && m->stable_signal_processed == 0) {
    m->stable_signal_processed = 1; // 标记该次到达事件已处理
    return 1;                       // 返回确认信号
  }
  return 0; // 否则返回未到达或已处理过
}

/**
 * @brief  获取指定电机的绝对位置角度 (累计角度)
 * @param  id: 电机ID (1 到 RM3508_IN_USE_NUM)
 * @return float: 当前累计总角度 (单位: °)。例如转 2 圈则返回 720.0
 */
float RM3508_Get_Absolute_Position(uint8_t id) {
  uint8_t idx = id - 1;
  if (idx >= RM3508_IN_USE_NUM) {
    return 0.0f;
  }
  return get_motor_total_angle(&motor_ctrl[idx]);
}

/**
 * @brief 格式化打印电机详细状态
 * @param id 电机ID (1-RM3508_IN_USE_NUM)
 */
void RM3508_Print_Status(uint8_t id) {
    if (id < 1 || id > RM3508_IN_USE_NUM) return;
    RM3508_Control_t *m = &motor_ctrl[id - 1];

    // 1. 预处理数据：提取整数和小数部分 (处理两位小数)
    float total_ang = get_motor_total_angle(m);
    int32_t ang_i = (int32_t)total_ang;
    int32_t ang_f = (int32_t)(fabsf(total_ang - ang_i) * 100);

    float target_a = m->target_angle;
    int32_t tar_a_i = (int32_t)target_a;
    int32_t tar_a_f = (int32_t)(fabsf(target_a - tar_a_i) * 100);

    // 2. 状态映射
    const char* mode_str = (m->mode == 0) ? "SPD" : (m->mode == 1 ? "REL" : "ABS");
    char arrived_char = m->position_arrived_signal ? 'Y' : 'N';
    // 3. 打印美化输出
    // 第一行：电机基本状态头
    LOG_INFO("=== MOTOR [%d] | %s | %s ===", 
              id, m->is_enable ? "ENABLE " : "DISABLE", mode_str);
    // 第二行：位置信息 (目标 vs 当前)
    LOG_INFO("  POS: Target:%4ld.%02ld | Current:%4ld.%02ld | Arrived:[%c]", 
              tar_a_i, tar_a_f, ang_i, ang_f, arrived_char);
    // 第三行：速度与动力信息
    LOG_INFO("  DYN: Tar_V:%5d | Cur_V:%5d | Out:%5d | Temp:%2dC", 
              (int)m->target_speed, m->raw.rotor_speed, (int)m->speed_pid.output, m->raw.temp);
    // 第四行：装饰线（可选，增加可读性）
    LOG_INFO("-------------------------------------------------");
}