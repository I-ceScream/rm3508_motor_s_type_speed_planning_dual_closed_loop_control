#ifndef S_CURVE_H
#define S_CURVE_H

#include <stdint.h>

/**
 * @brief S型曲线规划结构体
 */
typedef struct {
  float target_velocity;  // 最终目标速度 (User Input)
  float current_velocity; // 当前规划出的速度 (Output to PID)
  float current_accel;    // 当前加速度

  float max_accel; // 最大加速度限制
  float jerk;      // 加加速度 (Jerk)，决定加速度变化的速率

  float control_period; // 控制周期 (秒)，例如 0.001f (1ms)
} S_Curve_t;

/* 函数声明 */
void S_Curve_Init(S_Curve_t *s, float max_acc, float jerk, float period);
float S_Curve_Calculate(S_Curve_t *s, float target_v);
void S_Curve_Reset(S_Curve_t *s, float init_v);

#endif /* S_CURVE_H */