#include "pid.h"
#include <stdio.h>
#include <math.h>

/**
 * @brief PID 初始化
 * @param params p,i,d 参数数组
 * @param out_limit 输出限幅（3508电机对应 16384）
 * @param inte_limit 积分限幅（防止积分过冲）
 */
void PID_Init(PID_t *pid, const float params[3], float out_limit, float inte_limit, float i_sep) {
  if (pid == NULL || params == NULL)
    return;

  pid->Kp = params[0];
  pid->Ki = params[1];
  pid->Kd = params[2];
  pid->output_limit = out_limit;
  pid->integral_limit = inte_limit;
  pid->I_Sep_Thres = i_sep; // 赋值积分分离阈值

  PID_Reset(pid);
}

/**
 * @brief PID 计算
 * @note 放在中断中调用，建议频率 1kHz
 */
float PID_Calculate(PID_t *pid, float target, float measure) {
  if (pid == NULL)
    return 0.0f;

  pid->target = target;
  pid->measure = measure;
  pid->err = target - measure;

  // 1. 比例项
  pid->p_out = pid->Kp * pid->err;

  // 2. 积分项 + 积分分离逻辑
  if (fabsf(pid->err) < pid->I_Sep_Thres) {
    pid->i_out += pid->Ki * pid->err;
    
    // 积分限幅
    if (pid->i_out > pid->integral_limit)
      pid->i_out = pid->integral_limit;
    else if (pid->i_out < -pid->integral_limit)
      pid->i_out = -pid->integral_limit;
  } else {
    // 误差过大清零积分，消除长距离移动后的“记忆”延迟
    pid->i_out = 0.0f;
  }

  // 3. 微分项
  pid->d_out = pid->Kd * (pid->err - pid->last_err);

  // 4. 计算总输出并限幅
  pid->output = pid->p_out + pid->i_out + pid->d_out;

  if (pid->output > pid->output_limit)
    pid->output = pid->output_limit;
  else if (pid->output < -pid->output_limit)
    pid->output = -pid->output_limit;

  pid->last_err = pid->err;
  return pid->output;
}

/**
 * @brief 重置 PID 内部状态（清除积分和历史误差）
 */
void PID_Reset(PID_t *pid) {
  pid->err = 0.0f;
  pid->last_err = 0.0f;
  pid->i_out = 0.0f;
  pid->output = 0.0f;
}