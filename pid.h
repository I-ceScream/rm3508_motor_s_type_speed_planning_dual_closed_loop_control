#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
  float Kp;
  float Ki;
  float Kd;

  float target;
  float measure;
  float err;
  float last_err;

  float p_out;
  float i_out;
  float d_out;
  float output;

  float integral_limit;
  float output_limit;
  float I_Sep_Thres; // 积分分离阈值
} PID_t;

/* 更新函数声明：删掉 mode 参数，增加 i_sep */
void PID_Init(PID_t *pid, const float params[3], float out_limit,
              float inte_limit, float i_sep);
float PID_Calculate(PID_t *pid, float target, float measure);
void PID_Reset(PID_t *pid);

#endif /* PID_H */