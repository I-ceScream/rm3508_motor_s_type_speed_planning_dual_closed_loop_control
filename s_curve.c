#include "s_curve.h"
#include <stdio.h>

/**
 * @brief 初始化S曲线参数
 * @param max_acc 最大加速度 (unit/s^2)
 * @param jerk 加加速度 (unit/s^3)
 * @param period 控制周期 (s)，如 0.001f
 */
void S_Curve_Init(S_Curve_t *s, float max_acc, float jerk, float period) {
    if (s == NULL) return;
    s->max_accel = max_acc;
    s->jerk = jerk;
    s->control_period = period;
    s->current_velocity = 0.0f;
    s->current_accel = 0.0f;
    s->target_velocity = 0.0f;
}

/**
 * @brief S型速度规划计算
 * @param s 规划器结构体指针
 * @param target_v 用户期望的目标速度
 * @return float 下一时刻应执行的规划速度
 */
float S_Curve_Calculate(S_Curve_t *s, float target_v) {
    s->target_velocity = target_v;
    float v_diff = s->target_velocity - s->current_velocity;

    if (v_diff != 0.0f) {
        // 1. 确定加速度的方向
        float acc_step = s->jerk * s->control_period;
        
        if (v_diff > 0.0f) {
            // 需要加速：增加加速度
            s->current_accel += acc_step;
            if (s->current_accel > s->max_accel) s->current_accel = s->max_accel;
        } else {
            // 需要减速：减小加速度
            s->current_accel -= acc_step;
            if (s->current_accel < -s->max_accel) s->current_accel = -s->max_accel;
        }

        // 2. 更新当前规划速度
        float v_step = s->current_accel * s->control_period;
        
        // 防止过冲：如果这一步的增量超过了剩余差值
        if ((v_diff > 0.0f && v_step > v_diff) || (v_diff < 0.0f && v_step < v_diff)) {
            s->current_velocity = s->target_velocity;
            s->current_accel = 0.0f;
        } else {
            s->current_velocity += v_step;
        }
    } else {
        s->current_accel = 0.0f;
    }

    return s->current_velocity;
}

/**
 * @brief 重置规划器
 */
void S_Curve_Reset(S_Curve_t *s, float init_v) {
    s->current_velocity = init_v;
    s->current_accel = 0.0f;
    s->target_velocity = init_v;
}