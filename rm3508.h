#ifndef __RM3508_H
#define __RM3508_H

#include <stdint.h>
#include "main.h"

// 使用的电机数量
#define RM3508_IN_USE_NUM 2
// 外部接口句柄
extern CAN_HandleTypeDef hcan2;
#define RM3508_CAN_HANDLE hcan2
extern TIM_HandleTypeDef htim6;
#define RM3508_TIM_HANDLE htim6


  typedef struct {
    uint16_t rotor_angle;   // 0-8191
    int16_t rotor_speed;    // RPM
    int16_t torque_current; // 实际电流
    int8_t temp;           // 温度
    // 第一次得到的角度值作为初始化的值
    uint16_t init_offset_angle;
    // 计算圈数
    uint16_t last_angle;
    int32_t round_cnt;
  } RM3508_Raw_t;


void RM3508_Init(void);
void RM3508_Control_Loop(void);
void RM3508_Set_Speed(uint8_t id, float speed);
void RM3508_Set_Angle_Abs(uint8_t id, float angle);
void RM3508_Set_Angle_Rel(uint8_t id, float delta_angle);
void RM3508_Set_Enable(uint8_t id, uint8_t enable);
void RM3508_Rx_Callback(CAN_HandleTypeDef *hcan);
void RM3508_Print_Status(uint8_t id);
uint8_t RM3508_Is_Position_Arrived(uint8_t motor_idx);
float RM3508_Get_Absolute_Position(uint8_t id);


#endif // __RM3508_H
