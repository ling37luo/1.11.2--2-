#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h" 

// 引用外部电机控制头文件以获取反馈更新函数
#include "motor_control.h"

extern CAN_HandleTypeDef hcan1;

void BSP_CAN_Init(void);

// 导出电压缓冲区，供外部修改
extern int16_t motor_voltage_send_buf[4];

// 新增：发送电压控制指令函数
void BSP_CAN_Send_Voltage_Cmd(void);

void BSP_CAN_Sned_Current_Cmd(void);
#endif