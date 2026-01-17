#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "pid.h"
#include <stdint.h>

// === 参数定义 ===
#define GM6020_MAX_CURRENT     16384.0f // 对应 3A 电流 (0x1FE模式下)
#define GM6020_MAX_SPEED       320.0f   // 最大物理转速 (RPM)

// === 堵转检测参数 ===
#define STALL_CURRENT_THRESH   12000.0f // 判定堵转的电流阈值 (约1.8A)
#define STALL_SPEED_THRESH     100.0f    // 判定堵转的速度阈值 (RPM)
#define STALL_TIME_MS          1500     // 持续时间 (ms)，超过此时间判定为堵转

typedef enum {
    MOTOR_MODE_CURRENT_OPEN = 0,      // 电流开环 (直接给力矩)
    MOTOR_MODE_CURRENT_SPEED,         // 速度闭环 (输出电流)
    MOTOR_MODE_CURRENT_POS_CASCADE,   // 串级闭环 (位置->速度->电流)
    MOTOR_MODE_STOP,                  // 停止/保护模式
} Motor_Mode_e;

typedef struct {
    uint8_t id;
    Motor_Mode_e mode;
    
    // 反馈数据
    uint16_t ecd_raw;
    uint16_t last_ecd;
    int16_t  speed_rpm;
    int16_t  current_raw; // 实际转矩电流
    uint8_t  temp;
    
    // 角度计算
    int32_t  round_cnt;
    float    total_angle;
    
    // 控制目标
    float    target_val;  // 目标值 (根据模式不同，可能是电流、速度或角度)

    // PID 控制器
    PID_TypeDef speed_pid; // 速度环
    PID_TypeDef angle_pid; // 位置环
    PID_TypeDef current_pid; // 电流环
    
    // 输出
    int16_t  output_current; // 最终发送给电机的电流值 (-16384 ~ 16384)

    // 堵转检测
    uint32_t stall_cnt;    // 堵转计数器
    uint8_t  is_stalled;   // 堵转标志位 (1:堵转 0:正常)

    uint16_t horizon_ecd;   // 机械臂处于水平位置时的编码器原始值 (0-8191)
    float    gravity_gain;  // 重力补偿系数 (对应水平时需要的电流值)
    int16_t  feedforward;   // 计算出的前馈值

} GM6020_TypeDef;

extern GM6020_TypeDef gm6020_motors[4];
extern int16_t motor_current_send_buf[4];

void Motor_Init_All(void);
void Motor_Update_Feedback(uint8_t id, uint8_t *rx_data);
void Motor_Set_Target(uint8_t id, Motor_Mode_e mode, float target);
void Motor_Control_Loop(void);
void Motor_Clear_Stall(uint8_t id); // 新增：清除堵转状态

#endif