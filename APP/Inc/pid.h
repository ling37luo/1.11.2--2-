#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    // === 原 pid2 成员 ===
    float Kp;
    float Ki;
    float Kd;
    float max_out;  // 最大输出限幅
    float max_iout; // 积分限幅
    
    float set_point; // 目标值
    float fdb;       // 反馈值
    float err;       // 当前误差
    float last_err;  // 上次误差
    
    float Pout;
    float Iout;
    float Dout;
    float out;       // 总输出

    float I_Separation_Thresh; // 积分分离阈值 (0表示禁用)
    float LPF_RC;              // 微分低通滤波系数 [0, 1) (0表示禁用)
    float last_derivative;     // 上次微分值 (用于滤波)
    float dead_zone;  
} PID_TypeDef;


void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_iout,float dead_zone);
float PID_Calc(PID_TypeDef *pid, float target, float feedback);
void PID_Clear(PID_TypeDef *pid);

#endif