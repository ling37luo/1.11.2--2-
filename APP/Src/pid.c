/**
 * @file pid.c
 * @brief 通用 PID 实现 (融合了积分分离 + 微分低通滤波)
 */

#include "pid.h"
#include <math.h> // 需要用到 fabsf

/**
 * @brief 初始化 PID (保持 pid2 接口)
 * @note  新增的积分分离和滤波功能默认初始化为 0 (禁用)。
 *        如果需要使用，请在 Init 后手动修改结构体成员：
 *        pid.I_Separation_Thresh = value;
 *        pid.LPF_RC = value;
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_iout,float dead_zone) {
    if (!pid) return;

    // 1. 初始化基本参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;

    pid->I_Separation_Thresh = 0.0f; // 0 表示不开启积分分离
    pid->LPF_RC = 0.3f;              // 0 表示不开启微分滤波
    pid->dead_zone = 0.0f;
    // 3. 清除状态
    PID_Clear(pid);
}

/**
 * @brief 清除 PID 状态
 */
void PID_Clear(PID_TypeDef *pid) {
    if (!pid) return;

    pid->err = 0;
    pid->last_err = 0;
    pid->Iout = 0;
    pid->Pout = 0;
    pid->Dout = 0;
    pid->out = 0;
    
    // 清除新增的状态
    pid->last_derivative = 0;
}

/**
 * @brief 计算 PID 输出
 * @note  融合了 pid1 的积分分离和微分滤波逻辑
 */
float PID_Calc(PID_TypeDef *pid, float target, float feedback) {
    if (!pid) return 0.0f;

    pid->set_point = target;
    pid->fdb = feedback;
    pid->err = pid->set_point - pid->fdb;
if (fabsf(pid->err) < pid->dead_zone) {
        pid->err = 0.0f;
        pid->Iout = 0.0f; // 清除积分
        pid->Pout = 0.0f;
        pid->Dout = 0.0f;
        
        // 更新历史误差（保持状态）
        pid->last_err = pid->err;
        pid->out = 0.0f;
        return 0.0f;
    }
    // === 1. 比例项 (P) ===
    pid->Pout = pid->Kp * pid->err;
    
    // === 2. 积分项 (I) - 加入积分分离逻辑 ===
    // 逻辑：如果阈值为0(默认) 或者 当前误差绝对值 < 阈值，则进行积分
    if (pid->I_Separation_Thresh == 0.0f || fabsf(pid->err) < pid->I_Separation_Thresh) {
        pid->Iout += pid->Ki * pid->err;
        
        // 积分限幅
        if (pid->Iout > pid->max_iout) pid->Iout = pid->max_iout;
        else if (pid->Iout < -pid->max_iout) pid->Iout = -pid->max_iout;
    } else {
        // 误差过大，积分清零 (积分分离)
        pid->Iout = 0.0f;
    }

    // === 3. 微分项 (D) - 加入低通滤波逻辑 ===
    float raw_d = pid->err - pid->last_err; // 原始微分量
    
    if (pid->LPF_RC > 0.0f && pid->LPF_RC < 1.0f) {
        // 启用滤波: D(k) = D(k-1)*(1-RC) + Raw_D*RC
        float current_derivative = pid->last_derivative * (1.0f - pid->LPF_RC) + raw_d * pid->LPF_RC;
        pid->Dout = pid->Kd * current_derivative;
        pid->last_derivative = current_derivative; // 更新滤波后的微分状态
    } else {
        // 禁用滤波 (pid2 默认行为)
        pid->Dout = pid->Kd * raw_d;
        pid->last_derivative = raw_d; // 记录状态以便切换模式
    }

    // === 4. 总输出计算 ===
    pid->out = pid->Pout + pid->Iout + pid->Dout;

    // 总输出限幅
    if (pid->out > pid->max_out) pid->out = pid->max_out;
    else if (pid->out < -pid->max_out) pid->out = -pid->max_out;
    
    // 更新历史误差
    pid->last_err = pid->err;

    return pid->out;
}