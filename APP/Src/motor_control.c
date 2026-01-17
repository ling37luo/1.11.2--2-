#include "motor_control.h"
#include "can_bsp.h"
#include <math.h>

GM6020_TypeDef gm6020_motors[4];
int16_t motor_current_send_buf[4]; // 添加发送缓冲区定义

#define PI 3.1415926535f

/**
 * @brief 角度处理
 */
static void Motor_Process_Angle(GM6020_TypeDef *motor) {
    int diff = motor->ecd_raw - motor->last_ecd;
    if (diff > 4096) motor->round_cnt--;
    else if (diff < -4096) motor->round_cnt++;
    motor->total_angle = motor->round_cnt * 360.0f + (motor->ecd_raw / 8192.0f * 360.0f);
    motor->last_ecd = motor->ecd_raw;
}

/**
 * @brief 计算重力前馈
 */
static int16_t Motor_Calc_Gravity_FF(GM6020_TypeDef *motor) {
    if (motor->id == 4) {
        return 0;
    }
    int32_t ecd_diff = motor->ecd_raw - motor->horizon_ecd;
    
    if (ecd_diff > 4096)  ecd_diff -= 8192;
    if (ecd_diff < -4096) ecd_diff += 8192;

    float angle_rad = (float)ecd_diff * (2.0f * PI / 8192.0f);
    return (int16_t)(motor->gravity_gain * cosf(angle_rad));
}

// 辅助函数：限制电流范围
static int16_t limit_current(float val) {
    if (val > GM6020_MAX_CURRENT) return (int16_t)GM6020_MAX_CURRENT;
    if (val < -GM6020_MAX_CURRENT) return (int16_t)-GM6020_MAX_CURRENT;
    return (int16_t)val;
}

/**
 * @brief 堵转检测逻辑 (只保留一个定义)
 */
static void Motor_Check_Stall(GM6020_TypeDef *motor) {
    if (motor->mode == MOTOR_MODE_STOP || motor->is_stalled) return;

    if (fabsf(motor->output_current) > STALL_CURRENT_THRESH && 
        fabsf(motor->speed_rpm) < STALL_SPEED_THRESH) {
        
        motor->stall_cnt++;
        if (motor->stall_cnt > STALL_TIME_MS) {
            motor->is_stalled = 1;
            motor->mode = MOTOR_MODE_STOP;
        }
    } else {
        motor->stall_cnt = 0;
    }
}

// 清除堵转状态 (修复未使用变量警告)
void Motor_Clear_Stall(uint8_t id) {
    if (id < 1 || id > 4) return;
    // 直接使用数组，不需要中间变量
    gm6020_motors[id-1].is_stalled = 0;
    gm6020_motors[id-1].stall_cnt = 0;
    gm6020_motors[id-1].mode = MOTOR_MODE_CURRENT_SPEED;
    gm6020_motors[id-1].target_val = 0;
}

void Motor_Init_All(void) {
    for (int i = 0; i < 4; i++) {
        gm6020_motors[i].id = i + 1;
        gm6020_motors[i].mode = MOTOR_MODE_CURRENT_SPEED;
        gm6020_motors[i].target_val = 0;
        gm6020_motors[i].round_cnt = 0;
        gm6020_motors[i].is_stalled = 0;
        gm6020_motors[i].last_ecd = 0;
        gm6020_motors[i].feedforward = 0;
        gm6020_motors[i].horizon_ecd = 4096;
        
        // 上电机特殊处理
        if (i == 3) { // ID=4
            gm6020_motors[i].gravity_gain = 0.0f;
        } else {
            gm6020_motors[i].gravity_gain = 0.0f;
        }
        
        float speed_dead_zone = 1.0f;
        float angle_dead_zone = 0.5f; 
        
        // 优化PID参数（上电机）
        if (i == 3) { // ID=4
            PID_Init(&gm6020_motors[i].speed_pid, 
                     140.0f,   // Kp
                     5.0f,     // Ki
                     0.1f,     // Kd
                     GM6020_MAX_CURRENT,
                     5000.0f,  // 积分限幅
                     speed_dead_zone);
            
            PID_Init(&gm6020_motors[i].angle_pid, 
                     7.0f,     // Kp
                     0.0f,     
                     0.0f,
                     100.0f,   // MaxOut: 限制最大速度
                     0.0f,
                     angle_dead_zone);
        } 
        // 下电机参数
        else {
            PID_Init(&gm6020_motors[i].speed_pid, 
                     60.0f,    // Kp
                     3.3f,     // Ki
                     0.005f,   // Kd
                     GM6020_MAX_CURRENT, 
                     5000.0f,  // 积分限幅
                     0.0f);    // 无死区
            
            PID_Init(&gm6020_motors[i].angle_pid, 
                     5.0f,     // Kp
                     0.0f,     // Ki
                     0.0f,     // Kd
                     GM6020_MAX_SPEED, 
                     0.0f,     // MaxIOut
                     0.0f);    // 无死区
        }
        
        // 清除PID状态
        PID_Clear(&gm6020_motors[i].speed_pid);
        PID_Clear(&gm6020_motors[i].angle_pid);
    }
}

void Motor_Update_Feedback(uint8_t id, uint8_t *rx_data) {
    if (id < 1 || id > 4) return;
    GM6020_TypeDef *motor = &gm6020_motors[id - 1];
    
    motor->ecd_raw = (rx_data[0] << 8) | rx_data[1];
    motor->speed_rpm = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    motor->current_raw = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    motor->temp = rx_data[6];
    
    Motor_Process_Angle(motor);
}

void Motor_Set_Target(uint8_t id, Motor_Mode_e mode, float target) {
    if (id < 1 || id > 4) return;
    GM6020_TypeDef *motor = &gm6020_motors[id - 1];
    
    // 启用堵转保护
    if (motor->is_stalled) {
        return;
    }
    
    Motor_Mode_e old_mode = motor->mode;
    motor->mode = mode;
    motor->target_val = target;
    
    // 切换模式时清除 PID 积分
    if (old_mode != mode) {
        PID_Clear(&motor->speed_pid);
        PID_Clear(&motor->angle_pid);
    }
}

void Motor_Control_Loop(void) {
    for (int i = 0; i < 4; i++) {
        GM6020_TypeDef *motor = &gm6020_motors[i];
        float speed_target = 0;
        float current_target = 0;
        
        // 堵转保护
        if (motor->is_stalled || motor->mode == MOTOR_MODE_STOP) {
            motor->output_current = 0;
            motor_current_send_buf[i] = 0;
            continue;
        }

        switch (motor->mode) {
            case MOTOR_MODE_CURRENT_OPEN:
                current_target = motor->target_val;
                break;
                
            case MOTOR_MODE_CURRENT_SPEED:
                current_target = PID_Calc(&motor->speed_pid, motor->target_val, motor->speed_rpm);
                break;
                
            case MOTOR_MODE_CURRENT_POS_CASCADE:
                speed_target = PID_Calc(&motor->angle_pid, motor->target_val, motor->total_angle);
                current_target = PID_Calc(&motor->speed_pid, speed_target, motor->speed_rpm);
                break;
                
            default:
                current_target = 0;
                break;
        }

        if (motor->id != 4 && (motor->mode == MOTOR_MODE_CURRENT_SPEED || 
            motor->mode == MOTOR_MODE_CURRENT_POS_CASCADE)) {
            
            motor->feedforward = Motor_Calc_Gravity_FF(motor);
            current_target += motor->feedforward;
        }

        // 限制电流范围
        motor->output_current = limit_current(current_target);
        
        // 执行堵转检测
        Motor_Check_Stall(motor);
        
        // 填入发送缓冲区
        motor_current_send_buf[i] = motor->output_current;
    }
    
    // ✅ 修复函数名拼写错误
    BSP_CAN_Sned_Current_Cmd(); // 注意是 'Sned' 不是 'Send'
}

void Motor_Set_Dual_Target(uint8_t id1, Motor_Mode_e mode1, float target1,
                           uint8_t id2, Motor_Mode_e mode2, float target2) {
    Motor_Set_Target(id1, mode1, target1);
    Motor_Set_Target(id2, mode2, target2);
}