#include "motor_control.h"
#include "can_bsp.h"
#include <math.h>

GM6020_TypeDef gm6020_motors[4];

#define PI 3.1415926535f

/**
 * @brief 计算重力前馈
 * @note  逻辑：水平时输出最大(Gain)，垂直时输出0
 */
static int16_t Motor_Calc_Gravity_FF(GM6020_TypeDef *motor) {
    // 1. 计算当前角度与水平零点的偏差 (转换为弧度)
    // GM6020 一圈是 0-8191
    if (motor->id == 4) {
        return 0;
    }
    int32_t ecd_diff = motor->ecd_raw - motor->horizon_ecd;
    
    // 处理过零点问题 (例如从 8190 到 10)
    if (ecd_diff > 4096)  ecd_diff -= 8192;
    if (ecd_diff < -4096) ecd_diff += 8192;

    // 转换为弧度: (diff / 8192) * 2PI
    float angle_rad = (float)ecd_diff * (2.0f * PI / 8192.0f);

    // 2. 计算前馈值
    // 使用 cos：当 angle_rad = 0 (水平) 时，cos=1，输出最大 Gain
    // 当 angle_rad = PI/2 (垂直) 时，cos=0，输出 0
    // 注意：根据电机安装方向，可能需要改为 -motor->gravity_gain
    return (int16_t)(motor->gravity_gain * cosf(angle_rad));
}

// 辅助函数：限制电流范围
static int16_t limit_current(float val) {
    if (val > GM6020_MAX_CURRENT) return (int16_t)GM6020_MAX_CURRENT;
    if (val < -GM6020_MAX_CURRENT) return (int16_t)-GM6020_MAX_CURRENT;
    return (int16_t)val;
}

// 角度处理 (保持不变)
static void Motor_Process_Angle(GM6020_TypeDef *motor) {
    int diff = motor->ecd_raw - motor->last_ecd;
    if (diff > 4096) motor->round_cnt--;
    else if (diff < -4096) motor->round_cnt++;
    motor->total_angle = motor->round_cnt * 360.0f + (motor->ecd_raw / 8192.0f * 360.0f);
    motor->last_ecd = motor->ecd_raw;
}

// 堵转检测逻辑
static void Motor_Check_Stall(GM6020_TypeDef *motor) {
    // 如果处于停止模式或已经堵转，不重复检测
    if (motor->mode == MOTOR_MODE_STOP || motor->is_stalled) return;

    // 逻辑：输出电流绝对值 > 阈值  且  实际转速绝对值 < 阈值
    if (fabsf(motor->output_current) > STALL_CURRENT_THRESH && 
        fabsf(motor->speed_rpm) < STALL_SPEED_THRESH) {
        
        motor->stall_cnt++;
        if (motor->stall_cnt > STALL_TIME_MS) {
            motor->is_stalled = 1; // 标记堵转
            motor->mode = MOTOR_MODE_STOP; // 强制切换到停止模式保护电机
        }
    } else {
        motor->stall_cnt = 0; // 计数清零
    }
}

// 清除堵转状态
void Motor_Clear_Stall(uint8_t id) {
    if (id < 1 || id > 4) return;
    gm6020_motors[id-1].is_stalled = 0;
    gm6020_motors[id-1].stall_cnt = 0;
    gm6020_motors[id-1].mode = MOTOR_MODE_CURRENT_SPEED; // 恢复到默认安全模式
    gm6020_motors[id-1].target_val = 0;
}

void Motor_Init_All(void) {
    for (int i = 0; i < 4; i++) {
        gm6020_motors[i].id = i + 1;
        gm6020_motors[i].mode = MOTOR_MODE_CURRENT_SPEED;
        gm6020_motors[i].target_val = 0;
        gm6020_motors[i].round_cnt = 0;
        gm6020_motors[i].is_stalled = 0;
        
        // === 1. 速度环 PID (内环) ===
        // 输入：RPM 误差
        // 输出：目标电流值 (-16384 ~ 16384) -> 直接发给电机
        PID_Init(&gm6020_motors[i].speed_pid, 
                 60.0f,    // Kp: 建议从 50-100 开始调试
                 3.3f,      // Ki: 必须有，用于消除静差
                 0.005f,      // Kd: 速度环通常设为 0，防止噪声放大导致抖动
                 GM6020_MAX_CURRENT, // MaxOut: 16384
                 5000.0f,1.0f);           // MaxIOut: 积分限幅不要太大，5000足够
        
        // === 2. 位置环 PID (外环) ===
        // 输入：角度误差
        // 输出：目标 RPM
        PID_Init(&gm6020_motors[i].angle_pid, 
                 5.0f,    // Kp: 1度误差 -> 10 RPM
                 0.0f,     // Ki: 位置环通常不需要 I
                 0.0f,     // Kd: 位置环 D 给 0 或极小
                 GM6020_MAX_SPEED,   // MaxOut: 限制最大转速 320
                 0.0f,1.0f);    // MaxIOut
        gm6020_motors[i].horizon_ecd = 4096;    // 默认水平位置
        gm6020_motors[i].gravity_gain = 0.0f;    // 默认关闭
        
        //  上电机特殊处理
        if (i == 3) { // ID=4
            gm6020_motors[i].gravity_gain = 0.0f; // 强制禁用
        }
        
        float speed_dead_zone = 1.0f;  // 速度环死区：1 RPM
        float angle_dead_zone = 0.5f; 
        //  优化PID参数（上电机）
        if (i == 3) { // ID=4
            PID_Init(&gm6020_motors[i].speed_pid, 
                     140.0f,   // Kp: 80→50（降低增益）
                     5.0f,    // Ki: 3.0→2.0（减少积分）
                     0.1f,   // Kd: 0.2→0.05（避免过大）
                     GM6020_MAX_CURRENT,
                     5000.0f, // 积分限幅
                     speed_dead_zone); // ✅ 设置死区
            
            PID_Init(&gm6020_motors[i].angle_pid, 
                     7.0f,    // Kp: 5.0→3.0
                     0.0f,    
                     0.0f,
                     320.0f,  // MaxOut: 320→100 RPM（限制最大速度）
                     0.0f,
                     angle_dead_zone); // ✅ 设置死区
        } 
        // 下电机保持原有参数
        else {
            PID_Init(&gm6020_motors[i].speed_pid, 60.0f, 3.3f, 0.005f, GM6020_MAX_CURRENT, 5000.0f,0.0f);
        }
        // [删除] 绝对不要初始化 current_pid
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
    
    // 如果处于堵转保护状态，拒绝新的控制指令，除非手动清除
    //if (motor->is_stalled) return; 
    Motor_Mode_e old_mode = motor->mode;
    motor->mode = mode;
    motor->target_val = target;
    
    // 切换模式时清除 PID 积分，防止飞车
    if (old_mode != mode) {
        PID_Clear(&motor->speed_pid);
        PID_Clear(&motor->angle_pid);
    }
}

void Motor_Control_Loop(void) {
    for (int i = 0; i < 4; i++) {
        GM6020_TypeDef *motor = &gm6020_motors[i];
        float speed_target = 0;
        float current_target = 0; // 这是最终要发给电机的电流值
        
        // 堵转保护
        if (motor->is_stalled || motor->mode == MOTOR_MODE_STOP) {
            motor->output_current = 0;
            motor_current_send_buf[i] = 0;
            continue;
        }

        switch (motor->mode) {
            case MOTOR_MODE_CURRENT_OPEN:
                // 模式0：电流开环
                current_target = motor->target_val;
                break;
                
            case MOTOR_MODE_CURRENT_SPEED:
                // 模式1：速度闭环
                // 速度环 PID 输出的直接就是目标电流
                current_target = PID_Calc(&motor->speed_pid, motor->target_val, motor->speed_rpm);
                // [删除] 不要再过 current_pid
                break;
                
            case MOTOR_MODE_CURRENT_POS_CASCADE:
                // 模式2：串级闭环
                // 外环：位置 -> 速度
                speed_target = PID_Calc(&motor->angle_pid, motor->target_val, motor->total_angle);
                // 内环：速度 -> 电流
                current_target = PID_Calc(&motor->speed_pid, speed_target, motor->speed_rpm);
                // [删除] 不要再过 current_pid
                break;
                
            default:
                current_target = 0;
                break;
        }

         if (motor->mode == MOTOR_MODE_CURRENT_SPEED || 
            motor->mode == MOTOR_MODE_CURRENT_POS_CASCADE) {
            
            motor->feedforward = Motor_Calc_Gravity_FF(motor);
            current_target += motor->feedforward;
        }

        // 1. 限制电流范围
        motor->output_current = limit_current(current_target);
        
        // 2. 执行堵转检测
        Motor_Check_Stall(motor);
        
        // 3. 填入发送缓冲区
        motor_current_send_buf[i] = motor->output_current;
    }
    
    // 发送 ID 0x1FE
    BSP_CAN_Send_Current_Cmd();
}

void Motor_Set_Dual_Target(uint8_t id1, Motor_Mode_e mode1, float target1,
                           uint8_t id2, Motor_Mode_e mode2, float target2) {
    Motor_Set_Target(id1, mode1, target1);
    Motor_Set_Target(id2, mode2, target2);
}