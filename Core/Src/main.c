/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "pid.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bsp.h"
#include "motor_control.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOWER_MOTOR_ID    3
#define UPPER_MOTOR_ID    4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ANGLE_ACCUMULATION_THRESHOLD 2340.0f  // 累计角度阈值 (6.5圈)
#define LOWER_MOTOR_SPEED  73.0f    // 恒定转速 (RPM)下面电机的移动速度，建议是180.0f
#define SMOOTH_TRANSITION_TIME 500   // 平滑过渡时间 (ms)
#define UPPER_MOTOR_SPEED 37.0f   // 上电机目标速度(这里要么给0要么给37)
#define KEY_DEBOUNCE_TIME  50       // 按键消抖时间 (ms)
#define POSITION_RETURN_SPEED 120.0f // 归位速度 (RPM)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

extern GM6020_TypeDef gm6020_motors[4];
/* USER CODE BEGIN PV */

// 按键引脚定义 (根据实际硬件修改)
#define KEY_MODE_PIN GPIO_PIN_2    // PB2
#define KEY_MODE_GPIO GPIOB 

// 控制模式枚举
typedef enum {
    MODE_BOTH_MOVING,     // 0: 上动下动
    MODE_UPPER_MOVING,    // 1: 上动下不动 (下电机归位)
    MODE_LOWER_MOVING,    // 2: 上不动下动
    MODE_BOTH_STOPPED     // 3: 上下都不动
} ControlMode;

// 电机状态枚举
typedef enum {
    STATE_NORMAL,         // 正常运行
    STATE_RETURNING       // 返回初始位置中
} MotorState;

// 系统状态
ControlMode current_mode = MODE_BOTH_MOVING; // 初始模式
ControlMode target_mode = MODE_BOTH_MOVING;  // 目标模式
uint8_t last_mode = MODE_BOTH_MOVING;        // 上次模式
uint32_t last_key_press = 0;                  // 上次按键时间
uint32_t mode_transition_start = 0;          // 模式切换开始时间
const uint32_t MODE_TRANSITION_TIME = 200;   // 200ms平滑过渡
MotorState lower_motor_state = STATE_NORMAL; // 下电机状态
void Check_Key_Inputs(void);
void Prepare_Mode_Transition(ControlMode new_mode);
void Process_Mode_Transition(void);
void Configure_Motors_For_Mode(ControlMode mode);
void Update_Lower_Motor_Control(void);
void Return_Lower_Motor_To_Initial_Position(void);
void Brake_Upper_Motor(void);
void Reset_Upper_Motor_Position(void);
// 位置控制
float lower_motor_initial_angle = 0.0f;      // 下电机初始角度
float lower_motor_target_angle = 0.0f;       // 下电机目标角度
float lower_motor_speed = LOWER_MOTOR_SPEED; // 下电机速度

// 状态机变量 (下电机角度累计)
typedef enum {
    ACCUM_POS,     // 正向累计
    TRANS_NEG,     // 过渡到负向
    ACCUM_NEG,     // 负向累计
    TRANS_POS      // 过渡到正向
} LowerMotorState;
LowerMotorState lower_state = ACCUM_POS;
float accumulated_angle = 0.0f;
uint32_t transition_start_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Update_Lower_Motor_Control(void);
void Check_Key_Inputs(void);
void Return_Lower_Motor_To_Initial_Position(void);
void Brake_Upper_Motor(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief 初始化按键GPIO
  */


/**
  * @brief 检测按键输入
  */
/**
  * @brief 下电机返回初始位置
  */
void Return_Lower_Motor_To_Initial_Position(void) {
    if (lower_motor_state == STATE_NORMAL) {
        lower_motor_state = STATE_RETURNING;
        lower_motor_target_angle = lower_motor_initial_angle;
        
        //启用位置控制模式（而不是速度模式）
        Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_POS_CASCADE, lower_motor_target_angle);
        
        printf("Returning lower motor to initial position: %.1f°\n", lower_motor_initial_angle);
        
        //重置PID状态，防止积分饱和
        GM6020_TypeDef *motor = &gm6020_motors[LOWER_MOTOR_ID-1];
        PID_Clear(&motor->angle_pid);
    }
}
/* USER CODE BEGIN PD */
#define UPPER_MOTOR_RETURN_POSITION 0.0f    // 上电机归位角度 (可根据实际调整)
#define UPPER_MOTOR_POS_KP 8.0f             // 位置环P参数 (增强抗干扰)
#define UPPER_MOTOR_POS_KI 0.0f
#define UPPER_MOTOR_POS_KD 0.0f
#define UPPER_MOTOR_POS_MAX_SPEED 100.0f    // 位置环最大速度
/* USER CODE END PD */

/* USER CODE BEGIN PV */
// 新增上电机状态
float upper_motor_target_angle = 0.0f;     // 上电机目标角度
uint8_t upper_motor_needs_reset = 0;       // 是否需要复位标志
/* USER CODE END PV */
// 全局状态变量

void Check_Key_Inputs(void) {
    uint32_t current_time = HAL_GetTick();
    
    if (HAL_GPIO_ReadPin(KEY_MODE_GPIO, KEY_MODE_PIN) == GPIO_PIN_SET) {
        if (current_time - last_key_press > KEY_DEBOUNCE_TIME) {
            last_key_press = current_time;
            
            // 仅在状态稳定时接受新指令
            if (current_mode == target_mode) {
                target_mode = (ControlMode)((current_mode + 1) % 4);
                mode_transition_start = current_time;
                printf("=== REQUESTING MODE CHANGE: %d ===\n", target_mode);
                
                // 准备模式切换
                Prepare_Mode_Transition(target_mode);
            }
        }
    }
}

void Prepare_Mode_Transition(ControlMode new_mode) {
    GM6020_TypeDef *lower_motor = &gm6020_motors[LOWER_MOTOR_ID-1];
    GM6020_TypeDef *upper_motor = &gm6020_motors[UPPER_MOTOR_ID-1];
    
    switch (new_mode) {
        case MODE_BOTH_MOVING:
            // 确保两电机都在速度模式
            if (lower_motor_state == STATE_RETURNING) {
                lower_motor_state = STATE_NORMAL;
            }
            upper_motor_needs_reset = 1;
            break;
            
        case MODE_UPPER_MOVING:
            // 下电机归位，上电机速度控制
            if (lower_motor_state != STATE_RETURNING) {
                Return_Lower_Motor_To_Initial_Position();
            }
            Motor_Set_Target(UPPER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, UPPER_MOTOR_SPEED);
            break;
            
        case MODE_LOWER_MOVING:
            // 上电机制动，下电机速度控制
            Brake_Upper_Motor();
            Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, LOWER_MOTOR_SPEED);
            break;
            
        case MODE_BOTH_STOPPED:
            // 两电机都进入保持模式
            Brake_Upper_Motor();
            if (lower_motor_state != STATE_RETURNING) {
                Return_Lower_Motor_To_Initial_Position();
            }
            break;
    }
}

// 在主循环中调用
void Process_Mode_Transition(void) {
    uint32_t current_time = HAL_GetTick();
    
    if (current_mode != target_mode && 
        current_time - mode_transition_start > MODE_TRANSITION_TIME) {
        
        // 确认模式切换
        current_mode = target_mode;
        printf("=== MODE TRANSITION COMPLETE: %d ===\n", current_mode);
        
        // 根据模式设置电机控制
        Configure_Motors_For_Mode(current_mode);
    }
}

void Configure_Motors_For_Mode(ControlMode mode) {
    switch (mode) {
        case MODE_BOTH_MOVING:
            Motor_Set_Target(UPPER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, UPPER_MOTOR_SPEED);
            Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, LOWER_MOTOR_SPEED);
            break;
            
        case MODE_UPPER_MOVING:
            // 下电机制动保持位置
            if (lower_motor_state == STATE_NORMAL) {
                Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_OPEN, 400.0f); // 400mA制动力
            }
            Motor_Set_Target(UPPER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, UPPER_MOTOR_SPEED);
            break;
            
        case MODE_LOWER_MOVING:
            // 上电机进入位置保持模式
            Reset_Upper_Motor_Position();
            Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, LOWER_MOTOR_SPEED);
            break;
            
        case MODE_BOTH_STOPPED:
            Reset_Upper_Motor_Position();
            if (lower_motor_state == STATE_NORMAL) {
                Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_OPEN, 400.0f); // 400mA制动力
            }
            break;
    }
}
/* USER CODE BEGIN PFP */
void Reset_Upper_Motor_Position(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * @brief 上电机位置复位 (优化版)
  * 当给0速度时，使用位置闭环控制，即使被打中也会自动回到目标位置
  */
 
void Reset_Upper_Motor_Position(void) {
    GM6020_TypeDef *upper_motor = &gm6020_motors[UPPER_MOTOR_ID-1];
    
    // 保存当前角度作为目标位置 (只在第一次进入时保存)
    if (upper_motor_needs_reset) {
        upper_motor_target_angle = upper_motor->total_angle;
        upper_motor_needs_reset = 0;
        printf("Upper motor reset position saved: %.1f°\n", upper_motor_target_angle);
        
    // 位置环PID参数优化 (增强抗干扰能力)
    PID_Init(&upper_motor->angle_pid, 
            3.0f,    // ✅ Kp从8.0→3.0，减少超调
            0.02f,   // ✅ Ki从0.0→0.02，消除静差
            0.1f,    // ✅ Kd=0.1，抑制振荡
            50.0f,   // ✅ MaxSpeed从100→50，限制最大速度
            1000.0f, // ✅ 积分限幅
            2.0f); // 0.5°死区
    }
    
     // ✅ 智能死区处理
    float angle_error = upper_motor_target_angle - upper_motor->total_angle;
    if (fabsf(angle_error) < 2.0f) {
        // 在死区内，使用小电流保持位置
        if (upper_motor->mode != MOTOR_MODE_CURRENT_OPEN) {
            // 逐渐降低到保持电流
            static float brake_current = 200.0f; // 200mA保持力
            Motor_Set_Target(UPPER_MOTOR_ID, MOTOR_MODE_CURRENT_OPEN, brake_current);
        }
    } else {
        // 使用位置闭环控制
        Motor_Set_Target(UPPER_MOTOR_ID, MOTOR_MODE_CURRENT_POS_CASCADE, upper_motor_target_angle);
    }
    static uint32_t last_debug = 0;
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_debug > 500) {
        last_debug = current_time;
        float angle_error = upper_motor_target_angle - upper_motor->total_angle;
        printf("[Upper] RESET MODE: Target=%.1f°, Current=%.1f°, Error=%.1f°\n",
               upper_motor_target_angle, upper_motor->total_angle, angle_error);
    }
}

/**
  * @brief 上电机制动停止 (改为智能复位)
  */
void Brake_Upper_Motor(void) {upper_motor_needs_reset = 1;
    printf("Upper motor entering stable brake mode\n");
    
    GM6020_TypeDef *upper_motor = &gm6020_motors[UPPER_MOTOR_ID-1];
    if (upper_motor->mode != MOTOR_MODE_CURRENT_OPEN) {
        // 渐进式制动，防止冲击
        float brake_current = 300.0f; // 300mA制动力
        Motor_Set_Target(UPPER_MOTOR_ID, MOTOR_MODE_CURRENT_OPEN, brake_current);
        
        PID_Clear(&upper_motor->angle_pid);
    }
}
/* USER CODE END 0 */

/**
  * @brief 更新下电机控制
  */
void Update_Lower_Motor_Control(void) {
    uint32_t current_time = HAL_GetTick();
    GM6020_TypeDef *lower_motor = &gm6020_motors[LOWER_MOTOR_ID-1];
    float target_speed = 0.0f;
    
    accumulated_angle = lower_motor->total_angle;
    
    // 根据状态机更新控制
    if (lower_motor_state == STATE_NORMAL) {
        switch (lower_state) {
            case ACCUM_POS:
                target_speed = LOWER_MOTOR_SPEED;
                if (accumulated_angle >= ANGLE_ACCUMULATION_THRESHOLD) {
                    lower_state = TRANS_NEG;
                    transition_start_time = current_time;
                    printf("=== SWITCHING TO NEGATIVE TRANSITION ===\n");
                }
                break;
                
            case TRANS_NEG:
                if (current_time - transition_start_time < SMOOTH_TRANSITION_TIME) {
                    float ratio = (float)(current_time - transition_start_time) / SMOOTH_TRANSITION_TIME;
                    target_speed = LOWER_MOTOR_SPEED * (1.0f - 2.0f * ratio);
                } else {
                    lower_state = ACCUM_NEG;
                    accumulated_angle = 0.0f;
                    printf("=== TRANSITION TO NEGATIVE COMPLETE ===\n");
                }
                break;
                
            case ACCUM_NEG:
                target_speed = -LOWER_MOTOR_SPEED;
                if (accumulated_angle <= -ANGLE_ACCUMULATION_THRESHOLD) {
                    lower_state = TRANS_POS;
                    transition_start_time = current_time;
                    printf("=== SWITCHING TO POSITIVE TRANSITION ===\n");
                }
                break;
                
            case TRANS_POS:
                if (current_time - transition_start_time < SMOOTH_TRANSITION_TIME) {
                    float ratio = (float)(current_time - transition_start_time) / SMOOTH_TRANSITION_TIME;
                    target_speed = -LOWER_MOTOR_SPEED * (1.0f - 2.0f * ratio);
                } else {
                    lower_state = ACCUM_POS;
                    accumulated_angle = 0.0f;
                    printf("=== TRANSITION TO POSITIVE COMPLETE ===\n");
                }
                break;
        }
    } 
    // 返回初始位置模式
    else if (lower_motor_state == STATE_RETURNING) {
        // 计算角度误差
        float angle_error = lower_motor_target_angle - lower_motor->total_angle;
        float abs_error = fabsf(angle_error);
        
        const float DECELERATION_ZONE = 30.0f; // 30°减速区
        const float STOP_THRESHOLD = 2.0f;    // 2°停止阈值
        // 如果接近目标位置，停止返回
        if (abs_error < STOP_THRESHOLD) {
            // ✅ 到达目标位置，平滑停止
            lower_motor_state = STATE_NORMAL;
            lower_state = ACCUM_POS;
            accumulated_angle = 0.0f;
            target_speed = 0.0f;
            printf("=== RETURN TO INITIAL POSITION COMPLETE ===\n");
            
            // 根据当前模式设置下电机状态
            if (current_mode == MODE_UPPER_MOVING || current_mode == MODE_BOTH_STOPPED) {
                // 精确位置保持
                Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_POS_CASCADE, lower_motor_target_angle);
            } else {
                // 恢复正常运行
                Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, LOWER_MOTOR_SPEED);
            }
        } 
        // 否则，向目标位置移动
        else {
            // ✅ 使用PID控制代替固定速度
            if (abs_error > DECELERATION_ZONE) {
                // 远距离：最大速度
                target_speed = (angle_error > 0) ? POSITION_RETURN_SPEED : -POSITION_RETURN_SPEED;
                Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, target_speed);
            } else {
                // ✅ 减速区：使用位置控制
                float speed_target = PID_Calc(&lower_motor->angle_pid, 
                                             lower_motor_target_angle, 
                                             lower_motor->total_angle);
                Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, speed_target);
            }
        }
    }
    
    // 始终显示当前状态
    static uint32_t last_debug = 0;
    if (current_time - last_debug > 200) {
        last_debug = current_time;
        if (lower_motor_state == STATE_NORMAL) {
            printf("[Lower] Mode=%d, State=%d, Angle=%.1f°, Speed=%.1f RPM\n",
                   current_mode, lower_state, accumulated_angle, target_speed);
        } else {
            printf("[Lower] RETURNING: Current=%.1f°, Target=%.1f°, Error=%.1f°\n",
                   lower_motor->total_angle, lower_motor_target_angle, 
                   lower_motor_target_angle - lower_motor->total_angle);
        }
    }
    
    // 设置下电机目标
    if (lower_motor_state == STATE_NORMAL) {
        Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, target_speed);
    } else {
        // 位置模式需要串级控制
        Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_POS_CASCADE, lower_motor_target_angle);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    
    /* USER CODE BEGIN 2 */
    // 初始化按键
    
    Motor_Init_All();
    BSP_CAN_Init();
    
    // 保存下电机初始位置
    lower_motor_initial_angle = gm6020_motors[LOWER_MOTOR_ID-1].total_angle;
    printf("Lower motor initial angle: %.1f°\n", lower_motor_initial_angle);
    
    // 初始化电机
    Motor_Set_Target(UPPER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, UPPER_MOTOR_SPEED);
    Motor_Set_Target(LOWER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, LOWER_MOTOR_SPEED);
    
    // 重力补偿参数 (上电机)
    gm6020_motors[3].horizon_ecd = 4096;
    gm6020_motors[3].gravity_gain = 0.0f;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        static uint32_t last_update = 0;
        if (HAL_GetTick() - last_update >= 5) {
            last_update = HAL_GetTick();
            
            // 1. 检测按键输入
            Check_Key_Inputs();
            
            // 2. 处理模式过渡
            Process_Mode_Transition();
            
            // 3. 更新下电机控制
            if (current_mode == MODE_BOTH_MOVING || 
                current_mode == MODE_LOWER_MOVING) {
                Update_Lower_Motor_Control();
            }
            
            // 4. 更新上电机控制
            if (current_mode == MODE_BOTH_MOVING || 
                current_mode == MODE_UPPER_MOVING) {
                Motor_Set_Target(UPPER_MOTOR_ID, MOTOR_MODE_CURRENT_SPEED, UPPER_MOTOR_SPEED);
            } else {
                Reset_Upper_Motor_Position();
            }
            
            // 5. 执行电机控制
            Motor_Control_Loop();
        }
        HAL_Delay(1);
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */