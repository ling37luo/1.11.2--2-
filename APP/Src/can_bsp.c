#include "can_bsp.h"
#include "motor_control.h"

// 全局电机发送缓冲区 (ID 1-4)
// 由 motor_control.c 直接更新此数组，CAN发送时直接读取
int16_t motor_current_send_buf[4] = {0}; 

void BSP_CAN_Init(void) {
    CAN_FilterTypeDef can_filter_st;
    
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


// CAN接收中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        
        // GM6020 反馈ID: 0x205(ID1) ~ 0x208(ID4)
        if (rx_header.StdId >= 0x205 && rx_header.StdId <= 0x208) {
            uint8_t motor_id = rx_header.StdId - 0x204; // 转换出 1-4
            Motor_Update_Feedback(motor_id, rx_data);
        }
        Motor_Control_Loop();
    }
}
void BSP_CAN_Send_Voltage_Cmd(void) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    
    // 配置为电压控制帧 ID (针对 ID 1-4)
    tx_header.StdId = 0x1FF; 
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    // 填充数据 (Big Endian)
    // 范围 -25000 ~ 25000
    tx_data[0] = (motor_voltage_send_buf[0] >> 8) & 0xFF; // ID 1
    tx_data[1] = motor_voltage_send_buf[0] & 0xFF;
    tx_data[2] = (motor_voltage_send_buf[1] >> 8) & 0xFF; // ID 2
    tx_data[3] = motor_voltage_send_buf[1] & 0xFF;
    tx_data[4] = (motor_voltage_send_buf[2] >> 8) & 0xFF; // ID 3
    tx_data[5] = motor_voltage_send_buf[2] & 0xFF;
    tx_data[6] = (motor_voltage_send_buf[3] >> 8) & 0xFF; // ID 4
    tx_data[7] = motor_voltage_send_buf[3] & 0xFF;
    
    // 检查是否有空闲邮箱，防止发送过快导致 HAL_ERROR
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    
}

void BSP_CAN_Send_Current_Cmd(void) {
    uint8_t tx_data[8];
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    tx_header.StdId = 0x1FE; 
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    // 电机 1
    tx_data[0] = (motor_current_send_buf[0] >> 8) & 0xFF;
    tx_data[1] = motor_current_send_buf[0] & 0xFF;
    // 电机 2
    tx_data[2] = (motor_current_send_buf[1] >> 8) & 0xFF;
    tx_data[3] = motor_current_send_buf[1] & 0xFF;
    // 电机 3
    tx_data[4] = (motor_current_send_buf[2] >> 8) & 0xFF;
    tx_data[5] = motor_current_send_buf[2] & 0xFF;
    // 电机 4
    tx_data[6] = (motor_current_send_buf[3] >> 8) & 0xFF;
    tx_data[7] = motor_current_send_buf[3] & 0xFF;
    
    // 发送标准帧，ID = 0x1FE (GM6020 电流控制 ID)
    // 注意：请确保你的 CAN 句柄和 Mailbox 配置正确
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}