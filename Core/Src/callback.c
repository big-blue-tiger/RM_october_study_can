//
// Created by 28672 on 24-10-14.
//

#include "main.h"
#include "stm32f4xx_hal_can.h"
// Created by 28672 on 24-10-14.
//
uint8_t RxData[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1);
}
