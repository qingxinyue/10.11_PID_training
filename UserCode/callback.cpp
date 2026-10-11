
//
// Created by qingx on 2026/10/3.
//
#include "main.h"
#include "can_user.h"
#include "can.h"
#include "Motor.hpp"
volatile uint32_t stop_flag=0;

PID ppid(15.0f, 0.0f, 0.1f, 0.0f, 100.0f, 0.05f);
PID spid(130.0f, 0.0f, 0.1f,0.0f, 10000.0f, 0.05f);
motor motor_g(motor::M2006, 36.0f, motor::POSITION_SPEED, ppid, spid,4,10000.0f);

void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef *hcan)
{
    uint8_t data[8];
    if (hcan!=&hcan1) return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data)!=HAL_OK)
    {
        return;
    }
    if (rx_header.IDE == CAN_ID_STD &&
        rx_header.RTR == CAN_RTR_DATA &&
        rx_header.DLC == 8 &&
        rx_header.StdId == 0x204)
    {
        motor_g.canRxMsgCallback(data);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6)
    {
        return;
    }

    if (stop_flag !=0)
    {
        motor_g.setEnabled(false);
    }else
    {
        motor_g.setEnabled(true);
        motor_g.handle();
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header,motor_g.getTxData(), &can_tx_mailbox);
    }
}

extern "C" void motor_setPosition(float angle)
{
    motor_g.setPosition(angle,0.0f,0.0f);
}
extern "C" void motor_setSpeed(float speed)
{
    motor_g.setSpeed(speed,0.0f);
}
extern "C" void motor_setEnabled(int enabled)
{
    motor_g.setEnabled(enabled!=0);
}
extern "C" void motor_setIntensity(float intensity)
{
    motor_g.setIntensity(intensity);
}