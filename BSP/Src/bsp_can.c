#include "bsp_can.h"
#include "fdcan.h"
#include "motor.h"
#include <stdint.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;


static inline int16_t  FLoatToInt16(float val, float scale)
{
    float scaled = val * scale;
    if (scaled > 32767.0f) {
        return 32767;
    } else if (scaled < -32768.0f) {
        return -32768;
    }    
    return (int16_t)(scaled);

}

void BSP_CAN_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_ID_CONTROL_CMD;
    sFilterConfig.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);


    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

}


void BSP_CAN_Send (uint32_t id, const uint8_t *data, uint8_t len)
{
    if (data == NULL ||  len > 8) return;

    FDCAN_TxHeaderTypeDef txHeader = {0};
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = len << 16;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;


    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

void BSP_CAN_SendFloats(uint32_t id, const float *values, uint8_t count, float scale)
{
    if (values == NULL || count == 0 || count > 4) return;

    uint8_t buf[8] = {0};
    for (uint8_t i = 0; i< count; i++)
    {
        int v = FLoatToInt16(values[i], scale);
        buf[i*2] = (uint8_t)(v &  0xFF);
        buf[i*2+1] = (uint8_t)((v>>8) & 0xFF);


    }
    BSP_CAN_Send(id, buf, count*2);
}

void BSP_CAN_SendMotorState(void)
{
    {
        float values1[4] = {
            motor.speed,   /* 机械角速度, SCALE=100 → 精度 0.01 rad/s */
            motor.ia,      /* A 相电流,  SCALE=1000 → 精度 1mA */
            motor.ib,      /* B 相电流 */
            motor.ic       /* C 相电流 */
        };
        BSP_CAN_SendFloats(CAN_ID_MOTOR_STATE1, values1, 4,
                           CAN_SCALE_SPEED);
    }
    
    {
        float values2[4] = {
            motor.id,      /* d 轴电流 */
            motor.iq,      /* q 轴电流 */
            motor.vd,      /* d 轴电压 */
            motor.vq       /* q 轴电压 */
        };
        BSP_CAN_SendFloats(CAN_ID_MOTOR_STATE2, values2, 4,
                           CAN_SCALE_CURRENT);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0_ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader = {0};
    uint8_t               rxData[8] = {0};

    /* 从 RX FIFO 0 取出消息 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                               &rxHeader, rxData) != HAL_OK)
    {
        return;  /* 取消息失败, 忽略 */
    }
    uint32_t dlc_bytes = rxHeader.DataLength >> 16;

    switch (rxHeader.Identifier)
    {
        case CAN_ID_CONTROL_CMD:  /* 0x201: 控制命令 */
        {
            if (dlc_bytes < 5) break;  /* 至少需要 5 字节 */

            /* 解析 int16 字段 (小端) */
            int16_t id_raw = (int16_t)(rxData[0] | (rxData[1] << 8));
            int16_t iq_raw = (int16_t)(rxData[2] | (rxData[3] << 8));
            //uint8_t mode   = rxData[4];

            /* 转换为浮点并更新电机参考值 */
            motor.id_ref = (float)id_raw / CAN_SCALE_CURRENT;
            motor.iq_ref = (float)iq_raw / CAN_SCALE_CURRENT;

            /* mode 预留: 0=停止, 1=开环, 2=电流闭环, 3=速度闭环... */
            /* if (mode == 0) { ... } */

            break;
        }

        default:
            /* 未知 ID, 忽略 */
            break;
    }
}