#include "main.h"
#include "math.h"
#include <string.h>

/* ================= EXTERN ================= */
extern UART_HandleTypeDef huart4;
extern usartData uart4_Rx;
extern machine_control mCtr;
extern UART_DATA uartdata;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;

/* ================= DEFINE ================= */
/* 13 byte payload:
   0  : ID
   1  : enum_mState
   2  : enum_ctrMode
   3  : enum_ctrState
   4  : V_b
   5  : I_b
   6  : SOC_b
   7  : I_Bsp
   8  : VdcLink
   9  : V_sc
   10 : I_sc
   11 : SOC_sc
   12 : I_csp
*/
#define UART_FRAME_SIZE 13U

/* ================= BUFFER ================= */
uint8_t TX_Data[UART_FRAME_SIZE];
uint8_t RX_Buffer_UART4[UART_FRAME_SIZE];

/* ================= FLAG ================= */
volatile uint8_t uart4_tx_done_flag = 1U;
volatile uint8_t uart4_rx_done_flag = 0U;

static volatile uint8_t uart4_tx_busy = 0U;
static volatile uint8_t uart4_rx_busy = 0U;

/* ================= INIT ================= */
void vUsart_DataInit(usartData *pData)
{
    pData->rxPointer = 0U;
    pData->RxFlag    = 0U;
    pData->timeOut   = 0U;
    pData->STATE     = 0U;

    memset(pData->rxBuff, 0, sizeof(pData->rxBuff));
    memset(pData->txBuff, 0, sizeof(pData->txBuff));
}

/* ================= UPDATE TX DATA ================= */
void update_data_TX(machine_control *mctr, UART_DATA *data)
{
    data->ID            = 1;
    data->enum_ctrMode  = mctr->enum_ctrMode;
    data->enum_ctrState = mctr->enum_ctrState;
    data->enum_mState   = mctr->enum_mState;

//    data->V_b     = 13.0f;
//    data->V_sc    = 5.0f;
//    data->I_b     = 5.0f;
//    data->I_sc    = mctr->f_Ic_fb;
//    data->SOC_b   = 50.0f;
//    data->SOC_sc  = 60.0f;
//    data->I_Bsp   = 16.0f;
//    data->I_csp   = mctr->f_Ic_sp;
//    data->VdcLink = 30.0f;
}

/* ================= PACK TX FRAME ================= */
static void UART4_PackFrame(UART_DATA *data)
{
    TX_Data[0]  = data->ID;
    TX_Data[1]  = data->enum_mState;
    TX_Data[2]  = data->enum_ctrMode;
    TX_Data[3]  = data->enum_ctrState;

    TX_Data[4]  = (uint8_t)(data->V_b * 255.0f / 25.0f);
    TX_Data[5]  = (uint8_t)((data->I_b + 30.0f) * 255.0f / 60.0f);
    TX_Data[6]  = (uint8_t)(data->SOC_b * 255.0f / 100.0f);
    TX_Data[7]  = (uint8_t)((data->I_Bsp + 30.0f) * 255.0f / 60.0f);

    TX_Data[8]  = (uint8_t)(data->VdcLink * 255.0f / 70.0f);
    TX_Data[9]  = (uint8_t)(data->V_sc * 255.0f / 25.0f);
    TX_Data[10] = (uint8_t)((data->I_sc + 30.0f) * 255.0f / 60.0f);
    TX_Data[11] = (uint8_t)(data->SOC_sc * 255.0f / 100.0f);
    TX_Data[12] = (uint8_t)((data->I_csp + 30.0f) * 255.0f / 60.0f);
}

/* ================= RX PARSE ================= */
void update_data_RX(usartData *pRxData, machine_control *mctr)
{
    mctr->f_Vinc  = pRxData->rxBuff[9]  * 25.0f / 255.0f;
    mctr->f_Ic_fb = pRxData->rxBuff[10] * 60.0f / 255.0f - 30.0f;
}
/* ================= RX START ================= */
HAL_StatusTypeDef UART4_StartRxDMA(void)
{
    HAL_StatusTypeDef ret;

    if (uart4_rx_busy)
    {
        return HAL_BUSY;
    }

    uart4_rx_busy = 1U;
    uart4_rx_done_flag = 0U;
    uart4_Rx.RxFlag = 0U;

    ret = HAL_UART_Receive_DMA(&huart4, RX_Buffer_UART4, UART_FRAME_SIZE);

    if (ret != HAL_OK)
    {
        uart4_rx_busy = 0U;
    }

    return ret;
}

/* ================= TX START ================= */
HAL_StatusTypeDef USER_UART_TxData(UART_DATA *data)
{
    HAL_StatusTypeDef ret;

    if (uart4_tx_busy)
    {
        return HAL_BUSY;
    }

    UART4_PackFrame(data);


    uart4_tx_busy = 1U;
    uart4_tx_done_flag = 0U;

    ret = HAL_UART_Transmit_DMA(&huart4, TX_Data, UART_FRAME_SIZE);

    if (ret != HAL_OK)
    {
        uart4_tx_busy = 0U;
        uart4_tx_done_flag = 1U;
        HAL_GPIO_WritePin(RS485_nRW_GPIO_Port, RS485_nRW_Pin, GPIO_PIN_RESET);
    }

    return ret;
}

/* ================= RX HANDLE IN MAIN ================= */
void USER_UART_RxDataHandle(void)
{
    if (uart4_rx_done_flag == 0U)
    {
        return;
    }

    update_data_RX(&uart4_Rx, &mCtr);

    uart4_Rx.RxFlag = 0U;
    uart4_rx_done_flag = 0U;

    /* x? l� xong m?i cho nh?n ti?p */
    UART4_StartRxDMA();
}

/* ================= CALLBACK ================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        memcpy(uart4_Rx.rxBuff, RX_Buffer_UART4, UART_FRAME_SIZE);
        uart4_Rx.rxPointer = UART_FRAME_SIZE;
        uart4_Rx.RxFlag = 1U;

        uart4_rx_busy = 0U;
        uart4_rx_done_flag = 1U;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        uart4_tx_busy = 0U;
        uart4_tx_done_flag = 1U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        uart4_tx_busy = 0U;
        uart4_rx_busy = 0U;
        uart4_tx_done_flag = 1U;
        uart4_rx_done_flag = 0U;

        vUsart_DataInit(&uart4_Rx);
        UART4_StartRxDMA();
    }
}

/* ================= DMA INIT ================= */
void UART4_DMA_Init(void)
{
    vUsart_DataInit(&uart4_Rx);

    uart4_tx_busy = 0U;
    uart4_rx_busy = 0U;
    uart4_tx_done_flag = 1U;
    uart4_rx_done_flag = 0U;

    UART4_StartRxDMA();
}
