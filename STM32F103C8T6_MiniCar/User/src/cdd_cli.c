/************************************************************************
 * @file    cdd_cli.c
 * @author  gcg
 * @brief   api implement of cli
 * @date    2025-09-07
 ************************************************************************/

#include "cdd_cli.h"
#include "main.h"
#include "cdd_log.h"
#include "cdd_rb.h"

/************************************************************************
 * Local Macro Definitions
 ************************************************************************/
#define UART_RECEIVED_BUFF_SIZE     (64u)
/************************************************************************
 * Local Typedefs
 ************************************************************************/
typedef struct
{
    uint8 validContextPos;
    uint8 overflow;
    boolean completeLine;
    uint8 *cmdBuf;
} CDD_CLI;


/************************************************************************
 * Global Variable Definitions
 ************************************************************************/
extern UART_HandleTypeDef huart1;
/************************************************************************
 * Global constants
 ************************************************************************/

/************************************************************************
 * Local Variable Definitions
 ************************************************************************/
static uint8 dataReceived[UART_RECEIVED_BUFF_SIZE];
static uint8 commandBuf[UART_RECEIVED_BUFF_SIZE];
static CDD_CLI dataCurrent = 
{
    .completeLine = FALSE,
    .overflow = 0u,
    .validContextPos = 0,
    .cmdBuf = commandBuf,
};
/************************************************************************
 * Local constants
 ************************************************************************/
static const CDD_CLI_CFG cli_cfg = 
{
    .uart = &huart1,
    .buf = dataReceived,
    .rbId = CDD_RB_BUFFER_LOGGER_UART_RX,
};
/************************************************************************
 * Global function declarations
 ************************************************************************/

/************************************************************************
 * Local function declarations
 ************************************************************************/
static void CDD_CLI_ClearBuffer(void);
/************************************************************************
 * Function implementation
 ************************************************************************/

void CDD_CLI_Init(CDD_CLI_CFG *cfg)
{
    dataCurrent.validContextPos = 0u;
    HAL_UARTEx_ReceiveToIdle_IT(cli_cfg.uart, cli_cfg.buf, UART_RECEIVED_BUFF_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (E_OK == CDD_RB_Push(cli_cfg.rbId, cli_cfg.buf, UART_RECEIVED_BUFF_SIZE))
    {
        HAL_UARTEx_ReceiveToIdle_IT(cli_cfg.uart, cli_cfg.buf, UART_RECEIVED_BUFF_SIZE);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* Prevent unused argument(s) compilation warning */
    switch (huart->RxEventType)
    {
        case HAL_UART_RXEVENT_IDLE:
        {
            if(E_OK == CDD_RB_Push(cli_cfg.rbId, huart->pRxBuffPtr - Size, Size))
            {
                HAL_UARTEx_ReceiveToIdle_IT(cli_cfg.uart, cli_cfg.buf, UART_RECEIVED_BUFF_SIZE);
            }
    }
    
    default:
        break;
    }
}

void CDD_CLI_Mainfunction(void)
{
    uint8 dataGet[UART_RECEIVED_BUFF_SIZE];
    boolean processResult = TRUE;
    uint32 receivedLen = 0;
    do
    {
        if (E_OK != CDD_RB_NumberGet(cli_cfg.rbId, &receivedLen))
        {
            processResult = FALSE;
            break;
        }
        else
        {
            if (receivedLen > UART_RECEIVED_BUFF_SIZE)
            {
                receivedLen = UART_RECEIVED_BUFF_SIZE;
            }
        }
        
        if (E_OK != CDD_RB_Pop(cli_cfg.rbId, dataGet, receivedLen))
        {
            processResult = FALSE;
            break;
        }
        processResult = TRUE;
    } while (FALSE);
    if (TRUE == processResult)    
    {
        uint16 idx = 0;
        while (idx < receivedLen)
        {
            if ('\n' == dataGet[idx])
            {
                continue;
            }
            if ('\r' == dataGet[idx])
            {
                if (dataCurrent.validContextPos != 0)
                {
                    dataCurrent.completeLine = TRUE;
                }
                CDD_LOG_Printf("\r\n");
            }
            else if ('\b' == dataGet[idx])
            {
                if (dataCurrent.validContextPos > 0u)
                {
                    dataCurrent.validContextPos--;
                }
                if ('\n' == dataCurrent.cmdBuf[dataCurrent.validContextPos])
                {
                    if (dataCurrent.completeLine == TRUE)
                    {
                        dataCurrent.completeLine = FALSE;
                    }
                }
                dataCurrent.cmdBuf[dataCurrent.validContextPos] = '\0';
                uint8 delete = 0x7FU;
                CDD_LOG_Printf("%c", delete);
            }
            else if ((dataGet[idx] >= 0x20U) && (dataGet[idx] <= 0x7EU))
            {
                if (dataCurrent.validContextPos < UART_RECEIVED_BUFF_SIZE)
                {
                    dataCurrent.cmdBuf[dataCurrent.validContextPos] = dataGet[idx];
                    dataCurrent.validContextPos++;
                }
                CDD_LOG_Printf("%c", dataGet[idx]);
            }
            idx++;
        }

        if (TRUE == dataCurrent.completeLine)
        {
            CDD_CLI_ClearBuffer();
        }
    }
    
}

static void CDD_CLI_ClearBuffer(void)
{
    for (uint32 i = 0U; i < dataCurrent.validContextPos; i++)
    {
    	dataCurrent.cmdBuf[i] = '\0';
    }
    dataCurrent.validContextPos = 0U;
    dataCurrent.completeLine = FALSE;
}


/*****************************[End of file]*****************************/