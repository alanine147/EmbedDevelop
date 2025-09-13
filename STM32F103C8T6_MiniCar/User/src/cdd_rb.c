/************************************************************************
 * @file    cdd_rb.c
 * @author  gcg
 * @brief   Brief description of the file.
 * @date    2025-09-06
 ************************************************************************/
#include <string.h>
#include "cdd_rb.h"
#include "FreeRTOS.h"
#include "task.h"
/************************************************************************
 * Local Macro Definitions
 ************************************************************************/
#define CDD_RB_LOGGER_UART_ELEMENT_NUMBER       (1U * 1024U)
#define CDD_RB_LOGGER_UART_ELEMENT_SIZE         (1U)
#define CDD_RB_LOGGER_UART_BUFFER_SIZE          (CDD_RB_LOGGER_UART_ELEMENT_NUMBER * CDD_RB_LOGGER_UART_ELEMENT_SIZE)


#define CDD_RB_ENTER_CRITICAL()     taskENTER_CRITICAL_FROM_ISR()
#define CDD_RB_EXIT_CRITICAL(x)      taskEXIT_CRITICAL_FROM_ISR(x)

/************************************************************************
 * Local Typedefs
 ************************************************************************/

/************************************************************************
 * Global Variable Definitions
 ************************************************************************/

/************************************************************************
 * Global constants
 ************************************************************************/

/************************************************************************
 * Local Variable Definitions
 ************************************************************************/
static uint8 cdd_rb_logger_tx_Buffer[CDD_RB_LOGGER_UART_BUFFER_SIZE] = {0U};
static uint8 cdd_rb_logger_rx_Buffer[CDD_RB_LOGGER_UART_BUFFER_SIZE] = {0U};

static CDD_RB_Buffer rb_buffers[CDD_RB_BUFFER_NUMBER] = {0U};

static const CDD_RB_BufferCfg rb_bufferCfgs[CDD_RB_BUFFER_NUMBER] =
{
    {
        cdd_rb_logger_tx_Buffer, 
        cdd_rb_logger_tx_Buffer + CDD_RB_LOGGER_UART_BUFFER_SIZE, 
        CDD_RB_LOGGER_UART_ELEMENT_SIZE, 
        CDD_RB_LOGGER_UART_ELEMENT_NUMBER, 
        TRUE, 
        TRUE
    },
    {
        cdd_rb_logger_rx_Buffer, 
        cdd_rb_logger_rx_Buffer + CDD_RB_LOGGER_UART_BUFFER_SIZE, 
        CDD_RB_LOGGER_UART_ELEMENT_SIZE, 
        CDD_RB_LOGGER_UART_ELEMENT_NUMBER, 
        TRUE, 
        TRUE
    },
};

/************************************************************************
 * Local constants
 ************************************************************************/

const CDD_RB_Cfg cdd_rb_cfg =
{
    CDD_RB_BUFFER_NUMBER,
    rb_bufferCfgs,
    rb_buffers
};
static const CDD_RB_Cfg *cfgCurrent = NULL_PTR;
/************************************************************************
 * Global function declarations
 ************************************************************************/

/************************************************************************
 * Local function declarations
 ************************************************************************/
static uint32 CDD_RB_PushInternal(uint8 id, uint8 *data, uint32 elementNumber);
static uint32 CDD_RB_PeekInternal(uint8 id, uint8 *data, uint32 elementNumber);
static uint32 CDD_RB_PopInternal(uint8 id, uint8 *data, uint32 elementNumber);
/************************************************************************
 * Function implementation
 ************************************************************************/
static CDD_RB dataCurrent = {0U};

Std_ReturnType CDD_RB_Init(const CDD_RB_Cfg *cfg)
{
    Std_ReturnType ret = E_OK;
    uint32 bufferLength;
    uint32 bufferLengthCal;

    if(NULL_PTR == cfg)
    {
        ret = E_NULL_POINTER;
    }
    else if(TRUE == dataCurrent.isInit)
    {
        ret = E_STATUS;
    }
    else
    {
        for (uint8 i = 0U; i < cfg->bufferNumber; i++)
        {
            if (E_OK == ret)
            {
                const CDD_RB_BufferCfg *bufferCfg = &cfg->bufferCfgs[i];

                bufferLength = (uint32)(bufferCfg->bufferOverflow) - (uint32)(bufferCfg->buffer);
                bufferLengthCal = (uint32)(bufferCfg->elementLength) * (uint32)(bufferCfg->elementNumber);
                if((bufferLength != bufferLengthCal) || (0U == bufferCfg->elementLength))
                {
                    ret = E_CONFIG;
                }
                else
                {
                    CDD_RB_Buffer *buffer = &cfg->buffers[i];

                    buffer->headIndex = 0U;
                    buffer->tailIndex = 0U;
                }
            }
        }

        if (E_OK == ret)
        {
            cfgCurrent = cfg;
            dataCurrent.isInit = TRUE;
        }
    }
    return ret;
}

Std_ReturnType CDD_RB_Push(uint8 id, uint8 *data, uint32 elementNumber)
{
    Std_ReturnType ret = E_OK;
    uint32 len;

    if (FALSE == dataCurrent.isInit)
    {
        ret = E_STATUS;
    }
    else if((id >= cfgCurrent->bufferNumber) || (data == NULL_PTR) || (0U == elementNumber))
    {
        ret = E_PARAMETER;
    }
    else 
    {
        UBaseType_t uxSavedInterruptStatus;
        uxSavedInterruptStatus = CDD_RB_ENTER_CRITICAL();
        len = CDD_RB_PushInternal(id, data, elementNumber);
        if(elementNumber != len)
        {
            ret = E_FULL;
        }
        CDD_RB_EXIT_CRITICAL(uxSavedInterruptStatus);
    }
    return ret;
}

Std_ReturnType CDD_RB_Peek(uint8 id, uint8 *data, uint32 elementNumber)
{
    Std_ReturnType ret = E_OK;
    uint32 len;

    if (FALSE == dataCurrent.isInit)
    {
        ret = E_STATUS;
    }
    else if((id >= cfgCurrent->bufferNumber) || (data == NULL_PTR) || (0U == elementNumber))
    {
        ret = E_PARAMETER;
    }
    else
    {
        UBaseType_t uxSavedInterruptStatus;
        uxSavedInterruptStatus = CDD_RB_ENTER_CRITICAL();
        len = CDD_RB_PeekInternal(id, data, elementNumber);
        if(0U == len)
        {
            ret = E_EMPTY;
        }
        CDD_RB_EXIT_CRITICAL(uxSavedInterruptStatus);
    }
    return ret;
}

Std_ReturnType CDD_RB_Pop(uint8 id, uint8 *data, uint32 elementNumber)
{
    Std_ReturnType ret = E_OK;
    uint32 len;

    if (FALSE == dataCurrent.isInit)
    {
        ret = E_STATUS;
    }
    else if((id >= cfgCurrent->bufferNumber) || (data == NULL_PTR) || (0U == elementNumber))
    {
        ret = E_PARAMETER;
    }
    else
    {
        UBaseType_t uxSavedInterruptStatus;
        uxSavedInterruptStatus = CDD_RB_ENTER_CRITICAL();
        len = CDD_RB_PopInternal(id, data, elementNumber);
        if(0U == len)
        {
            ret = E_EMPTY;
        }
        CDD_RB_EXIT_CRITICAL(uxSavedInterruptStatus);
    }
    return ret;
}

Std_ReturnType CDD_RB_NumberGet(uint8 id, uint32 *number)
{
    Std_ReturnType ret = E_OK;
    uint32 elementUsed;

    if (FALSE == dataCurrent.isInit)
    {
        ret = E_STATUS;
    }
    else if((id >= cfgCurrent->bufferNumber) || (NULL_PTR == number))
    {
        ret = E_PARAMETER;
    }
    else
    {
        const CDD_RB_BufferCfg *bufferCfg = &cfgCurrent->bufferCfgs[id];
        CDD_RB_Buffer *buffer = &cfgCurrent->buffers[id];

        if(buffer->headIndex >= buffer->tailIndex)
        {
            elementUsed = buffer->headIndex - buffer->tailIndex;
        }
        else
        {
            elementUsed = buffer->headIndex + (bufferCfg->elementNumber << 1) - buffer->tailIndex;
        }
        *number = elementUsed;
    }
    return ret;
}

static uint32 CDD_RB_PushInternal(uint8 id, uint8 *data, uint32 elementNumber)
{
    uint32 elementUsed;
    uint32 elementUnused;
    uint8 *headPosition;
    uint32 elementOperate;
    uint32 bytesWrite;
    uint32 bytesHeadToBottom;
    uint32 nextHead;
    const CDD_RB_BufferCfg *bufferCfg = &cfgCurrent->bufferCfgs[id];
    CDD_RB_Buffer *buffer = &cfgCurrent->buffers[id];
    
    if(buffer->headIndex >= buffer->tailIndex)
    {
        elementUsed = buffer->headIndex - buffer->tailIndex;
    }
    else
    {
        elementUsed = buffer->headIndex + (bufferCfg->elementNumber << 1) - buffer->tailIndex;
    }
    /* element space unused */
    elementUnused = bufferCfg->elementNumber - elementUsed;

    /* get write buffer address depend on head index */
    if(buffer->headIndex >= bufferCfg->elementNumber)
    {
        /* element full */
        headPosition = bufferCfg->buffer + (buffer->headIndex - bufferCfg->elementNumber) * bufferCfg->elementLength;
    }
    else
    {
        /* element not full */
        headPosition = bufferCfg->buffer + buffer->headIndex * bufferCfg->elementLength;
    }

    /* guarantee elementOperate < elementUnused */
    if(elementUnused >= elementNumber)
    {
        /* element not full */
        elementOperate = elementNumber;
    }
    else
    {
        /* element full */
        if(TRUE == bufferCfg->pushMethod)
        {
            /* continue operate */
            elementOperate = elementUnused;
        }
        else
        {
            /* quit */
            elementOperate = 0U;
        }
    }

    if(elementOperate > 0U)
    {
        /* operate accordding to element number */
        bytesWrite = elementOperate * bufferCfg->elementLength;
        /* buffer space unused  */
        bytesHeadToBottom = ((uint32)bufferCfg->bufferOverflow - (uint32)headPosition);

        if(bytesHeadToBottom <= bytesWrite)
        {
            /* operate unused bytes */
            memcpy(headPosition, data, bytesHeadToBottom);

            /* operate remainder bytes */
            memcpy(bufferCfg->buffer, data + bytesHeadToBottom, bytesWrite - bytesHeadToBottom);         
        }
        else
        {
            memcpy(headPosition, data, bytesWrite);
        }

        nextHead = buffer->headIndex + elementOperate;
        if(nextHead >= (bufferCfg->elementNumber << 1))
        {
            buffer->headIndex = nextHead - (bufferCfg->elementNumber << 1);
        }
        else
        {
            buffer->headIndex = nextHead;
        }
    }
    return elementOperate;
}

static uint32 CDD_RB_PeekInternal(uint8 id, uint8 *data, uint32 elementNumber)
{
    uint32 elementUsed;
    uint8 *tailPosition;
    uint32 bytesTailToBottom;
    uint32 bytesRead;
    uint32 elementOperate;
    const CDD_RB_BufferCfg *bufferCfg = &cfgCurrent->bufferCfgs[id];
    CDD_RB_Buffer *buffer = &cfgCurrent->buffers[id];

    if(buffer->headIndex >= buffer->tailIndex)
    {
        elementUsed = buffer->headIndex - buffer->tailIndex;
    }
    else
    {
        elementUsed = buffer->headIndex + (bufferCfg->elementNumber << 1) - buffer->tailIndex;
    }

    if(buffer->tailIndex >= bufferCfg->elementNumber)
    {
        tailPosition = bufferCfg->buffer + (buffer->tailIndex - bufferCfg->elementNumber) * bufferCfg->elementLength;
    }
    else
    {
        tailPosition = bufferCfg->buffer + buffer->tailIndex * bufferCfg->elementLength;
    }
    bytesTailToBottom = ((uint32)bufferCfg->bufferOverflow - (uint32)tailPosition);
    
    if(elementNumber <= elementUsed)
    {
        elementOperate = elementNumber;
    }
    else
    {
        elementOperate = elementUsed;
    }

    if(elementOperate > 0U)
    {
        bytesRead = elementOperate * bufferCfg->elementLength;
        if(bytesTailToBottom <= bytesRead)
        {
            memcpy(data, tailPosition, bytesTailToBottom);
            memcpy(data + bytesTailToBottom, bufferCfg->buffer, bytesRead - bytesTailToBottom);
        }
        else
        {
            memcpy(data, tailPosition, bytesRead);
        }
    }
    return elementOperate;
}

static uint32 CDD_RB_PopInternal(uint8 id, uint8 *data, uint32 elementNumber)
{
    const CDD_RB_BufferCfg *bufferCfg = &cfgCurrent->bufferCfgs[id];
    CDD_RB_Buffer *buffer = &cfgCurrent->buffers[id];
    uint32 elementUsed = (uint32)((sint32)buffer->headIndex - (sint32)buffer->tailIndex);
    uint8 *tailPosition;
    uint32 bytesTailToBottom;
    uint32 bytesRead;
    uint32 elementOperate;
    uint32 nextTail;

    /* Get element unused */
    if(buffer->headIndex >= buffer->tailIndex)
    {
        elementUsed = buffer->headIndex - buffer->tailIndex;
    }
    else
    {
        elementUsed = buffer->headIndex + (bufferCfg->elementNumber << 1) - buffer->tailIndex;
    }

    /* tailPosition always point to buffer */
    if(buffer->tailIndex >= bufferCfg->elementNumber)
    {
        tailPosition = bufferCfg->buffer + (buffer->tailIndex - bufferCfg->elementNumber) * bufferCfg->elementLength;
    }
    else
    {
        tailPosition = bufferCfg->buffer + buffer->tailIndex * bufferCfg->elementLength;
    }
    bytesTailToBottom = ((uint32)bufferCfg->bufferOverflow - (uint32)tailPosition);

    /* guarantee pop size <= usedElement size,valid data */
    if(elementUsed >= elementNumber)
    {
        elementOperate = elementNumber;
    }
    else
    {
        if(TRUE == bufferCfg->popMethod)
        {
            elementOperate = elementUsed;
        }
        else
        {
            elementOperate = 0U;
        }
    }

    if(elementOperate > 0U)
    {
        bytesRead = elementOperate * bufferCfg->elementLength;
        if(bytesTailToBottom <= bytesRead)
        {
            memcpy(data, tailPosition, bytesTailToBottom);
            memcpy(data + bytesTailToBottom, bufferCfg->buffer, bytesRead - bytesTailToBottom);
        }
        else
        {
            memcpy(data, tailPosition, bytesRead);
        }
        
        nextTail = buffer->tailIndex + elementOperate;
        if(nextTail >= (bufferCfg->elementNumber << 1))
        {
            buffer->tailIndex = nextTail - (bufferCfg->elementNumber << 1);
        }
        else
        {
            buffer->tailIndex = nextTail;
        }
    }
    return elementOperate;
}

Std_ReturnType CDD_RB_Clear(uint8 id)
{
    Std_ReturnType ret = E_OK;
    
    if (FALSE == dataCurrent.isInit)
    {
        ret = E_STATUS;
    }
    else if(id >= cfgCurrent->bufferNumber)
    {
        ret = E_PARAMETER;
    }
    else
    {
        CDD_RB_Buffer *buffer = &cfgCurrent->buffers[id];

        buffer->headIndex = 0U;
        buffer->tailIndex = 0U;
    }
    return ret;
}


/*****************************[End of file]*****************************/