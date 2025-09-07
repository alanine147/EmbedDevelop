/************************************************************************
 * @file    cdd_log.c
 * @author  gcg
 * @brief   api implement of output log
 * @date    2025-09-06
 ************************************************************************/
#include <stdarg.h>
#include "cdd_log.h"
#include "common.h"
#include "cdd_rb.h"
#include "real_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
/************************************************************************
 * Local Macro Definitions
 ************************************************************************/
#define CDD_LOG_UART_STRING_BUFFER_SIZE     (100U)
#define FORMAT_FLAG_LEFT_JUSTIFY            (1u << 0)
#define FORMAT_FLAG_PAD_ZERO                (1u << 1)
#define FORMAT_FLAG_PRINT_SIGN              (1u << 2)
#define FORMAT_FLAG_ALTERNATE               (1u << 3)
#define XUARTPS_FIFO_NUMBER                 (32u)



#define CDD_LOG_ENTER_CRITICAL()    taskENTER_CRITICAL()
#define CDD_LOG_EXIT_CRITICAL()     taskEXIT_CRITICAL()
/************************************************************************
 * Local Typedefs
 ************************************************************************/

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
static uint8 cdd_log_uartStringBuffer[CDD_LOG_UART_STRING_BUFFER_SIZE] = {0U};
static CDD_LOG dataCurrent = {0u};
/************************************************************************
 * Local constants
 ************************************************************************/
const CDD_LOG_Cfg cdd_log_cfg =
{
    CDD_RB_BUFFER_LOGGER_UART,
    CDD_LOG_UART_STRING_BUFFER_SIZE,
    cdd_log_uartStringBuffer
};

/************************************************************************
 * Global function declarations
 ************************************************************************/
/************************************************************************
 * Local function declarations
 ************************************************************************/
static void CDD_LOG_Vprintf(const char *format, va_list *param);
static void CDD_LOG_StoreChar(char c);
static void CDD_LOG_PrintUnsigned(uint32 value, uint8 base, uint8 numDigits, uint8 fieldWidth, uint8 formatFlags);
static void CDD_LOG_PrintInt(sint32 value, uint8 base, uint8 numDigits, uint8 fieldWidth, uint8 formatFlags);

static boolean CDD_LOG_TriggerUartTransmit(void);
/************************************************************************
 * Function implementation
 ************************************************************************/

void CDD_LOG_String(const char *fmt, ...)
{
    va_list vp;
    RealClockTimeType currentTime = TimeNowGet();
    CDD_LOG_Printf("[%d.%d]", currentTime.sec, currentTime.microSec);
    va_start(vp, fmt);
    CDD_LOG_Vprintf(fmt, &vp);
    CDD_LOG_Printf("\r\n");
    va_end(vp);
}

void CDD_LOG_Printf(const char *fmt, ...)
{
    va_list vp;

    CDD_LOG_ENTER_CRITICAL();
    va_start(vp, fmt);
    CDD_LOG_Vprintf(fmt, &vp);
    va_end(vp);
    CDD_LOG_EXIT_CRITICAL();
}

int CDD_LOG_Printf_Ext(const char *fmt, ...)
{
    va_list vp;

    CDD_LOG_ENTER_CRITICAL();
    va_start(vp, fmt);
    CDD_LOG_Vprintf(fmt, &vp);
    va_end(vp);
    CDD_LOG_EXIT_CRITICAL();
    return 0;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    
}


static void CDD_LOG_Vprintf(const char *format, va_list *param)
{
    char c;
    sint32 value;
    uint8 numDigits;
    uint8 formatFlags;
    uint8 fieldWidth;

    do
    {
        c = *format;
        format++;
        if (c == 0)
        {
            break;
        }
        if (c == '%')
        {
            formatFlags = 0u;
            value = 1;
            do
            {
                c = *format;
                switch (c)
                {
                    case '-':
                    {
                        formatFlags |= FORMAT_FLAG_LEFT_JUSTIFY;
                        format++;
                    }
                    break;

                    case '0':
                    {
                        formatFlags |= FORMAT_FLAG_PAD_ZERO;
                        format++;
                    }
                    break;

                    case '+':
                    {
                        formatFlags |= FORMAT_FLAG_PRINT_SIGN;
                        format++;
                    }
                    break;

                    case '#':
                    {
                        formatFlags |= FORMAT_FLAG_ALTERNATE;
                        format++;
                    }
                    break;

                    default:
                    {
                        value = 0;
                    }
                    break;
                }
            }while (value);

            fieldWidth = 0u;
            do
            {
                c = *format;
                if ((c < '0') || (c > '9'))
                {
                    break;
                }
                format++;
                fieldWidth = (fieldWidth * 10) + ((unsigned)c - '0');
            } while (1);

            numDigits = 0u;
            c = *format;
            if (c == '.')
            {
                format++;
                do
                {
                    c = *format;
                    if ((c < '0') || (c > '9'))
                    {
                        break;
                    }
                    format++;
                    numDigits = numDigits * 10u + ((unsigned)c - '0');
                } while (1);
            }

            c = *format;
            do
            {
                if ((c == 'l') || (c == 'h'))
                {
                    c = *format;
                    format++;
                }
                else
                {
                    break;
                }
            } while (1);

            switch (c)
            {
                case 'c':
                {
                    char c0;
                    value = va_arg(*param, int);
                    c0 = (char)value;
                    CDD_LOG_StoreChar(c0);
                }
                break;

                case 'd':
                {
                    value = va_arg(*param, int);
                    CDD_LOG_PrintInt(value, 10u, numDigits, fieldWidth, formatFlags);
                }
                break;

                case 'F':
                case 'f':
                {
                    double f = va_arg(*param, double);
                    uint8 i;

                    value = (sint32)f;
                    CDD_LOG_PrintInt(value, 10u, 0u, fieldWidth, formatFlags);
                    CDD_LOG_StoreChar('.');
                    if(numDigits == 0u)
                        numDigits = 6u;
                    for(i = 0u; i < numDigits; i++)
                    {
                        f = (f - value) * 10.0f;
                        value = (sint32)f;
                        if(value < 0.0f)
                        {
                            value = -value;
                        }
                        CDD_LOG_PrintInt(value, 10u, 0u, fieldWidth, formatFlags);
                    }
                }
                break;

                case 'u':
                {
                    value = va_arg(*param, int);
                    CDD_LOG_PrintUnsigned((uint32)value, 10u, numDigits, fieldWidth, formatFlags);
                    break;
                }

                case 'x':
                case 'X':
                {
                    value = va_arg(*param, int);
                    CDD_LOG_PrintUnsigned((uint32)value, 16u, numDigits, fieldWidth, formatFlags);
                }
                break;

                case 's':
                {
                    const char * s = va_arg(*param, const char *);
                    do
                    {
                        c = *s;
                        s++;
                        if (c == '\0')
                        {
                            break;
                        }
                        CDD_LOG_StoreChar(c);
                    } while(1);
                }
                break;

                case 'p':
                {
                    value = va_arg(*param, int);
                    CDD_LOG_PrintUnsigned((uint32)value, 16u, 8u, 8u, 0u);
                }
                break;

                case '%':
                {
                    CDD_LOG_StoreChar('%');
                }
                break;

                default:
                break;
            }
            format++;
        }
        else
        {
            CDD_LOG_StoreChar(c);
        }
    }while (1);

    if (dataCurrent.bufferUsed > 0U)
	{
		(void)CDD_RB_Push(cdd_log_cfg.rbUart, cdd_log_cfg.buffer, dataCurrent.bufferUsed);
		dataCurrent.bufferUsed = 0U;
	}
}

static void CDD_LOG_StoreChar(char c)
{
    if (dataCurrent.bufferUsed >= cdd_log_cfg.bufferSize)
    {
        if (E_OK == CDD_RB_Push(cdd_log_cfg.rbUart, cdd_log_cfg.buffer, cdd_log_cfg.bufferSize))
        {
            dataCurrent.bufferUsed = 0U;
        }
    }

    if(dataCurrent.bufferUsed < cdd_log_cfg.bufferSize)
    {
        cdd_log_cfg.buffer[dataCurrent.bufferUsed] = c;
        dataCurrent.bufferUsed++;
    }
}

static void CDD_LOG_PrintUnsigned(uint32 value, uint8 base, uint8 numDigits, uint8 fieldWidth, uint8 formatFlags)
{
    static const char table[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
    uint32 div;
    uint32 digit;
    uint32 number;
    uint32 width;
    char c;

    number = value;
    digit = 1u;
    width = 1u;
    while (number >= base)
    {
        number = (number / base);
        width++;
    }
    if (numDigits > width)
    {
        width = numDigits;
    }

    if ((formatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u)
    {
        if (fieldWidth != 0u)
        {
            if (((formatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && (numDigits == 0))
            {
                c = '0';
            }
            else
            {
                c = ' ';
            }
            while ((fieldWidth != 0u) && (width < fieldWidth))
            {
                fieldWidth--;
                CDD_LOG_StoreChar(c);
            }
        }
    }

    while (1) 
    {
        if (numDigits > 1u)
        {
            numDigits--;
        }
        else
        {
            div = value / digit;
            if (div < base)
            {
                break;
            }
        }
        digit *= base;
    }

    do
    {
        div = value / digit;
        value -= div * digit;
        CDD_LOG_StoreChar(table[div]);
        digit /= base;
    } while (digit);

    if ((formatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == FORMAT_FLAG_LEFT_JUSTIFY)
    {
        if (fieldWidth != 0u)
        {
            while ((fieldWidth != 0u) && (width < fieldWidth))
            {
                fieldWidth--;
                CDD_LOG_StoreChar(' ');
            }
        }
    }
}

static void CDD_LOG_PrintInt(sint32 value, uint8 base, uint8 numDigits, uint8 fieldWidth, uint8 formatFlags)
{
    uint8 width;
    uint32 number;

    if(value < 0)
    {
        number = -value;
    }
    else
    {
        number = value;
    }

    width = 1u;
    while (number >= (uint32)base)
    {
        number = (number / (uint32)base);
        width++;
    }

    if (numDigits > width)
    {
        width = numDigits;
    }

    if ((fieldWidth > 0u) && ((value < 0) || ((formatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN)))
    {
        fieldWidth--;
    }

    if ((((formatFlags & FORMAT_FLAG_PAD_ZERO) == 0u) || (numDigits != 0u)) && ((formatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u))
    {
        if (fieldWidth != 0u)
        {
            while ((fieldWidth != 0u) && (width < fieldWidth))
            {
                fieldWidth--;
                CDD_LOG_StoreChar(' ');
            }
        }
    }

    if(value < 0)
    {
        number = -value;
    }
    else
    {
        number = value;
    }

    if (value < 0)
    {
        CDD_LOG_StoreChar('-');
    }
    else if ((formatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN)
    {
        CDD_LOG_StoreChar('+');
    }

    if (((formatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && ((formatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0) && (numDigits == 0))
    {
        if (fieldWidth != 0u)
        {
            while ((fieldWidth != 0u) && (width < fieldWidth))
            {
                fieldWidth--;
                CDD_LOG_StoreChar('0');
            }
        }
    }
    CDD_LOG_PrintUnsigned(number, base, numDigits, fieldWidth, formatFlags);
}

void CDD_LOG_MainFunction(void)
{
    CDD_LOG_TriggerUartTransmit();
}
static boolean CDD_LOG_TriggerUartTransmit(void)
{
    uint32 rbUartLen = 0U;
    boolean ret = FALSE;
    static uint8 data[XUARTPS_FIFO_NUMBER];

    if (E_OK == CDD_RB_NumberGet(cdd_log_cfg.rbUart, &rbUartLen))
    {
        if (rbUartLen > 0U)
        {
            if (rbUartLen > 32)
            {
                rbUartLen = 32;
            }
            if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
            {
                if (E_OK == CDD_RB_Pop(cdd_log_cfg.rbUart, data, rbUartLen))
                {
                    HAL_UART_Transmit_IT(&huart1, data, rbUartLen);
                    ret = TRUE;
                }
            }
        }
    }
    return ret;
}

/*****************************[End of file]*****************************/