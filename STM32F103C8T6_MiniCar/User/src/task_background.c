/************************************************************************
 * @file    task_background.c
 * @author  gcg
 * @brief   task definitions of background
 * @date    2025-09-07
 ************************************************************************/

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "event_groups.h"
#include "common.h"
#include "cdd_log.h"
#include "task_background.h"
#include "led.h"
/************************************************************************
 * Local Macro Definitions
 ************************************************************************/
#define EVENT_BGD_1MS   (0x1U)
#define EVENT_BGD_2MS   (0x2U)
#define EVENT_BGD_5MS   (0x4U)
#define EVENT_BGD_10MS  (0x8U)
#define EVENT_BGD_500MS (0x10U)
/************************************************************************
 * Local Typedefs
 ************************************************************************/

/************************************************************************
 * Global Variable Definitions
 ************************************************************************/
StaticEventGroup_t eventGroupBgd;

/************************************************************************
 * Global constants
 ************************************************************************/

/************************************************************************
 * Local Variable Definitions
 ************************************************************************/
static TimerHandle_t timerHandlerBgd[5];
static EventGroupHandle_t eventGroupHandlerBgd;
static StaticTimer_t timerBgd1ms;
static StaticTimer_t timerBgd2ms;
static StaticTimer_t timerBgd5ms;
static StaticTimer_t timerBgd10ms;
static StaticTimer_t timerBgd500ms;
/************************************************************************
 * Local constants
 ************************************************************************/

/************************************************************************
 * Global function declarations
 ************************************************************************/

/************************************************************************
 * Local function declarations
 ************************************************************************/
static void TimerHandlerBgd1ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerBgd, 0x1U);
}

static void TimerHandlerBgd2ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerBgd, 0x2U);
}

static void TimerHandlerBgd5ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerBgd, 0x4U);
}

static void TimerHandlerBgd10ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerBgd, 0x8U);
}

static void TimerHandlerBgd500ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerBgd, 0x10U);
}
/************************************************************************
 * Function implementation
 ************************************************************************/
void TaskBackgroundEntry(void const * argument)
{
    sys_log("Start to initialize foreground task");
    timerHandlerBgd[0] = xTimerCreateStatic(" 1ms", 1U, (UBaseType_t)pdTRUE, (void *)1U, TimerHandlerBgd1ms, &timerBgd1ms);
    timerHandlerBgd[1] = xTimerCreateStatic(" 2ms", 2U, (UBaseType_t)pdTRUE, (void *)2U, TimerHandlerBgd2ms, &timerBgd2ms);
    timerHandlerBgd[2] = xTimerCreateStatic(" 5ms", 5U, (UBaseType_t)pdTRUE, (void *)5U, TimerHandlerBgd5ms, &timerBgd5ms);
    timerHandlerBgd[3] = xTimerCreateStatic(" 10ms", 10U, (UBaseType_t)pdTRUE, (void *)10U, TimerHandlerBgd10ms, &timerBgd10ms);
    timerHandlerBgd[4] = xTimerCreateStatic(" 500ms", 500U, (UBaseType_t)pdTRUE, (void *)500U, TimerHandlerBgd500ms, &timerBgd500ms);

    eventGroupHandlerBgd = xEventGroupCreateStatic(&eventGroupBgd);

    for (uint32 i = 0U; i < 5U; i++)
    {
        xTimerStart(timerHandlerBgd[i], 0U);
    }

    sys_log("Finish to initialize Task");

    while (1)
    {
        EventBits_t event = xEventGroupWaitBits(eventGroupHandlerBgd, EVENT_BGD_1MS | EVENT_BGD_2MS | EVENT_BGD_5MS | EVENT_BGD_10MS | EVENT_BGD_500MS, pdTRUE, pdFALSE, 100U);
        if ((event & EVENT_BGD_1MS) != 0x00U)
        {
            /* 1ms runnable */
        }

        if ((event & EVENT_BGD_2MS) != 0x00U)
        {
            /* 2ms runnable */

        }

        if ((event & EVENT_BGD_5MS) != 0x00U)
        {
            /* 5ms runnable */
            CDD_LOG_MainFunction();
        }

        if ((event & EVENT_BGD_10MS) != 0x00U)
        {
            /* 10ms runnable */
        }

        if ((event & EVENT_BGD_500MS) != 0x00U)
        {
            /*500ms runnable*/
            Led_System_Running();
        }
    }
}


/*****************************[End of file]*****************************/