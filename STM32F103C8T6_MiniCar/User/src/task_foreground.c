/************************************************************************
 * @file    task_foreground.c
 * @author  Your Name
 * @brief   Brief description of the file.
 * @date    2025-09-06
 ************************************************************************/

#include "task_foreground.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "event_groups.h"
#include "common.h"
#include "cdd_log.h"
#include "cdd_cli.h"

/************************************************************************
 * Local Macro Definitions
 ************************************************************************/
#define EVENT_FGD_1MS   (0x1U)
#define EVENT_FGD_2MS   (0x2U)
#define EVENT_FGD_5MS   (0x4U)
#define EVENT_FGD_10MS  (0x8U)
#define EVENT_FGD_500MS (0x10U)
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
static TimerHandle_t timerHandlerFgd[5];
static EventGroupHandle_t eventGroupHandlerFgd;
static StaticTimer_t timerFgd1ms;
static StaticTimer_t timerFgd2ms;
static StaticTimer_t timerFgd5ms;
static StaticTimer_t timerFgd10ms;
static StaticTimer_t timerFgd500ms;
StaticEventGroup_t eventGroupFgd;
/************************************************************************
 * Local constants
 ************************************************************************/

/************************************************************************
 * Global function declarations
 ************************************************************************/

/************************************************************************
 * Local function declarations
 ************************************************************************/
static void TimerHandlerFgd1ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerFgd, 0x1U);
}

static void TimerHandlerFgd2ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerFgd, 0x2U);
}

static void TimerHandlerFgd5ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerFgd, 0x4U);
}

static void TimerHandlerFgd10ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerFgd, 0x8U);
}

static void TimerHandlerFgd500ms(TimerHandle_t xTimer)
{
    xEventGroupSetBits(eventGroupHandlerFgd, 0x10U);
}
 /************************************************************************
 * Function implement
 ************************************************************************/

void TaskForegroundEntry(void const * argument)
{
    sys_log("Start to initialize background task");
    timerHandlerFgd[0] = xTimerCreateStatic(" 1ms", 1U, (UBaseType_t)pdTRUE, (void *)1U, TimerHandlerFgd1ms, &timerFgd1ms);
    timerHandlerFgd[1] = xTimerCreateStatic(" 2ms", 2U, (UBaseType_t)pdTRUE, (void *)2U, TimerHandlerFgd2ms, &timerFgd2ms);
    timerHandlerFgd[2] = xTimerCreateStatic(" 5ms", 5U, (UBaseType_t)pdTRUE, (void *)5U, TimerHandlerFgd5ms, &timerFgd5ms);
    timerHandlerFgd[3] = xTimerCreateStatic(" 10ms", 10U, (UBaseType_t)pdTRUE, (void *)10U, TimerHandlerFgd10ms, &timerFgd10ms);
    timerHandlerFgd[4] = xTimerCreateStatic(" 500ms", 500U, (UBaseType_t)pdTRUE, (void *)500U, TimerHandlerFgd500ms, &timerFgd500ms);

    eventGroupHandlerFgd = xEventGroupCreateStatic(&eventGroupFgd);

    for (uint32 i = 0U; i < 5U; i++)
    {
        xTimerStart(timerHandlerFgd[i], 0U);
    }

    sys_log("Finish to initialize Task");

    while (1)
    {
        EventBits_t event = xEventGroupWaitBits(eventGroupHandlerFgd, EVENT_FGD_1MS | EVENT_FGD_2MS | EVENT_FGD_5MS | EVENT_FGD_10MS | EVENT_FGD_500MS, pdTRUE, pdFALSE, 100U);
        if ((event & EVENT_FGD_1MS) != 0x00U)
        {
            /* 1ms runnable */
            CDD_CLI_Mainfunction();
        }

        if ((event & EVENT_FGD_2MS) != 0x00U)
        {
            /* 2ms runnable */

        }

        if ((event & EVENT_FGD_5MS) != 0x00U)
        {
            /* 5ms runnable */
        }

        if ((event & EVENT_FGD_10MS) != 0x00U)
        {
            /* 10ms runnable */
        }

        if ((event & EVENT_FGD_500MS) != 0x00U)
        {
            /*500ms runnable*/
        }
    }
}



/*****************************[End of file]*****************************/