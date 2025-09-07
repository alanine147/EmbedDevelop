/************************************************************************
 * @file    real_clock.c
 * @author  gcg
 * @brief   api for real clock
 * @date    2025-09-06
 ************************************************************************/

#include "real_clock.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_rtc.h"
#include "common.h"

/************************************************************************
 * Local Macro Definitions
 ************************************************************************/

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

/************************************************************************
 * Local constants
 ************************************************************************/

/************************************************************************
 * Global function declarations
 ************************************************************************/

/************************************************************************
 * Local function declarations
 ************************************************************************/
static inline uint32_t RTC_ReadTimeCounter(RTC_TypeDef *RTCx);
/************************************************************************
 * Function implementation
 ************************************************************************/

RealClockTimeType TimeNowGet(void)
{
    RealClockTimeType timeCur;
    timeCur.microSec = (LSE_VALUE - LL_RTC_GetDivider(hrtc.Instance)) * 1000 / LSE_VALUE;
    timeCur.sec = RTC_ReadTimeCounter(hrtc.Instance);
    return timeCur;
}

static inline uint32_t RTC_ReadTimeCounter(RTC_TypeDef *RTCx)
{
  uint16_t high1 = 0U, high2 = 0U, low = 0U;
  uint32_t timecounter = 0U;

  high1 = READ_REG(RTCx->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(RTCx->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(RTCx->CNTH & RTC_CNTH_RTC_CNT);

  if (high1 != high2)
  {
    /* In this case the counter roll over during reading of CNTL and CNTH registers,
       read again CNTL register then return the counter value */
    timecounter = (((uint32_t) high2 << 16U) | READ_REG(RTCx->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  {
    /* No counter roll over during reading of CNTL and CNTH registers, counter
       value is equal to first value of CNTL and CNTH */
    timecounter = (((uint32_t) high1 << 16U) | low);
  }

  return timecounter;
}


/*****************************[End of file]*****************************/