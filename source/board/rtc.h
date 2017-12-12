#ifndef __RTC_H__
#define __RTC_H__

#include "fsl_rtc.h"

/**********************************************************************
 * Function declaration
 *********************************************************************/
void RTC_Initialize(void);
uint8_t RTC_SetTimeCount(uint32_t count);
uint8_t RTC_SetAlarmCount(uint32_t count);
uint8_t RTC_GetTime(uint8_t *rtc_time);
uint32_t RTC_GetCounter(void);
void RTC_EnableInterupt(void);
#endif //__RTC_H__
