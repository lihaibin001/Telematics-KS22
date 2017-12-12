#include "rtc.h"
#include "fsl_port.h"
/**********************************************************************
 * Function
 *********************************************************************/

static void BOARD_SetRtcClockSource(void);

/*******************************************************************************
*  Function: RTC_Initialize
*
*  Parameters: :none
*  Returns: none
*  Description: Initialze the RTC mode
*******************************************************************************/
void RTC_Initialize(void)
{
    rtc_config_t rtcConfig;
    RTC_GetDefaultConfig(&rtcConfig);
    rtcConfig.wakeupSelect = true;
    RTC_Init(RTC, &rtcConfig);
    /* Select RTC clock source */
    BOARD_SetRtcClockSource();
    RTC_StartTimer(RTC);
}

/*******************************************************************************
*  Function: RTC_Set
*
*  Parameters: :none
*  Returns: none
*  Description: Set the RTC counter
*******************************************************************************/
uint8_t RTC_SetTimeCount(uint32_t count)
{
    RTC_StopTimer(RTC);
    RTC->TSR = count;
    RTC_StartTimer(RTC);
    return 0;
}

uint8_t RTC_SetAlarmCount(uint32_t count)
{
    RTC->TAR = count;
    return 0;
}
/*******************************************************************************
*  Function: BOARD_SetRtcClockSource
*
*  Parameters: :none
*  Returns: none
*  Description: Select RTC clock source
*******************************************************************************/
void BOARD_SetRtcClockSource(void)
{
    /* Enable the RTC 32KHz oscillator */
    RTC->CR |= RTC_CR_OSCE_MASK;
}

uint8_t RTC_GetTime(uint8_t *rtc_time)
{
    rtc_datetime_t datetime;
    RTC_GetDatetime(RTC,&datetime);
    if (datetime.year > 2000)
    {
        *rtc_time=(uint8_t)((datetime.year - 2000) & 0xFF);
    }
    else
    {
        *rtc_time = (uint8_t )((datetime.year - 1900) & 0xFF);
    }
    *(rtc_time+1)=datetime.month;
    *(rtc_time+2)=datetime.day;
    *(rtc_time+3)=datetime.hour;
    *(rtc_time+4)=datetime.minute;
    *(rtc_time+5)=datetime.second;
    return 0;
}

uint32_t RTC_GetCounter(void)
{
    return RTC->TSR;
}

void RTC_EnableInterupt(void)
{
    PORT_SetPinMux(PORTE, 0u, kPORT_MuxAlt7);
    RTC_EnableInterrupts(RTC, kRTC_AlarmInterruptEnable);
    EnableIRQ(RTC_IRQn);
    
}