#include "power_mode_switch.h"
#include "fsl_port.h"
#include "BMS_Detect.h"
#include "ACC_Detect.h"
#include "system.h"
void PORTC_IRQHandler(void)
{
    if ((1U << BAT_WAKEUP_GPIO_PIN) & PORT_GetPinsInterruptFlags(BAT_WAKEUP_PORT))
    {
        PORT_ClearPinsInterruptFlags(BAT_WAKEUP_PORT, 1U << BAT_WAKEUP_GPIO_PIN);
        if(0 == GPIO_ReadPinInput(ACC_GPIO,ACC_GPIO_PIN))
        {
            EB_OnSignal();
        }
        else
        {
            EB_OffSignal();
        }

    }
    if ((1U << ACC_GPIO_PIN) & PORT_GetPinsInterruptFlags(ACC_PORT))
    {
        PORT_ClearPinsInterruptFlags(ACC_PORT, 1U << ACC_GPIO_PIN);
        Sys_Set_ACC_Wakeup_Flag();
#if 0
        if(0 == GPIO_ReadPinInput(ACC_GPIO,ACC_GPIO_PIN))
        {
            ACC_OnSignal();
        }
        else
        {
            ACC_OffSignal();
        }
#endif
    }
    if ((1U << BMS_GPIO_PIN) & PORT_GetPinsInterruptFlags(BMS_PORT))
    {
        Sys_Set_BMS_Wakeup_Flag();
        PORT_ClearPinsInterruptFlags(BMS_PORT, 1U << BMS_GPIO_PIN);
        if(0 == GPIO_ReadPinInput(BMS_GPIO,BMS_GPIO_PIN))
        {
            BMS_OnSignal();
        }
        else
        {
            BMS_OffSignal();
        }
    }
}

/*!
 * @brief LLWU interrupt handler.
 */
void LLWU_IRQHandler(void)
{
    /* If wakeup by external pin. */
    if (LLWU_GetExternalWakeupPinFlag(LLWU, BAT_LLWU_WAKEUP_PIN_IDX))
    {
        PORT_SetPinInterruptConfig(BAT_WAKEUP_PORT, BAT_WAKEUP_GPIO_PIN, kPORT_InterruptOrDMADisabled);
        PORT_ClearPinsInterruptFlags(BAT_WAKEUP_PORT, (1U << BAT_WAKEUP_GPIO_PIN));
        LLWU_ClearExternalWakeupPinFlag(LLWU, BAT_LLWU_WAKEUP_PIN_IDX);
    }
        /* If wakeup by external pin. */
    if (LLWU_GetExternalWakeupPinFlag(LLWU, ACC_LLWU_WAKEUP_PIN_IDX))
    {
        PORT_SetPinInterruptConfig(ACC_WAKEUP_PORT, ACC_WAKEUP_GPIO_PIN, kPORT_InterruptOrDMADisabled);
        PORT_ClearPinsInterruptFlags(ACC_WAKEUP_PORT, (1U << ACC_WAKEUP_GPIO_PIN));
        LLWU_ClearExternalWakeupPinFlag(LLWU, ACC_LLWU_WAKEUP_PIN_IDX);
    }
        /* If wakeup by external pin. */
    if (LLWU_GetExternalWakeupPinFlag(LLWU, BMS_LLWU_WAKEUP_PIN_IDX))
    {
        PORT_SetPinInterruptConfig(BMS_WAKEUP_PORT, BMS_WAKEUP_GPIO_PIN, kPORT_InterruptOrDMADisabled);
        PORT_ClearPinsInterruptFlags(BMS_WAKEUP_PORT, (1U << BMS_WAKEUP_GPIO_PIN));
        LLWU_ClearExternalWakeupPinFlag(LLWU, BMS_LLWU_WAKEUP_PIN_IDX);
    }
    
}

void LPTMR0_IRQHandler(void)
{
}

void RTC_IRQHandler(void)
{
    RTC_ClearStatusFlags(RTC, RTC_SR_TAF_MASK);
}
