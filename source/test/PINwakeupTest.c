#include "relays.h"
#include "lowpower.h"
#include "Vehicle.h"
#include "fsl_rcm.h"
#include "relays.h"
#include <FreeRTOS.h>
#include "fsl_pmc.h"

void LED_Toggle_Task(void *pvParameters);

static void delay()
{
    int i;
    for(i = 0; i < 0x2FFFFFF; i++);
}
int main(void)
{
    
//    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    BOARD_BootClockHSRUN();
    uint32_t resetSource = RCM_GetPreviousResetSources(RCM);    
    Uart_Initialize(0);
    PMS_SetWakeupConfig(kAPP_PowerModeVlps);
    UART_Transmit(0, "\r\npower on\r\n", strlen("\r\npower on\r\n")); 
#if 0
    if((resetSource & kRCM_SourceSw) != 1)
    {
        delay();
        UART_Transmit(0, "\r\nsleep\r\n", strlen("\r\nsleep\r\n"));
        delay();
        Micro_Go_Low_Power();       
    }
#endif
    xTaskCreate(LED_Toggle_Task, 
                "LED_Toggle_Task", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                configMAX_PRIORITIES - 1, 
                NULL);
    
    vTaskStartScheduler();
    for(;;)
    {}
}

void LED_Toggle_Task(void *pvParameters)
{
    /* power control port */
    const gpio_pin_config_t LED_1_config = {kGPIO_DigitalOutput, 0,};
    const gpio_pin_config_t LED_2_config = {kGPIO_DigitalOutput, 0,};
    portTickType xDelay = pdMS_TO_TICKS(250);
    CLOCK_EnableClock(kCLOCK_PortC);

    PORT_SetPinMux(PORTC, 10U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 11U, kPORT_MuxAsGpio);

    GPIO_PinInit(GPIOC, 10U, &LED_1_config);
    GPIO_PinInit(GPIOC, 11U, &LED_2_config);
    for(;;)
    {
        vTaskDelay(xDelay);
        GPIO_WritePinOutput(GPIOC, 10U, 1);
        GPIO_WritePinOutput(GPIOC, 11U, 0);
        vTaskDelay(xDelay);
        GPIO_WritePinOutput(GPIOC, 10U, 0);
        GPIO_WritePinOutput(GPIOC, 11U, 1);
        VHCL_set_ACC_Sta(VHCL_get_ACC_LineLevel());
        VHCL_set_BMS_Sta(VHCL_get_BMS_LineLevel());
    }
}
#if 0
void PORTC_IRQHandler(void)
{
    uint32_t IT_Mask = 0;
    IT_Mask = GPIO_GetPinsInterruptFlags(GPIOC);
    wakeup = 1; 
//    UART_Transmit(0, "-", 1);
    GPIO_ClearPinsInterruptFlags(GPIOC, IT_Mask);
}
#endif
