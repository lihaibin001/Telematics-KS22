#include "uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void LED_Toggle_Task(void *pvParameters);
void WakeupEnable(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static void CNA_Disable(void)
{
    const gpio_pin_config_t CAN_Control_config = {kGPIO_DigitalOutput, 0,};
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 0U, &CAN_Control_config);
    GPIO_WritePinOutput(GPIOC, 0U, 1);
}
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */

    BOARD_BootClockHSRUN();
    CNA_Disable();
    WakeupEnable();
    xTaskCreate(LED_Toggle_Task, "uart_idle_task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
    for (;;)
    { 
    }
}

void LED_Toggle_Task(void *pvParameters)
{
    /* power control port */
    const gpio_pin_config_t LED_1_config = {kGPIO_DigitalOutput, 0,};
    const gpio_pin_config_t LED_2_config = {kGPIO_DigitalOutput, 1,};
    portTickType xDelay = pdMS_TO_TICKS(500);
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
    }
}

void WakeupEnable(void)
{
    port_pin_config_t wakeup_pin_config;
    wakeup_pin_config.pullSelect = kPORT_PullDisable;
    wakeup_pin_config.slewRate = kPORT_SlowSlewRate;
    wakeup_pin_config.passiveFilterEnable = kPORT_PassiveFilterDisable;
    wakeup_pin_config.driveStrength = kPORT_LowDriveStrength;
    wakeup_pin_config.lockRegister = kPORT_UnlockRegister;
    wakeup_pin_config.mux = kPORT_MuxAsGpio;
    wakeup_pin_config.openDrainEnable = kPORT_OpenDrainDisable;
    
    CLOCK_EnableClock(kCLOCK_PortA);
    PORT_SetPinConfig(PORTA, 4U, &wakeup_pin_config); 
    const gpio_pin_config_t wakeup_pin_set = {kGPIO_DigitalInput, 1,};
    PORT_SetPinMux(PORTA, 4U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 4U, &wakeup_pin_set); 
}