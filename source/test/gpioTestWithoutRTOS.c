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
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void LED_Toggle_Task(void *pvParameters);
void test_pullupAllpin(void);
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
//    BOARD_InitPins();
    BOARD_BootClockHSRUN();
    Uart_Initialize(0);
//    test_pullupAllpin();
    LED_Toggle_Task(NULL);
    for (;;)
    { 
    }
}

void LED_Toggle_Task(void *pvParameters)
{
    /* power control port */
    const gpio_pin_config_t LED_1_config = {kGPIO_DigitalOutput, 0,};
    const gpio_pin_config_t LED_2_config = {kGPIO_DigitalOutput, 1,};
    uint32_t delay;
    portTickType xDelay = pdMS_TO_TICKS(500);
    CLOCK_EnableClock(kCLOCK_PortC);

    PORT_SetPinMux(PORTC, 10U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 11U, kPORT_MuxAsGpio);

    GPIO_PinInit(GPIOC, 10U, &LED_1_config);
    GPIO_PinInit(GPIOC, 11U, &LED_2_config);
    for(;;)
    {
        for(delay = 1; delay < 5000000; delay++)
        {
            UART_Transmit(0, 0x55, 1);
        }
        GPIO_WritePinOutput(GPIOC, 10U, 1);
        GPIO_WritePinOutput(GPIOC, 11U, 0);
        for(delay = 1; delay < 5000000; delay++)
        {
            UART_Transmit(0, 0x55, 1);
        }
        GPIO_WritePinOutput(GPIOC, 10U, 0);
        GPIO_WritePinOutput(GPIOC, 11U, 1);
    }
}


void test_pullupAllpin(void)
{
    /* power control port */
    const gpio_pin_config_t pin_config = {kGPIO_DigitalOutput, 1,};
    uint32_t delay;
//    portTickType xDelay = pdMS_TO_TICKS(500);
    CLOCK_EnableClock(kCLOCK_PortC);

    PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 1U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 2U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 3U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 4U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 5U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 6U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 7U, kPORT_MuxAsGpio);
    
    PORT_SetPinMux(PORTC, 8U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 9U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 12U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 13U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 14U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 15U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 0U, &pin_config);
    GPIO_PinInit(GPIOC, 1U, &pin_config);
    GPIO_PinInit(GPIOC, 2U, &pin_config);
    GPIO_PinInit(GPIOC, 3U, &pin_config);
    GPIO_PinInit(GPIOC, 4U, &pin_config);
    GPIO_PinInit(GPIOC, 5U, &pin_config);
    GPIO_PinInit(GPIOC, 6U, &pin_config);
    GPIO_PinInit(GPIOC, 7U, &pin_config);
    GPIO_PinInit(GPIOC, 8U, &pin_config);
    GPIO_PinInit(GPIOC, 9U, &pin_config);
    GPIO_PinInit(GPIOC, 12U, &pin_config);
    GPIO_PinInit(GPIOC, 13U, &pin_config);
    GPIO_PinInit(GPIOC, 14U, &pin_config);
    GPIO_PinInit(GPIOC, 15U, &pin_config);
    
    
  
}