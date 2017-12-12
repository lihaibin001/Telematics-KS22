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

#include "rtc.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
const char *to_send = "uart send test!\r\n";

uint8_t recv_buffer[4];

/* Task priorities. */


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void LED_Toggle_Task(void *pvParameters);
void Time_Show(void *pvParameters);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    rtc_datetime_t dateTime;
    BOARD_BootClockHSRUN();
    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;

    Uart_InitIO(0);
    DbgConsole_Init((uint32_t) UART0, 9600, DEBUG_CONSOLE_DEVICE_TYPE_UART, CLOCK_GetCoreSysClkFreq());
    PRINTF("RTC Test\r\n");
    PRINTF("Enter the data and time\r\ndata segments by space\r\nexample:2017 5 10 10 15 45\r\n");
   // SCANF("%d, %d, %d, %d, %d, %d", 
    //      &dateTime.year, &dateTime.month, &dateTime.day, &dateTime.hour, &dateTime.minute, &dateTime.second);
    RTC_Initialize();
    RTC_StartTimer(RTC);
    RTC_SetTimeCount(RTC_ConvertDatetimeToSeconds(&dateTime));
    xTaskCreate(Time_Show, "Time_Show", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(LED_Toggle_Task, "LED_Toggle_Task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
}

void Time_Show(void *pvParameters)
{
    portTickType xDelay = pdMS_TO_TICKS(1000);
    for(;;)
    {
        PRINTF("%08x\r", RTC->TSR);
        vTaskDelay(xDelay);
    }
}

void LED_Toggle_Task(void *pvParameters)
{
    /* power control port */
    const gpio_pin_config_t LED_1_config = {kGPIO_DigitalOutput, 0,};
    const gpio_pin_config_t LED_2_config = {kGPIO_DigitalOutput, 0,};
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
