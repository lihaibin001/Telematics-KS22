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
const char *to_send = "uart send test!\r\n";

uint8_t recv_buffer[4];

/* Task priorities. */


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void uart_send_task(void *pvParameters);
void uart_receive_send_task(void *pvParameters);
void GPS_pin_init(void);
void GPS_reset(void);
void GPS_poweron(void);
void GPS_poweroff(void);
void LED_Toggle_Task(void *pvParameters);
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
    /* Init board hardware. */
//    BOARD_InitPins();
    BOARD_BootClockHSRUN();
    BOADR_IO_Init();
    BOADR_Init();
    Uart_Initialize(0);
    Uart_Initialize(1);
    GPS_pin_init();
    GPS_poweron();
//    GPS_poweroff();
    xTaskCreate(LED_Toggle_Task, "LED_Toggle_Task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(uart_receive_send_task, "Uart_receive_task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);
    vTaskStartScheduler();
}
/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
void uart_send_task(void *pvParameters)
{
    char uart0_send[32] = "uart0 test\r\n";   
    portTickType xDelay = pdMS_TO_TICKS(500);
    for(;;)
    {
        UART_Transmit(0, uart0_send, strlen(uart0_send));
        vTaskDelay(xDelay);
    }
}

void uart_receive_send_task(void *pvParameters)
{
    uint8_t receive_byte = 0;
    for(;;)
    {
      if(Uart_Get_Char(0, &receive_byte))
      {
         UART_Transmit(1, &receive_byte, 1);
      }
      if(Uart_Get_Char(1, &receive_byte))
      {
         UART_Transmit(0, &receive_byte, 1);
      }
    }
}

void GPS_pin_init(void)
{
    /* power control port */
    const gpio_pin_config_t GPS_Startup_pin_config = {kGPIO_DigitalOutput, 0,};
    const gpio_pin_config_t GPS_pw_pin_config = {kGPIO_DigitalOutput, 0,};
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(PORTB, 2U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 2U, &GPS_Startup_pin_config);
    /* reset control port */

    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(PORTC, 5U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 5U, &GPS_pw_pin_config);   
}

void GPS_poweron(void)
{
    GPIO_WritePinOutput(GPIOB, 2U, 1);
}

void GPS_poweroff(void)
{
    GPIO_WritePinOutput(GPIOB, 2U, 0); 
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
