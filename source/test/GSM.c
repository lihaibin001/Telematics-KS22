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
const char *to_send = "uart send test!\r\n";

uint8_t recv_buffer[4];

/* Task priorities. */
#define uart0_task_PRIORITY (configMAX_PRIORITIES - 1)
#define uart1_task_PRIORITY (configMAX_PRIORITIES - 2)
#define uart2_task_PRIORITY (configMAX_PRIORITIES - 3)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void uart_send_task(void *pvParameters);
void uart_receive_send_task(void *pvParameters);
void uart_idle_task(void *pvParameters);
void GSM_pin_init(void);
void GSM_reset(void);
void GSM_poweron(void);
void GSM_poweroff(void);
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
    Uart_Initialize(0);
    Uart_Initialize(2);
    GSM_pin_init();
    GSM_poweron();
//    xTaskCreate(uart_idle_task, "uart_idle_task", configMINIMAL_STACK_SIZE, NULL, uart0_task_PRIORITY, NULL);
//    xTaskCreate(uart_send_task, "uart_send_task", configMINIMAL_STACK_SIZE, NULL, uart0_task_PRIORITY, NULL);
    xTaskCreate(uart_receive_send_task, "Uart_receive_task", configMINIMAL_STACK_SIZE, NULL, uart0_task_PRIORITY, NULL);
    vTaskStartScheduler();
    for (;;)
    { 
    }
}
void uart_idle_task(void *pvParameters)
{
    for(;;)
    {}
        
}
/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
void uart_send_task(void *pvParameters)
{
    char uart0_send[32] = "uart0 test\r\n";
    char uart1_send[32] = "uart1 test\r\n";
    char uart2_send[32] = "uart2 test\r\n";    
    portTickType xDelay = pdMS_TO_TICKS(500);
    for(;;)
    {
        UART_Transmit(0, uart0_send, strlen(uart0_send));
        UART_Transmit(1, uart1_send, strlen(uart1_send));
//        UART_Transmit(2, uart2_send, strlen(uart2_send));
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
         UART_Transmit(2, &receive_byte, 1);
      }
      if(Uart_Get_Char(2, &receive_byte))
      {
         UART_Transmit(0, &receive_byte, 1);
      }
    }
}

void GSM_pin_init(void)
{
    /* power control port */
    const gpio_pin_config_t GSM_PowerContrl = {kGPIO_DigitalOutput, 0,};
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(PORTB, 16U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 16U, &GSM_PowerContrl); 
}

void GSM_poweron(void)
{
    GPIO_WritePinOutput(GPIOB, 16U, 1);
}

void GSM_poweroff(void)
{
    GPIO_WritePinOutput(GPIOB, 16U, 0); 
}
