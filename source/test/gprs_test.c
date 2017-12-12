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
void GSM_pin_init(void);
void GSM_reset(void);
void GSM_poweron(void);
void GSM_poweroff(void);
void LED_Toggle_Task(void *pvParameters);
void GSM_PWRKEY_pin_init(void);
void GSM_PWERKEYP_on(void);

void GSM_PWERKEYP_off(void);
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
    GSM_PWRKEY_pin_init();
    GSM_PWERKEYP_on();
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

void GSM_PWRKEY_pin_init(void)
{
    const gpio_pin_config_t GSM_PowerContrl = {kGPIO_DigitalOutput, 0,};
    CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinMux(PORTD, 7U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 7U, &GSM_PowerContrl);     
}

void GSM_PWERKEYP_on(void)
{
    GPIO_WritePinOutput(GPIOD, 7U, 1);
}

void GSM_PWERKEYP_off(void)
{
    GPIO_WritePinOutput(GPIOD, 7U, 0); 
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
