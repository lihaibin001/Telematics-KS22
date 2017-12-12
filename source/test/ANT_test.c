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

static uint8_t isAnwser = 0;
uint8_t recv_buffer[4];

/* Task priorities. */
#define uart0_task_PRIORITY (configMAX_PRIORITIES - 1)
#define uart1_task_PRIORITY (configMAX_PRIORITIES - 2)
#define uart2_task_PRIORITY (configMAX_PRIORITIES - 3)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void uart_receive_send_task(void *pvParameters);
void uart_idle_task(void *pvParameters);
void GSM_pin_init(void);
void GSM_reset(void);
void GSM_poweron(void);
void GSM_poweroff(void);
void GSM_PWRKEY_pin_init(void);
void GSM_PWERKEYP_on(void);
void GSM_PWERKEYP_off(void);
void GPRS_ANT_TestTask(void *pvParameters);
void LED_Toggle(TimerHandle_t xTimer);
void CSQ_Check(void);
void GPS_pin_init(void);
void GPS_reset(void);
void GPS_poweron(void);
void GPS_poweroff(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
void * const led_toggle_timer_id;
TimerHandle_t  timer0;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    BOARD_BootClockHSRUN();
    
    /* initialize uart1 for debug */
    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
    Uart_InitIO(0);
    DbgConsole_Init((uint32_t) UART0, 9600, DEBUG_CONSOLE_DEVICE_TYPE_UART, CLOCK_GetCoreSysClkFreq());
     
    /* initialzie uart2 for gprs mode */
    Uart_Initialize(2);
    /* start up GPRS mode */
    GSM_pin_init();
    GSM_poweron();
    GSM_PWRKEY_pin_init();
    GSM_PWERKEYP_on();
    
    /* GPS mode start up */
    GPS_pin_init();
    GPS_poweron();
    
    /* ring pin config */
    {
        const gpio_pin_config_t gpio_config = {kGPIO_DigitalInput, 1,};
        CLOCK_EnableClock(kCLOCK_PortD);
        PORT_SetPinMux(PORTD, 4U, kPORT_MuxAsGpio);
        GPIO_PinInit(GPIOD, 4U, &gpio_config);   
    }
    
    /* led pin initialize */
    {
        const gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 1,};
        CLOCK_EnableClock(kCLOCK_PortC);
        PORT_SetPinMux(PORTC, 10U, kPORT_MuxAsGpio);
        PORT_SetPinMux(PORTC, 11U, kPORT_MuxAsGpio);
        GPIO_PinInit(GPIOC, 10U, &gpio_config);
        GPIO_PinInit(GPIOC, 11U, &gpio_config);
    }
    
    xEventGroupCreate();
    
    PRINTF("\r\nANT test\r\n");
    /* sim800c standby */
    {
        uint8_t rx_buffer[128] = "";
        uint8_t rx_idx = 0;
        uint8_t cmd_tryCount = 0;
        uint32_t waitcount = 0;
        uint8_t response_ok_flag = 0;
        /* send at */
        for(cmd_tryCount = 0; cmd_tryCount < 10; cmd_tryCount++)
        {
            UART_Transmit(2, "ATE0\r\n", strlen( "ATE0\r\n"));
            PRINTF("Send command \"ATE0\"\r\n");
            rx_idx = 0;
            /* sim800c initialize */
            for(waitcount = 0; waitcount < 0xFFFFF; waitcount++)
            {
                if(strstr(rx_buffer ,"OK"))
                {
                    PRINTF(rx_buffer);
                    response_ok_flag = 1;
                    break;
                }
                else if(Uart_Get_Char(2, &rx_buffer[rx_idx]) == true)
                {
                    rx_idx++;
                }
            }
            if(response_ok_flag == 1)
            {
                response_ok_flag = 0;
                break;
            }
        }
    }
    xTaskCreate(GPRS_ANT_TestTask, "GPRS_ANT_TestTask", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    timer0 = xTimerCreate( "time0",
                    pdMS_TO_TICKS(500),
                    pdTRUE,
                    led_toggle_timer_id,
                    LED_Toggle );
    xTimerStart(timer0,portMAX_DELAY);
    vTaskStartScheduler();
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


void GPRS_ANT_TestTask(void *pvParameters)
{
    const gpio_pin_config_t gpio_config = {kGPIO_DigitalInput, 0,};
    CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinMux(PORTD, 4U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 4U, &gpio_config);     
    portTickType xDelay = pdMS_TO_TICKS(10);
    uint8_t rx_buffer[128] = "";
    uint8_t rx_idx = 0;
    uint8_t cmd_tryCount = 0;
    uint32_t waitcount = 0;
    uint8_t response_ok_flag = 0;
    for(;;)
    {
        if(GPIO_ReadPinInput(GPIOD, 4U) == 0)
        {
            vTaskDelay(xDelay);
            if(GPIO_ReadPinInput(GPIOD, 4U) == 0)
            {
                for(cmd_tryCount = 0; cmd_tryCount < 10; cmd_tryCount++)
                {
                    UART_Transmit(2, "ATA\r\n", strlen("ATA\r\n"));
                    PRINTF("Send command \"ATA\"\r\n");
                    rx_idx = 0;
                    /* sim800c initialize */
                    for(waitcount = 0; waitcount < 0xFFFFFFFF; waitcount++)
                    {
                        if(strstr(rx_buffer ,"OK\r\n"))
                        {
                            PRINTF(rx_buffer);
                            memset(&rx_buffer, 0, 128);
                            response_ok_flag = 1;
                            isAnwser = 1;
                            break;
                        }
                        else if(Uart_Get_Char(2, &rx_buffer[rx_idx]) == true)
                        {
                            rx_idx++;
                        }
                    }
                    if(response_ok_flag == 1)
                    {
                        response_ok_flag = 0;
                        break;
                    }
                }
            }
            continue;
        }
        vTaskDelay(xDelay);
        
    }
}

void LED_Toggle(TimerHandle_t xTimer)
{
    /* power control port */
        if(isAnwser == 1)
        {
            GPIO_TogglePinsOutput(GPIOC, 1 << 10U);
        }
        else
        {
            GPIO_TogglePinsOutput(GPIOC, 1 << 11U);
        }
        //CSQ_Check();

}
uint8_t rx_buffer[128] = "";
void CSQ_Check(void)
{
    uint8_t rx_idx = 0;
    uint8_t cmd_tryCount = 0;
    uint32_t waitcount = 0;
    uint8_t response_ok_flag = 0;
    UART_Transmit(2, "AT+CSQ\r\n", strlen( "AT+CSQ\r\n"));
    /* sim800c initialize */
    for(waitcount = 0; waitcount < 0xFFFFFF; waitcount++)
    {
        if(strstr(rx_buffer ,"OK\r\n"))
        {
            PRINTF(rx_buffer);
            memset(rx_buffer, 0, rx_idx);
            response_ok_flag = 1;
            break;
        }
        else if(Uart_Get_Char(2, &rx_buffer[rx_idx]) == true)
        {
            rx_idx++;
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