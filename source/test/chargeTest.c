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
    BOADR_IO_Init();
    BOADR_Init();
    IO_CHARGE_CTL(1);
    while(1);
}