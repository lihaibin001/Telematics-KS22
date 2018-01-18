#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "standard.h"
/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "fsl_common.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Globals
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */

int main(void)
{
    if (kRCM_SourceWakeup & RCM_GetPreviousResetSources(RCM)) /* Wakeup from VLLS. */
    {
        PMC_ClearPeriphIOIsolationFlag(PMC);
        NVIC_ClearPendingIRQ(LLWU_IRQn);
    }
    BOARD_BootClockHSRUN();
    OS_Start();
}

int fputc( int ch, FILE *f )
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    while (!(UART0->S1 & UART_S1_TDRE_MASK))
    {
    }
    UART0->D = (uint8_t )ch;

    return ch;
}

