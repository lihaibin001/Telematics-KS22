#ifndef _UART_H_
#define _UART_H_
/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_port.h"
#include "fsl_uart.h"

#include "fsl_common.h"
#include "clock_config.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/
#define UART0_RX_BUF_SIZE   150  
#define UART0_TX_BUF_SIZE   150
#define UART1_RX_BUF_SIZE   600 
#define UART1_TX_BUF_SIZE   100
#define UART2_RX_BUF_SIZE   1100
#define UART2_TX_BUF_SIZE   1000

    

typedef enum UART_CHANNEL_Tag
{
    UART_DEBUG_CHANNEL = 0,
    UART_GPS_CHANNEL = 1,
    UART_GSM_CHANNEL = 2,
    UART_NUM_CHANNELS
}UART_CHANNEL_T;

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/
extern bool Uart_Put_Char (uint8_t chan, uint8_t data);
extern bool Uart_Get_Char (uint8_t chan, uint8_t* ptr);
extern void Uart_Initialize (uint8_t chan);
extern bool UART_Rx_Empty (uint8_t chan);
extern void UART_Reset_Buf(uint8_t chan);

extern void UART_TX_ISR(uint8_t chan); // Common Transmit Interrupt Service Routine
extern void UART_RX_ISR(uint8_t chan); // Common Receive Interrupt Service Routine
extern void UART_ERR_ISR(uint8_t chan);
void Uart_InitIO(uint8_t chan);
extern uint8_t UART_Transmit(uint8_t channel, const uint8_t* tx_buf, uint8_t bytes);
#if defined(__cplusplus)
}
#endif /* __cplusplus*/
#endif //_UART_H_