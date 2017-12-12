#include "uart.h"
// Information for each UART channel is stored in variables of this type:

uart_handle_t uart_hande[UART_NUM_CHANNELS];
/******************************************************************************/
/* ROM Const Variables With File Level Scope                                  */
/******************************************************************************/
// Information for each UART channel is stored in variables of this type:
#define UART_TMP_BUFF_LEN   50
typedef struct uart_chan_tag
{
   __IO uint16_t rx_in;                  // Rx buffer input index
   __IO uint16_t rx_out;                 // Rx buffer output index
   __IO uint16_t rx_count;               // Rx buffer byte count
   uint16_t rx_size;                // Rx buffer size
   uint8_t* rx_buf;                 // Rx ring buffer
  // uart_rx_func_ptr rx_func;        // Rx callback function pointer
   __IO uint16_t tx_in;                  // Tx buffer input index
   __IO uint16_t tx_out;                 // Tx buffer output index
   __IO uint16_t tx_count;               // Tx buffer byte counter
   uint16_t tx_size;                // Tx buffer size
   uint8_t* tx_buf;                 // Tx ring buffer
   bool     tx_progress;            // Tx in progress
   /* 2017.9.29 lihaibin modify*/
   __IO bool isTxLock;
   __IO bool isRxLock;
   //uint8_t tmpTxBuffer[UART_TMP_BUFF_LEN];
   uint8_t tmpRxBuffer[UART_TMP_BUFF_LEN];
   //uint8_t tmpTxIdx;
   uint8_t tmpRxIdx;
   /* end  2017.9.29 lihaibin modify */
} uart_chan_T;
/******************************************************************************/
/* Static Variables and Const Variables With File Level Scope                 */
/******************************************************************************/
__IO uart_chan_T  uart_chan[UART_NUM_CHANNELS];

static uint8_t uart0_rx_buf[UART0_RX_BUF_SIZE];
static uint8_t uart0_tx_buf[UART0_TX_BUF_SIZE];
static uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];
static uint8_t uart1_tx_buf[UART1_TX_BUF_SIZE];
static uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];
static uint8_t uart2_tx_buf[UART2_TX_BUF_SIZE];

/******************************************************************************/
/* Function Prototypes for Private Functions with File Level Scope            */
/******************************************************************************/

void u_do_tx (uint8_t chan);    // Transmit helper function

void Uart_InitIO(uint8_t chan);
#if 0
static UART_Type *Uart_GetInterface(uint8_t chan)
{
    switch(chan)
    {
        case 0:
            return UART0;
        case 1:
            return UART1;
        case 2:
            return UART2;
        default:
            return NULL;
    }
}
#endif
/*******************************************************************************
*    Function: Uart_Initialze
*
*  Parameters: chan, spcecify the uart channel
*     Returns: none
* Description: Initialize the spcecified uart channel
*******************************************************************************/
void Uart_Initialize(uint8_t chan)
{   
    if(chan < UART_NUM_CHANNELS)
    {
        uint32_t Priority = 0;
        uart_config_t config;
        UART_GetDefaultConfig(&config);
        Uart_InitIO(chan);
        config.enableTx = true;
        config.enableRx = true;
        uart_chan[chan].rx_count = 0;    // Clear rx byte counter
        uart_chan[chan].rx_in = 0;       // Clear rx buffer input before write index
        uart_chan[chan].rx_out = 0;      // Clear rx buffer output before read index
        //uart_chan[chan].rx_func = NULL;  // Initialize callback off

        uart_chan[chan].tx_count = 0;    // Clear tx byte counter
        uart_chan[chan].tx_in = 0;       // Clear tx buffer input before write index
        uart_chan[chan].tx_out = 0;      // Clear tx buffer output before read index
        uart_chan[chan].tx_progress = false;   // Clear tx in progress flag
        Priority = NVIC_EncodePriority(0, 15, 0);
        switch(chan)
        {
            case 0:
                uart_chan[chan].rx_buf = uart0_rx_buf;
                uart_chan[chan].rx_size = UART0_RX_BUF_SIZE;
                uart_chan[chan].tx_buf = uart0_tx_buf;
                uart_chan[chan].tx_size = UART0_TX_BUF_SIZE;
                NVIC_SetPriority(UART0_RX_TX_IRQn, Priority);
                config.baudRate_Bps = 115200;
                UART_Init(UART0, &config, CLOCK_GetFreq(UART0_CLK_SRC));
                EnableIRQ(UART0_RX_TX_IRQn);
                UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable);
                break;
            case 1:
                uart_chan[chan].rx_buf = uart1_rx_buf;
                uart_chan[chan].rx_size = UART1_RX_BUF_SIZE;
                uart_chan[chan].tx_buf = uart1_tx_buf;
                uart_chan[chan].tx_size = UART1_TX_BUF_SIZE;            
                NVIC_SetPriority(UART1_RX_TX_IRQn, Priority);
                config.baudRate_Bps = 9600;
                UART_Init(UART1, &config, CLOCK_GetFreq(UART1_CLK_SRC));
                EnableIRQ(UART1_RX_TX_IRQn);
                UART_EnableInterrupts(UART1, kUART_RxDataRegFullInterruptEnable);
                break;
            case 2:
                uart_chan[chan].rx_buf = uart2_rx_buf;
                uart_chan[chan].rx_size = UART2_RX_BUF_SIZE;
                uart_chan[chan].tx_buf = uart2_tx_buf;
                uart_chan[chan].tx_size = UART2_TX_BUF_SIZE;            
                NVIC_SetPriority(UART2_RX_TX_IRQn, Priority);
                config.baudRate_Bps = 115200;
                UART_Init(UART2, &config, CLOCK_GetFreq(UART2_CLK_SRC));
                EnableIRQ(UART2_RX_TX_IRQn);
                UART_EnableInterrupts(UART2,kUART_RxDataRegFullInterruptEnable);
                break;
            default:
                break;
        }
        
    }
    else
    {
        //PRINTF("Error during UART initialization with a unexpected parameter.\r\n");
    }
}

/*******************************************************************************
*    Function: Uart_Get_Char
*
*  Parameters: Channel, Pointer to variable which can receive one byte of data
*     Returns: TRUE if data is available and written to pointer, else FALSE
* Description: Reads one byte from the UART receive buffer and writes it to a
*              pointer provided by the caller. A value is also returned to
*              indicate whether a byte was read.
*******************************************************************************/
bool Uart_Get_Char (uint8_t chan, uint8_t* ptr)
{
   bool ret = false;        // Return value. Assume buffer empty!

   // Error checking
   if (chan >= UART_NUM_CHANNELS) 
     	return false; // Invalid channel!
   if (!ptr) 
     	return false;    // Do not accept NULL pointers!

   /* operate the temporary buffer first */
   Disable_Interrupts();
   //UART_EnableInterrupts(UART0, kUART_TxDataRegEmptyInterruptEnable);
   uart_chan[chan].isRxLock = true;
   if(uart_chan[chan].rx_in != uart_chan[chan].rx_out)// Rx buffer not empty
   {
      //uart_chan[chan].rx_count--;   // Decrement rx buffer byte count
      uart_chan[chan].rx_out++;     // Increment rx buffe output index
      if ((uart_chan[chan].rx_out) >= (uart_chan[chan].rx_size))
      {
         uart_chan[chan].rx_out = 0; // Wrap index
      }
      *ptr = uart_chan[chan].rx_buf[uart_chan[chan].rx_out]; // Store read data

      ret = true;
   }
   uart_chan[chan].isRxLock = false;
   Enable_Interrupts();

   return (ret);
}

/*******************************************************************************
*    Function: Uart_Put_Char
*
*  Parameters: Channel, Data to transmit
*     Returns: TRUE on success, FALSE on failure
* Description: Copy one byte to tx buffer
*******************************************************************************/
bool Uart_Put_Char (uint8_t chan, uint8_t data)
{
   bool ret = false;        // Return value. Assume buffer full!
   if (chan >= UART_NUM_CHANNELS) 
   	return false; // Invalid channel!

   /** Enter Critical Section can not restart during this time ***/
   Disable_Interrupts();
   /* operate the temporary buffer first */
   uart_chan[chan].isTxLock = true;
   if (uart_chan[chan].tx_count < (uart_chan[chan].tx_size))
   {
       // Tx buffer not full
      uart_chan[chan].tx_count++;   // Increment tx buffer byte count
      uart_chan[chan].tx_in++;      // Increment tx buffer input index
      if ((uart_chan[chan].tx_in) >= (uart_chan[chan].tx_size))
      {
         uart_chan[chan].tx_in = 0; // Wrap index
      }
      uart_chan[chan].tx_buf[uart_chan[chan].tx_in] = data; // Copy byte to tx buffer

      ret = true;
   }
   
   if (false == uart_chan[chan].tx_progress) // Send first byte. Interrupts do the rest.
   {
      u_do_tx(chan);          // Send to hardware
      uart_chan[chan].tx_progress = true;    // Flag tx in progress
      switch(chan)
      {
          case 0:
            UART_EnableInterrupts(UART0, kUART_TxDataRegEmptyInterruptEnable);
            break;
          case 1:
            UART_EnableInterrupts(UART1, kUART_TxDataRegEmptyInterruptEnable);
            break;
          case 2:
            UART_EnableInterrupts(UART2, kUART_TxDataRegEmptyInterruptEnable);
            break;
          default:
            break;
      }
   }
   uart_chan[chan].isTxLock = false;
   Enable_Interrupts();
   return (ret);
}

/*******************************************************************************
*    Function: u_do_tx
*
*  Parameters: Channel
*     Returns: Nothing
* Description: Transmit helper function. Takes one byte from transmit queue
*              and sends it to the hardware. Provides common code for first
*              byte transmission (before transmit interrupt is enabled) and
*              successive byte transmission (from transmit interrupt).
*******************************************************************************/
void u_do_tx(uint8_t chan)
{
    UART_Type* tmp_uartx;
    switch(chan)
    {
        case 0:
            tmp_uartx = UART0;
            break;
        case 1:
            tmp_uartx = UART1;
            break;
        case 2:
            tmp_uartx = UART2;
            break;
        default:
            return;
    }
    uart_chan[chan].tx_count--;      // Decrement tx buffer byte count
    uart_chan[chan].tx_out++;        // Increment index
    if ((uart_chan[chan].tx_out) >= (uart_chan[chan].tx_size))
    {
       uart_chan[chan].tx_out = 0; // Wrap index
    }
    // Write to hardware transmit register
    tmp_uartx->D = uart_chan[chan].tx_buf[uart_chan[chan].tx_out];
}

/*******************************************************************************
*    Function: UART_TX_ISR
*
*  Parameters: Channel
*     Returns: None
* Description: UART transmit Interrupt Service Routine
*******************************************************************************/
void UART_TX_ISR(uint8_t chan)      
{
    UART_Type* tmp_uartx;

    switch(chan)
    {
        case 0:
            tmp_uartx = UART0;
            break;
        case 1:
            tmp_uartx = UART1;
            break;
        case 2:
            tmp_uartx = UART2;
            break;
        default:
            return;
    }
    
    /* 2017.9.30 lihaibin modify, add lock for uart buffer */
    if(!uart_chan[chan].isTxLock)
    {
        uart_chan[chan].isTxLock = true;
        if (uart_chan[chan].tx_in != uart_chan[chan].tx_out) // Any bytes to send?
        {
            u_do_tx(chan);       // Send to hardware
        }
        else
        {
            uart_chan[chan].tx_progress = false;     // Disable transmit
            UART_DisableInterrupts(tmp_uartx, kUART_TxDataRegEmptyInterruptEnable);
        }
        uart_chan[chan].isTxLock = false;
    }
    else
    {
        uart_chan[chan].tx_progress = false;     // Disable transmit
        UART_DisableInterrupts(tmp_uartx, kUART_TxDataRegEmptyInterruptEnable); 
    }

}

/*******************************************************************************
*    Function: UART_RX_ISR
*
*  Parameters: Channel
*     Returns: None
* Description: UART recieve Interrupt Service Routine
*******************************************************************************/
void UART_RX_ISR(uint8_t chan)
{
    volatile uint8_t data;
    volatile uint8_t err;
    UART_Type* tmp_uartx;

    if (chan >= UART_NUM_CHANNELS)
        return;  // Invalid channel!

    switch(chan)
    {
        case 0:
            tmp_uartx = UART0;
            break;
        case 1:
            tmp_uartx = UART1;
            break;
        case 2:
            tmp_uartx = UART2;
            
            break;
        default:
            return;
    }
    
    err = tmp_uartx->S1;

    /* Read one byte from the receive data register */
    data = (uint16_t)(tmp_uartx->D & (uint16_t)0x01FF);
    
    /* 2017.9.30 lihaibin modify, add lock for uart buffer */
    if(uart_chan[chan].tmpRxIdx == UART_TMP_BUFF_LEN)
    {
        /* should not entry here!!!!!!!!!!! */
        uart_chan[chan].tmpRxIdx = 0;
    }
    uart_chan[chan].tmpRxBuffer[uart_chan[chan].tmpRxIdx++] = data;
    if(uart_chan[chan].isRxLock == false)
    {
        uint8_t idx;
        uart_chan[chan].isRxLock = true;
        for(idx=0; idx<uart_chan[chan].tmpRxIdx; idx++)
        {
            if(0==((kUART_FramingErrorInterruptEnable | kUART_ParityErrorInterruptEnable)&err))
            {
                //uart_chan[chan].rx_count++;
                uart_chan[chan].rx_in++;
                if ((uart_chan[chan].rx_in) >= (uart_chan[chan].rx_size))
                {
                    uart_chan[chan].rx_in = 0; // Wrap index
                }
                if(uart_chan[chan].rx_in != uart_chan[chan].rx_out)
                {
                    uart_chan[chan].rx_buf[uart_chan[chan].rx_in] = 
                        uart_chan[chan].tmpRxBuffer[idx]; // Copy data to receive buffer
                }
                else                                    // Rx buffer full
                {
                    __asm("nop");// Receive buffer overflow. How to handle?
                }
            }
            else
            {
                if(chan == UART_GSM_CHANNEL)
                {
                    __asm("nop");
                }
            }
        }
        uart_chan[chan].tmpRxIdx = 0;
        uart_chan[chan].isRxLock = false;
    }
    else
    {
        
    }
        

    //if(chan == UART_KLINE_CHANNEL)
        //DEBUG( DEBUG_MEDIUM, "%c\r\n", uart_chan[chan].rx_buf[uart_chan[chan].rx_in]);
}
/*******************************************************************************
*    Function: UART_ERR_ISR
*
*  Parameters: Channel
*     Returns: None
* Description: UART ERR Interrupt Service Routine
*******************************************************************************/
void UART_ERR_ISR(uint8_t chan)
{
    volatile uint8_t data;
    volatile uint8_t err;
    UART_Type* tmp_uartx;

    switch(chan)
    {
        case 0:
            tmp_uartx = UART0;
            break;
        case 1:
            tmp_uartx = UART1;
            break;
        case 2:
            tmp_uartx = UART2;
            break;
        default:
            return;

    }
    //clear error
    err = tmp_uartx->S1;
    data = (uint16_t)(tmp_uartx->D & (uint16_t)0x01FF);
}

/******************************************************************************/
/* Function Definitions                                                       */
/******************************************************************************/

/*******************************************************************************
*    Function: UART_Transmit
*  Parameters: Channel
*              pointer to transmit data buffer
*              number of bytes to send
*     Returns: Number of bytes successfully sent/buffered
* Description: This function is used only as legacy code for XM CBM diagnostics (not efficient)
*              Driver task using UART driver should use Uart_Put_Char directly to queue
*               bytes on transmit buffer and check return value to verify if byte to
*               transmit is successfully sent/buffered. If not, task should sleep to allow
*               Tx buffer to allow room for byte to be buffered.
*              Tx Buffer size should be configured accordingly
*******************************************************************************/
extern uint8_t UART_Transmit(uint8_t channel, const uint8_t* tx_buf, uint8_t bytes)
{
    uint8_t ret = 0;
    int i;

    for (i=0; i < bytes; i++)
    {
        if ( Uart_Put_Char(channel, *(tx_buf + i)) )
        {
            ret++;
        }
    }

    return (ret);
}

/*******************************************************************************
*    Function: UART_Rx_Empty
*  Parameters: Channel
*     Returns: Nothing
* Description: Reconfigures UART channel's pins to be I/O input (disables UART function)
*******************************************************************************/
bool UART_Rx_Empty (uint8_t chan)
{
    return ((0 ==uart_chan[chan].rx_count)&& (uart_chan[chan].rx_in == uart_chan[chan].rx_out));
}
/*******************************************************************************
*  Function: UART_Reset_Buf
*
*  Parameters: None
*  Returns: None
*  Description: Intialize the specified uart device
*******************************************************************************/
void UART_Reset_Buf(uint8_t chan)
{
    if (chan >= UART_NUM_CHANNELS)
        return;  // Invalid channel!

    if (NULL != &uart_chan[chan]) 
    {
        uart_chan[chan].rx_count = 0; // Clear rx byte counter
        uart_chan[chan].rx_in = 0;    // Increment before write index
        uart_chan[chan].rx_out = 0;   // Increment before read index

        uart_chan[chan].tx_count = 0; // Clear tx byte counter 
        uart_chan[chan].tx_in = 0;    // Clear tx buffer input before write index
        uart_chan[chan].tx_out = 0;   // Clear tx buffer output before read index
        uart_chan[chan].tx_progress = false;   // Clear tx in progress flag
    }
}
/*******************************************************************************
*  Function: UART0_RX_TX_DriverIRQHandler
*
*  Parameters: None
*  Returns: None
*  Description: UART0 receive/send irq handler
*******************************************************************************/
void UART0_RX_TX_DriverIRQHandler(void)
{
    UART_Type *base = UART0;
    /* If RX overrun. */
    if (UART_S1_OR_MASK & base->S1)
    {
        /* Read base->D, otherwise the RX does not work. */
        (void)base->D;
        UART_ERR_ISR(0);
    }
    
    /* Receive data register full */
    if ((UART_S1_RDRF_MASK & base->S1) && (UART_C2_RIE_MASK & base->C2))
    {
        UART_RX_ISR(0);
    }

    /* Send data register empty and the interrupt is enabled. */
    if ((base->S1 & UART_S1_TDRE_MASK) && (base->C2 & UART_C2_TIE_MASK))
    {
        UART_TX_ISR(0);
    }
    else
    {
        
    }
}

/*******************************************************************************
*  Function: UART0_RX_TX_DriverIRQHandler
*
*  Parameters: None
*  Returns: None
*  Description: UART0 receive/send irq handler
*******************************************************************************/
void UART1_RX_TX_DriverIRQHandler(void)
{
    UART_Type *base = UART1;
    /* If RX overrun. */
    if (UART_S1_OR_MASK & UART1->S1)
    {
        /* Read base->D, otherwise the RX does not work. */
        (void)base->D;
        UART_ERR_ISR(1);
    }
    
    /* Receive data register full */
    if ((UART_S1_RDRF_MASK & base->S1) && (UART_C2_RIE_MASK & base->C2))
    {
        UART_RX_ISR(1);
    }

    /* Send data register empty and the interrupt is enabled. */
    if ((UART1->S1 & UART_S1_TDRE_MASK) && (base->C2 & UART_C2_TIE_MASK))
    {
        UART_TX_ISR(1);
    }
}


/*******************************************************************************
*  Function: UART0_RX_TX_DriverIRQHandler
*
*  Parameters: None
*  Returns: None
*  Description: UART0 receive/send irq handler
*******************************************************************************/
void UART2_RX_TX_DriverIRQHandler(void)
{
    UART_Type *base = UART2;
    /* If RX overrun. */
    if (UART_S1_OR_MASK & base->S1)
    {
        /* Read base->D, otherwise the RX does not work. */
        (void)base->D;
        UART_ERR_ISR(2);
    }
    
    /* Receive data register full */
    if ((UART_S1_RDRF_MASK & base->S1) && (UART_C2_RIE_MASK & base->C2))
    {
        UART_RX_ISR(2);
    }

    /* Send data register empty and the interrupt is enabled. */
    if ((base->S1 & UART_S1_TDRE_MASK) && (base->C2 & UART_C2_TIE_MASK))
    {
        UART_TX_ISR(2);
    }
}
/*******************************************************************************
*    Function: Uart_InitIO
*
*  Parameters: chan, spcecify the uart channel
*     Returns: none
* Description: Initialize the spcecified uart IO port
*******************************************************************************/
void Uart_InitIO(uint8_t chan)
{
    if(chan < UART_NUM_CHANNELS)
    {
        switch(chan)
        {
            case 0:

                PORT_SetPinMux(PORTB, 0u, kPORT_MuxAlt7);
                PORT_SetPinMux(PORTB, 1u, kPORT_MuxAlt7);
                break;
            case 1:
 
                PORT_SetPinMux(PORTC, 3u, kPORT_MuxAlt3);
                PORT_SetPinMux(PORTC, 4u, kPORT_MuxAlt3);
                break;
            case 2:

                PORT_SetPinMux(PORTD, 2u, kPORT_MuxAlt3);
                PORT_SetPinMux(PORTD, 3u, kPORT_MuxAlt3);
                break;
            default:
                break;
        }
    }
}


