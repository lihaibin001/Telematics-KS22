#include "spi_flash_freertos.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "clock_config.h"
#include "debug.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SEND_BUFF_SIZE      (100)
#define READ_BUFF_SIZE      (4096)
/*******************************************************************************
* Prototypes
******************************************************************************/

/*******************************************************************************
* variable
******************************************************************************/
uint8_t send_buff[SEND_BUFF_SIZE];
uint8_t read_buff[READ_BUFF_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/


static void erase_block_test();
static void write_read_test();
static void read_id_test();

void spi_flash_task(void *argument)
{
    read_id_test();
//    write_read_test();
    erase_block_test();
}
/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_BootClockHSRUN();
    BOADR_IO_Init();
    BOADR_Init();
//    for(int i=0; i<0xFFFFFF; i++);
    xTaskCreate(spi_flash_task, "spi_flash_task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
    for(;;)
    {
    }
}


static void read_id_test()
{
    uint8_t vendorId = 0U;
    uint8_t devId[2] = {0};
    spi_flash_readid(&vendorId, devId);
    DEBUG(DEBUG_HIGH,"\r\vendorID :%x, devID: %x%x\r\n", vendorId, devId[1],devId[0]);
}

static void erase_block_test()
{
//    spi_flash_erase_all();
//    spi_flash_erase_block(4 * 1024, flash_block_size_32k);
//    spi_flash_write(4 * 1024, SEND_BUFF_SIZE, "hello deren hello deren hello deren hello deren");
    spi_flash_read(4 * 1024, READ_BUFF_SIZE, read_buff);
    DEBUG(DEBUG_HIGH,"\r\nerase_block_test :%s\r\n", read_buff);
//    spi_flash_erase_block(0, flash_block_size_32k);
//    spi_flash_read(0x0, READ_BUFF_SIZE, read_buff);
//    PRINTF("\r\nerase_block_test :%s\r\n", read_buff);
}

void write_read_test()
{
    uint8_t idx = 0;
    while(idx < 250)
    {
        memcpy(send_buff, "1234567890\r\n", 12);
        spi_flash_write(idx * 12, 12, send_buff);
        spi_flash_read(0x0, 12 * idx, read_buff);
        idx++;
    }
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

