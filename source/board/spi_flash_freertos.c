#include <string.h>
#include "spi_flash_freertos.h"
#include "fsl_gpio.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "clock_config.h"
#include "debug.h"
/*******************************************************************************
 * define
 ******************************************************************************/

#define FLASH_SPI                       SPI1
#define FLASH_SPI_CLK                   DSPI1_CLK_SRC

#define FLASH_SPI_SCK_CLK               kCLOCK_PortD
#define FLASH_SPI_SCK_GPIO_PORT         PORTD        
#define FLASH_SPI_SCK_PIN               (5U)                             
#define FLASH_SPI_SCK_PIN_MUX           kPORT_MuxAlt7

#define FLASH_SPI_MISO_CLK              kCLOCK_PortE
#define FLASH_SPI_MISO_GPIO_PORT        PORTE
#define FLASH_SPI_MISO_PIN              (1U)                  
#define FLASH_SPI_MISO_PIN_MUX          kPORT_MuxAlt7

#define FLASH_SPI_MOSI_CLK             kCLOCK_PortD
#define FLASH_SPI_MOSI_GPIO_PORT       PORTD
#define FLASH_SPI_MOSI_PIN             (6U)                  
#define FLASH_SPI_MOSI_PIN_MUX         kPORT_MuxAlt7

#define FLASH_CS_CLK                    kCLOCK_PortE
#define FLASH_CS_GPIO_PORT              PORTE 
#define FLAHS_CS_GPIO                   GPIOE
#define FLASH_CS_PIN                    (0U)                     
#define FLASH_CS_PIN_MUX                kPORT_MuxAsGpio
     
#define FLASH_HOLD_CLK                  kCLOCK_PortD
#define FLASH_HOLD_GPIO_PORT            PORTD
#define FLASH_HOLD_GPIO                 GPIOD
#define FLASH_HOLD_PIN                  (0U)
#define FLASH_HOLD_PIN_MUX              kPORT_MuxAsGpio
     
#define FLASH_WP_CLK                    kCLOCK_PortD
#define FLASH_WP_GPIO_PORT              PORTD
#define FLASH_WP_GPIO                   GPIOD
#define FLASH_WP_PIN                    (1U)
#define FLASH_WP_PIN_MUX                kPORT_MuxAsGpio

#define FLASH_IRQn                      SPI1_IRQn

static  SemaphoreHandle_t xMutex;
//static  SemaphoreHandle_t TxMutex;
static struct __flashInfo
{
    uint8_t devId[2];
    uint8_t venderId;
}FlashInfo;
static struct __flashInfo FlashInfo = 
{
    .devId = {0x40, 0x16},
    .venderId = 0xC8,
};
bool isFlashOk = false;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void spi_low_level_init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static dspi_rtos_handle_t master_rtos_handle;
/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
*    Function: spi_flash_init
*
*  Parameters: none
*     Returns: none
* Description: initialize the specified flash and the SPI port it used
*******************************************************************************/
void spi_flash_init(void)
{
    uint32_t sourceClock;
    status_t status;
    dspi_master_config_t masterConfig;
    static struct __flashInfo flashInfoOnShip;
    DEBUG(DEBUG_HIGH, "[Init] flash initializing\r\n");
    GPIO_WritePinOutput(FLASH_HOLD_GPIO, FLASH_HOLD_PIN, 1); //Keep 'hold input' pin high
    GPIO_WritePinOutput(FLASH_WP_GPIO, FLASH_WP_PIN, 1);     //Keep  'write protect input' pin high 
    GPIO_WritePinOutput(FLAHS_CS_GPIO, FLASH_CS_PIN, 1);
    spi_low_level_init();
    xMutex = xSemaphoreCreateMutex();
    NVIC_SetPriority(FLASH_IRQn, 7);

    DSPI_MasterGetDefaultConfig(&masterConfig);

    sourceClock = CLOCK_GetFreq(FLASH_SPI_CLK);
    
    status = DSPI_RTOS_Init(&master_rtos_handle, 
                            FLASH_SPI, 
                            &masterConfig, 
                            sourceClock);
    if (status != kStatus_Success)
    {
        DEBUG(DEBUG_HIGH,"[FLASH] initialization error\r\n");
        //for(;;);
    }
    spi_flash_readid(&flashInfoOnShip.venderId, flashInfoOnShip.devId);
    if(flashInfoOnShip.venderId != FlashInfo.venderId 
       || flashInfoOnShip.devId[0] != FlashInfo.devId[0]
           || flashInfoOnShip.devId[1] != FlashInfo.devId[1])
    {
        DEBUG(DEBUG_HIGH, "[FLASH] ERROR, flash id error.\r\n");
        DEBUG(DEBUG_HIGH,"On ship: Vender id [%d],Device id [%d%d]\r\n", 
                          flashInfoOnShip.venderId,
                          flashInfoOnShip.devId[0],
                          flashInfoOnShip.devId[1]);
        DEBUG(DEBUG_HIGH,"Expect: Vender id [%d],Device id [%d%d]\r\n", 
                  FlashInfo.venderId,
                  FlashInfo.devId[0],
                  FlashInfo.devId[1]);
    }
    else
    {
        isFlashOk = true;
        DEBUG(DEBUG_HIGH, "[Init] flash initialize complete!\r\n");
    }
}

/*******************************************************************************
*    Function: spi_cs_activate
*
*  Parameters: none
*     Returns: none
* Description: activate the specified chip select pin 
*******************************************************************************/
void spi_cs_activate(void)
{
    GPIO_WritePinOutput(FLAHS_CS_GPIO, FLASH_CS_PIN, 0);
}

/*******************************************************************************
*    Function: spi_cs_deactivate
*
*  Parameters: none
*     Returns: none
* Description: deactivate the specified chip select pin 
*******************************************************************************/
void spi_cs_deactivate(void)
{
    GPIO_WritePinOutput(FLAHS_CS_GPIO, FLASH_CS_PIN, 1);
}

/*******************************************************************************
*    Function: spi_flash_addr2cmd
*
*  Parameters: addr, point to the address. cmd point to the command to be converted
*     Returns: none
* Description: convert the address to command
*******************************************************************************/
void spi_flash_addr2cmd(uint32_t addr, uint8_t *cmd)
{
    cmd[1] = addr >> 16;
    cmd[2] = addr >> 8;
    cmd[3] = addr >> 0;
}

/*******************************************************************************
*    Function: spi_flash_rw
*
*  Parameters: cmd point to the operational command.
*               cmd _len express the the command len.
*               data_out point to the output data
*               data_in point to the input data
*               data_len express the length of data
*     Returns: 0 sucess. 1 fail
* Description: write operational command then read/write the specified data
*******************************************************************************/
uint32_t spi_flash_rw(uint8_t *cmd, uint32_t cmd_len, uint8_t *data_out, uint8_t *data_in, uint32_t data_len)
{
    status_t status;
    
    spi_cs_activate();

    dspi_transfer_t masterXfer;

    masterXfer.txData = cmd;
    masterXfer.rxData = NULL;
    masterXfer.dataSize = cmd_len;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    status = DSPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);

    if (status != kStatus_Success)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] Transfer error. \r\n\r\n");        
    }
    if (data_len != 0)
    {
        masterXfer.txData = data_out;
        masterXfer.rxData = data_in;
        masterXfer.dataSize = data_len;
        masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0;

        status = DSPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);

        if (status != kStatus_Success)
        {
            DEBUG(DEBUG_HIGH,"\r\n[FLASH]: Transfer error. \r\n\r\n");        
        }
    }

    spi_cs_deactivate();
    return 0;
}

/*******************************************************************************
*    Function: spi_flash_chk_status
*
*  Parameters: timeout experess the timeout value
*               cmd express the operation command
*               poll_bit express the bit we must observe 
*     Returns: 0 sucess. 1 fail
* Description: chek the status of the spcified operatoin comand
*******************************************************************************/
uint32_t spi_flash_chk_status(uint32_t timeout, uint8_t cmd, uint8_t poll_bit)
{
    uint32_t i = 0;
    uint8_t spi_status;
    status_t status;
    
    spi_cs_activate();
    dspi_transfer_t masterXfer;

    masterXfer.txData = &cmd;
    masterXfer.rxData = NULL;
    masterXfer.dataSize = 1;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0;

    status = DSPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);

    if (status != kStatus_Success)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] Transfer error. \r\n\r\n");        
    }

    for (i = 0; i < timeout; ++i)
    {
        masterXfer.txData = NULL;
        masterXfer.rxData = &spi_status;
        masterXfer.dataSize = 1;
        masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0;

        status = DSPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);

        if (status != kStatus_Success)
        {
            DEBUG(DEBUG_HIGH,"\r\n[FLASH] Transfer error. \r\n\r\n");        
        }

        if ((spi_status & poll_bit) == 0)
        {
            break;
        }
    }

    spi_cs_deactivate();

    if (i == timeout)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] Transfer time out!\n");
        return 1;
    }

    return 0;
}

/*******************************************************************************
*    Function: spi_flash_enable_write
*
*  Parameters: is_enable express is or isn't enable the write operation
*     Returns: 0 sucess, 1 fail
* Description: is or isn't enable the write operation
*******************************************************************************/
uint32_t spi_flash_enable_write(uint8_t is_enabled)
{
    uint8_t cmd;

    cmd = is_enabled ? CMD_WRITE_ENABLE : CMD_WRITE_DISABLE;

    return spi_flash_rw(&cmd, 1, NULL, NULL, 0);
}

/*******************************************************************************
*    Function: spi_flash_write_page
*
*  Parameters: buf point to the data to be sent
*               page_offset express the offset address of the page
*               byte_offset express the offset address of the data in the specified page 
*               len express the length of data
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t spi_flash_write_page(uint8_t *buf, uint32_t page_offset, uint32_t byte_offset, uint32_t len)
{
    uint8_t cmd[4] = {0};

    cmd[0] = CMD_PAGE_PROGRAM;
    cmd[1] = page_offset >> 8;
    cmd[2] = page_offset;
    cmd[3] = byte_offset;

    /* Each write need to enable write */
    if (spi_flash_enable_write(1))
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] enabling write failed\n");
        return 1;
    }

    if (spi_flash_rw(cmd, 4, buf, NULL, len))
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] write failed\n");
        return 1;
    }

    if (spi_flash_chk_status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY))
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] check status failed\n");
        return 1;
    }

    return 0;
}

/*******************************************************************************
*    Function: spi_flash_write
*
*  Parameters: offset express the address to be written 
*               len express the length of the data
*               buf point to the data bo be written 
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t spi_flash_write(uint32_t offset, uint32_t len, void *buf)
{
    xSemaphoreTake( xMutex, portMAX_DELAY );
    uint32_t page_offset = 0, byte_offset = 0;
    uint32_t data_chunk_len = 0, data_transferred = 0;
    uint32_t ret = 0;
    page_offset = offset / FLASH_PAGE_SIZE;
    byte_offset = offset % FLASH_PAGE_SIZE;

    while (data_transferred < len)
    {
        /* First and last sector might be unaligned to page_size,
           So transfer unaligned sector first. */
        data_chunk_len = (len - data_transferred) < (FLASH_PAGE_SIZE - byte_offset) ? (len - data_transferred) : (FLASH_PAGE_SIZE - byte_offset);
        
        ret = spi_flash_write_page(((uint8_t *)buf + data_transferred), page_offset, byte_offset, data_chunk_len);
        if (1 == ret)
        {
            DEBUG(DEBUG_HIGH,"[FLASH] write page err.\r\n");           
            break;
        }

        byte_offset += data_chunk_len;
        if (byte_offset == FLASH_PAGE_SIZE)
        {
            page_offset++;
            byte_offset = 0;
        }
        data_transferred += data_chunk_len;
    }
    xSemaphoreGive( xMutex );
    return ret;
}


/*******************************************************************************
*    Function: spi_flash_read
*
*  Parameters: offset express the address to be read
*               len express the length of the data
*               data point to the buffer bo store read data 
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t    spi_flash_read(uint32_t offset, uint32_t data_len, void *data)
{
    xSemaphoreTake( xMutex, portMAX_DELAY );
    uint8_t cmd[5];
    uint8_t ret = 0;
    cmd[0] = CMD_READ_ARRAY_FAST;
    spi_flash_addr2cmd(offset, cmd);
    cmd[4] = 0x00;   
    ret =spi_flash_rw(cmd, sizeof(cmd), NULL, data, data_len); 
    xSemaphoreGive( xMutex );
    return ret;
}

/*******************************************************************************
*    Function: spi_flash_erase_block
*
*  Parameters: offset express the address of the block 
*               blkSize express the block size.(reference block_szie_t)
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t spi_flash_erase_block(uint32_t offset, uint32_t blkSize)
{
    uint32_t ret;
    uint8_t cmd[4];
    if (offset % blkSize)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] parameter err\n");
        return 1;
    }

    switch(blkSize)
    {
        case flash_block_size_32k:
            cmd[0] = CMD_BLOCK_ERASE_32K;
            break;
        case flash_block_size_64k:
            cmd[0] = CMD_BLOCK_ERASE_64K;
            break;
        default:
            DEBUG(DEBUG_HIGH,"\r\n[FLASH] parameter err\n");           
            return 1;
    }
    
    spi_flash_addr2cmd(offset, cmd);

    ret = spi_flash_enable_write(1);
    if (ret)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] enable write err\n");
        return 1;
    }

    ret = spi_flash_rw(cmd, sizeof(cmd), NULL, NULL, 0);
    if (ret)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] rw err\n");
        return 1;
    }
    
    ret = spi_flash_chk_status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY);
    if (ret)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] check status err\n");        
        return 1;
    }
    return 0;
}

/*******************************************************************************
*    Function: spi_flash_erase_sector
*
*  Parameters: offset express the address of the block
*               len express the length of the chunk to be erased
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t spi_flash_erase_sector(uint32_t offset, uint32_t len)
{
    uint32_t eraseStart, eraseEnd;
    uint32_t ret;
    uint8_t cmd[4];

    if ((offset % FLASH_SECTOR_SIZE) || (len % FLASH_SECTOR_SIZE))
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] parameter err\r\n");
        return 1;
    }
    
    cmd[0] = CMD_SECTOR_ERASE;
    eraseStart = offset;
    eraseEnd = eraseStart + len;

    while (offset < eraseEnd)
    {
        spi_flash_addr2cmd(offset, cmd);
        offset += FLASH_SECTOR_SIZE;

        ret = spi_flash_enable_write(1);
        if (ret)
        {
            DEBUG(DEBUG_HIGH,"\r\n[FLASH] enable write err\r\n");
            return 1;
        }

        ret = spi_flash_rw(cmd, sizeof(cmd), NULL, NULL, 0);
        if (ret)
        {
            DEBUG(DEBUG_HIGH,"\r\n[FLASH] rw err\r\n");
            return 1;
        }

        ret = spi_flash_chk_status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY);
        if (ret)
        {
            DEBUG(DEBUG_HIGH,"\r\n[FLASH] check status err\r\n");
            return 1;
        }
    }

    DEBUG(DEBUG_LOW,"\r\n[FLASH] Sector erase sucess!\r\n");

    return 0;
}

/*******************************************************************************
*    Function: spi_flash_erase_all
*
*  Parameters: none
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t spi_flash_erase_all(void)
{
    uint32_t ret;
    uint8_t cmd = CMD_ERASE_CHIP;

    ret = spi_flash_enable_write(1);
    if (ret)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] enable write err\r\n");
        return 1;
    }

    ret = spi_flash_rw(&cmd, 1, NULL, NULL, 0);
    if (ret)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] rw err\r\n");
        return 1;
    }

    ret = spi_flash_chk_status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY);
    if (ret)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] checkstaus err\r\n");
        return 1;
    }

    DEBUG(DEBUG_LOW,"\r\n[FLASH] Chip successfully erased!\r\n");

    return 0;
}

/*******************************************************************************
*    Function: spi_flash_write_status
*
*  Parameters: sts_reg express the status register
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t spi_flash_write_status(uint8_t sts_reg)
{
    uint8_t cmd;
    uint32_t ret;

    ret = spi_flash_enable_write(1);
    if (ret != 0)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] enabling write failed\n");
        return ret;
    }

    cmd = CMD_WRITE_STATUS;
    ret = spi_flash_rw(&cmd, 1, &sts_reg, NULL, 1);
    if (ret != 0)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] fail to write status register\n");
        return ret;
    }

    ret = spi_flash_chk_status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY);
    if (ret != 0)
    {
        DEBUG(DEBUG_HIGH,"\r\n[FLASH] write status register timed out\n");
        return ret;
    }

    return 0;
}

/*******************************************************************************
*    Function: spi_flash_readid
*
*  Parameters: vnedorId store the manufacturer ID
*               devID store the device ID
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t spi_flash_readid(uint8_t *vendorId, uint8_t devId[])
{
    uint32_t ret;
    uint8_t idcode[IDCODE_LEN] = {0};
    uint8_t cmd = CMD_READ_ID;

    /* Send CMD_READ_ID to get flash chip ID codes */
    ret = spi_flash_rw(&cmd, 1, NULL, idcode, sizeof(idcode));
    if (ret)
    {
       DEBUG(DEBUG_HIGH,"\r\n[FLASH] rw err\n");
        return 1;
    }

    *vendorId = idcode[0];
    devId[0] = idcode[1];
    devId[1] = idcode[2];

    return 0;
}

/*******************************************************************************
*    Function: spi_flash_deepPwrDown
*
*  Parameters: None.
*     Returns: 0 sucess, 1 fail
* Description: 
*******************************************************************************/
uint32_t spi_flash_deepPwrDown(void)
{
    uint32_t ret;
    uint8_t idcode[IDCODE_LEN] = {0};
    uint8_t cmd = CMD_DEEP_PWR_DOWN;

    /* Send CMD_READ_ID to get flash chip ID codes */
    ret = spi_flash_rw(&cmd, 1, NULL, idcode, sizeof(idcode));
    if (ret)
    {
       DEBUG(DEBUG_HIGH,"\r\n[FLASH] rw err\n");
        return 1;
    }
    return 0;
}


/*******************************************************************************
*    Function: spi_low_level_init
*
*  Parameters: none
*     Returns: none
* Description: initialize the pin used by the FLASH's spi
*******************************************************************************/
static void spi_low_level_init(void)
{
    CLOCK_EnableClock(FLASH_SPI_SCK_CLK);
    PORT_SetPinMux(FLASH_SPI_SCK_GPIO_PORT, 
                   FLASH_SPI_SCK_PIN, 
                   FLASH_SPI_SCK_PIN_MUX);
    
    CLOCK_EnableClock(FLASH_SPI_MISO_CLK);
    PORT_SetPinMux(FLASH_SPI_MISO_GPIO_PORT, 
                   FLASH_SPI_MISO_PIN, 
                   FLASH_SPI_MISO_PIN_MUX);
    
    CLOCK_EnableClock(FLASH_SPI_MOSI_CLK);
    PORT_SetPinMux(FLASH_SPI_MOSI_GPIO_PORT, 
                   FLASH_SPI_MOSI_PIN, 
                   FLASH_SPI_MOSI_PIN_MUX);
    
    CLOCK_EnableClock(FLASH_CS_CLK);
    port_pin_config_t config = {0};
    config.pullSelect = kPORT_PullUp;
    config.mux = kPORT_MuxAsGpio;
    PORT_SetPinConfig(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, &config);
    gpio_pin_config_t csPinConfig = {
        kGPIO_DigitalOutput, 0,
    };
    GPIO_PinInit(FLAHS_CS_GPIO, FLASH_CS_PIN, &csPinConfig);
}

bool spi_flash_is_ok(void)
{
    return isFlashOk;
}
/******************************************************************************
 * End of module                                                              *
 ******************************************************************************/
