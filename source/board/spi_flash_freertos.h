#ifndef _SPI_FLASH_FREERTOS_H_
#define _SPI_FLASH_FREERTOS_H_

#include "fsl_common.h"
#include "fsl_dspi_freertos.h"

#if defined(__cplusplus)
extern "C" {
#endif
  
#define IDCODE_LEN                      (0x3)


/* flash capacity configuration */
#define FLASH_PAGE_SIZE                 (256U)                          /* 256B page size */
#define FLASH_SECTOR_SIZE               (FLASH_PAGE_SIZE * 16U)         /* 4K sector size */
#define FLASH_BLOCK_SIZE                (FLASH_SECTOR_SIZE * 8U)        /* 32K block size */
#define FLASH_TOTAL_SIZE                (FLASH_BLOCK_SIZE * 128U)       /* 4MB total size */

/* flash option timeout */
#define SPI_FLASH_TIMEOUT               (200000)

/* flash commands */
#define CMD_WRITE_ENABLE                (0x06)
#define CMD_WRITE_DISABLE               (0x04)
#define CMD_SR_WRITE_ENABLE             (0x50)
#define CMD_READ_STATUS                 (0x05)
#define CMD_READ_STATUS_1               (0x35)
#define CMD_WRITE_STATUS                (0x01)
#define CMD_READ_ARRAY_SLOW             (0x03)   
#define CMD_READ_ARRAY_FAST             (0x0b)
/* DUAL SPI */
#define CMD_DUAL_OUTPUT_FAST_READ       (0x3B)
#define CMD_DUAL_IO_FAST_READ           (0xBB)    
/* QUAD SPI */
#define CMD_QUAL_OUTPUT_FAST_READ       (0x6B)
#define CMD_QUAL_IO_FAST_READ           (0xEB)
#define CMD_QUAL_IO_WORD_READ           (0xE7)
    
#define CMD_CONTINUOUS_READ             (0xFF)
#define CMD_PAGE_PROGRAM                (0x02)
/* QUAD SPI */
#define CMD_QUAD_PAGE_PROGRAM           (0x32)
    
#define CMD_SECTOR_ERASE                (0x20)    
#define CMD_BLOCK_ERASE_32K             (0x52)
#define CMD_BLOCK_ERASE_64K             (0xd8)
#define CMD_ERASE_CHIP                  (0xc7)
#define CMD_READ_ID                     (0x9f)
#define CMD_ENABLE_RESET                (0x66)
#define CMD_RESET                       (0x99)
#define CMD_DEEP_PWR_DOWN               (0xB9)

/* default block erase configuration*/
#define CMD_BLOCK_ERASE                 CMD_BLOCK_ERASE_32K

/* Flash ID */
#define sFLASH_GD25Q16C_ID         0x00C84015
#define sFLASH_GD25Q32C_ID         0x00C84016

/* flash status */
#define STATUS_BUSY                     (0x01)
enum block_szie
{
    flash_block_size_32k = 32768,
    flash_block_size_64k = 65536,
};

 

/*******************************************************************************
 *  Declaration
 ******************************************************************************/
typedef struct 
{
    uint32_t size;
    uint32_t page_size;
    uint32_t sector_size;
}spi_flash;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void spi_flash_init();
void spi_cs_activate(void);
void spi_cs_deactivate(void);
void spi_flash_addr2cmd(uint32_t addr, uint8_t *cmd);
uint32_t spi_flash_rw(uint8_t *cmd, uint32_t cmd_len, uint8_t *data_out, uint8_t *data_in, uint32_t data_len);
uint32_t spi_flash_enable_write(uint8_t is_enabled);
uint32_t spi_flash_write_page(uint8_t *buf, uint32_t page_offset, uint32_t byte_offset, uint32_t len);
uint32_t spi_flash_chk_status(uint32_t timeout, uint8_t cmd, uint8_t poll_bit);
uint32_t spi_flash_write(uint32_t offset, uint32_t len, void *buf);
uint32_t spi_flash_read(uint32_t offset, uint32_t data_len, void *data);
uint32_t spi_flash_erase_block(uint32_t offset, uint32_t blkSize);
uint32_t spi_flash_erase_sector(uint32_t offset, uint32_t len);
uint32_t spi_flash_erase_all(void);
uint32_t spi_flash_write_status(uint8_t sts_reg);
uint32_t spi_flash_readid(uint8_t *vendorId, uint8_t devId[]);
uint32_t spi_flash_deepPwrDown(void);
bool spi_flash_is_ok(void);
#if defined(__cplusplus)
}
#endif

#endif /* _SPI_FLASH_H_ */
