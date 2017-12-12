/**
  ******************************************************************************
  * @file    spi_flash.c
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file provides a set of functions needed to manage the SPI M25Pxxx
  *          FLASH memory mounted on STM32xx-EVAL board (refer to stm32_eval.h
  *          to know about the boards supporting this memory). 
  *          It implements a high level communication layer for read and write 
  *          from/to this memory. The needed STM32 hardware resources (SPI and 
  *          GPIO) are defined in stm32xx_eval.h file, and the initialization is 
  *          performed in sFLASH_LowLevel_Init() function declared in stm32xx_eval.c 
  *          file.
  *          You can easily tailor this driver to any other development board, 
  *          by just adapting the defines for hardware resources and 
  *          sFLASH_LowLevel_Init() function.
  *            
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+---------------+-------------+
  *          |  STM32 SPI Pins             |     sFLASH    |    Pin      |
  *          +-----------------------------+---------------+-------------+
  *          | sFLASH_CS_PIN               | ChipSelect(/S)|    1        |
  *          | sFLASH_SPI_MISO_PIN / MISO  |   DataOut(Q)  |    2        |
  *          |                             |   VCC         |    3 (3.3 V)|
  *          |                             |   GND         |    4 (0 V)  |
  *          | sFLASH_SPI_MOSI_PIN / MOSI  |   DataIn(D)   |    5        |
  *          | sFLASH_SPI_SCK_PIN / SCLK   |   Clock(C)    |    6        |
  *          |                             |    VCC        |    7 (3.3 V)|
  *          |                             |    VCC        |    8 (3.3 V)|  
  *          +-----------------------------+---------------+-------------+  
  ******************************************************************************
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "spi_flash.h"

/** @addtogroup Utilities
  * @{
  */

/** @addtogroup Common
  * @{
  */
/*******************************************************************************
 * Variables
 ******************************************************************************/
static dspi_master_handle_t g_m_handle;

static void spi_flash_addr2cmd(uint32_t addr, uint8_t *cmd);
static void sFLASH_LowLevel_Init(void);
#if 0
static void sFLASH_LowLevel_DeInit(void);


static void SPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);

/**
  * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
static void sFLASH_LowLevel_DeInit(void)
{

}
#endif
/**
  * @brief  Initializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
static void sFLASH_LowLevel_Init(void)
{
    CLOCK_EnableClock(sFLASH_SPI_SCK_CLK);
    PORT_SetPinMux(sFLASH_SPI_SCK_GPIO_PORT, 
                   sFLASH_SPI_SCK_PIN, 
                   sFLASH_SPI_SCK_PIN_MUX);
    
    CLOCK_EnableClock(sFLASH_SPI_MISO_CLK);
    PORT_SetPinMux(sFLASH_SPI_MISO_GPIO_PORT, 
                   sFLASH_SPI_MISO_PIN, 
                   sFLASH_SPI_MISO_PIN_MUX);
    
    CLOCK_EnableClock(sFLASH_SPI_MOSI_CLK);
    PORT_SetPinMux(sFLASH_SPI_MOSI_GPIO_PORT, 
                   sFLASH_SPI_MOSI_PIN, 
                   sFLASH_SPI_MOSI_PIN_MUX);

    CLOCK_EnableClock(sFLASH_CS_CLK);
    port_pin_config_t config = {0};
    config.pullSelect = kPORT_PullDown;
    config.mux = sFLASH_CS_PIN_MUX;
    PORT_SetPinConfig(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, &config);
    gpio_pin_config_t csPinConfig = {
        kGPIO_DigitalOutput, 0,
    };
    GPIO_PinInit(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, &csPinConfig);
}

static void SPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    isDspiTransferCompleted = true;
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


/** @defgroup STM32_EVAL_SPI_FLASH_Private_Functions
  * @{
  */ 

/**
  * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
void sFLASH_DeInit(void)
{
}

/**
  * @brief  Initializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
void sFLASH_Init(void)
{
    dspi_master_config_t masterConfig;
    sFLASH_LowLevel_Init();
    DSPI_MasterGetDefaultConfig(&masterConfig);
    DSPI_MasterInit(sFLASH_SPI, &masterConfig, CLOCK_GetFreq(sFLASH_SPI_CLK));
    /* Set up master transfer */
    DSPI_MasterTransferCreateHandle(sFLASH_SPI, &g_m_handle, SPI_MasterUserCallback, NULL);
}

/**
  * @brief  Erases the specified FLASH sector.
  * @param  SectorAddr: address of the sector to erase.
  * @retval None
  */
void sFLASH_EraseSector(uint32_t SectorAddr)
{
    if(E_OK == OS_Wait_Resource(RES_SPIFLASH, RESOURCE_TIMEOUT))   /* wait on SPI Flash to be available */
    {
        uint32_t ret;
        uint8_t cmd[4];

        if (SectorAddr % FLASH_SECTOR_SIZE)
        {
            PRINTF("\r\nspi_flash_erase_sector: parameter err\r\n");
            return 1;
        }
        
        cmd[0] = sFLASH_CMD_SE;
        spi_flash_addr2cmd(offset, cmd);
        
        sFLASH_WriteEnable();
        
        if (ret)
        {
            PRINTF("\r\nspi_flash_erase_sector: enable write err\r\n");
            return 1;
        }
        
        ret = spi_flash_rw(cmd, sizeof(cmd), NULL, NULL, 0);
        if (ret)
        {
            PRINTF("\r\nspi_flash_erase_sector: rw err\r\n");
            return 1;
        }

        ret = spi_flash_chk_status(SPI_FLASH_TIMEOUT, CMD_READ_STATUS, STATUS_BUSY);
        if (ret)
        {
            PRINTF("\r\nspi_flash_erase_sector: check status err\r\n");
            return 1;
        }
        
        PRINTF("\r\nspi_flash_erase_sector: Sector(s) successfully erased!\r\n");


        OS_Release_Resource(RES_SPIFLASH); /* release sole access on EE */
        
        return 0;
    }
}

/**
  * @brief  Erases the entire FLASH.
  * @param  None
  * @retval None
  */
void sFLASH_EraseBulk(void)
{
    if(E_OK == OS_Wait_Resource(RES_SPIFLASH, RESOURCE_TIMEOUT))   /* wait on SPI Flash to be available */
    {
        /*!< Send write enable instruction */
        sFLASH_WriteEnable();

        /*!< Bulk Erase */
        /*!< Select the FLASH: Chip Select low */
        sFLASH_CS_LOW();
        /*!< Send Bulk Erase instruction  */
        sFLASH_SendByte(sFLASH_CMD_BE);
        /*!< Deselect the FLASH: Chip Select high */
        sFLASH_CS_HIGH();

        /*!< Wait the end of Flash writing */
        sFLASH_WaitForWriteEnd();
        
        OS_Release_Resource(RES_SPIFLASH); /* release sole access on EE */
    }
}

/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE cycle 
  *         (Page WRITE sequence).
  * @note   The number of byte can't exceed the FLASH page size.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
  *         or less than "sFLASH_PAGESIZE" value.
  * @retval None
  */
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  /*!< Enable the write access to the FLASH */
  sFLASH_WriteEnable();

  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();
  /*!< Send "Write to Memory " instruction */
  sFLASH_SendByte(sFLASH_CMD_WRITE);
  /*!< Send WriteAddr high nibble address byte to write to */
  sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /*!< Send WriteAddr medium nibble address byte to write to */
  sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /*!< Send WriteAddr low nibble address byte to write to */
  sFLASH_SendByte(WriteAddr & 0xFF);

  /*!< while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /*!< Send the current byte */
    sFLASH_SendByte(*pBuffer);
    /*!< Point on the next byte to be written */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();

  /*!< Wait the end of Flash writing */
  sFLASH_WaitForWriteEnd();
}

/**
  * @brief  Writes block of data to the FLASH. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH.
  * @retval None
  */
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  if(E_OK == OS_Wait_Resource(RES_SPIFLASH, RESOURCE_TIMEOUT))   /* wait on SPI Flash to be available */
  {
      Addr = WriteAddr % sFLASH_SPI_PAGESIZE;
      count = sFLASH_SPI_PAGESIZE - Addr;
      NumOfPage =  NumByteToWrite / sFLASH_SPI_PAGESIZE;
      NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

      if (Addr == 0) /*!< WriteAddr is sFLASH_PAGESIZE aligned  */
      {
        if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
        {
          sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
        }
        else /*!< NumByteToWrite > sFLASH_PAGESIZE */
        {
          while (NumOfPage--)
          {
            sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
            WriteAddr +=  sFLASH_SPI_PAGESIZE;
            pBuffer += sFLASH_SPI_PAGESIZE;
          }

          sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
        }
      }
      else /*!< WriteAddr is not sFLASH_PAGESIZE aligned  */
      {
        if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
        {
          if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
          {
            temp = NumOfSingle - count;

            sFLASH_WritePage(pBuffer, WriteAddr, count);
            WriteAddr +=  count;
            pBuffer += count;

            sFLASH_WritePage(pBuffer, WriteAddr, temp);
          }
          else
          {
            sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
          }
        }
        else /*!< NumByteToWrite > sFLASH_PAGESIZE */
        {
          NumByteToWrite -= count;
          NumOfPage =  NumByteToWrite / sFLASH_SPI_PAGESIZE;
          NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

          sFLASH_WritePage(pBuffer, WriteAddr, count);
          WriteAddr +=  count;
          pBuffer += count;

          while (NumOfPage--)
          {
            sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
            WriteAddr +=  sFLASH_SPI_PAGESIZE;
            pBuffer += sFLASH_SPI_PAGESIZE;
          }

          if (NumOfSingle != 0)
          {
            sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
          }
        }
      }
    OS_Release_Resource(RES_SPIFLASH); /* release sole access on EE */
    }
}

/**
  * @brief  Reads a block of data from the FLASH.
  * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the FLASH.
  * @retval None
  */
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    if(E_OK == OS_Wait_Resource(RES_SPIFLASH, RESOURCE_TIMEOUT))   /* wait on SPI Flash to be available */
    {
        /*!< Select the FLASH: Chip Select low */
        sFLASH_CS_LOW();

        /*!< Send "Read from Memory " instruction */
        sFLASH_SendByte(sFLASH_CMD_READ);

        /*!< Send ReadAddr high nibble address byte to read from */
        sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
        /*!< Send ReadAddr medium nibble address byte to read from */
        sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
        /*!< Send ReadAddr low nibble address byte to read from */
        sFLASH_SendByte(ReadAddr & 0xFF);

        while (NumByteToRead--) /*!< while there is data to be read */
        {
        /*!< Read a byte from the FLASH */
        *pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
        }

        /*!< Deselect the FLASH: Chip Select high */
        sFLASH_CS_HIGH();

        OS_Release_Resource(RES_SPIFLASH); /* release sole access on EE */
    }
}

/**
  * @brief  Reads FLASH identification.
  * @param  None
  * @retval FLASH identification
  */
uint32_t sFLASH_ReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "RDID " instruction */
  sFLASH_SendByte(0x9F);

  /*!< Read a byte from the FLASH */
  Temp0 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

  /*!< Read a byte from the FLASH */
  Temp1 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

  /*!< Read a byte from the FLASH */
  Temp2 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

  return Temp;
}

/**
  * @brief  Initiates a read data byte (READ) sequence from the Flash.
  *   This is done by driving the /CS line low to select the device, then the READ
  *   instruction is transmitted followed by 3 bytes address. This function exit
  *   and keep the /CS line low, so the Flash still being selected. With this
  *   technique the whole content of the Flash is read with a single READ instruction.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @retval None
  */
void sFLASH_StartReadSequence(uint32_t ReadAddr)
{
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "Read from Memory " instruction */
  sFLASH_SendByte(sFLASH_CMD_READ);

  /*!< Send the 24-bit address of the address to read from -------------------*/
  /*!< Send ReadAddr high nibble address byte */
  sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /*!< Send ReadAddr medium nibble address byte */
  sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /*!< Send ReadAddr low nibble address byte */
  sFLASH_SendByte(ReadAddr & 0xFF);
}

/**
  * @brief  Reads a byte from the SPI Flash.
  * @note   This function must be used only if the Start_Read_Sequence function
  *         has been previously called.
  * @param  None
  * @retval Byte Read from the SPI Flash.
  */
uint8_t sFLASH_ReadByte(void)
{
  return (sFLASH_SendByte(sFLASH_DUMMY_BYTE));
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t sFLASH_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(sFLASH_SPI, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(sFLASH_SPI);
}

/**
  * @brief  Sends a Half Word through the SPI interface and return the Half Word
  *         received from the SPI bus.
  * @param  HalfWord: Half Word to send.
  * @retval The value of the received Half Word.
  */
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send Half Word through the sFLASH peripheral */
  SPI_I2S_SendData(sFLASH_SPI, HalfWord);

  /*!< Wait to receive a Half Word */
  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the Half Word read from the SPI bus */
  return SPI_I2S_ReceiveData(sFLASH_SPI);
}

/**
  * @brief  Enables the write access to the FLASH.
  * @param  None
  * @retval None
  */
void sFLASH_WriteEnable(void)
{
  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "Write Enable" instruction */
  sFLASH_SendByte(sFLASH_CMD_WREN);

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();
}

/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
  *         status register and loop until write opertaion has completed.
  * @param  None
  * @retval None
  */
void sFLASH_WaitForWriteEnd(void)
{
  uint8_t flashstatus = 0;

  /*!< Select the FLASH: Chip Select low */
  sFLASH_CS_LOW();

  /*!< Send "Read Status Register" instruction */
  sFLASH_SendByte(sFLASH_CMD_RDSR);

  /*!< Loop as long as the memory is busy with a write cycle */
  do
  {
    /*!< Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    flashstatus = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

  }
  while ((flashstatus & sFLASH_WIP_FLAG) == SET); /* Write in progress */

  /*!< Deselect the FLASH: Chip Select high */
  sFLASH_CS_HIGH();
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */  

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

