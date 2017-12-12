/**
  ******************************************************************************
  * @file    spi_flash.h
  * @author  
  * @version 
  * @date    
  * @brief   This file contains all the functions prototypes for the stm32_eval_spi_flash
  *          firmware driver.
  ******************************************************************************  
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "fsl_common.h"
#include "fsl_dspi.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "semphr.h"



/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup Common
  * @{
  */
  
/** @addtogroup STM32_EVAL_SPI_FLASH
  * @{
  */  

/** @defgroup STM32_EVAL_SPI_FLASH_Exported_Types
  * @{
  */ 
/**
  * @}
  */
  
/** @defgroup STM32_EVAL_SPI_FLASH_Exported_Constants
  * @{
  */
/**
  * @brief  M25P SPI Flash supported commands
  */  
#define sFLASH_CMD_WRITE          0x02  /*!< Write to Memory instruction */
#define sFLASH_CMD_WRSR           0x01  /*!< Write Status Register instruction */
#define sFLASH_CMD_WREN           0x06  /*!< Write enable instruction */
#define sFLASH_CMD_READ           0x03  /*!< Read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /*!< Read Status Register instruction  */
#define sFLASH_CMD_RDID           0x9F  /*!< Read identification */
#define sFLASH_CMD_SE             0x20  /*!< Sector Erase instruction */
#define sFLASH_CMD_BE             0xD8  /*!< Bulk Erase instruction */

#define sFLASH_WIP_FLAG           0x01  /*!< Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_SPI_PAGESIZE       0x100

//#define sFLASH_S25FL208K_ID         0x00014014
#define sFLASH_S25FL116K_ID         0x00C84015

/*
  * @brief  FLASH SPI Interface pins
  */  
#define sFLASH_SPI                       SPI0
#define sFLASH_SPI_CLK                   DSPI0_CLK_SRC

#define sFLASH_SPI_SCK_GPIO_PORT         GPIOD        
#define sFLASH_SPI_SCK_PIN               (5U)                              
#define sFLASH_SPI_SCK_PIN_MUX           kPORT_MuxAlt2
#define sFLASH_SPI_SCK_CLK               kCLOCK_PortD

#define sFLASH_SPI_MISO_GPIO_PORT        GPIOE
#define sFLASH_SPI_MISO_PIN              (1U)                  
#define sFLASH_SPI_MISO_PIN_MUX          kPORT_MuxAlt2
#define sFLASH_SPI_MISO_CLK              kCLOCK_PortE

#define sFLASH_SPI_MOSI_GPIO_PORT        GPIOD
#define sFLASH_SPI_MOSI_PIN              (6U)                  
#define sFLASH_SPI_MOSI_PIN_MUX         kPORT_MuxAlt7
#define sFLASH_SPI_MOSI_CLK             kCLOCK_PortD

#define sFLASH_CS_GPIO_PORT              GPIOE 
#define sFLASH_CS_PIN                    (0U)                     
#define sFLASH_CS_PIN_MUX                kPORT_MuxAsGpio
#define sFLASH_CS_CLK                    kCLOCK_PortE
  
/**
  * @}
  */ 
  
/** @defgroup STM32_EVAL_SPI_FLASH_Exported_Macros
  * @{
  */
/**
  * @brief  Select sFLASH: Chip Select pin low
  */
#define sFLASH_CS_LOW()       GPIO_WritePinOutput(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, 0)
/**
  * @brief  Deselect sFLASH: Chip Select pin high
  */
#define sFLASH_CS_HIGH()      GPIO_WritePinOutput(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, 1)   
/**
  * @}
  */ 
  
#define FLASH_INIT_FLAG_ADDR   0x00000000
#define FLASH_CONFIG_ADDR      0x00000004
#define FLASH_CAN_BACKUP_ADDR  0x00000100
#define FLASH_GPS_BACKUP_ADDR  0x00060000
#define FLASH_OTA_ADDR         0x000C0000

#define FLASH_INIT_FLAG 0x55aa55aa

/** @defgroup STM32_EVAL_SPI_FLASH_Exported_Functions
  * @{
  */
/**
  * @brief  High layer functions
  */
void sFLASH_DeInit(void);
void sFLASH_Init(void);
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sFLASH_ReadID(void);
void sFLASH_StartReadSequence(uint32_t ReadAddr);

/**
  * @brief  Low layer functions
  */
uint8_t sFLASH_ReadByte(void);
uint8_t sFLASH_SendByte(uint8_t byte);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_FLASH_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

