#ifndef _SPI_FREERTOS_H_
#define _SPI_FREERTOS_H_

#include "fsl_common.h"
#include "fsl_dspi_freertos.h"
#include "stdint.h"
#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * Declaration
 ******************************************************************************/
typedef struct
{
    dspi_transfer_t masterXfer;
    dspi_rtos_handle_t master_rtos_handle;
    dspi_master_config_t masterConfig;
    uint32_t sourceClock;
    SPI_Type *base;
}spi_freertos_handle_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
uint8_t spi_freertos_initialize(spi_freertos_handle_t *handle);
uint8_t spi_freertos_deinitialize(spi_freertos_handle_t *handle);
uint8_t spi_freertos_write(void);
uint8_t spi_freertos_read(void);
#if defined(__cplusplus)
}
#endif
#endif /* _SPI_FREERTOS_H_ */
