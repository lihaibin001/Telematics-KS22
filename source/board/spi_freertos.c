#include "spi_freertos.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Declaration
 ******************************************************************************/
extern uint32_t DSPI_GetInstance(SPI_Type *base);
/*******************************************************************************
*    Function: spi_freertos_initialize
*
*  Parameters: handle point to spi handle
*     Returns: 0, sucess. 1, parameter error. 2, low lever error 
* Description: initialize the specified spi port
*******************************************************************************/
uint8_t spi_freertos_initialize(spi_freertos_handle_t *handle)
{
    uint32_t status;
    status = DSPI_RTOS_Init(&handle->master_rtos_handle, handle->base, &handle->masterConfig, handle->sourceClock);
    if (status != kStatus_Success)
    {
        PRINTF("DSPI master: error during initialization spi%d. \r\n", DSPI_GetInstance(handle->base));
        return 1;
    }
    PRINTF("DSPI master: initialization spi%d complete. \r\n", DSPI_GetInstance(handle->base));
    return 0;
}

/*******************************************************************************
*    Function: spi_freertos_deinitialize
*
*  Parameters: handle point to spi handle
*     Returns: 0, sucess. 1, parameter error. 2, low lever error 
* Description: deinitialize the specified SPI port
*******************************************************************************/
uint8_t spi_freertos_deinitialize(spi_freertos_handle_t *handle)
{
    if(handle == NULL)
    {
        PRINTF("DSPI master: deinitialization spi with a err parameter. \r\n");
        return 1;
    }
    DSPI_RTOS_Deinit(&handle->master_rtos_handle);
    return 0;
}
/*******************************************************************************
*    Function: spi_freertos_write
*
*  Parameters: handle point to spi handle
*     Returns: 0, sucess. 1, parameter error. 2, low lever error 
* Description: deinitialize the specified SPI port
*******************************************************************************/
uint8_t spi_freertos_write(dspi_rtos_handle_t *handle, dspi_transfer_t *transfer)
{
    
    return 0;
}

