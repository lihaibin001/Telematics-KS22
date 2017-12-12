#include "fsl_adc16.h"
#include "adc.h"

/**********************************************************************
 * Function
 *********************************************************************/

/*******************************************************************************
*  Function: ADC_Initialize
*
*  Parameters: :none
*  Returns: none
*  Description: Initialze the ADC and pin
*******************************************************************************/
void ADC_Initialize(void)
{
    adc16_config_t config;
    ADC16_GetDefaultConfig(&config);
    config.enableContinuousConversion = true;
    config.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
    config.resolution = kADC16_Resolution16Bit;
    ADC16_Init(ADC0, &config);
    ADC16_EnableHardwareTrigger(ADC0, false);
    CLOCK_EnableClock(kCLOCK_Adc0);
}

/*******************************************************************************
*  Function: ADC_Deinitialize
*
*  Parameters: :none
*  Returns: none
*  Description: Initialze the ADC and pin
*******************************************************************************/
void ADC_Deinitialize(void)
{
    ADC16_Deinit(ADC0);
}

/*******************************************************************************
*  Function: ADC_Read
*
*  Parameters: :channel
*  Returns: return value
*  Description: Read the ADC value
*******************************************************************************/
uint32_t ADC_Read(uint32_t channel)
{
    uint32_t rt = 0;
    adc16_channel_config_t channel_config;

    channel_config.channelNumber = channel;
    channel_config.enableInterruptOnConversionCompleted = false;
    channel_config.enableDifferentialConversion = false;
    ADC16_SetChannelConfig(ADC0, 0U, &channel_config);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0, 0U)))
    {
    }
    rt = ADC16_GetChannelConversionValue(ADC0, 0U);
    return rt;
}
