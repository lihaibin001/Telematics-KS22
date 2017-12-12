#include "adc16.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BAT_ADC16_BASE ADC0
#define BAT_ADC16_CHANNEL_GROUP 0U
#define BAT_ADC16_USER_CHANNEL 17U /* PTE24, A0-ADC0_SE17, on MAPS-KS22. */

void ADC_Initialzation(void)
{
    adc16_config_t adc16ConfigStruct;
    adc16_channel_config_t adc16ChannelConfigStruct;
    
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
}