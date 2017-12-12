#ifndef __ADC_H__
#define __ADC_H__

#include "fsl_common.h"

/**********************************************************************
 * Function declaration
 *********************************************************************/
void ADC_Initialize(void);
uint32_t ADC_Read(uint32_t channel);
void ADC_Deinitialize(void);
#endif //__ADC_H__
