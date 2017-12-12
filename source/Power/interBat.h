#ifndef __INTERBAT_H__
#define __INTERBAT_H__

/*===========================================================================*
 * Header Files
 *===========================================================================*/

/*===========================================================================*
 * Exported Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Exported Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Exported Type Declarations
 *===========================================================================*/

/*===========================================================================*
 * Exported Const Object Declarations
 *===========================================================================*/

/*===========================================================================*
 * Exported Function Prototypes
 *===========================================================================*/
//extern bool Fan_force_off_is_Condition (void);
extern bool Pwr_Fail_Is_Mute_Condition(void);
extern bool Pwr_Fail_Is_Reset_Condition (void);
extern void Pwr_Fail_Check_Voltage(void);
extern void Pwr_Fail_Monitor(void);
extern bool Pwr_Fail_Is_Voltage_Good (void);
extern void Pwr_Fail_AD_ISR(void);
extern void Pwr_Fail_Initialize(void);
extern void Pwr_Fail_Shutdown(void);
//extern apc_pwr_battery_status_t_T Get_Pwr_Fail_status(void);
//extern void Pwr_Fail_DTC_Status_Update(void);
extern uint16_t Pwr_Fail_AD_get_Voltage(void);
extern uint16_t Pwr_Fail_Get_Voltage(void);
extern uint16_t Pwr_Fail_AD_get_Int_Voltage(void);

/*===========================================================================*/

#endif /* POWERFAIL_H */
