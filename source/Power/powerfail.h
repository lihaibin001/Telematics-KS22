#ifndef POWERFAIL_H
#define POWERFAIL_H

/*===========================================================================*
 * Header Files
 *===========================================================================*/

/*===========================================================================*
 * Exported Preprocessor #define Constants
 *===========================================================================*/
#define VOLTAGE_VAL_COPENSTATION       6//   6 //0.06V voltage drop between BATT and FBATT

/*--------------------------------------------------------------------- */
/* Relays Low Voltage Warning (fbatt) ISR macros                        */
/* ADINT interrupt service routine macros                               */
/*--------------------------------------------------------------------- */
#define AD_Interrupt_Configure()             ADIC = 0x47; /*ADIC = 0x40 */
//#define AD_Interrupt_Enable()                if(ADMK)  ADMK = false;
//#define AD_Interrupt_Disable()               if(!ADMK) ADMK = true;
#define AD_Interrupt_Enable()               
#define AD_Interrupt_Disable() 
/*===========================================================================*
 * Exported Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Exported Type Declarations
 *===========================================================================*/
#define MAX_DEBOUNCE_COUNT          6  /* 6*5ms =30 ms*/

#define MUTE_OFF_WAIT_TICKS         (500)

#define INTER_BAT_CHARGE_THRESHOLD      390
#define INTER_BAT_UNCHARGE_THRESHOLD    410
#define INTER_BAT_CHARGE_DURTION        60*1000  

typedef enum PF_V_Low_Treshold_Tag
{
   V_LOW_WARNING,
   V_LOW_RECOVERY,
   NUM_LOW_VOLTAGE_TRESHOLD
}PF_V_Low_Treshold_Type;

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
extern uint16_t Pwr_Fail_Get_Int_Voltage(void);
extern uint16_t Pwr_Fail_AD_get_Int_Voltage(void);
bool Pwr_Fail_Is_Inter_Batt_Chraging(void);
void Pwr_Fail_Set_Inter_batt_Charge_Sta(bool sta);
/*===========================================================================*/

#endif /* POWERFAIL_H */
