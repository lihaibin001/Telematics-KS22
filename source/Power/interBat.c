/*===========================================================================*/
/**
 * @file powerfail.c
 *
 *
 */
/*==========================================================================*/

/*===========================================================================*
 * Header Files
 *===========================================================================*/ 
#include    "standard.h"
//#define USE_DEBUG
#include "Debug.h"

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/
#define VOLTAGE_VAL_COPENSTATION 6 //0.06V voltage drop between BATT and FBATT
#define CHARGE_THRESHOLD 380 //Begin charge when detecting internal battery lower than 3.8V
#define UNCHARGE_THRESHOLD 410 //Stop charge when detecting internal battery higher than 4.1V
#define MAX_DEBOUNCE_COUNT 6 // 6*5ms =30 ms
     
#define write_timer(x, y)           (timer[x] = y)
#define read_timer(x)               (timer[x])
#define timer_running(x)            (timer[x] > OS_Time())

#define AD_EXT_BATTERY 0 /* battery voltage channel */
#define AD_INT_BATTERY 1 /* internal battery voltage channel */
//#define AD_GAIN() 2156 //for HW  =(65520*(40.2/(330+40.2)))/(3.3) ,theoretical value
#define EXT_AD_GAIN() 1805 //for HW  =(65535*(10/(100+10)))/(3.3) ,theoretical value
#define INT_AD_GAIN() 1805 //for HW  =(65535*(10/(100+10)))/(3.3) ,theoretical value


#define MAX_DEBOUNCE_WARNING_COUNT          12  

/*===========================================================================*
 * Local Type Declarations
 *===========================================================================*/
enum
{
    MUTE_DELAY_TIMER,
    NUM_PF_TIMERS
};

enum
{
    V_LOW,
    MUTE_LOW,
    MUTE_HIGH,
    V_HIGH,
    V_IGN_PULSE,  
    NUM_MAX_VOLTAGE_TRESHOLD
};

typedef struct PF_Voltage_Treshold_Limits_Tag
{
    uint16_t  lower_voltage;
    uint16_t  upper_voltage;
} PF_Voltage_Treshold_Limits_Type;

typedef struct PF_Voltage_Treshold_Data_Tag
{
    uint8_t   count;
    bool      lower_voltage_is;
} PF_Voltage_Treshold_Data_Type;

/*===========================================================================*
 * Local Object Definitions
 *===========================================================================*/
static const PF_Voltage_Treshold_Limits_Type voltage_limits[NUM_MAX_VOLTAGE_TRESHOLD] =
{
    { 500,  550 }, /* V_LOW          RESET threshold    */
    { 600,  650 }, /* MUTE_LOW    WARNING  threshold  */
    { 2600, 2650}, /* MUTE_HIGH  WARNING threshold */
    { 2650, 2700}  /* V_HIGH       RESET threshold */
};

static PF_Voltage_Treshold_Data_Type voltage_data[NUM_MAX_VOLTAGE_TRESHOLD] =
{
    {MAX_DEBOUNCE_COUNT, false},
    {12, false},
    {MAX_DEBOUNCE_COUNT, true },
    {MAX_DEBOUNCE_COUNT, true }
};

//static bool pwr_fail_mute_is;
//static bool pwr_fail_reset_is;
static Tick_Type timer[NUM_PF_TIMERS];
//static uint8_t pwr_warning_debounce = 30;

static uint32_t   batt_volt_avg;
static uint16_t   batt_volt_avg_last;
/*===========================================================================*
 * Local Function Prototypes
 *===========================================================================*/
static void Pwr_Fail_Voltage_Hysteresis(uint16_t voltage, uint8_t index);
bool Pwr_Fail_Is_Mute_Condition(void);
bool Pwr_Fail_Is_Reset_Condition (void);

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/**********************************************************************
*
*    Function: Pwr_Fail_Initialize
*
*  Parameters: none
*
*     Returns: none
*
* Description: initializes powerfail monitor module
*
**********************************************************************/
void Pwr_Fail_Initialize(void)
{
    /* configure AD-LVW interrupt */
    //AD_Interrupt_Configure();
    //AD_Interrupt_Enable();
}

/**********************************************************************
*
*    Function: Pwr_Fail_Shutdown
*
*  Parameters: none
*
*     Returns: none
*
* Description: This function configures powerfail module for idle mode.
*
*********************************************************************/
void Pwr_Fail_Shutdown(void)
{
    /* AD conversion interrupt does not work in STOP! */
    //AD_Interrupt_Disable();
}

/**********************************************************************
*
*    Function: Pwr_Fail_Monitor
*
*  Parameters: none
*
*     Returns: none
*
* Description: This function handles all powerfail cases.
*
*********************************************************************/
void Pwr_Fail_Monitor(void)
{
    Pwr_Fail_Check_Voltage();
}

/**********************************************************************
*
*    Function: Pwr_Fail_AD_get_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: get  the AD convertion result and convert to the value fo voltage.
*              example:1200 means 12.00v
**********************************************************************/
uint16_t Pwr_Fail_AD_get_Voltage(void)
{
    return (((uint32_t)(ADC_Read(AD_EXT_BATTERY))) * 100)/(EXT_AD_GAIN());
}

uint16_t Pwr_Fail_AD_get_Int_Voltage(void)
{
    return  (((uint32_t)(ADC_Read(AD_INT_BATTERY))) * 100)/(INT_AD_GAIN());
}

/**********************************************************************
*
*    Function: Pwr_Fail_Get_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: It is the average value of battery voltage in 20ms
**********************************************************************/
uint16_t Pwr_Fail_Get_Voltage(void)
{
    return batt_volt_avg_last;
}

/**********************************************************************
*
*    Function: Pwr_Fail_Check_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: Determine's if the voltage is above/below a certain votage.
*
**********************************************************************/
void Pwr_Fail_Check_Voltage(void)
{
    static uint32_t period_slice = 0;
    uint16_t batt_voltage;

    batt_voltage = Pwr_Fail_AD_get_Voltage();
    batt_volt_avg += batt_voltage;
    period_slice ++;
    if(period_slice%4 == 0)//20ms
    {
        batt_volt_avg_last = (batt_volt_avg >> 2);
        batt_volt_avg = 0;
    }

    /* first check all LOW thresholds on AN_VLOW line */
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, V_LOW   );
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, MUTE_LOW);

    /* then check all HIGH thresholds on AN_BATTERY line */
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, MUTE_HIGH);
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, V_HIGH   );

    /* then check ign thresholds on AN_BATTERY line */
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, V_IGN_PULSE   );
}

/**********************************************************************
*
*    Function: Pwr_Fail_Voltage_Hysteresis
*
*  Parameters: uint16_t voltage - voltage to process
*              uint8_t index    - index to voltage threshold table
*
*     Returns: none
*
* Description: runs the voltage hysteresis
*
**********************************************************************/
static void Pwr_Fail_Voltage_Hysteresis (uint16_t voltage, uint8_t index)
{
    uint8_t debounce = 0;

    if((index == (uint8_t)MUTE_LOW)||(index == (uint8_t)V_IGN_PULSE))
    {
        debounce = MAX_DEBOUNCE_WARNING_COUNT;
    }
    else
    {
        debounce = MAX_DEBOUNCE_COUNT;
    }

    if (voltage_data[index].lower_voltage_is == false)
    {
        if (voltage < voltage_limits[index].lower_voltage)
        {
            /*count down if voltage below */
            if (!voltage_data[index].count--)
            {
                voltage_data[index].count = debounce;
                voltage_data[index].lower_voltage_is = true;
            }
        }
        else if (voltage_data[index].count < debounce)
        {
            /* count up if voltage above */
            voltage_data[index].count++;
        }
    }
    else
    {
        if (voltage > voltage_limits[index].upper_voltage)
        {    
            /*count down if voltage above */
            if (!voltage_data[index].count--)
            {
                voltage_data[index].count = debounce;
                voltage_data[index].lower_voltage_is = false;
            }
        }
        else if (voltage_data[index].count < debounce)
        {
            /*count up if voltage below */
            voltage_data[index].count++;
        }
    }
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Mute_Condition
*
*  Parameters: none
*
*     Returns: true/false
*
* Description: returns powerfail mute condition
*
**********************************************************************/
bool Pwr_Fail_Is_Mute_Condition(void)
{
    return((voltage_data[MUTE_LOW].lower_voltage_is  == true)||
          (voltage_data[MUTE_HIGH].lower_voltage_is == false)) ;
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Reset_Condition
*
*  Parameters: none
*
*     Returns: true/false
*
* Description: returns powerfail reset condition
*
**********************************************************************/
bool Pwr_Fail_Is_Reset_Condition (void)
{
    return((voltage_data[V_LOW].lower_voltage_is  == true));
}
/**********************************************************************
*
*    Function: Pwr_Fail_Is_Voltage_Good
*
*  Parameters: None
*
*     Returns: true/false
*
* Description: returns if the voltage is over a working threshold
*
*********************************************************************/
bool Pwr_Fail_Is_Voltage_Good (void)
{
    return(!Pwr_Fail_Is_Mute_Condition() && !timer_running(MUTE_DELAY_TIMER));
}
/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *
 *********************************************************************/
