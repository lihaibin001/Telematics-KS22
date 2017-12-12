#include "fsl_common.h"
#include "fsl_smc.h"
#include "fsl_llwu.h"
#include "fsl_rcm.h"
#include "fsl_lptmr.h"
#include "fsl_port.h"
#include "power_mode_switch.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "fsl_gpio.h"
#include "fsl_pmc.h"
#include "fsl_uart.h"
#include "clock_config.h"
#include "ACC_Detect.h"
#include "BMS_Detect.h"
#include "debug.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
*    Function: PMS_SetWakeupConfig
*
*  Parameters: 'WU_From' unused
*     Returns: none
* Description: Configures the wakeup socurce.
*******************************************************************************/
void PMS_SetWakeupConfig(app_power_mode_t WU_From)
{
    LLWU_SetExternalWakeupPinMode(LLWU, BAT_LLWU_WAKEUP_PIN_IDX, BAT_LLWU_WAKEUP_PIN_TYPE);
    LLWU_SetExternalWakeupPinMode(LLWU, ACC_LLWU_WAKEUP_PIN_IDX, ACC_LLWU_WAKEUP_PIN_TYPE);
    LLWU_SetExternalWakeupPinMode(LLWU, BMS_LLWU_WAKEUP_PIN_IDX, BMS_LLWU_WAKEUP_PIN_TYPE);
    NVIC_ClearPendingIRQ(LLWU_IRQn);
    NVIC_EnableIRQ(LLWU_IRQn);
}

/*******************************************************************************
*    Function: PMS_CheckPowerMode
*
*  Parameters: 'curPowerState' ..
*               'targetPowerMode' ..
*     Returns: none
* Description: Verify if current power state can transmite to the target mode
*******************************************************************************/
bool PMS_CheckPowerMode(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode)
{
    bool modeValid = true;
    switch (curPowerState)
    {
        case kSMC_PowerStateHsrun:
            if (kAPP_PowerModeRun != targetPowerMode)
            {
                DEBUG(DEBUG_HIGH, 
                      "Could only enter HSRUN mode from RUN mode.\r\n");
                modeValid = false;
            }
            break;

        case kSMC_PowerStateRun:
            if (kAPP_PowerModeVlpw == targetPowerMode)
            {
                DEBUG(DEBUG_HIGH, 
                      "Could not enter VLPW mode from RUN mode.\r\n");
                modeValid = false;
            }
            break;

        case kSMC_PowerStateVlpr:
            if ((kAPP_PowerModeWait == targetPowerMode) || 
                (kAPP_PowerModeHsrun == targetPowerMode) ||
                (kAPP_PowerModeStop == targetPowerMode))
            {
                DEBUG(DEBUG_HIGH, 
                      "Could not enter VLPW mode from RUN mode.\r\n");
                modeValid = false;
            }
            break;
        default:
            DEBUG(DEBUG_HIGH, 
                  "Wrong power state.\r\n");
            modeValid = false;
            break;
    }

    if (!modeValid)
    {
        return false;
    }

    /* Don't need to change power mode if current mode is already the target mode. */
    if (((kAPP_PowerModeRun == targetPowerMode) && (kSMC_PowerStateRun == curPowerState)) ||
        ((kAPP_PowerModeHsrun == targetPowerMode) && (kSMC_PowerStateHsrun == curPowerState)) ||
        ((kAPP_PowerModeVlpr == targetPowerMode) && (kSMC_PowerStateVlpr == curPowerState)))
    {
        DEBUG(DEBUG_HIGH, 
              "Already in the target power mode.\r\n");
        return false;
    }

    return true;
}

/*******************************************************************************
*    Function: PMS_PowerModeSwitch
*
*  Parameters: 'curPowerState' ..
*               'targetPowerMode' ..
*     Returns: none
* Description: Switch the current power mode to the target mode
*******************************************************************************/
void PMS_PowerModeSwitch(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode)
{
    smc_power_mode_vlls_config_t vlls_config;
    vlls_config.enablePorDetectInVlls0 = true;
    vlls_config.enableLpoClock = true; /*!< Enable LPO clock in VLLS mode */

    smc_power_mode_lls_config_t lls_config;
    lls_config.enableLpoClock = true;
    lls_config.subMode = kSMC_StopSub3;
    //DEBUG(DEBUG_LOW, "[PMS] Switch from %d to %d\r\n",curPowerState, targetPowerMode);
    switch (targetPowerMode)
    {
        case kAPP_PowerModeVlpr:
            MCG_SetClockVlpr();
            SMC_SetPowerModeVlpr(SMC);
            while (kSMC_PowerStateVlpr != SMC_GetPowerModeState(SMC))
            {
            }
            break;

        case kAPP_PowerModeRun:
            /* If enter RUN from HSRUN, fisrt change clock. */
            if (kSMC_PowerStateHsrun == curPowerState)
            {
                MCG_SetClockRunFromHsrun();
            }

            /* Power mode change. */
            SMC_SetPowerModeRun(SMC);
            while (kSMC_PowerStateRun != SMC_GetPowerModeState(SMC))
            {
            }

            /* If enter RUN from VLPR, change clock after the power mode change. */
            if (kSMC_PowerStateVlpr == curPowerState)
            {
                MCG_SetClockRunFromVlpr();
            }
            break;

        case kAPP_PowerModeHsrun:
            SMC_SetPowerModeHsrun(SMC);
            while (kSMC_PowerStateHsrun != SMC_GetPowerModeState(SMC))
            {
            }

            MCG_SetClockHsrun(); /* Change clock setting after power mode change. */
            break;

        case kAPP_PowerModeWait:
            SMC_SetPowerModeWait(SMC);
            break;

        case kAPP_PowerModeStop:
            SMC_SetPowerModeStop(SMC, kSMC_PartialStop);
            break;

        case kAPP_PowerModeVlpw:
            SMC_SetPowerModeVlpw(SMC);
            break;

        case kAPP_PowerModeVlps:
            SMC_SetPowerModeVlps(SMC);
            break;

        case kAPP_PowerModeLls:
            SMC_SetPowerModeLls(SMC, &lls_config);
            break;

        case kAPP_PowerModeVlls0:
            vlls_config.subMode = kSMC_StopSub0;
            SMC_SetPowerModeVlls(SMC, &vlls_config);
            break;

        case kAPP_PowerModeVlls1:
            vlls_config.subMode = kSMC_StopSub1;
            SMC_SetPowerModeVlls(SMC, &vlls_config);
            break;

        case kAPP_PowerModeVlls2:
            vlls_config.subMode = kSMC_StopSub2;
            SMC_SetPowerModeVlls(SMC, &vlls_config);
            break;

        case kAPP_PowerModeVlls3:
            vlls_config.subMode = kSMC_StopSub3;
            SMC_SetPowerModeVlls(SMC, &vlls_config);
            break;

        default:
            //PRINTF("Wrong value");
            break;
    }
}

