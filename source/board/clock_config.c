/*
 * How to setup clock using clock driver functions:
 *
 * 1. CLOCK_SetSimSafeDivs, to make sure core clock, bus clock, flexbus clock
 *    and flash clock are in allowed range during clock mode switch.
 *
 * 2. Call CLOCK_Osc0Init to setup OSC clock, if it is used in target mode.
 *
 * 3. Set MCG configuration, MCG includes three parts: FLL clock, PLL clock and
 *    internal reference clock(MCGIRCLK). Follow the steps to setup:
 *
 *    1). Call CLOCK_BootToXxxMode to set MCG to target mode.
 *
 *    2). If target mode is FBI/BLPI/PBI mode, the MCGIRCLK has been configured
 *        correctly. For other modes, need to call CLOCK_SetInternalRefClkConfig
 *        explicitly to setup MCGIRCLK.
 *
 *    3). Don't need to configure FLL explicitly, because if target mode is FLL
 *        mode, then FLL has been configured by the function CLOCK_BootToXxxMode,
 *        if the target mode is not FLL mode, the FLL is disabled.
 *
 *    4). If target mode is PEE/PBE/PEI/PBI mode, then the related PLL has been
 *        setup by CLOCK_BootToXxxMode. In FBE/FBI/FEE/FBE mode, the PLL could
 *        be enabled independently, call CLOCK_EnablePll0 explicitly in this case.
 *
 * 4. Call CLOCK_SetSimConfig to set the clock configuration in SIM.
 */

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v3.0
processor: MKS22FN256xxx12
package_id: MKS22FN256VLH12
mcu_data: ksdk2_0
processor_version: 2.0.0
board: MAPS-KS22
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

#include "fsl_smc.h"
#include "fsl_rtc.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2S_MCLK_AS_OUTPUT                                1U  /*!< I2S MCLK configured as output from the MCLK. MCLK divider is enabled. */
#define I2S_MCLK_DIV_20                                  19U  /*!< I2S MCLK divider: divided by  20 */
#define I2S_MCLK_DIV_24                                  23U  /*!< I2S MCLK divider: divided by  24 */
#define I2S_MCLK_FRAC_1                                   0U  /*!< I2S MCLK fractional divider: multiplied by  1 */
#define I2S_MCLK_FRAC_5                                   4U  /*!< I2S MCLK fractional divider: multiplied by  5 */
#define I2S_MCLK_INPUT_CLK_SEL_CORE_SYSTEM_CLK            0U  /*!< I2S MCLK input clock select: Core/system clock */
#define MCG_IRCLK_DISABLE                                 0U  /*!< MCGIRCLK disabled */
#define MCG_PLL_DISABLE                                   0U  /*!< MCGPLLCLK disabled */
#define OSC_ER_CLK_DISABLE                                0U  /*!< Disable external reference clock */
#define RTC_OSC_CAP_LOAD_0PF                            0x0U  /*!< RTC oscillator capacity load: 0pF */
#define RTC_RTC32KCLK_PERIPHERALS_ENABLED                 1U  /*!< RTC32KCLK to other peripherals: enabled */
#define SIM_CLKOUT_SEL_OSCERCLK_CLK                       6U  /*!< CLKOUT pin clock select: OSCERCLK clock */
#define SIM_FLEXIOS0_CLK_SEL_CORE_SYSTEM_CLK              0U  /*!< FLEXIOS0 clock select: Core/system clock */
#define SIM_FLEXIO_CLK_SEL_PLLFLLSEL_CLK                  1U  /*!< FLEXIO clock select: PLLFLLSEL output clock */
#define SIM_LPI2C_CLK_SEL_PLLFLLSEL_CLK                   1U  /*!< LPI2C clock select: PLLFLLSEL output clock */
#define SIM_LPUART_CLK_SEL_PLLFLLSEL_CLK                  1U  /*!< LPUART clock select: PLLFLLSEL output clock */
#define SIM_OSC32KSEL_RTC32KCLK_CLK                       2U  /*!< OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
#define SIM_PLLFLLSEL_DIV_0                               0U  /*!< PLLFLLSEL clock divider divisor: divided by 1 */
#define SIM_PLLFLLSEL_DIV_1                               1U  /*!< PLLFLLSEL clock divider divisor: divided by 2 */
#define SIM_PLLFLLSEL_FRAC_0                              0U  /*!< PLLFLLSEL clock divider fraction: multiplied by 1 */
#define SIM_PLLFLLSEL_IRC48MCLK_CLK                       3U  /*!< PLLFLL select: IRC48MCLK clock */
#define SIM_PLLFLLSEL_MCGFLLCLK_CLK                       0U  /*!< PLLFLL select: MCGFLLCLK clock */
#define SIM_PLLFLLSEL_MCGPLLCLK_CLK                       1U  /*!< PLLFLL select: MCGPLLCLK clock */
#define SIM_RTC_CLKOUT_SEL_RTC32KCLK_CLK                  1U  /*!< RTC clock output select: RTC32KCLK clock (32.768kHz) */
#define SIM_TPM_CLK_SEL_PLLFLLSEL_CLK                     1U  /*!< TPM clock select: PLLFLLSEL output clock */
#define SIM_TRACE_CLK_SEL_MCGOUTCLK_CLK                   0U  /*!< Trace clock select: FlexBus clock */
#define SIM_USB_CLK_120000000HZ                   120000000U  /*!< Input SIM frequency for USB: 120000000Hz */
#define SIM_USB_CLK_48000000HZ                     48000000U  /*!< Input SIM frequency for USB: 48000000Hz */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetRtcClock
 * Description   : This function is used to configuring RTC clock including 
 * enabling RTC oscillator.
 * Param capLoad : RTC oscillator capacity load
 * Param enableOutPeriph : Enable (1U)/Disable (0U) clock to peripherals
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetRtcClock(uint32_t capLoad, uint8_t enableOutPeriph)
{
  /* RTC clock gate enable */
  CLOCK_EnableClock(kCLOCK_Rtc0);
  if ((RTC->CR & RTC_CR_OSCE_MASK) == 0u) { /* Only if the Rtc oscillator is not already enabled */
    /* Set the specified capacitor configuration for the RTC oscillator */
    RTC_SetOscCapLoad(RTC, capLoad);
    /* Enable the RTC 32KHz oscillator */
    RTC->CR |= RTC_CR_OSCE_MASK;
  }
  /* Output to other peripherals */
  if (enableOutPeriph) {
    RTC->CR &= ~RTC_CR_CLKO_MASK;
  }
  else {
    RTC->CR |= RTC_CR_CLKO_MASK;
  }
  /* Set the XTAL32/RTC_CLKIN frequency based on board setting. */
  CLOCK_SetXtal32Freq(BOARD_XTAL32K_CLK_HZ);
  /* Set RTC_TSR if there is fault value in RTC */
  if (RTC->SR & RTC_SR_TIF_MASK) {
    RTC-> TSR = RTC -> TSR;
  }
  /* RTC clock gate disable */
  CLOCK_DisableClock(kCLOCK_Rtc0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetPllFllSelDivFrac
 * Description   : Configure PLLFLLSEL divider divison and fraction in SIM
 * Param div     : The value to set the divider division.
 * Param frac    : The value to set the divider fraction.
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetPllFllSelDivFrac(uint8_t div, uint8_t frac)
{
    SIM->CLKDIV3 = ((SIM->CLKDIV3 & ~(SIM_CLKDIV3_PLLFLLDIV_MASK | SIM_CLKDIV3_PLLFLLFRAC_MASK))
                   | SIM_CLKDIV3_PLLFLLDIV(div)
                   | SIM_CLKDIV3_PLLFLLFRAC(frac));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetI2s0MasterClock
 * Description   : Set SAI master clock generation.
 * Param source  : MCLK input clock select.
 * Param frac    : MCLK fraction.
 * Param div     : MCLK divide.
 * Param outputEnable : MCLK output enable. 0U-MCLK as input, 1U-MCLK as output.
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetI2s0MasterClock(uint8_t source, uint16_t frac, uint16_t div, uint8_t outputEnable)
{
    /* Enable the SAI clock */
    CLOCK_EnableClock(kCLOCK_Sai0);
    /* Configure Master clock output enable */
    I2S0->MCR = ((I2S0->MCR & ~I2S_MCR_MOE_MASK)) | I2S_MCR_MOE(outputEnable);
    /* Master clock source setting */
    I2S0->MCR = ((I2S0->MCR & ~I2S_MCR_MICS_MASK)) | I2S_MCR_MICS(source);
    /* Check if master clock divider enabled, then set master clock divider */
    if (I2S0->MCR & I2S_MCR_MOE_MASK)
    {
        /* Set fract and divider */
        I2S0->MDR = ((I2S0->MDR & ~(I2S_MDR_DIVIDE_MASK | I2S_MDR_FRACT_MASK))
                    | I2S_MDR_DIVIDE(div)
                    | I2S_MDR_FRACT(frac));
        /* Waiting for the divider updated */
        while (I2S0->MCR & I2S_MCR_DUF_MASK)
        {
        }
    }
    /* Disable the SAI clock */
    CLOCK_DisableClock(kCLOCK_Sai0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetFlexioS0Clock
 * Description   : Set FlexIOS0 clock source.
 * Param src     : The value to set FlexIOS0 clock source.
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetFlexioS0Clock(uint8_t src)
{
    SIM->MISCCTL = ((SIM->MISCCTL & ~SIM_MISCCTL_FlexIOS0_MASK) | SIM_MISCCTL_FlexIOS0(src));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetFllExtRefDiv
 * Description   : Configure FLL external reference divider (FRDIV).
 * Param frdiv   : The value to set FRDIV.
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetFllExtRefDiv(uint8_t frdiv)
{
    MCG->C1 = ((MCG->C1 & ~MCG_C1_FRDIV_MASK) | MCG_C1_FRDIV(frdiv));
}

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}

/*******************************************************************************
 ********************* Configuration BAORD_BootClockVLPR ***********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BAORD_BootClockVLPR
outputs:
- {id: Bus_clock.outFreq, value: 4 MHz}
- {id: Core_clock.outFreq, value: 4 MHz}
- {id: Flash_clock.outFreq, value: 800 kHz}
- {id: LPO_clock.outFreq, value: 1 kHz}
- {id: System_clock.outFreq, value: 4 MHz}
settings:
- {id: MCGMode, value: BLPI}
- {id: powerMode, value: HSRUN}
- {id: MCG.CLKS.sel, value: MCG.IRCS}
- {id: MCG.FCRDIV.scale, value: '1', locked: true}
- {id: MCG.IRCS.sel, value: MCG.FCRDIV}
- {id: MCG_C2_OSC_MODE_CFG, value: ModeOscLowPower}
- {id: OSC_CR_SYS_OSC_CAP_LOAD_CFG, value: SC16PF}
- {id: RTCCLKOUTConfig, value: 'yes'}
- {id: RTC_CR_OSCE_CFG, value: Enabled}
- {id: RTC_CR_OSC_CAP_LOAD_CFG, value: SC22PF}
- {id: SIM.OSC32KSEL.sel, value: RTC.RTC32KCLK}
- {id: SIM.OUTDIV4.scale, value: '5', locked: true}
sources:
- {id: OSC.OSC.outFreq, value: 32.768 kHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BAORD_BootClockVLPR configuration
 ******************************************************************************/
const mcg_config_t mcgConfig_BAORD_BootClockVLPR =
    {
        .mcgMode = kMCG_ModeBLPI,                 /* BLPI - Bypassed Low Power Internal */
        .irclkEnableMode = MCG_IRCLK_DISABLE,     /* MCGIRCLK disabled */
        .ircs = kMCG_IrcFast,                     /* Fast internal reference clock selected */
        .fcrdiv = 0x0U,                           /* Fast IRC divider: divided by 1 */
        .frdiv = 0x0U,                            /* FLL reference clock divider: divided by 1 */
        .drs = kMCG_DrsLow,                       /* Low frequency range */
        .dmx32 = kMCG_Dmx32Default,               /* DCO has a default range of 25% */
        .oscsel = kMCG_OscselOsc,                 /* Selects System Oscillator (OSCCLK) */
        .pll0Config =
            {
                .enableMode = MCG_PLL_DISABLE,    /* MCGPLLCLK disabled */
                .prdiv = 0x0U,                    /* PLL Reference divider: divided by 1 */
                .vdiv = 0x0U,                     /* VCO divider: multiplied by 24 */
            },
    };
const sim_clock_config_t simConfig_BAORD_BootClockVLPR =
    {
        .pllFllSel = SIM_PLLFLLSEL_MCGFLLCLK_CLK, /* PLLFLL select: MCGFLLCLK clock */
        .er32kSrc = SIM_OSC32KSEL_RTC32KCLK_CLK,  /* OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
        .clkdiv1 = 0x40000U,                      /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /1, OUTDIV4: /5 */
    };
const osc_config_t oscConfig_BAORD_BootClockVLPR =
    {
        .freq = 0U,                               /* Oscillator frequency: 0Hz */
        .capLoad = (kOSC_Cap16P),                 /* Oscillator capacity load: 16pF */
        .workMode = kOSC_ModeOscLowPower,         /* Oscillator low power */
        .oscerConfig =
            {
                .enableMode = OSC_ER_CLK_DISABLE, /* Disable external reference clock */
                .erclkDiv = 0,                    /* Divider for OSCERCLK: divided by 1 */
            }
    };

/*******************************************************************************
 * Code for BAORD_BootClockVLPR configuration
 ******************************************************************************/
void BAORD_BootClockVLPR(void)
{
    /* Set HSRUN power mode */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }
    /* Set the system clock dividers in SIM to safe value. */
    CLOCK_SetSimSafeDivs();
    /* Set MCG to BLPI mode. */
    CLOCK_BootToBlpiMode(mcgConfig_BAORD_BootClockVLPR.fcrdiv,
                         mcgConfig_BAORD_BootClockVLPR.ircs,
                         mcgConfig_BAORD_BootClockVLPR.irclkEnableMode);
    /* Set the clock configuration in SIM module. */
    CLOCK_SetSimConfig(&simConfig_BAORD_BootClockVLPR);
    /* Set the PLLFLLDIV and PLLFLLFRAC in SIM module. */
    CLOCK_CONFIG_SetPllFllSelDivFrac(SIM_PLLFLLSEL_DIV_0, 
                                     SIM_PLLFLLSEL_FRAC_0);
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BAORD_BOOTCLOCKVLPR_CORE_CLOCK;
}

/*******************************************************************************
 ********************* Configuration BOARD_BootClockHSRUN **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockHSRUN
outputs:
- {id: Bus_clock.outFreq, value: 60 MHz}
- {id: CLKOUT.outFreq, value: 16 MHz}
- {id: Core_clock.outFreq, value: 120 MHz}
- {id: ERCLK32K.outFreq, value: 32.768 kHz}
- {id: FLEXIOCLK.outFreq, value: 120 MHz}
- {id: Flash_clock.outFreq, value: 24 MHz}
- {id: LPI2C0CLK.outFreq, value: 60 MHz}
- {id: LPI2C1CLK.outFreq, value: 60 MHz}
- {id: LPO_clock.outFreq, value: 1 kHz}
- {id: LPUARTCLK.outFreq, value: 120 MHz}
- {id: MCGIRCLK.outFreq, value: 4 MHz}
- {id: OSCERCLK.outFreq, value: 16 MHz}
- {id: OSCERCLK_UNDIV.outFreq, value: 16 MHz}
- {id: PLLFLLCLK.outFreq, value: 120 MHz}
- {id: RTC_CLKOUT.outFreq, value: 32.768 kHz}
- {id: System_clock.outFreq, value: 120 MHz}
- {id: TPMCLK.outFreq, value: 60 MHz}
- {id: TRACECLKIN.outFreq, value: 120 MHz}
- {id: USB48MCLK.outFreq, value: 48 MHz}
settings:
- {id: MCGMode, value: PEE}
- {id: powerMode, value: HSRUN}
- {id: CLKOUTConfig, value: 'yes'}
- {id: FLEXIOClkConfig, value: FlexIO_I2S0}
- {id: LPI2C0ClkConfig, value: 'yes'}
- {id: LPI2C1ClkConfig, value: 'yes'}
- {id: LPUARTClkConfig, value: 'yes'}
- {id: MCG.FCRDIV.scale, value: '1', locked: true}
- {id: MCG.FLL_mul.scale, value: '640', locked: true}
- {id: MCG.FRDIV.scale, value: '64'}
- {id: MCG.IRCS.sel, value: MCG.FCRDIV}
- {id: MCG.IREFS.sel, value: MCG.FRDIV}
- {id: MCG.PLLS.sel, value: MCG.PLL}
- {id: MCG.PRDIV.scale, value: '4', locked: true}
- {id: MCG.VDIV.scale, value: '30', locked: true}
- {id: MCG_C1_IRCLKEN_CFG, value: Enabled}
- {id: MCG_C2_OSC_MODE_CFG, value: ModeOscLowPower}
- {id: MCG_C2_RANGE0_CFG, value: Very_high}
- {id: MCG_C2_RANGE0_FRDIV_CFG, value: Very_high}
- {id: OSC.ERPS.scale, value: '1', locked: true}
- {id: OSC_CR_ERCLKEN_CFG, value: Enabled}
- {id: OSC_CR_ERCLKEN_UNDIV_CFG, value: Enabled}
- {id: OSC_CR_SYS_OSC_CAP_LOAD_CFG, value: SC16PF}
- {id: RTCCLKOUTConfig, value: 'yes'}
- {id: RTC_CR_OSCE_CFG, value: Enabled}
- {id: SIM.CLKOUTSEL.sel, value: OSC.OSCERCLK}
- {id: SIM.FLEXIOSRCSEL.sel, value: SIM.PLLFLLSEL}
- {id: SIM.I2S0_DIVIDE.scale, value: '24', locked: true}
- {id: SIM.I2S0_FRACT.scale, value: '5', locked: true}
- {id: SIM.I2S0_MOESEL.sel, value: SIM.I2S0_DIVIDE}
- {id: SIM.LPI2C0SRCSEL.sel, value: SIM.PLLFLLDIV}
- {id: SIM.LPI2C1SRCSEL.sel, value: SIM.PLLFLLDIV}
- {id: SIM.LPUARTSRCSEL.sel, value: SIM.PLLFLLSEL}
- {id: SIM.OSC32KSEL.sel, value: RTC.RTC32KCLK}
- {id: SIM.OUTDIV1.scale, value: '1', locked: true}
- {id: SIM.OUTDIV2.scale, value: '2', locked: true}
- {id: SIM.OUTDIV4.scale, value: '5', locked: true}
- {id: SIM.PLLFLLDIV.scale, value: '2', locked: true}
- {id: SIM.PLLFLLSEL.sel, value: MCG.MCGPLLCLK}
- {id: SIM.RTCCLKOUTSEL.sel, value: RTC.RTC32KCLK}
- {id: SIM.TPMSRCSEL.sel, value: SIM.PLLFLLDIV}
- {id: SIM.TRACECLKSEL.sel, value: MCG.MCGOUTCLK}
- {id: SIM.USBDIV.scale, value: '5', locked: true}
- {id: SIM.USBFRAC.scale, value: '2', locked: true}
- {id: SIM.USBSRCSEL.sel, value: SIM.USBDIV}
- {id: TPMClkConfig, value: 'yes'}
- {id: TraceClkConfig, value: 'yes'}
- {id: USBClkConfig, value: 'yes'}
sources:
- {id: OSC.OSC.outFreq, value: 16 MHz, enabled: true}
- {id: RTC.RTC32kHz.outFreq, value: 32.768 kHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockHSRUN configuration
 ******************************************************************************/
const mcg_config_t mcgConfig_BOARD_BootClockHSRUN =
    {
        .mcgMode = kMCG_ModePEE,                  /* PEE - PLL Engaged External */
        .irclkEnableMode = kMCG_IrclkEnable,      /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
        .ircs = kMCG_IrcFast,                     /* Fast internal reference clock selected */
        .fcrdiv = 0x0U,                           /* Fast IRC divider: divided by 1 */
        .frdiv = 0x1U,                            /* FLL reference clock divider: divided by 64 */
        .drs = kMCG_DrsLow,                       /* Low frequency range */
        .dmx32 = kMCG_Dmx32Default,               /* DCO has a default range of 25% */
        .oscsel = kMCG_OscselOsc,                 /* Selects System Oscillator (OSCCLK) */
        .pll0Config =
            {
                .enableMode = MCG_PLL_DISABLE,    /* MCGPLLCLK disabled */
                .prdiv = 0x3U,                    /* PLL Reference divider: divided by 4 */
                .vdiv = 0x6U,                     /* VCO divider: multiplied by 30 */
            },
    };
const sim_clock_config_t simConfig_BOARD_BootClockHSRUN =
    {
        .pllFllSel = SIM_PLLFLLSEL_MCGPLLCLK_CLK, /* PLLFLL select: MCGPLLCLK clock */
        .er32kSrc = SIM_OSC32KSEL_RTC32KCLK_CLK,  /* OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
        .clkdiv1 = 0x1040000U,                    /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /2, OUTDIV4: /5 */
    };
const osc_config_t oscConfig_BOARD_BootClockHSRUN =
    {
        .freq = 16000000U,                        /* Oscillator frequency: 16000000Hz */
        .capLoad = (kOSC_Cap16P),                 /* Oscillator capacity load: 16pF */
        .workMode = kOSC_ModeOscLowPower,         /* Oscillator low power */
        .oscerConfig =
            {
                .enableMode = kOSC_ErClkEnable,   /* Enable external reference clock, disable external reference clock in STOP mode */
                .erclkDiv = 0,                    /* Divider for OSCERCLK: divided by 1 */
            }
    };

/*******************************************************************************
 * Code for BOARD_BootClockHSRUN configuration
 ******************************************************************************/
void BOARD_BootClockHSRUN(void)
{
    /* Set HSRUN power mode */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }
    /* Set the system clock dividers in SIM to safe value. */
    CLOCK_SetSimSafeDivs();
    /* Configure RTC clock including enabling RTC oscillator. */
    CLOCK_CONFIG_SetRtcClock(RTC_OSC_CAP_LOAD_0PF, RTC_RTC32KCLK_PERIPHERALS_ENABLED);
    /* Initializes OSC0 according to board configuration. */
    CLOCK_InitOsc0(&oscConfig_BOARD_BootClockHSRUN);
    CLOCK_SetXtal0Freq(oscConfig_BOARD_BootClockHSRUN.freq);
    /* Configure the Internal Reference clock (MCGIRCLK). */
    CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_BootClockHSRUN.irclkEnableMode,
                                  mcgConfig_BOARD_BootClockHSRUN.ircs, 
                                  mcgConfig_BOARD_BootClockHSRUN.fcrdiv);
    /* Configure FLL external reference divider (FRDIV). */
    CLOCK_CONFIG_SetFllExtRefDiv(mcgConfig_BOARD_BootClockHSRUN.frdiv);
    /* Set MCG to PEE mode. */
    CLOCK_BootToPeeMode(mcgConfig_BOARD_BootClockHSRUN.oscsel,
                        kMCG_PllClkSelPll0,
                        &mcgConfig_BOARD_BootClockHSRUN.pll0Config);
    /* Set the clock configuration in SIM module. */
    CLOCK_SetSimConfig(&simConfig_BOARD_BootClockHSRUN);
    /* Set the PLLFLLDIV and PLLFLLFRAC in SIM module. */
    CLOCK_CONFIG_SetPllFllSelDivFrac(SIM_PLLFLLSEL_DIV_1, 
                                     SIM_PLLFLLSEL_FRAC_0);
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKHSRUN_CORE_CLOCK;
    /* Set RTC_CLKOUT source. */
    CLOCK_SetRtcClkOutClock(SIM_RTC_CLKOUT_SEL_RTC32KCLK_CLK);
    /* Enable USB FS clock. */
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0, SIM_USB_CLK_120000000HZ);
    /* Set LPUART clock source. */
    CLOCK_SetLpuart0Clock(SIM_LPUART_CLK_SEL_PLLFLLSEL_CLK);
    /* Set LPI2C0 clock source. */
    CLOCK_SetLpi2c0Clock(SIM_LPI2C_CLK_SEL_PLLFLLSEL_CLK);
    /* Set LPI2C1 clock source. */
    CLOCK_SetLpi2c1Clock(SIM_LPI2C_CLK_SEL_PLLFLLSEL_CLK);
    /* Set I2S/SAI MCLK clock */
    CLOCK_CONFIG_SetI2s0MasterClock(I2S_MCLK_INPUT_CLK_SEL_CORE_SYSTEM_CLK, 
                                    I2S_MCLK_FRAC_5, 
                                    I2S_MCLK_DIV_24, 
                                    I2S_MCLK_AS_OUTPUT);
    /* Set FLEXIOS0 clock source. */
    CLOCK_CONFIG_SetFlexioS0Clock(SIM_FLEXIOS0_CLK_SEL_CORE_SYSTEM_CLK);
    /* Set FLEXIO clock source. */
    CLOCK_SetFlexio0Clock(SIM_FLEXIO_CLK_SEL_PLLFLLSEL_CLK);
    /* Set TPM clock source. */
    CLOCK_SetTpmClock(SIM_TPM_CLK_SEL_PLLFLLSEL_CLK);
    /* Set CLKOUT source. */
    CLOCK_SetClkOutClock(SIM_CLKOUT_SEL_OSCERCLK_CLK);
    /* Set debug trace clock source. */
    CLOCK_SetTraceClock(SIM_TRACE_CLK_SEL_MCGOUTCLK_CLK);
}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockRUN
called_from_default_init: true
outputs:
- {id: Bus_clock.outFreq, value: 36 MHz}
- {id: CLKOUT.outFreq, value: 16 MHz}
- {id: Core_clock.outFreq, value: 72 MHz}
- {id: ERCLK32K.outFreq, value: 32.768 kHz}
- {id: FLEXIOCLK.outFreq, value: 48 MHz}
- {id: Flash_clock.outFreq, value: 24 MHz}
- {id: IRC48MCLK.outFreq, value: 48 MHz}
- {id: LPI2C0CLK.outFreq, value: 24 MHz}
- {id: LPI2C1CLK.outFreq, value: 24 MHz}
- {id: LPO_clock.outFreq, value: 1 kHz, locked: true, accuracy: '0.001'}
- {id: LPUARTCLK.outFreq, value: 48 MHz}
- {id: MCGIRCLK.outFreq, value: 4 MHz}
- {id: OSCERCLK.outFreq, value: 16 MHz}
- {id: OSCERCLK_UNDIV.outFreq, value: 16 MHz}
- {id: PLLFLLCLK.outFreq, value: 48 MHz}
- {id: RTC_CLKOUT.outFreq, value: 32.768 kHz}
- {id: System_clock.outFreq, value: 72 MHz}
- {id: TPMCLK.outFreq, value: 24 MHz}
- {id: TRACECLKIN.outFreq, value: 72 MHz}
- {id: USB48MCLK.outFreq, value: 48 MHz}
settings:
- {id: MCGMode, value: PEE}
- {id: CLKOUTConfig, value: 'yes'}
- {id: FLEXIOClkConfig, value: FlexIO_I2S0}
- {id: LPI2C0ClkConfig, value: 'yes'}
- {id: LPI2C1ClkConfig, value: 'yes'}
- {id: LPUARTClkConfig, value: 'yes'}
- {id: MCG.FCRDIV.scale, value: '1', locked: true}
- {id: MCG.FLL_mul.scale, value: '640', locked: true}
- {id: MCG.FRDIV.scale, value: '64', locked: true}
- {id: MCG.IRCS.sel, value: MCG.FCRDIV}
- {id: MCG.IREFS.sel, value: MCG.FRDIV}
- {id: MCG.PLLS.sel, value: MCG.PLL}
- {id: MCG.PRDIV.scale, value: '6', locked: true}
- {id: MCG.VDIV.scale, value: '27', locked: true}
- {id: MCG_C1_IRCLKEN_CFG, value: Enabled}
- {id: MCG_C2_OSC_MODE_CFG, value: ModeOscLowPower}
- {id: MCG_C2_RANGE0_CFG, value: Very_high}
- {id: MCG_C2_RANGE0_FRDIV_CFG, value: Very_high}
- {id: OSC_CR_ERCLKEN_CFG, value: Enabled}
- {id: OSC_CR_ERCLKEN_UNDIV_CFG, value: Enabled}
- {id: OSC_CR_SYS_OSC_CAP_LOAD_CFG, value: SC16PF}
- {id: RTCCLKOUTConfig, value: 'yes'}
- {id: RTC_CR_OSCE_CFG, value: Enabled}
- {id: SIM.CLKOUTSEL.sel, value: OSC.OSCERCLK}
- {id: SIM.FLEXIOSRCSEL.sel, value: SIM.PLLFLLSEL}
- {id: SIM.I2S0_DIVIDE.scale, value: '20', locked: true}
- {id: SIM.I2S0_MOESEL.sel, value: SIM.I2S0_DIVIDE}
- {id: SIM.LPI2C0SRCSEL.sel, value: SIM.PLLFLLDIV}
- {id: SIM.LPI2C1SRCSEL.sel, value: SIM.PLLFLLDIV}
- {id: SIM.LPUARTSRCSEL.sel, value: SIM.PLLFLLSEL}
- {id: SIM.OSC32KSEL.sel, value: RTC.RTC32KCLK}
- {id: SIM.OUTDIV1.scale, value: '1', locked: true}
- {id: SIM.OUTDIV2.scale, value: '2', locked: true}
- {id: SIM.OUTDIV4.scale, value: '3', locked: true}
- {id: SIM.PLLFLLDIV.scale, value: '2', locked: true}
- {id: SIM.PLLFLLSEL.sel, value: IRC48M.IRC48MCLK}
- {id: SIM.RTCCLKOUTSEL.sel, value: RTC.RTC32KCLK}
- {id: SIM.TPMSRCSEL.sel, value: SIM.PLLFLLDIV}
- {id: SIM.TRACECLKSEL.sel, value: MCG.MCGOUTCLK}
- {id: SIM.USBDIV.scale, value: '1', locked: true}
- {id: SIM.USBFRAC.scale, value: '1', locked: true}
- {id: TPMClkConfig, value: 'yes'}
- {id: TraceClkConfig, value: 'yes'}
- {id: USBClkConfig, value: 'yes'}
sources:
- {id: IRC48M.IRC48M.outFreq, value: 48 MHz}
- {id: OSC.OSC.outFreq, value: 16 MHz, enabled: true}
- {id: RTC.RTC32kHz.outFreq, value: 32.768 kHz, enabled: true}
- {id: SIM.USBCLK_EXT.outFreq, value: 48 MHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
const mcg_config_t mcgConfig_BOARD_BootClockRUN =
    {
        .mcgMode = kMCG_ModePEE,                  /* PEE - PLL Engaged External */
        .irclkEnableMode = kMCG_IrclkEnable,      /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
        .ircs = kMCG_IrcFast,                     /* Fast internal reference clock selected */
        .fcrdiv = 0x0U,                           /* Fast IRC divider: divided by 1 */
        .frdiv = 0x1U,                            /* FLL reference clock divider: divided by 64 */
        .drs = kMCG_DrsLow,                       /* Low frequency range */
        .dmx32 = kMCG_Dmx32Default,               /* DCO has a default range of 25% */
        .oscsel = kMCG_OscselOsc,                 /* Selects System Oscillator (OSCCLK) */
        .pll0Config =
            {
                .enableMode = MCG_PLL_DISABLE,    /* MCGPLLCLK disabled */
                .prdiv = 0x5U,                    /* PLL Reference divider: divided by 6 */
                .vdiv = 0x3U,                     /* VCO divider: multiplied by 27 */
            },
    };
const sim_clock_config_t simConfig_BOARD_BootClockRUN =
    {
        .pllFllSel = SIM_PLLFLLSEL_IRC48MCLK_CLK, /* PLLFLL select: IRC48MCLK clock */
        .er32kSrc = SIM_OSC32KSEL_RTC32KCLK_CLK,  /* OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
        .clkdiv1 = 0x1020000U,                    /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /2, OUTDIV4: /3 */
    };
const osc_config_t oscConfig_BOARD_BootClockRUN =
    {
        .freq = 16000000U,                        /* Oscillator frequency: 16000000Hz */
        .capLoad = (kOSC_Cap16P),                 /* Oscillator capacity load: 16pF */
        .workMode = kOSC_ModeOscLowPower,         /* Oscillator low power */
        .oscerConfig =
            {
                .enableMode = kOSC_ErClkEnable,   /* Enable external reference clock, disable external reference clock in STOP mode */
                .erclkDiv = 0,                    /* Divider for OSCERCLK: divided by 1 */
            }
    };

/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
    /* Set the system clock dividers in SIM to safe value. */
    CLOCK_SetSimSafeDivs();
    /* Configure RTC clock including enabling RTC oscillator. */
    CLOCK_CONFIG_SetRtcClock(RTC_OSC_CAP_LOAD_0PF, RTC_RTC32KCLK_PERIPHERALS_ENABLED);
    /* Initializes OSC0 according to board configuration. */
    CLOCK_InitOsc0(&oscConfig_BOARD_BootClockRUN);
    CLOCK_SetXtal0Freq(oscConfig_BOARD_BootClockRUN.freq);
    /* Configure the Internal Reference clock (MCGIRCLK). */
    CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_BootClockRUN.irclkEnableMode,
                                  mcgConfig_BOARD_BootClockRUN.ircs, 
                                  mcgConfig_BOARD_BootClockRUN.fcrdiv);
    /* Configure FLL external reference divider (FRDIV). */
    CLOCK_CONFIG_SetFllExtRefDiv(mcgConfig_BOARD_BootClockRUN.frdiv);
    /* Set MCG to PEE mode. */
    CLOCK_BootToPeeMode(mcgConfig_BOARD_BootClockRUN.oscsel,
                        kMCG_PllClkSelPll0,
                        &mcgConfig_BOARD_BootClockRUN.pll0Config);
    /* Set the clock configuration in SIM module. */
    CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
    /* Set the PLLFLLDIV and PLLFLLFRAC in SIM module. */
    CLOCK_CONFIG_SetPllFllSelDivFrac(SIM_PLLFLLSEL_DIV_1, 
                                     SIM_PLLFLLSEL_FRAC_0);
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
    /* Set RTC_CLKOUT source. */
    CLOCK_SetRtcClkOutClock(SIM_RTC_CLKOUT_SEL_RTC32KCLK_CLK);
    /* Enable USB FS clock. */
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcExt, SIM_USB_CLK_48000000HZ);
    /* Set LPUART clock source. */
    CLOCK_SetLpuart0Clock(SIM_LPUART_CLK_SEL_PLLFLLSEL_CLK);
    /* Set LPI2C0 clock source. */
    CLOCK_SetLpi2c0Clock(SIM_LPI2C_CLK_SEL_PLLFLLSEL_CLK);
    /* Set LPI2C1 clock source. */
    CLOCK_SetLpi2c1Clock(SIM_LPI2C_CLK_SEL_PLLFLLSEL_CLK);
    /* Set I2S/SAI MCLK clock */
    CLOCK_CONFIG_SetI2s0MasterClock(I2S_MCLK_INPUT_CLK_SEL_CORE_SYSTEM_CLK, 
                                    I2S_MCLK_FRAC_1, 
                                    I2S_MCLK_DIV_20, 
                                    I2S_MCLK_AS_OUTPUT);
    /* Set FLEXIOS0 clock source. */
    CLOCK_CONFIG_SetFlexioS0Clock(SIM_FLEXIOS0_CLK_SEL_CORE_SYSTEM_CLK);
    /* Set FLEXIO clock source. */
    CLOCK_SetFlexio0Clock(SIM_FLEXIO_CLK_SEL_PLLFLLSEL_CLK);
    /* Set TPM clock source. */
    CLOCK_SetTpmClock(SIM_TPM_CLK_SEL_PLLFLLSEL_CLK);
    /* Set CLKOUT source. */
    CLOCK_SetClkOutClock(SIM_CLKOUT_SEL_OSCERCLK_CLK);
    /* Set debug trace clock source. */
    CLOCK_SetTraceClock(SIM_TRACE_CLK_SEL_MCGOUTCLK_CLK);
}

