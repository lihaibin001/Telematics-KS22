#include "uart.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "canif_cbk.h"
/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "canif.h"
#include "canif_cfg.h"
#include "pdur_pbCfg.h"
#include "Com_PbCfg.h"

#include "vehicle.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void LED_Toggle(TimerHandle_t xTimer);

/*******************************************************************************
 * Variables
 ******************************************************************************/
void * const led_toggle_timer_id;
TimerHandle_t  timer0;

extern const Can_HardwareObjectType CanHardwareObjectConfig_Controller_1[];
extern const Can_ControllerConfigType CanControllerConfigData[];
extern const Can_CallbackType CanCallbackConfigData;
//extern const Com_ConfigType ComConfiguration;
const Can_ConfigSetType TestCanConfigSetData =
{
  .CanController =	CanControllerConfigData,
  .CanCallbacks =	&CanCallbackConfigData,
};

const Can_ConfigType TestCanConfigData = 
{
  .CanConfigSet =	&TestCanConfigSetData,
};

/*******************************************************************************
 * Code
 ******************************************************************************/


static void CAN0_transmitterInit(void)
{
    const gpio_pin_config_t can_transmitter_switch = {kGPIO_DigitalOutput, 0,};
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 0U, &can_transmitter_switch);   
    const gpio_pin_config_t can_transmitter_standby = {kGPIO_DigitalOutput, 0,};
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(PORTC, 1U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 1U, &can_transmitter_standby);   
}

static void CAN0_transmitterOn(void)
{
    GPIO_WritePinOutput(GPIOC, 0U, 1);
}

static void CAN0_outofStandby(void)
{    
    GPIO_WritePinOutput(GPIOC, 1U, 1);        
}

//VehicleInfo_t Vehicle;

/*!
 * @brief Application entry point.
 */
int main(void)
{
    BOARD_BootClockHSRUN();
    /* initialize uart1 for debug */
    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
    Uart_InitIO(0);
    DbgConsole_Init((uint32_t) UART0, 9600, DEBUG_CONSOLE_DEVICE_TYPE_UART, CLOCK_GetCoreSysClkFreq());     
    
    CAN0_transmitterInit();
    CAN0_transmitterOn();
    CAN0_outofStandby();
    
        /* led pin initialize */
    {
        gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 1,};
        CLOCK_EnableClock(kCLOCK_PortC);
        PORT_SetPinMux(PORTC, 10U, kPORT_MuxAsGpio);
        GPIO_PinInit(GPIOC, 10U, &gpio_config);
        gpio_config.outputLogic = 0;
        PORT_SetPinMux(PORTC, 11U, kPORT_MuxAsGpio);
        GPIO_PinInit(GPIOC, 11U, &gpio_config);
    }
    Can_Init(&CanConfigData );
    CanIf_Init(&CanIf_Config); 
    CanIf_SetControllerMode(CanIf_Config.ControllerConfig->CanIfControllerIdRef, CANIF_CS_STARTED);
//    CanIf_SetPduMode(0, CANIF_SET_ONLINE);
//    PduR_Init(&PduR_Config);
//    Com_Init(&ComConfiguration);
    timer0 = xTimerCreate( "time0",
                    pdMS_TO_TICKS(200),
                    pdTRUE,
                    led_toggle_timer_id,
                    LED_Toggle );
    xTimerStart(timer0,portMAX_DELAY);

    
    vTaskStartScheduler();
}

uint8_t data[256];
void LED_Toggle(TimerHandle_t xTimer)
{

    GPIO_TogglePinsOutput(GPIOC, 1 << 10U);

    GPIO_TogglePinsOutput(GPIOC, 1 << 11U);
    VHCL_GB_GetData(data);
//    Com_MainFunctionRx();
//    CanIf_Transmit();
}


