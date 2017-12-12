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
#include "debug.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void LED_Toggle(TimerHandle_t xTimer);
void ADC16_ReadTask(void *pvParameters);
/*******************************************************************************
 * Variables
 ******************************************************************************/
void * const led_toggle_timer_id;
TimerHandle_t  timer0;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Application entry point.
 */
int main(void)
{
    BOARD_BootClockHSRUN();
    BOADR_IO_Init();
    BOADR_Init();    
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
    timer0 = xTimerCreate( "time0",
                    pdMS_TO_TICKS(200),
                    pdTRUE,
                    led_toggle_timer_id,
                    LED_Toggle );
    xTaskCreate(ADC16_ReadTask, "ADC16 Read Task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);

//    xTimerStart(timer0,portMAX_DELAY);

    
    vTaskStartScheduler();
}

uint8_t data[256];
void LED_Toggle(TimerHandle_t xTimer)
{

    GPIO_TogglePinsOutput(GPIOC, 1 << 10U);

    GPIO_TogglePinsOutput(GPIOC, 1 << 11U);
//    uint32_t adc = ADC_Read(0);
//    DEBUG(DEBUG_HIGHT, "adc=%d\r\n",adc);
//    VHCL_GB_GetData(data);
//    Com_MainFunctionRx();
//    CanIf_Transmit();
}

void ADC16_ReadTask(void *pvParameters)
{
    for(;;)
    {
        uint32_t adc = ADC_Read(0);
        DEBUG(DEBUG_HIGHT, "adc=%d\r\n",adc);
        uint32_t adc2 = ADC_Read(0);
    }
}
