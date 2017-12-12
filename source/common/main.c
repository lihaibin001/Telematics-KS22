/*----------------------------------------------------------------------------/
 *  (C)Dedao, 2016
 *-----------------------------------------------------------------------------/
 *
 * Copyright (C) 2016, Dedao, all right reserved.
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this condition and the following disclaimer.
 *
 * This software is provided by the copyright holder and contributors "AS IS"
 * and any warranties related to this software are DISCLAIMED.
 * The copyright owner or contributors be NOT LIABLE for any damages caused
 * by use of this software.
 *----------------------------------------------------------------------------*/
/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
//#include "stm32f10x_it.h"

#include "standard.h"
#include "uart.h"

#define USE_DEBUG
#include "Debug.h"

/*===========================================================================*\
 * Constant and Macro Definitions using #define
\*===========================================================================*/

/*===========================================================================*\
 * Enumerations and Structures and Typedefs
\*===========================================================================*/

/*===========================================================================*\
 * Global and Const Variable Defining Definitions / Initializations
\*===========================================================================*/

/*===========================================================================*\
 * Static Variables and Const Variables With File Level Scope
\*===========================================================================*/

/*===========================================================================*\
 * Function Prototypes for Private Functions with File Level Scope
\*===========================================================================*/

static void mn_immediate_init(void);
static void mn_rcc_config(void);
static void mn_nvic_config(void);
static void mn_gpio_init(void);

/*
 * Retargets the C library printf function to the USART.
 */
int fputc( int ch, FILE *f);

/**********************************************************************
 *    Function: mn_immediate_init
 *  Parameters: void
 *     Returns: void
 * Description: Calls the initialize routines for various modules that
 *              must be done on BEFORE beginning the power up sequence.
 *
 **********************************************************************/

static void mn_immediate_init(void)
{
    /* System Clocks Configuration */
    mn_rcc_config();
    /* NVIC configuration */
    mn_nvic_config();
    /* GPIO configuration */
    mn_gpio_init();
    /*For debug, to be removed*/
    Uart_Initialize(UART_DEBUG_CHANNEL);
    if(!Sys_Get_Standby_Req_Flag())
    {
        /* Initialize watchdog */
        IWDG_Init();
    }
    else
    {
        DEBUG_PRINT0( DEBUG_HIGH, "[MAIN]:Deep Standby...\r\n");
    }

    if (Cold_Start()) 
    {
        Sys_Clear_Wakeup_Src_Flags();
        Sys_Clear_Standby_Req_Flag();

        rl_set_rtc_timeout(0);
        rl_set_can_interrupt_state(false);
        rl_set_low_batt_sleep(0);
    }
}

/**********************************************************************
 *    Function: mn_Initialize
 *
 *  Parameters: void
 *
 *     Returns: void
 *
 * Description: Calls the initialize routines for various modules.
 *
 *
 **********************************************************************/
void mn_Initialize(void)
{
#ifdef USE_DEBUG
    uint8_t i;
    int8_t * swdate = SY_Sw_Version();

    if(Cold_Start()){
        DEBUG_PRINT0( DEBUG_MEDIUM, "[MAIN]:Software Version:");
        for(i = 0;i < 16;i++)
        {
             DEBUG_PRINT1( DEBUG_MEDIUM, "%c", swdate[i]);
        }
        DEBUG_PRINT0( DEBUG_MEDIUM, "\r\n");

        swdate = SY_Sw_Date();
        DEBUG_PRINT0( DEBUG_HIGH, "[MAIN]:Software Date:");
        for(i = 0;i < 9;i++)
        {
            DEBUG_PRINT1( DEBUG_HIGH, "%c", swdate[i]);
        }
        DEBUG_PRINT0( DEBUG_HIGH, "\r\n");
        DEBUG_PRINT0( DEBUG_HIGH, "[MAIN]:Cold Start...\r\n");
    }
    else
    {
        DEBUG_PRINT0( DEBUG_HIGH, "[MAIN]:Warm Start...\r\n");
    }
#endif

    /* Initialize RTC */
    RTC_Config();
    /* Initialize PSYNC */
    PS_Initialize();
}

int main(void)
{
   /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
    Task_Type   task_id;
    //DBGMCU_Config(DBGMCU_SLEEP|DBGMCU_IWDG_STOP,ENABLE);
    //DBGMCU->CR = 0X00000307;

    Init_Cold_Start_Flag();
//    Sys_Clear_Standby_Req_Flag();
    mn_immediate_init();

    /* Creates all the tasks, then starts the scheduler. */
    OS_Init();                                   /* initialize operating system structures */

    for (task_id = (Task_Type)0; task_id < (OS_NUM_TASKS-2); task_id++)
    {
        OS_Activate_Task(task_id);                /* initialize all tasks */
    }
    OS_Start();                                  /* Start task switching. This function should not return */

    /* If this line is reached then vTaskStartScheduler() returned because there
    was insufficient heap memory remaining for the idle task to be created. */
    for( ;; );

    //return 0;
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
static void mn_rcc_config(void)
{   
    /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
                           | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE );

    /* SPI2 Periph clock enable */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );

    /* Enable USART1 clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    /* Enable USART2 clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* Enable USART3 clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    /* Enable UART4 clocks */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
//    /* Enable ADC1 clocks */
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

/**
  * @brief  Configures the NVIC for CAN.
  * @param  None
  * @retval None
  */
void mn_nvic_config(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Enable the USART3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Enable the UART4 Interrupt */
/*    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
*/
  /* Enable the SPI2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Set the Vector Table base address at 0x08002000 */
    NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 ); //add by scb
}

static void mn_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;  
    // JTAG-DP Disabled and SW-DP Enabled,so PA15 & PB4 can be used for I/O ports. 
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

/*    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);*/
    /* Init L line as idle state(high) */
//    IO_L_LINE_OUT(Bit_SET);
//    IO_GSM_4V_CTRL_OUT(Bit_SET);
    IO_GSM_4V_CTRL_OUT(Bit_RESET);
    IO_GSM_PWR_ON_OUT(Bit_RESET);
    IO_4V_CTRL_OUT(Bit_SET);
    IO_MCU_3V3_GPS_EN_OUT(Bit_SET);
    IO_MCU_LED1_CTL_OUT(Bit_RESET);
    IO_MCU_LED2_CTL_OUT(Bit_RESET);
    IO_FLASH_WP_OUT(Bit_SET);
    IO_FLASH_HOLD_OUT(Bit_SET);
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

int fputc( int ch, FILE *f )
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(USART1, (uint8_t) ch); 
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    return ch;
}
/*-----------------------------------------------------------*/

