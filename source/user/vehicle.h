#ifndef __VEHICLE_STATUS_H__
#define __VEHICLE_STATUS_H__

#include "stdint.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_llwu.h"
#include "CAN_HW.h"

/*******************************************************************************
 * define
 ******************************************************************************/
#define ACC_PORT        PORTC
#define ACC_GPIO        GPIOC
#define ACC_PORT_CLK    kCLOCK_PortC
#define ACC_PORT_PIN    9U
#define ACC_AWAKE_IRQ   PORTC_IRQn

#define BWP_PORT        PORTC
#define BWP_GPIO        GPIOC
#define BWP_PORT_CLK    kCLOCK_PortC
#define BWP_PORT_PIN    1U
#define BWP_AWAKE_IRQ   PORTC_IRQn

#define BWP_LLWP_PIN    6U
#define BWP_LLWP_TYPE   kLLWU_ExternalPinRisingEdge /* Rising edge wakeup. */

/* Operation about the transceiver connected with mobileye */
#define ME_CAN_ON                       (uint8_t)1
#define ME_CAN_OFF                      (uint8_t)0
#define ME_CAN_TRANSCEIVER_ON(on_off)   GPIO_WritePinOutput(GPIOC, 0U, on_off)
#define ME_CAN_STB_ON                   (uint8_t)0
#define ME_CAN_STB_OFF                  (uint8_t)1
#define ME_CAN_TRANSCEIVER_STB(on_off)  GPIO_WritePinOutput(GPIOB, 17U, on_off)

#define DRIVER_MOTOR_CNT        1
#define BATTERY_ENERGY_CNT      1
#define BAT_ENG_TMP_PRB_CNT     37
#define BATTERY_CELL_CNT        37
#define BATTERY_CELL_TMEMP_CNT  16

#define ALARM_NO_ALARM            (uint8_t)
#define ALARM_LEVEL1              (uint8_t)1
#define ALARM_LEVEL2              (uint8_t)2
#define ALARM_LEVEL3              (uint8_t)4
/*******************************************************************************
 * typedef 
 ******************************************************************************/

typedef enum
{
    id_0x1801FFF4 = 0,
    id_0x1802FFF4,
    id_0x1803FFF4,
    id_0x1804FFF4,
    id_0x1805FFF4,
    id_0x1806FFF4,
    id_0x1801D2F4,
    id_0x1802D2F4,
    id_0x1803D2F4,
    id_0x1804D2F4,
    id_0x1805D2F4,
    id_0x1806D2F4,
    id_0x1807D2F4,
    id_0x1808D2F4,
    id_0x1809D2F4,
    id_0x180AD2F4,
    id_0x180BD2F4,
    id_0x180CD2F4,
    id_0x180DD2F4,
    id_0x180ED2F4,
    id_0x180FD2F4,
    id_0x1810D2F4,
    id_0x1850D2F4,
    id_0x1851D2F4,
    id_0x1806E5F4,
    id_0x18FF50E5,
    id_0xC08A7F0,
    id_0xC09A7F0,
    id_0xC0AA7F0,
    id_0xC0BA7F0,
    id_0xC0401D0,
    id_0xC0501D0,
    id_0xC0601D0,
    id_0x7E0,
    id_FrameCnt,
}CanFrameID_t;

/*******************************************************************************
 *  variate declaretion
 ******************************************************************************/

/*******************************************************************************
 *  function declaretion
 ******************************************************************************/
const flexcan_frame_t *VHCL_CAN_GetDataList(void);
void VHCL_CAN_GiveDataList(void);
bool VHCL_CAN_UDS_GetVIN(flexcan_frame_t *buff, uint8_t idx);
bool VHCL_CAN_GetFrameById(flexcan_frame_t *pData, CanFrameID_t id);
bool VHCL_CAN_Init(void);
void UDS_GetData(flexcan_frame_t *pData);
void UDS_SetData(flexcan_frame_t *pData);
uint8_t VHCL_getAlarmLevel(void);
#endif //__VEHICLE_STATUS_H__
