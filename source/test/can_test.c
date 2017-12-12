#include "can_ks22.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "portable.h"
#include "semphr.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
const char *to_send = "can send test!\r\n";

/*******************************************************************************
 * CAN data parse struct
 ******************************************************************************/
/* 0x1801FFF4 */
typedef struct
{
    float BatVoltage;
    float BatCurrent;
    union {
        uint8_t BattryStatus2;
        struct{
            uint8_t Reserved : 3;
            uint8_t BatOverTemperatureWarning : 1;
            uint8_t InsulationFault : 1;
            uint8_t DropoutVoltageWarning : 1;
            uint8_t PowerBatMatch : 1;
            uint8_t SOC_OverWarning : 1;
        }BattryStatus2_Struct;
    }BattryStatus2_union;
    union
    {
        uint8_t BattryStatus1;
        struct
        {
            uint8_t SOC_UnderWarning : 1;
            uint8_t SigleUndervoltageWarning : 1;
            uint8_t SigleOvervoltageWarning : 1;
            uint8_t SOC_LowWarning : 1;
            uint8_t TotalUndervoltageWarning : 1;
            uint8_t TotalOvervoltageWarning : 1;
            uint8_t TerminalPostOverTmpWarnig : 1;
            uint8_t DropoutTemperatureWarning : 1;
        }BattryStatus1_Struct;
    }BattryStatus1_union;
    float SOC;
}AY01_0x1801FFF4_Type;

/* BMS_MCU2 */
typedef struct
{
    float BatMaxVoltage;
    float BatMimVoltage;
    uint8_t BatMaxVoltageNo;
    uint8_t BatMaxVoltagePackNo;
    uint8_t BatMinVoltageNo;
    uint8_t BatMinVoltagePackNo;

}AY01_0x1802FFF4_Type;

/* BMS_MCU3 */
typedef struct
{

    uint8_t BatMaxTemperaturePackNo;
    uint8_t BatMinTemperature;
    uint8_t BatMaxTemperature ;
    uint8_t BatMaxTemperatureNo;
    union
    {
        uint8_t fault;
        struct 
        {
            uint8_t BatTemperatureHigh : 2;
            uint8_t BatTemperatureLow : 2;
            uint8_t BatCellVoltageLow : 2;
            uint8_t BatTemperatureUnBalance : 2;
        }fault_struct;
    }fault_union;
    uint8_t BMSLife : 8;    
    uint8_t BatMinTemperatureNo : 8;
    uint8_t BatMinTemperaturePackNo : 8;

}AY01_0x1803FFF4_Type;

/* BMS_MCU4*/
typedef struct
{
    float CathodeInsulationResistance;
    float AnodeInsulationResistance;
    uint16_t SOE;
    uint8_t SOH;
    union
    {
        uint8_t BatteryStatus3;
        struct
        {
            uint64_t chargRelaySta : 1;
            uint64_t reserve : 1;
            uint64_t headRelaySta : 1;
            uint64_t normolDriveMode : 1;
            uint64_t externalChargeMode : 1;
            uint64_t warning_1 : 1;
            uint64_t warning_2 : 1;
            uint64_t warning_3 : 1;
        }BatteryStatus3_struct;
    }BatteryStatus3_union;
}AY01_0x1804FFF4_Type;

/* BMS_MCU5 */
typedef struct 
{
    uint16_t BatSeriesCelsSum;
    uint16_t BatTemperatureSum;
    uint8_t BatPackSum;
}AY01_0x1805FFF4_Type;
#if 0
typedef enum
{
    lithium_iron_phosphate = 0x01,
    ternary_material = 0x02,
}BatteryType_t;

typedef enum
{
    tianneng = 0x01,
}BatteryManufactury_t;
#endif
/* BMS_MCU6 */
typedef struct
{
    uint8_t BatteryType;
    uint8_t BatNominalCapacity;
    uint8_t BatNominalVoltage;
    uint8_t BatteryManufactury;
    uint8_t SN;
    uint8_t Year;
    uint8_t Month;
    uint8_t Day;
}AY01_0x1806FFF4_Type;
typedef struct
{
    float BatCellVoltageNo1;
    float BatCellVoltageNo2;
    float BatCellVoltageNo3;
    float BatCellVoltageNo4;
}AY01_0x1801D2F4_Type;

typedef struct
{
    float BatCellVoltageNo5;
    float BatCellVoltageNo6;
    float BatCellVoltageNo7;
    float BatCellVoltageNo8;
}AY01_0x1802D2F4_Type;
typedef struct
{
    float BatCellVoltageNo9;
    float BatCellVoltageNo10;
    float BatCellVoltageNo11;
    float BatCellVoltageNo12;
}AY01_0x1803D2F4_Type;

typedef struct
{
    float BatCellVoltageNo13;
    float BatCellVoltageNo14;
    float BatCellVoltageNo15;
    float BatCellVoltageNo16;
}AY01_0x1804D2F4_Type;
typedef struct
{
    float BatCellVoltageNo17;
    float BatCellVoltageNo18;
    float BatCellVoltageNo19;
    float BatCellVoltageNo20;
}AY01_0x1805D2F4_Type;
typedef struct
{
    float BatCellVoltageNo21;
    float BatCellVoltageNo22;
    float BatCellVoltageNo23;
    float BatCellVoltageNo24;
}AY01_0x1806D2F4_Type;
typedef struct
{
    float BatCellVoltageNo25;
    float BatCellVoltageNo26;
    float BatCellVoltageNo27;
    float BatCellVoltageNo28;
}AY01_0x1807D2F4_Type;
typedef struct
{
    float BatCellVoltageNo29;
    float BatCellVoltageNo30;
    float BatCellVoltageNo31;
    float BatCellVoltageNo32;
}AY01_0x1808D2F4_Type;
typedef struct
{
    float BatCellVoltageNo33;
    float BatCellVoltageNo34;
    float BatCellVoltageNo35;
    float BatCellVoltageNo36;
}AY01_0x1809D2F4_Type;
typedef struct
{
    float BatCellVoltageNo37;
    float BatCellVoltageNo38;
    float BatCellVoltageNo39;
    float BatCellVoltageNo40;
}AY01_0x180AD2F4_Type;
typedef struct
{
    float BatCellVoltageNo41;
    float BatCellVoltageNo42;
    float BatCellVoltageNo43;
    float BatCellVoltageNo44;
}AY01_0x180BD2F4_Type;
typedef struct
{
    float BatCellVoltageNo45;
    float BatCellVoltageNo46;
    float BatCellVoltageNo47;
    float BatCellVoltageNo48;
}AY01_0x180CD2F4_Type;
typedef struct
{
    float BatCellVoltageNo49;
    float BatCellVoltageNo50;
    float BatCellVoltageNo51;
    float BatCellVoltageNo52;
}AY01_0x180DD2F4_Type;
typedef struct
{
    float BatCellVoltageNo53;
    float BatCellVoltageNo54;
    float BatCellVoltageNo55;
    float BatCellVoltageNo56;
}AY01_0x180ED2F4_Type;
typedef struct
{
    float BatCellVoltageNo57;
    float BatCellVoltageNo58;
    float BatCellVoltageNo59;
    float BatCellVoltageNo60;
}AY01_0x180FD2F4_Type;
typedef struct
{
    float BatCellVoltageNo61;
    float BatCellVoltageNo62;
    float BatCellVoltageNo63;
    float BatCellVoltageNo64;
}AY01_0x1810D2F4_Type;
typedef struct
{
    uint8_t BatCellTemperatureNo1;
    uint8_t BatCellTemperatureNo2;
    uint8_t BatCellTemperatureNo3;
    uint8_t BatCellTemperatureNo4;
    uint8_t BatCellTemperatureNo5;
    uint8_t BatCellTemperatureNo6;
    uint8_t BatCellTemperatureNo7;
    uint8_t BatCellTemperatureNo8;
}AY01_0x1850D2F4_Type;
typedef struct
{

    uint8_t BatCellTemperatureNo9;
    uint8_t BatCellTemperatureNo10;
    uint8_t BatCellTemperatureNo11;
    uint8_t BatCellTemperatureNo12;
    uint8_t BatCellTemperatureNo13;
    uint8_t BatCellTemperatureNo14;
    uint8_t BatCellTemperatureNo15;
    uint8_t BatCellTemperatureNo16;
}AY01_0x1851D2F4_Type;

typedef struct
{
    float VehicleSpeed;
    float Distance_Range;
    union
    {
        uint8_t Shift_State;
        struct
        {
            uint8_t Shift_State : 4;
            uint8_t Deboost_State : 1;
            uint8_t Dirve_State : 1;
            uint8_t Reserved : 2;
        }ShiftState_struct;
    }ShiftState_union;
    uint8_t ChargeState;
}AY01_0xc0401D0_Type;
typedef struct
{
    uint8_t RangeOfAcceleratePedal;
    uint8_t RangeOfBreakPedal;
    uint16_t MotorSpeed;
    float MotorVoltageInput;
    float MotorCurrentInput;
}AY01_0xc0501D0_Type;
typedef struct
{
    uint8_t MCU_Temperature;
    uint8_t Motor_Temperature;
    uint8_t Motor_Erro_Sum;
    uint8_t Motor_Erro_Code_List;
    uint8_t Other_Erro_Sum;
    uint8_t Other_Erro_Code_List;
}AY01_0xc0601D0_Type;

//MCU_ICU
typedef struct
{
    uint16_t MotorTorque;
    uint16_t MotorSpeed;
    uint16_t DC;
    union 
    {
        uint16_t MotorContorlerStatus;
        struct
        {
            uint16_t starting : 1;
            uint16_t idle : 1;
            uint16_t spin : 1;
            uint16_t drive : 1;
            uint16_t fault : 1;
            uint16_t Ready : 1;
            uint16_t mode : 1;
            uint16_t pwnEnable : 1;
            uint16_t preCharge : 1;
            uint16_t relayStatus : 1;
            uint16_t gearsStatus : 2;
            uint16_t canLife : 4;
        }MotorContorlerStatus_struct;
    }MotorContorlerStatus;
}AY01_0xc08A7F0_Type;
typedef struct
{
    uint16_t relayHeadVol;
    uint16_t relayTailVol;
    uint16_t ACeffectiveValue;
    uint8_t motorControllerTempratrue;
    uint8_t motorTemprature;
}AY01_0xc09A7F0_Type;
typedef struct
{
    union
    {
        uint16_t faultCode1;
        struct
        {
            uint16_t preCharge : 1;
            uint16_t mainRelay : 1;
            uint16_t IGBT : 1;
            uint16_t overcurrent : 1;
            uint16_t GeneralOvertemperature : 1;
            uint16_t SeriousOvertemperature : 1;
            uint16_t reserved1 : 1;
            uint16_t SeriousOverVoltage : 1;
            uint16_t reserved2 : 1;
            uint16_t SeriousUnderVoltage : 1;
            uint16_t GeneralStalled : 1;
            uint16_t SeriousStalled : 1;
            uint16_t Speeding : 1;
            uint16_t reserved3 : 1;
            uint16_t reserved4 : 1;
            uint16_t bus : 1;
        }faultCode1_struct;
    }faultCode1;
    union
    {
        uint16_t faultCode2;
        struct
        {
            uint16_t midpointVoltage : 1;
            uint16_t reserved1 : 7;
            uint16_t GeneralMotorOverTemprature : 1;
            uint16_t SeriousMOtorOverTemprature : 1;
            uint16_t reserved2 : 3;
            uint16_t DoubleThrottle : 1;
            uint16_t reserved3 : 2;
        }faultCode2_struct;
    }faultCode2;
    uint16_t MaximumDriveTorque;
    uint16_t MaximumDriveSpeed;
}AY01_0xc0AA7F0_Type;
typedef struct
{
    float mileage;
    uint8_t StallsStatus;
    uint8_t chargeInfo;
    uint8_t accelerationPedalDepth;
    uint8_t brakePedalDepth;
}AY01_0xc0BA7F0_Type;		

typedef struct
{
    uint32_t frameID;
    SemaphoreHandle_t mutex;
}CAN_FrameShareBuffer_t;
/* Task priorities. */
#define receive_send_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void send_task(void *pvParameters);
void receive_send_task(void *pvParameters);
void recieve_display_task(void *pvParameters);
static void can_praser(flexcan_frame_t *frame);
void LED_Toggle_Task(void *pvParameters);
void CAN0_transmitterInit(void);

void CAN0_transmitterOn(void);
void CAN0_outofStandby(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
AY01_0x1801FFF4_Type AY01_0x1801FFF4_Data;
AY01_0x1802FFF4_Type AY01_0x1802FFF4_Data;
AY01_0x1803FFF4_Type AY01_0x1803FFF4_Data;
AY01_0x1804FFF4_Type AY01_0x1804FFF4_Data;
AY01_0x1805FFF4_Type AY01_0x1805FFF4_Data;
AY01_0x1806FFF4_Type AY01_0x1806FFF4_Data;
AY01_0x1801D2F4_Type AY01_0x1801D2F4_Data;
AY01_0x1802D2F4_Type AY01_0x1802D2F4_Data;
AY01_0x1803D2F4_Type AY01_0x1803D2F4_Data;
AY01_0x1804D2F4_Type AY01_0x1804D2F4_Data;
AY01_0x1805D2F4_Type AY01_0x1805D2F4_Data;
AY01_0x1806D2F4_Type AY01_0x1806D2F4_Data;
AY01_0x1807D2F4_Type AY01_0x1807D2F4_Data;
AY01_0x1808D2F4_Type AY01_0x1808D2F4_Data;
AY01_0x1809D2F4_Type AY01_0x1809D2F4_Data;
AY01_0x180AD2F4_Type AY01_0x180AD2F4_Data;
AY01_0x180BD2F4_Type AY01_0x180BD2F4_Data;
AY01_0x180CD2F4_Type AY01_0x180CD2F4_Data;
AY01_0x180DD2F4_Type AY01_0x180DD2F4_Data;
AY01_0x180ED2F4_Type AY01_0x180ED2F4_Data;
AY01_0x180FD2F4_Type AY01_0x180FD2F4_Data;
AY01_0x1810D2F4_Type AY01_0x1810D2F4_Data;
AY01_0x1850D2F4_Type AY01_0x1850D2F4_Data;
AY01_0x1851D2F4_Type AY01_0x1851D2F4_Data;
AY01_0xc0401D0_Type AY01_0xc0401D0_Data;
AY01_0xc0501D0_Type AY01_0xc0501D0_Data;
AY01_0xc0601D0_Type AY01_0xc0601D0_Data;
AY01_0xc08A7F0_Type AY01_0xc08A7F0_Data;
AY01_0xc09A7F0_Type AY01_0xc09A7F0_Data;
AY01_0xc0AA7F0_Type AY01_0xc0AA7F0_Data;
AY01_0xc0BA7F0_Type AY01_0xc0BA7F0_Data;
CAN_FrameShareBuffer_t CAN0_FrameBuffer;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_BootClockHSRUN();
    CAN0_transmitterInit();
    CAN0_transmitterOn();
    CAN0_outofStandby();
    CAN0_FrameBuffer.mutex = xSemaphoreCreateMutex();
    CAN_PraserRegister(0, can_praser);
    CAN_RTOS_Init(0, 250000);

    xTaskCreate(LED_Toggle_Task, "LED_Toggle_Task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
void send_task(void *pvParameters)
{
    portTickType xDelay = pdMS_TO_TICKS(500);
    for(;;)
    {
        vTaskDelay(xDelay);
    }
}
static void can_praser(flexcan_frame_t *frame)
{
    if(frame == NULL)
    {
        return ;
    }
    xSemaphoreTake(CAN0_FrameBuffer.mutex, portMAX_DELAY);
    switch(frame->id)
    {

        case 0x1801FFF4:
            AY01_0x1801FFF4_Data.BatVoltage = (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.1;
            AY01_0x1801FFF4_Data.BatCurrent = (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.1 - 3200;
            AY01_0x1801FFF4_Data.BattryStatus1_union.BattryStatus1 = frame->dataByte6;
            AY01_0x1801FFF4_Data.BattryStatus2_union.BattryStatus2 = frame->dataByte7;
            AY01_0x1801FFF4_Data.SOC = (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.1;
            break;

        case 0x1802FFF4:
            AY01_0x1802FFF4_Data.BatMaxVoltage = (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1802FFF4_Data.BatMaxVoltageNo = frame->dataByte2;
            AY01_0x1802FFF4_Data.BatMaxVoltagePackNo = frame->dataByte3;
            AY01_0x1802FFF4_Data.BatMimVoltage = (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1802FFF4_Data.BatMinVoltageNo = frame->dataByte6;
            AY01_0x1802FFF4_Data.BatMinVoltagePackNo = frame->dataByte7;
            break;
        case 0x1803FFF4:   
            AY01_0x1803FFF4_Data.BatMaxTemperature = frame->dataByte0 - 40;
            AY01_0x1803FFF4_Data.BatMaxTemperatureNo = frame->dataByte1;
            AY01_0x1803FFF4_Data.BatMaxTemperaturePackNo = frame->dataByte2;
            AY01_0x1803FFF4_Data.BatMinTemperature = frame->dataByte3 - 40;
            AY01_0x1803FFF4_Data.BatMinTemperatureNo = frame->dataByte4;
            AY01_0x1803FFF4_Data.BatMinTemperaturePackNo = frame->dataByte5;
            AY01_0x1803FFF4_Data.fault_union.fault = frame->dataByte6;
            AY01_0x1803FFF4_Data.BMSLife = frame->dataByte7;
            break;
        case 0x1804FFF4:
            AY01_0x1804FFF4_Data.AnodeInsulationResistance = (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.1;
            AY01_0x1804FFF4_Data.CathodeInsulationResistance = (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.1;
            AY01_0x1804FFF4_Data.BatteryStatus3_union.BatteryStatus3 = frame->dataByte4;
            AY01_0x1804FFF4_Data.SOH = frame->dataByte5;
            AY01_0x1804FFF4_Data.SOE = frame->dataByte7 << 8 | frame->dataByte6;
            break;
        case 0x1805FFF4:
            AY01_0x1805FFF4_Data.BatSeriesCelsSum = frame->dataByte1 << 8 | frame->dataByte0;
            AY01_0x1805FFF4_Data.BatTemperatureSum = frame->dataByte3 << 8 | frame->dataByte2;
            AY01_0x1805FFF4_Data.BatPackSum = frame->dataByte4;
            break;
        case 0x1806FFF4:
            AY01_0x1806FFF4_Data.BatteryType = frame->dataByte0;
            AY01_0x1806FFF4_Data.BatNominalCapacity = frame->dataByte1;
            AY01_0x1806FFF4_Data.BatNominalVoltage = frame->dataByte2;
            AY01_0x1806FFF4_Data.BatteryManufactury = frame->dataByte3;
            AY01_0x1806FFF4_Data.SN = frame->dataByte4;
            AY01_0x1806FFF4_Data.Year = frame->dataByte5;
            AY01_0x1806FFF4_Data.Month = frame->dataByte6;
            AY01_0x1806FFF4_Data.Day = frame->dataByte7;
            break;
        case 0x1801D2F4:
            AY01_0x1801D2F4_Data.BatCellVoltageNo1 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1801D2F4_Data.BatCellVoltageNo2 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1801D2F4_Data.BatCellVoltageNo3 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1801D2F4_Data.BatCellVoltageNo4 = (frame->dataByte7 == 0xff && frame->dataByte5 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x1802D2F4:
            AY01_0x1802D2F4_Data.BatCellVoltageNo5 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1802D2F4_Data.BatCellVoltageNo6 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1802D2F4_Data.BatCellVoltageNo7 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1802D2F4_Data.BatCellVoltageNo8 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x1803D2F4:
            AY01_0x1803D2F4_Data.BatCellVoltageNo9 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1803D2F4_Data.BatCellVoltageNo10 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1803D2F4_Data.BatCellVoltageNo11 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1803D2F4_Data.BatCellVoltageNo12 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x1804D2F4:
            AY01_0x1804D2F4_Data.BatCellVoltageNo13 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1804D2F4_Data.BatCellVoltageNo14 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1804D2F4_Data.BatCellVoltageNo15 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1804D2F4_Data.BatCellVoltageNo16 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x1805D2F4:
            AY01_0x1805D2F4_Data.BatCellVoltageNo17 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1805D2F4_Data.BatCellVoltageNo18 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1805D2F4_Data.BatCellVoltageNo19 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1805D2F4_Data.BatCellVoltageNo20 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x1806D2F4:
            AY01_0x1806D2F4_Data.BatCellVoltageNo21 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1806D2F4_Data.BatCellVoltageNo22 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1806D2F4_Data.BatCellVoltageNo23 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1806D2F4_Data.BatCellVoltageNo24 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x1807D2F4:
            AY01_0x1807D2F4_Data.BatCellVoltageNo25 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1807D2F4_Data.BatCellVoltageNo26 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1807D2F4_Data.BatCellVoltageNo27 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1807D2F4_Data.BatCellVoltageNo28 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x1808D2F4:
            AY01_0x1808D2F4_Data.BatCellVoltageNo29 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1808D2F4_Data.BatCellVoltageNo30 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1808D2F4_Data.BatCellVoltageNo31 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1808D2F4_Data.BatCellVoltageNo32 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x1809D2F4:
            AY01_0x1809D2F4_Data.BatCellVoltageNo33 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x1809D2F4_Data.BatCellVoltageNo34 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x1809D2F4_Data.BatCellVoltageNo35 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x1809D2F4_Data.BatCellVoltageNo36 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case 0x180AD2F4:
            AY01_0x180AD2F4_Data.BatCellVoltageNo37 = (frame->dataByte1 == 0xff && frame->dataByte0 == 0xff) ? 0xFF : (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.001;
            AY01_0x180AD2F4_Data.BatCellVoltageNo38 = (frame->dataByte3 == 0xff && frame->dataByte2 == 0xff) ? 0xFF : (float)(frame->dataByte3 << 8 | frame->dataByte2) * 0.001;
            AY01_0x180AD2F4_Data.BatCellVoltageNo39 = (frame->dataByte5 == 0xff && frame->dataByte4 == 0xff) ? 0xFF : (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.001;
            AY01_0x180AD2F4_Data.BatCellVoltageNo40 = (frame->dataByte7 == 0xff && frame->dataByte6 == 0xff) ? 0xFF : (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.001;
            break;
        case  0x1850D2F4:
            AY01_0x1850D2F4_Data.BatCellTemperatureNo1 = frame->dataByte0 == 0xFF ? 0xFF : frame->dataByte0 - 40;
            AY01_0x1850D2F4_Data.BatCellTemperatureNo2 = frame->dataByte1 == 0xFF ? 0xFF : frame->dataByte1 - 40;
            AY01_0x1850D2F4_Data.BatCellTemperatureNo3 = frame->dataByte2 == 0xFF ? 0xFF : frame->dataByte2 - 40;
            AY01_0x1850D2F4_Data.BatCellTemperatureNo4 = frame->dataByte3 == 0xFF ? 0xFF : frame->dataByte3 - 40;
            AY01_0x1850D2F4_Data.BatCellTemperatureNo5 = frame->dataByte4 == 0xFF ? 0xFF : frame->dataByte4 - 40;
            AY01_0x1850D2F4_Data.BatCellTemperatureNo6 = frame->dataByte5 == 0xFF ? 0xFF : frame->dataByte5 - 40;
            AY01_0x1850D2F4_Data.BatCellTemperatureNo7 = frame->dataByte6 == 0xFF ? 0xFF : frame->dataByte6 - 40;
            AY01_0x1850D2F4_Data.BatCellTemperatureNo8 = frame->dataByte7 == 0xFF ? 0xFF : frame->dataByte7 - 40;
            break;
        case  0x1851D2F4:
            AY01_0x1851D2F4_Data.BatCellTemperatureNo9 = frame->dataByte0 == 0xFF ? 0xFF : frame->dataByte0 - 40;
            AY01_0x1851D2F4_Data.BatCellTemperatureNo10 = frame->dataByte1 == 0xFF ? 0xFF : frame->dataByte1 - 40;
            AY01_0x1851D2F4_Data.BatCellTemperatureNo11 = frame->dataByte2 == 0xFF ? 0xFF : frame->dataByte2 - 40;
            AY01_0x1851D2F4_Data.BatCellTemperatureNo12 = frame->dataByte3 == 0xFF ? 0xFF : frame->dataByte3 - 40;
            AY01_0x1851D2F4_Data.BatCellTemperatureNo13 = frame->dataByte4 == 0xFF ? 0xFF : frame->dataByte4 - 40;
            AY01_0x1851D2F4_Data.BatCellTemperatureNo14 = frame->dataByte5 == 0xFF ? 0xFF : frame->dataByte5 - 40;
            AY01_0x1851D2F4_Data.BatCellTemperatureNo15 = frame->dataByte6 == 0xFF ? 0xFF : frame->dataByte6 - 40;
            AY01_0x1851D2F4_Data.BatCellTemperatureNo16 = frame->dataByte7 == 0xFF ? 0xFF : frame->dataByte7 - 40;
            break;
        case 0xc0401D0:
      
            AY01_0xc0401D0_Data.VehicleSpeed = (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.1;
            AY01_0xc0401D0_Data.Distance_Range = (float)(frame->dataByte5 << 24 |\
                                                       frame->dataByte4 << 16 |\
                                                       frame->dataByte3 << 8 |\
                                                       frame->dataByte2) * 0.1;
            AY01_0xc0401D0_Data.ShiftState_union.Shift_State = frame->dataByte6;
            AY01_0xc0401D0_Data.ChargeState = frame->dataByte7;
            break;
        case 0xc0501D0:
            AY01_0xc0501D0_Data.RangeOfAcceleratePedal = frame->dataByte0;
            AY01_0xc0501D0_Data.RangeOfBreakPedal = frame->dataByte1;
            AY01_0xc0501D0_Data.MotorSpeed = frame->dataByte3 << 8 | frame->dataByte2;
            AY01_0xc0501D0_Data.MotorVoltageInput = (float)(frame->dataByte5 << 8 | frame->dataByte4) * 0.1;
            AY01_0xc0501D0_Data.MotorCurrentInput = (float)(frame->dataByte7 << 8 | frame->dataByte6) * 0.1;
            break;
        case 0xc0601D0:
            AY01_0xc0601D0_Data.MCU_Temperature = frame->dataByte0 - 40;
            AY01_0xc0601D0_Data.Motor_Temperature = frame->dataByte1 - 40;
            AY01_0xc0601D0_Data.Motor_Erro_Sum = frame->dataByte2;
            AY01_0xc0601D0_Data.Motor_Erro_Code_List = frame->dataByte3;
            AY01_0xc0601D0_Data.Other_Erro_Sum = frame->dataByte4;
            AY01_0xc0601D0_Data.Other_Erro_Code_List = frame->dataByte5;
            break;
        case 0xc08A7F0:
            AY01_0xc08A7F0_Data.MotorTorque = (frame->dataByte1 << 8 | frame->dataByte0) - 32000;
            AY01_0xc08A7F0_Data.MotorSpeed = (frame->dataByte3 << 8 | frame->dataByte2) - 32000;
            AY01_0xc08A7F0_Data.DC = (frame->dataByte5 << 8 | frame->dataByte4) - 32000;
            AY01_0xc08A7F0_Data.MotorContorlerStatus.MotorContorlerStatus = (frame->dataByte7 << 8 | frame->dataByte6);
            break;
        case  0xc09A7F0:
            AY01_0xc09A7F0_Data.relayHeadVol = (frame->dataByte1 << 8 | frame->dataByte0) - 32000;
            AY01_0xc09A7F0_Data.relayTailVol = (frame->dataByte3 << 8 | frame->dataByte2) - 32000;
            AY01_0xc09A7F0_Data.ACeffectiveValue = (frame->dataByte5 << 8 | frame->dataByte4) - 32000;
            AY01_0xc09A7F0_Data.motorControllerTempratrue = frame->dataByte6 - 40;
            AY01_0xc09A7F0_Data.motorTemprature = frame->dataByte7 - 40;
            break;
        case 0xc0AA7F0:
            AY01_0xc0AA7F0_Data.faultCode1.faultCode1 = frame->dataByte1 << 8 | frame->dataByte0;
            AY01_0xc0AA7F0_Data.faultCode2.faultCode2 = frame->dataByte3 << 8 | frame->dataByte2;
            AY01_0xc0AA7F0_Data.MaximumDriveTorque = (frame->dataByte5 << 8 | frame->dataByte4) - 32000;
            AY01_0xc0AA7F0_Data.MaximumDriveSpeed = (frame->dataByte7 << 8 | frame->dataByte6) - 32000;
            break;
        case 0xc0BA7F0:
            AY01_0xc0BA7F0_Data.mileage = (float)(frame->dataByte1 << 8 | frame->dataByte0) * 0.1;
            AY01_0xc0BA7F0_Data.StallsStatus = frame->dataByte2;
            AY01_0xc0BA7F0_Data.chargeInfo = frame->dataByte3;
            AY01_0xc0BA7F0_Data.accelerationPedalDepth = frame->dataByte4;
            AY01_0xc0BA7F0_Data.brakePedalDepth = frame->dataByte5;
            break;
        default:
            break;
    }
    xSemaphoreGive(CAN0_FrameBuffer.mutex);
}
/*******************************************************************************
*    Function: CAN_GetFrame
*
*  Parameters: channel, spcecify the can channel
*               id express the frame's id 
*     Returns: A pointer point to the frame
* Description: Get a frme by it's id
*******************************************************************************/
void* CAN_GetFrame(CAN_CHANNEL_T channel, uint32_t id)
{
    flexcan_frame_t *frame = NULL;
    switch(frame->id)
    {
        case 0x1801FFF4:
            frame = (void *)&AY01_0x1801FFF4_Data;
            break;
        case 0x1802FFF4:
            frame = (void *)&AY01_0x1802FFF4_Data;
            break;
        case 0x1803FFF4:   
            frame = (void *)&AY01_0x1803FFF4_Data;
            break;
        case 0x1804FFF4:
            frame = (void *)&AY01_0x1804FFF4_Data;
            break;
        case 0x1805FFF4:
            frame = (void *)&AY01_0x1805FFF4_Data;
            break;
        case 0x1806FFF4:
            frame = (void *)&AY01_0x1806FFF4_Data;
            break;
        case 0x1801D2F4:
            frame = (void *)&AY01_0x1801D2F4_Data;
            break;
        case 0x1802D2F4:
            frame = (void *)&AY01_0x1802D2F4_Data;
            break;
        case 0x1803D2F4:
            frame = (void *)&AY01_0x1803D2F4_Data;
            break;
        case 0x1804D2F4:
            frame = (void *)&AY01_0x1804D2F4_Data;
            break;
        case 0x1805D2F4:
            frame = (void *)&AY01_0x1805D2F4_Data;
            break;
        case 0x1806D2F4:
            frame = (void *)&AY01_0x1806D2F4_Data;
            break;
        case 0x1807D2F4:
            frame = (void *)&AY01_0x1807D2F4_Data;
            break;
        case 0x1808D2F4:
            frame = (void *)&AY01_0x1808D2F4_Data;
            break;
        case 0x1809D2F4:
            frame = (void *)&AY01_0x1809D2F4_Data;
            break;
        case 0x180AD2F4:
            frame = (void *)&AY01_0x180AD2F4_Data;
            break;
        case  0x1850D2F4:
            frame = (void *)&AY01_0x1850D2F4_Data;
            break;
        case  0x1851D2F4:
            frame = (void *)&AY01_0x1851D2F4_Data;
            break;
        case 0xc0401D0:     
            frame = (void *)&AY01_0xc0401D0_Data;
            break;
        case 0xc0501D0:
            frame = (void *)&AY01_0xc0501D0_Data;
            break;
        case 0xc0601D0:
            frame = (void *)&AY01_0xc0601D0_Data;
            break;
        case 0xc08A7F0:
            frame = (void *)&AY01_0xc08A7F0_Data;
            break;
        case  0xc09A7F0:
            frame = (void *)&AY01_0xc09A7F0_Data;
            break;
        case 0xc0AA7F0:
            frame = (void *)&AY01_0xc0AA7F0_Data;
            break;
        case 0xc0BA7F0:
            frame = (void *)&AY01_0xc0BA7F0_Data;
            break;
        default:
            break;
    }
    return NULL;
}


void LED_Toggle_Task(void *pvParameters)
{
    /* power control port */
    const gpio_pin_config_t LED_1_config = {kGPIO_DigitalOutput, 0,};
    const gpio_pin_config_t LED_2_config = {kGPIO_DigitalOutput, 1,};
    portTickType xDelay = pdMS_TO_TICKS(500);
    CLOCK_EnableClock(kCLOCK_PortC);

    PORT_SetPinMux(PORTC, 10U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 11U, kPORT_MuxAsGpio);

    GPIO_PinInit(GPIOC, 10U, &LED_1_config);
    GPIO_PinInit(GPIOC, 11U, &LED_2_config);
    for(;;)
    {
        vTaskDelay(xDelay);
        GPIO_WritePinOutput(GPIOC, 10U, 1);
        GPIO_WritePinOutput(GPIOC, 11U, 0);
        vTaskDelay(xDelay);
        GPIO_WritePinOutput(GPIOC, 10U, 0);
        GPIO_WritePinOutput(GPIOC, 11U, 1);
    }
}


void CAN0_transmitterInit(void)
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

void CAN0_transmitterOn(void)
{
    GPIO_WritePinOutput(GPIOC, 0U, 1);
}

void CAN0_outofStandby(void)
{    
    GPIO_WritePinOutput(GPIOC, 1U, 1);
        
}