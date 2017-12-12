/* $Header:   Regulator.h $*/
/**********************************************************************
   Title                    : Regulator.h

   Module Description       : This file has all of the standard global defines
                              for module regulator

   Author                   : 

   Created                  : 

   Configuration ID         : 

 *********************************************************************/

#ifndef  REGULATOR_H
#define  REGULATOR_H 1

#include   "standard.h"
/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/

#define  REG_IIC_CHAN           0x00
// DSLR Chip Write Address and Register Address bytes (I2C)
#define DSLR_CHIP_WR_ADDR      0x10
#define DSLR_REG_ADDR        0x01 


#define SYNC_PWM_CHANNEL      1
#define SYNC_FREQ_DEFAULT     300000
#define SYNC_DUTY_CYCLE_ON    50   //50%
#define SYNC_DUTY_CYCLE_OFF   0   //0%

#define SYNC_PWM_3V8_CHANNEL      2
#define SYNC_FREQ_3V8_ON     416000 //2MHZ
#define SYNC_FREQ_3V8_OFF    0x0000  //2MHZ
/* this could be defined as callouts later */
#define reg_drv_set_sync_on(x)      Reg_TPS_Set_Sync_On(x)
//#define set_3v8_sync(x)               Reg_3v8_Set_Sync_On(x)

//#define RLp_DSLR_EN(x)          IO_EN_REG_PM(x)
//#define RLpi_DSLR_EN()           IO_EN_REG_Pi()
//#define RLpo_DSLR_EN(x)         IO_EN_REG_Po(x) 

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

typedef enum Reg_Tag
{
   NO_REG    = 0x0000,
   DSLR_HSD             = 0x1001,   /*DSLR HSD 12V CTRL*/
   DSLR_VLR              = 0x1002,   /*DSLR 8.5V CTRL*/
   DSLR_SW2EN        = 0x1004,   /*DSLR 3V3 CTRL*/
   
   REG1      = 0x2010,        /*VBUS 5V,DLSR_EN(TPS43331),IO*/
   REG2      = 0x2020,        /*3V3SW,IO*/
   REG3      = 0x2040,        /*1V8SW,IO*/
   REG4      = 0x2080,        /*1V5,IO*/
   REG5      = 0x2100,        /*1V42SW,IO*/
   REG_FULL = REG1|REG2|REG3|REG4|REG5|DSLR_HSD|DSLR_VLR|DSLR_SW2EN,//just for avoiding compile warning
} Reg_Type;

typedef enum Reg_Cmd_Tag
{
   REG_OFF = 0,
   REG_ON,
   NUM_REG_CMDS
} Reg_Command_Type;


/*********************************************************************/
/* Global Variable extern Declarations                               */
/*********************************************************************/



/*********************************************************************/
/* Function Prototypes                                               */
/*********************************************************************/

extern void Reg_Request(Reg_Type reg, Reg_Command_Type reg_cmd);
//extern void Reg_Set_Sync_On(bool state);
#endif

/**********************************************************************
 *                 
 * REVISION RECORDS
 *                 
 *********************************************************************/
/*
 *
 *********************************************************************/
