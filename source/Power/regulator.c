/* $Header:   regulator.c $*/
/**********************************************************************
   Title                    : regulator.c

   Module Description       : This is the standard code file for regulator.
                              Other modules should use the interface provided
                              by regulator to turn on/off regulators.

   Author                   : 

   Created                  : 

   Configuration ID         : 

 *********************************************************************/

/*********************************************************************/
/* Include header files                                              */
#include "standard.h"        /* include standard includes */
/*********************************************************************/
/* File level pragmas                                                */
/*********************************************************************/

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
#define REG_IIC_REQUEST     0x1000

#define REG_IIC_MASK        0x0007
#define REG_IIC_START_MASK  0x0001

#define DSLR_HSD_WAIT_OFF_MSEC 50
#define DSLR_NORMAL_WAIT_OFF_MSEC 25

#define DSLR_HSD_WAIT_ON_MSEC 20
#define DSLR_NORMAL_WAIT_ON_MSEC 20

//#define DSLR_IIC_REFRESH_TICKS       (MSec_To_Ticks(250))

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef union reg_status_tag
{
   uint16_t all;
   struct
   {
      uint8_t iic             : 8;      /*IIC controlled power supply*/
      uint8_t pin            : 2;      /*for IO controlled power supply,2 in vip*/
      uint8_t reserved1  : 6;   /*reserved for external io controlled power supply*/
    } group;
   struct
   {
      uint8_t dslr1_hsd      : 1;
      uint8_t dslr1_vlr        : 1;
      uint8_t dslr1_sw2en    : 1;
      uint8_t unused1  : 5;
      
      uint8_t reg1    : 1;
      uint8_t reg2    : 1;
      uint8_t reg3    : 1;
      uint8_t reg4    : 1;
      uint8_t reg5    : 1;
      uint8_t reg6    : 1;	  
      uint8_t unused2  : 2;
   } field; 
} reg_status_type;
/*********************************************************************/
/* Global and Const Variable Defining Definitions / Initializations  */
/*********************************************************************/

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/

static reg_status_type reg_status;
/*********************************************************************/
/* ROM Const Variables With File Level Scope                         */
/*********************************************************************/


/*********************************************************************/
/* Add User defined functions                                        */
/*********************************************************************/


/*********************************************************************/
/* ROM Const Variables With File Level Scope                         */
/*********************************************************************/

/*********************************************************************/
/* Function Prototypes for Private Functions with File Level Scope   */
/*********************************************************************/
static Status_Type reg_tx_iic(uint8_t address, uint8_t data);

/*********************************************************************/
/* Global Function Definitions                                       */
/*********************************************************************/
/**********************************************************************
 * Description: Formats the PRM data IIC buffer and sends it to IIC_Write
 *              for transmission.
 *  Parameters: address = DSLR chip address value (either 1st DSLR or 2nd DSLR)
 *              data = PRM data value
 *     Returns: Status of the IIC write
 *********************************************************************/
static Status_Type reg_tx_iic(uint8_t address, uint8_t data)
{
   Status_Type status;

   status=E_OK;
   return (status);
}

/**********************************************************************
 *    Function: Reg_Request
 *
 *  Parameters: reg - the bitwise or'd regulators involved in the request
 *              reg_cmd - REG_OFF or REG_ON, depending on the caller's needs
 *              reg_delay_format - one of REG_DELAY_IN_TICKS or NO_REG_DELAY 
 *              depending on the caller's needs
 *
 *     Returns: Amount of delay needed (if any) for the regulator request,
 *              in the format of the request (ticks).
 *
 * Description: This function is the interface to the ASPM regulators.
 *              It will turn on or off the appropriate regulators based on
 *              the caller's requests.  It will also optionally return a
 *              delay value to allow the regulator(s) involved to settle.
 *
 **********************************************************************/

void Reg_Request(Reg_Type reg, Reg_Command_Type reg_cmd)
{
   uint16_t    old_reg_status;

   if (reg & REG_IIC_REQUEST)//control 
   {
      old_reg_status = reg_status.all;
      if (REG_ON == reg_cmd)
      {
         reg_status.all |= reg;
      }
      else if (REG_OFF == reg_cmd)
      {
         reg_status.all &= ~reg;
      }

      if (old_reg_status != reg_status.all)
      {
         if (E_OK == reg_tx_iic(DSLR_CHIP_WR_ADDR,reg_status.group.iic))
         {
         }
         else
         {
            reg_status.all = old_reg_status;
         }
      }
   }
}


/*********************************************************************/
/* Local Function Definitions                                        */
/*********************************************************************/

/**********************************************************************
 *                  
 * REVISION RECORDS 
 *                  
 *********************************************************************/
/*===========================================================================*\
 *
 *********************************************************************/

