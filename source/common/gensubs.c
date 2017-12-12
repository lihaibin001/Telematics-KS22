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

/**********************************************************************
 * FileName       : GenSubs.c
 * Author         : 
 * Description    :
 * Version        : Initial of SimpleRTOS
 * Function List  :
 * Config ID      : 
**********************************************************************/
#ifndef GENSUBS_C
#define GENSUBS_C

/*********************************************************************/
/* Include files                                                     */
/*********************************************************************/
#include "standard.h"        /* include standard includes */
                                                                       
/*********************************************************************/
/* File level pragmas                                                */
/*********************************************************************/
                                                                       
/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
#define  CHKSUM_OFFSET     0x5555
#define  CHECKSUM_SIZE     2                             //define number of bytes in a checksum
#define  UPDATE_CHECKSUM   true                          // update (change) checksum
#define  VERIFY_CHECKSUM   false                         // do not modifiy checksum
#define  NEW_CHECKSUM      true                          // Start a new checksum (not partial sum)
#define  RESET_CHECKSUM    true                          // Start a new checksum (not partial sum)
#define  CONTINUE_CHECKSUM false                         // Add to previous sum  (partial sum)

/*********************************************************************/
/* Function Prototypes for Private Functions with File Level Scope   */
/*********************************************************************/
                                                                       
/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
                                                                       
/*********************************************************************/
/* Global and Const Variable Defining Definitions / Initializations  */
/*********************************************************************/
                                                                       
/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
static const uint8_t reverse_bits_TBL[16] =
{
   0x00,
   0x08,
   0x04,
   0x0C,
   0x02,
   0x0A,
   0x06,
   0x0E,
   0x01,
   0x09,
   0x05,
   0x0D,
   0x03,
   0x0B,
   0x07,
   0x0F,
};

/*********************************************************************/
/* ROM Const Variables With File Level Scope                         */
/*********************************************************************/                                                                                                                                              
const uint8_t bits[8]     = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
const uint8_t not_bits[8] = {0x7f, 0xbf, 0xdf, 0xef, 0xf7, 0xfb, 0xfd, 0xfe};                                                                       
                                                                       
/*********************************************************************/
/* Function Definitions                                              */
/*********************************************************************/
/**********************************************************************
 * Function      : Assign_Bit
 * Description   : to set or to clear a bit given an address and a bit offset - used to implement a packed array of booleans
 * Parameters    : addr: address of object in which to set the bit
                 : bitnum: the bit offset, counted from the lowest address, MSB
                 : num_bits: size of bit array
                 : Set_or_Clear: 0--clear; 1--set
                 : RangeCheck: 0--no range check ; 1-- range check
 * Returns       : 
 * Description   : 
 * Create&Verlog : Xinfen Zheng  Version: Initial of SimpleRTOS
 *********************************************************************/
extern void Assign_Bit( void *addr, unsigned int bitnum, unsigned int num_bits, bool Set_or_Clear, bool RangeCheck)
{
   uint8_t *bit_ptr = (uint8_t *)addr;

   if (!RangeCheck || (bitnum < (num_bits + 1) ))
   {
	   if (Set_or_Clear)
	   {
	      *(bit_ptr + (bitnum / (8*sizeof(uint8_t)))) |= bits[bitnum % (8*sizeof(uint8_t))];
	   }
	   else
	   {
	      *(bit_ptr + (bitnum / (8*sizeof(uint8_t)))) &= not_bits[bitnum % (8*sizeof(uint8_t))];
	   }
   }
}

/**********************************************************************
 * Purpose:
 *   to read a bit given an address and a bit offset - used to implement
 *   a packed array of booleans
 *
 * Parameters:
 *   addr: address of object from which to read the bit
 *   bit: the bit offset, counted from the lowest address, MSB
 *
 *   returns: 1 if bit set, 0 if bit clear
 **********************************************************************/
bool Read_Bit_Subr( void *addr, unsigned int bitnum )
{
   uint8_t *bit_ptr = (uint8_t *)addr;

  return (0 != (*(bit_ptr + (bitnum / (8*sizeof(uint8_t)))) & bits[bitnum % (8*sizeof(uint8_t))]));
}

/**********************************************************************
 * Purpose: To find and clear the first active bit from an array
 *
 * Parameters:
 * array - the address of the array
 * bit_size - the size of the array in bits (not bytes)
 * consume_bit - if true the bit is cleared
 *
 * return: -1 if no bit is set, otherwise the offset, in bits, from the MSB
 *         of the lowest addressed byte in the array
 *          CLEARS the BIT
 **********************************************************************/
extern int Find_First_Set_Bit(void *addr, size_t bit_size, bool consume_bit)
{
   uint8_t *bit_ptr = (uint8_t *)addr;
   int byte_index;
   uint_fast8_t bit_index;

   for (byte_index = 0; (byte_index * 8) < bit_size; byte_index++)
   {
      if (0 != *bit_ptr)                                       // is byte non-zero?
      {
         for   (bit_index = 0; bit_index < 8; bit_index++)     // search for bit within byte
         {
            if (0 != (bits[bit_index] & *bit_ptr))             // find it?
            {
               if (consume_bit)
               {
                  *bit_ptr &= not_bits[bit_index];             // clear bit
               }
               if ( ((byte_index * 8) + bit_index) < bit_size)
               {
                  return((byte_index * 8) + bit_index);
               }
               else
               {
                  return(-1);                               // bit was past last valid bit in bit_ptr
               }
            }
         }
      }
      bit_ptr++;
   }
   return(-1);                                              // no bits set
}

/**********************************************************************
 * Purpose: To find next set bit in direction specified
 *
 * Parameters:
 * addr - the address of the array
 * bit_size - the size of the array in bits (not bytes)
 * starting_bit - bit position to start from (does not look at starting position)
 * up - 1 = look in up direction
 *
 * return: -1 if no bit is set, otherwise the offset, in bits, from the MSB
 *         of the lowest addressed byte in the array
 *
 **********************************************************************/
extern int Find_Next_Bit_with_Wrap(void *addr, size_t bit_size, int current_bit, bool up)
{
   uint8_t *bit_ptr = (uint8_t *)addr;
   int    bits_tried = 0;
   int    increment = (up ? 1 : -1);

   do
   {
      current_bit += increment;

      if (current_bit < 0)
      {
         current_bit = bit_size - 1;
      }
      else if (current_bit >= bit_size)
      {
         current_bit = 0;
      }

      bits_tried++;

   } while ((0 == (*(bit_ptr + ((unsigned int)current_bit / (8*sizeof(uint8_t)))) &
                   bits[(unsigned int)current_bit % (8*sizeof(uint8_t))])) &&
            (bits_tried < bit_size));

   if (bits_tried >= bit_size)
   {
      return(-1);                                              // no bits set
   }
   else
   {
      return(current_bit);
   }

}

/**********************************************************************
 *    Function:  GETSUM
 *
 *    Parameters: *Strt_Addr - pointer of begin address of data to be checksumed
 *                bool update - true = update this checksum byte
 *                bool initialize_chksum - true = write the new checksum value
 *                *checksum - pointer to address of checksum
 *
 *     Returns:  Status_Type (E_OK or E_ERROR)
 *
 * Description:  This module will calculate a 16 bit checksum, and either update
 *               the variable or not.  Has capability to calculate a continuation
 *               of a previous call.  If 8 bit Checksum is only needed, then the
 *               MSB can simply be ignored.
 *
 *
 *********************************************************************/
Status_Type Getsum (const void *Strt_Addr, size_t size,
               bool update, bool initialize_chksum, uint16_t *checksum)
{
   FAR const uint8_t *ptr = (FAR uint8_t *) Strt_Addr;   // pointer to start addr.
   Status_Type cs_succ;
   uint_fast16_t  chs_sum;                               // working summation
   static uint16_t chs_build;                            // store checksum between calls

   chs_sum = (initialize_chksum) ? CHKSUM_OFFSET : chs_build;  // if not a continuation, then start with CHKSUM_OFFSET

   for (; (0 != size); size--)
   {
      chs_sum += *ptr++;                                 // add the bytes
   }

   chs_build = chs_sum;

   if ((chs_build != *checksum))
   {
      cs_succ = OS_E_ERROR;
   }
   else
   {
      cs_succ = OS_E_OK;
   }

   if (update)
   {
      *checksum = chs_build;                             // if update requested write in the calculated checksum
   }

   return(cs_succ);
}
/**********************************************************************
 * Function      : CS_Write
 * Description   : writes 1 byte to checksum variable
 * Parameters    : dest     adddress of old data
                 : source   address  in new data
                 : count    number of bytes changing
                 : checksum address of checksum 
 * Returns       : 
 * Description   : 
 * Create&Verlog : Xinfen Zheng  Version: Initial of SimpleRTOS
 *********************************************************************/
void *CS_Write(void *dest, const void *source, size_t count, uint16_t *checksum)
{
   uint8_t *dptr = (uint8_t *) dest;                  // pointer to dest addr
   const FAR uint8_t*sptr = (FAR uint8_t *) source;   // pointer to source addr

   for (; (0 != count); count--)
   {
      *checksum  -= *dptr;                      // add the delta to the checksum
      *checksum  += *sptr;
      *dptr++ = *sptr++;                        // copy byte
   }
   return(dest);
}


/***********************************************************************
 *    Function: limit_check
 *
 *  Parameters: value, min, and max
 *
 *     Returns: Limit checked value
 *
 * Description: Forces value to within min and max inclusive 
 *
 ***********************************************************************/
int Limit_Check (int value, int min, int max)
{
   if (value < min)
   {
      value = min;
   }
   else if (value > max)
   {
      value = max;
   }
   return(value);
}

#ifndef   ASSEMBLY_PROJECT_IS
/***********************************************************************
 *    Function: Wrap_Check
 *
 *  Parameters: value, min, and max
 *
 *     Returns: wrapped value
 *
 * Description: wraps a value if out of limits (i.e > max goes to min)
 *
 ***********************************************************************/
int Wrap_Check(int value, int min, int max)
{
   if (min > value)
   {
      value = max;
   }
   else if (max < value)
   {
      value = min;
   }
   return(value);
}

/**********************************************************************
 *    Function: BCD_To_Hex
 *
 *  Parameters: BCD_Value   BCD value to be convereed to hex
 *
 *     Returns: Equivalent value in hexidecimal format
 *
 * Description: This routine converts a received value in BCD and
 *              converts it to a Hexidecimal value.
 *              Hex_value = (BCD_value & F0h) * ?
 *
 **********************************************************************/

uint8_t BCD_To_Hex (uint8_t BCD_Value)
{
   return(((BCD_Value / 16) * 10) + (BCD_Value % 16));
}
#endif   //ASSEMBLY_PROJECT_IS

/**********************************************************************
 *    Function: Hex_To_BCD
 *
 *  Parameters: HexValue    Hexidecimal value to be converted to BCD
 *
 *     Returns: Equivalent value in BCD
 *
 * Description: This routine converts a received value in Hexidecimal
 *              and converts it to a BCD value.  If the value is too
 *              large a value of 99h is returned
 *
 *              BCD_Value = (HexValue / 10) * 10h  +  remainder
 *                          interger division    int modulo division
 **********************************************************************/

uint8_t Hex_To_BCD (uint8_t HexValue)
{
   if (99 < HexValue)     // overflow condition true if HexValue > 99
   {
       return(0x99);
   }
   else
   {
       return(((HexValue / 10) * 0x10) + (HexValue % 10));
   }
}

/**********************************************************************
 *    Function: Hex_To_WBCD
 *
 *  Parameters: HexValue    Hexidecimal value to be converted to BCD
 *
 *     Returns: Equivalent value in packed BCD (0x000 .. 0x0255)
 *
 * Description: This routine converts a received value in Hexidecimal
 *              and converts it to a word BCD value.
 **********************************************************************/
uint16_t Hex_To_WBCD (uint8_t HexValue)
{
   return(  ((HexValue / 100) * 0x100) +
           (((HexValue % 100) / 10) * 0x10) +
             (HexValue % 10));
}

/**********************************************************************
 *    Function: WHEX_To_WBCD
 *
 *  Parameters: 16 bit HexValue    Hexidecimal value to be converted to BCD
 *
 *     Returns: Equivalent value in packed BCD
 *
 * Description: This routine converts a 16 bit value in Hexidecimal
 *              to a 16 bit BCD value.
 **********************************************************************/
uint16_t WHEX_To_WBCD (uint16_t HexValue)
{
   if (9999 < HexValue)
   {
       return(0x9999);  // overflow condition true if HexValue > 9999
   }
   else
   {
       return( (((HexValue % 10000) / 1000) * 0x1000) + 
               (((HexValue % 1000) / 100) * 0x100) +
               (((HexValue % 100) / 10) * 0x10) +
                 (HexValue % 10));
   }
}

#ifndef   ASSEMBLY_PROJECT_IS
/**********************************************************************
 *    Function: WBCD_To_Hex
 *
 *  Parameters: Word BCD value (000 - 255)
 *
 *     Returns: Equivalent value in hexidecimal format (1 byte)
 *
 * Description: This routine converts a received word length BCD 
 *              value to a hex value (up to 255).
 **********************************************************************/
uint8_t WBCD_To_Hex (uint16_t WBCD_value)
{
   if (0x255 < WBCD_value)
   {
       return(0xFF); // overflow condition true if WBCD_value > 255
   }
   else
   {
       return((uint8_t) ((WBCD_value & 0x000F) + 
                        ((WBCD_value & 0x00F0) >> 4) * 10 +
                        ((WBCD_value & 0x0F00) >> 8) * 100) );
   }
}

/**********************************************************************
 *    Function: WBCD_To_WHEX
 *
 *  Parameters: Word BCD value (0000 - 9999)
 *
 *     Returns: Equivalent value in hexidecimal format (2 byte)
 *
 * Description: This routine converts a received word length BCD 
 *              value to a hex value (up to 9999).
 **********************************************************************/
uint16_t WBCD_To_WHEX (uint16_t WBCD_value)
{
   return(  (WBCD_value & 0x000F) 
         + ((WBCD_value & 0x00F0) >>  4) * 10
         + ((WBCD_value & 0x0F00) >>  8) * 100
         + ((WBCD_value & 0xF000) >> 12) * 1000);
}

/**********************************************************************
 *    Function: Is_BCD
 *
 *  Parameters: Potential_BCD_Value   BCD value to be convereed to hex
 *
 *     Returns: True if BCD, false if not BCD
 *
 * Description: This routine tests both nibbles to see if the value ia a
 *              valid BCD value.
 *
 **********************************************************************/

bool Is_BCD(uint8_t Potential_BCD_Value)
{
   return( (Potential_BCD_Value <= 0x99) &&           // valid range 0x00 -- 0x99
           ((Potential_BCD_Value & 0x0F) <= 0x09));   // check lower nibble <= 9
}

/**********************************************************************
 *    Function: BCD_increment
 *
 *  Parameters: BCD_value:   BCD value to be incremented
 *
 *     Returns: BCD value incremented by one if valid BCD received.
 *              0xFF if invalid BCD received.
 *
 * Description: This routine increments a received value in BCD and
 *              returns the incremented value if valid, otherwise 0xFF
 *
 **********************************************************************/

uint8_t BCD_increment(uint8_t BCD_value)
{
   if (Is_BCD(BCD_value))                    // verify valid BCD value
   {
      if (BCD_value == 0x99)                 // test for rollover case
      {
         BCD_value = 0x00;                   // correct for rollover case
      }
      else                                   // is not rollover case
      {
         if ((BCD_value & 0x0F) == 0x09)     // test for last digit = 9
         {
            BCD_value += 0x07;               // add 7 for next valid BCD
         }
         else                                // last digit != 9
         {
            BCD_value++;                     // regular increment
         }
      }
   }
   else                                      // invalid BCD
   {
      BCD_value = 0x99;                      // set value to max for return
   }
   return(BCD_value);                        // return inc'd BCD value
}

/**********************************************************************
 *    Function: BCD_decrement
 *
 *  Parameters: BCD_value:   BCD value to be decremented
 *
 *     Returns: BCD value decremented by one if valid BCD received.
 *              0xFF if invalid BCD received.
 *
 * Description: This routine decrements a received value in BCD and
 *              returns the decremented value if valid, otherwise 0xFF
 *
 **********************************************************************/

uint8_t BCD_decrement(uint8_t BCD_value)
{
   if ( Is_BCD(BCD_value) )                  // verify valid BCD value
   {
      if ( BCD_value == 0x00 )               // test for rollover case
      {
         BCD_value = 0x99;                   // correct for rollover case
      }
      else                                   // is not rollover case
      {
         if ( (BCD_value & 0x0F) == 0x00 )   // test for last digit = 0
         {
            BCD_value -= 0x07;               // sub. 7 for next valid BCD
         }
         else                                // last digit != 0
         {
            BCD_value--;                     // regular decrement
         }
      }
   }
   else                                      // invalid BCD
   {
      BCD_value = 0x00;                      // set value to min for return
   }
   return(BCD_value);                        // return dec'd BCD value
}

/**********************************************************************
 *    Function: memcmpch
 *
 *  Parameters: src - pointer to beginning of string to compare
 *              ch  - character to compare against
 *              n   - how many bytes to compare
 *
 *     Returns: int
 *
 * Description: Compares src against a single character.  Returns 0 if
 *              the src string matches n ch characters, non-0 otherwise.
 *
 **********************************************************************/

int memcmpch(void *src, char ch, size_t n)
{
    char *char_ptr = src;

    for (; (0 != n); n--)
    {
       if (*char_ptr++ != ch)
       {
         break;
       }
    }
    return(n);
}

/**********************************************************************
 *    Function: memexch
 *
 *  Parameters: src1 - pointer to beginning of first area to exchange
 *              src2 - pointer to beginning of second area to exchange
 *              n    - how many bytes to exchange
 *
 *     Returns: void
 *
 * Description: exchanges n bytes between an object indicated by src1
 *              and an object indicated by src2 provided they do not
 *              overlap.
 *
 **********************************************************************/

void memexch(void *src1, void *src2, size_t n)
{
   char *pch1;
   char *pch2;
   char temp;
   
   if (src1 < src2)           // set lower address to ch1 
   {
      pch1 = (char *) src1;
      pch2 = (char *) src2;
   }                                                      
   else
   {
      pch1 = (char *) src2;
      pch2 = (char *) src1;
   }

   if ((pch1 + n) <= pch2)    // check for address overlap
   {
      for (; (0 != n); n--)
      {
         temp = *pch1;
         *pch1++ = *pch2;
         *pch2++ = temp;
      }
   }
}

/**********************************************************************
 *    Function: Convert_To_Big_Endian
 *
 *  Parameters: *from - pointer to half_word or word to be converted
 *              *to - pointer to destination of the conversion
 *              num_bytes - # of bytes to convert
 *
 *     Returns: None
 *
 * Description: Converts a half_word or word from Little Endian to Big Endian
 *
 **********************************************************************/

void Convert_To_Big_Endian (void *to, const void *from, size_t num_bytes)
{
   #if LITTLE_ENDIAN                       // If true, do the conversion

   char *temp_to = (char *)to;
   const char *temp_from = ((const char *)from) + num_bytes;

   for ( ; 0 != num_bytes ; num_bytes--)
   {
      *(temp_to++) = *(--temp_from);
   }
   #else //LITTLE_ENDIAN                   // Else, temp_to = temp_from
   memcpy(to, from, num_bytes);
   #endif //LITTLE_ENDIAN
}

/**********************************************************************
 *    Function: Scale
 *
 *  Parameters: input   - value to be scaled
 *              old_min - minimum value of old range
 *              old_max - maximum value of old range
 *              new_min - minimum value of new scaled range
 *              new_max - maximum value of new scaled range
 *
 *     Returns: Scaled value
 *
 * Description: Scales a value from one range to another.
 *
 **********************************************************************/

int Scale (int input, int old_min, int old_max, int new_min, int new_max)
{
   int old_range = old_max - old_min;

   return(((((input - old_min) * (new_max - new_min)) + (old_range/2))
            / old_range) + new_min);
}
#endif   //ASSEMBLY_PROJECT_IS

/**********************************************************************
 *    Function: Reverse_Bits_In_Byte
 *
 *  Parameters: input - byte to be bit reversed
 *
 *     Returns: byte with reversed bits
 *
 * Description: reverses bits in a byte D7 D6 D5 D4 D3 D2 D1 D0
 *                                      D0 D1 D2 D3 D4 D5 D6 D7
 *
 **********************************************************************/
uint8_t Reverse_Bits_In_Byte (uint8_t byte)
{
   return( (reverse_bits_TBL[byte & 0x0F] << 4) | reverse_bits_TBL[byte >> 4] );
}

/**********************************************************************
 * REVISION RECORDS
 *********************************************************************/
/*********************************************************************/
/*
 *
 *********************************************************************/

#endif
