#ifndef _STR_LIB_H_
#define _STR_LIB_H_
/**********************************************************************
   Title                      : str_lib.h         
                                                                         
   Module Description         : string related operations.


   Author                     : 
   
 *********************************************************************/

// Not used
extern uint8_t str_float_split(uint8_t *data,uint8_t len,uint8_t *data_before_point,uint8_t *data_after_point);

extern uint16_t DECtoStr(uint8_t *dest,uint8_t *src,uint16_t len);
extern uint8_t StrtoHex(uint8_t *data);
extern uint32_t StrtoDec(uint8_t *data, uint8_t len);
extern uint8_t Get_Next_Data_Len_Before_Comma(uint8_t *rx_buf);
extern uint8_t* Get_Filed_Data_Pointer(uint8_t* rx_buf, uint8_t field);
extern uint8_t Is_Leap_Year(uint16_t year);
extern void HextoChar(uint8_t data, uint8_t *chr_out);
extern uint8_t HexToStr(uint8_t *pSrc, uint8_t *pDest, uint16_t len);
#endif
