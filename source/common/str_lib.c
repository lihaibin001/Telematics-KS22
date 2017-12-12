#include <stdlib.h>
#include "standard.h"
#include "str_lib.h"

#define COMMA_CH  0x2C //","


uint8_t str_float_split(uint8_t *data,uint8_t len,uint8_t *data_before_point,uint8_t *data_after_point)
{
	uint8_t i=0;
	uint32_t int_before_point=0;
	uint32_t int_after_point=0;
	uint8_t *p=data;
	uint8_t point_pos=0;
	if (len==0)
		return 0;
	for(i=0;i<len;i++)
	{
        if(*p=='.')
		{
			point_pos=i;
			break;
		}
		p++;
		int_before_point=int_before_point*10+*p-'0';
	}
	p++;
	for(i=point_pos+1;i<len;i++)
	{
		int_after_point=int_after_point*10+*p-'0';
	}
	return 1;
}

uint16_t DECtoStr(uint8_t *dest,uint8_t *src,uint16_t len)
{
    uint16_t cnt = 0;
    uint32_t tmp_total = 0;
    uint8_t i;
    uint8_t tmp_data[10];
    uint8_t dec_len = 0;
/*    if (len > (TCOM_TX_BUF_SIZE - 10)/2)
    {
        len = (TCOM_TX_BUF_SIZE - 10)/2;
    }*/
    while(cnt < len)
    {
        tmp_total += (*src << (cnt*8));
        src++;
        cnt++;
    }
    for (i=0; tmp_total>0; i++)
    {
        tmp_data[i] = tmp_total%10;
        tmp_total = tmp_total/10;
        dec_len++;
    }
    cnt = dec_len;
    for (i=0; dec_len>0; i++)
    {
        *(dest+i) = tmp_data[dec_len-1]+'0';
        dec_len--;
    }
    return cnt;
}

// input hex data, output 2 bytes data
void HextoChar(uint8_t data, uint8_t *chr_out)
{
    // convert hex to two byte char
//    uint8_t ret=0;
    if (data > 0x9f)
    {
        *chr_out=((data & 0xf0)>>4) - 0x0a + 'a';
    }
    else
    {
        *chr_out=((data & 0xf0)>>4) + '0';
    }
    if ((data & 0x0f) > 9)
    {
        *(chr_out+1)=(data & 0x0f) - 0x0a + 'a';
    }
    else
    {
        *(chr_out+1)=(data & 0x0f) + '0';
    }
}

// Convert two byte string to hex
uint8_t StrtoHex(uint8_t *data)
{
   uint8_t ret = 0;

   if ((*(data) >= 'a') && (*(data) <= 'f'))
   {
       ret = ((*data - 'a'+10)*16);
   }
   else
   {
       ret = (*data > '9') ? ((*data - 'A'+10)*16): ((*data - '0')*16); //high
   }
   if ((*(data+1) >= 'a') && (*(data+1) <= 'f'))
   {
       ret += ((*(data+1) - 'a'+10));
   }
   else
   {
       ret +=  (*(data+1) > '9') ? (*(data+1) - 'A'+10): (*(data+1) - '0'); //low
   }

   return ret;
}

// Convert string to decimal, not greater than uint32
uint32_t StrtoDec(uint8_t *data, uint8_t len)
{
   uint32_t ret = 0;
   uint8_t i = 0;

   for(i=0;i<len;i++)
   {
        ret=ret*10 + *(data+i)-'0';
   }
   return ret;
}

uint8_t ChartoInit(uint8_t byte)
{
    return byte - '0';
}

/*******************************************************************************
*    Function:  Get_Next_Data_Before_Comma
*
*  Parameters:  Tx Message
*     Returns:  Status
* Description:  Get_Next_Data_Before_Comma
*******************************************************************************/
uint8_t Get_Next_Data_Len_Before_Comma(uint8_t * rx_buf)
{
    uint8_t len = 0;
        
    while((*rx_buf != COMMA_CH) &&(*rx_buf != 0))
    {
        rx_buf++;

        if(len == 0xFF)
	   break;//break out 

        len++;
    }
    return len;
}

/*******************************************************************************
*    Function:  Get_Filed_Data_Pointer
*
*  Parameters:  Tx Message
*     Returns:  Status
* Description:  Get_Filed_Data_Pointer
*******************************************************************************/
uint8_t* Get_Filed_Data_Pointer(uint8_t* rx_buf, uint8_t field)
{
    uint8_t field_no= 0;
    uint8_t i = 0;
    while(*rx_buf)
    {
        if(field_no == field)
        {
            break;
        }
        
        if(COMMA_CH== *rx_buf)
        {
            field_no++;
        }
        rx_buf++;
        if(i == 0xFF)
	   break;//break out

        i++;
    }

    return rx_buf;
}

uint8_t Is_Leap_Year(uint16_t year)
{
    if(year%4==0) //必须能被4整除
    {
        if(year%100==0)
        {
            if(year%400==0)return 1;//如果以00结尾,还要能被400整除         
            else return 0;  
        } else {
            return 1;
        }
    } else {
        return 0;
    }
}

uint8_t HexToStr(uint8_t *pSrc, uint8_t *pDest, uint16_t len)
{
    uint16_t idx;
    for(idx=0; idx<len; idx++)
    {
        HextoChar(pDest[idx], &pSrc[idx*2]);
    }
    return 0;
}
