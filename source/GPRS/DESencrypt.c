#include "standard.h"
#include "GPRS.h"
#ifdef TCOM_DATA_DES_ENCDEC
const uint8_t DATA_IP_Table[64] = 
{
	57,49,41,33,25,17,9,1,
	59,51,43,35,27,19,11,3,
	61,53,45,37,29,21,13,5,
	63,55,47,39,31,23,15,7,
	56,48,40,32,24,16,8,0,
	58,50,42,34,26,18,10,2,
	60,52,44,36,28,20,12,4,
	62,54,46,38,30,22,14,6
};

const uint8_t DATA_IP_1_Table[64] = 
{
	39,7,47,15,55,23,63,31,
	38,6,46,14,54,22,62,30,
	37,5,45,13,53,21,61,29,
	36,4,44,12,52,20,60,28,
	35,3,43,11,51,19,59,27,
	34,2,42,10,50,18,58,26,
	33,1,41,9,49,17,57,25,
	32,0,40,8,48,16,56,24
};

const uint8_t DATA_E_Table[48] = 
{
	31, 0, 1, 2, 3, 4,
	3, 4, 5, 6, 7, 8,
	7, 8,9,10,11,12,
	11,12,13,14,15,16,
	15,16,17,18,19,20,
	19,20,21,22,23,24,
	23,24,25,26,27,28,
	27,28,29,30,31, 0
};

const uint8_t DATA_P_Table[32] = 
{
	15,6,19,20,28,11,27,16,
	0,14,22,25,4,17,30,9,
	1,7,23,13,31,26,2,8,
	18,12,29,5,21,10,3,24
};

const uint8_t DATA_S_Table[8][4][16] =
{
	/* S1 */
	{
		{14,4,13,1,2,15,11,8,3,10,6,12,5,9,0,7},
		{0,15,7,4,14,2,13,1,10,6,12,11,9,5,3,8},
		{4,1,14,8,13,6,2,11,15,12,9,7,3,10,5,0},
		{15,12,8,2,4,9,1,7,5,11,3,14,10,0,6,13}
	},
	/* S2 */
	{
		{15,1,8,14,6,11,3,4,9,7,2,13,12,0,5,10},
		{3,13,4,7,15,2,8,14,12,0,1,10,6,9,11,5},
		{0,14,7,11,10,4,13,1,5,8,12,6,9,3,2,15},
		{13,8,10,1,3,15,4,2,11,6,7,12,0,5,14,9}
	},
	/* S3 */
	{
		{10,0,9,14,6,3,15,5,1,13,12,7,11,4,2,8},
		{13,7,0,9,3,4,6,10,2,8,5,14,12,11,15,1},
		{13,6,4,9,8,15,3,0,11,1,2,12,5,10,14,7},
		{1,10,13,0,6,9,8,7,4,15,14,3,11,5,2,12}
	},
	/* S4 */
	{
		{7,13,14,3,0,6,9,10,1,2,8,5,11,12,4,15},
		{13,8,11,5,6,15,0,3,4,7,2,12,1,10,14,9},
		{10,6,9,0,12,11,7,13,15,1,3,14,5,2,8,4},
		{3,15,0,6,10,1,13,8,9,4,5,11,12,7,2,14}
	},
	/* S5 */
	{
		{2,12,4,1,7,10,11,6,8,5,3,15,13,0,14,9},
		{14,11,2,12,4,7,13,1,5,0,15,10,3,9,8,6},
		{4,2,1,11,10,13,7,8,15,9,12,5,6,3,0,14},
		{11,8,12,7,1,14,2,13,6,15,0,9,10,4,5,3}
	},
	/* S6 */
	{
		{12,1,10,15,9,2,6,8,0,13,3,4,14,7,5,11},
		{10,15,4,2,7,12,9,5,6,1,13,14,0,11,3,8},
		{9,14,15,5,2,8,12,3,7,0,4,10,1,13,11,6},
		{4,3,2,12,9,5,15,10,11,14,1,7,6,0,8,13}
	},
	/* S7 */
	{
		{4,11,2,14,15,0,8,13,3,12,9,7,5,10,6,1},
		{13,0,11,7,4,9,1,10,14,3,5,12,2,15,8,6},
		{1,4,11,13,12,3,7,14,10,15,6,8,0,5,9,2},
		{6,11,13,8,1,4,10,7,9,5,0,15,14,2,3,12}
	},
	/* S8 */
	{
		{13,2,8,4,6,15,11,1,10,9,3,14,5,0,12,7},
		{1,15,13,8,10,3,7,4,12,5,6,11,0,14,9,2},
		{7,11,4,1,9,12,14,2,0,6,10,13,15,3,5,8},
		{2,1,14,7,4,10,8,13,15,12,9,0,3,5,6,11}
	}
};

const uint8_t KEY_PC_1[56] = 
{	
	56,48,40,32,24,16,8,
	0,57,49,41,33,25,17,
	9,1,58,50,42,34,26,
	18,10,2,59,51,43,35,
	62,54,46,38,30,22,14,
	6,61,53,45,37,29,21,
	13,5,60,52,44,36,28,
	20,12,4,27,19,11,3
};

const uint8_t KEY_PC_2[48] = 
{
	13,16,10,23,0,4,2,27,
	14,5,20,9,22,18,11,3,
	25,7,15,6,26,19,12,1,
	40,51,30,36,46,54,29,39,
	50,44,32,46,43,48,38,55,
	33,52,45,41,49,35,28,31
};

const uint8_t KEY_Shift[16] = {1,1,2,2,2,2,2,2,1,2,2,2,2,2,2,1};

void DES_Bytes8ToBits64(uint8_t res[8], uint8_t *dst);
/************************************************************
 * 8 bytes to 64 bits										*
 ***********************************************************/
void DES_Bytes8ToBits64(uint8_t res[8], uint8_t *dst)
{
	int8_t i, j;
	uint8_t temp;

	for (i = 0; i < 8; i++)
	{
		temp = res[i];
		for (j = 0; j < 8; j++)
		{
			*dst = temp & 0x01;
			temp >>= 1;
			dst++;
		}
	}
}

void DES_Bits64ToBytes8(uint8_t dst[8], uint8_t *src);
/************************************************************
 * 64 bits to 8 bytes										*
 ***********************************************************/
void DES_Bits64ToBytes8(uint8_t dst[8], uint8_t *src)
{
	int32_t i, j;
	uint8_t temp;

	for (i = 0; i < 8; i++)
	{
		temp = 0;
		for (j = 0; j < 8; j++)
		{
			temp >>= 1;
			if ((*src) & 0x01)
			{
				temp |= 0x80;
			}

			src++;
		}

		dst[i] = temp;
	}
}

void DES_IP_Transfer(uint8_t data[64]);
/************************************************************
 * IP transfer												*
 ***********************************************************/
void DES_IP_Transfer(uint8_t data[64])
{
	uint8_t temp[64];
	int32_t i;

	for (i = 0; i < 64; i++)
	{
		temp[i] = data[DATA_IP_Table[i]];
	}

	memcpy(data, temp, 64);
}

void DES_IP_1_Transfer(uint8_t data[64]);
/************************************************************
 * IP-1 transfer											*
 ***********************************************************/
void DES_IP_1_Transfer(uint8_t data[64])
{
	uint8_t temp[64];
	int32_t i;

	for (i = 0; i < 64; i++)
	{
		temp[i] = data[DATA_IP_1_Table[i]];
	}

	memcpy(data, temp, 64);
}

void DES_P_Transfer(uint8_t data[32]);
/************************************************************
 * P transfer												*
 ***********************************************************/
void DES_P_Transfer(uint8_t data[32])
{
	int32_t i;
	uint8_t temp[32];

	for (i = 0; i < 32; i++)
	{
		temp[i] = data[DATA_P_Table[i]];
	}

	memcpy(data, temp, 32);
}

void DES_PC1_Transfer(uint8_t key[64], uint8_t pc1[56]);
/************************************************************
 * KEY PC-1 transfer										*
 ***********************************************************/
void DES_PC1_Transfer(uint8_t key[64], uint8_t pc1[56])
{
	int32_t i;

	for (i = 0; i < 56; i++)
	{
		*(pc1+i) = key[KEY_PC_1[i]];
	}
}

void DES_PC2_Transfer(uint8_t pc1[56], uint8_t pc2[48]);
/************************************************************
 * KEY PC-2 transfer										*
 ***********************************************************/
void DES_PC2_Transfer(uint8_t pc1[56], uint8_t pc2[48])
{
	int32_t i;

	for (i = 0; i < 48; i++)
	{
		*(pc2+i) = pc1[KEY_PC_2[i]];
	}
}

void DES_Shift(uint8_t subkey[56], uint8_t time);
/************************************************************
 * subkeys shift left										*
 ***********************************************************/
void DES_Shift(uint8_t subkey[56], uint8_t time)
{
	uint8_t temp[56];

	memcpy(temp, (subkey+time), 28-time);
	memcpy((temp+28-time), subkey, time);
	memcpy((temp+28), (subkey+time+28), 28-time);
	memcpy((temp+56-time), (subkey+28), time);

	memcpy(subkey, temp, 56);
}

void DES_MakeSubKeys(uint8_t key[64], uint8_t subkeys[16][48]);
/************************************************************
 * make sub keys											*
 ***********************************************************/
void DES_MakeSubKeys(uint8_t key[64], uint8_t subkeys[16][48])
{
	uint8_t temp[56];
	int32_t i;

	DES_PC1_Transfer(key, temp);
	for (i = 0; i < 16; i++)
	{
		DES_Shift(temp, KEY_Shift[i]);
		DES_PC2_Transfer(temp, subkeys[i]);
	}
}	
/************************************************************
 * make sub keys											*
 ***********************************************************/
void DES_MakeSubKeys_56bit(uint8_t temp[56], uint8_t subkeys[16][48])
{
	int32_t i;

	for (i = 0; i < 16; i++)
	{
		DES_Shift(temp, KEY_Shift[i]);
		DES_PC2_Transfer(temp, subkeys[i]);
	}
}	

void DES_DataExpand(uint8_t data[48]);
/************************************************************
 * data expand												*
 ***********************************************************/
void DES_DataExpand(uint8_t data[48])
{
	uint8_t temp[48];
	int32_t i;

	for (i = 0; i < 48; i++)
	{
		temp[i] = data[DATA_E_Table[i]];
	}

	memcpy(data, temp, 48);
}

void DES_SBox(uint8_t data[48]);
/************************************************************
 * S-Box	 												*
 ***********************************************************/
void DES_SBox(uint8_t data[48])
{
	int32_t i;
	uint8_t row, col;
	uint8_t in_p, out_p;
	uint8_t out;

	in_p = 0;
	out_p = 0;
	for (i = 0; i < 8; i++)
	{
		row = (data[in_p] << 1) + data[in_p+5];
		col = (data[in_p+1] << 3) | (data[in_p+2] << 2) | (data[in_p+3] << 1) + data[in_p+4];

		out = DATA_S_Table[i][row][col];
		in_p += 6;

		data[out_p++] = (out >> 3) & 0x01;
		data[out_p++] = (out >> 2) & 0x01;
		data[out_p++] = (out >> 1) & 0x01;
		data[out_p++] = out & 0x01;
	}
}	

void DES_XOR(uint8_t *dst, uint8_t *src, int count);
/************************************************************
 * DES XOR			 										*
 ***********************************************************/
void DES_XOR(uint8_t *dst, uint8_t *src, int count)
{
	while (count--)
	{
		*dst ^= *src;
		dst++;
		src++;
	}
}

void DES_Swap(uint8_t *left, uint8_t *right);
/************************************************************
 * Swap L&R			 										*
 ***********************************************************/
void DES_Swap(uint8_t *left, uint8_t *right)
{
	uint8_t temp[32];

	memcpy(temp, left, 32);
	memcpy(left, right, 32);
	memcpy(right, temp, 32);
}

/************************************************************
 * Encrypt Block	 										*
 ***********************************************************/
uint32_t encrypt_start;
uint32_t encrypt_stop;
void DES_EncryptBlock(uint8_t plain_data[8], uint8_t sub_keys[16][48], uint8_t cipher_data[8])
{
	uint8_t plain_bits[64];
	uint8_t temp[48];
	uint8_t i; 

	encrypt_start = OS_Time();

	DES_Bytes8ToBits64(plain_data, plain_bits);

	DES_IP_Transfer(plain_bits);
	for (i = 0; i < 16; i++)
	{
		memcpy(temp, (plain_bits+32), 32);
		DES_DataExpand(temp);
		DES_XOR(temp, sub_keys[i], 48);
		DES_SBox(temp);
		DES_P_Transfer(temp);
		DES_XOR(plain_bits, temp, 32);
		if (i != 15)
		{
			DES_Swap(plain_bits, plain_bits+32);
		}
	}

	DES_IP_1_Transfer(plain_bits);
	DES_Bits64ToBytes8(cipher_data, plain_bits);
	encrypt_stop = OS_Time();
}

/************************************************************
 * Dencrypt Block	 										*
 ***********************************************************/
uint32_t decrypt_start;
uint32_t decrypt_stop;
void DES_DecryptBlock(uint8_t cipher_data[8], uint8_t sub_keys[16][48], uint8_t plain_data[8])
{
	uint8_t cipher_bits[64];
	uint8_t temp[48];
	int8_t i; 

	decrypt_start = OS_Time();
	DES_Bytes8ToBits64(cipher_data, cipher_bits);

	DES_IP_Transfer(cipher_bits);
	for (i = 15; i >= 0; i--)
	{
		memcpy(temp, (cipher_bits+32), 32);
		DES_DataExpand(temp);
		DES_XOR(temp, sub_keys[i], 48);
		DES_SBox(temp);
		DES_P_Transfer(temp);
		DES_XOR(cipher_bits, temp, 32);
		if (i != 0)
		{
			DES_Swap(cipher_bits, cipher_bits+32);
		}
	}

	DES_IP_1_Transfer(cipher_bits);
	DES_Bits64ToBytes8(plain_data, cipher_bits);
	decrypt_stop = OS_Time();
}

#endif

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/