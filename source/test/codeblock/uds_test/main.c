#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main()
{
    uint8_t seed[4] = "";
    uint8_t key[4] = "";
    uint32_t iKey = 0;
    uint32_t iSeed = 0;
    scanf("%x",iSeed);
    iKey = ((((iSeed>>4) ^ iSeed) <<3) ^ iSeed);
    printf("%x\n", iKey);
    getchar();
    return 0;
}
