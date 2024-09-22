#include "byte2string.h"


const uint8_t HexTable[] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};


//һ���ֽ�ת��Ϊ16�����ַ���ʽ���ַ���
void OneByteToStr(uint8_t byte, uint8_t *str)
{
    *str = HexTable[byte / 16];
    str++;
    *str = HexTable[byte % 16];
    str++;
    *str = 0;
}

//����ֽ�ת��Ϊ16�����ַ���ʽ���ַ������Կո����
void MultiByteToStr(uint8_t *byteBuf, uint16_t len, uint8_t *str)
{
    while (len--)
    {
        *str = HexTable[(*byteBuf) / 16];
        str++;
        *str = HexTable[(*byteBuf) % 16];
        str++;
        *str = ' ';
        str++;
        byteBuf++;
    }
    *str = 0;
}

