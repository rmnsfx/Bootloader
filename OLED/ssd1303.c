
#include "ssd1303.h"



static const unsigned char font6_7[] = {
        0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x4F,0x00,0x00,     // !
        0x00,0x00,0x07,0x00,0x07,0x00,     // "
        0x00,0x14,0x7F,0x14,0x7F,0x14,     // #
        0x00,0x24,0x2A,0x7F,0x2A,0x12,     // $
        0x00,0x23,0x13,0x08,0x64,0x62,     // %
        0x00,0x36,0x49,0x55,0x22,0x50,     // &
        0x00,0x00,0x05,0x03,0x00,0x00,     // '
        0x00,0x00,0x1C,0x22,0x41,0x00,     // (
        0x00,0x00,0x41,0x22,0x1C,0x00,     // )
        0x00,0x14,0x08,0x3E,0x08,0x14,     // *
        0x00,0x08,0x08,0x3E,0x08,0x08,     // +
        0x00,0x00,0x50,0x30,0x00,0x00,     // ,
        0x00,0x08,0x08,0x08,0x08,0x08,     // -
        0x00,0x00,0x60,0x60,0x00,0x00,     // .
        0x00,0x20,0x10,0x08,0x04,0x02,     // /
        0x00,0x3E,0x51,0x49,0x45,0x3E,     // 0
        0x00,0x00,0x42,0x7F,0x40,0x00,     // 1
        0x00,0x42,0x61,0x51,0x49,0x46,     // 2
        0x00,0x21,0x41,0x45,0x4B,0x31,     // 3
        0x00,0x18,0x14,0x12,0x7F,0x10,     // 4
        0x00,0x27,0x45,0x45,0x45,0x39,     // 5
        0x00,0x3C,0x4A,0x49,0x49,0x30,     // 6
        0x00,0x01,0x71,0x09,0x05,0x03,     // 7
        0x00,0x36,0x49,0x49,0x49,0x36,     // 8
        0x00,0x06,0x49,0x49,0x29,0x1E,     // 9
        0x00,0x00,0x36,0x36,0x00,0x00,     // :
        0x00,0x00,0x56,0x36,0x00,0x00,     // ;
        0x00,0x08,0x14,0x22,0x41,0x00,     // <
        0x00,0x14,0x14,0x14,0x14,0x14,     // =
        0x00,0x00,0x41,0x22,0x14,0x08,     // >
        0x00,0x02,0x01,0x51,0x09,0x06,     // ?
        0x00,0x32,0x49,0x79,0x41,0x3E,     // @
        0x00,0x7E,0x11,0x11,0x11,0x7E,     // A
        0x00,0x7F,0x49,0x49,0x49,0x36,     // B
        0x00,0x3E,0x41,0x41,0x41,0x22,     // C
        0x00,0x7F,0x41,0x41,0x22,0x1C,     // D
        0x00,0x7F,0x49,0x49,0x49,0x41,     // E
        0x00,0x7F,0x09,0x09,0x09,0x01,     // F
        0x00,0x3E,0x41,0x49,0x49,0x7A,     // G
        0x00,0x7F,0x08,0x08,0x08,0x7F,     // H
        0x00,0x00,0x41,0x7F,0x41,0x00,     // I
        0x00,0x20,0x40,0x41,0x3F,0x01,     // J
        0x00,0x7F,0x08,0x14,0x22,0x41,     // K
        0x00,0x7F,0x40,0x40,0x40,0x40,     // L
        0x00,0x7F,0x02,0x0C,0x02,0x7F,     // M
        0x00,0x7F,0x04,0x08,0x10,0x7F,     // N
        0x00,0x3E,0x41,0x41,0x41,0x3E,     // O
        0x00,0x7F,0x09,0x09,0x09,0x06,     // P
        0x00,0x3E,0x41,0x51,0x21,0x5E,     // Q
        0x00,0x7F,0x09,0x19,0x29,0x46,     // R
        0x00,0x46,0x49,0x49,0x49,0x31,     // S
        0x00,0x01,0x01,0x7F,0x01,0x01,     // T
        0x00,0x3F,0x40,0x40,0x40,0x3F,     // U
        0x00,0x1F,0x20,0x40,0x20,0x1F,     // V
        0x00,0x3F,0x40,0x38,0x40,0x3F,     // W
        0x00,0x63,0x14,0x08,0x14,0x63,     // X
        0x00,0x07,0x08,0x70,0x08,0x07,     // Y
        0x00,0x61,0x51,0x49,0x45,0x43,     // Z
        0x00,0x00,0x7F,0x41,0x41,0x00,     // [
        0x00,0x15,0x16,0x7C,0x16,0x15,     // slash
        0x00,0x00,0x41,0x41,0x7F,0x00,     // ]
        0x00,0x04,0x02,0x01,0x02,0x04,     // ^
        0x00,0x40,0x40,0x40,0x40,0x40,     // _
        0x00,0x00,0x01,0x02,0x04,0x00,     // `
        0x00,0x20,0x54,0x54,0x54,0x78,     // a
        0x00,0x7F,0x48,0x44,0x44,0x38,     // b
        0x00,0x38,0x44,0x44,0x44,0x20,     // c
        0x00,0x38,0x44,0x44,0x48,0x7F,     // d
        0x00,0x38,0x54,0x54,0x54,0x18,     // e
        0x00,0x08,0x7E,0x09,0x01,0x02,     // f
        0x00,0x0C,0x52,0x52,0x52,0x3E,     // g
        0x00,0x7F,0x08,0x04,0x04,0x78,     // h
        0x00,0x00,0x44,0x7D,0x40,0x00,     // i
        0x00,0x20,0x40,0x44,0x3D,0x00,     // j
        0x00,0x7F,0x10,0x28,0x44,0x00,     // k
        0x00,0x00,0x41,0x7F,0x40,0x00,     // l
        0x00,0x7C,0x04,0x18,0x04,0x78,     // m
        0x00,0x7C,0x08,0x04,0x04,0x78,     // n
        0x00,0x38,0x44,0x44,0x44,0x38,     // o
        0x00,0x7C,0x14,0x14,0x14,0x08,     // p
        0x00,0x08,0x14,0x14,0x18,0x7C,     // q
        0x00,0x7C,0x08,0x04,0x04,0x08,     // r
        0x00,0x48,0x54,0x54,0x54,0x20,     // s
        0x00,0x04,0x3F,0x44,0x40,0x20,     // t
        0x00,0x3C,0x40,0x40,0x20,0x7C,     // u
        0x00,0x1C,0x20,0x40,0x20,0x1C,     // v
        0x00,0x3C,0x40,0x30,0x40,0x3C,     // w
        0x00,0x44,0x28,0x10,0x28,0x44,     // x
        0x00,0x0C,0x50,0x50,0x50,0x3C,     // y
        0x00,0x44,0x64,0x54,0x4C,0x44,     // z
        0x00,0x00,0x08,0x36,0x41,0x00,     // {
        0x00,0x00,0x00,0x7F,0x00,0x00,     // |
        0x00,0x00,0x41,0x36,0x08,0x00,     // }
        0x00,0x08,0x08,0x2A,0x1C,0x08,     // pfeil rechts
        0x00,0x08,0x1C,0x2A,0x08,0x08,     // pfeil links
};



void ssd_SEND(unsigned char cmd, unsigned char type)
{
        pin_SSD_DC(type);
        pin_SSD_CS(LOW);
        pin_SSS_DATA(cmd);
        pin_SSD_CS(HIGTH);
}


void ssd_RESET(void)
{
        pin_SSD_CS(HIGTH);
		pin_SSD_RD(HIGTH);
		pin_SSD_WR(LOW);
		pin_SSD_DC(HIGTH);
		
		pin_SSD_RES(LOW);
		//��� ������ ���� ��������
		pin_SSD_RES(HIGTH);
        
}

void ssd_SET_POS(unsigned int x, unsigned int y)
{
        //x += 2;
        ssd_SEND(SD_SET_LINE | (y & 0x0f), SSD_CMD);
        ssd_SEND(SD_SET_COL_LO | (x & 0x0f), SSD_CMD);
        ssd_SEND(SD_SET_COL_HI | ((x & 0xf0) >> 4), SSD_CMD);
}



void ssd_CLS(unsigned char fon)
{
 	  	int i, j;
        for(i = 0; i < 8; i++){
                ssd_SET_POS(0, i);
                for(j = 0; j < 128; j++){
                        ssd_SEND(fon, SSD_DATA);
                };
        }
        //sd1303_offset(0);

}

 

void ssd_INIT(void)
{
 		unsigned char i = 0;
 		unsigned char init_sequence[] = {
						0x00,
						0x10,
						0xAD,
						0x8E,
						0xA8,
						0x3F,
						0x40,
						0xA6,
						0xA0,
						0xC8,
						0xD3,
						0x00,
						0xD8,
						0x00,
						0xDA,
						0x12,
						0x81,
						0xFE,
						0x82,
						0xFE,
						0xD5,
						0x70,//0x70,
						0xD9,
						0x22,
						0xA4,
						0xAF,					
                        0xff};
        
        //genconfig_shadow |= IO_MASK(R_GEN_CONFIG,g8_15dir);     
        //*R_GEN_CONFIG = genconfig_shadow;                       
        
        while(init_sequence[i] != 0xff){
                ssd_SEND(init_sequence[i], SSD_CMD);
                i++;
        };
       // sd1303_cls();

}

void ssd_OLED_INIT(void)
{
 		ssd_RESET();
 		ssd_INIT();
 		ssd_CLEAR_OLED();
}

void ssd_TEST()
{
		static unsigned char s = 0x00;
 
 		if (s<0x80) s++;
 		else s = 0;

 		ssd_SEND(0x81,SSD_CMD);
 		ssd_SEND(s,SSD_CMD);
 
}

void ssd_PRINT(unsigned char *text, unsigned char x, unsigned char y)
{
        unsigned char i;
		unsigned char cols = 0;
		unsigned char s;
        ssd_SET_POS(0, y);
        
        for(i = 0; i < x; i++){
                ssd_SEND(0x00, SSD_DATA);
                cols++;
        }
        while(*text){
                s = *text-0x20;
                if(s > 127){
                        s = 0;
                };
                if(s == 0){
                        ssd_SEND(0x00, SSD_DATA);
                        ssd_SEND(0x00, SSD_DATA);     
                        cols += 2;
                } else {
                        for(i = 0; i < 6; i++){
                                if(font6_7[(s * 6) + i] != 0){
                                        if(cols < 128){
                                                ssd_SEND(font6_7[(s * 6) + i] , SSD_DATA);
                                                cols++;
                                        }
                                }
                        }
                }
                ssd_SEND(0x00, SSD_DATA);
                cols++;
                text++;
        }
        for(i = cols; i < 128; i++){
                ssd_SEND(0x00, SSD_DATA);
        }
        ssd_SEND(0xe3, SSD_DATA);
}













