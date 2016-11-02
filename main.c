/**
  ******************************************************************************
  * @file    Project/Template/main.c 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
//nclude <io.h>
#include "FILTER.h"
#include "File_Config.h"
#include <string.h>
#include <math.h>
#include "stm32f10x_exti.h"
#include "stm32f10x_iwdg.h"
#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "usbcore.h" 
#include "mscuser.h"
#include "memory.h"
#include "type.h"
#include "FileIO.h"
#include "wav.h"
#include "IIR.h"
#include "ADC.h"
#include "Pin.h"
#include "ssd1303.h"
#include "FM25V10.h"
#include "FLASH_AT45DB.h"
#include "EXT_ADC.h"
#include "vga_lib.h"
#include "Types.h"
#include "Keyboard.h"
#include "Menu.h"
#include "Reg.h"
#include "RealTime.h"
#include "Road.h"
#include "ff.h"
#include "diskio.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_crc.h"




  extern  FATFS   fls=0;            
  extern  FIL     FileTmp=0;          
	extern  FRESULT res_t=0; 
	extern 	FILINFO fno=0;

 #define TIME_POWER_OFF	(10*60*1000)


int GLOBAL_TIMER_1ms = 0;
int POWER_OFF = TIME_POWER_OFF;


 u32 GLOBAL_ERROR;
 extern int USB_CONNECT=0;
 extern u8	measure_stat=0;
 extern u8	usb_stat=0; 
 extern unsigned int	Num_of_Signals=0;
 u16 test_reg;
 u16 Sec_timer;
 u16 Moto_Sec;		//мотосекунды
 u8  Flag_1S;
 float		   k_reg;
 
unsigned int prgb_bat = 0;
bool usb_transit=0;


 u16 T;
 
 
int first_flag = 0; /// флаг первого включения
int message_status = 0;

int timer1=0;
int timer2=0;
int timer3=0;
int timer4=0;
int timer5=0;
 
int frzbat1=0;
int frzbat2=0; 

int usb_addr=0; 
int usb_count=0;

int id_akb = 0; //ид аккумма (1-старый, 0-новый)

unsigned int usb_charge_state = 1; //Управление подсчетом емкости (0 - выкл, 1- вкл)

float akbtemp = 0;
float akbtimer1 = 0;
float akbtimer2 = 0;
float akbemk = 0;
float akbemk_count = 0;
float akbemk_tek = 0;
float akbemk_percent = 0;
float akbemk_volt = 0;
int akbstatus = -1;
float akbemk_menu = 0;
FATINFO sdinfo;





USB_DEVICE_DESCRIPTOR USB_D;

 #define GET_SYS_TIMER() 	GLOBAL_TIMER_1ms;
 #define SET_SYS_TIMER(set)	GLOBAL_TIMER_1ms = set;

#define FULL_BATTERY_VOLT 4500
	

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 USART_InitTypeDef USART_InitStructure;
 GPIO_InitTypeDef  GPIO_InitStructure;
 DMA_InitTypeDef   DMA;
 ADC_InitTypeDef   ADC_InitStructure;

 

u8				buf[10];			
extern Tmen_status				men_STATUS;

extern unsigned int road_pos;
extern unsigned int road_cursor_pos;



//калбак функция

void Timer_1ms_CallBack(void)
{
	
			IWDG_ReloadCounter();

			GLOBAL_TIMER_1ms++;			 
			
			men_1ms();			
			key_1ms();

			if(timer4 == 5)
			{				
				timer4=0;	
			}
				
			POWER_OFF--;
				
			if (Sec_timer==0) 
			{
				Sec_timer = 1000; 
				Flag_1S   = 1;								 
			}
			else Sec_timer--;

			
			if (timer1 == 1000) 
			{				
				timer1 = 0;
				timer2++;
				timer3++;
			}
			else timer1++;
			
			if (timer2 == 30) timer2 = 0; ///30 секунд
			if (timer3 == 900000) timer3 = 0; ///15 мин.
			timer4++;
									
			if(timer5 == 10)
			{
				disk_timerproc();
				timer5=0;	
			}
			else timer5++;
			
}


/* Private functions ---------------------------------------------------------*/

void InitAll(void)
{
   RCC->APB2ENR	|= RCC_APB2ENR_IOPCEN;		// Подали такты на порт. Без этого работать не будет. 
						// Жоские вилы STM32 =))) А в симуляторе без этого работает на ура
   GPIOC->CRH   |= GPIO_CRH_MODE12_0;  		// Настроили порт на выход
   return;
}

void Delay(long i)
{
 while (i>0) i--;
}

void SET_CLOCK_SPEED(unsigned char CLOCK)
{
	if (CLOCK)
  if ((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_PLL)
  {
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	
  }
  else;
	else
  if ((RCC->CFGR & RCC_CFGR_SWS)!=RCC_CFGR_SWS_HSI)  
  {
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));

		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
  }
 
	if (CLOCK)	SysTick_Config(9000*8);	//настройка 1ms таймера  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
	else		SysTick_Config(8000); //настройка 1ms таймера  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */     
}


void GPIO_SETUP()
{
  GPIO_InitTypeDef  GPIO_InitStructure; 

   //включаем порт A
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
   //настраиваем выход P0
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   //GPIO_SetBits(GPIOA,GPIO_Pin_4);
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   //настраиваем вход аналоговый вход PFI
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   //настраиваем вход USB5V
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   //настраиваем вход DISCHARGE
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   //настраиваем выход SPI MCS
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOA,GPIO_Pin_4);
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   //настраиваем выход SPI MCLK,MISO,MOSI
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOA,GPIO_Pin_5);
   GPIO_SetBits(GPIOA,GPIO_Pin_7);
   GPIO_Init(GPIOA, &GPIO_InitStructure);	 
	 //Зарядка
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   //входы A8-A15 пропускаем

   //включаем порт B
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
   //настраиваем вход KEY1
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOB, &GPIO_InitStructure);   	 
	 //настраиваем вход KEY2
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOB, &GPIO_InitStructure);	  
	 
   //входы B2-B4 пропускаем
   //настраиваем выход CS
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;															  
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOB,GPIO_Pin_5);
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   //настраиваем выход RES
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   pin_SSD_RES(LOW);
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   //настраиваем выход D/C
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOB,GPIO_Pin_7);
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   //настраиваем выход WR
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOB,GPIO_Pin_8);
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   //настраиваем выход RD
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOB,GPIO_Pin_9);
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   //настраиваем вход KEY3
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   //настраиваем вход KEY4
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOB, &GPIO_InitStructure); 	 

	 
	 
   //внешний АЦП
   //настраиваем ADC_CS
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   //GPIO_SetBits(GPIOB,GPIO_Pin_12);
   pin_ADC_CS(LOW);
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   //настраиваем ADC_CLK
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   pin_ADC_CLK(LOW);
   //GPIO_SetBits(GPIOB,GPIO_Pin_12);
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   //настраиваем ADC_DATA
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_IPU; //наверно нужно настроить как плавающий вход
   GPIO_Init(GPIOB, &GPIO_InitStructure);
	 //Идентификатор АКБ (1-старый, 0-новый)
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   //включаем порт C
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
   //настраиваем выход D0-D7
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   //настраиваем вход KEY5
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   //настраиваем выход 13V_ON
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   pin_13V(LOW);
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   //настраиваем выход DISC
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOC,GPIO_Pin_11);
   GPIO_Init(GPIOC, &GPIO_InitStructure);
	 
    //настраиваем выход OFF
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_ResetBits(GPIOC,GPIO_Pin_12);
	 
	 GPIO_ResetBits(GPIOC,GPIO_Pin_13);
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   //включаем порт D
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
   //включаем порт 3.3VA_ON
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   //GPIO_SetBits(GPIOD,GPIO_Pin_2);
   pin_3VA(ON);
   GPIO_Init(GPIOD, &GPIO_InitStructure);


//**добавлено для кового варианта*******
//вкл. /выкл.
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOC,GPIO_Pin_9);
	 GPIO_Init(GPIOC, &GPIO_InitStructure);
	 
	 Delay(50000);
	 
	 // вкл/выкл подсветки
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_SetBits(GPIOC,GPIO_Pin_10);	 
   GPIO_Init(GPIOC, &GPIO_InitStructure);
	 
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	 
	 AFIO->EXTICR[0] &= ~AFIO_EXTICR3_EXTI9_PA;
	 AFIO->EXTICR[0] |= AFIO_EXTICR3_EXTI9_PA;
	 EXTI->FTSR |= EXTI_FTSR_TR9;
	 EXTI->IMR |= EXTI_IMR_MR9;
	 
	 NVIC_SetPriority(EXTI9_5_IRQn, 15);
	 NVIC_EnableIRQ(EXTI9_5_IRQn);
	 
	 //************
	 
	// RCC_APB1PeriphResetCmd(RCC_APB1Periph_BKP,ENABLE);
	
//	//Если старый аккум., настраиваем на выход
//	if ( GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15) == 1) 
//	{		
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//		GPIO_Init(GPIOA, &GPIO_InitStructure);		
//	}
	
}

void ShowPowerOffForm ( void )
{
  vga_CLEAR();
  vga_SET_POS_TEXT(28,25);
	vga_PRINT_STR("ВЫКЛЮЧЕНИЕ",&FONT_7x10_bold);
	vga_UPDATE();
}

void EXTI9_5_IRQHandler ( void )
{
	uint32_t i;  
	
	if ( (pin_USB_5V) || (usb_transit) )
	{		
			EXTI->PR |= EXTI_PR_PR1; //Сброс флага прерывания		
	}
	else
	{				
			rod_DEINIT();
			SET_CLOCK_SPEED(CLK_72MHz); 
			
			ShowPowerOffForm();
			for(i=0;i<0x1FFFFF;i++){__NOP();}	

			pin_DISC(1);
			DISPLAY_OFF();
			pin_OFF();	
			while(1) IWDG_ReloadCounter();
	}	
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void progressbar_percent(void)
{
	 char str_out2[5];
	 int ii = 1000;
	
	 while (ii--)
	 {
			vga_CLEAR();
			vga_SET_POS_TEXT(1,1);
			vga_PRINT_STR("Форматирование...",&FONT_6x8);
		 
			vga_SET_POS_TEXT(1,20);
			sprintf(str_out2,"%.1f%%", (1000 - ii)/10.0);		 								
			vga_PRINT_STR(str_out2,&FONT_6x8);
			vga_UPDATE();			
		 
	 }	 
	 
}

//процедура форматирования дискаа
TStatus FORMAT(void)
{
  FILE * pFile;
  unsigned char temp_data[512];
	s8 i;
	char str_out[5];
	int i2=0;
	long res = 0;

		
	vga_CLEAR();
	men_SHOW_MESSAGE("Форматирование...","",0);
	
	
	/// Прогресс-бар
	vga_SET_DRAW_MODE(drMODE_NORMAL);
	vga_RECTANGLE(1,30,vga_GET_WIDTH_DISPLAY-3,33,drRECT_NO_FILL);
	vga_UPDATE();	
	vga_SET_DRAW_MODE(drMODE_NORMAL);
	vga_RECTANGLE(1,30,20,33,drRECT_FILL);
	vga_UPDATE();	
	
	IWDG_ReloadCounter();
	
	memset(temp_data,0,512);
		
	
  //форматирование
  mmc_format();//очистка флэш, создание таблицы FAT


	vga_SET_DRAW_MODE(drMODE_NORMAL);
	vga_RECTANGLE(1,30,vga_GET_WIDTH_DISPLAY-3,33,drRECT_NO_FILL);
	vga_UPDATE();		
	vga_SET_DRAW_MODE(drMODE_NORMAL);
	vga_RECTANGLE(1,30,60,33,drRECT_FILL);
	vga_UPDATE();	
	
	
	IWDG_ReloadCounter();
	
 	REGW(NUMFILE,1);
	REGW(NUMFILE_CURENT,0);
	REGW(ROUTE_NUM,0);
	REGW(BEYOND_ROAD,1);

  IWDG_ReloadCounter();


	vga_SET_DRAW_MODE(drMODE_NORMAL);
	vga_RECTANGLE(1,30,vga_GET_WIDTH_DISPLAY-3,33,drRECT_NO_FILL);
	vga_UPDATE();	
	vga_SET_DRAW_MODE(drMODE_NORMAL);
	vga_RECTANGLE(1,30,100,33,drRECT_FILL);
	vga_UPDATE();	

	IWDG_ReloadCounter();

	
	res = rod_CreateFile_edit();	
	
	
//	vga_CLEAR();
//	men_SHOW_MESSAGE("Форматирование...","",0);
	vga_SET_DRAW_MODE(drMODE_NORMAL);
	vga_RECTANGLE(1,30,vga_GET_WIDTH_DISPLAY-3,33,drRECT_NO_FILL);
	vga_UPDATE();	
	vga_SET_DRAW_MODE(drMODE_NORMAL);
	vga_RECTANGLE(1,30,vga_GET_WIDTH_DISPLAY-3,33,drRECT_FILL);
	vga_UPDATE();	

	Delay(1400000);
	
	vga_CLEAR();
	vga_SET_POS_TEXT(1,1);
	sprintf(str_out,"Форматирование");		 								
	vga_PRINT_STR(str_out,&FONT_6x8);
	//vga_UPDATE();						
	vga_SET_POS_TEXT(1,12);
	sprintf(str_out,"завершено.");		 								
	vga_PRINT_STR(str_out,&FONT_6x8);
	vga_UPDATE();	
			

	IWDG_ReloadCounter();

	Delay(1500000);
		

	vga_CLEAR();
  vga_SET_POS_TEXT(28,25);  
	vga_PRINT_STR("ВЫКЛЮЧЕНИЕ",&FONT_7x10_bold);
	vga_UPDATE();
	
	Delay(1500000);
	
	BKP_WriteBackupRegister(BKP_DR12, 0); ///Индикация A,V
		
	pin_OFF();	
	
	return _OK;	
}

TStatus FAT_Init(void)
{
  u32 res;
	unsigned int e = 0;
	char str[5];
  
	SPI_SETUP();		
	
	res = disk_initialize(0);
	
	if (res != 0) 
	{
		while(e++ < 5)
		{
			res = disk_initialize(0);	
			if (res == 0) break;
			Delay(10000);
		}
	}
	
	res = finit();   
	
	//sprintf(str, "%d", res);
	if (res == 2) men_SHOW_MESSAGE("Ошибка MBR", "", 100);		
	if (res == 3) men_SHOW_MESSAGE("Ошибка Boot Record", "", 100);		
	if (res == 4) men_SHOW_MESSAGE("Ошибка FAT", "", 100);				

  return res;	 
}

void CONTROL_RTC(void)
{
//событие 1 секунда
  if (rtc_CHECK_EVENT_1S())
	 {
	  rtc_COPY_RTC_TO_BUFER();
	  REGW(YEAR,rtc_READ(rtc_YEAR));
	  REGW(DATA,rtc_READ(rtc_MON)*100+rtc_READ(rtc_DAY));
	  REGW(TIME,rtc_READ(rtc_HOUR)*100+rtc_READ(rtc_MIN));
	  REGW(SECOND,rtc_READ(rtc_SEC));
	 }     
}	


void JumpToApplication(uint32_t addr)
{
		typedef  void (*pFunction)(void);
		pFunction Jump_To_Application;
		uint32_t JumpAddress;

		JumpAddress = *(uint32_t*) (addr + 4);
		Jump_To_Application = (pFunction) JumpAddress;

		/* Initialize user application's Stack Pointer */
		__set_MSP(*(vu32*) addr);
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, addr-0x8000000);
		Jump_To_Application();		 
}





int main(void)
{

	unsigned int *var = (unsigned int*)0x8010000;
  unsigned int fl=0;
  unsigned int temp_reg;  
  unsigned int i = 0, j = 0, res = 0;
  char 		  FileName[25];
	char temp[25];
	FIL Fil;
	FILINFO fno;
  FILE * pRFile;
	FATFS fatfs;
	unsigned char persent_bat_char = 5;	
	uint16_t Number,Num;	
	uint32_t bt[5];
	unsigned int br = 1;
	unsigned int f_res;
	unsigned int a, carry_flag;
	uint16_t crc = 0xffff;
	uint8_t bf[2];
	
  GPIO_SETUP();

  Delay(100000);
  Delay(100000);
  Delay(100000);		

  //подать 13В
  pin_13V(HIGTH);
  Delay(100000);
  Delay(100000);
  Delay(100000);
  
  SystemInit();
	
  SET_CLOCK_SPEED(CLK_72MHz);   
	
  vga_INIT();	
	
	rtc_SETUP();
	
	res = FAT_Init();
			
	
	///Копируем параметры sd-карты в глоб. переменную для иниц. карты по usb 
	sdinfo = get_mmc();

			
	//men_SHOW_MESSAGE("bootloader1", "", 100);			
	
	
	if (key_CHECK_EV(key_EVENT_PRESSED_MESUARE))
	{
		vga_SET_POS_TEXT(1,1);
		vga_PRINT_STR("Загрузчик 3.0",&FONT_6x8);	
		vga_UPDATE();

		while (!pin_USB_5V);		
			
		USB_Init();
		USB_CONNECT = pin_USB_5V;
		USB_Connect(USB_CONNECT);	
		
		while (1);				
	}	
	else
	{
		//res = disk_status(0);
		res = f_mount(&fatfs, "0:", 1);
		
		res = f_stat("prog1.bin", &fno);
		
		if (res == 0)
		{			
			res = f_open(&Fil, "prog1.bin", FA_OPEN_ALWAYS | FA_READ);
			
			while (1)
			{			
				res = f_read(&Fil, bf, sizeof(uint8_t), &br);
				if (br == 0) break;
				
				crc = crc^bf[0];
				
				for (j = 0; j < 8; j++)
				{
					a = crc;
					carry_flag = a & 0x0001;
					crc = crc >> 1;
					if (carry_flag == 1) crc = crc ^ 0xA001;
				}							
			}			
			

			if (crc != 0) 
			{
				vga_CLEAR();
				vga_SET_POS_TEXT(1,15);
				vga_PRINT_STR("Файл поврежден, ",&FONT_6x8);
				vga_SET_POS_TEXT(1,25);
				vga_PRINT_STR("ошибка CRC.",&FONT_6x8);
				vga_SET_POS_TEXT(1,45);
				vga_PRINT_STR("Нажмите ENTER.",&FONT_6x8);
				vga_UPDATE();								
				
				while (!key_CHECK_EV(key_EVENT_PRESSED_ENTER));		
		
				EXTI9_5_IRQHandler();
				
			}
		}					
							
							
		if (res == 0)
		{			
			vga_CLEAR();
			vga_SET_POS_TEXT(1,1);			
			vga_PRINT_STR("Обновление ПО...",&FONT_6x8);			
			vga_UPDATE();	
			
			
			FLASH_Unlock();
			
			for(i = 0; i<191; i++)
			{			
				FLASH_ErasePage(0x8010000+i*0x800);
			}			
			
			res = f_open(&Fil, "prog1.bin", FA_OPEN_ALWAYS | FA_READ);
			
			if (res == 0)
			{
				i = 0;
				
				while (1)
				{						
						res = f_read( &Fil, bt, sizeof(unsigned int), &br );	
						if (br == 0) break;
						f_res = FLASH_ProgramWord(0x8010000+i*4,bt[0]);
						i++;						
				}
				
				res = f_close(&Fil);				
			}
			
			FLASH_Lock();				
			
			vga_CLEAR();
			vga_SET_POS_TEXT(1,25);
			vga_PRINT_STR("Обновление завершено!",&FONT_6x8);
			vga_SET_POS_TEXT(1,45);
			vga_PRINT_STR("Нажмите ENTER.",&FONT_6x8);
			vga_UPDATE();	
			
			while (!key_CHECK_EV(key_EVENT_PRESSED_ENTER));
			
			res = f_unlink("prog1.bin");
			
			res = f_mount(0, "0:", 0);		
			
			JumpToApplication(0x8010000);			
		}
			
		res = f_mount(0, "0:", 0);
	}	
	
	
	
	vga_SET_POS_TEXT(1,1);
	vga_PRINT_STR("*",&FONT_6x8);		
	vga_UPDATE();
	
	if (*var == 0xffffffff) 
	{			
		while (1)
		{
			if (key_CHECK_EV(key_EVENT_PRESSED_ESC_MENU))
			POWER_OFF();
		}							
	}	
	
	JumpToApplication(0x8010000);

}//конец программы

 


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
