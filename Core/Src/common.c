
#include "common.h"
#include "ILI9488.h"
#include <stdlib.h>

#define SEC_A_DAY 86400

extern RTC_HandleTypeDef hrtc;
/*----------------------------------GLOBVARS--------------------------------*/
tFlags CFlags = {1, 0, 0, 0, 0, 0};
tButton B1;
tButton B2;
tButton B3;
tButton B4;
tButton BJ;
tJoystick joystick = {0};

uint32 timestamp = 0;    // System timer (ms), starts counting from power on or last restart
//tRTC rtcbcd;          // structure for clock/date from RTC module (BCD format))
tRTC rtcraw;          // structure for system clock/date (uint8)
uint8 Ubat = 255;              // ADC data from battery level measurement
uint8 batlvl = 0;            // battery level for display (0...5)
uint8 brightlvl;         // brightness level for display (0...7)
uint8 brightPWM = 220;   // PWM duty cycle value for regulate display brightness

/*----------------------------------UTILITIES-------------------------------*/
int clamp(int val, int min, int max)
{
  if(val > max) val = max;
  if(val < min) val = min;
  return val;
}

void randinit(void)
{
  srand((uint8)timestamp);
}

uint8 getrand(uint8 N)
{
  return (uint8_t)(rand() % (N + 1));
}

uint8 dig_to_smb(uint8 dig)
{
  switch (dig)
  {
    case 0: return '0';
    case 1: return '1'; 
    case 2: return '2';
    case 3: return '3'; 
    case 4: return '4';
    case 5: return '5'; 
    case 6: return '6'; 
    case 7: return '7'; 
    case 8: return '8';
    case 9: return '9';
    default: return 0;
  }
  return 0;
}

void u16_to_str(uint8* str, uint16 num, uint8 N)
{ 
  //sprintf((char*)str, "%u", num);
  
  str[0] = dig_to_smb((uint8)(num/10000));
  num %= 10000;
  str[1] = dig_to_smb((uint8)(num/1000));
  num %= 1000;
  str[2] = dig_to_smb((uint8)(num/100));
  num %= 100;
  str[3] = dig_to_smb((uint8)(num/10));
  str[4] = dig_to_smb((uint8)(num%10));
  str[5] = '\0';
  
  if(N == 10)
  {
    uint8 chars = 0;
    for(int i = 0; i < 6; i++)
    {
      if((str[i] == '0') && !chars) continue;
      str[chars] = str[i];
      chars++;
    }
    if(chars == 1)
    {
      str[0] = '0';
      str[1] = '\0';
    }
  }
      
  if((N < 5) && (N > 0))
  {
    for(uint8 i = 0; i <= N; i++) 
      str[i] = str[5 - N + i];
  }
}
/*----------------------------------------------------------------------------*/

/*-------------------------------INITIALIZATION-------------------------------*/

void commoninit(void)
{
  HAL_GPIO_WritePin(PWR_OFF_GPIO_Port, PWR_OFF_Pin, (GPIO_PinState)SET);
  //HAL_GPIO_WritePin(SOUND__GPIO_Port, SOUND__Pin, RESET);
  //HAL_GPIO_WritePin(SOUND_B5_GPIO_Port, SOUND_B5_Pin, RESET);
  //initbuttons();
  brightPWM = EEPROM_readbyte(PWM_MEMADR);
  if(brightPWM == 0) brightPWM = 220;
  BrightPWMgen(brightPWM);
}

/*void initbuttons(void)
{
  B1 = CreateBtn(BTN_1_GPIO_Port, BTN_1_Pin);
  B2 = CreateBtn(BTN_2_GPIO_Port, BTN_2_Pin);
  B3 = CreateBtn(BTN_3_GPIO_Port, BTN_3_Pin);
  B4 = CreateBtn(BTN_4_GPIO_Port, BTN_4_Pin);
}*/
/*----------------------------------------------------------------------------*/

/*------------------------------SYSTEM FUNCTIONS------------------------------*/
uint8 getbatlvl(uint8 Ub)
{
  uint8 lvl = (uint8_t)((2550 - (((uint16_t)Ub) * 10)) / 133);
  static uint8 Umax = 255;
  static uint8 Umin = 242;
  static uint8 reslvl = 0;
  if(Ub < 175) return 100; // bat to low, immediately shotdown code - 100
  Ub = clamp(Ub, 175, 255);
  if((Ub <= Umax) && (Ub >= Umin))
  {
	  return reslvl;
  }
  else 
  {
    switch(lvl)
    {
      case 0: Umin = 242; Umax = 255; reslvl = 0; break;
      case 1: Umin = 228; Umax = 246; reslvl = 1; break;
      case 2: Umin = 215; Umax = 232; reslvl = 2; break;
      case 3: Umin = 202; Umax = 219; reslvl = 3; break;
      case 4: Umin = 189; Umax = 206; reslvl = 4; break;
      case 5: Umin = 175; Umax = 193; reslvl = 5; break;
    }
  }
  return reslvl;
}

void batcheck(void)
{
  batlvl = getbatlvl(Ubat);
  if(batlvl == 100) ShutDownLB();
}
        
void ShutDownLB(void)
{
	draw_fast_string(25,100, COLOR_RED, COLOR_BLACK, "BATTARY LOW!");
	draw_fast_string(25,115, COLOR_RED, COLOR_BLACK, "The device will turn off now!");
	HAL_GPIO_WritePin(PWR_OFF_GPIO_Port, PWR_OFF_Pin, RESET);
	while(1);
}

void ShutDown(void)
{
  draw_fast_string(25,100, COLOR_RED, COLOR_BLACK, "Shutdown!");
  HAL_GPIO_WritePin(PWR_OFF_GPIO_Port, PWR_OFF_Pin, RESET);
  while(1);
}

void getbrightlvl(void)
{
  brightlvl = ((brightPWM - 10) / 30) - 1;
}

void decbright(void)
{
  if(brightPWM <= 30) brightPWM = 50;
  brightPWM -=20;
  EEPROM_writebyte(PWM_MEMADR, brightPWM);
  BrightPWMgen(brightPWM);
}

void incbright(void)
{
  if(brightPWM >= 250) brightPWM = 230;
  brightPWM +=20;
  EEPROM_writebyte(PWM_MEMADR, brightPWM);
  BrightPWMgen(brightPWM);
}

extern TIM_HandleTypeDef htim11;

void BrightPWMgen(uint8 duty_cycle)
{
	uint16 dutyCycle = ((uint16)duty_cycle) * 20;
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, dutyCycle);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}

/*
void Sounds(uint16 delay)
{
  volatile uint32 j;
  for(uint16 i = 0; i < (15000/delay); i++)
  {  
  	HAL_GPIO_WritePin(SOUND__GPIO_Port, SOUND__Pin, SET);
  	HAL_GPIO_WritePin(SOUND_B5_GPIO_Port, SOUND_B5_Pin, RESET);
    j = delay * 6;
    while(--j){}
  	HAL_GPIO_WritePin(SOUND__GPIO_Port, SOUND__Pin, RESET);
  	HAL_GPIO_WritePin(SOUND_B5_GPIO_Port, SOUND_B5_Pin, SET);
    j = delay * 6;
    while(--j){}
  }
}

void RTC_SetCounter_(uint32_t count)                                                    //Ã‡Ã Ã¯Ã¨Ã±Ã Ã²Ã¼ Ã­Ã®Ã¢Ã®Ã¥ Ã§Ã­Ã Ã·Ã¥Ã­Ã¨Ã¥ Ã±Ã·Ã¥Ã²Ã·Ã¨ÃªÃ 
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;            //Ã¢ÃªÃ«Ã¾Ã·Ã¨Ã²Ã¼ Ã²Ã ÃªÃ²Ã¨Ã°Ã®Ã¢Ã Ã­Ã¨Ã¥ PWR Ã¨ Backup
  PWR->CR |= PWR_CR_DBP;                                            //Ã°Ã Ã§Ã°Ã¥Ã¸Ã¨Ã²Ã¼ Ã¤Ã®Ã±Ã²Ã³Ã¯ Ãª Backup Ã®Ã¡Ã«Ã Ã±Ã²Ã¨
  while (!(RTC->CRL & RTC_CRL_RTOFF));                              //Ã¯Ã°Ã®Ã¢Ã¥Ã°Ã¨Ã²Ã¼ Ã§Ã ÃªÃ®Ã­Ã·Ã¥Ã­Ã» Ã«Ã¨ Ã¨Ã§Ã¬Ã¥Ã­Ã¥Ã­Ã¨Ã¿ Ã°Ã¥Ã£Ã¨Ã±Ã²Ã°Ã®Ã¢ RTC
  RTC->CRL |= RTC_CRL_CNF;                                          //Ã�Ã Ã§Ã°Ã¥Ã¸Ã¨Ã²Ã¼ Ã‡Ã Ã¯Ã¨Ã±Ã¼ Ã¢ Ã°Ã¥Ã£Ã¨Ã±Ã²Ã°Ã» RTC
  RTC->CNTH = count>>16;                                                              //Ã§Ã Ã¯Ã¨Ã±Ã Ã²Ã¼ Ã­Ã®Ã¢Ã®Ã¥ Ã§Ã­Ã Ã·Ã¥Ã­Ã¨Ã¥ Ã±Ã·Ã¥Ã²Ã­Ã®Ã£Ã® Ã°Ã¥Ã£Ã¨Ã±Ã²Ã°Ã 
  RTC->CNTL = count;
  RTC->CRL &= ~RTC_CRL_CNF;                                                       //Ã‡Ã Ã¯Ã°Ã¥Ã²Ã¨Ã²Ã¼ Ã§Ã Ã¯Ã¨Ã±Ã¼ Ã¢ Ã°Ã¥Ã£Ã¨Ã±Ã²Ã°Ã» RTC
  while (!(RTC->CRL & RTC_CRL_RTOFF));                                         //Ã„Ã®Ã¦Ã¤Ã Ã²Ã¼Ã±Ã¿ Ã®ÃªÃ®Ã­Ã·Ã Ã­Ã¨Ã¿ Ã§Ã Ã¯Ã¨Ã±Ã¨
  PWR->CR &= ~PWR_CR_DBP;                                                         //Ã§Ã Ã¯Ã°Ã¥Ã²Ã¨Ã²Ã¼ Ã¤Ã®Ã±Ã²Ã³Ã¯ Ãª Backup Ã®Ã¡Ã«Ã Ã±Ã²Ã¨
}

uint32 RTC_GetCounter_(void)                                                             //Ã�Ã®Ã«Ã³Ã·Ã¨Ã²Ã¼ Ã§Ã­Ã Ã·Ã¥Ã­Ã¨Ã¥ Ã±Ã·Ã¥Ã²Ã·Ã¨ÃªÃ 
{
          return  (uint32)((RTC->CNTH << 16) | RTC->CNTL);
}

void timer_to_cal (uint32 timer, tRTC* RTCdat)
{
	uint32 a;
	uint8 b;
	uint8 c;
	uint8 d;
	uint32 time;

	time = timer % SEC_A_DAY;
	a = ((timer + 43200) / (86400 >> 1)) + (2440587 <<1 ) + 1;
	a >>= 1;
	RTCdat->weekday = a % 7;
	a += 32044;
	b = (4 * a + 3) / 146097;
	a = a - (146097 * b) / 4;
	c = (4 * a + 3) / 1461;
	a = a - (1461 * c) / 4;
	d = (5 * a + 2) / 153;
	RTCdat->day = a - (153 * d + 2) / 5 + 1;
	RTCdat->month = d + 3 - 12 * (d / 10);
	RTCdat->year = 100 * b + c - 4800 + (d / 10);
	RTCdat->hour = time / 3600;
	RTCdat->min = (time % 3600) / 60;
	RTCdat->sec = (time % 3600) % 60;
}

uint32 cal_to_timer (tRTC* RTCdat)
{
	uint32 a;
	int32 y;
	uint32 m;
	uint32 Uday;
	uint32 time;

	a = ((14 - RTCdat->month) / 12);
	y = RTCdat->year + 4800 - a;
	m = RTCdat->month + (12 * a) - 3;
	Uday = (RTCdat->day + ((153 * m + 2) / 5) + 365 * y + (y / 4) - (y / 100) + (y / 400) - 32045) - 2440588;
	time = Uday * 86400;
	time += RTCdat->sec + RTCdat->min * 60 + RTCdat->hour * 3600;
	return time;
}


void RTCgetdata(tRTC* RTCdat)
{
	timer_to_cal (RTC_GetCounter_(), RTCdat);
}

void RTCsenddata(tRTC* RTCdat)
{
	RTC_SetCounter_(cal_to_timer(RTCdat));
}
*/
/*
void rtcrawtobcd(void)
{
  rtcbcd.year = (uint8)((rtcraw.year / 10) << 4) | (rtcraw.year % 10);
  rtcbcd.month = (0x80 | (uint8)((rtcraw.month / 10) << 4)) | rtcraw.month % 10;
  rtcbcd.day = (uint8)((rtcraw.day / 10) << 4) | (rtcraw.day % 10);
  rtcbcd.weekday = rtcraw.weekday;
  rtcbcd.hour = (uint8)((rtcraw.hour / 10) << 4) | (rtcraw.hour % 10);
  rtcbcd.min = (uint8)((rtcraw.min / 10) << 4) | (rtcraw.min % 10);
  rtcbcd.sec = (uint8)((rtcraw.sec / 10) << 4) | (rtcraw.sec % 10);
}

void rtcbcdtoraw(void)
{
  rtcraw.year = (rtcbcd.year >> 4) * 10 + (rtcbcd.year & 0x0F);
  rtcraw.month = ((rtcbcd.month & 0x1F) >> 4) * 10 + (rtcbcd.month & 0x0F);
  rtcraw.day = ((rtcbcd.day & 0x3F) >> 4) * 10 + (rtcbcd.day & 0x0F);
  rtcraw.weekday = rtcbcd.weekday & 0x07;
  rtcraw.hour = ((rtcbcd.hour & 0x3F) >> 4) * 10 + (rtcbcd.hour & 0x0F);
  rtcraw.min = ((rtcbcd.min & 0x7F) >> 4) * 10 + (rtcbcd.min & 0x0F);
  rtcraw.sec = ((rtcbcd.sec & 0x7F) >> 4) * 10 + (rtcbcd.sec & 0x0F);
}*/
/*----------------------------------------------------------------------------*/

/*-----------------------------BUTTONS FUNCTIONS------------------------------*/
uint16  BTN_HOLD_ON_DELAY = 300;
uint16  BTN_STUCK_ON_DELAY = 2000;

tButton CreateBtn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  tButton BTN;
  BTN.Port = GPIOx;
  BTN.Pin = GPIO_Pin;
  BTN.BtnFl = 0;
  BTN.BtnON = 0;
  BTN.Toggle = 0;
  BTN.HoldON = 0;
  BTN.StuckON = 0;
  BTN.btnTimer = 0;
  return BTN;
}

void check_btn_jstk(void) //Test buttons and joystick
{
  TestBtn(&B1);
  TestBtn(&B2);
  TestBtn(&B3);
  TestBtn(&B4);
  checkjoydir();
}

void TestBtn(tButton* btn)
{
  if (!HAL_GPIO_ReadPin(btn->Port, btn->Pin) && !btn->BtnFl && ((HAL_GetTick() - btn->btnTimer) > 30)) {
    if(btn->Toggle) btn->Toggle = 0; 
    else btn->Toggle = 1;
    btn->BtnFl = 1;
    btn->btnTimer = HAL_GetTick();
    btn->BtnON = 1;
  }
  if (HAL_GPIO_ReadPin(btn->Port, btn->Pin) && ((HAL_GetTick() - btn->btnTimer) > 30)) {
    btn->BtnFl = 0;
    btn->HoldON = 0;
    btn->StuckON = 0;
    btn->btnTimer = HAL_GetTick();
  }
  if (!HAL_GPIO_ReadPin(btn->Port, btn->Pin) && btn->BtnFl && ((HAL_GetTick() - btn->btnTimer) > BTN_HOLD_ON_DELAY) && ((HAL_GetTick() - btn->btnTimer) <= BTN_STUCK_ON_DELAY)) {
    btn->HoldON = 1;
    if(btn->Toggle) btn->Toggle = 0; 
    else btn->Toggle = 1;
  }
  if (!HAL_GPIO_ReadPin(btn->Port, btn->Pin) && btn->BtnFl && ((HAL_GetTick() - btn->btnTimer) > BTN_STUCK_ON_DELAY)) {
    btn->HoldON = 0;
    btn->StuckON = 1;
    if(btn->Toggle) btn->Toggle = 0; 
    else btn->Toggle = 1;
  }
}

void checkjoydir(void)
{
    if(joystick.oy > 150 && joystick.joyFl == 0) {
        joystick.up = 1; 
        joystick.joyFl = 1;
    }
    if(joystick.oy < 100 && joystick.joyFl == 0) {
        joystick.down = 1; 
        joystick.joyFl = 1;
    }
    if(joystick.ox > 150 && joystick.joyFl == 0) {
        joystick.right = 1; 
        joystick.joyFl = 1;
    }
    if(joystick.ox < 100 && joystick.joyFl == 0) {
        joystick.left = 1; 
        joystick.joyFl = 1;
    }
    if(joystick.oy < 150 && joystick.oy > 100 && joystick.ox < 150 && joystick.ox > 100) joystick.joyFl = 0;
}

void pressbutton(tButton* btn, void (*func)(void))
{
	if(btn->BtnON)
	{
		btn->BtnON = 0;
		func();
	}
}
/*----------------------------------------------------------------------------*/

/*---------------------------SAVE/LOAD FUNCTIONS------------------------------*/

void EEPROM_writebyte(uint8 addr, uint8 byte) // addr 0......39
{
	uint32 temp = 0;
	temp = HAL_RTCEx_BKUPRead(&hrtc, (1 + addr/2));
	if(addr % 2) {
		temp &= 0x000000FF;
		temp |= ((uint32)byte) << 8;
		HAL_RTCEx_BKUPWrite(&hrtc, (1 + addr/2), temp);
	}
	else {
		temp &= 0x0000FF00;
		temp |= (uint32)byte;
		HAL_RTCEx_BKUPWrite(&hrtc, (1 + addr/2), temp);
	}
}

uint8 EEPROM_readbyte(uint8 addr)   // addr 0......39
{
	uint32 temp = 0;
	temp = HAL_RTCEx_BKUPRead(&hrtc, (1 + addr/2));
	if(addr % 2) {
		return (uint8)(temp >> 8);
	}
	else {
		return (uint8)temp;
	}
}

