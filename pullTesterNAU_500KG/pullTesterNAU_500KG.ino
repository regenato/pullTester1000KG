/*
 * Date Of Creation: 11 June 2019
 * Customer: Crimp
 * on: 24/10/19: minimum pul is set to 1.5 KGF
 */
#ifndef F_CPU
#define F_CPU  16e6
#endif

#include <TimerThree.h>
#include <SPI.h>  // include the SPI library
#include <SD.h>
#include "DigitalIO.h"
#include <Time.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <EEPROM.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define PULL1000                     //PULL50 for SP01, PULL100 for SP02
#define SPDDOUBLE                   //hide this for speed 25, 50, 75, 100
#define WIFIEN                      //hide this line to disable WIFI
//#define SER_DEBUG

#define SERVICE 60000
#define SERVICELMT  59500

const int chipSelect = 56;

const uint8_t SOFT_SPI_MISO_PIN = 27;
const uint8_t SOFT_SPI_MOSI_PIN = 39;
const uint8_t SOFT_SPI_SCK_PIN  = 26;
const uint8_t SPI_MODE = 0;

SoftSPI<SOFT_SPI_MISO_PIN, SOFT_SPI_MOSI_PIN, SOFT_SPI_SCK_PIN, SPI_MODE> SPI2;
tmElements_t tm;
NAU7802 myScale; //Create instance of the NAU7802 class


#define INTEE
/*no address should be assign between 100 to 6350*/
#define ADDPULLMODE 0
#define ADDPULLVAL  2
#define ADDPULLUNIT 6
#define ADDPULLSPD  8
#define ADDFD   12
#define ADDOFF    16
#define ADDSDID   20
#define ADDRAW  30
#define ADDFINAL  40
#define ADDVAL  50
#define ADDCALDATE  60
#define ADDSERVICE  70
/*****************************Internal EEPROM addresses************************************************/
/*#define ADR_RAW   10
#define ADR_FINAL 18
#define ADR_VAL   26*/
#define ADR_SERVICE 34
#define ADR_CALDATE 40    //38  8 byte value
#define ADR_NOLOAD  50    //48

#define LOCATION_CALIBRATION_FACTOR 10 //Float, requires 4 bytes of EEPROM
#define LOCATION_ZERO_OFFSET 20 //Must be more than 4 away from previous spot. Long, requires 4 bytes of EEPROM

/***********************units for convertion*****************************************/
#define U_Kgf    0
#define U_N     1
#define U_Lbf   2
#define SDMAX   200
//for 100KG PullTester

#ifdef  PULL1000
#define MAX_PULL_K 1000
#define MAX_PULL_N  9800
#define MAX_PULL_L  2200
#define MAX_SPD   250
#define MAX_LOAD  9500
#endif

#ifdef  PULL500
#define MAX_PULL_K 500
#define MAX_PULL_N  4900
#define MAX_PULL_L  1100
#define MAX_SPD   250
#define MAX_LOAD  4950
#endif

#ifdef  PULL200
#define MAX_PULL_K 200
#define MAX_PULL_N  1960
#define MAX_PULL_L  440
#define MAX_SPD   250
#define MAX_LOAD  1800
#endif

//for 100KG PullTester
#ifdef  PULL100
#define MAX_PULL_K 100
#define MAX_PULL_N  980
#define MAX_PULL_L  220
#define MAX_SPD   250
#define MAX_LOAD  900
#endif

//for 50KG PullTester
#ifdef  PULL50
#define MAX_PULL_K  50
#define MAX_PULL_N  490
#define MAX_PULL_L  110
#define MAX_SPD   250
#define MAX_LOAD  450
#endif

#define EXP_NUMBER  20
#define PAGE_START  100
#define PAGE_SIZE 25
#define TOTAL_PAGE  250
#define NEX_FREE_SPACE  5101

#define EXP_OFF     0
#define DATE_OFF    1
#define TIME_OFF    9
#define SPEED_OFF   14
#define UNIT_OFF    15
#define PEEKLOAD_OFF  16

#define LCD_RS  5
#define LCD_EN  2
#define LCD_D0  3
#define LCD_D1  17
#define LCD_D2  16
#define LCD_D3  6
#define LCD_D4  7
#define LCD_D5  8
#define LCD_D6  10
#define LCD_D7  11

#define LINE1 0x00
#define LINE2 0x40
#define LINE3 0x14
#define LINE4 0x54
#define SCALE_PDWN  25//8
#define SCALE_TEMP  28//9
#define SCALE_DRDY  27//12
#define SCALE_CLK   26//13

#define MOTOR_PWM 44
#define MOTOR_D1  45
#define MOTOR_D2  46
#define LIMIT_SW  37
#define LIMIT_FLT 36
#define LED_H 32
#define LED_O 31

#define LED 69

#define BUZZ  12
#define HOMEK   68//83
#define STOPK   67//84
#define DOWNK   66//85
#define EXITK   65//86
#define SAVEK   64//87
#define UPK     63//88
#define LINE4K  62//89
#define LINE3K  61//90
#define LINE2K  60//91
#define LINE1K  59//92
#define STARTK  58//93
#define MEMK    57//94
#define FHOME    0x0001
#define FSTOP   0x0002
#define FDOWN   0x0004
#define FEXIT   0x0008
#define FSAVE   0x0010
#define FUP     0x0020
#define FLINE4    0x0040
#define FLINE3    0x0080
#define FLINE2    0x0100
#define FLINE1    0x0200
#define FSTART    0x0400
#define FMEM    0x0800

#define NVRAM_CS  53  //19

#define   WREN      0x06            // Set Write Enable Latch
#define   WRDI      0x04            // Write Disable
#define   RDSR      0x05            // Read Status Register
#define   WRSR      0x01            // Write Status Register
#define   READ      0x03            // Read Memory Data.
#define   FSTRD     0x0B            // Fast Read Memory Data.
#define   WRITE     0x02            // Write Memory Data
#define   SLEEP     0xB9            // Enter Sleep Mode.
#define   RDID      0x9F            // Read Device ID.
#define   SNR       0xC3            // Read S/N.

unsigned int timeOut;
unsigned char dta[4];

unsigned char wIndx;
bool wDone=0, ipDone=0, wStart=0;
char wBuf[25], ipStr[20];

unsigned char lcd_rs, wCmd;
/* Private variables ---------------------------------------------------------*/
unsigned char tBuf[10], lineCnt, passBuf[5], pwmBuf[4]={50, 100, 150, 200};
unsigned int temp, pwmCount, difVal, diffComp1;
float diff, fd, finalValue;
unsigned long rawCount, finalCount, calVal;
unsigned int tempFd, offSet;
unsigned char done, hr, mn, sc, dy, mnt, yar, dte,yr;
unsigned char buf[50], buf3[10], peakBuf[10];//, systemVar[10];
unsigned long tempValue, initCnt, initCnt2, peakValue, convVal;
unsigned char pullMode, pullUnit, pullSpeed, sign, prvUnit;
unsigned long resultCount, prvCount, finalResultCount, startTime, diffTime, tempTime;
unsigned int minPull, minPullExp, spd, maxPull, maxOver;
unsigned char expNumber, lmt1, lmt2, lmt3, lmt4, lmt5;
#ifdef SPDDOUBLE
//unsigned char noLoadArr[]={35, 28, 23, 21};
unsigned char spdArr[4]={50,100,150,200};
#else
//unsigned char noLoadArr[]={65, 50, 40, 35};
unsigned char spdArr[4]={25,50,75,100};
#endif
//unsigned char noLoadArr[]={65, 50, 36, 28};
unsigned char noLoadArr[]={48, 36, 29, 25};
unsigned char minLoad=50, setPt, pcFlag, cmd;
unsigned char cmdBuf[10];
unsigned char buzFlag, buzSync, runSync, ddg;
unsigned int buzTimer, buzPitchOn, buzPitchOff, ovlCount, mnlCount;
unsigned int serviceCount, sdCounter, sizId, sizTerm, sizMin, sizUnit, sizInfo, pullSucc;
unsigned char calDay, calMonth, calYear;
bool serviceAlFlag, serviceFlag;
unsigned char tgtDay, tgtMonth, tgtMonthAl, tgtYear, leftServiceDay;

unsigned char exemode1(void);
void callback(void);
void buzInit(void);

void HAL_Delay(unsigned int dly)
{
  delay(dly);
}

void shout_595(unsigned char val)
{
  if(val&0x80)
    digitalWrite(LCD_D7, HIGH);
  else
    digitalWrite(LCD_D7, LOW);
  if(val&0x40)
    digitalWrite(LCD_D6, HIGH);
  else
    digitalWrite(LCD_D6, LOW);
  if(val&0x20)
    digitalWrite(LCD_D5, HIGH);
  else
    digitalWrite(LCD_D5, LOW);
  if(val&0x10)
    digitalWrite(LCD_D4, HIGH);
  else
    digitalWrite(LCD_D4, LOW);
  if(val&0x08)
    digitalWrite(LCD_D3, HIGH);
  else
    digitalWrite(LCD_D3, LOW);
  if(val&0x04)
    digitalWrite(LCD_D2, HIGH);
  else
    digitalWrite(LCD_D2, LOW);
  if(val&0x02)
    digitalWrite(LCD_D1, HIGH);
  else
    digitalWrite(LCD_D1, LOW);
  if(val&0x01)
    digitalWrite(LCD_D0, HIGH);
  else
    digitalWrite(LCD_D0, LOW);
}

void lcd_write(unsigned char cd)
{
  if(lcd_rs==1)
          digitalWrite(LCD_RS,HIGH);
  else
          digitalWrite(LCD_RS,LOW);
  shout_595(cd);                      //high byte
  digitalWrite(LCD_EN,HIGH);
  delayMicroseconds(300);
  digitalWrite(LCD_EN,LOW);
  delayMicroseconds(600);
  //delay(3);
}

void lcd_init(void)
{
//  TRISD=0x00;

  lcd_rs=0;
  delay(100);
  lcd_write(0x30);
  delay(100);
  lcd_write(0x30);
  delay(100);
  lcd_write(0x30);
  delay(50);
  lcd_write(0x38);
  delay(50);
  lcd_write(0x08);
  delay(50);
  lcd_write(0x01);
  delay(50);
  lcd_write(0x06);
  delay(50);
  lcd_write(0x0c);
  delay(50);
}

void switch_cursor(unsigned char cursor)
{
  lcd_rs=0;
  delay(50);
  if(cursor)
    lcd_write(0x0E|0x01);
  else
    lcd_write(0x0C);
  delay(50);
} 

void lcd_goto(unsigned char pos)
{
  lcd_rs=0;
  delay(50);
  lcd_write(0x80+pos);
  //delay(100);
}

void lcd_clear(void)
{
  lcd_rs=0;
  lcd_write(0x01);
  delay(50);

}

void lcd_putch(unsigned char ch)
{
  lcd_rs=1;
  //wait1();
  lcd_write(ch);
  //wait1();
}

void lcd_puti2(unsigned char ch)
{
  lcd_putch(ch/10+0x30);
  lcd_putch(ch%10+0x30);
}

void lcd_putstr(unsigned char *str,int noc,int pos)
{
  lcd_goto(pos);
  lcd_rs=1;   
  while(noc--)
  {
    //wait1();
    lcd_write(*str++);    
  }
}

void lcd_putstr(const char *str,int noc,int pos)
{
  lcd_goto(pos);
  lcd_rs=1;   
  while(noc--)
  {
    //wait1();
    lcd_write(*str++);    
  }
}

void lcd_puti(unsigned int v)
{
  unsigned int a;
  lcd_putch((v/10000)+0x30);
  a=v%10000;
  lcd_putch((a/1000)+0x30);
  a=a%1000;
  lcd_putch((a/100)+0x30);
  a=a%100;
  lcd_putch((a/10)+0x30);
  lcd_putch((a%10)+0x30);
}

void lcd_putid(unsigned int v)
{
  unsigned int a;
  lcd_putch((v/10000)+0x30);
  a=v%10000;
  lcd_putch((a/1000)+0x30);
  a=a%1000;
  lcd_putch((a/100)+0x30);
  a=a%100;
  lcd_putch((a/10)+0x30);
  lcd_putch('.');
  lcd_putch((a%10)+0x30);
}

void lcd_putistr(unsigned char *str,unsigned char len,unsigned char pos)
{
  unsigned char chl, enb=0;
  lcd_goto(pos);
  lcd_rs=1;
  while(len--)
  {
    chl=*str++;
    if((chl==0x30)&&(!enb)&&(len>0))
      lcd_write(' ');
    else
    {
      enb=1;
      lcd_write(chl);
    }
  }
}

void lcd_puthstr(unsigned char *str,unsigned char len,unsigned char pos)
{
  unsigned char chl;
  lcd_goto(pos);
  lcd_rs=1;
  while(len--)
  {
    chl=*str++;
    lcd_write(((chl&0xf0)>>4)+0x30);
    lcd_write((chl&0x0f)+0x30);
  }
}

void lcd_putbch(unsigned char ch, unsigned char pos)
{
  lcd_goto(pos);
  lcd_putch((ch/10)+0x30);
  lcd_putch((ch%10)+0x30);
}

void lcd_putbch3(unsigned char ch, unsigned char pos)
{
  lcd_goto(pos);
  lcd_putch((ch/100)+0x30);
  ch=ch%100;
  lcd_putch((ch/10)+0x30);
  lcd_putch((ch%10)+0x30);
}

void lcd_putTime(unsigned char ch, unsigned char pos)
{
  lcd_goto(pos);
  lcd_putch(((ch&0xf0)>>4)+0x30);
  lcd_putch((ch&0x0f)+0x30);
}

void lcd_putstr2(unsigned char *str,int noc,int pos)
{
  lcd_goto(pos);
  lcd_rs=1;   
  while(noc--)
  {
    //wait1();
    lcd_write(*str++);    
  }
}

long scaleRead(void)
{
  long data = 0,abc;
  data = myScale.getWeight(true, 10);//getReading();
  //data>>=3;
  return data;
}

/**********KEYPAD ROUTINE STARTS********************/
unsigned int scanKey(void)
{
  unsigned int retKey;
  retKey=0;
  if(!digitalRead(HOMEK))
  {
    retKey|=0x0001;
  }
  if(!digitalRead(STOPK))
  {
    retKey|=0x0002;
  }
  if(!digitalRead(DOWNK))
  {
      retKey|=0x0004;
  }
  if(!digitalRead(EXITK))
  {
      retKey|=0x0008;
  }
  if(!digitalRead(SAVEK))
  {
      retKey|=0x0010;
  }
  if(!digitalRead(UPK))
  {
      retKey|=0x0020;
  }
  if(!digitalRead(LINE4K))
  {
    retKey|=0x0040;
  }
  if(!digitalRead(LINE3K))
  {
    retKey|=0x0080;
  }
  if(!digitalRead(LINE2K))
  {
      retKey|=0x0100;
    }
  if(!digitalRead(LINE1K))
  {
      retKey|=0x0200;
    }
  if(!digitalRead(STARTK))
  {
      retKey|=0x0400;
    }
  if(!digitalRead(MEMK))
  {
      retKey|=0x0800;
    }
  return retKey;
}

unsigned int getKey(void)
{
  unsigned int retVal;
  do{
    retVal=scanKey();
  }while(retVal==0);
  delay(100);
  return retVal;
}

unsigned int getKeyh(void)
{
  unsigned int prvVal, keyVal;
  prvVal=getKey();
  do{
    keyVal=scanKey();
  }while(keyVal!=0);
  delay(100);
  return prvVal;
}
/**********************KEYPAD ENDS********************************/
/**********************NVRAM ROUTINES STARTS**********************/
void FM25V02_Init(void)
{
  // Initialization FM Device Control.
  digitalWrite(NVRAM_CS, HIGH);
}

static unsigned char MSPISendData(unsigned char uiData)
{
  unsigned char retVal;
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
  retVal=SPI.transfer(uiData);//HAL_SPI_Transmit(&hspi2,&uiData,1,100);
  SPI.endTransaction();
  return(retVal);
}

static void FM25V02_WREN(void)
{
  digitalWrite(NVRAM_CS,LOW);
  MSPISendData(WREN);
  delay(1);
  digitalWrite(NVRAM_CS,HIGH);
}

static void FM25V02_WRDI(void)
{
  digitalWrite(NVRAM_CS,LOW);
  MSPISendData(WRDI);
  delay(1);
  digitalWrite(NVRAM_CS,HIGH);
}

void FM25V02_WRSR(unsigned char Reg_Status)
{
  FM25V02_WREN();
  delay(1);
  digitalWrite(NVRAM_CS, LOW);
  MSPISendData(WRSR);
  MSPISendData(Reg_Status);
  delay(1);
  digitalWrite(NVRAM_CS, HIGH);
}

unsigned char FM25V02_RDSR(void)
{
  unsigned char Reg_Status;
  digitalWrite(NVRAM_CS, LOW);
  MSPISendData(RDSR);
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
  Reg_Status=(unsigned char)SPI.transfer(0);
  SPI.endTransaction();
  delay(1);
  digitalWrite(NVRAM_CS, HIGH);
  return(Reg_Status);
}

void FM25V02_PWDN(void)
{
  digitalWrite(NVRAM_CS, LOW);
  MSPISendData(SLEEP);
  delay(2);
  digitalWrite(NVRAM_CS, HIGH);
}

void FM25V02_WKUP(void)
{
  digitalWrite(NVRAM_CS, HIGH);
  digitalWrite(NVRAM_CS, LOW);
  delay(4);
  digitalWrite(NVRAM_CS, HIGH);
}

void FM25V02_WRITE(unsigned long WAddr, unsigned char *pBuf, unsigned long num)
{
  unsigned char  WAddrL, WAddrM;
  WAddrL =   WAddr & 0x000000FF;
  WAddrM = ((WAddr >>  8) & 0x0000007F);

  FM25V02_WREN();                                                     /* step 1 .  WEL = 1            */
  delay(1);
  digitalWrite(NVRAM_CS, LOW);
  MSPISendData(WRITE);

  MSPISendData(WAddrM);
    MSPISendData(WAddrL);

    for (; num > 0; num--) {                                            /* step 4 . send out bytes      */
        MSPISendData(*pBuf++);
    }
    digitalWrite(NVRAM_CS, HIGH);
    FM25V02_WRDI();
}

void FM25V02_READ(unsigned long RAddr, unsigned char *pBuf, unsigned long num)
{
  unsigned char RAddrL, RAddrM;
  int k;
  RAddrL =   RAddr & 0x000000FF;
  RAddrM = ((RAddr >>  8) & 0x0000007F);

  digitalWrite(NVRAM_CS, LOW);
  MSPISendData(READ);

    MSPISendData(RAddrM);
    MSPISendData(RAddrL);
    
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
  k=0;
    for (;num > 0; num--) {
      buf[k]=SPI.transfer(0xff);
        //HAL_SPI_Receive(&hspi2,pBuf,1,100);
        k++;
    }
  SPI.endTransaction();
    delay(1);
    digitalWrite(NVRAM_CS, HIGH);
}
/*********************NVRAM ENDS**********************************/
void int2str(unsigned long tvar, unsigned char *tbuf, unsigned char len)
{
  unsigned long tmp1,tmp3;
  unsigned char tmp2;
  tmp3=1;
  for(tmp2=0;tmp2<(len-1);tmp2++)
  {
    tmp3=(10*tmp3);
  }
  tmp1=tvar;
  for(;len>0;len--)
  {
    tmp2=tmp1/tmp3;
    *tbuf++=(tmp2+0x30);
    tmp1=tvar%tmp3;
    tmp3/=10;
  }
  *tbuf='\0';
}

unsigned int atolN(char *p)
{
  unsigned int tmpN, tmpValN=0;
  while(*p)
  {
    tmpValN*=10;
    tmpN=*p++;
    tmpValN+=(tmpN&0x0f);
  }
  return tmpValN;
}

unsigned long atolL(char *p)
{
  unsigned long tmpN, tmpValN=0;
  while(*p)
  {
    tmpValN*=10;
    tmpN=*p++;
    tmpValN+=(tmpN&0x0f);
  }
  return tmpValN;
}

int i;

void dispTime(unsigned char pos)
{
  unsigned char chi;
  RTC.read(tm);
  chi=tm.Hour;
  lcd_putbch(chi, pos);
  lcd_putch(':');
  chi=tm.Minute;
  lcd_putbch(chi, pos+3);
#ifdef SEC
  lcd_putch(':');
  chi=tm.Second;
  lcd_putbch(chi, pos+6);
#endif
}

void getTime(void)
{
  unsigned char chi;
  RTC.read(tm);
  chi=tm.Hour;//rtc_read(RTC_ADDR_HR);
  buf[0]=((chi/10)+0x30);
  buf[1]=((chi%10)+0x30);
  buf[2]=':';
  chi=tm.Minute;//rtc_read(RTC_ADDR_MIN);
  buf[3]=((chi/10)+0x30);
  buf[4]=((chi%10)+0x30);
}

void dispDate(unsigned char pos)
{
  unsigned char chi;
  //lcd_goto(pos);
  RTC.read(tm);
  chi=tm.Day;//rtc_read(RTC_ADDR_DD);
  lcd_putbch(chi,pos);
  lcd_putch('/');
  chi=tm.Month;//rtc_read(RTC_ADDR_MM);
  lcd_putbch(chi,pos+3);
  lcd_putch('/');
  yr=tm.Year;
  yr=tmYearToCalendar(yr);
  chi=yr;//rtc_read(RTC_ADDR_YY);
  lcd_putbch(chi, pos+6);
}

void getDate(void)
{
  unsigned char chi;
  RTC.read(tm);
  chi=tm.Day;//rtc_read(RTC_ADDR_DD);
  buf[0]=((chi/10)+0x30);
  buf[1]=((chi%10)+0x30);
  buf[2]='/';
  chi=tm.Month;//rtc_read(RTC_ADDR_MM);
  buf[3]=((chi/10)+0x30);
  buf[4]=((chi%10)+0x30);
  buf[5]='/';
  chi=tm.Year;//rtc_read(RTC_ADDR_YY);
  chi=tmYearToCalendar(chi);
  buf[6]=((chi/10)+0x30);
  buf[7]=((chi%10)+0x30);
}

void dispUnit(unsigned char unt)
{
  if(unt==0)
  {
    lcd_putch('K');
    lcd_putch('G');
    lcd_putch('F');
  }
  else if(unt==1)
  {
    lcd_putch('N');
    lcd_putch(' ');
    lcd_putch(' ');
  }
  else if(unt==2)
  {
    lcd_putch('L');
    lcd_putch('B');
    lcd_putch('F');
  }
}

void dispArrow(unsigned char arrow)
{
  if(arrow==0)
  {
    lcd_goto(LINE1);
    lcd_putch('>');
    lcd_goto(LINE2);
    lcd_putch(' ');
    lcd_goto(LINE3);
    lcd_putch(' ');
    lcd_goto(LINE4);
    lcd_putch(' ');
  }
  else if(arrow==1)
  {
    lcd_goto(LINE1);
    lcd_putch(' ');
    lcd_goto(LINE2);
    lcd_putch('>');
    lcd_goto(LINE3);
    lcd_putch(' ');
    lcd_goto(LINE4);
    lcd_putch(' ');
  }
  else if(arrow==2)
  {
    lcd_goto(LINE1);
    lcd_putch(' ');
    lcd_goto(LINE2);
    lcd_putch(' ');
    lcd_goto(LINE3);
    lcd_putch('>');
    lcd_goto(LINE4);
    lcd_putch(' ');
  }
  else if(arrow==3)
  {
    lcd_goto(LINE1);
    lcd_putch(' ');
    lcd_goto(LINE2);
    lcd_putch(' ');
    lcd_goto(LINE3);
    lcd_putch(' ');
    lcd_goto(LINE4);
    lcd_putch('>');
  }
}

void scrMain()
{
  lcd_clear();
  //lcd_putstr("MENU",4,LINE1+3);
  lcd_putstr("1.SETUP",7,LINE1+1);
  lcd_putstr("2.NORMAL TEST",13,LINE2+1);
  lcd_putstr("3.DATA LOGGING",14,LINE3+1);
  lcd_putstr("4.Wi-Fi CONNECT.",16,LINE4+1);
  lineCnt=0;
  dispArrow(lineCnt);
}

void scrSetting(void)
{
  lcd_putstr("PULL MODE:",10,LINE1);
  lcd_goto(LINE1+10);
  lcd_putch(pullMode+0x30);
  lcd_putstr("MIN PULL:",9,LINE2);
  //convVal=convert(minPull);
  lcd_goto(LINE2+10);
  lcd_puti(minPull);
  lcd_putstr("UNIT:",5,LINE3);
  lcd_goto(LINE3+10);
  dispUnit(pullUnit);
  lcd_putstr("SPEED:",6,LINE4);
  lcd_goto(LINE4+7);
  lcd_puti(spdArr[pullSpeed]);
  lcd_putstr("MM/MIN",6,LINE4+14);
}

void scrNormal(void)
{
  lcd_putstr("PULL MODE:          ",20,LINE1);
  lcd_goto(LINE1+10);
  lcd_putch(pullMode+0x30);
  lcd_putstr("MIN PULL:           ",20,LINE2);
  //convVal=convert(minPull);
  lcd_goto(LINE2+10);
  lcd_puti(minPull);
  lcd_goto(LINE2+17);
  dispUnit(pullUnit);
  lcd_putstr("SPEED:              ",20,LINE3);
    lcd_goto(LINE3+7);
    lcd_puti(spdArr[pullSpeed]);
    lcd_putstr("MM/MIN",6,LINE3+14);
  lcd_putstr(" PRESS START BUTTON ",20,LINE4);
  setPt=0;
}

void scrExperiment(void)
{
  lcd_putstr("Select Wire:",12,LINE1);
  lcd_putstr("Wire:",5,LINE2);
  lcd_putstr("Terminal:",9,LINE3);
  lcd_putstr("Pull Out:",6,LINE4);
}

void scrExtExp(void)
{
  lcd_putstr("    SPEED:",10,LINE1);
  lcd_putstr("Term. No :",10,LINE2);
  lcd_putstr("Min Pull :",10,LINE4);
  lcd_putstr("Wire Spec:",10,LINE3);
}

void memScr(void)
{
  lcd_putstr("EXP No.",7,LINE1);
  lcd_putstr("SPEED:",6,LINE3);
  lcd_putstr("PEAK LOAD:",10,LINE4);
}

/********************ADC Calibration******************************/
void updatePWM(unsigned char pwmc)
{
  uint16_t pw;
  if(pwmc==0)
  {
    cbi(TCCR5A, COM5C0);
  }
  else
  {
    sbi(TCCR5A, COM5C0);
  }
  //Serial.print("Speed update: ");
  //Serial.println(pwmc);
  if((pwmc>0)&&(pwmc<75))
  {
    pw = 1450;
  }
  else if((pwmc>=75)&&(pwmc<125))
  {
    pw = 1000;
  }
  else if((pwmc>=125)&&(pwmc<175))
  {
    pw = 550;
  }
  else if((pwmc>=175)&&(pwmc<225))
  {
    pw = 410;
  }
  //Serial.println(pw);
  OCR5A = pw;//(65535-pw);
}

void motorRunRvs(void)
{
        digitalWrite(MOTOR_D1, LOW);
        digitalWrite(MOTOR_D2, LOW);
}

void motorRunFwd(void)
{
        digitalWrite(MOTOR_D1, HIGH);
        digitalWrite(MOTOR_D2, HIGH);
}

void motorStop(void)
{
  updatePWM(0);   
        digitalWrite(MOTOR_D1, LOW);
        digitalWrite(MOTOR_D2, LOW);
}

void retFwd(void)
{
  updatePWM(80);
  motorRunFwd();
  delay(6000);
  motorStop();
  updatePWM(0);
}

void home(void)
{
  //unsigned int cc;
  buzPitchOn=20;
  buzPitchOff=20;
#ifdef SER_DEBUG    
    Serial.println("Preparing Home");
#endif    
    buzFlag=1;
    lcd_clear();
    //updatePWM(180);
    updatePWM(200);
    lcd_putstr("Returning Home",14,LINE2+3);
    motorRunRvs();
#ifdef SER_DEBUG    
    Serial.println("Returning Home");
#endif    
    while((digitalRead(LIMIT_SW))&&(digitalRead(LIMIT_FLT)));
#ifdef SER_DEBUG    
    Serial.println("Reched Home");
#endif    
    motorStop();
    updatePWM(0);
    if(!digitalRead(LIMIT_FLT))
    {
      digitalWrite(LED_H, HIGH);
      retFwd();
    }
    else
    {
    ///lcd_clear();
      buzFlag=0;
      buzPitchOn=10;
      buzPitchOff=20;
    }
}

void clkSet()
{
  unsigned int cc;
  RTC.read(tm);
  lcd_clear();
  lcd_putstr("Clock setting",13,LINE4+3);
  lcd_putstr("SET Hour",8,LINE1+6);
  done=0;
  hr=tm.Hour;//rtc_read_reg(RTC_ADDR_HR);//sTime2.Hours;
  //hr=bcd_to_hex(hour);
  while(!done)
  {
    lcd_putbch(hr, LINE2+9);
      cc=getKey();
      HAL_Delay(500);
      if(cc==FUP)
      {
        if(hr<23)
          hr++;
        else
          hr=0;
      }
      else if(cc==FDOWN)
      {
        if(hr>0)
          hr--;
        else
          hr=23;
      }
      else if(cc==FSAVE)
      {
        done=1;
        tm.Hour=hr;
        //hr=hex_to_bcd(hr);
        //rtc_write_reg(RTC_ADDR_HR, hr);
        //HAL_RTC_SetDate(&hrtc, &sTime2, RTC_FORMAT_BIN);
      }
    }
    lcd_clear();
    lcd_putstr("SET minutes",11,LINE1+4);
    done=0;
    mn=tm.Minute;//rtc_read_reg(RTC_ADDR_MIN);
    //mn=bcd_to_hex(mn);
    while(!done)
    {
      lcd_putbch(mn, LINE2+9);
      cc=getKey();
      HAL_Delay(500);
      if(cc==FUP)
      {
        if(mn<59)
          mn++;
        else
          mn=0;
      }
      else if(cc==FDOWN)
      {
        if(mn>0)
          mn--;
        else
          mn=59;
      }
      else if(cc==FSAVE)
      {
        done=1;
        tm.Minute=mn;
        //mn=hex_to_bcd(mn);
        //rtc_write_reg(RTC_ADDR_MIN, mn);
        //HAL_RTC_SetDate(&hrtc, &sTime2, RTC_FORMAT_BIN);
      }
    }
    lcd_clear();
#ifdef SEC
    lcd_putstr("SET Seconds",11,LINE1+4);
    done=0;
    sc=tm.Second;//rtc_read_reg(RTC_ADDR_SEC);//sTime2.Seconds;
    //sc=bcd_to_hex(sc);
    while(!done)
    {
      lcd_putbch(sc, LINE2+9);
      cc=getKey();
      HAL_Delay(500);
      if(cc==FUP)
      {
        if(sc<59)
          sc++;
        else
          sc=0;
      }
      else if(cc==FDOWN)
      {
        if(sc>0)
          sc--;
        else
          sc=59;
      }
      else if(cc==FSAVE)
      {
        done=1;
        tm.Second=sc
      //  sc=hex_to_bcd(sc);
        //rtc_write_reg(RTC_ADDR_SEC, sc);
        lcd_clear();
      }
    }
    lcd_clear();
#endif
    lcd_putstr("SET Date",8,LINE1+6);
    done=0;
    dte=tm.Day;//rtc_read_reg(RTC_ADDR_DD);//DateToUpdate.Date;
    //dte=bcd_to_hex(dte);
    while(!done)
    {
      lcd_putbch(dte, LINE2+9);
      cc=getKey();
      HAL_Delay(500);
      if(cc==FUP)
      {
        if(dte<31)
          dte++;
        else
          dte=0;
      }
      else if(cc==FDOWN)
      {
        if(dte>0)
          dte--;
        else
          dte=31;
      }
      else if(cc==FSAVE)
      {
        done=1;
        tm.Day=dte;
    //    dte=hex_to_bcd(dte);
        //rtc_write_reg(RTC_ADDR_DD, dte);
      }
    }
    lcd_clear();
    lcd_putstr("SET Month",9,LINE1+5);
    done=0;
    mnt=tm.Month;//rtc_read_reg(RTC_ADDR_MM);//DateToUpdate.Month;
    //mnt=bcd_to_hex(mnt);
    while(!done)
    {
      lcd_putbch(mnt, LINE2+9);
      cc=getKey();
      HAL_Delay(500);
      if(cc==FUP)
      {
        if(mnt<12)
          mnt++;
        else
          mnt=1;
      }
      else if(cc==FDOWN)
      {
        if(mnt>1)
          mnt--;
        else
          mnt=12;
      }
      else if(cc==FSAVE)
      {
        done=1;
        tm.Month=mnt;
  //      mnt=hex_to_bcd(mnt);
        //rtc_write_reg(RTC_ADDR_MM, mnt);
      }
    }
    lcd_clear();
    lcd_putstr("SET Year",8,LINE1+6);
    done=0;
    yar=tm.Year;//rtc_read_reg(RTC_ADDR_YY);//DateToUpdate.Year;
    Serial.println("Year: "+String(yar));
    //yar=bcd_to_hex(yar);
    //yar+=1970;
    //yar%=100;
    yar=tmYearToCalendar(yar);
    Serial.println("Year1: "+String(yar));
    yar%=100;
    while(!done)
    {
      lcd_putbch(yar, LINE2+9);
      cc=getKey();
      HAL_Delay(500);
      if(cc==FUP)
      {
        if(yar<31)
          yar++;
        else
          yar=0;
      }
      else if(cc==FDOWN)
      {
        if(yar>0)
          yar--;
        else
          yar=31;
      }
      else if(cc==FSAVE)
      {
        done=1;
        Serial.print("Year2: "+String(yar));
        yar=CalendarYrToTm(yar);
        Serial.print("Year3: "+String(yar));
        tm.Year=yar;//+30;
//        yar=hex_to_bcd(yar);
        //rtc_write_reg(RTC_ADDR_YY, yar);
        RTC.write(tm);
      }
    }
    lcd_clear();
}

long adcReadAvg(unsigned char avg)
{
  long res;
  unsigned char u;
  res=0;
  for(u=0;u<avg;u++)
  {
    res+=scaleRead();
    delay(10);
  }
  res/=avg;
  //res*=10;
  return res;
}

void showWeight(unsigned long weg, unsigned char p)
{
  unsigned long tempValue;
  lcd_goto(p);
  lcd_putch((weg/1000000)+0x30);
  tempValue=weg%1000000;
  lcd_putch((tempValue/100000)+0x30);
  tempValue=tempValue%100000;
  lcd_putch((tempValue/10000)+0x30);
  tempValue=tempValue%10000;
  lcd_putch((tempValue/1000)+0x30);
  tempValue=tempValue%1000;
  lcd_putch((tempValue/100)+0x30);
  tempValue=tempValue%100;
  lcd_putch((tempValue/10)+0x30);
  lcd_putch((tempValue%10)+0x30);
}

unsigned char sensorRead()
{
  unsigned char ret1;
  ret1=digitalRead(LIMIT_SW);
  return ret1;
}

void tareWeight()
{
  //initCnt=adcReadAvg(5);
  myScale.calculateZeroOffset();
}

void setPre()
{
  unsigned long ress;
  int ch;
  lcd_clear();
  lcd_putstr("Pre-Load setting",16,LINE1+2);
  HAL_Delay(3000);
  lcd_putstr("NO Load Counts:",15,LINE2+2);
  while(1)
  {
    ress=adcReadAvg(5);
#ifdef SER_DEBUG    
    Serial.println(ress);
#endif    
    showWeight(ress,LINE4+5);
    HAL_Delay(300);
    ch=scanKey();
    if((ch==FEXIT)||(ch==FSAVE))
    {
      break;
    }
  }
}

unsigned long actualWeight()
{
  long tempValue1;
  tempValue1=myScale.getWeight(true, 10);//adcReadAvg(3);
  if(tempValue1<0)
  {
    sign=1;
    tempValue1*=-1;
  }
  else
    sign=0;
 // tempValue*=10;
  /*if(tempValue>=initCnt)
  {
    sign=0;
    finalValue=tempValue-initCnt;
  }
  else
  {
    sign=1;
    finalValue=initCnt-tempValue;
  }
  tempValue=(finalValue/fd);*/
  if(pullUnit==1)
  {
    tempValue1*=98;
    tempValue1/=10;
  }
  else if(pullUnit==2)
  {
    tempValue1*=22;
    tempValue1/=10;
  }
  return tempValue1;
}

void adcCal()
{
  unsigned long ress, noLoad, loadCnt;
  unsigned int ch, count;
  unsigned char tch, passPoint, exitF=0;
  passPoint=0;
  lcd_clear();
#ifdef SER_DEBUG    
  Serial.println("In calibration");
#endif  
  lcd_putstr("Enter Password",14,LINE1+3);
  done=0;
  passBuf[0]=0;
  passBuf[1]=0;
  passBuf[2]=0;
  passBuf[3]=0;
  while(!done)
  {
    lcd_putstr("****",4,LINE2+8);
    HAL_Delay(150);
    if(passPoint==0)
      lcd_goto(LINE2+8);
    else if(passPoint==1)
      lcd_goto(LINE2+9);
    else if(passPoint==2)
      lcd_goto(LINE2+10);
    else if(passPoint==3)
      lcd_goto(LINE2+11);
    tch=passBuf[passPoint];
    tch&=0x0f;
    switch_cursor(1);
    ch=getKey();
    if(ch==FUP)
    {
      if(tch<9)
        tch++;
      else
        tch=0;
      lcd_putch(tch+0x30);
    }
    else if(ch==FDOWN)
    {
      if(tch>0)
        tch--;
      else
        tch=9;
      lcd_putch(tch+0x30);
    }
    else if(ch==FLINE1)
    {
      passPoint=0;
    }
    else if(ch==FLINE2)
    {
      passPoint=1;
    }
    else if(ch==FLINE3)
    {
      passPoint=2;
    }
    else if(ch==FLINE4)
    {
      passPoint=3;
    }
    else if(ch==FSAVE)
    {
      done=1;
    }
    passBuf[passPoint]=tch+0x30;
    switch_cursor(0);
    //getKeyh();
    HAL_Delay(150);
  }
  if(strncmp((const char*)passBuf,"2453",4))
  {
    lcd_putstr("Wrong Password",14,LINE4+3);
    HAL_Delay(3000);
    return;
  }
  //adcOffset();
  lcd_clear();
  lcd_putstr("ADC Calibration",15,LINE1+2);
  HAL_Delay(3000);
  lcd_putstr("NO Load Counts:",15,LINE2+2);
  while(1)
  {
    ress=myScale.getAverage(10);
#ifdef SER_DEBUG    
    Serial.println(ress);
#endif    
    showWeight(ress,LINE4+5);
    HAL_Delay(300);
    ch=scanKey();
    if(ch==FSAVE)
    {
      noLoad=ress;
      myScale.calculateZeroOffset(64);
      break;
    }
    else if(ch==FEXIT)
    {
      exitF=1;
      break;
    }
  }
  if(exitF)
  {
    lcd_putstr("Exiting 1",9, LINE4);
    HAL_Delay(3000);
#ifdef INTEE
    /*EEPROM.put(ADR_RAW, 0);
    EEPROM.put(ADR_FINAL, 0);
    EEPROM.put(ADR_VAL, 0);*/
    EEPROM.put(LOCATION_CALIBRATION_FACTOR, 0);
    EEPROM.put(LOCATION_ZERO_OFFSET, 0);
#else
    for(int o=0; o<8; o++)
      buf[o]=0;
    FM25V02_WRITE(ADDRAW, buf, 8);
    FM25V02_WRITE(ADDFINAL, buf, 8);
    FM25V02_WRITE(ADDVAL, buf, 8);
#endif
    return;
  }
  lcd_putstr("Load Counts:   ",15,LINE2+2);
  //updatePWM(10);
  exitF=0;
  HAL_Delay(1000);
  while(1)
  {
    ress=myScale.getAverage(10);//adcReadAvg(5);
#ifdef SER_DEBUG    
    Serial.println(ress);
#endif    
    showWeight(ress,LINE4+5);
    //HAL_Delay(50);
    ch=scanKey();
    if(ch==FSAVE)
    {
      updatePWM(0);
      loadCnt=ress;
      break;
    }
    else if(ch==FEXIT)
    {
      exitF=1;
      break;
    }
    else if(ch==FUP)
    {
      updatePWM(pwmBuf[2]);
      motorRunFwd();
      lcd_putstr("Forward",7,LINE3);
      /*HAL_Delay(300);
      motorStop();*/
    }
    else if(ch==FDOWN)
    {
      updatePWM(pwmBuf[2]);
      if(sensorRead())
        motorRunRvs();
      else
        motorStop();
      lcd_putstr("Reverse",7,LINE3);
      /*HAL_Delay(300);
      motorStop();*/
    }
    else
    {
      motorStop();
      updatePWM(0);
      lcd_putstr("       ",7,LINE3);
    }
  }
  if(exitF)
  {
    lcd_putstr("Exiting 2",9, LINE4);
    HAL_Delay(3000);
#ifdef INTEE
    /*EEPROM.put(ADR_RAW, 0);
    EEPROM.put(ADR_FINAL, 0);
    EEPROM.put(ADR_VAL, 0);*/
    EEPROM.put(LOCATION_CALIBRATION_FACTOR, 0);
    EEPROM.put(LOCATION_ZERO_OFFSET, 0);
#else
    for(int o=0; o<8; o++)
      buf[o]=0;
    FM25V02_WRITE(ADDRAW, buf, 8);
    FM25V02_WRITE(ADDFINAL, buf, 8);
    FM25V02_WRITE(ADDVAL, buf, 8);
#endif
    return;
  }
  exitF=0;
  lcd_clear();
  lcd_putstr("Enter Load Value",16,LINE1+2);
  done=0;
  count=0;
  HAL_Delay(1000);
  while(!done)
  {
    lcd_goto(LINE2+8);
    lcd_puti(count);
    ch=scanKey();
    if(ch==FSAVE)
    {
      count*=10;
      myScale.calculateCalibrationFactor(count, 64);
      Serial.print(F("New cal factor: "));
      Serial.println(myScale.getCalibrationFactor(), 2);
    
      Serial.print(F("New Scale Reading: "));
      Serial.println(myScale.getWeight(), 2);
#ifdef INTEE
  EEPROM.put(LOCATION_CALIBRATION_FACTOR, myScale.getCalibrationFactor());
  EEPROM.put(LOCATION_ZERO_OFFSET, myScale.getZeroOffset());
      /*EEPROM.put(ADR_RAW, noLoad);
      EEPROM.put(ADR_FINAL, loadCnt);
      calVal=count*10;
      EEPROM.put(ADR_VAL, calVal);*/
#else
      int2str(noLoad, buf, 8);
      FM25V02_WRITE(ADDRAW, buf, 8);
      int2str(loadCnt, buf, 8);
      FM25V02_WRITE(ADDFINAL, buf, 8);
      calVal=count*10;
      int2str(calVal, buf, 8);
      FM25V02_WRITE(ADDVAL, buf, 8);
#endif
      diff=loadCnt-noLoad;
      fd=diff/count;
      lcd_clear();
      lcd_goto(LINE1);
      lcd_puti(diff);
      lcd_goto(LINE1+8);
      lcd_puti(fd);
      getDate();
      lcd_putstr(buf,8,LINE2+6);
#ifdef INTEE
      for(int r=0;r<8;r++)
      {
          EEPROM.write(ADR_CALDATE+r, buf[r]);
          Serial.print(buf[r]);
      }
#else
      FM25V02_WRITE(ADDCALDATE, buf, 8);
#endif
      delay(500);
      serviceCount=0;
#ifdef INTEE
      EEPROM.put(ADR_SERVICE, serviceCount);
#else
      Serial.print(F("SERVICE COUNT :"));
      Serial.println(serviceCount);
      int2str(serviceCount, buf, 8);
  Serial.println();
  for(int t=0; t<8; t++)
  {
    Serial.write(buf[t]);
    Serial.print(' ');
  }
  Serial.println();
            FM25V02_WRITE(ADDSERVICE, buf, 8);
#endif
      HAL_Delay(3000);
      lcd_clear();
      done=1;
    }
    else if(ch==FEXIT)
    {
      done=1;
    }
    else if(ch==FUP)
    {
      HAL_Delay(500);
      if(count<200)
        count++;
      else
        count=0;
    }
    else if(ch==FDOWN)
    {
      HAL_Delay(500);
      if(count>0)
        count--;
      else
        count=200;
    }
  }
  home();
  ddg=1;
}

unsigned long convert(unsigned long vv)
{
  unsigned long retvv=0;
  if(pullUnit==1)
  {
    retvv=vv*98;
    retvv=vv/10;
  }
  else if(pullUnit==2)
  {
    retvv=vv*22;
    retvv=vv/10;
  }
  else
    retvv=vv;
  return retvv;
}

unsigned long convert2(unsigned long val,unsigned char su,unsigned char du)
{
  if(su==U_N)
  {
    if(du==U_Kgf)
      val=((unsigned long)val*10)/98;
    if(du==U_Lbf)
      val=((unsigned long)val*22)/98;
  }
  if(su==U_Lbf)
  {
    if(du==U_N)
      val=((unsigned long)val*98)/22;
    if(du==U_Kgf)
      val=((unsigned long)val*10)/22;
  }
  if(su==U_Kgf)
  {
    if(du==U_N)
      val=((unsigned long)val*98)/10;
    if(du==U_Lbf)
      val=((unsigned long)val*22)/10;
  }
  return val;
}

void weighTest(void)
{
  unsigned int ch;
  unsigned long ress, ttm;
  unsigned char spdFlag=0;
  lcd_clear();
  lcd_putstr("weight testing",14,LINE1+3);
  //adcOffset();
  HAL_Delay(1000);
  tareWeight();
  while(1)
  {
    tempValue=actualWeight();
    //tempValue*=10;
    lcd_goto(LINE2+3);
    lcd_putid(tempValue);
#ifdef SER_DEBUG    
    Serial.println(tempValue);
    Serial.print('$');
    Serial1.print('$');
    Serial1.write((tempValue/10000)+0x30);
    Serial.write((tempValue/10000)+0x30);
    ttm=tempValue%10000;
    Serial1.write((ttm/1000)+0x30);
    Serial.write((ttm/1000)+0x30);
    ttm=ttm%1000;
    Serial1.write((ttm/100)+0x30);
    Serial.write((ttm/100)+0x30);
    ttm=ttm%100;
    Serial1.write((ttm/10)+0x30);
    Serial.write((ttm/10)+0x30);
    Serial1.write((ttm%10)+0x30);
    Serial.write((ttm%10)+0x30);
    Serial1.print('!');
    Serial.print('!');
#endif    
    //HAL_Delay(50);
    ch=scanKey();
    if(ch==FEXIT)
    {
      break;
    }
    else if(ch==FUP)
    {
      updatePWM(pwmBuf[spdFlag]);
      motorRunFwd();
      lcd_putstr("Forward",7,LINE3);
    }
    else if(ch==FDOWN)
    {
      updatePWM(pwmBuf[spdFlag]);
      if(sensorRead())
        motorRunRvs();
      else
        motorStop();
      lcd_putstr("Reverse",7,LINE3);
    }
    else if(ch==FLINE1)
    {
      spdFlag=0;
      lcd_putstr("SPPED 1",7,LINE4+5);
      //updatePWM(pwmBuf[0]);
    }
    else if(ch==FLINE2)
    {
      spdFlag=1;
      lcd_putstr("SPPED 2",7,LINE4+5);
      //updatePWM(pwmBuf[1]);
    }
    else if(ch==FLINE3)
    {
      spdFlag=2;
      lcd_putstr("SPPED 3",7,LINE4+5);
      //updatePWM(pwmBuf[2]);
    }
    else if(ch==FLINE4)
    {
      spdFlag=3;
      lcd_putstr("SPPED 4",7,LINE4+5);
      //updatePWM(pwmBuf[3]);
    }
    else
    {
      motorStop();
      lcd_putstr("       ",7,LINE3);
    }
  }
}

void animate(unsigned char ps)
{
  static unsigned char ani=0;
    lcd_goto(ps);
    if(ani==0)
    {
      ani++;
      lcd_putch('|');
    }
    /*else if(ani==1)
    {
      lcd_putch(0x5c);
        ani++;
    }*/
    else if(ani==1)
    {
      lcd_putch('-');
        ani=0;
    }
}

void setBuf(unsigned long abc)
{
  peakBuf[0]=((abc/100000)+0x30);
  abc=abc%100000;
  peakBuf[1]=((abc/10000)+0x30);
  abc=abc%10000;
  peakBuf[2]=((abc/1000)+0x30);
  abc=abc%1000;
  peakBuf[3]=((abc/100)+0x30);
  abc=abc%100;
  peakBuf[4]=((abc/10)+0x30);
  peakBuf[5]='.';
  peakBuf[6]=((abc%10)+0x30);
  peakBuf[7]=0x0a;
  peakBuf[8]=0x0d;
}

void getWeight(unsigned long abc)
{
  buf[0]=((abc/100000)+0x30);
  abc=abc%100000;
  buf[1]=((abc/10000)+0x30);
  abc=abc%10000;
  buf[2]=((abc/1000)+0x30);
  abc=abc%1000;
  buf[3]=((abc/100)+0x30);
  abc=abc%100;
  buf[4]=((abc/10)+0x30);
  buf[5]='.';
  buf[6]=((abc%10)+0x30);
  buf[7]='\0';
}

void getWeight2(unsigned long abc)
{
  buf[0]='[';
  buf[1]=((abc/1000)+0x30);
  abc=abc%1000;
  buf[2]=((abc/100)+0x30);
  abc=abc%100;
  buf[3]=((abc/10)+0x30);
  buf[4]=((abc%10)+0x30);
  buf[5]='.';
  buf[6]=']';
}

unsigned char extBit;
unsigned int chx, ch, chF, diffCount, memTemp, error, pwmInit, updateCnt;
unsigned long ress, indxW;
unsigned char seq, err, expFlag, homeFlag, st, keySys;
unsigned char genBuf[100], arr[]="NAMAN Bhatnagar\n\r";
unsigned int byteswritten, line, col, indx, t;
unsigned int id, charCnt, dlyCount, dly, keyCnt;
unsigned char unit[5], idBuf[5],terminal[15], minpull[10], information[20];
//void lcd_putstr(char *str,unsigned char noc,char pos);

void getPullSpeed(void)
{
  int p;
  p=spdArr[pullSpeed];
  buf[0]=((p/100)+0x30);
  p=p%100;
  buf[1]=((p/10)+0x30);
  buf[2]=((p%10)+0x30);
}

void getUnit(unsigned char unt)
{
  if(unt==0)
  {
    buf[0]='K';
    buf[1]='G';
    buf[2]='F';
  }
  else if(unt==1)
  {
    buf[0]='N';
    buf[1]=' ';
    buf[2]=' ';
  }
  else if(unt==2)
  {
    buf[0]='L';
    buf[1]='B';
    buf[2]='F';
  }
}

void noLoadCal(void)
{
  unsigned char vvl;
  lcd_clear();
  for(int y=0; y<4; y++)
  {
    lcd_putstr("No Load Calibration", 19, LINE1);
    lcd_putstr("In speed: ", 10, LINE2+4);
    lcd_putbch(y, LINE2+14);
    lcd_putstr("Press START", 11, LINE3+4);
    while(1)
    {     
      ch=scanKey();
      if(ch==FSAVE)
      {
        motorStop();
        updatePWM(0);
        diffTime=millis();
        diffTime=diffTime-startTime;
        vvl=diffTime/1000;
        noLoadArr[y]=vvl;
        lcd_goto(LINE4+15);
        lcd_puti(vvl);
        EEPROM.write(ADR_NOLOAD+y, vvl);
        delay(5000);
        home();
        //lcd_putstr("          ", 10, LINE4+8);
        break;
      }
      else if(ch==FSTART)
      {
        lcd_putstr("Press SAVE ", 11, LINE3+4);
        updatePWM(pwmBuf[y]);
        startTime=millis();
        motorRunFwd();
      }
      delay(500);
    }
    
  }
}

void setup() {
  float settingCalibrationFactor; //Value used to convert the load cell reading to lbs or kg
  long settingZeroOffset; //Zero value that is found when scale is tared
  // put your setup code here, to run once7
  pinMode(SCALE_PDWN, OUTPUT);
  pinMode(SCALE_TEMP, OUTPUT);
  pinMode(SCALE_CLK, OUTPUT);
  pinMode(SCALE_DRDY, INPUT_PULLUP);
  digitalWrite(SCALE_PDWN, LOW);
  digitalWrite(SCALE_TEMP,LOW);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_EN, OUTPUT);
  pinMode(LCD_D0, OUTPUT);
  pinMode(LCD_D1, OUTPUT);
  pinMode(LCD_D2, OUTPUT);
  pinMode(LCD_D3, OUTPUT);
  pinMode(LCD_D4, OUTPUT);
  pinMode(LCD_D5, OUTPUT);
  pinMode(LCD_D6, OUTPUT);
  pinMode(LCD_D7, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_D1, OUTPUT);
  pinMode(MOTOR_D2, OUTPUT);
  pinMode(NVRAM_CS, OUTPUT);
  pinMode(LED_H, OUTPUT);
  pinMode(LED_O, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(BUZZ, OUTPUT);
  pinMode(LIMIT_FLT, INPUT_PULLUP);
  pinMode(LIMIT_SW, INPUT_PULLUP);
  for(int u=57;u<=68;u++)
    pinMode(u,INPUT_PULLUP);
  //FM25V02_Init();
  digitalWrite(MOTOR_D1, HIGH);
  digitalWrite(MOTOR_D2, HIGH);
  digitalWrite(LED_H, LOW);
  digitalWrite(LED_O, LOW);
  TCCR5A = 0x04;
  TCCR5B = 0x0A;//TCCR5B & B11111000 | B00001001;
  TCCR5C = 0;
  //OCR5A = 200; 
  TCNT5 = 0;
  updatePWM(0);
  Serial.begin(115200);
  Serial1.begin(115200);
  SPI2.begin();
  
  Wire.begin();
  if (myScale.begin() == false)
  {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Scale detected!");
  
  delay(100);
  digitalWrite(SCALE_PDWN, HIGH);
#ifdef INTEE
  Serial.println(F("internal eeprom used"));
#else
  Serial.println(F("External eeprom use"));
#endif
  /*digitalWrite(BUZZ, HIGH);
  delay(2000);
  digitalWrite(BUZZ, LOW);*/
  /*for(int h=0;h<100;h++)
  {
    long scal;
    scal = scaleRead(0);
    Serial.println(scal);
    delay(50);
  }*/
  lcd_init();
  lcd_clear();
  SPI.begin();
#ifdef INTEE
  EEPROM.get(ADR_SERVICE, serviceCount);
#else
  FM25V02_READ(ADDSERVICE, buf, 8);
  Serial.println();
  for(int t=0; t<8; t++)
  {
    Serial.write(buf[t]);
    Serial.print(' ');
  }
  Serial.println();
  serviceCount= atolN((char *)buf);
  Serial.print(F("Service Count:"));
  Serial.println(serviceCount);
#endif
#ifdef SER_DEBUG
  Serial.print("Initializing SD card...");
#endif  
  if (!SD.begin(chipSelect)) {
    st=0x01;
    Serial.println("Error opening");
  }
  else
  {
    //Serial.println("initialized OK");
    st=0xfe;
  }
#ifdef SER_DEBUG
  Serial.println("card initialized.");
#endif
  uint8_t vvl;
  ch=scanKey();
  if(ch==FSTOP)
  {
    ddg=1;
  }
  else if(ch==FLINE1)
  {
    adcCal();
  }
  else if(ch==FEXIT)
  {
   clkSet();
  }
  else if(ch==FUP)
  {
   setPre();
  }
  else if(ch==FLINE2)
  {
    lcd_putstr("SERVICE CYCLE:",14,LINE1+3);
    lcd_goto(LINE2+4);
    lcd_puti(serviceCount);
    lcd_putch('/');
    lcd_puti(SERVICE);
#ifdef INTEE
    for(int r=0;r<8;r++)
    {
        buf[r]=EEPROM.read(ADR_CALDATE+r);
    }
#else
    FM25V02_READ(ADDCALDATE, buf, 8);
#endif
    lcd_putstr("CALIBRATION DATE:",17,LINE3+1);
    lcd_putstr(buf,8,LINE4+6);
    delay(5000);
    lcd_clear();
  }
  else if(ch==FLINE3)
  {
    noLoadCal();
  }  
 // if(ddg)
  {
    for(int y=0; y<4; y++)
    {
      vvl=EEPROM.read(ADR_NOLOAD+y);
      noLoadArr[y]=vvl;
      if(ddg)
        lcd_putbch(vvl, (LINE1+(4*y)));
    }
    if(ddg)
      delay(5000);
  }
#ifdef INTEE
  for(int r=0;r<8;r++)
  {
     buf[r]=EEPROM.read(ADR_CALDATE+r);
  }
#else
    FM25V02_READ(ADDCALDATE, buf, 8);
#endif
  calDay=(((buf[0]&0x0f)*10)+(buf[1]&0x0f));
  calMonth=(((buf[3]&0x0f)*10)+(buf[4]&0x0f));
  calYear=(((buf[6]&0x0f)*10)+(buf[7]&0x0f));
  Serial.print(calDay);
  Serial.print('\t');
  Serial.print(calMonth);
  Serial.print('\t');
  Serial.println(calYear);
  RTC.read(tm);
  Serial.print(tm.Day);
  Serial.print('\t');
  Serial.print(tm.Month);
  Serial.print('\t');
  yr=tm.Year;
  yr=tmYearToCalendar(yr);
  Serial.println(yr);
  tgtYear=calYear+1;
  tgtMonthAl=calMonth-1;
  tgtMonth=calMonth;
  tgtDay=calDay;
  serviceFlag=0;
  serviceAlFlag=0;
  if(yr>=tgtYear)
  {
    if(tm.Month==tgtMonth)
    {
      if(tm.Day>=tgtDay)
        serviceFlag=1;
    }
    else if(tm.Month>tgtMonth)
    {
      serviceFlag=1;
    }
    if(tm.Month==tgtMonthAl)
    {
      if(tm.Day>=tgtDay)
      {
        serviceAlFlag=1;
        leftServiceDay=tm.Day-tgtDay;
        leftServiceDay=30-leftServiceDay;
      }
    }
    else if((tm.Month>tgtMonthAl)&&(tm.Month<=tgtMonth))
    {
      if(tm.Day<=tgtDay)
      {
        serviceAlFlag=1;
        leftServiceDay=tgtDay-tm.Day;
      }
    }
  }
  if(serviceAlFlag)
  {
#ifdef SER_DEBUG    
    Serial.println("left days to service: ");
    Serial.println(leftServiceDay);
#endif    
    lcd_clear();
    lcd_putstr("Calibration Due",15,LINE2+2);
    lcd_putstr("Days Left : ",12,LINE3+1);
    lcd_puti(leftServiceDay);
    delay(5000);
  }
  if(serviceFlag)
  {
#ifdef SER_DEBUG    
    Serial.println("Need Service");    
#endif    
    lcd_clear();
    lcd_putstr("Calibration Due D",17,LINE2+2);
//    lcd_putstr("    By Date    ",15,LINE3+2);
    //while(1);
    delay(10000);
  }
  lcd_clear();
  lcd_putstr("***REGENATO***",14,LINE1+3);
  lcd_putstr("MOTORIZED PULLTESTER",20,LINE2);
#ifdef PULL500
#ifdef  WIFIEN
  lcd_putstr("MODEL-SP04W VER:1.21",20,LINE3);
#else  
  lcd_putstr("MODEL-SP04N VER:1.21",20,LINE3);
#endif  
  lcd_putstr("CAPACITY-500  KGF",17,LINE4+1);
#endif
#ifdef PULL200
#ifdef  WIFIEN
  lcd_putstr("MODEL-SP03W VER:1.21",20,LINE3);
#else  
  lcd_putstr("MODEL-SP03N VER:1.21",20,LINE3);
#endif  
  lcd_putstr("CAPACITY-200  KGF",17,LINE4+1);
#endif
#ifdef PULL100
#ifdef  WIFIEN
  lcd_putstr("MODEL-SP02W VER:1.21",20,LINE3);
#else  
  lcd_putstr("MODEL-SP02N VER:1.21",20,LINE3);
#endif  
  lcd_putstr("CAPACITY-100  KGF",17,LINE4+1);
#endif
#ifdef PULL50
#ifdef  WIFIEN
  lcd_putstr("MODEL-SP01W VER:1.21",20,LINE3);
#else  
  lcd_putstr("MODEL-SP01N VER:1.21",20,LINE3);
#endif  
  //lcd_putstr("MODEL-SP01W VER:1.20",20,LINE3);
  lcd_putstr("CAPACITY-50  KGF",16,LINE4+2);
#endif
  buzInit();
  Timer3.initialize(10000);  
  Timer3.attachInterrupt(callback); 
  buzFlag=1;
  HAL_Delay(3000);
  if(serviceCount>=SERVICE)//)//25000)
  {
    lcd_clear();
    lcd_putstr("Calibration Due C",17,LINE2+2);
//    lcd_putstr("   By Counts   ",15,LINE3+2);
    //while(1);
    delay(10000);
  }
  else if(serviceCount>=SERVICELMT)//24900)
  {
    lcd_clear();
    int serviceDiff=SERVICE-serviceCount;
    lcd_putstr("Calibration due",15,LINE2+2);
    lcd_putstr("after: ",7,LINE3+1);
    lcd_puti(serviceDiff);
    lcd_putstr(" Cycle",6,LINE3+13);
    delay(3000);
  }
  buzFlag=0;
  //HAL_Delay(3000);
  FM25V02_Init();
  FM25V02_READ(ADDPULLMODE,buf,1);
  pullMode=buf[0];
  if((pullMode<1)||(pullMode>4))
  {
   pullMode=1;
    buf[0]=pullMode;
    FM25V02_WRITE(ADDPULLMODE,buf,1);
  }
  HAL_Delay(500);
  FM25V02_READ(ADDPULLUNIT,buf,1);
  pullUnit=buf[0];
  HAL_Delay(500);
  FM25V02_READ(ADDPULLVAL,buf,2);
  minPull=((buf[0]<<8)+buf[1]);
  //if(minPull)
  HAL_Delay(500);
  FM25V02_READ(ADDPULLSPD,buf,1);
  pullSpeed=buf[0];
  FM25V02_READ(ADDFD,buf,2);
  fd=((buf[0]<<8)+buf[1]);
  FM25V02_READ(ADDOFF,buf,1);
  offSet=buf[0];
  FM25V02_READ(EXP_NUMBER,buf,1);
  expNumber=buf[0];
  if(expNumber>250)
  {
    expNumber=0;
    buf[0]=expNumber;
    FM25V02_WRITE(EXP_NUMBER,buf,1);
  }
  FM25V02_READ(ADDSDID,buf,2);
  id=((buf[0]<<8)+buf[1]);
  //pullUnit=0;
  lcd_clear();
  lcd_putstr("Please Wait...",14,LINE1+3);
  lcd_putstr("System is getting",17,LINE2+1);
  lcd_putstr("Ready...",8,LINE3+6);
 #ifdef INTEE
  EEPROM.get(LOCATION_CALIBRATION_FACTOR, settingCalibrationFactor);
  if (settingCalibrationFactor == 0xFFFFFFFF)
  {
    settingCalibrationFactor = 0; //Default to 0
    EEPROM.put(LOCATION_CALIBRATION_FACTOR, settingCalibrationFactor);
  }

  //Look up the zero tare point
  EEPROM.get(LOCATION_ZERO_OFFSET, settingZeroOffset);
  if (settingZeroOffset == 0xFFFFFFFF)
  {
    settingZeroOffset = 1000L; //Default to 1000 so we don't get inf
    EEPROM.put(LOCATION_ZERO_OFFSET, settingZeroOffset);
  }

  //Pass these values to the library
  myScale.setCalibrationFactor(settingCalibrationFactor);
  myScale.setZeroOffset(settingZeroOffset);
  Serial.println("adc cal:");
  Serial.println(settingCalibrationFactor);
  Serial.println(settingZeroOffset);
  myScale.setSampleRate(NAU7802_SPS_320); //Increase to max sample rate
  myScale.calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel 

 /* EEPROM.get(ADR_RAW, rawCount);
  EEPROM.get(ADR_FINAL, finalCount);
  EEPROM.get(ADR_VAL, calVal);*/
#else
  FM25V02_READ(ADDRAW, buf, 8);
  rawCount= atolL((char *)buf); //(((buf[2]&0x0f)*100000)+((buf[3]&0x0f)*10000)+((buf[4]&0x0f)*1000)+((buf[5]&0x0f)*100)+((buf[6]&0x0f)*10)+(buf[7]&0x0f));//atolN((char *)buf);
  Serial.print(F("Raw Count:"));
  Serial.println(rawCount);
  FM25V02_READ(ADDFINAL, buf, 8);
  finalCount=atolL((char *)buf); //(((buf[2]&0x0f)*100000)+((buf[3]&0x0f)*10000)+((buf[4]&0x0f)*1000)+((buf[5]&0x0f)*100)+((buf[6]&0x0f)*10)+(buf[7]&0x0f));
  Serial.print(F("Final Count:"));
  Serial.println(finalCount);
  FM25V02_READ(ADDVAL, buf, 8);
  calVal= atolN((char *)buf); //(((buf[2]&0x0f)*100000)+((buf[3]&0x0f)*10000)+((buf[4]&0x0f)*1000)+((buf[5]&0x0f)*100)+((buf[6]&0x0f)*10)+(buf[7]&0x0f));//atolN((char *)buf);
  Serial.print(F("cal Count:"));
  Serial.println(calVal);
#endif
  diff=finalCount-rawCount;
  fd=diff/calVal;
#ifdef SER_DEBUG
  Serial.print(rawCount);
  Serial.print('\t'); 
  Serial.print(finalCount);
  Serial.print('\t');
  Serial.print(calVal);
  Serial.print('\t');
  Serial.println(fd);
#endif  
  /*while(1)
  {
    int k=getKey();
    lcd_goto(LINE1);
    lcd_puti(k);
  }*/
  delay(1000);
  lcd_clear();
  scrNormal();
  seq=2;
}

unsigned long adcCounter=0, adcValue;
int itr=0;//777
unsigned char b2[10];

void loop() {
  while(1)
  {
    if(seq==0)
    {
      dispTime(LINE2+15);
      dispDate(LINE1+12);
      ch=scanKey();
      if(ch==FUP)
      {
        HAL_Delay(500);
        if(lineCnt>0)
          lineCnt--;
        dispArrow(lineCnt);
      }
      else if(ch==FDOWN)
      {
        HAL_Delay(500);
        if(lineCnt<3)
          lineCnt++;
        dispArrow(lineCnt);
      }
      else if(ch==FLINE1)
      {
        HAL_Delay(500);
        lineCnt=0;
        dispArrow(lineCnt);
      }
      else if(ch==FHOME)
      {
        home();
        buzFlag=0;
        HAL_Delay(100);
        lcd_clear();
        scrMain();
      }
      /*else if(ch==FEXIT)
      {
        HAL_Delay(500);
        lcd_clear();
        Serial1.print("$I!");
        Serial.print("$I!");
        lcd_putstr("IP Address",10, LINE1+5);
        seq=8;
        //dispArrow(lineCnt);
      }*/
      else if(ch==FLINE2)
      {
        HAL_Delay(500);
        lineCnt=1;
        dispArrow(lineCnt);
      }
      else if(ch==FLINE3)
      {
        HAL_Delay(500);
        lineCnt=2;
        dispArrow(lineCnt);
      }
      else if(ch==FLINE4)
      {
        HAL_Delay(500);
        lineCnt=9;
        //dispArrow(lineCnt);
      }
      else if(ch==FSAVE)
      {
        HAL_Delay(500);
        seq=lineCnt+1;
        lcd_clear();
        if(lineCnt==0)
        {
          lineCnt=0;
          scrSetting();
          if(pullUnit==0)
            maxPull=MAX_PULL_K;
          else if(pullUnit==1)
            maxPull=MAX_PULL_N;
          else if(pullUnit==2)
            maxPull=MAX_PULL_L;
          minPull=convert2(minPull,prvUnit,pullUnit);
          if(minPull>maxPull)
            minPull=maxPull;
          lcd_goto(LINE1+10);
          switch_cursor(1);
        }
        else if(lineCnt==1)
          scrNormal();
        else if(lineCnt==2)
        {
          lcd_clear();
        }
      }
      else if(ch==FSTART)
      {
        if(ddg==1)
        {
          weighTest();
          lcd_clear();
          HAL_Delay(1000);
          scrMain();
        }
      }
      else if(ch==FMEM)
      {
        HAL_Delay(500);
        done=0;
        lcd_clear();
        memScr();
        FM25V02_READ(EXP_NUMBER, buf,1);
        expNumber=buf[0];
        while(!done)
        {
          lcd_putbch3(expNumber,LINE1+10);
          temp=(expNumber*PAGE_SIZE)+PAGE_START;
          memTemp=temp+DATE_OFF;
          FM25V02_READ(memTemp,buf,8);
          //lcd_goto(LINE2);
          lcd_putstr(buf,8,LINE2);
          memTemp=temp+ TIME_OFF;
          FM25V02_READ(memTemp,buf,5);
          lcd_putstr(buf,5,LINE2+12);
          memTemp=temp+SPEED_OFF;
          FM25V02_READ(memTemp,buf,1);
          lcd_goto(LINE3+9);
          lcd_puti(spdArr[buf[0]]);
          lcd_putstr("mm/min",6,LINE3+14);
          memTemp=temp+UNIT_OFF;
          FM25V02_READ(memTemp,buf,1);
          lcd_goto(LINE4+17);
          dispUnit(buf[0]);
          memTemp=temp+PEEKLOAD_OFF;
          FM25V02_READ(memTemp,buf,7);
          lcd_putstr(buf,7,LINE4+10);
          chx=getKeyh();
          if(chx==FEXIT)
          {
            lcd_clear();
            done=1;
            FM25V02_READ(EXP_NUMBER, buf,1);
            expNumber=buf[0];
            HAL_Delay(1000);
          }
          else if(chx==FUP)
          {
            HAL_Delay(500);
            if(expNumber<250)
              expNumber++;
            else
              expNumber=0;
          }
          else if(chx==FDOWN)
          {
            HAL_Delay(500);
            if(expNumber>0)
              expNumber--;
            else
              expNumber=250;
          }
        }
        lcd_clear();
        scrMain();
      }      
      if(pcFlag==1)
      {
        Serial.write('#');
        seq=5;
        lcd_clear();
        lcd_putstr("Connected to PC",15,LINE2+3);
        cmd=0;
        expFlag=0;
        homeFlag=0;
      }
    }
   else if(seq==1)
    {
       //lcd_putstr("in sequence2",12,LINE2);
      ch=scanKey();
      if(ch==FEXIT)
      {
        HAL_Delay(500);
        seq=0;
        switch_cursor(0);
        lcd_clear();
        scrMain();
      }
      else if(ch==FSAVE)
      {
        HAL_Delay(300);
        buf[0]=pullMode;
        FM25V02_WRITE(ADDPULLMODE,buf,1);
        buf[0]=(unsigned char)((minPull&0xff00)>>8);
        buf[1]=(unsigned char)(minPull&0x00ff);
        FM25V02_WRITE(ADDPULLVAL,buf,2);
        buf[0]=pullUnit;
        FM25V02_WRITE(ADDPULLUNIT,buf,1);
        buf[0]=pullSpeed;
        FM25V02_WRITE(ADDPULLSPD,buf,2);
        seq=0;
        switch_cursor(0);
        lcd_clear();
        scrMain();
      }
      else if(ch==FLINE1)
      {
        HAL_Delay(300);
        lineCnt=0;
        lcd_goto(LINE1+10);
      }
      else if(ch==FLINE2)
      {
        HAL_Delay(500);
        lineCnt=1;
        lcd_goto(LINE2+10);
      }
      else if(ch==FLINE3)
      {
        HAL_Delay(500);
        lineCnt=2;
        lcd_goto(LINE3+10);
      }
      else if(ch==FLINE4)
      {
        HAL_Delay(500);
        lineCnt=3;
        lcd_goto(LINE4+7);
      }
      else if(ch==FUP)
      {
        if(lineCnt==0)    //mode selection
        {
          if(pullMode<4)
            pullMode++;
          else
            pullMode=1;
          lcd_goto(LINE1+10);
          lcd_putch(pullMode+0x30);
          dly=300;
        }
        else if(lineCnt==1)
        {
          if(minPull<maxPull)
            minPull++;
          else
            minPull=0;
          //convVal=convert(minPull);
          lcd_goto(LINE2+10);
          lcd_puti(minPull);
          if(keyCnt>0)
            keyCnt-=10;
          dly=keyCnt;
        }
        else if(lineCnt==2)
        {
          prvUnit=pullUnit;
          if(pullUnit<2)
            pullUnit++;
          else
            pullUnit=0;
          lcd_goto(LINE3+10);
          dispUnit(pullUnit);
          dly=300;
          if(pullUnit==0)
            maxPull=MAX_PULL_K;
          else if(pullUnit==1)
            maxPull=MAX_PULL_N;
          else if(pullUnit==2)
            maxPull=MAX_PULL_L;
          minPull=convert2(minPull,prvUnit,pullUnit);
          if(minPull>maxPull)
            minPull=maxPull;
          lcd_goto(LINE2+10);
          lcd_puti(minPull);
        }
        else if(lineCnt==3)
        {
          if(pullSpeed<3)
            pullSpeed++;
          else
            pullSpeed=0;

          lcd_goto(LINE4+7);
          lcd_puti(spdArr[pullSpeed]);
          dly=300;
        }
        HAL_Delay(dly);
      }
      else if(ch==FDOWN)
      {
        if(lineCnt==0)    //mode selection
        {
          if(pullMode>1)
            pullMode--;
          else
            pullMode=4;
          lcd_goto(LINE1+10);
          lcd_putch(pullMode+0x30);
          dly=300;
        }
        else if(lineCnt==1)
        {
          if(minPull>0)
            minPull--;
          else
            minPull=maxPull;
          //convVal=convert(minPull);
          lcd_goto(LINE2+10);
          lcd_puti(minPull);
          if(keyCnt>0)
            keyCnt-=10;
          dly=keyCnt;
        }
        else if(lineCnt==2)
        {
          prvUnit=pullUnit;
          if(pullUnit>0)
            pullUnit--;
          else
            pullUnit=2;
          lcd_goto(LINE3+10);
          dispUnit(pullUnit);
          dly=300;
          if(pullUnit==0)
            maxPull=MAX_PULL_K;
          else if(pullUnit==1)
            maxPull=MAX_PULL_N;
          else if(pullUnit==2)
            maxPull=MAX_PULL_L;
          minPull=convert2(minPull,prvUnit,pullUnit);
          if(minPull>maxPull)
            minPull=maxPull;
          lcd_goto(LINE2+10);
          lcd_puti(minPull);
        }
        else if(lineCnt==3)
        {
          if(pullSpeed>0)
            pullSpeed--;
          else
            pullSpeed=3;
          lcd_goto(LINE4+7);
          lcd_puti(spdArr[pullSpeed]);
          dly=300;
        }
        HAL_Delay(dly);
      }
      else
        keyCnt=300;
    }
   else if(seq==2)
    {
     tareWeight();
      if((pullMode==1)||(pullMode==4))
      {
        chx=getKey();
        if(chx==FSTART)
        {
          if(serviceCount<SERVICE)//25000)
          {
            serviceCount++;
#ifdef INTEE
            EEPROM.put(ADR_SERVICE, serviceCount);
#else
            int2str(serviceCount, buf, 8);
            Serial.println();
            for(int u=0; u<8; u++)
            {
              Serial.write(buf[u]);
              Serial.print(' ');
            }
            Serial.println();
            FM25V02_WRITE(ADDSERVICE, buf, 8);
#endif
            delay(500);
          }
          /*else
          {
            lcd_clear();
            lcd_putstr("Calibration Due",15,LINE2+2);
            //while(1);
            delay(1000);
          }*/
          maxOver=MAX_LOAD;
          maxOver=convert2(maxOver, U_Kgf,pullUnit);
          HAL_Delay(500);
          lcd_clear();
          if(sensorRead())
          {
            motorStop();
            lcd_putstr("Not at Home",11,LINE1+4);
            HAL_Delay(2000);
            home();
            seq=2;
            scrNormal();
            continue;
          }
          
          lcd_putstr("PULL TO BREAK",13,LINE1+3);
          lcd_putstr("LOAD:",5,LINE2);
          lcd_goto(LINE2+17);
          dispUnit(pullUnit);
          lcd_putstr("SPEED:",6,LINE3);
          lcd_goto(LINE3+7);
          lcd_puti(spdArr[pullSpeed]);
          lcd_putstr("MM/MIN",6,LINE3+14);
          int sp=spdArr[pullSpeed];
            pwmCount=pwmBuf[pullSpeed];//updatePWM(pwmBuf[0]);
          updatePWM(pwmCount);
          tareWeight();
          updateCnt=0;
          lcd_putstr("PULLING.......",14,LINE4);
          startTime=millis();
          tempTime=noLoadArr[pullSpeed];
          tempTime*=1000;
          err=0;
          tareWeight();
          minLoad=convert2(45, U_Kgf, pullUnit);//10;
          motorRunFwd();
          peakValue=0;
          prvCount=0;
          //updatePWM(100);                     //for speed calibration pourpose
          //adcOffset();
          if(ddg)
          {
            lcd_putstr("                    ",20,LINE1);
            lcd_goto(LINE1+6);
            lcd_puti(maxOver);
            //lcd_puti(minPull);
          }
          while(1)
          {
            chx=scanKey();
            if(chx==FSTOP)
            {
              motorStop();
              HAL_Delay(500);
              break;
            }
            if(!ddg)
              animate(LINE4+19);
            tempValue=actualWeight();
           // tempValue*=10;
            if(tempValue>400)
              minLoad=convert2(35, U_Kgf, pullUnit);//20;
            else if(tempValue>600)
              minLoad=convert2(45, U_Kgf, pullUnit);//30;
            difVal=0;
            if((tempValue>prvCount)&&(!sign))
            {
              difVal=tempValue-prvCount;
              if(tempValue>peakValue)//&&(difVal<=100))
                peakValue=tempValue;
            }
              prvCount=tempValue;
            lcd_goto(LINE2+7);
            if(sign==1)
            {
              lcd_putch('-');
            }
            else
            {
              lcd_putch(' ');
            }
            lcd_putid(tempValue);
            if((tempValue>=maxOver)&&(!sign))
            {
              if(ovlCount<5)
                ovlCount++;
              else
              {
                ovlCount=0;
                buzFlag=1;
                finalResultCount=prvCount;
                motorStop();
                lcd_clear();
                if(ddg==1)
                {
                  lcd_putstr("PEAK LOAD: ",11,LINE3);
                  lcd_putid(peakValue);
                  dispUnit(pullUnit);
                }
                lcd_putstr("OVER LOAD",9,LINE1+5);
                HAL_Delay(3000);
                updatePWM(0);
                seq=2;
                home();
                buzFlag=0;
                //scrMain();
                scrNormal();
                break;

              }
            }
            if((setPt)&&(sign))
              sign=0;
            if(!sign)
            {
              if(tempValue>(minLoad+5))
                setPt=1;
              if((tempValue<minLoad)&&(setPt))
              {
                buzFlag=1;
                motorStop();
                lcd_clear();
                lcd_putstr("Service Cycle:",14,LINE4);
                lcd_puti(serviceCount);
                lcd_putstr("PEAK LOAD: ",11,LINE3);
                lcd_putid(peakValue);
                dispUnit(pullUnit);
                lcd_putstr("Done!",5,LINE1+7);
                lcd_putstr("Press Save/Exit",15,LINE2+2);
                HAL_Delay(300);
                updatePWM(0);
                extBit=1;
                //buzFlag=0;
                break;
              }
            }
            diffTime=millis();
            diffTime=diffTime-startTime;
            if(ddg)
            {
              lcd_goto(LINE4+15);
              lcd_puti(diffTime/10);
              lcd_goto(LINE1+15);
              lcd_puti(initCnt);
            }
            if(diffTime>=tempTime)
            {
              buzFlag=1;
              digitalWrite(LED_O, HIGH);
              err=0;
              extBit=1;
              motorStop();
              lcd_clear();
              if(setPt)
                lcd_putstr("  ERROR MAX. LIMIT  ",20,LINE2);
              else
                lcd_putstr("   NO LOAD DETECT   ",20,LINE2);
              lcd_putstr("PEAK LOAD: ",11,LINE3);
              lcd_putid(peakValue);
              dispUnit(pullUnit);
                lcd_putstr("Press Save/Exit",15,LINE4+2);
              HAL_Delay(3000);
              updatePWM(0);
              buzFlag=0;
              break;
            }
            if(!digitalRead(LIMIT_FLT))
            {
              buzFlag=1;
              digitalWrite(LED_H, HIGH);
              motorStop();
              updatePWM(0);
              lcd_clear();
              lcd_putstr("Limit Reached",13, LINE2+2);
              delay(3000);
              retFwd();
              buzFlag=0;
              break;
            }
          }
          minLoad=45;
        }
        else if(chx==FEXIT)
        {
          buzFlag=0;
          HAL_Delay(500);
              digitalWrite(LED_O, LOW);
          home();
          lcd_clear();
          if(extBit)
          {
            extBit=0;
            seq=2;
            scrNormal();
          }
          else
          {
            scrMain();
            seq=0;
          }
        }
        else if(chx==FHOME)
        {
          buzFlag=0;
          HAL_Delay(500);
              digitalWrite(LED_O, LOW);
          seq=2;
          home();
          lcd_clear();
          scrNormal();//scrMain();
        }
        else if((chx==FSAVE)&&(!err)&&(extBit))
        {
          if(pullMode==4)
          {
            //setBuf(peakValue);getWeight
            getWeight(peakValue);
            for(int t=0;t<7;t++)
              Serial.write(buf[t]);
          }
          buzFlag=0;
          HAL_Delay(500);
              digitalWrite(LED_O, LOW);
          seq=2;
          home();
          lcd_clear();
          if(expNumber<250)
            expNumber++;
          else
            expNumber=0;
          lcd_putstr("Experiment No.",14,LINE1);
          lcd_putbch3(expNumber,LINE1+17);
          lcd_putstr("Date:",5,LINE2);
          dispDate(LINE2+6);
          dispTime(LINE2+15);
          lcd_putstr("PULL SPEED:",11,LINE3);
          lcd_puti(spdArr[pullSpeed]);
          lcd_putstr("PEAK LOAD:",10,LINE4);
          lcd_putid(peakValue);
          lcd_goto(LINE4+17);
          dispUnit(pullUnit);
          buf[0]=expNumber;
          FM25V02_WRITE(EXP_NUMBER,buf,1);
          temp=(expNumber*PAGE_SIZE)+PAGE_START;
          memTemp=temp+EXP_OFF;
          FM25V02_WRITE(memTemp,buf,1);
          memTemp=temp+ DATE_OFF;
          getDate();
          lcd_putstr(buf,8,LINE2+6);
          FM25V02_WRITE(memTemp,buf,8);
          memTemp=temp+ TIME_OFF;
          getTime();
          lcd_putstr(buf,5,LINE2+15);
          FM25V02_WRITE(memTemp,buf,5);
          memTemp=temp+SPEED_OFF;
          buf[0]=pullSpeed;
          FM25V02_WRITE(memTemp,buf,1);
          memTemp=temp+UNIT_OFF;
          buf[0]=pullUnit;
          FM25V02_WRITE(memTemp,buf,1);
          memTemp=temp+PEEKLOAD_OFF;
          getWeight(peakValue);
          FM25V02_WRITE(memTemp,buf,7);
          HAL_Delay(3000);
          lcd_clear();
          seq=2;
          scrNormal();//scrMain();
        }
      }
      else if(pullMode==2)
      {
        maxOver=MAX_LOAD;
        maxOver=convert2(maxOver, U_Kgf, pullUnit);
        minPullExp=minPull*10;
        chx=getKey();
        if(chx==FSTART)
        {
          if(serviceCount<SERVICE)//25000)
          {
            serviceCount++;
#ifdef INTEE
            EEPROM.put(ADR_SERVICE, serviceCount);
#else
            int2str(serviceCount, buf, 8);
            FM25V02_WRITE(ADDSERVICE, buf, 8);
#endif
          }
          lcd_clear();
          if(sensorRead())
          {
            motorStop();
            lcd_putstr("Not at Home",11,LINE1+4);
            HAL_Delay(2000);
            home();
            continue;
          }
          lcd_putstr("PULL to HOLD ",13,LINE1+3);
          lcd_putstr("TGT LOAD:",9,LINE2);
          //convVal=convert(minPullExp);
          lcd_putid(minPullExp);
          lcd_putstr("CUR LOAD:",9,LINE3);
          lcd_goto(LINE2+17);
          dispUnit(pullUnit);
          lcd_goto(LINE3+17);
          dispUnit(pullUnit);
          lcd_putstr("SPEED:",6,LINE4);
          lcd_goto(LINE4+7);
          lcd_puti(spdArr[pullSpeed]);
          lcd_putstr("MM/MIN",6,LINE4+14);
          minLoad=convert2(45, U_Kgf, pullUnit);
          diffComp1=convert2(10, U_Kgf, pullUnit);
          /*lmt1=convert2(5, U_Kgf, pullUnit);
          lmt2=convert2(20, U_Kgf, pullUnit);//20;
          lmt3=convert2(40, U_Kgf, pullUnit);//40;
          lmt4=convert2(60, U_Kgf, pullUnit);//60;
          lmt5=convert2(70, U_Kgf, pullUnit);//70;*/
          for(int k=0;k<3;k++)
          {
            tareWeight();
            HAL_Delay(150);
          }
          startTime=millis();
          tempTime=noLoadArr[pullSpeed];
          tempTime*=1000;
          err=0;
          motorRunFwd();
          pwmCount=pwmBuf[pullSpeed];
          updatePWM(pwmCount);
          peakValue=0;
          prvCount=0;
          //adcOffset();
          
          prvCount=0;//actualWeight();
          while(1)
          {
            chx=scanKey();
            if(chx==FSTOP)
            {
              motorStop();
              HAL_Delay(500);
              seq=2;
              lcd_clear();
              lcd_putstr("Press EXIT/HOME",15,LINE2+2);
              extBit=1;
              int h=getKey();
              HAL_Delay(300);
              if(h==FHOME)
                home();
              buzFlag=0;
              lcd_clear();
              scrNormal();//scrMain();
              break;
            }
            tempValue=actualWeight();
            lcd_goto(LINE3+9);
            lcd_putid(tempValue);
            difVal=0;
            if(!sign)
            {
              if(tempValue>(minLoad+5))
                setPt=1;
              if((tempValue<minLoad)&&(setPt))
              {
                buzFlag=1;
                 finalResultCount=prvCount;
                  motorStop();
                  lcd_clear();
                  lcd_putstr("PEAK LOAD: ",11,LINE3);
                  lcd_putid(peakValue);
                  dispUnit(pullUnit);
                  lcd_putstr("ABNORMAL BREAK",14,LINE1);
                  HAL_Delay(5000);
                  updatePWM(0);
                  seq=2;
                  home();
                  //buzFlag=0;
                  scrNormal();//scrMain();
                  //buzFlag=0;
                  break;
              }
              if(tempValue>prvCount)
              {
                if(tempValue>peakValue)
                  peakValue=tempValue;
              }
              else
              {
                diffCount= prvCount-tempValue;
                if(diffCount>diffComp1)//20)
                {
                  buzFlag=1;
                  finalResultCount=prvCount;
                  motorStop();
                  lcd_clear();
                  lcd_putstr("PEAK LOAD: ",11,LINE3);
                  lcd_putid(peakValue);
                  dispUnit(pullUnit);
                  lcd_putstr("ABNORMAL BREAK",14,LINE1);
                  HAL_Delay(5000);
                  updatePWM(0);
                  seq=2;
                  home();
                  //buzFlag=0;
                  scrNormal();//scrMain();
                  //buzFlag=0;
                  break;
                }
              }
              prvCount=tempValue;
              /*error=minPullExp-tempValue;
              if((error<=lmt3)&&(error>lmt2))
              {
                if(pwmCount>=6)
                  pwmCount--;
                updatePWM(pwmCount);
              }
              else if((error<=lmt2)&&(error>lmt1))
              {
                if(pwmCount>=4)
                  pwmCount--;
                updatePWM(pwmCount);
              }*/
              }
            if((tempValue>=maxOver)&&(!sign))
            {
              if(ovlCount<5)
                ovlCount++;
              else
              {
                ovlCount=0;
                buzFlag=1;
                finalResultCount=prvCount;
                motorStop();
                lcd_clear();
                lcd_putstr("PEAK LOAD: ",11,LINE3);
                lcd_putid(peakValue);
                dispUnit(pullUnit);
                lcd_putstr("OVER LOAD",9,LINE1+5);
                HAL_Delay(3000);
                updatePWM(0);
                seq=2;
                home();
                scrNormal();
                //buzFlag=0;
                break;

              }
            }
            if(tempValue>=minPullExp)
            {
              motorStop();
              lcd_goto(LINE3+9);
              lcd_putid(tempValue);
              HAL_Delay(300);
              //updatePWM(0);
              prvCount=0;
              lcd_clear();
              //counter=0;
              for(int tt=20;tt>0;tt--)
              {
                lcd_goto(LINE1+4);
                lcd_puti(tt);
                chx=scanKey();
                if((chx==FSTOP)||(chx==FHOME))
                {
                  HAL_Delay(500);
                  seq=2;
                  home();
                  buzFlag=0;
                  lcd_clear();
                  scrNormal();//scrMain();
                  break;
                }
                tempValue=actualWeight();
                lcd_goto(LINE3+9);
                if(ddg==1)
                {
                  lcd_putid(tempValue);
                  dispUnit(pullUnit);
                }
                if(tempValue<diffComp1)//minPullExp)
                {
                  if(mnlCount<5)
                  {
                    mnlCount++;
                  }
                  else
                  {
                    mnlCount=0;
                    buzFlag=1;
                    finalResultCount=prvCount;
                    motorStop();
                    lcd_clear();
                    lcd_putstr("PEAK LOAD: ",11,LINE3);
                    lcd_putid(peakValue);
                    dispUnit(pullUnit);
                    lcd_putstr("ERROR3",6,LINE1+7);
                    HAL_Delay(3000);
                    updatePWM(0);
                    seq=2;
                    home();
                    scrNormal();
                    //buzFlag=0;
                    break;
                  }
                }
                if(tempValue>prvCount)
                {
                  if(tempValue>peakValue)
                    peakValue=tempValue;
                }
                else
                {
                  diffCount= prvCount-tempValue;
                  if(diffCount>diffComp1)
                  {
                    buzFlag=1;
                    finalResultCount=prvCount;
                    motorStop();
                    lcd_clear();
                    lcd_putstr("PEAK LOAD: ",11,LINE3);
                    lcd_putid(peakValue);
                    dispUnit(pullUnit);
                    lcd_putstr("ERROR2",6,LINE1+7);
                    lcd_putstr("!Abnormal Break",15, LINE4);
                    HAL_Delay(3000);
                    updatePWM(0);
                    seq=2;
                    home();
                    //buzFlag=0;
                    break;
                  }
                }
                prvCount=tempValue;
                HAL_Delay(375);
              }
              lcd_goto(LINE1+4);
              lcd_puti(0);
              lcd_clear();
              buzFlag=1;
              lcd_putstr("Returning to HOME",17,LINE2+1);
              updatePWM(100);
              while(sensorRead())
              {
                motorRunRvs();
              }
              motorStop();
              updatePWM(0);
              buzFlag=0;
              lcd_clear();
              scrNormal();//scrMain();
              seq=2;
              break;
            }
            diffTime=millis();
            diffTime=diffTime-startTime;
            if(diffTime>=tempTime)
            {
              buzFlag=1;
              digitalWrite(LED_O, HIGH);
              err=0;
              motorStop();
              lcd_clear();
              lcd_putstr("  ERROR MAX. LIMIT  ",20,LINE2);
              lcd_putstr("PEAK LOAD: ",11,LINE3);
              lcd_putid(peakValue);
              dispUnit(pullUnit);
              lcd_putstr("Press Save/Exit",15,LINE4+2);
              HAL_Delay(3000);
              updatePWM(0);
              buzFlag=0;
              break;
            }
            if(!digitalRead(LIMIT_FLT))
            {
              buzFlag=1;
              digitalWrite(LED_H, HIGH);
              motorStop();
              updatePWM(0);
              lcd_clear();
              lcd_putstr("Limit Reached",13, LINE2+2);
              delay(3000);
              retFwd();
              buzFlag=0;
              break;
            }
          }
        }
        else if(chx==FEXIT)
        {
          buzFlag=0;
          HAL_Delay(500);
              digitalWrite(LED_O, LOW);
          home();
          lcd_clear();
          if(extBit)
          {
            extBit=0;
            seq=2;
            scrNormal();
          }
          else
          {
            seq=0;          
            scrMain();
          }
        }
        else if(chx==FHOME)
        {
          buzFlag=0;
          HAL_Delay(500);
              digitalWrite(LED_O, LOW);
          seq=2;
          home();
          buzFlag=0;
          lcd_clear();
          scrNormal();//scrMain();
        }
      }
      else if(pullMode==3)
      {
        int sp=spdArr[pullSpeed];
        minLoad=convert2(45, U_Kgf, pullUnit);
        diffComp1=convert2(10, U_Kgf, pullUnit);
        minPullExp=minPull*10;
        chx=getKey();
        if(chx==FSTART)
        {
          if(serviceCount<SERVICE)//25000)
          {
            serviceCount++;
#ifdef INTEE
            EEPROM.put(ADR_SERVICE, serviceCount);
#else
            int2str(serviceCount, buf, 8);
            FM25V02_WRITE(ADDSERVICE, buf, 8);
#endif
          }
          lcd_clear();
          if(sensorRead())
          {
            motorStop();
            lcd_putstr("Not at Home",11,LINE1+4);
            HAL_Delay(2000);
            home();
            buzFlag=0;
            continue;
          }
          lcd_putstr("PULL 2 RETURN",13,LINE1+3);
          lcd_putstr("TGT LOAD:",9,LINE2);
          //convVal=convert(minPullExp);
          lcd_putid(minPullExp);
          lcd_putstr("CUR LOAD:",9,LINE3);
          lcd_goto(LINE2+17);
          dispUnit(pullUnit);
          lcd_goto(LINE3+17);
          dispUnit(pullUnit);
          lcd_putstr("SPEED:",6,LINE4);
          lcd_goto(LINE4+7);
          lcd_puti(spdArr[pullSpeed]);
          lcd_putstr("MM/MIN",6,LINE4+14);
          prvCount=0;
          tareWeight();
          startTime=millis();
          tempTime=noLoadArr[pullSpeed];
          tempTime*=1000;
          err=0;
          motorRunFwd();
          pwmCount=pwmBuf[pullSpeed];
          updatePWM(pwmCount);
          peakValue=0;
          while(1)
          {
            chx=scanKey();
            if(chx==FSTOP)
            {
              HAL_Delay(500);
              seq=2;
              home();
              buzFlag=0;
              lcd_clear();
              scrNormal();//scrMain();
              break;
            }
            tempValue=actualWeight();
            if(!sign)
            {
              if(tempValue>(minLoad+5))
                setPt=1;
              if((tempValue<minLoad)&&(setPt))
              {
                buzFlag=1;
                 finalResultCount=prvCount;
                  motorStop();
                  lcd_clear();
                  lcd_putstr("PEAK LOAD: ",11,LINE3);
                  lcd_putid(peakValue);
                  dispUnit(pullUnit);
                  lcd_putstr("ABNORMAL BREAK",14,LINE1);
                  HAL_Delay(5000);
                  updatePWM(0);
                  seq=2;
                  home();
                  //buzFlag=0;
                  scrNormal();//scrMain();
                  //buzFlag=0;
                  break;
              }
              lcd_goto(LINE3+10);
              lcd_putid(tempValue);
              if(tempValue>prvCount)
              {
                if(tempValue>peakValue)
                  peakValue=tempValue;
              }
              else//if(tempValue<prvCount)
              {
                diffCount= prvCount-tempValue;
                if(diffCount>diffComp1)
                {
                  buzFlag=1;
                  finalResultCount=prvCount;
                  motorStop();
                  lcd_clear();
                  lcd_putstr("PEAK LOAD: ",11,LINE3);
                  lcd_putid(peakValue);
                  dispUnit(pullUnit);
                  lcd_putstr("ERROR",5,LINE1+7);
                  lcd_putstr("!Abnormal Break",15, LINE4);
                  HAL_Delay(5000);
                  updatePWM(0);
                  home();
                  //buzFlag=0;
                  lcd_clear();
                  seq=2;
                  scrNormal();////mainScr();
                  break;
                }
              }
              error=minPullExp-tempValue;
            }
            if(tempValue>=minPullExp)
            {
              //convVal=convert(minPullExp);
              lcd_goto(LINE3+9);
              lcd_putid(minPullExp);
              motorStop();
              buzFlag=1;
              //updatePWM(0);
              HAL_Delay(2000);
              lcd_clear();
              lcd_putstr("Running to Home",15,LINE2+2);
              updatePWM(100);
              while(sensorRead())
              {
                motorRunRvs();
              }
              motorStop();
              updatePWM(0);
              buzFlag=0;
              lcd_clear();
              scrNormal();//scrMain();
              seq=2;
              break;
            }
            prvCount=tempValue;
            diffTime=millis();
            diffTime=diffTime-startTime;
            if(diffTime>=tempTime)
            {
              buzFlag=1;
              digitalWrite(LED_O, HIGH);
              err=0;
              motorStop();
              lcd_clear();
              lcd_putstr("  ERROR MAX. LIMIT  ",20,LINE2);
              lcd_putstr("PEAK LOAD: ",11,LINE3);
              lcd_putid(peakValue);
              dispUnit(pullUnit);
              lcd_putstr("Press Save/Exit",15,LINE4+2);
              HAL_Delay(3000);
              updatePWM(0);
              buzFlag=0;
              break;
            }
            if(!digitalRead(LIMIT_FLT))
            {
              buzFlag=1;
              digitalWrite(LED_H, HIGH);
              motorStop();
              updatePWM(0);
              lcd_clear();
              lcd_putstr("Limit Reached",13, LINE2+2);
              delay(3000);
              retFwd();
              buzFlag=0;
              break;
            } 
          }
        }
        else if(chx==FEXIT)
        {
          buzFlag=0;
          HAL_Delay(500);
              digitalWrite(LED_O, LOW);
          home();
          lcd_clear();
          if(extBit)
          {
            extBit=0;
            seq=2;
            scrNormal();
          }
          else
          {
            scrMain();
            seq=0;
          }
        }
        else if(chx==FHOME)
        {
          buzFlag=0;
          HAL_Delay(500);
              digitalWrite(LED_O, LOW);
          seq=2;
          home();
          lcd_clear();
          scrNormal();//scrMain();
        }
      }
    }
   else if(seq==3)
    {
#ifndef WIFIEN
        lcd_clear();
        lcd_putstr("This feature",12,LINE2+4);
        lcd_putstr("NOT available",13,LINE3+4);
        delay(5000);
        seq=0;
        lcd_clear();
        scrMain();
#else      
      if(st==0x01)
      {
        buzFlag=1;
        lcd_putstr("SD Card not present",19,LINE1);
        seq=0;
        HAL_Delay(3000);
        scrMain();
        buzFlag=0;
        continue;
      }
      HAL_Delay(300);
      //HAL_GPIO_WritePin(SD_Port, SD_Pin, GPIO_PIN_RESET);
      
      /*if(f_open(&fil, "INPUT.CSV", FA_READ) != FR_OK)
      {
        buzFlag=1;
        lcd_putstr("Can not open the fil",20,LINE2);
        HAL_Delay(3000);
        buzFlag=0;
      }*/
      lcd_goto(0);
      line=0;
      indx=0;
      col=0;
      lcd_clear();
      //scrExtExp();
      dlyCount=0;
      keySys=0;
      indx=id*38;
      //lcd_putbch3(id,LINE1);
      //lcd_putbch3(indx,LINE1+3);
      //f_lseek(&fil, 0);
      /*for(t=0;t<id-1;t++)
        f_read(&fil,buf,38,&byteswritten);*/
      keyCnt=300;
      lcd_putstr("Serial No.",10,LINE2);
      while(1)
      {
        lcd_goto(LINE2+11);
        lcd_puti(sdCounter);
        chF=getKey();
        if(chF==FEXIT)
        {
          seq=0;
          scrMain();
          //dataFile.close();
          //f_close(&fil);
          //HAL_GPIO_WritePin(SD_Port, SD_Pin, GPIO_PIN_SET);
          break;
        }
        else if(chF==FSAVE)
        {
          File dataFile = SD.open("INPUT.CSV");
          int pptr=sdCounter*40;
          int nLine=1, ind=0;
          while(1)
          {
            char chs=dataFile.read();
            if(chs=='\n')
            {
              if(nLine==sdCounter)
                break;
              nLine++;
            }
          }
          t=0;
          char c2;
          int siz, q;
          do{
            c2=dataFile.read();
            if(c2!='\r')
            {
              if(c2==',')
              {
                siz=t;
                if(ind==0)
                {
#ifdef SER_DEBUG
                  Serial.print("id buf size:");
                  Serial.println(t);
#endif
                  sizId=t;
                  idBuf[t]='\0';
                }
                else if(ind==1)
                {
#ifdef SER_DEBUG
                  Serial.print("terminal:");
                  Serial.println(t);
#endif
                  sizTerm=t;
                  terminal[t]=0;
                }
                else if(ind==2)
                {
#ifdef SER_DEBUG
                  Serial.print("minpull:");
                  Serial.println(t);
#endif
                  sizMin=t;
                  minpull[t]=0;
                }
                else if(ind==3)
                {
#ifdef SER_DEBUG
                  Serial.print("unit:");
                  Serial.println(t);
#endif
                  sizUnit=t;
                  unit[t]=0;
                }
                else if(ind==4)
                {
#ifdef SER_DEBUG
                  Serial.print("information:");
                  Serial.println(t);
#endif
                  sizInfo=t;
                  information[t]=0;
                }
                t=0;
                ind++;
              }
              else
              {
                if(ind==0)
                {
                  idBuf[t]=c2;
                  t++;
                }
                else if(ind==1)
                {
                  terminal[t]=c2;
                  t++;
                }
                else if(ind==2)
                {
                  minpull[t]=c2;
                  t++;
                }
                else if(ind==3)
                {
                  unit[t]=c2;
                  t++;
                }
                else if(ind==4)
                {
                  information[t]=c2;
                  t++;
                }
                else if(ind==5)
                {
                  if(t==0)
                  {
#ifdef SER_DEBUG
                    Serial.println(c2);
#endif
                    pullSpeed=c2&0x0f;
#ifdef SER_DEBUG
                    Serial.println("pull Speed :"+pullSpeed);
#endif
                  }
                  t++;
                }
              }
            }
          }while(c2!='\n');
          //lcd_goto(LINE1);
          //lcd_puti(sdCounter);
          HAL_Delay(500);
          int nor=0;
          lcd_clear();
          if(unit[0]!='K')
          {
            if(unit[0]!='N')
            {
              if(unit[0]!='L')
              {
                lcd_putstr("Invalid UNIT Select.",20,LINE1);
                delay(3000);
                seq=0;
                lcd_clear();
                scrMain();
                nor=1;
              }
            }
          }
#ifdef SER_DEBUG        
          Serial.println("!unit OK");
#endif          
          if(pullSpeed>3)
          {
            lcd_putstr("Invalid SPEED Select",20,LINE2);
            delay(3000);
            seq=0;
            lcd_clear();
            scrMain();
            nor=1;
          }
#ifdef SER_DEBUG        
          Serial.println("!speed OK");
#endif     
          {
            int mul=1, ic;
            minPull=0;
            for(int t=sizMin ; t>0 ;t--)
            {
              ic=((minpull[t-1]&0x0f));
              if((ic<=9)&&(ic>=0))
              {
               // Serial.println("Valid:");
               // Serial.println(ic);
                minPull=minPull+(ic*mul);
                mul*=10;
              }
              else
              {
               // Serial.println("invalid:");
               // Serial.print(ic);
                lcd_putstr("Invalid Minimum Pull",20,LINE3);
#ifdef SER_DEBUG        
          Serial.println("!invalid minimum pull");
#endif     
                nor=1;
                delay(3000);
                seq=0;
                lcd_clear();
                scrMain();
                break;
              }
            }
#ifdef SER_DEBUG        
          Serial.println("!minimum pull ok");
#endif     
            if(unit[0]=='K')
            {
              if(minPull>MAX_PULL_K)
              {
#ifdef SER_DEBUG
                Serial.println("Capacity Mismatch in KGF");
#endif                              
                lcd_putstr("Capacity Mismatch",17,LINE4);
              
                delay(3000);
              seq=0;
              lcd_clear();
              scrMain();
              nor=1;  
              }
            }
            else if(unit[0]=='N')
            {
              if(minPull>MAX_PULL_N)
              {
#ifdef SER_DEBUG
                Serial.println("Capacity Mismatch in Newton");
#endif                     
                lcd_putstr("Capacity Mismatch",17,LINE4);
              
              delay(3000);
            seq=0;
            lcd_clear();
            scrMain();
            nor=1;
              }
            }
            else if(unit[0]=='L')
            {
              if(minPull>MAX_PULL_L)
              {
#ifdef SER_DEBUG
                Serial.println("Capacity Mismatch in Lbf");
#endif                     
                lcd_putstr("Capacity Mismatch",17,LINE4);
              
              delay(3000);
            seq=0;
            lcd_clear();
            scrMain();
            nor=1;
              }
            }
#ifdef SER_DEBUG        
          Serial.println("!capacity OK");
#endif     
          }
          if(nor==0)
          {
#ifdef SER_DEBUG        
          Serial.println("!display result");
#endif     
            scrExtExp();
            //id=(((idBuf[0]&0x0f)*100)+((idBuf[1]&0x0f)*10)+((idBuf[2]&0x0f)*1));
            id=getValue(idBuf, sizId);
            //lcd_putbch3(id, LINE1+10);
            lcd_putstr(terminal,sizTerm,LINE2+10);
            lcd_putstr(minpull,sizMin,LINE4+10);
            lcd_putstr(unit,sizUnit,LINE4+17);
            lcd_putstr(information,sizInfo,LINE3+10);
            lcd_goto(LINE1+10);
            //if(unit[0]=='K')
    
#ifdef SER_DEBUG        
            Serial.println();
            Serial.println(minPull);
#endif
            lcd_puti(spdArr[pullSpeed]);
            //float abc=minpull.toFloat();
            //minPull=(unsigned int)(abc*10);
            HAL_Delay(300);
            //getKey();
            if(unit[0]=='K')
              pullUnit=0;
            else if(unit[0]=='N')
              pullUnit=1;
            else if(unit[0]=='L')
              pullUnit=2;
            chF=0;
            seq=6;
            //lcd_clear();
            dataFile.close();
            //f_close(&fil);
            //HAL_GPIO_WritePin(SD_Port, SD_Pin, GPIO_PIN_SET);
            HAL_Delay(1000);
            buf[0]=(unsigned char)((id&0xff00)>>8);
            buf[1]=(unsigned char)(id&0x00ff);
            FM25V02_WRITE(ADDSDID,buf,2);
          }
          else
          {
#ifdef SER_DEBUG        
          Serial.println("!nor fail");
#endif      
          }
          break;
        }
        else if(chF==FDOWN)
        {
         // HAL_Delay(dly);
          if(sdCounter>1)
            sdCounter--;
          else
            sdCounter=SDMAX;
          if(keyCnt>10)
            keyCnt-=10;
          dly=keyCnt;
          HAL_Delay(dly);
        }
        else if(chF==FUP)
        {
          //
          if(sdCounter<SDMAX)
            sdCounter++;
          else
            sdCounter=1;
          if(keyCnt>10)
            keyCnt-=10;
          dly=keyCnt;
          HAL_Delay(dly);
        }
        else
          keyCnt=300;
        //HAL_Delay(150);
      }
     // HAL_Delay(500);
#endif     
    }
    else if(seq==4)
    {
        HAL_Delay(500);
        lcd_clear();
#ifndef WIFIEN
        lcd_putstr("Wi-Fi not available ", 20, LINE2);
        delay(5000);
        seq=0;
        lcd_clear();
        scrMain();
#else
        Serial1.print("$I!");
#ifdef SER_DEBUG
        Serial.print("$I!");
#endif
        lcd_putstr("IP Address:",10, LINE1);
        lcd_putstr("SSID:",5, LINE3);
        seq=8;
#endif
    }
   else if(seq==5)           //PC CONNECTION
    {
      if(pcFlag==0)
      {
        seq=0;
        motorStop();
        lcd_clear();
        scrMain();
      }
      if(cmd)
      {
        lcd_goto(LINE3);
        lcd_putch(cmd+0x30);
        if((cmdBuf[0]=='A')&&(expFlag==0))          //start experiment
        {
          lcd_putstr("start Cmd",9,LINE1+10);
          pullSpeed=cmdBuf[1]-0x30;
          pullMode=cmdBuf[2]-0x30;
          pullUnit=cmdBuf[3]-0x31;
          if(serviceCount<SERVICE)//25000)
          {
            serviceCount++;
#ifdef INTEE
            EEPROM.put(ADR_SERVICE, serviceCount);
#else
            int2str(serviceCount, buf, 8);
            FM25V02_WRITE(ADDSERVICE, buf, 8);
#endif
          }
          /*else
          {
            lcd_clear();
            lcd_putstr("Calibration Due",15,LINE2+2);
            //while(1);
            delay(1000);
          }*/
          if(pullUnit>2)
            pullUnit=0;
         // lcd_putch(cmdBuf[1]);
          lcd_putstr("SPEED :",7,LINE1);
          lcd_goto(LINE1+8);  
          lcd_puti(spdArr[pullSpeed]);
          lcd_putstr(" MODE :",7, LINE2);
          lcd_goto(LINE2+8); 
          lcd_putch(pullMode+0x30);
          //convVal=convert(minPull);
          lcd_goto(LINE3+10);
          lcd_puti(minPull);
          dispUnit(pullUnit);
          lcd_putstr(" UNIT :",7,LINE3);
         // lcd_putch(cmdBuf[0]);
          expFlag=1;
          tareWeight();
         // cleanAdcBuff();
          peakValue=0;
          setPt=0;
          startTime=millis();
          tempTime=noLoadArr[pullSpeed];
          tempTime*=1000;
          int sp=spdArr[pullSpeed];
          if(sp==50)
            updatePWM(13);
          else if(sp==100)
            updatePWM(20);
          else if(sp==150)
            updatePWM(27);
          else if(sp==200)
            updatePWM(34);
          //updatePWM(spdArr[pullSpeed]);
          if(pullMode==1)
            motorRunFwd();
        }
        else if(cmdBuf[0]=='S')
        {
          //stop experiment
          expFlag=0;
          motorStop();
          buf[0]='T';
#ifdef SER_DEBUG
          Serial.write(buf[0]);
#endif
          //CDC_Transmit_FS(buf,1);
          lcd_clear();
          lcd_putstr("Connected to PC",15,LINE2+3);
        }
        else if((cmdBuf[0]=='H')&&(expFlag==0))
        {
          //goto home
          homeFlag=1;
        }
        /*else if(cmd==4)
        {
          lcd_putstr("paramerters",11,LINE4);
          minPull=((cmdBuf[0]-0x30)*100)+((cmdBuf[1]-0x30)*10)+(cmdBuf[2]-0x30);
          lcd_goto(LINE1);
          //convVal=convert(minPull);
          lcd_puti(minPull);
          motorRunFwd();
        }*/
        cmd=0;
      }
      if(expFlag)
      {
        tempValue=actualWeight();
        //Serial.println(String("[")+tempValue+"]");
        getWeight2(tempValue);
        if(!sign)
          serWrite(7);
        if((tempValue>prvCount)&&(!sign))
        {
          if(tempValue>peakValue)
            peakValue=tempValue;
        }
        prvCount=tempValue;
        if((setPt)&&(sign))
          sign=0;
        if(!sign)
        {
          if(tempValue>(minLoad+5))
            setPt=1;
          if((tempValue<minLoad)&&(setPt))
          {
            buzFlag=1;
            motorStop();
            lcd_goto(LINE4);
            lcd_putid(peakValue);
            HAL_Delay(3000);
            while(sensorRead())
            {
              motorRunRvs();
            }
            motorStop();
            buzFlag=0;
            updatePWM(0);
            expFlag=0;
            //getWeight(peakValue);
            buf[0]='[';
            buf[1]='N';
            buf[2]='1';
            buf[3]='2';
            buf[4]='3';
            buf[5]='.';
            buf[6]=']';
            serWrite(7);
            //Serial.print(buf);
            //CDC_Transmit_FS(buf,5);
            lcd_clear();
            lcd_putstr("Connected to PC",15,LINE2+3);
            continue;
          }
        }
        diffTime=millis();//HAL_GetTick();
        diffTime=diffTime-startTime;
        if(diffTime>=tempTime)
        {
          err=1;
          motorStop();
              digitalWrite(LED_O, HIGH);
          lcd_clear();
          lcd_putstr("Error in Experiment",19,LINE2);
          lcd_putstr("PEAK LOAD: ",11,LINE3);
          lcd_putid(peakValue);
          dispUnit(pullUnit);
          HAL_Delay(3000);
          updatePWM(0);
        }
      }
      else
      {
        motorStop();
        updatePWM(0);
      }
      if(homeFlag)
      {
        homeFlag=0;
        buzFlag=0;
        updatePWM(100);
        while(sensorRead())
        {
          motorRunRvs();
        }
        motorStop();
        buzFlag=1;
        updatePWM(0);
      }
    }
   else if(seq==6)
    {
      //if(pullMode==1)
        wStart=0;
        err=exemode1();
        delay(3000);
        home();
      /*else
      {
        buzFlag=1;
        lcd_putstr("Select Pull Mode.1",18,LINE1+1);
        HAL_Delay(3000);
        buzFlag=0;
      }*/
      lcd_clear();
      if(err==0)
      {
        File fdPtr = SD.open("RESULT.CSV",FILE_WRITE);
        buf[0]=((id/100)+0x30);
        
        t=id%100;
        buf[1]=((t/10)+0x30);
        fdPtr.write(buf[1]);
        buf[2]=((t%10)+0x30);
        fdPtr.write(buf[2]);
        buf[3]=',';
        fdPtr.write(buf[3]);
        for(int q=0; q<sizTerm; q++)
        {
          fdPtr.write(terminal[q]);
        }
        fdPtr.write(',');
        for(int q=0; q<sizInfo; q++)
        {
          fdPtr.write(information[q]);
        }
        fdPtr.write(',');
        getDate();
        buf[8]=',';
        for(int q=0; q<9; q++)
        {
          fdPtr.write(buf[q]);
        }
        getTime();
        buf[5]=',';
        for(int q=0; q<6; q++)
        {
          fdPtr.write(buf[q]);
        }
        getPullSpeed();
        buf[3]=',';
        for(int q=0; q<4; q++)
        {
          fdPtr.write(buf[q]);
        }
        fdPtr.print("mm/min,");
        getWeight(peakValue);
        buf[7]=',';
        for(int q=0; q<8; q++)
        {
          fdPtr.write(buf[q]);
        }
        getUnit(pullUnit);
        buf[3]=',';
        for(int q=0; q<4; q++)
        {
          fdPtr.write(buf[q]);
        }
        if(pullSucc)
        {
          fdPtr.write('Y');
        }
        else
        {
          fdPtr.write('N');
        }
        fdPtr.println();
        fdPtr.close();
      }
      //HAL_Delay(3000);
      seq=0;
      scrMain();
    }
    else if(seq==7)
    {
      lcd_clear();
      lcd_putstr("Connected to WEB",16,LINE2+2);
      /*if(wStart>1)
        wStart=0;*/
      Serial1.print("$A");
      Serial1.write('0');
              Serial1.write('0');
              Serial1.write('0');
              Serial1.write('0');
              Serial1.write('0');
              Serial1.write('!');
#ifdef SER_DEBUG              
              Serial.print("$A");
              Serial.write('0');
              Serial.write('0');
              Serial.write('0');
              Serial.write('0');
              Serial.write('!');
        Serial.print("wStart");
        Serial.println(wStart);
       Serial.print("going in seq 9");
#endif        
      seq=9;
       delay(3000);
       delay(3000);
    }
    else if(seq==8)
    {
      if(scanKey() == FEXIT)
      {
        delay(1000);
        seq=0;
        lcd_clear();
        scrMain(); 
      }
        if(ipDone)
        {
          ipDone=0;
          int sz=sizeof(ipStr);
#ifdef SER_DEBUG
          Serial.println(sz);
#endif
          sz-=8;
          lcd_putstr(&ipStr[1],sz,LINE2);
          lcd_putstr(&ipStr[1],sz,LINE4+1);
          lcd_putstr("competent_0",11,LINE4);
          delay(500);
          getKeyh();
          delay(500);
          lcd_clear();
          seq=0;
          scrMain();  
        }
    }
    else if(seq==9)
    {
      //Serial.println("in seq 9");
      if(wStart==1)
      {
#ifdef SER_DEBUG
        Serial.print("Start bit:");
        Serial.println(wStart);
#endif
        exemode1();
        Serial1.print("$S");
          Serial1.write('0');
          Serial1.write(buf[1]);
          Serial1.write(buf[2]);
          Serial1.write(buf[3]);
          Serial1.write(buf[4]);
              Serial1.write('!');
#ifdef SER_DEBUG
              Serial.print("$S");
          Serial.write('0');
          Serial.write(buf[1]);
          Serial.write(buf[2]);
          Serial.write(buf[3]);
          Serial.write(buf[4]);
              Serial.write('!');
#endif
        wStart=0;
        home();
        wStart=0;
        seq=7;
        //scrMain();
#ifdef SER_DEBUG        
        Serial.print("wStart:");
        Serial.println(wStart);
#endif        
        delay(5000);
      }
    }
   else if(seq==10)
    {
      adcCal();
      seq=0;
      lcd_clear();
      scrMain();
    }
    //Serial.println('.');
      int srIndex;
      if(Serial.available())
      {
        char sr=Serial.read();
        if((sr=='*')&&(pcFlag==0))
        {
          pcFlag=1;
        }
        else if(sr=='@')
        {
          pcFlag=0;
        }
        else if(sr=='[')
        {
          srIndex=0; 
        }
        else if(sr==']')
        {
          cmd=1;
        }
        else
        {
          cmdBuf[srIndex]=sr;
          srIndex++;
        }
      }
#ifdef WIFIEN
     if(Serial1.available())
     {
      do{
        char wCh=Serial1.read();
        Serial.write('-');
        Serial.write(wCh);
        if(wCh=='&')
        {
          wIndx=0;
        }
        else if(wCh=='\n')
        {
          wDone=1;
          wBuf[wIndx]='\0';
        }
        else
        {
          wBuf[wIndx]=wCh;
          wIndx++;
        }
      }while(Serial1.available());
     }
     if(wDone)
     {
      wDone=0;
      wCmd=wBuf[0];
      if(wCmd=='C')
      {
        seq=7;
        wStart=0;
      }
      else if(wCmd=='B')
      {
        wStart=1;
      }
      else if(wCmd=='I')
      {
#ifdef SER_DEBUG
        Serial.println(wBuf);
#endif
        strcpy(ipStr, wBuf);
        ipDone=1;
      }
      else if(wCmd=='S')
      {
        pullSpeed=(wBuf[1]&0x0f);
#ifdef SER_DEBUG
        Serial.println("pull Speed "+pullSpeed);
#endif        
      }
     }
#endif
  }
}

unsigned char exemode1()
{
  unsigned char err;
  unsigned int chx;
  err=0;
  tempValue=0;
  prvCount=0;
  peakValue=0;
  setPt=0;
  minLoad=convert2(45, U_Kgf, pullUnit);
    tareWeight();
  //lcd_putstr("Press Start Key",15,LINE2+2);
#ifdef SER_DEBUG
  Serial.print("wStart ->");
  Serial.println(wStart);
#endif
  if(wStart)
  {
    pullUnit=U_N;
    chx=FSTART;
  }
  else
    chx=getKeyh();
    if(chx==FSTART)
    {
          if(serviceCount<SERVICE)//25000)
          {
            serviceCount++;
 #ifdef INTEE
            EEPROM.put(ADR_SERVICE, serviceCount);
#else
            int2str(serviceCount, buf, 8);
            FM25V02_WRITE(ADDSERVICE, buf, 8);
#endif
          }
          /*else
          {
            lcd_clear();
            lcd_putstr("Calibration Due",15,LINE2+2);
            //while(1);
            delay(1000);
          }*/
      maxOver=MAX_LOAD;
      maxOver=convert2(maxOver, U_Kgf,pullUnit);
      HAL_Delay(500);
      lcd_clear();
      //scrExperiment();
      if(sensorRead())
      {
        motorStop();
        lcd_putstr("Not at Home",11,LINE1+4);
        HAL_Delay(2000);
        home();
        //continue;
      }
      lcd_clear();
      lcd_putstr("PULL TO BREAK",13,LINE1+3);
      lcd_putstr("LOAD:",5,LINE2);
      lcd_goto(LINE2+17);
      dispUnit(pullUnit);
      lcd_putstr("SPEED:",6,LINE3);
      lcd_goto(LINE3+7);
      lcd_puti(spdArr[pullSpeed]);
      lcd_putstr("MM/MIN",6,LINE3+14);
      prvCount=0;
      int sp=spdArr[pullSpeed];
      pwmCount=pwmBuf[pullSpeed];
        /*if(pullSpeed==50)
          pwmCount=pwmBuf[0];
        else if(sp==100)
          pwmCount=pwmBuf[1];
        else if(sp==150)
          pwmCount=pwmBuf[2];
        else if(sp==200)
          pwmCount=pwmBuf[3];*/
        updatePWM(pwmCount);
      tareWeight();
      lcd_putstr("PULLING.......",14,LINE4);
      startTime=millis();
      tempTime=noLoadArr[pullSpeed];
      tempTime*=1000;
      err=0;
      tareWeight();
      motorRunFwd();
      peakValue=0;
//      cleanAdcBuff();
      while(1)
      {
        animate(LINE4+19);
        tempValue=actualWeight();
        if(tempValue>400)
          minLoad=convert2(35, U_Kgf, pullUnit);//20;
        else if(tempValue>600)
          minLoad=convert2(45, U_Kgf, pullUnit);//30;
        /*if(tempValue>(prvCount+100))
          continue;*/
        //tempValue=((tempValue*offSet)/100)+tempValue;
        if((tempValue>prvCount)&&(!sign))
        {
          if(tempValue>peakValue)
            peakValue=tempValue;
        }
        prvCount=tempValue;
        lcd_goto(LINE2+7);
        if(sign==1)
        {
          lcd_putch('-');
        }
        else
        {
          lcd_putch(' ');
        }
        lcd_putid(tempValue);
        if(wStart)
        {
          getWeight(tempValue);
          Serial1.print("$A");
          Serial1.write('0');
          Serial1.write(buf[1]);
          Serial1.write(buf[2]);
          Serial1.write(buf[3]);
          Serial1.write(buf[4]);
          Serial1.write('!');
#ifdef SER_DEBUG
          Serial.print("$A");
          Serial.write('0');
          Serial.write(buf[1]);
          Serial.write(buf[2]);
          Serial.write(buf[3]);
          Serial.write(buf[4]);
          Serial.write('!');
#endif          
        }
        if((setPt)&&(sign))
          sign=0;
        if(!sign)
        {
          if(tempValue>(minLoad+5))
            setPt=1;
          if((tempValue<minLoad)&&(setPt))
          {
            buzFlag=1;
            motorStop();
            if(wStart)
            {
              getWeight(peakValue);
              Serial1.print("$A");
              Serial1.write('0');
              Serial1.write('0');
              Serial1.write('0');
              Serial1.write('0');
              Serial1.write('0');
              Serial1.write('!');
             /* Serial1.print("$S");
          Serial1.write(buf[1]);
          Serial1.write(buf[2]);
          Serial1.write(buf[3]);
          Serial1.write(buf[4]);
          Serial1.write(buf[6]);
              Serial.write('!');*/
            }
            lcd_clear();
                lcd_putstr("Service Cycle:",14,LINE4);
                lcd_puti(serviceCount);
            lcd_putstr("PEAK LOAD: ",11,LINE3);
            lcd_putid(peakValue);
            dispUnit(pullUnit);
            int minPullTemp=minPull*10;
            if(!wStart)
            {
              if(peakValue<minPullTemp)
              {
                lcd_putstr("!NOT OK!",8,LINE1+6);
                pullSucc=0;
              }
              else
              {
                lcd_putstr("!  OK  !",8,LINE1+6);
                pullSucc=1;
              }
              lcd_putstr("Press Save/Exit",15,LINE2+2);
              HAL_Delay(3000);
            }
            updatePWM(0);
            buzFlag=0;
            break;
          }
        }
        diffTime=millis();
        diffTime=diffTime-startTime;
        if(diffTime>=tempTime)
        {
          buzFlag=1;
              digitalWrite(LED_O, HIGH);
          err=1;
          motorStop();
          lcd_clear();
          lcd_putstr("Error in Experiment",19,LINE2);
          lcd_putstr("PEAK LOAD: ",11,LINE3);
          lcd_putid(peakValue);
          dispUnit(pullUnit);
          HAL_Delay(3000);
          updatePWM(0);
          buzFlag=0;
          break;
        }
        chx=scanKey();
        if(chx==FSTOP)
        {
          HAL_Delay(500);
          motorStop();
          break;
        }
      }
    }
    //chx=getKeyh();
    else if(chx==FEXIT)
    {
      home();
      err=1;
      HAL_Delay(500);
    }
    else if((chx==FSAVE)&&(!err))
    {
      home();
      lcd_clear();
      err=0;
      HAL_Delay(500);
    }
    return err;
}

int getValue(unsigned char *abc, int sz)
{
  int ret=0;
#ifdef SER_DEBUG
  Serial.println(sz);
#endif  
  for(int y=0;y<sz;y++)
  {
    ret+=(*abc&0x0f);
#ifdef SER_DEBUG
    Serial.print(ret);
    Serial.print('\t');
#endif    
    abc++;
    ret*=10;
#ifdef SER_DEBUG
    Serial.print(ret);
    Serial.print('\t');
#endif    
  }
  ret/=10;
#ifdef SER_DEBUG
  Serial.print(ret);
#endif  
  return ret;
}

void serWrite(int cn)
{
  int s;
  for(s=0;s<cn;s++)
  {
    Serial.write(buf[s]);
  }
}

void buzInit(void)
{
  buzTimer=0;
  buzPitchOn=10;
  buzPitchOff=20;
  buzSync=0;
  buzFlag=0;
}
unsigned char buzzOff=0;

void callback()
{
  if(buzFlag)
  {
    if(buzTimer>0)
    {
      buzTimer--;
    }
    else
    {
      if(buzSync)
      {
        buzSync=0;
        buzTimer=buzPitchOn;
        digitalWrite(BUZZ, HIGH);
      }
      else
      {
        buzSync=1;
        buzTimer=buzPitchOff;
        digitalWrite(BUZZ, LOW);
      }
    }
  }
  else
  {
    if(buzzOff<100)
    {
      buzzOff++;
    }
    else
    {
      buzzOff=0;
      digitalWrite(BUZZ, LOW);
    }
  }
}
