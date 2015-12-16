/*

 Kamerton5_0_1ArduinoSpeed.ino
 VisualStudio
 
 ��������� ������������ ������ "��������" (������� �������)
 ������:      - 5_0_1Speed
 ����:        - 15.12.2015�.
 �����������: - ��� "������"
 �����:       - �������� �.�.
 ������: ����������� ������ �� 15.12.2015�. ����� ����� � ������ ���������� 
 ���������������  ������ ��� ��������� �� ������.
 ������ ���������� MODBUS

 �����������:
 -
 - ���������� 30��,
 - ��������/����� �� ��� �����,
 - ������� ���������� ����, ����� � ����������, 
 - ���������� MCP23017
 - ������ ����, 
 - ������ ���� ������, 
 - ��������� �������� ���������
 - ���������� SD ������
 - ���������� ����, ������, 
 */

#include <Modbus.h>
#include <ModbusSerial.h>
#include <Wire.h> 
#include <RTClib.h>
#include <MsTimer2.h> 
#include "MCP23017.h"
#include <AH_AD9850.h>
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>




//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ModbusSerial mb;
/*
����� ��������� ��������� MODBUS

mb.addCoil(1);   ��������  �������   1-1000
addCoil(word offset, bool value)   //  this->addReg(offset + 1, value?0xFF00:0x0000);
mb.Coil(1));    

addIsts(word offset, bool value)   // addReg(offset + 10001, value?0xFF00:0x0000);
mb.addIsts(1,1);
mb.addIsts(1);
mb.Ists(1, 1);

mb.addIreg(1);
addIreg(word offset, word value)   30000 - 39999
mb.Ireg(1, 1);

addHreg(word offset, word value)  40000-49999
Hreg(word offset, word value)     40000-49999
mb.addHreg(1, 1);
mb.Hreg(1);

*/


//************************************************************************************************

RTC_DS1307 RTC;                                     // define the Real Time Clock object

//-----------------------------------------------------------------------------------------------

uint8_t second = 0;       //Initialization time
uint8_t minute = 0;
uint8_t hour   = 0;
uint8_t dow    = 1;
uint8_t day    = 1;
uint8_t month  = 1;
uint16_t year  = 15 ;

//------------------------------------------------------------------------------------------------------------
//****************** �������� ������ � ������� �����-1 ******************************************************

//byte regs_in[5];                                           // �������� ������ � ������ �����-1
byte regs_out[4];                                            // �������� ������ � ������ �����-1
byte regs_crc[1];                                            // �������� ������ � ������ �����-1 ����������� �����
byte regs_temp                 = 0;
byte regs_temp1                = 0;
byte Stop_Kam                  = 0;                          // ���� ��������� ������ ���. �� �����-1
bool test_repeat               = true;                       // ���� ���������� �����
volatile bool prer_Kmerton_On  = true;                       // ���� ���������� ���������� �����-1
volatile bool prer_Kmerton_Run = false;                      // ���� ���������� ���������� �����-1
unsigned char bufferK;                                       // ������� ���������� ����������� ����
#define BUFFER_SIZEK           64                            // ������ ������ �������� �� ����� 128 ����

//------------------------------------------------------------------------------------------------------------

#define  ledPin13  13                               // ���������� ����������� �� �����
#define  ledPin12  12                               // ���������� ����������� �� �����
#define  ledPin11  11                               // ���������� ����������� �� �����
#define  ledPin10  10                               // ���������� ����������� �� �����
#define  Front_led_Blue 14                          // ���������� ����������� �� �������� ������
#define  Front_led_Red  15                          // ���������� ����������� �� �������� ������
//--------------------------------------------------------------------------------------------------------

//  ����� ���������� ������ ����� -1
#define DTR  8                                      // DTR out �������� ������  ������������ 0 ��� ������
#define RTS  9                                      // RTS out �������� ������   
#define CTS  5                                      // CTS in  ������� ������  ���� ������� �������� ��������������!!!!
#define DSR  6                                      // DSR in  ������� ������  ���� ������� "����� - ��������"
#define DCD  7                                      // DCD in  ������� ������  ���� ������ ������ � ��������� 


//  ����� ���������� ������ Arduino Nano
#define  kn1Nano   34                               // ���������� ������ ���������� Nano ��������� �������
#define  kn2Nano   36                               // ���������� ������ ���������� Nano ��������� 1000 ��
#define  kn3Nano   38                               // ���������� ������ ���������� Nano ��������� 2000 ��
#define  InNano12  40                               // ���������� ������ - ��������� ��������� 1000 ��� 2000 ��
#define  InNano13  39                               // ���������� ������ - ��������� ��������� ������� 

//------------------------------------------------------------------------------------------------------------
MCP23017 mcp_Out1;                                  // ���������� ������ ���������� MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                  // ���������� ������ ���������� MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                // ���������� ������ ���������� MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------

//******************** ��������� ��������� ������ *************************************************************
//AH_AD9850(int CLK, int FQUP, int BitData, int RESET);
AH_AD9850 AD9850(23, 25, 27, 29);
//-------------------------------------------------------------------------------------------------------------
//+++++++++++++++++++++++++++++ ������� ������ +++++++++++++++++++++++++++++++++++++++
int deviceaddress        = 80;                      // ����� ���������� ������
unsigned int eeaddress   =  0;                      // ����� ������ ������
byte hi;                                            // ������� ���� ��� �������������� �����
byte low;                                           // ������� ���� ��� �������������� �����

//+++++++++++++++++++++++ ��������� ������������ ��������� +++++++++++++++++++++++++++++++++++++
#define address_AD5252   0x2F                       // ����� ���������� AD5252  
#define control_word1    0x07                       // ���� ���������� �������� �1
#define control_word2    0x87                       // ���� ���������� �������� �2
byte resistance        = 0x00;                      // ������������� 0x00..0xFF - 0��..100���
//byte level_resist      = 0;                       // ���� ��������� ������ �������� ���������
//-----------------------------------------------------------------------------------------------


#define FASTADC 1                                   // ��������� ���������� ����������� �������
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ������� ����� 
const unsigned int adr_kontrol_day        PROGMEM      = 46; // ����� ����
const unsigned int adr_kontrol_month      PROGMEM      = 47; // ����� �����
const unsigned int adr_kontrol_year       PROGMEM      = 48; // ����� ���
const unsigned int adr_kontrol_hour       PROGMEM      = 49; // ����� ���
const unsigned int adr_kontrol_minute     PROGMEM      = 50; // ����� ������
const unsigned int adr_kontrol_second     PROGMEM      = 51; // ����� �������

// ��������� ������� � �����������
const unsigned int adr_set_kontrol_day    PROGMEM      = 52;   // ����� ����
const unsigned int adr_set_kontrol_month  PROGMEM      = 53;   // ����� �����
const unsigned int adr_set_kontrol_year   PROGMEM      = 54;   // ����� ���
const unsigned int adr_set_kontrol_hour   PROGMEM      = 55;   // ����� ���
const unsigned int adr_set_kontrol_minute PROGMEM      = 56;   // ����� ������

// ����� ������ �����
const unsigned int adr_Mic_Start_day      PROGMEM      = 96;   // ����� ����
const unsigned int adr_Mic_Start_month    PROGMEM      = 97;   // ����� �����
const unsigned int adr_Mic_Start_year     PROGMEM      = 98;   // ����� ���
const unsigned int adr_Mic_Start_hour     PROGMEM      = 99;   // ����� ���
const unsigned int adr_Mic_Start_minute   PROGMEM      = 100;  // ����� ������
const unsigned int adr_Mic_Start_second   PROGMEM      = 101;  // ����� �������
// ����� ��������� �����
const unsigned int adr_Mic_Stop_day       PROGMEM       = 102; // ����� ����
const unsigned int adr_Mic_Stop_month     PROGMEM       = 103; // ����� �����
const unsigned int adr_Mic_Stop_year      PROGMEM       = 104; // ����� ���
const unsigned int adr_Mic_Stop_hour      PROGMEM       = 105; // ����� ���
const unsigned int adr_Mic_Stop_minute    PROGMEM       = 106; // ����� ������
const unsigned int adr_Mic_Stop_second    PROGMEM       = 107; // ����� �������

// ����������������� ���������� �����
const unsigned int adr_Time_Test_day      PROGMEM       = 108; // ����� ����
const unsigned int adr_Time_Test_hour     PROGMEM       = 109; // ����� ���
const unsigned int adr_Time_Test_minute   PROGMEM       = 110; // ����� ������
const unsigned int adr_Time_Test_second   PROGMEM       = 111; // ����� �������
// ����� �������� �����
const unsigned int adr_reg_temp_year      PROGMEM       = 112; // ������� �������� ���������� ���  
const unsigned int adr_reg_temp_mon       PROGMEM       = 113; // ������� �������� ���������� �����
const unsigned int adr_reg_temp_day       PROGMEM       = 114; // ������� �������� ���������� ���� 
const unsigned int adr_reg_file_name      PROGMEM       = 115; // ������� �������� ������� ������  
const unsigned int adr_reg_file_tek       PROGMEM       = 116; // ������� �������� ������� ������  

const unsigned int adr_control_command    PROGMEM       = 120; // ����� �������� ������� �� ���������� 
const unsigned int adr_reg_count_err      PROGMEM       = 121; // ����� �������� ���� ������

const unsigned int adr_set_time           PROGMEM       = 36;  // ����� ���� ���������
//-------------------------------------------------------------------------------------------------

const int adr_reg_ind_CTS                 PROGMEM       = 81;        // ����� ����a ��������� ��������� ������� CTS
const int adr_reg_ind_DSR                 PROGMEM       = 82;        // ����� ����a ��������� ��������� ������� DSR
const int adr_reg_ind_DCD                 PROGMEM       = 83;        // ����� ����a ��������� ��������� ������� DCD

//----------------------------------------------------------------------------------------------

//++++++++++++++++++++++ ������ � ������� +++++++++++++++++++++++++++++++++++++++
//#define chipSelect SS
#define chipSelect 49   // ��������
SdFat sd;
File myFile;
SdFile file;
//SdFile dirFile;
Sd2Card card;
//
uint32_t cardSizeBlocks;
uint16_t cardCapacityMB;

// cache for SD block
cache_t cache;
//--------------------------------------------------------------------------------------------------
// ������� ����������, ������������ ������� ���������� SD utility library functions: +++++++++++++++
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;


//++++++++++++++++++++ ���������� ����� ����� ++++++++++++++++++++++++++++++++++++++++++++
//const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "150101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13]            = FILE_BASE_NAME "00.KAM";
char fileName_p[13];
char fileName_F[13];
//------------------------------------------------------------------------------

char c;  // ��� ����� ������� � ��� �����

// Serial output stream
ArduinoOutStream cout(Serial);
char bufferSerial2[128];  

//*********************������ � ������ ����� ******************************

//byte file_name_count = 0;
char str_day_file[3];
char str_day_file0[3];
char str_day_file10[3];
char str_mon_file[3];
char str_mon_file0[3];
char str_mon_file10[3];
char str_year_file[3];

char str0[10];
char str1[10];
char str2[10];
//--------------------------------------------------------------------------

//------------------------- ������ ��������� �������� �������� ��� ������������ ���������--------------------------------------
//++++++++++++++++++++++++++++ ��������� ��������� ������� ������� +++++++++++++++++++++++++++++++++++++

// ������ ������� ������ ��� �������� ������� ������� ��������� ��������

const  int adr_porog_instruktor            = 0;                // 19 ������� 
const  int adr_porog_dispatcher            = 30;               // 19 ������� 
const  int adr_porog_MTT                   = 60;               // 21 ������� 
const  int adr_porog_GGS                   = 90;               // 29 ������� 
const  int adr_porog_Radio1                = 120;              // 20 ������� 
const  int adr_porog_Radio2                = 150;              // 20 ������� 
const  int adr_porog_Microphone            = 180;              //  
const  int adr_set_USB                     = 200;  
// end                                     = 160
//

//����� ������ ��� ������, 2 �����
const  int adr_int_porog_instruktor            = 200;      // 19 ������� 
const  int adr_int_porog_dispatcher            = 250;      // 19 ������� 
const  int adr_int_porog_MTT                   = 300;      // 21 ������� 
const  int adr_int_porog_GGS                   = 350;      // 29 ������� 
const  int adr_int_porog_Radio1                = 420;      // 20 ������� 
const  int adr_int_porog_Radio2                = 460;      // 20 ������� 
const  int adr_int_porog_Microphone            = 500;      //
//end                                            550;


byte por_buffer[30] ;
int por_int_buffer[50] ;                 // ����� �������� ��������� ���������� ������� �������                      
//const byte porog_instruktor[]    PROGMEM  = {30,30,35,35,35,35,35,35,35,35,35,150,150,35,35,35,35,35,35,35,254};
const byte porog_instruktor[]    PROGMEM  = {
		 //++++++++++++++++  Test headset instructor ++++++++++++++++++++++++++++
25,			// 0                          // resistor(1, 30);     ���������� ������� ������� 30 ��
25,			// 1                          // resistor(2, 30);     ���������� ������� ������� 30 ��
15,			// 2                          // measure_vol_min(analog_FrontL,   40230,230,35);  ������� ������� �� ������ FrontL  
15,			// 3                          // measure_vol_min(analog_FrontR,   40231,231,35);  ������� ������� �� ������ FrontR
15,			// 4                          // measure_vol_min(analog_LineL,    40232,232,35);  ������� ������� �� ������ LineL 
15,			// 5                          // measure_vol_min(analog_LineR,    40233,233,35);  ������� ������� �� ������ LineR
15,			// 6                          // measure_vol_min(analog_mag_radio,40234,234,35);  ������� ������� �� ������ mag radio 
15,			// 7                          // measure_vol_min(analog_mag_phone,40235,235,35);  ������� ������� �� ������ mag phone
15,			// 8                          // measure_vol_min(analog_ggs,      40236,236,35);  ������� ������� �� ������ GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1,40237,237,35);  ������� ������� �� ������ GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2,40238,238,35);  ������� ������� �� ������ GG Radio2 
			//---------- ������ �� ���� ����� ---------------------
101,      	// 11                         // measure_vol_max(analog_LineL,    40224,224,150); ������� ������� �� ������ LineL
80,		    // 12                         // measure_vol_max(analog_mag_phone,40226,226,150); ������� ������� �� ������ mag phone 
15, 		// 13                         // measure_vol_min(analog_FrontL,   40230,230,35);  ������� ������� �� ������ FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,   40231,231,35);  ������� ������� �� ������ FrontR 
15,			// 15                         // measure_vol_min(analog_LineR,    40233,233,35);  ������� ������� �� ������ LineR 
15,			// 16                         // measure_vol_min(analog_ggs,      40236,236,35);  ������� ������� �� ������ GGS 
15,			// 17                         // measure_vol_min(analog_gg_radio1,40237,237,35);  ������� ������� �� ������ GG Radio1
15  		// 18                         // measure_vol_min(analog_gg_radio2,40238,238,35);  ������� ������� �� ������ GG Radio2
};
  
const byte  porog_dispatcher[]    PROGMEM = {

		//	+++++++++++++Test headset dispatcher ++++++++++++++++++++++++++++++
40,			// 0                          // resistor(1, 30);   ���������� ������� ������� 30 ��
40,			// 1                          // resistor(2, 30);   ���������� ������� ������� 30 ��
15,			// 2                          // measure_vol_min(analog_FrontL,   40240,240,35);  ������� ������� �� ������ FrontL 
15,			// 3                          // measure_vol_min(analog_FrontR,   40241,241,35);  ������� ������� �� ������ FrontR
15,			// 4                          // measure_vol_min(analog_LineL,    40242,242,35);  ������� ������� �� ������ LineL
15,			// 5                          // measure_vol_min(analog_LineR,    40243,243,35);  ������� ������� �� ������ LineR
15,			// 6                          // measure_vol_min(analog_mag_radio,40244,244,35);  ������� ������� �� ������ mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone,40245,245,35);  ������� ������� �� ������ mag phone
15,			// 8                          // measure_vol_min(analog_ggs,      40246,246,35);  ������� ������� �� ������ GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1,40247,247,35);  ������� ������� �� ������ GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2,40248,248,35);  ������� ������� �� ������ GG Radio2 
			// ++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
100,	   	// 11                         // measure_vol_max(analog_LineL,    40227,227,200); ������� ������� �� ������ LineL
75,	        // 12                         // measure_vol_max(analog_mag_phone,40229,229,200); ������� ������� �� ������ mag phone
15,			// 13                         // measure_vol_min(analog_FrontL,   40240,240,35);  ������� ������� �� ������ FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,   40241,241,35);  ������� ������� �� ������ FrontR 
15,			// 15                         // measure_vol_min(analog_LineR,    40243,243,35);  ������� ������� �� ������ LineR 
15,			// 16                         // measure_vol_min(analog_ggs,      40246,246,35);  ������� ������� �� ������ GGS
15,			// 17                         // measure_vol_min(analog_gg_radio1,40247,247,35);  ������� ������� �� ������ GG Radio1
15			// 18                         // measure_vol_min(analog_gg_radio2,40248,248,35);  ������� ������� �� ������ GG Radio2
};

const byte  porog_MTT[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++++ Test MTT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
200,		// 0                          // resistor(1, 130);  ���������� ������� ������� 60 ��
200,		// 1                          // resistor(2, 130);  ���������� ������� ������� 60 ��
15,			// 2                          // measure_vol_min(analog_FrontL,    40250,250,35); ������� ������� �� ������ FrontL  
15,			// 3                          // measure_vol_min(analog_FrontR,    40251,251,35); ������� ������� �� ������ FrontR 
15,			// 4                          // measure_vol_min(analog_LineL,     40252,252,35); ������� ������� �� ������ LineL
15,			// 5                          // measure_vol_min(analog_LineR,     40253,253,35); ������� ������� �� ������ LineR
15,			// 6                          // measure_vol_min(analog_mag_radio, 40254,254,35); ������� ������� �� ������ mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone, 40255,255,35); ������� ������� �� ������ mag phone
15,			// 8                          // measure_vol_min(analog_ggs,       40256,256,35); ������� ������� �� ������ GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1, 40257,257,35); ������� ������� �� ������ GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2, 40258,258,35); ������� ������� �� ������ GG Radio2 
			//++++++++++++++++++++++++++++++++++ ������ ����� �� ���� ��� ++++++++++++++++++++++++++++++++++++++++++++
15,			// 11                         // measure_vol_min(analog_FrontL,    40250,250,35); ������� ������� �� ������ FrontL
15,			// 12                         // measure_vol_min(analog_FrontR,    40251,251,35); ������� ������� �� ������ FrontR
15,			// 13                         // measure_vol_min(analog_mag_radio, 40254,254,35); ������� ������� �� ������ mag radio
15,			// 14                         // measure_vol_min(analog_ggs,       40256,256,35); ������� ������� �� ������ GGS
15,			// 15                         // measure_vol_min(analog_gg_radio1, 40257,257,35); ������� ������� �� ������ GG Radio1
15,			// 16                         // measure_vol_min(analog_gg_radio2, 40258,258,35); ������� ������� �� ������ GG Radio2 
70,	    	// 17                         // measure_vol_max(analog_LineL,     40260,260,35);  "Test MTT ** Signal LineL 
55,	        // 18                         // measure_vol_max(analog_mag_phone, 40262,262, + 18)); 
15,         // 19                         // measure_vol_min(analog_ggs,       40256,256,por_buffer[19]);  // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
120			// 20                         // measure_vol_max(analog_ggs,       40259,259,  + 20));         //  �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS             On 
};




const byte  porog_Microphone[]    PROGMEM = {
		//+++++++++++++++++++++++++++++++++ Test Microphone +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
150,		// 0                          // resistor(1, 200);  ���������� ������� ������� 60 ��
150,		// 1                          // resistor(2, 200);  ���������� ������� ������� 60 ��
15,			// 2                          // measure_vol_min(analog_FrontL,    40320,320,35); ������� ������� �� ������ FrontL
15,			// 3                          // measure_vol_min(analog_FrontR,    40321,321,35); ������� ������� �� ������ FrontR 
15,			// 4                          // measure_vol_min(analog_LineL,     40322,322,35); ������� ������� �� ������ LineL
15,			// 5                          // measure_vol_min(analog_LineR,     40323,323,35); ������� ������� �� ������ LineR
15,			// 6                          // measure_vol_min(analog_mag_radio, 40324,324,35); ������� ������� �� ������ mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone, 40325,325,35); ������� ������� �� ������ mag phone
15,			// 8                          // measure_vol_min(analog_ggs,       40326,326,35); ������� ������� �� ������ GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1, 40327,327,35); ������� ������� �� ������ GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2, 40328,328,35); ������� ������� �� ������ GG Radio2
			//++++++++++++++++++++++++++++++++++ ������ ����� �� ���� ��������� ++++++++++++++++++++++++
60,	    	// 11                         // measure_vol_max(analog_mag_phone, 40298,298,180);������� ������� �� ������ mag phone
75,		    // 12                         // measure_vol_max(analog_LineL,     40299,299,180);������� ������� �� ������ "Test Microphone ** Signal LineL 
15,			// 13                         // measure_vol_min(analog_FrontL,    40320,320,35); ������� ������� �� ������ FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,    40321,321,35); ������� ������� �� ������ FrontR
15,			// 15                         // measure_vol_min(analog_LineR,     40323,323,35); ������� ������� �� ������ LineR
15,			// 16                         // measure_vol_min(analog_mag_radio, 40324,324,35); ������� ������� �� ������ mag radio 
15,			// 17                         // measure_vol_min(analog_ggs,       40326,326,35); ������� ������� �� ������ GGS
15,			// 18                         // measure_vol_min(analog_gg_radio1, 40327,327,35); ������� ������� �� ������ GG Radio1
15          // 19                         // measure_vol_min(analog_gg_radio2, 40328,328,35); ������� ������� �� ������ GG Radio2
};

const byte  porog_GGS[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++ Test GGS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
230,		//  0                         // resistor(1, 200); ���������� ������� ������� xx ��
230,		//  1                         // resistor(2, 200); ���������� ������� ������� xx ��
				//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15, 		//  2                         // measure_vol_min(analog_FrontL,    40280,280,35); ������� ������� �� ������ "Test GGS ** Signal FrontL 
15, 		//  3                         // measure_vol_min(analog_FrontR,    40281,281,35); ������� ������� �� ������ "Test GGS ** Signal FrontR        
15, 		//  4                         // measure_vol_min(analog_LineL,     40282,282,35); ������� ������� �� ������ "Test GGS ** Signal LineL 
15, 		//  5                         // measure_vol_min(analog_LineR,     40283,283,35); ������� ������� �� ������ "Test GGS ** Signal LineR 
15, 		//  6                         // measure_vol_min(analog_mag_radio, 40284,284,35); ������� ������� �� ������ "Test GGS ** Signal mag radio
15, 		//  7                         // measure_vol_min(analog_mag_phone, 40285,285,35); ������� ������� �� ������ "Test GGS ** Signal mag phone 
15, 		//  8                         // measure_vol_min(analog_ggs,       40286,286,35); ������� ������� �� ������ "Test GGS ** Signal GGS    
15, 		//  9                         // measure_vol_min(analog_gg_radio1, 40287,287,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio1   
15, 		//  10                        // measure_vol_min(analog_gg_radio2, 40288,288,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio2
			   //++++++++++++++++++++++++++++++++++ ������ ����� �� ���� GGS ++++++++++++++++++++++++
110,    	// 11    					  //measure_vol_max(analog_FrontL,    40290,290,40); ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
130,	   	// 12  						  //measure_vol_max(analog_FrontR,    40291,291,40); ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";
15,			// 13						  //measure_vol_min(analog_LineL,     40282,282,35); ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
15,			// 14						  //measure_vol_min(analog_LineR,     40283,283,35); ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
16,			// 15	                 	  //measure_vol_max(analog_mag_radio, 40332,332,35); ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
16,	    	// 16						  //measure_vol_max(analog_mag_phone, 40292,292,50); ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
130,	    // 17			     		  //measure_vol_max(analog_ggs,       40286,289,160); ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
15,			// 18						  //measure_vol_min(analog_gg_radio1, 40287,287,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
15			// 19						  //measure_vol_min(analog_gg_radio2, 40288,288,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

};

const byte  porog_Radio1[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio1 +++++++++++++++++++++++++++++++++++++++++++++++++++++

150,		// 0							//resistor(1, 250);  ���������� ������� ������� xx ��
150,		// 1							//resistor(2, 250);  ���������� ������� ������� xx ��
				 //+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15,			// 2							//measure_vol_min(analog_FrontL,    40300,300,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
15,			// 3							//measure_vol_min(analog_FrontR,    40301,301,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
15,			// 4							//measure_vol_min(analog_LineL,     40302,302,35); ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
15,			// 5							//measure_vol_min(analog_LineR,     40303,303,35); ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
15,			// 6							//measure_vol_min(analog_mag_radio, 40304,304,35); ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
15,			// 7							//measure_vol_min(analog_mag_phone, 40305,305,35); ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
15,			// 8							//measure_vol_min(analog_ggs,       40306,306,35); ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
15,			// 9							//measure_vol_min(analog_gg_radio1, 40307,307,35); ������� ������� �� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
15,			// 10							//measure_vol_min(analog_gg_radio2, 40308,308,35); �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
				  //++++++++++++++++++++++++++++++++++ ������ ����� �� ����  Radio1 ++++++++++++++++++++++++
15,			// 11							//measure_vol_min(analog_FrontL,    40300,300,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
15,			// 12							//measure_vol_min(analog_FrontR,    40301,301,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
15,			// 13							//measure_vol_min(analog_LineL,     40302,302,35); ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
15,			// 14							//measure_vol_min(analog_LineR,     40303,303,35); ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
40,			// 15							//measure_vol_max(analog_mag_radio, 40330,330,35); ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
15,			// 16							//measure_vol_min(analog_mag_phone, 40305,305,35); ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
15,			// 17							//measure_vol_min(analog_ggs,       40306,306,35); ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
220,		// 18							//measure_vol_max(analog_gg_radio1, 40309,309,250);������� ������� �� ������ "Test Radio1 ** Signal Radio1                                ON  - ";
15			// 19							//measure_vol_min(analog_gg_radio2, 40308,308,35); ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2       
};

const byte  porog_Radio2[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio2 +++++++++++++++++++++++++++++++++++++++++++++++++++++

150,		// 0							//resistor(1, 250);  ���������� ������� ������� xx ��
150,		// 1							//resistor(2, 250);  ���������� ������� ������� xx ��
				 //+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15,			// 2							//measure_vol_min(analog_FrontL,    40310,310,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
15,			// 3							//measure_vol_min(analog_FrontR,    40311,311,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
15,			// 4							//measure_vol_min(analog_LineL,     40312,312,35); ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
15,			// 5							//measure_vol_min(analog_LineR,     40313,313,35); ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
15,			// 6 							//measure_vol_min(analog_mag_radio, 40314,314,35); ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
15,			// 7							//measure_vol_min(analog_mag_phone, 40315,315,35); ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
15,			// 8							//measure_vol_min(analog_ggs,       40316,316,35); ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
15,			// 9							//measure_vol_min(analog_gg_radio1, 40317,317,35); ������� ������� �� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
15,			// 10							//measure_vol_min(analog_gg_radio2, 40318,318,35); ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
					//++++++++++++++++++++++++++++++++++ ������ ����� �� ����  Radio2 ++++++++++++++++++++++++
15,			// 11							//measure_vol_min(analog_FrontL,    40310,310,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
15,			// 12							//measure_vol_min(analog_FrontR,    40311,311,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
15,			// 13							//measure_vol_min(analog_LineL,     40312,312,35); ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
15,			// 14							//measure_vol_min(analog_LineR,     40313,313,35); ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
40,			// 15							//measure_vol_max(analog_mag_radio, 40331,331,35); ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
15,			// 16							//measure_vol_min(analog_mag_phone, 40315,315,35); ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
15,			// 17							//measure_vol_min(analog_ggs,       40316,316,35); ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
15,			// 18							//measure_vol_min(analog_gg_radio1, 40317,317,35); ������� ������� �� ������ "Test Radio2 ** Signal Radio1                                ON  - ";
200			// 19							//measure_vol_max(analog_gg_radio2, 40319,319,250);������� ������� �� ������ "Test Radio2 ** Signal GG Radio2        
};





//---------------------------������ ���������   ---------------------------------------------------
const char  txt_message0[]    PROGMEM            = "  Error! - ";                           
const char  txt_message1[]    PROGMEM            = "Pass";      
const char  txt_message2[]    PROGMEM            = " ****** Test sensor OFF start! ******";    
const char  txt_message3[]    PROGMEM            = " ****** Test sensor ON  start! ******";      
const char  txt_message4[]    PROGMEM            = "Signal headset instructor microphone 30mv     ON"            ;   
const char  txt_message5[]    PROGMEM            = "Microphone headset instructor signal          ON"            ;  
const char  txt_message6[]    PROGMEM            = "Command sensor OFF headset instructor 2             send!"   ;    
const char  txt_message7[]    PROGMEM            = "Command sensor OFF headset instructor               send!"   ;    
const char  txt_message8[]    PROGMEM            = "Command PTT    OFF headset instructor               send!"   ;    
const char  txt_message9[]    PROGMEM            = "Command sensor OFF microphone                       send!"   ;    

const char  txt_message10[]   PROGMEM            = "Command sensor ON  headset instructor 2             send!"   ;     
const char  txt_message11[]   PROGMEM            = "Command sensor ON  headset instructor               send!"   ;     
const char  txt_message12[]   PROGMEM            = "Command        ON  PTT headset instructor (CTS)     send!"   ;  
const char  txt_message13[]   PROGMEM            = "Signal headset dispatcher microphone 30 mV    ON"            ;   
const char  txt_message14[]   PROGMEM            = "Microphone headset dispatcher signal          ON"            ;     
const char  txt_message15[]   PROGMEM            = "Command sensor OFF headset dispatcher 2             send!"   ;    
const char  txt_message16[]   PROGMEM            = "Command sensor OFF headset dispatcher               send!"   ;   
const char  txt_message17[]   PROGMEM            = "Command PTT    OFF headset dispatcher               send!"   ;    
const char  txt_message18[]   PROGMEM            = "Command sensor OFF microphone                       send!"   ;   
const char  txt_message19[]   PROGMEM            = "Command sensor ON  headset dispatcher 2             send!"   ;   

const char  txt_message20[]   PROGMEM            = "Command sensor ON  headset dispatcher               send!"   ;    
const char  txt_message21[]   PROGMEM            = "Command        ON  PTT headset dispatcher (CTS)     send!"   ;  
const char  txt_message22[]   PROGMEM            = " ****** Test headset instructor start! ******"               ; 
const char  txt_message23[]   PROGMEM            = " ****** Test headset dispatcher start! ******"               ;
const char  txt_message24[]   PROGMEM            = " ****** Test MTT start! ******"                              ;  
const char  txt_message25[]   PROGMEM            = " ****** Test tangenta nognaja start! ********"               ;  
const char  txt_message26[]   PROGMEM            = " ****** Test tangenta ruchnaja start! ********"              ; 
const char  txt_message27[]   PROGMEM            = "Command sensor OFF MTT                              send! "  ;
const char  txt_message28[]   PROGMEM            = "Command PTT    OFF MTT                              send! "  ;
const char  txt_message29[]   PROGMEM            = "Command HangUp OFF MTT                              send! "  ;

const char  txt_message30[]   PROGMEM            = "Command sensor ON  MTT                              send!"   ;
const char  txt_message31[]   PROGMEM            = "Command PTT    ON  MTT                              send!"   ;
const char  txt_message32[]   PROGMEM            = "Command HangUp ON  MTT                              send!"   ;
const char  txt_message33[]   PROGMEM            = "Signal MTT microphone 30 mV                   ON"            ;
const char  txt_message34[]   PROGMEM            = "Microphone MTT signal                         ON"            ;  
const char  txt_message35[]   PROGMEM            = "Signal FrontL, FrontR                         ON "           ;
const char  txt_message36[]   PROGMEM            = " ****** Test tangenta ruchnaja start! ******"                ;
const char  txt_message37[]   PROGMEM            = "Command sensor OFF tangenta ruchnaja                send!"   ;
const char  txt_message38[]   PROGMEM            = "Command PTT1   OFF tangenta ruchnaja                send!"   ;
const char  txt_message39[]   PROGMEM            = "Command PTT2   OFF tangenta ruchnaja                send!"   ; 

const char  txt_message40[]   PROGMEM            = "Command sensor ON  tangenta ruchnaja                send!"   ;
const char  txt_message41[]   PROGMEM            = "Command PTT1   ON  tangenta ruchnaja                send!"   ;
const char  txt_message42[]   PROGMEM            = "Command PTT2   ON  tangenta ruchnaja                send!"   ;
const char  txt_message43[]   PROGMEM            = " ****** Test tangenta nognaja start! ******"                 ;
const char  txt_message44[]   PROGMEM            = "Command sensor OFF tangenta nognaja                 send!"   ;
const char  txt_message45[]   PROGMEM            = "Command PTT    OFF tangenta nognaja                 send!"   ;
const char  txt_message46[]   PROGMEM            = "Command sensor ON  tangenta nognaja                 send!"   ;
const char  txt_message47[]   PROGMEM            = "Command PTT    ON  tangenta nognaja                 send!"   ;
const char  txt_message48[]   PROGMEM            = " ****** Test GGS start! ******"      ;
const char  txt_message49[]   PROGMEM            = "Signal GGS  FrontL, FrontR   0,7V             ON"            ;

const char  txt_message50[]   PROGMEM            = " ****** Test Radio1 start! ******"                           ;
const char  txt_message51[]   PROGMEM            = "Signal Radio1 300 mV    LFE                   ON"            ;
const char  txt_message52[]   PROGMEM            = " ****** Test Radio2 start! ******"      ;
const char  txt_message53[]   PROGMEM            = "Signal Radio1 300 mV    Center                ON"            ;
const char  txt_message54[]   PROGMEM            = " ****** Test microphone start! ******"                       ;
const char  txt_message55[]   PROGMEM            = "Signal microphone 30  mV                      ON"            ;
const char  txt_message56[]   PROGMEM            = "Command PTT    OFF microphone                       send!"   ;
const char  txt_message57[]   PROGMEM            = "Command PTT    ON  microphone                       send!"   ;
const char  txt_message58[]   PROGMEM            = "Command sensor OFF microphone                       send!"   ;  
const char  txt_message59[]   PROGMEM            = "Command sensor ON  microphone                       send!"   ;

const char  txt_message60[]   PROGMEM            = "Power amperage                              mA - "   ;
const char  txt_message61[]   PROGMEM            = "Power module Audio 2                           - "   ;
const char  txt_message62[]   PROGMEM            = "Power amperage                              mA - "   ;
const char  txt_message63[]   PROGMEM            = "Power Radio1                                   - "   ;
const char  txt_message64[]   PROGMEM            = "Power Radio2                                   - "   ;
const char  txt_message65[]   PROGMEM            = "Power GGS                                      - "   ;
const char  txt_message66[]   PROGMEM            = "Power Led microphone                           -  "  ;
const char  txt_message67[]   PROGMEM            = " ****** Test power start! ******"                            ;
const char  txt_message68[]   PROGMEM            = " ****** Test Adjusting the brightness of the display! ******"; 
const char  txt_message69[]   PROGMEM            = "Adjusting the brightness code                              - "   ;

const char  txt_message70[]   PROGMEM            = "Adjusting the brightness mks                               - "   ;
const char  txt_message71[]   PROGMEM            = "Signal GGS  FrontL, FrontR   0,7V             OFF"            ;



//++++++++++++++++++++++++++++++ ������ ������ ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char  txt_error0[]  PROGMEM              = "Sensor MTT                          XP1- 19 HaSs    OFF - ";
const char  txt_error1[]  PROGMEM              = "Sensor tangenta ruchnaja            XP7 - 2         OFF - ";
const char  txt_error2[]  PROGMEM              = "Sensor tangenta nognaja             XP8 - 2         OFF - "; 
const char  txt_error3[]  PROGMEM              = "Sensor headset instructor 2         XP1- 16 HeS2Rs  OFF - ";
const char  txt_error4[]  PROGMEM              = "Sensor headset instructor           XP1- 13 HeS2Ls  OFF - "; 
const char  txt_error5[]  PROGMEM              = "Sensor headset dispatcher 2         XP1- 5  HeS1Rs  OFF - "; 
const char  txt_error6[]  PROGMEM              = "Sensor headset dispatcher           XP1- 1  HeS1Ls  OFF - ";
const char  txt_error7[]  PROGMEM              = "Sensor microphone  (bit 5, byte 3)  XS1 - 6         OFF - "; 
const char  txt_error8[]  PROGMEM              = "Microphone headset instructor Sw.   XP1 12 HeS2e    OFF - "; 
const char  txt_error9[]  PROGMEM              = "Microphone headset dispatcher Sw.   XP1 12 HeS2e    OFF - ";  

const char  txt_error10[]  PROGMEM             = "Sensor MTT                          XP1- 19 HaSs    ON~ - "; 
const char  txt_error11[]  PROGMEM             = "Sensor tangenta ruchnaja            XP7 - 2         ON~ - ";
const char  txt_error12[]  PROGMEM             = "Sensor tangenta nognaja             XP8 - 2         ON~ - "; 
const char  txt_error13[]  PROGMEM             = "Sensor headset instructor 2         XP1- 16 HeS2Rs  ON~ - ";
const char  txt_error14[]  PROGMEM             = "Sensor headset instructor           XP1- 13 HeS2Ls  ON~ - "; 
const char  txt_error15[]  PROGMEM             = "Sensor headset dispatcher 2         XP1- 5  HeS1Rs  ON~ - "; 
const char  txt_error16[]  PROGMEM             = "Sensor headset dispatcher           XP1- 1  HeS1Ls  ON~ - ";
const char  txt_error17[]  PROGMEM             = "Sensor microphone (bit 5, byte 3)   XS1 - 6         ON~ - "; 
const char  txt_error18[]  PROGMEM             = "Microphone headset instructor Sw.   XP1 12 HeS2e    ON~ - "; 
const char  txt_error19[]  PROGMEM             = "Microphone headset dispatcher Sw.   XP1 10 HeS1e    ON~ - "; 
  
const char  txt_error20[]  PROGMEM             = "Command PTT headset instructor (CTS)                OFF - ";
const char  txt_error21[]  PROGMEM             = "Command PTT headset instructor (CTS)                ON~ - ";
const char  txt_error22[]  PROGMEM             = "Command PTT headset dispatcher (CTS)                OFF - ";
const char  txt_error23[]  PROGMEM             = "Command PTT headset dispatcher (CTS)                ON~ - ";
const char  txt_error24[]  PROGMEM             = "Test headset instructor ** Signal LineL             ON~ - ";
const char  txt_error25[]  PROGMEM             = "Test headset instructor ** Signal LineR             ON~ - ";   
const char  txt_error26[]  PROGMEM             = "Test headset instructor ** Signal Mag phone         ON~ - ";
const char  txt_error27[]  PROGMEM             = "Test headset dispatcher ** Signal LineL             ON~ - ";
const char  txt_error28[]  PROGMEM             = "Test headset dispatcher ** Signal LineR             ON~ - ";  
const char  txt_error29[]  PROGMEM             = "Test headset dispatcher ** Signal Mag phone         ON~ - ";

const char  txt_error30[]  PROGMEM             = "Test headset instructor ** Signal FrontL            OFF - ";
const char  txt_error31[]  PROGMEM             = "Test headset instructor ** Signal FrontR            OFF - ";
const char  txt_error32[]  PROGMEM             = "Test headset instructor ** Signal LineL             OFF - ";
const char  txt_error33[]  PROGMEM             = "Test headset instructor ** Signal LineR             OFF - ";
const char  txt_error34[]  PROGMEM             = "Test headset instructor ** Signal mag radio         OFF - ";
const char  txt_error35[]  PROGMEM             = "Test headset instructor ** Signal mag phone         OFF - ";
const char  txt_error36[]  PROGMEM             = "Test headset instructor ** Signal GGS               OFF - ";
const char  txt_error37[]  PROGMEM             = "Test headset instructor ** Signal GG Radio1         OFF - ";
const char  txt_error38[]  PROGMEM             = "Test headset instructor ** Signal GG Radio2         OFF - ";
const char  txt_error39[]  PROGMEM             = "";

const char  txt_error40[]  PROGMEM             = "Test headset dispatcher ** Signal FrontL            OFF - ";
const char  txt_error41[]  PROGMEM             = "Test headset dispatcher ** Signal FrontR            OFF - ";
const char  txt_error42[]  PROGMEM             = "Test headset dispatcher ** Signal LineL             OFF - ";
const char  txt_error43[]  PROGMEM             = "Test headset dispatcher ** Signal LineR             OFF - ";
const char  txt_error44[]  PROGMEM             = "Test headset dispatcher ** Signal mag radio         OFF - ";
const char  txt_error45[]  PROGMEM             = "Test headset dispatcher ** Signal mag phone         OFF - ";
const char  txt_error46[]  PROGMEM             = "Test headset dispatcher ** Signal GGS               OFF - ";
const char  txt_error47[]  PROGMEM             = "Test headset dispatcher ** Signal GG Radio1         OFF - ";
const char  txt_error48[]  PROGMEM             = "Test headset dispatcher ** Signal GG Radio2         OFF - ";
const char  txt_error49[]  PROGMEM             = "";

const char  txt_error50[]  PROGMEM             = "Test MTT ** Signal FrontL                           OFF - ";
const char  txt_error51[]  PROGMEM             = "Test MTT ** Signal FrontR                           OFF - ";
const char  txt_error52[]  PROGMEM             = "Test MTT ** Signal LineL                            OFF - ";
const char  txt_error53[]  PROGMEM             = "Test MTT ** Signal LineR                            OFF - ";
const char  txt_error54[]  PROGMEM             = "Test MTT ** Signal mag radio                        OFF - ";
const char  txt_error55[]  PROGMEM             = "Test MTT ** Signal mag phone                        OFF - ";
const char  txt_error56[]  PROGMEM             = "Test MTT ** Signal GGS                              OFF - ";
const char  txt_error57[]  PROGMEM             = "Test MTT ** Signal GG Radio1                        OFF - ";
const char  txt_error58[]  PROGMEM             = "Test MTT ** Signal GG Radio2                        OFF - ";
const char  txt_error59[]  PROGMEM             = "Test MTT ** Signal GGS                              ON~ - ";

const char  txt_error60[]  PROGMEM             = "Test MTT ** Signal LineL                            ON~ - ";
const char  txt_error61[]  PROGMEM             = "Test MTT ** Signal LineR                            ON~ - ";  
const char  txt_error62[]  PROGMEM             = "Test MTT ** Signal Mag phone                        ON~ - ";
const char  txt_error63[]  PROGMEM             = "Test MTT PTT    (CTS)                               OFF - ";
const char  txt_error64[]  PROGMEM             = "Test microphone PTT  (CTS)                          OFF - ";
const char  txt_error65[]  PROGMEM             = "Test MTT PTT    (CTS)                               ON~ - ";
const char  txt_error66[]  PROGMEM             = "Test microphone PTT  (CTS)                          ON~ - ";
const char  txt_error67[]  PROGMEM             = "Test MTT HangUp (DCD)                               OFF - ";
const char  txt_error68[]  PROGMEM             = "Test MTT HangUp (DCD)                               ON~ - ";
const char  txt_error69[]  PROGMEM             = "";

const char  txt_error70[]  PROGMEM             = "Command PTT1 tangenta ruchnaja (CTS)                OFF - ";
const char  txt_error71[]  PROGMEM             = "Command PTT2 tangenta ruchnaja (DCR)                OFF - ";
const char  txt_error72[]  PROGMEM             = "Command PTT1 tangenta ruchnaja (CTS)                ON~ - ";
const char  txt_error73[]  PROGMEM             = "Command PTT2 tangenta ruchnaja (DCR)                ON~ - ";
const char  txt_error74[]  PROGMEM             = "Command sensor tangenta ruchnaja    XP7 - 2         OFF - ";
const char  txt_error75[]  PROGMEM             = "Command sensor tangenta ruchnaja    XP7 - 2         ON~ - ";
const char  txt_error76[]  PROGMEM             = "Command sensor tangenta nognaja     XP8 - 2         OFF - ";
const char  txt_error77[]  PROGMEM             = "Command sensor tangenta nognaja     XP8 - 2         ON~ - ";
const char  txt_error78[]  PROGMEM             = "Command PTT tangenta nognaja (CTS)  XP8 - 1         OFF - ";
const char  txt_error79[]  PROGMEM             = "Command PTT tangenta nognaja (CTS)  XP8 - 1         ON~ - ";

const char  txt_error80[]  PROGMEM             = "Test GGS ** Signal FrontL                           OFF - ";
const char  txt_error81[]  PROGMEM             = "Test GGS ** Signal FrontR                           OFF - ";
const char  txt_error82[]  PROGMEM             = "Test GGS ** Signal LineL                            OFF - ";
const char  txt_error83[]  PROGMEM             = "Test GGS ** Signal LineR                            OFF - ";
const char  txt_error84[]  PROGMEM             = "Test GGS ** Signal mag radio                        OFF - ";
const char  txt_error85[]  PROGMEM             = "Test GGS ** Signal mag phone                        OFF - ";
const char  txt_error86[]  PROGMEM             = "Test GGS ** Signal GGS                              OFF - ";
const char  txt_error87[]  PROGMEM             = "Test GGS ** Signal GG Radio1                        OFF - ";
const char  txt_error88[]  PROGMEM             = "Test GGS ** Signal GG Radio2                        OFF - ";
const char  txt_error89[]  PROGMEM             = "Test GGS ** Signal GGS                              ON~ - ";

const char  txt_error90[]  PROGMEM             = "Test GGS ** Signal FrontL                           ON~ - ";
const char  txt_error91[]  PROGMEM             = "Test GGS ** Signal FrontR                           ON~ - ";
const char  txt_error92[]  PROGMEM             = "Test GGS ** Signal mag phone                        ON~ - ";
const char  txt_error93[]  PROGMEM             = "";  
const char  txt_error94[]  PROGMEM             = "";
const char  txt_error95[]  PROGMEM             = ""; 
const char  txt_error96[]  PROGMEM             = "";    
const char  txt_error97[]  PROGMEM             = "";  
const char  txt_error98[]  PROGMEM             = "Test Microphone ** Signal mag phone                 ON~ - ";  
const char  txt_error99[]  PROGMEM             = "Test Microphone ** Signal LineL                     ON~ - "; 

const char  txt_error100[]  PROGMEM            = "Test Radio1 ** Signal FrontL                        OFF - ";
const char  txt_error101[]  PROGMEM            = "Test Radio1 ** Signal FrontR                        OFF - ";
const char  txt_error102[]  PROGMEM            = "Test Radio1 ** Signal LineL                         OFF - ";
const char  txt_error103[]  PROGMEM            = "Test Radio1 ** Signal LineR                         OFF - ";
const char  txt_error104[]  PROGMEM            = "Test Radio1 ** Signal mag radio                     OFF - ";
const char  txt_error105[]  PROGMEM            = "Test Radio1 ** Signal mag phone                     OFF - ";
const char  txt_error106[]  PROGMEM            = "Test Radio1 ** Signal GGS                           OFF - ";
const char  txt_error107[]  PROGMEM            = "Test Radio1 ** Signal GG Radio1                     OFF - ";
const char  txt_error108[]  PROGMEM            = "Test Radio1 ** Signal GG Radio2                     OFF - ";
const char  txt_error109[]  PROGMEM            = "Test Radio1 ** Signal Radio1                        ON~ - ";

const char  txt_error110[]  PROGMEM            = "Test Radio2 ** Signal FrontL                        OFF - ";
const char  txt_error111[]  PROGMEM            = "Test Radio2 ** Signal FrontR                        OFF - ";
const char  txt_error112[]  PROGMEM            = "Test Radio2 ** Signal LineL                         OFF - ";
const char  txt_error113[]  PROGMEM            = "Test Radio2 ** Signal LineR                         OFF - ";
const char  txt_error114[]  PROGMEM            = "Test Radio2 ** Signal mag radio                     OFF - ";
const char  txt_error115[]  PROGMEM            = "Test Radio2 ** Signal mag phone                     OFF - ";
const char  txt_error116[]  PROGMEM            = "Test Radio2 ** Signal GGS                           OFF - ";
const char  txt_error117[]  PROGMEM            = "Test Radio2 ** Signal GG Radio1                     OFF - ";
const char  txt_error118[]  PROGMEM            = "Test Radio2 ** Signal GG Radio2                     OFF - ";
const char  txt_error119[]  PROGMEM            = "Test Radio2 ** Signal Radio2                        ON~ - ";

const char  txt_error120[]  PROGMEM            = "Test Microphone ** Signal FrontL                    OFF - ";
const char  txt_error121[]  PROGMEM            = "Test Microphone ** Signal FrontR                    OFF - ";
const char  txt_error122[]  PROGMEM            = "Test Microphone ** Signal LineL                     OFF - ";
const char  txt_error123[]  PROGMEM            = "Test Microphone ** Signal LineR                     OFF - ";
const char  txt_error124[]  PROGMEM            = "Test Microphone ** Signal mag radio                 OFF - ";
const char  txt_error125[]  PROGMEM            = "Test Microphone ** Signal mag phone                 OFF - ";
const char  txt_error126[]  PROGMEM            = "Test Microphone ** Signal GGS                       OFF - ";
const char  txt_error127[]  PROGMEM            = "Test Microphone ** Signal GG Radio1                 OFF - ";
const char  txt_error128[]  PROGMEM            = "Test Microphone ** Signal GG Radio2                 OFF - ";
const char  txt_error129[]  PROGMEM            = "";
const char  txt_error130[]  PROGMEM            = "Test Radio1 ** Signal mag radio                     ON~ - ";
const char  txt_error131[]  PROGMEM            = "Test Radio2 ** Signal mag radio                     ON~ - ";
const char  txt_error132[]  PROGMEM            = "Test GGS ** Signal mag radio                        ON~ - ";

char buffer[130];  

const char* const table_message[] PROGMEM = 
{
txt_message0,                                 // "    Error! - ";                           
txt_message1,                                 // "Pass";      
txt_message2,                                 // " ****** Test sensor OFF start! ******";    
txt_message3,                                 // " ****** Test sensor ON  start! ******";      
txt_message4,                                 // "Signal headset instructor microphone 30mv     ON"            ;   
txt_message5,                                 // "Microphone headset instructor signal          ON"            ;  
txt_message6,                                 // "Command sensor OFF headset instructor 2          send!"      ; 
txt_message7,                                 // "Command sensor OFF headset instructor            send!"      ;  
txt_message8,                                 // "Command PTT    OFF headset instructor            send!"      ;    
txt_message9,                                 // "Command sensor OFF microphone                    send!"      ;  
txt_message10,                                // "Command sensor ON  headset instructor 2          send!"      ;    
txt_message11,                                // "Command sensor ON  headset instructor            send!"      ;   
txt_message12,                                // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
txt_message13,                                // "Signal headset dispatcher microphone 30mv     ON"            ;   
txt_message14,                                // "Microphone headset dispatcher signal          ON"            ;     
txt_message15,                                // "Command sensor OFF headset dispatcher 2          send!"      ;  
txt_message16,                                // "Command sensor OFF headset dispatcher            send!"      ;  
txt_message17,                                // "Command PTT    OFF headset dispatcher            send!"      ;    
txt_message18,                                // "Command sensor OFF microphone                    send!"      ;   
txt_message19,                                // "Command sensor ON  headset dispatcher 2          send!"      ;  
txt_message20,                                // "Command sensor ON  headset dispatcher            send!"      ;   
txt_message21,                                // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
txt_message22,                                // " ****** Test headset instructor start! ******"               ; 
txt_message23,                                // " ****** Test headset dispatcher start! ******"               ;
txt_message24,                                // " ****** Test MTT start! ******"                              ;  
txt_message25,                                // " ****** Test tangenta nognaja start! ********"               ;  
txt_message26,                                // " ****** Test tangenta ruchnaja start! ********"              ; 
txt_message27,                                // "Command sensor OFF MTT                           send! "     ;
txt_message28,                                // "Command PTT    OFF MTT                           send! "     ;
txt_message29,                                // "Command HangUp OFF MTT                           send! "     ;
txt_message30,                                // "Command sensor ON  MTT                           send!"      ;
txt_message31,                                // "Command PTT    ON  MTT                           send!"      ;
txt_message32,                                // "Command HangUp ON  MTT                           send!"      ;
txt_message33,                                // "Signal MTT microphone 30mv                    ON"            ;
txt_message34,                                // "Microphone MTT signal                         ON"            ;  
txt_message35,                                // "Signal FrontL, FrontR                         ON "           ;
txt_message36,                                // " ****** Test tangenta ruchnaja start! ******"                ;
txt_message37,                                // "Command sensor OFF tangenta ruchnaja             send!"      ;
txt_message38,                                // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
txt_message39,                                // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
txt_message40,                                // "Command sensor ON  tangenta ruchnaja             send!"      ;
txt_message41,                                // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
txt_message42,                                // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
txt_message43,                                // " ****** Test tangenta nognaja start! ******"                 ;
txt_message44,                                // "Command sensor OFF tangenta nognaja              send!"      ;
txt_message45,                                // "Command PTT    OFF tangenta nognaja              send!"      ;
txt_message46,                                // "Command sensor ON  tangenta nognaja              send!"      ;
txt_message47,                                // "Command PTT    ON  tangenta nognaja              send!"      ;
txt_message48,                                // " ****** Test GGS start! ******"      ;
txt_message49,                                // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;

txt_message50,                                // " ****** Test Radio1 start! ******"      ;
txt_message51,                                // "Signal Radio1 200 mV    LFE                   ON"            ;
txt_message52,                                //" ****** Test Radio2 start! ******"      ;
txt_message53,                                // "Signal Radio1 300 mV    Center                ON"            ;
txt_message54,                                // " ****** Test mi�rophone start! ******"                       ;
txt_message55,                                // "Signal mi�rophone 30  mV                      ON"            ;
txt_message56,                                // "Command PTT    OFF microphone                    send!"      ;
txt_message57,                                // "Command PTT    ON  microphone                    send!"      ;
txt_message58,                                // "Command sensor OFF microphone                    send!"      ;  
txt_message59,                                // "Command sensor ON  microphone                    send!"      ;

txt_message60,                                // "Power amperage mA - "                                        ;
txt_message61,                                // "Power Kamerton V  - "                                        ;
txt_message62,                                // "Power amperage mA - "                                        ;
txt_message63,                                // "Power Radio1 V    - "                                        ;
txt_message64,                                // "Power Radio2 V    - "                                        ;
txt_message65,                                // "Power GGS    V    - "                                        ;
txt_message66,                                // "Power Led mic.V   - "                                        ;
txt_message67,                                // " ****** Test power start! ******"                            ;
txt_message68,                                // " ****** Test Adjusting the brightness of the display! ******"; 
txt_message69,                                // "Adjusting the brightness code                              - "   ;

txt_message70,                                // "Adjusting the brightness mks                               - "   ;
txt_message71                                 //  "Signal GGS  FrontL, FrontR   0,7V             OFF"            ;

};

const char* const string_table_err[] PROGMEM = 
{
txt_error0,                                   // "Sensor MTT                          XP1- 19 HaSs            OFF - ";
txt_error1,                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
txt_error2,                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
txt_error3,                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
txt_error4,                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
txt_error5,                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
txt_error6,                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
txt_error7,                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
txt_error8,                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
txt_error9,                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - "; 

txt_error10,                                  // "Sensor MTT                          XP1- 19 HaSs            ON  - ";
txt_error11,                                  // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
txt_error12,                                  // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
txt_error13,                                  // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
txt_error14,                                  // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
txt_error15,                                  // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
txt_error16,                                  // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
txt_error17,                                  // "Sensor microphone                   XS1 - 6                 ON  - "; 
txt_error18,                                  // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
txt_error19,                                  // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 

txt_error20,                                  // "Command PTT headset instructor (CTS)                        OFF - ";
txt_error21,                                  // "Command PTT headset instructor (CTS)                        ON  - ";
txt_error22,                                  // "Command PTT headset dispatcher (CTS)                        OFF - ";
txt_error23,                                  // "Command PTT headset dispatcher (CTS)                        ON  - ";
txt_error24,                                  // "Test headset instructor ** Signal LineL                     ON  - ";
txt_error25,                                  // "Test headset instructor ** Signal LineR                     ON  - ";   
txt_error26,                                  // "Test headset instructor ** Signal Mag phone                 ON  - ";
txt_error27,                                  // "Test headset dispatcher ** Signal LineL                     ON  - ";
txt_error28,                                  // "Test headset dispatcher ** Signal LineR                     ON  - ";  
txt_error29,                                  // "Test headset dispatcher ** Signal Mag phone                 ON  - ";

txt_error30,                                  // "Test headset instructor ** Signal FrontL                    OFF - ";
txt_error31,                                  // "Test headset instructor ** Signal FrontR                    OFF - ";
txt_error32,                                  // "Test headset instructor ** Signal LineL                     OFF - ";
txt_error33,                                  // "Test headset instructor ** Signal LineR                     OFF - ";
txt_error34,                                  // "Test headset instructor ** Signal mag radio                 OFF - "; 
txt_error35,                                  // "Test headset instructor ** Signal mag phone                 OFF - ";
txt_error36,                                  // "Test headset instructor ** Signal GGS                       OFF - ";
txt_error37,                                  // "Test headset instructor ** Signal GG Radio1                 OFF - ";
txt_error38,                                  // "Test headset instructor ** Signal GG Radio2                 OFF - ";
txt_error39,                                  //

txt_error40,                                  // "Test headset dispatcher ** Signal FrontL                    OFF - ";
txt_error41,                                  // "Test headset dispatcher ** Signal FrontR                    OFF - ";
txt_error42,                                  // "Test headset dispatcher ** Signal LineL                     OFF - "; 
txt_error43,                                  // "Test headset dispatcher ** Signal LineR                     OFF - ";
txt_error44,                                  // "Test headset dispatcher ** Signal mag radio                 OFF - "; 
txt_error45,                                  // "Test headset dispatcher ** Signal mag phone                 OFF - ";
txt_error46,                                  // "Test headset dispatcher ** Signal GGS                       OFF - "; 
txt_error47,                                  // "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
txt_error48,                                  // "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
txt_error49,                                  //  

txt_error50,                                  // "Test MTT ** Signal FrontL                                   OFF - ";
txt_error51,                                  // "Test MTT ** Signal FrontR                                   OFF - ";
txt_error52,                                  // "Test MTT ** Signal LineL                                    OFF - ";
txt_error53,                                  // "Test MTT ** Signal LineR                                    OFF - "; 
txt_error54,                                  // "Test MTT ** Signal mag radio                                OFF - ";
txt_error55,                                  // "Test MTT ** Signal mag phone                                OFF - ";
txt_error56,                                  // "Test MTT ** Signal GGS                                      OFF - ";
txt_error57,                                  // "Test MTT ** Signal GG Radio1                                OFF - ";
txt_error58,                                  // "Test MTT ** Signal GG Radio2                                OFF - "; 
txt_error59,                                  // "Test MTT ** Signal GGS                                      ON  - ";

txt_error60,                                  // "Test MTT ** Signal LineL                                    ON  - ";
txt_error61,                                  // "Test MTT ** Signal LineR                                    ON  - ";  
txt_error62,                                  // "Test MTT ** Signal Mag phone                                ON  - ";
txt_error63,                                  // "Test MTT PTT    (CTS)                                       OFF - ";
txt_error64,                                  // "Test microphone PTT  (CTS)                                  OFF - ";
txt_error65,                                  // "Test MTT PTT    (CTS)                                       ON  - ";
txt_error66,                                  // "Test microphone PTT  (CTS)                                  ON  - ";
txt_error67,                                  // "Test MTT HangUp (DCD)                                       OFF - ";
txt_error68,                                  // "Test MTT HangUp (DCD)                                       ON  - ";
txt_error69,                                  //

txt_error70,                                  // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
txt_error71,                                  // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
txt_error72,                                  // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
txt_error73,                                  // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
txt_error74,                                  // "Command sensor tangenta ruchnaja                            OFF - ";
txt_error75,                                  // "Command sensor tangenta ruchnaja                            ON  - ";
txt_error76,                                  // "Command sensor tangenta nognaja     XP8 - 2                 OFF - ";
txt_error77,                                  // "Command sensor tangenta nognaja     XP8 - 2                 ON  - ";
txt_error78,                                  // "Command PTT tangenta nognaja (CTS)  XP8 - 1                 OFF - ";  
txt_error79,                                  // "Command PTT tangenta nognaja (CTS)  XP8 - 1                 ON  - "; 

txt_error80,                                  // "Test GGS ** Signal FrontL                                   OFF - ";
txt_error81,                                  // "Test GGS ** Signal FrontR                                   OFF - ";
txt_error82,                                  // "Test GGS ** Signal LineL                                    OFF - ";
txt_error83,                                  // "Test GGS ** Signal LineR                                    OFF - ";
txt_error84,                                  // "Test GGS ** Signal mag radio                                OFF - ";
txt_error85,                                  // "Test GGS ** Signal mag phone                                OFF - ";
txt_error86,                                  // "Test GGS ** Signal GGS                                      OFF - ";
txt_error87,                                  // "Test GGS ** Signal GG Radio1                                OFF - ";
txt_error88,                                  // "Test GGS ** Signal GG Radio2                                OFF - ";
txt_error89,                                  // "Test GGS ** Signal GGS                                      ON  - ";

txt_error90,                                  // "Test GGS ** Signal FrontL                                   ON  - ";
txt_error91,                                  // "Test GGS ** Signal FrontR                                   ON  - ";
txt_error92,                                  // "Test GGS ** Signal mag phone                                ON  - ";
txt_error93,                                  // 
txt_error94,                                  // 
txt_error95,                                  // 
txt_error96,                                  // 
txt_error97,                                  // 
txt_error98,                                  // "Test Microphone ** Signal mag phone                         ON  - ";   
txt_error99,                                  // "Test Microphone ** Signal LineL                             ON  - "; 

txt_error100,                                 // "Test Radio1 ** Signal FrontL                                OFF - ";
txt_error101,                                 // "Test Radio1 ** Signal FrontR                                OFF - ";
txt_error102,                                 // "Test Radio1 ** Signal LineL                                 OFF - ";
txt_error103,                                 // "Test Radio1 ** Signal LineR                                 OFF - ";
txt_error104,                                 // "Test Radio1 ** Signal mag radio                             OFF - ";
txt_error105,                                 // "Test Radio1 ** Signal mag phone                             OFF - ";
txt_error106,                                 // "Test Radio1 ** Signal GGS                                   OFF - ";
txt_error107,                                 // "Test Radio1 ** Signal GG Radio1                             OFF - ";
txt_error108,                                 // "Test Radio1 ** Signal GG Radio2                             OFF - ";
txt_error109,                                 // "Test Radio1 ** Signal Radio1                                ON  - ";

txt_error110,                                 // "Test Radio2 ** Signal FrontL                                OFF - ";
txt_error111,                                 // "Test Radio2 ** Signal FrontR                                OFF - ";
txt_error112,                                 // "Test Radio2 ** Signal LineL                                 OFF - ";
txt_error113,                                 // "Test Radio2 ** Signal LineR                                 OFF - ";
txt_error114,                                 // "Test Radio2 ** Signal mag radio                             OFF - ";
txt_error115,                                 // "Test Radio2 ** Signal mag phone                             OFF - ";
txt_error116,                                 // "Test Radio2 ** Signal GGS                                   OFF - ";
txt_error117,                                 // "Test Radio2 ** Signal GG Radio1                             OFF - ";
txt_error118,                                 // "Test Radio2 ** Signal GG Radio2                             OFF - ";
txt_error119,                                 // "Test Radio2 ** Signal Radio2                                ON  - ";

txt_error120,                                 // "Test Microphone ** Signal FrontL                            OFF - ";
txt_error121,                                 // "Test Microphone ** Signal FrontR                            OFF - ";
txt_error122,                                 // "Test Microphone ** Signal LineL                             OFF - ";
txt_error123,                                 // "Test Microphone ** Signal LineR                             OFF - ";
txt_error124,                                 // "Test Microphone ** Signal mag radio                         OFF - ";
txt_error125,                                 // "Test Microphone ** Signal mag phone                         OFF - ";
txt_error126,                                 // "Test Microphone ** Signal GGS                               OFF - ";
txt_error127,                                 // "Test Microphone ** Signal GG Radio1                         OFF - ";
txt_error128,                                 // "Test Microphone ** Signal GG Radio2                         OFF - ";
txt_error129,                                 // ";
txt_error130,                                 // "Test Radio1 ** Signal mag radio                             ON  - ";
txt_error131,                                 // "Test Radio2 ** Signal mag radio                             ON  - ";
txt_error132                                  // "Test GGS    ** Signal mag radio                             ON  - ";
};














// ========================= ���� �������� ============================================
void dateTime(uint16_t* date, uint16_t* time)                  // ��������� ������ ������� � ���� �����
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void serial_print_date()                           // ������ ���� � �������    
{
	  DateTime now = RTC.now();
	  Serial.print(now.day(), DEC);
	  Serial.print('/');
	  Serial.print(now.month(), DEC);
	  Serial.print('/');
	  Serial.print(now.year(), DEC);//Serial display time
	  Serial.print(' ');
	  Serial.print(now.hour(), DEC);
	  Serial.print(':');
	  Serial.print(now.minute(), DEC);
	  Serial.print(':');
	  Serial.print(now.second(), DEC);
}


const int8_t ERROR_LED_PIN = 13;



void set_time()
{
	RTC.adjust(DateTime(__DATE__, __TIME__));
//	DateTime now = RTC.now();
//	second = now.second();       //Initialization time
//	minute = now.minute();
//	hour   = now.hour();
//	day    =  now.day();
//	day++;
//	if(day > 31)day = 1;
//	month  = now.month();
//	year   = now.year();
//	DateTime set_time = DateTime(year, month, day, hour, minute, second); // ������� ������ � ������� � ������ "set_time"
//	RTC.adjust(set_time);             
}

void flash_time()                                              // ��������� ���������� ���������� 
{ 
		//prer_Kmerton_Run = true;
		////	digitalWrite(ledPin13,HIGH);
		//prer_Kamerton();
		//// mb.task();
	    //	digitalWrite(ledPin13,LOW);
		//prer_Kmerton_Run = false;
}
void prer_Kamerton()                                          // ���������� ����� ���������� � ������� ��������
{
//	clear_serial1();
	sendPacketK ();  
	// ��������� ���������� � ������ ��������
	waiting_for_replyK();                                  // �������� �������������
}
void sendPacketK () 
{              // ��������� �������� ������ � ��������
	calculateCRC_Out();
	for (int i = 0; i <3; i++)
		{
			Serial1.write(regs_out[i]);
			mb.addHreg(1+i,regs_out[i]);
		}
}
void waiting_for_replyK()                                  // ������ ������ �� ���������
{
//	delayMicroseconds(5);
	//blink_red = !blink_red;
	//digitalWrite(ledPin13,!digitalRead(ledPin13));
	//  �������� �������� � ���������� Stop_Kam = 0; 
	if (Serial1.available())                               // ���� ���-�� ���������? ���� ������ � ������?
		  {
			unsigned char overflowFlag = 0 ;               // ���� ���������� ������� ������
			unsigned char buffer = 0;                      // ���������� � ������ ������ ������

			while (Serial1.available())
				{
				  if (overflowFlag)                        // ���� ����� ���������� - ��������
					 Serial1.read();
				  else                                     // ������ ������ � �����, ������� ����������
					{
					if (bufferK == BUFFER_SIZEK)           // ��������� ������ ������
						{
							overflowFlag = 1;              // ���������� ���� ���������� ������� ������
						}
						 mb.addHreg(4+buffer,Serial1.read());
						buffer++;
					}
				}
//			calculateCRC_In();
			mb.Coil(124,1);                               // ����� � "��������" �����������
		   }
	 else 
		{
			Stop_Kam = 0;                                 // ���� ��������. ���. �� ���������
			mb.Coil(124,0);                               // ���� ������  ����� � "��������"
		}

	  //if( regBank.get(40007) != regs_temp)
	  //{
		 //Serial.println(regBank.get(40004),BIN);
		 //Serial.println(regBank.get(40005),BIN);
		 //Serial.println(regBank.get(40006),BIN);
		 //Serial.println(regBank.get(40007),BIN);
	  //}
   //   regs_temp = regBank.get(40007);

}
void Stop_Kamerton ()                  //���� �� �������� ���������� � ��������� - �������� ��������
  {
	 for (unsigned char i = 0; i <4; i++)
	 mb.Hreg(4+i,0);
  }

void calculateCRC_Out()                // ���������� ����������� ����� ������ �����
{ 
  byte temp1, temp2, temp3, temp4, crc;
  temp1 = regs_out[1];                 // ��������  
  temp1 = temp1&0xF0;                  // �������� ����� F0 �� ������� ���� 1 �����
  temp2 = temp1>>4;                    // ����������� ������� ���� � �������
  temp3 = regs_out[2];                 // ��������
  temp3 = temp3&0xF0;                  // �������� ����� F0 �� ������� ���� 2 �����
  temp3 = temp3>>4;                    // ����������� ������� ���� � �������
  temp4 = regs_out[2];                 // ��������
  temp4 = temp4&0x0F;                  // �������� ����� F0 �� ������� ���� 2 �����
  crc =  temp2 ^  temp3 ^  temp4  ;
  crc = crc&0x0F;                      // �������� ����� F0 �� ������� ���� 2 �����
  regs_out[1]= temp1 | crc;
}
void calculateCRC_In()                 // ���������� ����������� ����� ������ �����
{ 
	/*
  byte temp1,temp1H,temp1L, temp2,temp2H,temp2L, temp3,temp3H,temp3L, temp4, temp4H, crc_in;

  temp1 = regs_in[0];                  // ��������  
  temp1 = temp1&0xF0;                  // �������� ����� F0 �� ������� ���� 1 �����
  temp1H = temp1>>4;                   // ����������� ������� ���� � �������
  temp1 = regs_in[0];                  // �������� 
  temp1L = temp1&0x0F;                 // �������� ����� 0F �� ������� ���� 1 �����

  temp2 = regs_in[1];                  // ��������  
  temp2 = temp2&0xF0;                  // �������� ����� F0 �� ������� ���� 2 �����
  temp2H = temp2>>4;                   // ����������� ������� ���� � �������
  temp2 = regs_in[1];                  // �������� 
  temp2L = temp2&0x0F;                 // �������� ����� 0F �� ������� ���� 2 �����

  temp3 = regs_in[2];                  // ��������  
  temp3 = temp3&0xF0;                  // �������� ����� F0 �� ������� ���� 3 �����
  temp3H = temp3>>4;                   // ����������� ������� ���� � �������
  temp3 = regs_in[2];                  // �������� 
  temp3L = temp3&0x0F;                 // �������� ����� 0F �� ������� ���� 3 �����

  temp4 = regs_in[3];                  // ��������  
  temp4 = temp4&0xF0;                  // �������� ����� F0 �� ������� ���� 3 �����
  temp4H = temp4>>4;                   // ����������� ������� ���� � �������
  crc_in =   temp1H ^  temp1L  ^   temp2H ^  temp2L  ^  temp3H ^  temp3L  ^  temp4H ;
  crc_in =  crc_in&0x0F;               // �������� ����� F0 �� ������� ���� 4 �����
  regs_crc[0]= crc_in;
  */
}

void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data )
{
	int rdata = data;
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.write(rdata);
	Wire.endTransmission();
	delay(10);
}
byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress ) {
	byte rdata = 0xFF;
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(deviceaddress,1);
	if (Wire.available()) rdata = Wire.read();
	return rdata;
}
void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length )
{
	
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(deviceaddress,length);
	int c = 0;
	for ( c = 0; c < length; c++ )
	if (Wire.available()) buffer[c] = Wire.read();
	
}
void i2c_eeprom_write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length ) 
{
	
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddresspage >> 8)); // MSB
	Wire.write((int)(eeaddresspage & 0xFF)); // LSB
	byte c;
	for ( c = 0; c < length; c++)
	Wire.write(data[c]);
	Wire.endTransmission();
	
}
void UpdateRegs()                                        // �������� ��������
{
	//-----������ ���� ------------
	//-----���������� ��� 0
	// while(prer_Kmerton_Run == true){}                  // ���� ��������� ��������� ������ �� ��������
	 boolean set_rele ;
	 prer_Kmerton_On = false;                            // ��������� ���������� �������� ??
	// reg_Kamerton();                                     // �������� ������ �� �������� �    �������� 
		// ������������ �������� ������ �� ��������� �� ����� ������
	  //-----���������� ��� 0
	 set_rele = mb.Coil(1);
	 mcp_Out1.digitalWrite(0, set_rele);                 // ���� RL0 ����  ���� Mic1p ���������

	 //-----���������� ��� 1
	  set_rele = mb.Coil(2);
	  mcp_Out1.digitalWrite(1, set_rele);               // ���� RL1 ���� Mic2p  ���������

	 //-----���������� ��� 2
	  set_rele = mb.Coil(3);
	  mcp_Out1.digitalWrite(2, set_rele);               // ���� RL2 ���� Mic3p MTT
  
	 //-----���������� ��� 3
	  set_rele = mb.Coil(4);
	  mcp_Out1.digitalWrite(3, set_rele);               // ���� RL3 ����

	 //-----���������� ��� 4                            // ���� RL4 XP1 12
	  set_rele = mb.Coil(5);
	  mcp_Out1.digitalWrite(4, set_rele);    

	 //-----���������� ��� 5
	  set_rele = mb.Coil(6);                        // ���� RL5 ����
	  mcp_Out1.digitalWrite(5, set_rele);              

	 //-----���������� ��� 6	 
	  set_rele = mb.Coil(7);
	  mcp_Out1.digitalWrite(6, set_rele);              // ���� RL6 ����

	 //-----���������� ��� 7
	  set_rele = mb.Coil(8);
	  mcp_Out1.digitalWrite(7, set_rele);              // ���� RL7 ������� �����

	 //---- ������ ����----------
	 //-----���������� ��� 8
	  set_rele = mb.Coil(9);                        // ���� RL8 ���� �� ��������
	  mcp_Out1.digitalWrite(8, set_rele);    

	 //-----���������� ��� 9
	  set_rele = mb.Coil(10);
	  mcp_Out1.digitalWrite(9, set_rele);               // ���� RL9 XP1 10

	 //-----���������� ��� 10                           // ���� RL10 ��������� ������� �� �������������� ������ 
	  set_rele = mb.Coil(11);
	  mcp_Out1.digitalWrite(10, set_rele);    


	//-----���������� ��� 11                            // �������� 
	  set_rele = mb.Coil(12);
	  mcp_Out1.digitalWrite(11, set_rele);    

	 //-----���������� ��� 12
	  set_rele = mb.Coil(13);
	  mcp_Out1.digitalWrite(12, set_rele);              // XP8 - 2   sensor �������� ������

	 //-----���������� ��� 13
	  set_rele = mb.Coil(14);
	  mcp_Out1.digitalWrite(13, set_rele);              // XP8 - 1   PTT �������� ������

	 //-----���������� ��� 14

	  set_rele = mb.Coil(15);
	  mcp_Out1.digitalWrite(14, set_rele);              // XS1 - 5   PTT ���

	  //-----���������� ��� 15
	  set_rele = mb.Coil(16);
	  mcp_Out1.digitalWrite(15, set_rele);              // XS1 - 6   sensor ���

	  //  Test 3
	 //-----������ ���� ------------
	 //-----���������� ��� 0

	  set_rele = mb.Coil(17);
	  mcp_Out2.digitalWrite(0, set_rele);                // J8-12     XP7 4 PTT2   ����. �.

	 //-----���������� ��� 1
	  set_rele = mb.Coil(18);
	  mcp_Out2.digitalWrite(1, set_rele);                // XP1 - 20  HangUp  DCD

	 //-----���������� ��� 2
	  set_rele = mb.Coil(19);
	  mcp_Out2.digitalWrite(2, set_rele);                // J8-11     XP7 2 sensor  ����. �.
  
	//-----���������� ��� 3

	  set_rele = mb.Coil(20);
	  mcp_Out2.digitalWrite(3, set_rele);                 // J8-23     XP7 1 PTT1 ����. �.

	 //-----���������� ��� 4
	  set_rele = mb.Coil(21);
	  mcp_Out2.digitalWrite(4, set_rele);                 // XP2-2     sensor "���." 

	 //-----���������� ��� 5

	  set_rele = mb.Coil(22);
	  mcp_Out2.digitalWrite(5, set_rele);                  // XP5-3     sensor "��C."

	 //-----���������� ��� 6
	  set_rele = mb.Coil(23);
	  mcp_Out2.digitalWrite(6, set_rele);                  // XP3-3     sensor "��-�����1."

	 //-----���������� ��� 7
	  set_rele = mb.Coil(24);
	  mcp_Out2.digitalWrite(7, set_rele);                  // XP4-3     sensor "��-�����2."

	  // Test 4
	//-----������ ���� ------------
	 //-----���������� ��� 8
	  set_rele = mb.Coil(25);
	  mcp_Out2.digitalWrite(8, set_rele);                  // XP1- 19 HaSs      ���� ����������� ������  

	  //-----���������� ��� 9
	  set_rele = mb.Coil(26);
	  mcp_Out2.digitalWrite(9, set_rele);                  // XP1- 17 HaSPTT    CTS DSR ���.

	  //-----���������� ��� 10
	  set_rele = mb.Coil(27);
	  mcp_Out2.digitalWrite(10, set_rele);                 // XP1- 16 HeS2Rs    ���� ����������� ��������� ����������� � 2 ����������

	  //-----���������� ��� 11
	  set_rele = mb.Coil(28);
	  mcp_Out2.digitalWrite(11, set_rele);                 // XP1- 15 HeS2PTT   CTS ���

	  //-----���������� ��� 12
	  set_rele = mb.Coil(29);
	  mcp_Out2.digitalWrite(12, set_rele);                 // XP1- 13 HeS2Ls    ���� ����������� ��������� ����������� 

	  //-----���������� ��� 13
	  set_rele = mb.Coil(30);
	  mcp_Out2.digitalWrite(13, set_rele);                 // XP1- 6  HeS1PTT   CTS ���

	  //-----���������� ��� 14
	  set_rele = mb.Coil(31);
	  mcp_Out2.digitalWrite(14, set_rele);                 // XP1- 5  HeS1Rs    ���� ���������� ��������� ���������� � 2 ����������

	  //-----���������� ��� 15
	  set_rele = mb.Coil(32);
	  mcp_Out2.digitalWrite(15, set_rele);                 // XP1- 1  HeS1Ls    ���� ���������� ��������� ����������

		 if (mb.Coil(118)== 0)
		 {
			 test_repeat = false;
		 }
	else
		 {
			test_repeat = true;
		 }


	  //*******************************************************

	  mb.Ists(adr_reg_ind_CTS, !mcp_Analog.digitalRead(CTS));
	  mb.Ists(adr_reg_ind_DSR, !mcp_Analog.digitalRead(DSR));
	  mb.Ists(adr_reg_ind_DCD, !mcp_Analog.digitalRead(DCD));

	  time_control();
	  prer_Kmerton_On = true;
}

void set_rezistor()
{
	int mwt = mb.Hreg(60);             // ����� �������� �������� �������
	resistor(1, mwt);
	resistor(2, mwt);
	mb.Hreg(adr_control_command,0);
}
void resistor(int resist, int valresist)
{
	resistance = valresist;
	switch (resist)
	{
	case 1:
			Wire.beginTransmission(address_AD5252);     // transmit to device
			Wire.write(byte(control_word1));            // sends instruction byte  
			Wire.write(resistance);                     // sends potentiometer value byte  
			Wire.endTransmission();                     // stop transmitting
			break;
	case 2:				
			Wire.beginTransmission(address_AD5252);     // transmit to device
			Wire.write(byte(control_word2));            // sends instruction byte  
			Wire.write(resistance);                     // sends potentiometer value byte  
			Wire.endTransmission();                     // stop transmitting
			break;
	}
			//Wire.requestFrom(address_AD5252, 1, true);  // ������� ��������� ������ ��������� 
			//level_resist = Wire.read();                 // sends potentiometer value byte  
}

void Reg_count_clear()
{
	for(unsigned int i = 200; i<=319;i++)
	{
		mb.Hreg(i,0);
	}
		for(unsigned int i = 400; i<=519;i++)
	{
		mb.Hreg(i,0);
	}
	for(int k = 200; k<=319;k++)
	{
		mb.Coil(k,false);
	}
	mb.Hreg(121,0);                                                             // �������� ������� ������
	mb.Hreg(adr_control_command,0);
}
void set_clock()
{    
		int day    = mb.Hreg(adr_set_kontrol_day);  
		int month  = mb.Hreg(adr_set_kontrol_month);          
		int year   = mb.Hreg(adr_set_kontrol_year);  
		int hour   = mb.Hreg(adr_set_kontrol_hour);  
		int minute = mb.Hreg(adr_set_kontrol_minute);  
		int second = 0;
		DateTime set_time = DateTime(year, month, day, hour, minute, second);   // ������� ������ � ������� � ������ "set_time"
		RTC.adjust(set_time);                                                   // �������� ����� � ���������� �����  
		mb.Hreg(adr_set_time, 0);                                               // �������� � ������� ������� ��������� ���������� �������
		mb.Hreg(adr_control_command,0);
}
void data_clock_exchange()
{
	/*
	DateTime now = RTC.now();
	uint16_t year_temp = now.year()-2000;
	uint8_t day_temp = now.day();
	uint8_t mon_temp = now.month();

	byte  b = i2c_eeprom_read_byte(0x50, adr_temp_day);                             //access an address from the memory
		delay(10);

		if (b != day_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_day, day_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		  regBank.set(adr_reg_temp_day,day_temp);  
		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon);                             //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		  regBank.set(adr_reg_temp_mon,mon_temp); 
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year);                            //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		 regBank.set(adr_reg_temp_year,year_temp); 

		  b = i2c_eeprom_read_byte(0x50, adr_file_name_count);                             //access an address from the memory
		  regBank.set(adr_reg_file_name,b);                                                // �������  �������� ���������� ����� �����
		  */
}
void time_control() // ��������� ������ �������� ������� � �������� ��� �������� � ��
{
	DateTime now = RTC.now();
	mb.Hreg(adr_kontrol_day  , now.day());
	mb.Hreg(adr_kontrol_month, now.month());
	mb.Hreg(adr_kontrol_year, now.year());
	mb.Hreg(adr_kontrol_hour, now.hour());
	mb.Hreg(adr_kontrol_minute, now.minute());
	mb.Hreg(adr_kontrol_second, now.second());
}
void time_control_get()   // �������� ��������� �������� ���������� ��������� �������
{
  for (unsigned int i = 0; i < 6; i++)     // 
	{
	   Serial.print(mb.Hreg(46+i));   
	   Serial.print(" "); 
	}
Serial.println();   
}

void list_file()
{
 while (file.openNext(sd.vwd(), O_READ))
  {
	file.printName(&Serial);
	Serial.write(' ');
	file.printModifyDateTime(&Serial);
	Serial.write(' ');
	file.printFileSize(&Serial);
	if (file.isDir()) {
	  // Indicate a directory.
	  Serial.write('/');
	}
	Serial.println();
	file.close();
  }
}
void preob_num_str() // ��������� ������������ ����� �����, ���������� �� ������� ���� � �������� ������
{
	DateTime now = RTC.now();
	day   = now.day();
	month = now.month();
	year  = now.year();
	int year_temp = year-2000;
	itoa (year_temp,str_year_file, 10);                                        // �������������� ���� ��� � ������ ( 10 - ���������� ������) 

	if (month <10)
		{
		   itoa (0,str_mon_file0, 10);                                         //  �������������� ���� �����  � ������ ( 10 - ���������� ������) 
		   itoa (month,str_mon_file10, 10);                                    //  �������������� ����� � ������ ( 10 - ���������� ������) 
		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);       // �������� 2 �����
		}
	else
		{
		   itoa (month,str_mon_file, 10);                                      // �������������� ����� � ������ ( 10 - ���������� ������) 
		}
	if (day <10)
		{
		   itoa (0,str_day_file0, 10);                                         // �������������� ����� � ������ ( 10 - ���������� ������) 
		   itoa (day,str_day_file10, 10);                                      // �������������� ����� � ������ ( 10 - ���������� ������) 
		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);       // �������� 2 �����
		}
	else
		{
		itoa (day,str_day_file, 10);                                           // �������������� ����� � ������ ( 10 - ���������� ������) 
		}
		 
	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // �������� 2 �����
	sprintf(str2, "%s%s",str1, str_day_file);                                  // �������� 2 �����
	sprintf(fileName, "%s%s", str2, "00.KAM");                                 // ��������� ����� ����� � file_name
	//Serial.println(fileName);
	mb.Hreg(adr_reg_temp_day, day);  
	mb.Hreg(adr_reg_temp_mon, month); 
	mb.Hreg(adr_reg_temp_year, year-2000); 
	//char* strcpy(char* fileName_p, const char* fileName);
	//Serial.println(fileName_p);
}









//------------------------------------------------------------------------------------

void serialEvent3()
{
	//mb.task();
	//control_command();
}

void control_command()
{
	/*
	��� ������ ������������ �������� ���������� �������� ����� �������� �� ������ adr_control_command (40120) 
	��� ��������
	0 -  ���������� ������� ��������
	1 -  ��������� ��� �������
	2 -  �������� ��� �������
	3 -  ���� �����������
	4 -  ���� ����������
	5 -  ���� ���
	6 -  ���� ����.�
	7 -  ���� ��������
	8 -  ���� ���
	9 -  ���� ����� 1
	10 - ���� ����� 2
	11 - ���� ����������
	12 - ������� ����
	13 - ������� ����
	14 - �������� �����
	15 - ���������� ������� �������
	16 - Reg_count_clear();			                                        // ����� ��������� ������                    
	17 - test_power();                                                    	// ��������� ����������  �������
	18 - set_video();				 //
	19 - test_video();				 //
	20 - �������� ������ ������� ���������
	21 - �������� ������ ������� ����������������
	22 - �������� ������ ������� ����������������
	23 - �������� ����� �����
	*/
	UpdateRegs() ;

	int test_n = mb.Hreg(adr_control_command);                                  //�����  40120
	if (test_n != 0)
	{
//	Serial.println(test_n);	
	switch (test_n)
	{
		case 1:
			// sensor_all_off();                                                      // ��������� ��� �������
			 break;
		case 2:		
			// sensor_all_on();                                                       // �������� ��� �������
			 break;
		case 3:
			// test_headset_instructor();
			 break;
		case 4:	
			// test_headset_dispatcher();                                             //
			 break;
		case 5:
			// test_MTT();                                                            //
			 break;
		case 6:	
			// test_tangR();                                                          //
			 break;
		case 7:
			//test_tangN();
			break;
		case 8:				
			// testGGS();
			 break;
		case 9:
			// test_GG_Radio1();
			 break;
		case 10:	
			// test_GG_Radio2();
			 break;
		case 11:				
			// test_mikrophon();                                                      // ������������ ���������
			 break;
		case 12:
			//  FileOpen();
			  break;
		case 13:
			//  FileClose();
			  break;
		case 14:
			//  set_clock();
				break;
		case 15:
			//  set_rezistor();
				break;
		case 16:
			//	Reg_count_clear();			                                        // ����� ��������� ������                    
				break;
		case 17:
			//	test_power();                                                    	// ��������� ����������  �������
				break;
		case 18:
			//	set_video();				              //
				break;
		case 19:
			//	test_video();				              //
				break;
		case 20:                                           // �������� ������ ������� ���������
			//	default_mem_porog();
				break;
		case 21:                                           // 	21 - �������� ������ ������� ����������������
			//	set_mem_porog();
				break;
		case 22:                                           // 22 - �������� ������ ������� ����������������
			//	read_mem_porog();
				break;
		case 23:   
			//	controlFileName();                         // �������� ����� �����
				break;
		case 24:   
			//	 set_SD();                                 // �������� SD ������
				break;
		case 25:   
			//	send_file_PC();                                 // 
				break;
		case 26:   
			//	load_list_files();  
				break;
		case 27:   
			//	file_del_SD();
				break;
		 case 28:   
			//	clear_serial2();
				break;
		 case 29:   
			//	set_USB0();
				break;
		 case 30:   
			//	mem_byte_trans_read();
				break;
		 case 31:   
			//	mem_byte_trans_save();
				break;
	
		default:

		break;
	 }
	 mb.Hreg(adr_control_command,0);
	}
}



void setup_mcp()
{
	// ��������� ����������� ������

 
  mcp_Out1.begin(4);              //  ����� (4) �������  ����������� ������
  mcp_Out1.pinMode(0, OUTPUT);    // ���� �0  ����    
  mcp_Out1.pinMode(1, OUTPUT);    // ���� �1  ����    
  mcp_Out1.pinMode(2, OUTPUT);    // ���� �2  ����    
  mcp_Out1.pinMode(3, OUTPUT);    // ���� �3  ����    
  mcp_Out1.pinMode(4, OUTPUT);    // ���� �4  ����   XP1 10-12
  mcp_Out1.pinMode(5, OUTPUT);    // ���� �5  ����    
  mcp_Out1.pinMode(6, OUTPUT);    // ���� �6  ����   
  mcp_Out1.pinMode(7, OUTPUT);    // ���� �7  �������� +12�����  ������� ����� ��������
  
  mcp_Out1.pinMode(8, OUTPUT);    //  ���� �8 ���� �� �������� ����.  
  mcp_Out1.pinMode(9, OUTPUT);    // �������� J24 - 3    
  mcp_Out1.pinMode(10, OUTPUT);   // �������� J24 - 2    
  mcp_Out1.pinMode(11, OUTPUT);   // �������� J24 - 1   
  mcp_Out1.pinMode(12, OUTPUT);   // XP8 - 2  sensor     
  mcp_Out1.pinMode(13, OUTPUT);   // XP8 - 1  PTT       
  mcp_Out1.pinMode(14, OUTPUT);   // XS1 - 5   PTT      
  mcp_Out1.pinMode(15, OUTPUT);   // XS1 - 6 sensor      

	
  mcp_Out2.begin(6);              //  ����� (6) �������  ����������� ������
  mcp_Out2.pinMode(0, OUTPUT);    // J8-12    XP7 4 PTT2    
  mcp_Out2.pinMode(1, OUTPUT);    // XP1 - 20  HandUp    
  mcp_Out2.pinMode(2, OUTPUT);    // J8-11    XP7 2 sensor
  mcp_Out2.pinMode(3, OUTPUT);    // J8-23    XP7 1 PTT1    
  mcp_Out2.pinMode(4, OUTPUT);    // XP2-2    sensor "���."    
  mcp_Out2.pinMode(5, OUTPUT);    // XP5-3    sensor "��C." 
  mcp_Out2.pinMode(6, OUTPUT);    // XP3-3    sensor "��-�����1."
  mcp_Out2.pinMode(7, OUTPUT);    // XP4-3    sensor "��-�����2."
  
  mcp_Out2.pinMode(8, OUTPUT);    // XP1- 19 HaSs
  mcp_Out2.pinMode(9, OUTPUT);    // XP1- 17 HaSPTT
  mcp_Out2.pinMode(10, OUTPUT);   // XP1- 16 HeS2Rs
  mcp_Out2.pinMode(11, OUTPUT);   // XP1- 15 HeS2PTT
  mcp_Out2.pinMode(12, OUTPUT);   // XP1- 13 HeS2Ls           
  mcp_Out2.pinMode(13, OUTPUT);   // XP1- 6  HeS1PTT            
  mcp_Out2.pinMode(14, OUTPUT);   // XP1- 5  HeS1Rs            
  mcp_Out2.pinMode(15, OUTPUT);   // XP1- 1  HeS1Ls          

 
  mcp_Analog.begin(5);            //  ����� (5)  ����������� ������ 
  mcp_Analog.pinMode(8, OUTPUT);  // DTR_D
  mcp_Analog.pinMode(9, OUTPUT);  // RTS_D
  mcp_Analog.pinMode(10, OUTPUT); // J15-2 ��������
  mcp_Analog.pinMode(11, OUTPUT); // J15-3 ��������
  mcp_Analog.pinMode(12, OUTPUT); // J15-4 ��������
  mcp_Analog.pinMode(13, OUTPUT); // J15-5
  mcp_Analog.pinMode(14, OUTPUT); // J15-6
  mcp_Analog.pinMode(15, OUTPUT); // J15-7 
  
  mcp_Analog.pinMode(0, INPUT);   //  J22-1 ��������
  mcp_Analog.pullUp(0, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(1, INPUT);   // J22-2 ��������  ����������� ������ �� ����
  mcp_Analog.pullUp(1, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(2, INPUT);   // J22-3 �������� �������� ����������� ������ �� ���� 
  mcp_Analog.pullUp(2, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(3, INPUT);   // J22-4 �������� ����������� ������ �� ����
  mcp_Analog.pullUp(3, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(4, INPUT);   // J22-5 ��������  ����������� ������ �� ����
  mcp_Analog.pullUp(4, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(5, INPUT);   //CTS ����������� ������ �� ����
  mcp_Analog.pullUp(5, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(6, INPUT);   // DSR ����������� ������ �� ����
  mcp_Analog.pullUp(6, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(7, INPUT);   //  DCD ����������� ������ �� ����
  mcp_Analog.pullUp(7, HIGH);     // ���������� ���������� �������� 100K � 5�.

}
void setup_regModbus()
{
	// Set the Slave ID (1-247)
	mb.setSlaveId(1);  
   // ����������� � ��������� MODBUS ���������� Serial3    

	mb.addCoil(1);                           // ���� RL0 ����  MIC1P
	mb.addCoil(2);                           // ���� RL1 ����  MIC2P
	mb.addCoil(3);                           // ���� RL2 ����  MIC3P
	mb.addCoil(4);                           // ���� RL3 ����  LFE  "���."
	mb.addCoil(5);                           // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
	mb.addCoil(6);                           // ���� RL5 ���� Front L, Front R
	mb.addCoil(7);                           // ���� RL6 ���� Center
	mb.addCoil(8);                           // ���� RL7 ������� �����
  
	mb.addCoil(9);                           // ���� RL8 ���� �� ��������
	mb.addCoil(10);                          // ���� RL9 XP1 10 ��������� ��������� ����������
	mb.addCoil(11);                          // ���� RL10 ��������� ������� �� �������������� ������ 
	mb.addCoil(12);                          // �������� J24 - 1 
	mb.addCoil(13);                          // XP8 - 2   sensor �������� ������
	mb.addCoil(14);                          // XP8 - 1   PTT �������� ������
	mb.addCoil(15);                          // XS1 - 5   PTT ���
	mb.addCoil(16);                          // XS1 - 6   sensor ���
 
	mb.addCoil(17);                          // J8-12     XP7 4 PTT2   ����. �.
	mb.addCoil(18);                          // XP1 - 20  HangUp  DCD
	mb.addCoil(19);                          // J8-11     XP7 2 sensor  ����. �.
	mb.addCoil(20);                          // J8-23     XP7 1 PTT1 ����. �.
	mb.addCoil(21);                          // XP2-2     sensor "���."  
	mb.addCoil(22);                          // XP5-3     sensor "��C."
	mb.addCoil(23);                          // XP3-3     sensor "��-�����1."
	mb.addCoil(24);                          // XP4-3     sensor "��-�����2."
 
	mb.addCoil(25);                          // XP1- 19 HaSs      sensor ����������� ������    MTT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	mb.addCoil(26);                          // XP1- 17 HaSPTT    CTS DSR ���.  
	mb.addCoil(27);                          // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.addCoil(28);                          // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	mb.addCoil(29);                          // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	mb.addCoil(30);                          // XP1- 6  HeS1PTT   CTS ���   ��� ����������
	mb.addCoil(31);                          // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.addCoil(32);                          // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������


	mb.addCoil(118);                         // ���� ��������� ������������ ��������
	mb.addCoil(119);                         // 

	mb.addCoil(120);                         // ���� ��������� ������������� ����� ������
	mb.addCoil(122);                         // ���� ��������� �������� �����
	mb.addCoil(123);                         // ���� ��������� �������� �����
	mb.addCoil(124);                         // ���� ��������� ����� � ������� "��������"
	mb.addCoil(125);                         // ���� ��������� ������������� SD ������
	mb.addCoil(126);                         //  
	mb.addCoil(127);                         //  
	mb.addCoil(128);                         //  
	mb.addCoil(129);                         //  

	mb.addCoil(130);                         //  ���� ��������� ����� 0 - RS232, 1 - USB0
	mb.addCoil(131);                         //  


	mb.addCoil(200);                         // ���� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	mb.addCoil(201);                         // ���� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	mb.addCoil(202);                         // ���� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	mb.addCoil(203);                         // ���� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	mb.addCoil(204);                         // ���� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	mb.addCoil(205);                         // ���� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	mb.addCoil(206);                         // ���� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	mb.addCoil(207);                         // ���� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	mb.addCoil(208);                         // ���� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	mb.addCoil(209);                         // ���� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	mb.addCoil(210);                         // ���� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	mb.addCoil(211);                         // ���� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	mb.addCoil(212);                         // ���� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	mb.addCoil(213);                         // ���� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	mb.addCoil(214);                         // ���� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	mb.addCoil(215);                         // ���� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	mb.addCoil(216);                         // ���� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	mb.addCoil(217);                         // ���� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	mb.addCoil(218);                         // ���� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	mb.addCoil(219);                         // ���� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
	 
	mb.addCoil(220);                         // ���� ������ "Command PTT headset instructor (CTS)                        OFF - ";
	mb.addCoil(221);                         // ���� ������ "Command PTT headset instructor (CTS)                        ON  - ";
	mb.addCoil(222);                         // ���� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
	mb.addCoil(223);                         // ���� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
	mb.addCoil(224);                         // ���� ������ "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addCoil(225);                         // ���� ������ "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addCoil(226);                         // ���� ������ "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addCoil(227);                         // ���� ������ "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addCoil(228);                         // ���� ������ "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addCoil(229);                         // ���� ������ "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addCoil(230);                         // ���� ������ "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addCoil(231);                         // ���� ������ "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addCoil(232);                         // ���� ������ "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addCoil(233);                         // ���� ������ "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addCoil(234);                         // ���� ������ "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addCoil(235);                         // ���� ������ "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addCoil(236);                         // ���� ������ "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addCoil(237);                         // ���� ������ "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addCoil(238);                         // ���� ������ "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addCoil(239);                         // ���� ������  ADC0  ��� x1 

	mb.addCoil(240);                         // ���� ������ "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addCoil(241);                         // ���� ������ "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addCoil(242);                         // ���� ������ "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addCoil(243);                         // ���� ������ "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addCoil(244);                         // ���� ������ "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addCoil(245);                         // ���� ������ "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addCoil(246);                         // ���� ������ "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addCoil(247);                         // ���� ������ "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addCoil(248);                         // ���� ������ "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addCoil(249);                         // ���� ������ ADC2 ��� x10  

	mb.addCoil(250);                         // ���� ������ "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addCoil(251);                         // ���� ������ "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addCoil(252);                         // ���� ������ "Test MTT ** Signal LineL                                    OFF - ";
	mb.addCoil(253);                         // ���� ������ "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addCoil(254);                         // ���� ������ "Test MTT ** Signal mag radio                                OFF - ";
	mb.addCoil(255);                         // ���� ������ "Test MTT ** Signal mag phone                                OFF - ";
	mb.addCoil(256);                         // ���� ������ "Test MTT ** Signal GGS                                      OFF - ";
	mb.addCoil(257);                         // ���� ������ "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addCoil(258);                         // ���� ������ "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addCoil(259);                         // ���� ������ "Test MTT ** Signal GGS                                      ON  - ";

	mb.addCoil(260);                         // ���� ������ "Test MTT ** Signal LineL                                    ON  - ";
	mb.addCoil(261);                         // ���� ������ "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addCoil(262);                         // ���� ������ "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addCoil(263);                         // ���� ������ "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addCoil(264);                         // ���� ������ "Test microphone PTT  (CTS)                                  OFF - ";
	mb.addCoil(265);                         // ���� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addCoil(266);                         // ���� ������ "Test microphone PTT  (CTS)                                  ON  - ";
	mb.addCoil(267);                         // ���� ������ "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addCoil(268);                         // ���� ������ "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addCoil(269);                         // ���� ������ ������������ ����������� ������� 

	mb.addCoil(270);                         // ���� ������ "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addCoil(271);                         // ���� ������ "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addCoil(272);                         // ���� ������ "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addCoil(273);                         // ���� ������ "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addCoil(274);                         // ���� ������ "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addCoil(275);                         // ���� ������ "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addCoil(276);                         // ���� ������ "Command sensor tangenta nognaja                             OFF - ";
	mb.addCoil(277);                         // ���� ������ "Command sensor tangenta nognaja                             ON  - ";
	mb.addCoil(278);                         // ���� ������ "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addCoil(279);                         // ���� ������ "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addCoil(280);                         // ���� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addCoil(281);                         // ���� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addCoil(282);                         // ���� ������ "Test GGS ** Signal LineL                                    OFF - ";
	mb.addCoil(283);                         // ���� ������ "Test GGS ** Signal LineR                                    OFF - ";
	mb.addCoil(284);                         // ���� ������ "Test GGS ** Signal mag radio                                OFF - ";
	mb.addCoil(285);                         // ���� ������ "Test GGS ** Signal mag phone                                OFF - ";
	mb.addCoil(286);                         // ���� ������ "Test GGS ** Signal GGS                                      OFF - ";
	mb.addCoil(287);                         // ���� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addCoil(288);                         // ���� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addCoil(289);                         // ���� ������ "Test GGS ** Signal GGS                                      ON  - ";

	mb.addCoil(290);                         // ���� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addCoil(291);                         // ���� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addCoil(292);                         // ���� ������ "Test GGS ** Signal mag phone                                ON  - ";
	mb.addCoil(293);                         // ���� ������ ADC1 ���������� 12/3 �����
	mb.addCoil(294);                         // ���� ������ ADC14 ���������� 12/3 ����� Radio1
	mb.addCoil(295);                         // ���� ������ ADC14 ���������� 12/3 ����� Radio2
	mb.addCoil(296);                         // ���� ������ ADC14 ���������� 12/3 ����� ���
	mb.addCoil(297);                         // ���� ������ ADC15 ���������� ���������� 3,6 ������
	mb.addCoil(298);                         // ���� ������ "Test Microphone ** Signal mag phone                         ON  - ";      
	mb.addCoil(299);                         // ���� ������ "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addCoil(300);                         // ���� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addCoil(301);                         // ���� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addCoil(302);                         // ���� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addCoil(303);                         // ���� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addCoil(304);                         // ���� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addCoil(305);                         // ���� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addCoil(306);                         // ���� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addCoil(307);                         // ���� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addCoil(308);                         // ���� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addCoil(309);                         // ���� ������ "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addCoil(310);                         // ���� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addCoil(311);                         // ���� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addCoil(312);                         // ���� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addCoil(313);                         // ���� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addCoil(314);                         // ���� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addCoil(315);                         // ���� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addCoil(316);                         // ���� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addCoil(317);                         // ���� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addCoil(318);                         // ���� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addCoil(319);                         // ���� ������ "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addCoil(320);                         // ���� ������ "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addCoil(321);                         // ���� ������ "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addCoil(322);                         // ���� ������ "Test Microphone ** Signal LineL                             OFF - ";
	mb.addCoil(323);                         // ���� ������ "Test Microphone ** Signal LineR                             OFF - ";
	mb.addCoil(324);                         // ���� ������ "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addCoil(325);                         // ���� ������ "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addCoil(326);                         // ���� ������ "Test Microphone ** Signal GGS                               OFF - ";
	mb.addCoil(327);                         // ���� ������ "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addCoil(328);                         // ���� ������ "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addCoil(329);                         // ���� ������ ��� ������� �������

	mb.addCoil(330);                         // ���� ������ "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addCoil(331);                         // ���� ������ "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	mb.addCoil(332);                         // ���� ������ "Test GGS ** Signal mag radio   


	mb.addIsts(81);    // ����� ����a ��������� ��������� ������� CTS
	mb.addIsts(82);    // ����� ����a ��������� ��������� ������� DSR
	mb.addIsts(83);    // ����� ����a ��������� ��������� ������� DCD

						 //Add Input registers 30001-30040 to the register bank


	//regBank.set(40004+buffer,Serial1.read());

	//mb.addHreg(40000);  // 
	mb.addHreg(1,1);  // �������� ������ � ����� 1
	mb.addHreg(2,2);  // �������� ������ � ����� 1
	mb.addHreg(3,3);  // �������� ������ � ����� 1
	mb.addHreg(4,4);  // �������� ������ � ����� 1
	mb.addHreg(5,5);  // �������� ������ � ����� 1
	mb.addHreg(6,6);  // �������� ������ � ����� 1
	mb.addHreg(7,7);  // �������� ������ � ����� 1
	mb.addHreg(8,8);  // 
	mb.addHreg(9,9);  // 




	mb.addHreg(10);  // �  ����� 1
	mb.addHreg(11);  // �  ����� 1
	mb.addHreg(12);  // �  ����� 1
	mb.addHreg(13);  // �  ����� 1
	mb.addHreg(14);  // 
	mb.addHreg(15);  // 
	mb.addHreg(16);  // 
	mb.addHreg(17);  // 
	mb.addHreg(18);  // 
	mb.addHreg(19);  // 

						 // ������� ����� 
	mb.addHreg(46,1);  // ����� ���� ������ ����� �����������
	mb.addHreg(47,2);  // ����� ����� ������ ����� �����������
	mb.addHreg(48,3);  // ����� ��� ������ ����� �����������
	mb.addHreg(49,4);  // ����� ��� ������ ����� �����������
	mb.addHreg(50,5);  // ����� ������ ������ ����� �����������
	mb.addHreg(51,6);  // ����� ������� ������ ����� �����������
							 // ������� ����� 
	mb.addIreg(1,6);  // ����� ���� ������ ����� �����������
	mb.addIreg(2,5);  // ����� ����� ������ ����� �����������
	mb.addIreg(3,4);  // ����� ��� ������ ����� �����������
	mb.addIreg(4,3);  // ����� ��� ������ ����� �����������
	mb.addIreg(5,2);  // ����� ������ ������ ����� �����������
	mb.addIreg(6,1);  // ����� ������� ������ ����� �����������
	mb.addIreg(7,2);  // ����� ������ ������ ����� �����������
	mb.addIreg(8,1);  // ����� ������� ������ ����� �����������


//  mb.addIreg(SENSOR_IREG);
						 // ��������� ������� � �����������
	mb.addHreg(52);  // ����� ����
	mb.addHreg(53);  // ����� �����
	mb.addHreg(54);  // ����� ���
	mb.addHreg(55);  // ����� ���
	mb.addHreg(56);  // ����� ������
	mb.addHreg(57);  // 
	mb.addHreg(58);  // 
	mb.addHreg(59);  // 
	
	mb.addHreg(60);  // ����� �������� �������� ������� �����������
	mb.addHreg(61);  // ����� �������� �������� ������� ��� ����������
	mb.addHreg(62);  // ����� �������� �������� ������� ��� �������� � ���������
	mb.addHreg(63);  // ����� �������� ������������ �������� ������� ��� �������� � ��������� ��


	/*
	mb.addHreg(40061); // ����� �������� ������
	mb.addHreg(40062); // ����� �������� ������
	mb.addHreg(40063); // ����� �������� ������
	mb.addHreg(40064); // ����� ������
	mb.addHreg(40065); // ����� ������
	mb.addHreg(40066); // ����� ������
	mb.addHreg(40067); // ����� ������
	mb.addHreg(40068); // ����� ������
	mb.addHreg(40069); // ����� ������
	mb.addHreg(40070); // ����� ������
	mb.addHreg(40071); // ����� ������

	mb.addHreg(40072); // ����� ������ � %
	mb.addHreg(40073); // ����� ������ � %
	mb.addHreg(40074); // ����� ������ � %
	mb.addHreg(40075); // ����� ������ %
	mb.addHreg(40076); // ����� ������ %
	mb.addHreg(40077); // ����� ������ %
	mb.addHreg(40078); // ����� ������ %
	mb.addHreg(40079); // ����� ������ %
	mb.addHreg(40080); // ����� ������ %
	mb.addHreg(40081); // ����� ������ %
	mb.addHreg(40082); // ����� ������ %
	mb.addHreg(40083); // ����� ������ %

	// ����� ������ �� ���������
	mb.addHreg(40084); // ����� ���� adr_Mic_On_day 
	mb.addHreg(40085); // ����� ����� adr_Mic_On_month  
	mb.addHreg(40086); // ����� ��� adr_Mic_On_year  
	mb.addHreg(40087); // ����� ��� adr_Mic_On_hour 
	mb.addHreg(40088); // ����� ������ adr_Mic_On_minute 
	mb.addHreg(40089); // ����� �������  adr_Mic_On_second    

	// ����� ������ �� ����������
	mb.addHreg(40090); // ����� ���� adr_Mic_Off_day    
	mb.addHreg(40091); // ����� �����  adr_Mic_Off_month 
	mb.addHreg(40092); // ����� ��� adr_Mic_Off_year  
	mb.addHreg(40093); // ����� ��� adr_Mic_Off_hour   
	mb.addHreg(40094); // ����� ������ adr_Mic_Off_minute   
	mb.addHreg(40095); // ����� ������� adr_Mic_Off_second    
	*/
	// ����� ������ �����
	mb.addHreg(96);  // ����� ����  adr_Mic_Start_day    
	mb.addHreg(97);  // ����� ����� adr_Mic_Start_month  
	mb.addHreg(98);  // ����� ��� adr_Mic_Start_year  
	mb.addHreg(99);  // ����� ��� adr_Mic_Start_hour 
	mb.addHreg(100);  // ����� ������ adr_Mic_Start_minute 
	mb.addHreg(101);  // ����� ������� adr_Mic_Start_second  

	// ����� ��������� �����
	mb.addHreg(102);  // ����� ���� adr_Mic_Stop_day 
	mb.addHreg(103);  // ����� ����� adr_Mic_Stop_month 
	mb.addHreg(104);  // ����� ��� adr_Mic_Stop_year
	mb.addHreg(105);  // ����� ��� adr_Mic_Stop_hour 
	mb.addHreg(106);  // ����� ������ adr_Mic_Stop_minute  
	mb.addHreg(107);  // ����� ������� adr_Mic_Stop_second 

	// ����������������� ���������� �����
	mb.addHreg(108);  // ����� ���� adr_Time_Test_day 
	mb.addHreg(109);  // ����� ��� adr_Time_Test_hour 
	mb.addHreg(110);  // ����� ������ adr_Time_Test_minute
	mb.addHreg(111);  // ����� ������� adr_Time_Test_second

	mb.addHreg(112);  // ����� �������� ���������� ��� 
	mb.addHreg(113);  // ����� �������� ���������� ����� 
	mb.addHreg(114);  // ����� �������� ���������� ����
	mb.addHreg(115);  // ����� �������� ���������� �������� ���������� ������ �����
	mb.addHreg(116);  // ����� �������� ���������� �������� �������� ������ �����

	mb.addHreg(120);  // adr_control_command ����� �������� ������� �� ����������
	mb.addHreg(121);  // ����� �������� ���� ������
	mb.addHreg(122);  //
	mb.addHreg(123);  //
	mb.addHreg(124);  //
	mb.addHreg(125);  //  
	mb.addHreg(126);  //  
	mb.addHreg(127);  //  ����� ����� ��������� ��� �������� � �� ������� �������.
	mb.addHreg(128);  //  ����� ����� ������ ��� �������� � �� ������� �������.
	mb.addHreg(129);  //  ����� ����� ����� ������ ��� �������� � �� ������� �������.

	mb.addHreg(130);  //  �������� ���������� �������� ��� �������� ������� ������� 
	mb.addHreg(131);  //
	mb.addHreg(132);  //  
	mb.addHreg(133);  //
	mb.addHreg(134);  //  
	mb.addHreg(135);  //
	mb.addHreg(136);  //  
	mb.addHreg(137);  //
	mb.addHreg(138);  //  
	mb.addHreg(139);  //

	mb.addHreg(140);  //  
	mb.addHreg(141);  //
	mb.addHreg(142);  //  
	mb.addHreg(143);  //
	mb.addHreg(144);  //  
	mb.addHreg(145);  //
	mb.addHreg(146);  //  
	mb.addHreg(147);  //
	mb.addHreg(148);  //  
	mb.addHreg(149);  //

	mb.addHreg(150);  //  
	mb.addHreg(151);  //
	mb.addHreg(152);  //  
	mb.addHreg(153);  //
	mb.addHreg(154);  //  
	mb.addHreg(155);  //
	mb.addHreg(156);  //  
	mb.addHreg(157);  //
	mb.addHreg(158);  //  
	mb.addHreg(159);  //

	mb.addHreg(160);  //  
	mb.addHreg(161);  //
	mb.addHreg(162);  //  
	mb.addHreg(163);  //
	mb.addHreg(164);  //  
	mb.addHreg(165);  //
	mb.addHreg(166);  //  
	mb.addHreg(167);  //
	mb.addHreg(168);  //  
	mb.addHreg(169);  //


	mb.addHreg(200);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	mb.addHreg(201);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	mb.addHreg(202);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	mb.addHreg(203);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	mb.addHreg(204);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	mb.addHreg(205);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	mb.addHreg(206);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	mb.addHreg(207);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	mb.addHreg(208);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	mb.addHreg(209);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	mb.addHreg(210);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	mb.addHreg(211);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	mb.addHreg(212);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	mb.addHreg(213);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	mb.addHreg(214);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	mb.addHreg(215);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	mb.addHreg(216);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	mb.addHreg(217);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	mb.addHreg(218);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	mb.addHreg(219);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

	mb.addHreg(220);                         // A���� �������� ������ "Command PTT headset instructor (CTS)                        OFF - ";
	mb.addHreg(221);                         // A���� �������� ������ "Command PTT headset instructor (CTS)                        ON  - ";
	mb.addHreg(222);                         // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
	mb.addHreg(223);                         // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
	mb.addHreg(224);                         // A���� �������� ������ "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addHreg(225);                         // A���� �������� ������ "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addHreg(226);                         // A���� �������� ������ "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addHreg(227);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addHreg(228);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addHreg(229);                         // A���� �������� ������ "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addHreg(230);                         // A���� �������� ������ "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addHreg(231);                         // A���� �������� ������ "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addHreg(232);                         // A���� �������� ������ "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addHreg(233);                         // A���� �������� ������ "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addHreg(234);                         // A���� �������� ������ "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addHreg(235);                         // A���� �������� ������ "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addHreg(236);                         // A���� �������� ������ "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addHreg(237);                         // A���� �������� ������ "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addHreg(238);                         // A���� �������� ������ "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addHreg(239);                         // A���� �������� ������ ADC0  ��� x1 

	mb.addHreg(240);                         // A���� �������� ������ "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addHreg(241);                         // A���� �������� ������ "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addHreg(242);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addHreg(243);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addHreg(244);                         // A���� �������� ������ "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addHreg(245);                         // A���� �������� ������ "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addHreg(246);                         // A���� �������� ������ "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addHreg(247);                         // A���� �������� ������ "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addHreg(248);                         // A���� �������� ������ "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addHreg(249);                         // A���� �������� ������ ADC2 ��� x10

	mb.addHreg(250);                         // A���� �������� ������ "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addHreg(251);                         // A���� �������� ������ "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addHreg(252);                         // A���� �������� ������ "Test MTT ** Signal LineL                                    OFF - ";
	mb.addHreg(253);                         // A���� �������� ������ "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addHreg(254);                         // A���� �������� ������ "Test MTT ** Signal mag radio                                OFF - ";
	mb.addHreg(255);                         // A���� �������� ������ "Test MTT ** Signal mag phone                                OFF - ";
	mb.addHreg(256);                         // A���� �������� ������ "Test MTT ** Signal GGS                                      OFF - ";
	mb.addHreg(257);                         // A���� �������� ������ "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addHreg(258);                         // A���� �������� ������ "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addHreg(259);                         // A���� �������� ������ "Test MTT ** Signal GGS                                      ON  - ";

	mb.addHreg(260);                         // A���� �������� ������ "Test MTT ** Signal LineL                                    ON  - ";
	mb.addHreg(261);                         // A���� �������� ������ "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addHreg(262);                         // A���� �������� ������ "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addHreg(263);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addHreg(264);                         // A���� �������� ������ "Test microphone PTT  (CTS)                                  OFF - ";
	mb.addHreg(265);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addHreg(266);                         // A���� �������� ������ "Test microphone PTT  (CTS)                                  ON  - ";
	mb.addHreg(267);                         // A���� �������� ������ "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addHreg(268);                         // A���� �������� ������ "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addHreg(269);                         // A���� �������� ������ ������������ ����������� �������

	mb.addHreg(270);                         // A���� �������� ������ "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addHreg(271);                         // A���� �������� ������ "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addHreg(272);                         // A���� �������� ������ "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addHreg(273);                         // A���� �������� ������ "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addHreg(274);                         // A���� �������� ������ "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addHreg(275);                         // A���� �������� ������ "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addHreg(276);                         // A���� �������� ������ "Command sensor tangenta nognaja                             OFF - ";
	mb.addHreg(277);                         // A���� �������� ������ "Command sensor tangenta nognaja                             ON  - ";
	mb.addHreg(278);                         // A���� �������� ������ "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addHreg(279);                         // A���� �������� ������ "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addHreg(280);                         // A���� �������� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addHreg(281);                         // A���� �������� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addHreg(282);                         // A���� �������� ������ "Test GGS ** Signal LineL                                    OFF - ";
	mb.addHreg(283);                         // A���� �������� ������ "Test GGS ** Signal LineR                                    OFF - ";
	mb.addHreg(284);                         // A���� �������� ������ "Test GGS ** Signal mag radio                                OFF - ";
	mb.addHreg(285);                         // A���� �������� ������ "Test GGS ** Signal mag phone                                OFF - ";
	mb.addHreg(286);                         // A���� �������� ������ "Test GGS ** Signal GGS                                      OFF - ";
	mb.addHreg(287);                         // A���� �������� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addHreg(288);                         // A���� �������� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addHreg(289);                         // A���� �������� ������ "Test GGS ** Signal GGS                                      ON  - ";

	mb.addHreg(290);                         // A���� �������� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addHreg(291);                         // A���� �������� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addHreg(292);                         // A���� �������� ������ "Test GGS ** Signal mag phone                                ON  - ";
	mb.addHreg(293);                         // A���� ��������  ������ ADC1 ���������� 12/3 �����
	mb.addHreg(294);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio1
	mb.addHreg(295);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio2
	mb.addHreg(296);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� ���
	mb.addHreg(297);                         // A���� ��������  ������ ADC15 ���������� ���������� 3,6 ������
	mb.addHreg(298);                         // A���� �������� ������ "Test Microphone ** Signal mag phone                         ON  - ";    
	mb.addHreg(299);                         // A���� �������� ������ "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addHreg(300);                         // A���� �������� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addHreg(301);                         // A���� �������� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addHreg(302);                         // A���� �������� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addHreg(303);                         // A���� �������� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addHreg(304);                         // A���� �������� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addHreg(305);                         // A���� �������� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addHreg(306);                         // A���� �������� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addHreg(307);                         // A���� �������� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(308);                         // A���� �������� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(309);                         // A���� �������� ������ "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addHreg(310);                         // A���� �������� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addHreg(311);                         // A���� �������� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addHreg(312);                         // A���� �������� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addHreg(313);                         // A���� �������� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addHreg(314);                         // A���� �������� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addHreg(315);                         // A���� �������� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addHreg(316);                         // A���� �������� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addHreg(317);                         // A���� �������� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(318);                         // A���� �������� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(319);                         // A���� �������� ������ "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addHreg(320);                         // A���� �������� ������ "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addHreg(321);                         // A���� �������� ������ "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addHreg(322);                         // A���� �������� ������ "Test Microphone ** Signal LineL                             OFF - ";
	mb.addHreg(323);                         // A���� �������� ������ "Test Microphone ** Signal LineR                             OFF - ";
	mb.addHreg(324);                         // A���� �������� ������ "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addHreg(325);                         // A���� �������� ������ "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addHreg(326);                         // A���� �������� ������ "Test Microphone ** Signal GGS                               OFF - ";
	mb.addHreg(327);                         // A���� �������� ������ "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addHreg(328);                         // A���� �������� ������ "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addHreg(329);                         // A���� �������� ������ ��� ����������� �������                             // 

	mb.addHreg(330);                         // A���� �������� ������ "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addHreg(331);                         // A���� �������� ������ "Test Radio2 ** Signal mag radio                             ON  - ";
	mb.addHreg(332);                         // A���� �������� ������ "Test GGS    ** Signal mag radio                             ON  - ";


	
	// ++++++++++++++++++++++ �������� �������� ������ ��� �������� ������� ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	mb.addHreg(400);                         // A���� ���������� ADC0  ��� x1 
	mb.addHreg(401);                         // A���� ���������� ADC1 ���������� 12/3 �����
	mb.addHreg(402);                         // A���� ���������� ADC2 ��� x10
	mb.addHreg(403);                         // A���� ���������� ADC14 ���������� 12/3 ����� Radio1
	mb.addHreg(404);                         // A���� ���������� ADC14 ���������� 12/3 ����� Radio2
	mb.addHreg(405);                         // A���� ���������� ADC14 ���������� 12/3 ����� ���
	mb.addHreg(406);                         // A���� ���������� ADC15 ���������� ���������� 3,6 ������
	mb.addHreg(407);                         // A���� 
	mb.addHreg(408);                         // A���� 
	mb.addHreg(409);                         // A����  

	mb.addHreg(410);                         // A���� �������� 
	mb.addHreg(411);                         // A���� ��������  
	mb.addHreg(412);                         // A���� ��������  
	mb.addHreg(413);                         // A���� ��������  
	mb.addHreg(414);                         // A���� ��������  
	mb.addHreg(415);                         // A���� ��������  
	mb.addHreg(416);                         // A���� ��������  
	mb.addHreg(417);                         // A���� ��������  
	mb.addHreg(418);                         // A���� ��������  
	mb.addHreg(419);                         // A���� ��������  

	mb.addHreg(420);                         // A����  ;
	mb.addHreg(421);                         // A����  ;
	mb.addHreg(422);                         // A����  ;
	mb.addHreg(423);                         // A����  ;
	mb.addHreg(424);                         // A���� ������ ��������� "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addHreg(425);                         // A���� ������ ��������� "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addHreg(426);                         // A���� ������ ��������� "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addHreg(427);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addHreg(428);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addHreg(429);                         // A���� ������ ��������� "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addHreg(430);                         // A���� ������ ��������� "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addHreg(431);                         // A���� ������ ��������� "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addHreg(432);                         // A���� ������ ��������� "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addHreg(433);                         // A���� ������ ��������� "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addHreg(434);                         // A���� ������ ��������� "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addHreg(435);                         // A���� ������ ��������� "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addHreg(436);                         // A���� ������ ��������� "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addHreg(437);                         // A���� ������ ��������� "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addHreg(438);                         // A���� ������ ��������� "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addHreg(439);                         // A���� ������ ��������� ADC0  ��� x1 

	mb.addHreg(440);                         // A���� ������ ��������� "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addHreg(441);                         // A���� ������ ��������� "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addHreg(442);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addHreg(443);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addHreg(444);                         // A���� ������ ��������� "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addHreg(445);                         // A���� ������ ��������� "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addHreg(446);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addHreg(447);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addHreg(448);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addHreg(449);                         // A���� ������ ��������� ADC2 ��� x10

	mb.addHreg(450);                         // A���� ������ ��������� "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addHreg(451);                         // A���� ������ ��������� "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addHreg(452);                         // A���� ������ ��������� "Test MTT ** Signal LineL                                    OFF - ";
	mb.addHreg(453);                         // A���� ������ ��������� "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addHreg(454);                         // A���� ������ ��������� "Test MTT ** Signal mag radio                                OFF - ";
	mb.addHreg(455);                         // A���� ������ ��������� "Test MTT ** Signal mag phone                                OFF - ";
	mb.addHreg(456);                         // A���� ������ ��������� "Test MTT ** Signal GGS                                      OFF - ";
	mb.addHreg(457);                         // A���� ������ ��������� "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addHreg(458);                         // A���� ������ ��������� "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addHreg(459);                         // A���� ������ ��������� "Test MTT ** Signal GGS                                      ON  - ";

	mb.addHreg(460);                         // A���� ������ ��������� "Test MTT ** Signal LineL                                    ON  - ";
	mb.addHreg(461);                         // A���� ������ ��������� "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addHreg(462);                         // A���� ������ ��������� "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addHreg(463);                         // A���� ������ ��������� "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addHreg(464);                         // 
	mb.addHreg(465);                         // A���� ������ ��������� "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addHreg(466);                         // 
	mb.addHreg(467);                         // A���� ������ ��������� "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addHreg(468);                         // A���� ������ ��������� "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addHreg(469);                         // ������������ �������� ����������� ������� �������

	mb.addHreg(470);                         // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addHreg(471);                         // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addHreg(472);                         // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addHreg(473);                         // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addHreg(474);                         // A���� ������ ��������� "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addHreg(475);                         // A���� ������ ��������� "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addHreg(476);                         // A���� ������ ��������� "Command sensor tangenta nognaja                             OFF - ";
	mb.addHreg(477);                         // A���� ������ ��������� "Command sensor tangenta nognaja                             ON  - ";
	mb.addHreg(478);                         // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addHreg(479);                         // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addHreg(480);                         // A���� ������ ��������� "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addHreg(481);                         // A���� ������ ��������� "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addHreg(482);                         // A���� ������ ��������� "Test GGS ** Signal LineL                                    OFF - ";
	mb.addHreg(483);                         // A���� ������ ��������� "Test GGS ** Signal LineR                                    OFF - ";
	mb.addHreg(484);                         // A���� ������ ��������� "Test GGS ** Signal mag radio                                OFF - ";
	mb.addHreg(485);                         // A���� ������ ��������� "Test GGS ** Signal mag phone                                OFF - ";
	mb.addHreg(486);                         // A���� ������ ��������� "Test GGS ** Signal GGS                                      OFF - ";
	mb.addHreg(487);                         // A���� ������ ��������� "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addHreg(488);                         // A���� ������ ��������� "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addHreg(489);                         // A���� ������ ��������� "Test GGS ** Signal GGS                                      ON  - ";

	mb.addHreg(490);                         // A���� ������ ��������� "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addHreg(491);                         // A���� ������ ��������� "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addHreg(492);                         // A���� ������ ��������� "Test GGS ** Signal mag phone                                ON  - ";
	mb.addHreg(493);                         // A���� ������ ��������� ADC1 ���������� 12/3 �����
	mb.addHreg(494);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio1
	mb.addHreg(495);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio2
	mb.addHreg(496);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� ���
	mb.addHreg(497);                         // A���� ������ ��������� ADC15 ���������� ���������� 3,6 ������
	mb.addHreg(498);                         // A���� ������ ��������� "Test Microphone ** Signal mag phone                         ON  - "; 
	mb.addHreg(499);                         // A���� ������ ��������� "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addHreg(500);                         // A���� ������ ��������� "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addHreg(501);                         // A���� ������ ��������� "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addHreg(502);                         // A���� ������ ��������� "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addHreg(503);                         // A���� ������ ��������� "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addHreg(504);                         // A���� ������ ��������� "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addHreg(505);                         // A���� ������ ��������� "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addHreg(506);                         // A���� ������ ��������� "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addHreg(507);                         // A���� ������ ��������� "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(508);                         // A���� ������ ��������� "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(509);                         // A���� ������ ��������� "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addHreg(510);                         // A���� ������ ��������� "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addHreg(511);                         // A���� ������ ��������� "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addHreg(512);                         // A���� ������ ��������� "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addHreg(513);                         // A���� ������ ��������� "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addHreg(514);                         // A���� ������ ��������� "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addHreg(515);                         // A���� ������ ��������� "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addHreg(516);                         // A���� ������ ��������� "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addHreg(517);                         // A���� ������ ��������� "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(518);                         // A���� ������ ��������� "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(519);                         // A���� ������ ��������� "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addHreg(520);                         // A���� ������ ��������� "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addHreg(521);                         // A���� ������ ��������� "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addHreg(522);                         // A���� ������ ��������� "Test Microphone ** Signal LineL                             OFF - ";
	mb.addHreg(523);                         // A���� ������ ��������� "Test Microphone ** Signal LineR                             OFF - ";
	mb.addHreg(524);                         // A���� ������ ��������� "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addHreg(525);                         // A���� ������ ��������� "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addHreg(526);                         // A���� ������ ��������� "Test Microphone ** Signal GGS                               OFF - ";
	mb.addHreg(527);                         // A���� ������ ��������� "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addHreg(528);                         // A���� ������ ��������� "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addHreg(529);                         // ��� ����������� ������� �������
	mb.addHreg(530);                         // A���� ������ ��������� "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addHreg(531);                         // A���� ������ ��������� "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	mb.addHreg(532);                         // A���� ������ ��������� "Test GGS ** Signal mag radio   

//
//	
//	//regBank.set(40004+buffer,Serial1.read());
//
	mb.Coil(21,0);                              // XP2-2     sensor "���."  
	mb.Coil(22,0);                              // XP5-3     sensor "��C."
	mb.Coil(23,0);                              // XP3-3     sensor "��-�����1."
	mb.Coil(24,0);                              // XP4-3     sensor "��-�����2."

//	UpdateRegs();                                   // �������� ���������� � ���������

	for (int i = 120; i <= 131; i++)                  // �������� ����� ������
	{
	   mb.Coil(i,0);   
	}

	for (int i = 200; i <= 330; i++)                  // �������� ����� ������
	{
	  mb.Coil(i,0);   
	}
	
	for (unsigned int i = 200; i <= 330; i++)         // �������� ����� ������
	{
	   mb.Hreg(i,0);   
	}
		for (unsigned int i = 400; i <= 530; i++)     // �������� ����� ������
	{
	   mb.Hreg(i,0);   
	} 

	mb.Hreg(120,0);                                  // 
	mb.Hreg(adr_reg_count_err,0);                    // �������� ������ �������� ���� ������
}
void setup_resistor()
{ 
	Wire.beginTransmission(address_AD5252);      // transmit to device
	Wire.write(byte(control_word1));             // sends instruction byte  
	Wire.write(0);                               // sends potentiometer value byte  
	Wire.endTransmission();                      // stop transmitting
	Wire.beginTransmission(address_AD5252);      // transmit to device
	Wire.write(byte(control_word2));             // sends instruction byte  
	Wire.write(0);                               // sends potentiometer value byte  
	Wire.endTransmission();                      // stop transmitting
}


void setup()
{
	Serial.begin(9600);                                        // ����������� � USB ��
	Serial1.begin(115200);                                     // ����������� � ��������� ������ ��������
	Serial2.begin(38400);                                      // 
	mb.config(&Serial3, 19200, SERIAL_8N1);                   // Config Modbus Serial (port, speed, byte format) 

	Serial.println(" ");
	Serial.println(" ***** Start system  *****");
	Serial.println(" ");

	setup_regModbus();                                        // ��������� ��������� MODBUS

	Wire.begin();
	if (!RTC.begin())                                         // ��������� ����� 
		{
			Serial.println("RTC failed");
			while(1);
		};
	// DateTime set_time = DateTime(15, 6, 15, 10, 51, 0);    // ������� ������ � ������� � ������ "set_time"
	// RTC.adjust(set_time);                                  // ������
	serial_print_date();
	Serial.println(" ");
	MsTimer2::set(30, flash_time);                            // 30ms ������ ������� ���������
	setup_mcp();
	mcp_Analog.digitalWrite(DTR, HIGH);                       // ���������� ������ (������)���������� � ����������
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, HIGH); 

	pinMode(ledPin13, OUTPUT);  
	pinMode(ledPin12, OUTPUT);  
	pinMode(ledPin11, OUTPUT);  
	pinMode(ledPin10, OUTPUT);  
	pinMode(kn1Nano, OUTPUT);                        // ���������� ������ ���������� Nano ��������� �������
	pinMode(kn2Nano, OUTPUT);                        // ���������� ������ ���������� Nano ��������� 1000 ��
	pinMode(kn3Nano, OUTPUT);                        // ���������� ������ ���������� Nano ��������� 2000 ��
	pinMode(InNano12, INPUT);                        // ���������� ������ - ��������� ��������� 1000 ��� 2000 ��
	pinMode(InNano13, INPUT);                        // ���������� ������ - ��������� ��������� ������� 
 
	digitalWrite(kn1Nano, LOW);
	digitalWrite(kn2Nano, HIGH);
	digitalWrite(kn3Nano, HIGH);

	//********* ��������� ��������� ������ **************************
	AD9850.reset();                                  //reset module
	delay(500);
	AD9850.powerDown();                              //set signal output to LOW
	delay(100);
	AD9850.set_frequency(0,0,1000);                  //set power=UP, phase=0, 1kHz frequency
	delay(1000); 
	//-------------------------------------------------------------------
	
	setup_resistor();                               // ��������� ��������� ���������

	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 196 �������� � �������� �����
	regs_out[2]= 0x7F;                              // 127 �������� � �������� �����

	#if FASTADC                                     // �������� ���������� ����������� ������
	// set prescale to 16
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
	#endif
	Serial.println("Initializing SD card...");
	pinMode(49, OUTPUT);//    �������� 
	if (!sd.begin(chipSelect)) 
		{
			Serial.println("initialization SD failed!");
			mb.Coil(125,false); 
		}
	else
		{
			Serial.println("initialization SD successfully.");
			mb.Coil(125,true); 
		}

	SdFile::dateTimeCallback(dateTime);             // ��������� ������� ������ �����
	// Serial.println("Files found on the card (name, date and size in bytes): ");
	// list all files in the card with date and size
	// sd.ls (LS_R | LS_DATE | LS_SIZE);
	preob_num_str();                                 // �������� ��������� ��� ����� 
	list_file();                                     // ����� ������ ������ � ��� ����  






	//MsTimer2::start();                                        // �������� ������ ����������
	
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	Serial.println(" ");                                      //
	Serial.println(" ***** System initialization OK!. ****"); // ���������� � ���������� ���������

	//wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
}

void loop()
{
	mb.task();
 

}
