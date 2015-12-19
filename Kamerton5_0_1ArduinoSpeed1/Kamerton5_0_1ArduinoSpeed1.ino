/*

 Kamerton5_0_1ArduinoSpeed1.ino
 VisualStudio
 
 ��������� ������������ ������ "��������" (������� �������)
 ������:      - 5.0.1
 ����:        - 19.12.2015�.
 �����������: - ��� "������"
 �����:       - �������� �.�.
 ������: ����������� ������ ��  19.12.2015�.. ����� ����� � ������ ���������� 
 ���������������  ������ ��� ��������� �� ������.
 �����������: ����� �������� MODBUS
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
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <Wire.h> 
#include <RTClib.h>
#include <MsTimer2.h> 
#include "MCP23017.h"
#include <avr/pgmspace.h>
#include <AH_AD9850.h>
#include <avr/wdt.h>


//AH_AD9850(int CLK, int FQUP, int BitData, int RESET);
//AH_AD9850 AD9850(6, 7, 8, 9);
AH_AD9850 AD9850(23, 25, 27, 29);


#define  ledPin13  13                               // ���������� ����������� �� �����
#define  ledPin12  12                               // ���������� ����������� �� �����
#define  ledPin11  11                               // ���������� ����������� �� �����
#define  ledPin10  10                               // ���������� ����������� �� �����
#define  Front_led_Blue 14                          // ���������� ����������� �� �������� ������
#define  Front_led_Red  15                          // ���������� ����������� �� �������� ������

//+++++++++++++++++++++++++++++ ������� ������ +++++++++++++++++++++++++++++++++++++++
int deviceaddress        = 80;                      // ����� ���������� ������
unsigned int eeaddress   =  0;                      // ����� ������ ������
byte hi;                                            // ������� ���� ��� �������������� �����
byte low;                                           // ������� ���� ��� �������������� �����


//  ����� ���������� ������ ��������
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




//+++++++++++++++++++++++ ��������� ������������ ��������� +++++++++++++++++++++++++++++++++++++
#define address_AD5252   0x2F                       // ����� ���������� AD5252  
#define control_word1    0x07                       // ���� ���������� �������� �1
#define control_word2    0x87                       // ���� ���������� �������� �2
byte resistance        = 0x00;                      // ������������� 0x00..0xFF - 0��..100���
//byte level_resist      = 0;                       // ���� ��������� ������ �������� ���������
//-----------------------------------------------------------------------------------------------
unsigned int volume1     = 0;                       //
unsigned int volume_max  = 0;                       //
unsigned int volume_min  = 0;                       //
unsigned int volume_fact = 0;                       //
unsigned int Array_volume[260];                     //
unsigned int Array_min[40];                         //
unsigned int Array_max[40];                         //
unsigned int volume_porog_D = 40;                   // ������������ �������� ������ ��� �������� ����������� FrontL,FrontR
unsigned int volume_porog_L = 200;                  // ����������� �������� ������ ��� �������� ����������� FrontL,FrontR
float voltage ;
//float voltage_test = 0.60;                        // ����� �������� ��������� �����
unsigned int  voltage10 ;
unsigned long number_audio = 0;   


#define FASTADC 1                                   // ��������� ���������� ����������� �������
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//***************************** ���������� ���������� ������   ****************************************
int analog_tok            = 0;       //   ��������� ���� ������� ����� ��������
int analog_12V            = 1;       //   ��������� ���������� ������� 12�. ����� ��������
int analog_tok_x10        = 2;       //   ��������� ���� ������� ����� �������� � 10
int analog_mag_radio      = 3;       //
int analog_mag_phone      = 4;       //
int analog_gg_radio1      = 5;       //
int analog_gg_radio2      = 6;       //
int analog_ggs            = 7;       //
int analog_LineL          = 8;       //
int analog_LineR          = 9;       //
int analog_FrontL         = 10;       //
int analog_FrontR         = 11;       //
int analog_W              = 12;       //
int analog_13             = 13;       // ��������� ���������� �������  12�.�� ��������  ����� ��������
int analog_14             = 14;       // ��������� ���������� �������  12�.�� ��������  ����� ��������
int analog_3_6            = 15;       // ��������� ���������� ������� 3,6�. �� �������� ����� ��������

//-----------------------------------------------------------------------------------
bool portFound = false;
byte inputByte_0;
byte inputByte_1;
byte inputByte_2;
byte inputByte_3;
byte inputByte_4;
uint32_t logTime = 0;
int32_t diff = 0;

bool blink_red = false;
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
MCP23017 mcp_Out1;                                  // ���������� ������ ���������� MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                  // ���������� ������ ���������� MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                // ���������� ������ ���������� MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------
const int adr_reg_ind_CTS      PROGMEM           = 81;        // ����� ����a ��������� ��������� ������� CTS
const int adr_reg_ind_DSR      PROGMEM           = 82;        // ����� ����a ��������� ��������� ������� DSR
const int adr_reg_ind_DCD      PROGMEM           = 83;        // ����� ����a ��������� ��������� ������� DCD

// **************** ������ ������� ������ ��� �������� ����. ����������� ��������������� ����� ����� *************
//const int adr_temp_day         PROGMEM           = 240;          // ����� �������� ���������� ����
//const int adr_temp_mon         PROGMEM           = 241;          // ����� �������� ���������� �����
//const int adr_temp_year        PROGMEM           = 242;          // ����� �������� ���������� ���  
//const int adr_file_name_count  PROGMEM           = 243;          // ����� �������� ���������� �������� ������ �����
//------------------------------------------------------------------------------------------------------------------
int regcount_err        = 0;                                     // ���������� ��� �������� ���� ������
	//	hi=highByte(n_str_electro);
	//	low=lowByte(n_str_electro);

//++++++++++++++++++++++ ������ � ������� +++++++++++++++++++++++++++++++++++++++
//#define chipSelect SS
#define chipSelect 49   // ��������
SdFat sd;
File myFile;
SdFile file;
//Sd2Card card;
//
//uint32_t cardSizeBlocks;
//uint16_t cardCapacityMB;
//
//// cache for SD block
//cache_t cache;


// ������� ����������, ������������ ������� ���������� SD utility library functions: +++++++++++++++
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;


//++++++++++++++++++++ ���������� ����� ����� ++++++++++++++++++++++++++++++++++++++++++++
//const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "150101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13]            = FILE_BASE_NAME "00.TXT";
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

//+++++++++++++++++++++ ��������� ���������� +++++++++++++++++++++++++++++++

unsigned int sampleCount1 = 0;

//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ModbusSerial mb;

//byte regs_in[5];                                    // �������� ������ � ������ �������� CPLL
byte regs_out[4];                                   // �������� ������ � ������ ��������
byte regs_crc[1];                                   // �������� ������ � ������ �������� ����������� �����
byte regs_temp = 0;
byte regs_temp1 = 0;
byte Stop_Kam = 0;                                  // ���� ��������� ������ ���. �� ���������
bool prer_Kmerton_On = true;                        // ���� ���������� ���������� ��������
bool test_repeat     = true;                        // ���� ���������� �����
volatile bool prer_Kmerton_Run = false;             // ���� ���������� ���������� ��������
#define BUFFER_SIZEK 64                             // ������ ������ �������� �� ����� 128 ����
#define BUFFER_SIZEKF 128                           // ������ ������ Serial2 �� ����� 128 ����
unsigned char bufferK;                              // ������� ���������� ����������� ����

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
const unsigned int adr_Mic_Start_day      PROGMEM      = 96; // ����� ����
const unsigned int adr_Mic_Start_month    PROGMEM      = 97; // ����� �����
const unsigned int adr_Mic_Start_year     PROGMEM      = 98; // ����� ���
const unsigned int adr_Mic_Start_hour     PROGMEM      = 99; // ����� ���
const unsigned int adr_Mic_Start_minute   PROGMEM      = 100; // ����� ������
const unsigned int adr_Mic_Start_second   PROGMEM      = 101; // ����� �������
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

const unsigned int adr_reg_temp_year      PROGMEM       = 112; // ������� �������� ���������� ���  
const unsigned int adr_reg_temp_mon       PROGMEM       = 113; // ������� �������� ���������� �����
const unsigned int adr_reg_temp_day       PROGMEM       = 114; // ������� �������� ���������� ���� 
const unsigned int adr_reg_file_name      PROGMEM       = 115; // ������� �������� ������� ������  
const unsigned int adr_reg_file_tek       PROGMEM       = 116; // ������� �������� ������� ������  

const unsigned int adr_control_command    PROGMEM       = 120; // ����� �������� ������� �� ���������� 
const unsigned int adr_reg_count_err      PROGMEM       = 121; // ����� �������� ���� ������

const unsigned int adr_set_time           PROGMEM       = 36;    // ����� ���� ���������

//------------------------- ������ ��������� �������� �������� ��� ������������ ���������--------------------------------------
//++++++++++++++++++++++++++++ ��������� ��������� ������� ������� +++++++++++++++++++++++++++++++++++++

// ������ ������� ������ ��� �������� ������� ������� ��������� ��������

const  int adr_porog_instruktor            = 0;     // 19 ������� 
const  int adr_porog_dispatcher            = 20;    // 19 ������� 
const  int adr_porog_MTT                   = 40;    // 21 ������� 
const  int adr_porog_GGS                   = 170;    // 29 ������� 
const  int adr_porog_Radio1                = 100;   // 20 ������� 
const  int adr_porog_Radio2                = 120;   // 20 ������� 
const  int adr_porog_Microphone            = 140;   //  
// end                                     = 160

//����� ������ ��� ������, 2 �����
const  int adr_int_porog_instruktor            = 200;     // 19 ������� 
const  int adr_int_porog_dispatcher            = 250;    // 19 ������� 
const  int adr_int_porog_MTT                   = 300;    // 21 ������� 
const  int adr_int_porog_GGS                   = 350;    // 29 ������� 
const  int adr_int_porog_Radio1                = 420;   // 20 ������� 
const  int adr_int_porog_Radio2                = 460;   // 20 ������� 
const  int adr_int_porog_Microphone            = 500;   //
//end                                            550;


byte por_buffer[30] ;

//const byte porog_instruktor[]    PROGMEM  = {30,30,35,35,35,35,35,35,35,35,35,150,150,35,35,35,35,35,35,35,254};
const byte porog_instruktor[]    PROGMEM  = {
		 //++++++++++++++++  Test headset instructor ++++++++++++++++++++++++++++
100,			// 0                          // resistor(1, 30);     ���������� ������� ������� 30 ��
100,			// 1                          // resistor(2, 30);     ���������� ������� ������� 30 ��
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
160,    	// 11                         // measure_vol_max(analog_LineL,    40224,224,150); ������� ������� �� ������ LineL
160,		// 12                         // measure_vol_max(analog_mag_phone,40226,226,150); ������� ������� �� ������ mag phone 
15, 		// 13                         // measure_vol_min(analog_FrontL,   40230,230,35);  ������� ������� �� ������ FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,   40231,231,35);  ������� ������� �� ������ FrontR 
15,			// 15                         // measure_vol_min(analog_LineR,    40233,233,35);  ������� ������� �� ������ LineR 
15,			// 16                         // measure_vol_min(analog_ggs,      40236,236,35);  ������� ������� �� ������ GGS 
15,			// 17                         // measure_vol_min(analog_gg_radio1,40237,237,35);  ������� ������� �� ������ GG Radio1
15  		// 18                         // measure_vol_min(analog_gg_radio2,40238,238,35);  ������� ������� �� ������ GG Radio2
};
  
const byte  porog_dispatcher[]    PROGMEM = {

		//	+++++++++++++Test headset dispatcher ++++++++++++++++++++++++++++++
100,			// 0                          // resistor(1, 30);   ���������� ������� ������� 30 ��
100,			// 1                          // resistor(2, 30);   ���������� ������� ������� 30 ��
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
160,		// 11                         // measure_vol_max(analog_LineL,    40227,227,200); ������� ������� �� ������ LineL
160,		// 12                         // measure_vol_max(analog_mag_phone,40229,229,200); ������� ������� �� ������ mag phone
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
150,		// 17                         // measure_vol_max(analog_LineL,    40260,260,35);  "Test MTT ** Signal LineL 
150,		// 18                         // measure_vol_max(analog_LineR,    40261,261,35);  "Test MTT ** Signal LineR 
15,         // 19                         // measure_vol_min(analog_ggs,       40256,256,por_buffer[19]);                    // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
120			// 20                         // measure_vol_max(analog_mag_phone,40262,262,90);  ������� ������� �� ������ mag phone
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
150,		// 11                         // measure_vol_max(analog_mag_phone, 40298,298,180);������� ������� �� ������ mag phone
150,		// 12                         // measure_vol_max(analog_LineL,     40299,299,180);������� ������� �� ������ "Test Microphone ** Signal LineL 
15,			// 13                         // measure_vol_min(analog_FrontL,    40320,320,35); ������� ������� �� ������ FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,    40321,321,35); ������� ������� �� ������ FrontR
15,			// 15                         // measure_vol_min(analog_LineR,     40323,323,35); ������� ������� �� ������ LineR
15,			// 16                         // measure_vol_min(analog_mag_radio, 40324,324,35); ������� ������� �� ������ mag radio 
15,			// 17                         // measure_vol_min(analog_ggs,       40326,326,35); ������� ������� �� ������ GGS
15,			// 18                         // measure_vol_min(analog_gg_radio1, 40327,327,35); ������� ������� �� ������ GG Radio1
15,         // 19                         // measure_vol_min(analog_gg_radio2, 40328,328,35); ������� ������� �� ������ GG Radio2
};

const byte  porog_GGS[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++ Test GGS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
200,		//  0                         // resistor(1, 200); ���������� ������� ������� xx ��
200,		//  1                         // resistor(2, 200); ���������� ������� ������� xx ��
				//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
35, 		//  2                         // measure_vol_min(analog_FrontL,    40280,280,35); ������� ������� �� ������ "Test GGS ** Signal FrontL 
35, 		//  3                         // measure_vol_min(analog_FrontR,    40281,281,35); ������� ������� �� ������ "Test GGS ** Signal FrontR        
35, 		//  4                         // measure_vol_min(analog_LineL,     40282,282,35); ������� ������� �� ������ "Test GGS ** Signal LineL 
35, 		//  5                         // measure_vol_min(analog_LineR,     40283,283,35); ������� ������� �� ������ "Test GGS ** Signal LineR 
35, 		//  6                         // measure_vol_min(analog_mag_radio, 40284,284,35); ������� ������� �� ������ "Test GGS ** Signal mag radio
35, 		//  7                         // measure_vol_min(analog_mag_phone, 40285,285,35); ������� ������� �� ������ "Test GGS ** Signal mag phone 
35, 		//  8                         // measure_vol_min(analog_ggs,       40286,286,35); ������� ������� �� ������ "Test GGS ** Signal GGS    
35, 		//  9                         // measure_vol_min(analog_gg_radio1, 40287,287,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio1   
35, 		//  10                        // measure_vol_min(analog_gg_radio2, 40288,288,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio2
			   //++++++++++++++++++++++++++++++++++ ������ ����� �� ���� GGS ++++++++++++++++++++++++
40,	    	// 11    					  //measure_vol_max(analog_FrontL,    40290,290,40); ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
40,	    	// 12  						  //measure_vol_max(analog_FrontR,    40291,291,40); ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";
35,			// 13						  //measure_vol_min(analog_LineL,     40282,282,35); ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
35,			// 14						  //measure_vol_min(analog_LineR,     40283,283,35); ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
50,	    	// 15						  //measure_vol_max(analog_mag_phone, 40292,292,50); ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
160,		// 16			     		  //measure_vol_max(analog_ggs,       40286,289,160); ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
35,			// 17						  //measure_vol_min(analog_gg_radio1, 40287,287,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
35,			// 18						  //measure_vol_min(analog_gg_radio2, 40288,288,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

40,			// 19						  //measure_vol_max(analog_FrontL,    40290,290,40); ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
40,			// 20 						  //measure_vol_max(analog_FrontR,    40291,291,40); ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";
35,			// 21					      //measure_vol_min(analog_LineL,     40282,282,35); ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
35,			// 22						  //measure_vol_min(analog_LineR,     40283,283,35); ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
35,			// 23						  //measure_vol_min(analog_mag_radio, 40284,284,35); ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
50,			// 24						  //measure_vol_max(analog_mag_phone, 40292,292,50); ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
35,			// 25						  //measure_vol_min(analog_ggs,       40286,286,35); ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
35,			// 26						  //measure_vol_min(analog_gg_radio1, 40287,287,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
35			// 27						  //measure_vol_min(analog_gg_radio2, 40288,288,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
};

const byte  porog_Radio1[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio1 +++++++++++++++++++++++++++++++++++++++++++++++++++++

200,		// 0							//resistor(1, 250);  ���������� ������� ������� xx ��
200,		// 1							//resistor(2, 250);  ���������� ������� ������� xx ��
				 //+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
35,			// 2							//measure_vol_min(analog_FrontL,    40300,300,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
35,			// 3							//measure_vol_min(analog_FrontR,    40301,301,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
35,			// 4							//measure_vol_min(analog_LineL,     40302,302,35); ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
35,			// 5							//measure_vol_min(analog_LineR,     40303,303,35); ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
35,			// 6							//measure_vol_min(analog_mag_radio, 40304,304,35); ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
35,			// 7							//measure_vol_min(analog_mag_phone, 40305,305,35); ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
35,			// 8							//measure_vol_min(analog_ggs,       40306,306,35); ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
35,			// 9							//measure_vol_min(analog_gg_radio1, 40307,307,35); ������� ������� �� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
35,			// 10							//measure_vol_min(analog_gg_radio2, 40308,308,35); �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
				  //++++++++++++++++++++++++++++++++++ ������ ����� �� ����  Radio1 ++++++++++++++++++++++++
35,			// 11							//measure_vol_min(analog_FrontL,    40300,300,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
35,			// 12							//measure_vol_min(analog_FrontR,    40301,301,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
35,			// 13							//measure_vol_min(analog_LineL,     40302,302,35); ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
35,			// 14							//measure_vol_min(analog_LineR,     40303,303,35); ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
35,			// 15							//measure_vol_min(analog_mag_radio, 40304,304,35); ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
35,			// 16							//measure_vol_min(analog_mag_phone, 40305,305,35); ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
35,			// 17							//measure_vol_min(analog_ggs,       40306,306,35); ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
250,		// 18							//measure_vol_max(analog_gg_radio1, 40309,309,250);������� ������� �� ������ "Test Radio1 ** Signal Radio1                                ON  - ";
35			// 19							//measure_vol_min(analog_gg_radio2, 40308,308,35); ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2       
};

const byte  porog_Radio2[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio2 +++++++++++++++++++++++++++++++++++++++++++++++++++++

200,		// 0							//resistor(1, 250);  ���������� ������� ������� xx ��
200,		// 1							//resistor(2, 250);  ���������� ������� ������� xx ��
				 //+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
35,			// 2							//measure_vol_min(analog_FrontL,    40310,310,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
35,			// 3							//measure_vol_min(analog_FrontR,    40311,311,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
35,			// 4							//measure_vol_min(analog_LineL,     40312,312,35); ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
35,			// 5							//measure_vol_min(analog_LineR,     40313,313,35); ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
35,			// 6 							//measure_vol_min(analog_mag_radio, 40314,314,35); ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
35,			// 7							//measure_vol_min(analog_mag_phone, 40315,315,35); ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
35,			// 8							//measure_vol_min(analog_ggs,       40316,316,35); ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
35,			// 9							//measure_vol_min(analog_gg_radio1, 40317,317,35); ������� ������� �� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
35,			// 10							//measure_vol_min(analog_gg_radio2, 40318,318,35); ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
					//++++++++++++++++++++++++++++++++++ ������ ����� �� ����  Radio2 ++++++++++++++++++++++++
35,			// 11							//measure_vol_min(analog_FrontL,    40310,310,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
35,			// 12							//measure_vol_min(analog_FrontR,    40311,311,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
35,			// 13							//measure_vol_min(analog_LineL,     40312,312,35); ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
35,			// 14							//measure_vol_min(analog_LineR,     40313,313,35); ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
35,			// 15							//measure_vol_min(analog_mag_radio, 40314,314,35); ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
35,			// 16							//measure_vol_min(analog_mag_phone, 40315,315,35); ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
35,			// 17							//measure_vol_min(analog_ggs,       40316,316,35); ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
35,			// 18							//measure_vol_min(analog_gg_radio1, 40317,317,35); ������� ������� �� ������ "Test Radio2 ** Signal Radio1                                ON  - ";
250			// 19							//measure_vol_max(analog_gg_radio2, 40319,319,250);������� ������� �� ������ "Test Radio2 ** Signal GG Radio2        
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
const char  txt_message55[]   PROGMEM            = "Signal mi�rophone 30  mV                      ON"            ;
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




//++++++++++++++++++++++++++++++ ������ ������ ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const char  txt_error0[]  PROGMEM              = "Sensor MTT                          XP1- 19 HaSs    OFF - ";
const char  txt_error1[]  PROGMEM              = "Sensor tangenta ruchnaja            XP7 - 2         OFF - ";
const char  txt_error2[]  PROGMEM              = "Sensor tangenta nognaja             XP8 - 2         OFF - "; 
const char  txt_error3[]  PROGMEM              = "Sensor headset instructor 2         XP1- 16 HeS2Rs  OFF - ";
const char  txt_error4[]  PROGMEM              = "Sensor headset instructor           XP1- 13 HeS2Ls  OFF - "; 
const char  txt_error5[]  PROGMEM              = "Sensor headset dispatcher 2         XP1- 5  HeS1Rs  OFF - "; 
const char  txt_error6[]  PROGMEM              = "Sensor headset dispatcher           XP1- 1  HeS1Ls  OFF - ";
const char  txt_error7[]  PROGMEM              = "Sensor microphone                   XS1 - 6         OFF - "; 
const char  txt_error8[]  PROGMEM              = "Microphone headset instructor Sw.   XP1 12 HeS2e    OFF - "; 
const char  txt_error9[]  PROGMEM              = "Microphone headset dispatcher Sw.   XP1 12 HeS2e    OFF - ";  

const char  txt_error10[]  PROGMEM             = "Sensor MTT                          XP1- 19 HaSs    ON  - "; 
const char  txt_error11[]  PROGMEM             = "Sensor tangenta ruchnaja            XP7 - 2         ON  - ";
const char  txt_error12[]  PROGMEM             = "Sensor tangenta nognaja             XP8 - 2         ON  - "; 
const char  txt_error13[]  PROGMEM             = "Sensor headset instructor 2         XP1- 16 HeS2Rs  ON  - ";
const char  txt_error14[]  PROGMEM             = "Sensor headset instructor           XP1- 13 HeS2Ls  ON  - "; 
const char  txt_error15[]  PROGMEM             = "Sensor headset dispatcher 2         XP1- 5  HeS1Rs  ON  - "; 
const char  txt_error16[]  PROGMEM             = "Sensor headset dispatcher           XP1- 1  HeS1Ls  ON  - ";
const char  txt_error17[]  PROGMEM             = "Sensor microphone                   XS1 - 6         ON  - "; 
const char  txt_error18[]  PROGMEM             = "Microphone headset instructor Sw.   XP1 12 HeS2e    ON  - "; 
const char  txt_error19[]  PROGMEM             = "Microphone headset dispatcher Sw.   XP1 10 HeS1e    ON  - "; 
  
const char  txt_error20[]  PROGMEM             = "Command PTT headset instructor (CTS)                OFF - ";
const char  txt_error21[]  PROGMEM             = "Command PTT headset instructor (CTS)                ON  - ";
const char  txt_error22[]  PROGMEM             = "Command PTT headset dispatcher (CTS)                OFF - ";
const char  txt_error23[]  PROGMEM             = "Command PTT headset dispatcher (CTS)                ON  - ";
const char  txt_error24[]  PROGMEM             = "Test headset instructor ** Signal LineL             ON  - ";
const char  txt_error25[]  PROGMEM             = "Test headset instructor ** Signal LineR             ON  - ";   
const char  txt_error26[]  PROGMEM             = "Test headset instructor ** Signal Mag phone         ON  - ";
const char  txt_error27[]  PROGMEM             = "Test headset dispatcher ** Signal LineL             ON  - ";
const char  txt_error28[]  PROGMEM             = "Test headset dispatcher ** Signal LineR             ON  - ";  
const char  txt_error29[]  PROGMEM             = "Test headset dispatcher ** Signal Mag phone         ON  - ";

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
const char  txt_error59[]  PROGMEM             = "Test MTT ** Signal GGS                              ON  - ";

const char  txt_error60[]  PROGMEM             = "Test MTT ** Signal LineL                            ON  - ";
const char  txt_error61[]  PROGMEM             = "Test MTT ** Signal LineR                            ON  - ";  
const char  txt_error62[]  PROGMEM             = "Test MTT ** Signal Mag phone                        ON  - ";
const char  txt_error63[]  PROGMEM             = "Test MTT PTT    (CTS)                               OFF - ";
const char  txt_error64[]  PROGMEM             = "Test microphone PTT  (CTS)                          OFF - ";
const char  txt_error65[]  PROGMEM             = "Test MTT PTT    (CTS)                               ON  - ";
const char  txt_error66[]  PROGMEM             = "Test microphone PTT  (CTS)                          ON  - ";
const char  txt_error67[]  PROGMEM             = "Test MTT HangUp (DCD)                               OFF - ";
const char  txt_error68[]  PROGMEM             = "Test MTT HangUp (DCD)                               ON  - ";
const char  txt_error69[]  PROGMEM             = "";

const char  txt_error70[]  PROGMEM             = "Command PTT1 tangenta ruchnaja (CTS)                OFF - ";
const char  txt_error71[]  PROGMEM             = "Command PTT2 tangenta ruchnaja (DCR)                OFF - ";
const char  txt_error72[]  PROGMEM             = "Command PTT1 tangenta ruchnaja (CTS)                ON  - ";
const char  txt_error73[]  PROGMEM             = "Command PTT2 tangenta ruchnaja (DCR)                ON  - ";
const char  txt_error74[]  PROGMEM             = "Command sensor tangenta ruchnaja    XP7 - 2         OFF - ";
const char  txt_error75[]  PROGMEM             = "Command sensor tangenta ruchnaja    XP7 - 2         ON  - ";
const char  txt_error76[]  PROGMEM             = "Command sensor tangenta nognaja     XP8 - 2         OFF - ";
const char  txt_error77[]  PROGMEM             = "Command sensor tangenta nognaja     XP8 - 2         ON  - ";
const char  txt_error78[]  PROGMEM             = "Command PTT tangenta nognaja (CTS)  XP8 - 1         OFF - ";
const char  txt_error79[]  PROGMEM             = "Command PTT tangenta nognaja (CTS)  XP8 - 1         ON  - ";

const char  txt_error80[]  PROGMEM             = "Test GGS ** Signal FrontL                           OFF - ";
const char  txt_error81[]  PROGMEM             = "Test GGS ** Signal FrontR                           OFF - ";
const char  txt_error82[]  PROGMEM             = "Test GGS ** Signal LineL                            OFF - ";
const char  txt_error83[]  PROGMEM             = "Test GGS ** Signal LineR                            OFF - ";
const char  txt_error84[]  PROGMEM             = "Test GGS ** Signal mag radio                        OFF - ";
const char  txt_error85[]  PROGMEM             = "Test GGS ** Signal mag phone                        OFF - ";
const char  txt_error86[]  PROGMEM             = "Test GGS ** Signal GGS                              OFF - ";
const char  txt_error87[]  PROGMEM             = "Test GGS ** Signal GG Radio1                        OFF - ";
const char  txt_error88[]  PROGMEM             = "Test GGS ** Signal GG Radio2                        OFF - ";
const char  txt_error89[]  PROGMEM             = "Test GGS ** Signal GGS                              ON  - ";

const char  txt_error90[]  PROGMEM             = "Test GGS ** Signal FrontL                           ON  - ";
const char  txt_error91[]  PROGMEM             = "Test GGS ** Signal FrontR                           ON  - ";
const char  txt_error92[]  PROGMEM             = "Test GGS ** Signal mag phone                        ON  - ";
const char  txt_error93[]  PROGMEM             = "";  
const char  txt_error94[]  PROGMEM             = "";
const char  txt_error95[]  PROGMEM             = ""; 
const char  txt_error96[]  PROGMEM             = "";    
const char  txt_error97[]  PROGMEM             = "";  
const char  txt_error98[]  PROGMEM             = "Test Microphone ** Signal mag phone                 ON  - ";  
const char  txt_error99[]  PROGMEM             = "Test Microphone ** Signal LineL                     ON  - "; 

const char  txt_error100[]  PROGMEM            = "Test Radio1 ** Signal FrontL                        OFF - ";
const char  txt_error101[]  PROGMEM            = "Test Radio1 ** Signal FrontR                        OFF - ";
const char  txt_error102[]  PROGMEM            = "Test Radio1 ** Signal LineL                         OFF - ";
const char  txt_error103[]  PROGMEM            = "Test Radio1 ** Signal LineR                         OFF - ";
const char  txt_error104[]  PROGMEM            = "Test Radio1 ** Signal mag radio                     OFF - ";
const char  txt_error105[]  PROGMEM            = "Test Radio1 ** Signal mag phone                     OFF - ";
const char  txt_error106[]  PROGMEM            = "Test Radio1 ** Signal GGS                           OFF - ";
const char  txt_error107[]  PROGMEM            = "Test Radio1 ** Signal GG Radio1                     OFF - ";
const char  txt_error108[]  PROGMEM            = "Test Radio1 ** Signal GG Radio2                     OFF - ";
const char  txt_error109[]  PROGMEM            = "Test Radio1 ** Signal Radio1                        ON  - ";

const char  txt_error110[]  PROGMEM            = "Test Radio2 ** Signal FrontL                        OFF - ";
const char  txt_error111[]  PROGMEM            = "Test Radio2 ** Signal FrontR                        OFF - ";
const char  txt_error112[]  PROGMEM            = "Test Radio2 ** Signal LineL                         OFF - ";
const char  txt_error113[]  PROGMEM            = "Test Radio2 ** Signal LineR                         OFF - ";
const char  txt_error114[]  PROGMEM            = "Test Radio2 ** Signal mag radio                     OFF - ";
const char  txt_error115[]  PROGMEM            = "Test Radio2 ** Signal mag phone                     OFF - ";
const char  txt_error116[]  PROGMEM            = "Test Radio2 ** Signal GGS                           OFF - ";
const char  txt_error117[]  PROGMEM            = "Test Radio2 ** Signal GG Radio1                     OFF - ";
const char  txt_error118[]  PROGMEM            = "Test Radio2 ** Signal GG Radio2                     OFF - ";
const char  txt_error119[]  PROGMEM            = "Test Radio2 ** Signal Radio2                        ON  - ";

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
txt_message70                                 // "Adjusting the brightness mks                               - "   ;
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
		prer_Kmerton_Run = true;
	//	digitalWrite(ledPin12,HIGH);
		prer_Kamerton();
		 mb.task();
	//	digitalWrite(ledPin12,LOW);
		prer_Kmerton_Run = false;
}

void serialEvent3()
{
	/* while (prer_Kmerton_Run){}
	 mb.task();*/
}
//fileName_F
void serialEvent2()
{

}
int readString(char *buffer, int max_len, int terminator)
{
  int pos = 0;

  while (true) {
	if (Serial2.available() > 0) 
	{
	  int readch = Serial2.read();
	  if (readch == terminator) 
	  {
		buffer[pos] = '\0';
		break;
	  }
	  else if (readch != '\n' && pos < max_len-1) 
	  {  // Ignore new-lines
		buffer[pos++] = readch;
		fileName_F[pos++] = readch;
	  }
	}
  }
  return pos;
}

void read_Serial2()
{
	if (Serial2.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {
			unsigned char overflowFlag = 0 ;               // ���� ���������� ������� ������
			unsigned char buffer_count = 0;                      // ���������� � ������ ������ ������

			while (Serial2.available())
				{
				  if (overflowFlag)                        // ���� ����� ���������� - ��������
					 Serial2.read();
				  else                                     // ������ ������ � �����, ������� ����������
					{
					if (buffer_count == BUFFER_SIZEKF)           // ��������� ������ ������
						{
							overflowFlag = 1;              // ���������� ���� ���������� ������� ������
						}
							 bufferSerial2[buffer_count] = Serial2.read(); 
						buffer_count++;
					}
				}
		   }
	 else 
		{
	
		}
	
	for (int i = 0; i < 13; i++)
	{
        fileName_F[i] = bufferSerial2[i];
    }

	Serial.println(bufferSerial2);
	Serial.println(fileName_F);
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
			mb.Hreg(1+i,regs_out[i]);
		}
}
void waiting_for_replyK()                                  // ������ ������ �� ���������
{
	delayMicroseconds(5);

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
						 mb.Hreg(4+buffer,Serial1.read());
						//regs_in[buffer] = Serial1.read(); 
						buffer++;
					}
				}
//			calculateCRC_In();
			mb.Coil(124,1);                              // ����� � "��������" �����������
		   }
	 else 
		{
			Stop_Kam = 0;                                    // ���� ��������. ���. �� ���������
			mb.Coil(124,0);                              // ���� ������  ����� � "��������"
		}

	  //if( mb.Hreg(40007) != regs_temp)
	  //{
		 //Serial.println(mb.Hreg(40004),BIN);
		 //Serial.println(mb.Hreg(40006),BIN);
		 //Serial.println(mb.Hreg(40007),BIN);
	  //}
   //   regs_temp = mb.Hreg(40007);

}
void Stop_Kamerton ()                  //���� �� �������� ���������� � ��������� - �������� ��������
  {
	 for (unsigned char i = 0; i <4; i++)
	mb.Hreg(4+i,0);
	// regs_in[i]=0;
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
		  mb.Hreg(adr_reg_temp_day,day_temp);  
		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon);                             //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		  mb.Hreg(adr_reg_temp_mon,mon_temp); 
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year);                            //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		 mb.Hreg(adr_reg_temp_year,year_temp); 

		  b = i2c_eeprom_read_byte(0x50, adr_file_name_count);                             //access an address from the memory
		  mb.Hreg(adr_reg_file_name,b);                                                // �������  �������� ���������� ����� �����
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
void load_list_files()
{
	//wdt_reset();
	if (!sd.begin(chipSelect)) 
		{
			Serial.println("initialization SD failed!");
		}
	else
		{
	
		while (file.openNext(sd.vwd(), O_READ))
		  {
			file.printName(&Serial2);
			Serial2.println();
			file.printName(&Serial);
			Serial.println();
	    	file.close();
		  } 
		   Serial2.flush();
		 }
		delay(1000);
		Serial.println("Files end");
  		mb.Hreg(adr_control_command,0);
}

void file_print_date()  //���������  ������ ���� � ����
	{
	  DateTime now = RTC.now();
	  myFile.print(now.day(), DEC);
	  myFile.print('/');
	  myFile.print(now.month(), DEC);
	  myFile.print('/');
	  myFile.print(now.year(), DEC);//Serial display time
	  myFile.print(' ');
	  myFile.print(now.hour(), DEC);
	  myFile.print(':');
	  myFile.print(now.minute(), DEC);
	  myFile.print(':');
	  myFile.print(now.second(), DEC);
  }
//void serial_print_date()                           // ������ ���� � �������    
//{
//	  DateTime now = RTC.now();
//	  Serial.print(now.day(), DEC);
//	  Serial.print('/');
//	  Serial.print(now.month(), DEC);
//	  Serial.print('/');
//	  Serial.print(now.year(), DEC);//Serial display time
//	  Serial.print(' ');
//	  Serial.print(now.hour(), DEC);
//	  Serial.print(':');
//	  Serial.print(now.minute(), DEC);
//	  Serial.print(':');
//	  Serial.print(now.second(), DEC);
//}
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

void controlFileName()
{
  int temp_file_name = 0;
  preob_num_str();
  while (sd.exists(fileName)) 
  {
	if (fileName[BASE_NAME_SIZE + 1] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1]++;
	}
	else if (fileName[BASE_NAME_SIZE] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1] = '0';
	  fileName[BASE_NAME_SIZE]++;
	}
	else 
	{
		mb.Coil(122,1);                              // ���� ������  �������� �����
	}
  }
 
  temp_file_name = ((fileName[BASE_NAME_SIZE]-48)*10) + (fileName[BASE_NAME_SIZE + 1]-48); // �������������� ����������� ������ ����� � �����
 
  if (temp_file_name == 0)
  {
	 mb.Hreg(adr_reg_file_name,temp_file_name);   
  }
  else
  {
	  mb.Hreg(adr_reg_file_name,temp_file_name-1); 
  }

  delay(200);
  mb.Hreg(adr_control_command,0);  
}
void FileOpen()
{
  int temp_file_name = 0;
  preob_num_str();
  while (sd.exists(fileName)) 
  {
	if (fileName[BASE_NAME_SIZE + 1] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1]++;
	}
	else if (fileName[BASE_NAME_SIZE] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1] = '0';
	  fileName[BASE_NAME_SIZE]++;
	}
	else 
	{
		mb.Coil(122,1);                              // ���� ������  �������� �����
	}
  }

 
  temp_file_name = ((fileName[BASE_NAME_SIZE]-48)*10) + (fileName[BASE_NAME_SIZE + 1]-48); // �������������� ����������� ������ ����� � �����
  mb.Hreg(adr_reg_file_name,temp_file_name);      
//  i2c_eeprom_write_byte(0x50, adr_file_name_count,temp_file_name);                 // ��� ����� ���� ������� ������ ����� �������� � "0"

  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  {
	mb.Coil(122,1);                              // ���� ������  �������� �����
  }
  else
  {
	Serial.print(fileName);
	Serial.println(F("  Open Ok!"));

	DateTime now = RTC.now();

	mb.Hreg(adr_Mic_Start_day , now.day());           // ����� ������ �����
	mb.Hreg(adr_Mic_Start_month, now.month());
	mb.Hreg(adr_Mic_Start_year, now.year());
	mb.Hreg(adr_Mic_Start_hour, now.hour());
	mb.Hreg(adr_Mic_Start_minute, now.minute());
	mb.Hreg(adr_Mic_Start_second, now.second());
	// �������� 			
	mb.Hreg(adr_Time_Test_day, 0); 
	mb.Hreg(adr_Time_Test_hour, 0); 
	mb.Hreg(adr_Time_Test_minute, 0); 
	mb.Hreg(adr_Time_Test_second, 0); 
	myFile.println ("");
	myFile.print ("Report of test module Audio-1 N ");
	byte y[4];                                //������ �� ������ ������� ������ �������� 
		y[3]= mb.Hreg(10);
		y[2]= mb.Hreg(11);
		y[1]= mb.Hreg(12);
		y[0]= mb.Hreg(13);
		number_audio = (unsigned long&) y;       // ������� ��������������� ������� ������ � count_colwater_old
	myFile.print (number_audio);
	myFile.println ("");
	myFile.println ("");
	myFile.println ("");
	myFile.print ("Start test   ");
	file_print_date();
	myFile.println ("");
	mb.Coil(122,0);                              // ���� ��������� �������� �����                                   
	delay(100);
   }
  mb.Hreg(adr_control_command,0);  
}
void FileClose()
{
	
	myFile.println ("");
	myFile.print ("Stop test   ");
	file_print_date();
	myFile.println ("");
	myFile.close();

	if (sd.exists(fileName))
		{ 
			Serial.println();
			Serial.print(fileName);
			Serial.println("  Close  OK!.");
			mb.Coil(123,0);                                  // ���� �������� �����
		}
	else 
		{
			Serial.println();
			Serial.print(fileName);
			Serial.println(" doesn't exist.");  
			mb.Coil(123,1);                              // ���� ������  �������� �����
		}
	mb.Hreg(adr_control_command,0);                                         // ��������� ���������    
	delay(100);
}

void file_name()
{

   preob_num_str();

  while (sd.exists(fileName)) 
  {
	if (fileName[BASE_NAME_SIZE + 1] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1]++;
	}
	else if (fileName[BASE_NAME_SIZE] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1] = '0';
	  fileName[BASE_NAME_SIZE]++;
	}
	else 
	{
		Serial.println("Can't create file name");
//	  sdError("Can't create file name");
	}
  }
  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  {
  }
  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  myFile.close();
  Serial.println("done.");
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
	sprintf(fileName, "%s%s", str2, "00.TXT");                                 // ��������� ����� ����� � file_name
	//Serial.println(fileName);
	mb.Hreg(adr_reg_temp_day, day);  
	mb.Hreg(adr_reg_temp_mon, month); 
	mb.Hreg(adr_reg_temp_year, year-2000); 
	//char* strcpy(char* fileName_p, const char* fileName);
	//Serial.println(fileName_p);
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
			 sensor_all_off();                                                      // ��������� ��� �������
			 break;
		case 2:		
			 sensor_all_on();                                                       // �������� ��� �������
			 break;
		case 3:
			 test_headset_instructor();
			 break;
		case 4:	
			 test_headset_dispatcher();                                             //
			 break;
		case 5:
			 test_MTT();                                                            //
			 break;
		case 6:	
			 test_tangR();                                                          //
			 break;
		case 7:
			test_tangN();
			break;
		case 8:				
			 testGGS();
			 break;
		case 9:
			 test_GG_Radio1();
			 break;
		case 10:	
			 test_GG_Radio2();
			 break;
		case 11:				
			 test_mikrophon();                                                      // ������������ ���������
			 break;
		case 12:
			   FileOpen();
			  break;
		case 13:
		
			  FileClose();
			  break;
		case 14:
			  set_clock();
				break;
		case 15:
			  set_rezistor();
				break;
		case 16:
				Reg_count_clear();			                                        // ����� ��������� ������                    
				break;
		case 17:
				test_power();                                                    	// ��������� ����������  �������
				break;
		case 18:
				set_video();				              //
				
				break;
		case 19:
				test_video();				              //
				break;
		case 20:                                           // �������� ������ ������� ���������
				default_mem_porog();
				break;
		case 21:                                           // 	21 - �������� ������ ������� ����������������
				set_mem_porog();
				break;
		case 22:                                           // 22 - �������� ������ ������� ����������������
				read_mem_porog();
				break;
		case 23:   
				controlFileName();                         // �������� ����� �����
				break;
		case 24:   
			//	 set_SD();                                 // �������� SD ������
				break;
		case 25:   
			 //   Serial.println(test_n);	
			    readFileName();
				send_file_PC();                                 // 
				break;
		case 26:   
				load_list_files();  
				break;
		case 27:   
				file_del_SD();
				break;
		 case 28:   
				test_RS232();
				break;
		
		default:

			wdt_reset();
		break;
	 }
	 mb.Hreg(adr_control_command,0);
	}
}

void sensor_all_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[2])));                    //  " ****** Test sensor OFF start! ******" ;      
	myFile.println(buffer);                                                         //  " ****** Test sensor OFF start! ******" ;      
	file_print_date();
	myFile.println();
	mb.Coil(8,1);                                                               // �������� ������� ��������
	UpdateRegs(); 
	delay(500);
	mb.Coil(5,0);                                                               // �������� ����������� ���������
	mb.Coil(10,0);                                                              // �������� ���������� ���������
	mb.Coil(13,0);                                                              // XP8 - 2   sensor �������� ������
	mb.Coil(14,0);                                                              // XP8 - 1   PTT     �������� ������
	mb.Coil(15,0);                                                              // XS1 - 5   PTT ��� CTS
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ����������� ���������
 
	mb.Coil(17,0);                                                              // J8-12    XP7 4 PTT2 �������� ������ DSR
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	mb.Coil(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	mb.Coil(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���.
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.Coil(28,0);                                                              // XP1- 15 HeS2PTT   CTS ���
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	mb.Coil(30,0);                                                              // XP1- 6  HeS1PTT   CTS ���
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	UpdateRegs(); 
	delay(500);
	UpdateRegs(); 
/*	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];   */ 

	byte i50 = mb.Hreg(4);    
	byte i52 = mb.Hreg(6);     
	byte i53 = mb.Hreg(7);     




		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = mb.Hreg(0);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Hreg(200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			mb.Coil(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ ��������  - Pass
			   }
		  }
	
		if(bitRead(i50,3) != 0)                                                     // J8-11  �������� ������                           "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
		  {
			regcount = mb.Hreg(201);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			mb.Hreg(201,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			mb.Coil(201,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));         // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));     // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja ��������  - Pass
				}
		  }

		if(bitRead(i50,4) != 0)                                                     // XP8 - 2   sensor �������� ������                  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
		  {
			regcount = mb.Hreg(202);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			mb.Hreg(202,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			mb.Coil(202,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));         // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));     // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";  ��������  - Pass
			  }
		  }

		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = mb.Hreg(203);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Hreg(203,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Coil(203,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 ��������  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = mb.Hreg(204);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			mb.Hreg(204,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			mb.Coil(204,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor ��������  - Pass
			  }
		  }

		if(bitRead(i52,3) != 0)                                                     // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
		  {
			regcount = mb.Hreg(205);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Hreg(205,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Coil(205,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 ��������  - Pass
			  }
		  }

		
		if(bitRead(i52,4) != 0)                                                     // XP1- 1  HeS1Ls   sensor ���������� ��������� ���������� 
		  {
			regcount = mb.Hreg(206);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			mb.Hreg(206,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			mb.Coil(206,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
			  }
		  }

		if(bitRead(i52,5) != 0)                                                     // XS1 - 6   sensor ���������� ���������
		  {
			regcount = mb.Hreg(207);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			mb.Hreg(207,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			mb.Coil(207,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone ��������  - Pass
			  }
		  }

		if(bitRead(i53,4) != 0)                                                     // ���� RL4 XP1 12  HeS2e   ���������� ��������� �����������
		  {
			regcount = mb.Hreg(208);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			mb.Hreg(208,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			mb.Coil(208,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));         // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� ��������  - Pass
			  }
		  }

		if(bitRead(i53,6) != 0)                                                     // ���� RL9 XP1 10 ���������� ��������� ����������
		  {
			regcount = mb.Hreg(209);                                          // ����� �������� ������ ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ���������� ��������� ����������
			mb.Hreg(209,regcount);                                            // ����� �������� ������ ���������� ��������� ����������
			mb.Coil(209,1);                                                     // ���������� ���� ������ ���������� ��������� ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));         // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));     // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. ��������  - Pass
			   }
		  }
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);

	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	//delay(100);
}
void sensor_all_on()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3])));                    // " ****** Test sensor ON start! ******";    
	myFile.println(buffer);                                                         // " ****** Test sensor ON start! ******";    
	file_print_date();
	myFile.println();
	mb.Coil(8,1);                                                               // �������� ������� ��������
	UpdateRegs(); 
	delay(500);
	mb.Coil(5,1);                                                               // �������� ����������� ��������
	mb.Coil(10,1);                                                              // �������� ���������� ��������
	mb.Coil(13,1);                                                              // XP8 - 2   sensor �������� ������
	mb.Coil(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
	mb.Coil(25,0);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	//mb.Coil(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	//mb.Coil(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	//mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	//mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);

	byte i50 = mb.Hreg(4);    
	byte i52 = mb.Hreg(6);     
	byte i53 = mb.Hreg(7);    

	 //Serial.print(regs_in[0],HEX);
	 //Serial.print("--");
	 //Serial.print(regs_in[1],HEX);
	 //Serial.print("--");
	 //Serial.print(regs_in[2],HEX);
	 //Serial.print("--");
	 //Serial.print(regs_in[3],HEX);
	 //Serial.println("    ");

	 //Serial.println(i50,BIN);
	 //Serial.println(i52,BIN);
	 //Serial.println(i53,BIN);

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = mb.Hreg(210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Hreg(210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			mb.Coil(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ �������  - Pass
			   }
		  }
	
		if(bitRead(i50,3) == 0)                                                     // J8-11  �������� ������                           "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
		  {
			regcount = mb.Hreg(211);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Hreg(211,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Coil(211,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja �������  - Pass
				}
		  }

		if(bitRead(i50,4) == 0)                                                     // XP8 - 2   sensor �������� ������                  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
		  {
			regcount = mb.Hreg(212);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Hreg(212,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Coil(212,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  �������  - Pass
			  }
		  }
//UpdateRegs(); 
//delay(500);
// test_instr_on();
// UpdateRegs(); 
// delay(500);
// test_disp_on();

/*
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	Serial.println("Instr2");
	mb.Coil(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);
	int kl=0;

	do{
		kl++;

	  }while (mb.Hreg(40007) == regs_temp1);
	  //if( mb.Hreg(40007) != regs_temp)
	  //{
		 //Serial.println(mb.Hreg(40004),BIN);
		 //Serial.println(mb.Hreg(40006),BIN);
		 //Serial.println(mb.Hreg(40007),BIN);
	  //}
	  regs_temp1 = mb.Hreg(40007);
	  kl=0;


	i52 = mb.Hreg(40006);     

		if(bitRead(i52,1) == 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = mb.Hreg(40213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Hreg(40213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Coil(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 �������  - Pass
			  }
		  }

//--------------------------------------------------------------------------------
		Serial.println("Instr");
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.Coil(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);
	i52 = mb.Hreg(40006);     


		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = mb.Hreg(40214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			mb.Hreg(40214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			mb.Coil(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor �������  - Pass
			  }
		  }
//-------------------------------------------------------------------------------
		Serial.println("Disp2");
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);

	i52 = mb.Hreg(40006);    

		if(bitRead(i52,3) == 0)                                                     // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
		  {
			regcount = mb.Hreg(40215);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Hreg(40215,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Coil(215,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 �������  - Pass
			  }
		  }
		//--------------------------------------------------------------------------------------------------
		Serial.println("Disp");
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);

	i52 = mb.Hreg(40006);    
		
		if(bitRead(i52,4) == 0)                                                     // XP1- 1  HeS1Ls   sensor ���������� ��������� ���������� 
		  {
			regcount = mb.Hreg(40216);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			mb.Hreg(40216,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			mb.Coil(216,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
			  }
		  }
//------------------------------------------------------------------------------------------------------------------------

*/
		if(bitRead(i52,5) == 0)                                                     // XS1 - 6   sensor ��������� ���������
		  {
			regcount = mb.Hreg(217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			mb.Hreg(217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			mb.Coil(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass
			  }
		  }

		if(bitRead(i53,4) == 0)                                                     // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
		  {
			regcount = mb.Hreg(218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			mb.Hreg(218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			mb.Coil(218,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� �������  - Pass
			  }
		  }

		if(bitRead(i53,6) == 0)                                                     // ���� RL9 XP1 10 ���������� ��������� ����������
		  {
			regcount = mb.Hreg(219);                                          // ����� �������� ������ ��������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������
			mb.Hreg(219,regcount);                                            // ����� �������� ������ ��������� ��������� ����������
			mb.Coil(219,1);                                                     // ���������� ���� ������ ��������� ��������� ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. �������  - Pass
			   }
		  }



	mb.Coil(5,0);                                                               // �������� ����������� ���������
	mb.Coil(10,0);                                                              // �������� ���������� ���������




	/*
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3])));                    // " ****** Test sensor ON start! ******";    
	myFile.println(buffer);                                                         // " ****** Test sensor ON start! ******";    
	file_print_date();
	myFile.println();
	mb.Coil(8,1);                                                               // �������� ������� ��������
	UpdateRegs(); 
	delay(1000);
	//++++++++++++++++++++++++++++++++++++++++++ ������ �������� ++++++++++++++++++++++++++++++++++++++
	bool test_sens = true;
	mb.Coil(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	UpdateRegs(); 
	delay(500);
	UpdateRegs(); 
	delay(500);
	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];  

	 Serial.print(regs_in[0],HEX);
	 Serial.print("--");
	 Serial.print(regs_in[2],HEX);
	 Serial.print("--");
	 Serial.print(regs_in[3],HEX);
	 Serial.println("    ");

   if (i50 == 0xA3)                                                       
	   {
		if(i52 == 0x43 )
			{
			if(i53 == 0x02 )
				{
				if (test_repeat == false)
					{
						strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
						myFile.print(buffer);                                               // 
						strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
						if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 �������  - Pass
					}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)

	   {
			regcount = mb.Hreg(40213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Hreg(40213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Coil(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.Coil(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x45 )
			   {
				if(i53 == 0x04 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			mb.Hreg(40214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			mb.Coil(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x49 )
			   {
				if(i53 == 0x08 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40215);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Hreg(40215,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Coil(215,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������                                            // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x51 )
			   {
				if(i53 == 0x01 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40216);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			mb.Hreg(40216,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			mb.Coil(216,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	mb.Coil(25,0);                                                              // XP1- 19 HaSs      sensor ����������� ������        
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA7)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x04 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Hreg(40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			mb.Coil(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������        
	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xAB)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x08 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40211);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Hreg(40211,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Coil(211,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	mb.Coil(13,1);                                                              // XP8 - 2   sensor �������� ������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xB3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x01 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  �������  -
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40212);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Hreg(40212,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Coil(212,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	mb.Coil(13,0);                                                              // XP8 - 2   sensor �������� ������
	mb.Coil(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x61 )
			   {
				if(i53 == 0x02 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			mb.Hreg(40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			mb.Coil(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }



	test_sens = true;                                                               // ���� ���������� �����
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ����������� ��������� ���������
	mb.Coil(5,1);                                                               // �������� ����������� �������� "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x11 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			mb.Hreg(40218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(218,true);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,1);                                                              // �������� ���������� ��������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x44 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. �������  - Pass
						}
				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = mb.Hreg(40219);                                          // ����� �������� ������ ��������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������
			regBank.set(40219,regcount);                                            // ����� �������� ������ ��������� ��������� ����������
			mb.Coil(219,1);                                                     // ���������� ���� ������ ��������� ��������� ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Coil(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	mb.Coil(5,0);                                                               // �������� ����������� ���������
	mb.Coil(10,0);                                                              // �������� ���������� ���������
	mb.Coil(13,0);                                                              // XP8 - 2   sensor �������� ������
	mb.Coil(14,0);                                                              // XP8 - 1   PTT     �������� ������
	mb.Coil(15,0);                                                              // XS1 - 5   PTT ��� CTS
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ����������� ���������
 
	mb.Coil(17,0);                                                              // J8-12    XP7 4 PTT2 �������� ������ DSR
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	mb.Coil(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	mb.Coil(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���.
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.Coil(28,0);                                                              // XP1- 15 HeS2PTT   CTS ���
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	mb.Coil(30,0);                                                              // XP1- 6  HeS1PTT   CTS ���
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	*/
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(500);
	UpdateRegs(); 
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	//delay(100);
}

void set_rezistor()
{
	int mwt = mb.Hreg(60);             // ����� �������� �������� �������
	resistor(1, mwt);
	resistor(2, mwt);
	mb.Hreg(adr_control_command,0);
}

void test_headset_instructor()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22])));
	myFile.println(buffer);                                                         // " ****** Test headset instructor start! ******"               ; 
	file_print_date();
	myFile.println("");
	unsigned int regcount = 0;
	test_instr_off();                                                               // ��������� ���� � �������, �������� ����������
	test_instr_on();                                                                // �������� ����������� �������, ��������� ���������
	//myFile.println("");
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 0));                                                                // ���������� ������� ������� 30 ��
	resistor(2, i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 1));                                                                // ���������� ������� ������� 30 ��
	mb.Coil(2,1);                                                               // ������ ������ �� ���� ��������� �����������  Mic2p
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4])));                    // "Signal headset instructor microphone 30mv     ON"            ;   
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset instructor microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 2));                                 // �������� ������� ������� �� ������ FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 3));                                 // �������� ������� ������� �� ������ FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40232,232,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 4));                                 // �������� ������� ������� �� ������ LineL     "Test headset instructor ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40233,233,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 5));                                 // �������� ������� ������� �� ������ LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40234,234,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 6));                                 // �������� ������� ������� �� ������ mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	measure_vol_min(analog_mag_phone,40235,235,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 7));                                 // �������� ������� ������� �� ������ mag phone "Test headset instructor ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 8));                                 // �������� ������� ������� �� ������ GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 9));                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 10));                                // �������� ������� ������� �� ������ GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
												   //
	mb.Coil(5,1);                                                               // ������ ����������� ������� �� ����� 12 ��1 HeS2e (�������� ��������)
	mb.Coil(28,1);                                                              // XP1- 15 HeS2PTT �������� PTT �����������
	mb.Coil(16,0);                                                              // ������ ��������� ���������
	mb.Coil(15,0);                                                              // ��� ��������� ���������
	mb.Coil(29,1);                                                              // ��� XP1- 13 HeS2Ls ������  ��� ���� ����������� ��������� ����������� 
	UpdateRegs();                                                                   // 
	delay(500); 
	wdt_reset();    //
	byte i53 = mb.Hreg(7);                                                  // �������� ������� ��������� ���������
		if(bitRead(i53,4) == 0)                                                     // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
		  {
			regcount = mb.Hreg(218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			mb.Hreg(218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			mb.Coil(218,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // �������� ����������� �������  - Pass
			  }
		  }
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5])));                    // "Microphone headset instructor signal          ON"            ;  
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone headset instructor signal          ON"            ;    �������� ������ ����� �� ���� ��������� �����������
	delay(500);
	wdt_reset();
	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40224,224,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 10));                                // �������� ������� ������� �� ������ LineL      "Test headset instructor ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40226,226,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 12));                                // �������� ������� ������� �� ������ mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 13));                                 // �������� ������� ������� �� ������ FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 14));                                 // �������� ������� ������� �� ������ FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40233,233,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 15));                                 // �������� ������� ������� �� ������ LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 16));                                 // �������� ������� ������� �� ������ GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 17));                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 18));                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";
	wdt_reset();
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls  ��������� ������ �����������
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	mb.Coil(16,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	mb.Coil(15,0);                                                              // ��� ��������� ���������
	mb.Coil(5,0);                                                               // ������ ����������� ������� �� ����� 12 ��1 HeS2e (��������� �������� �����������)
	mb.Coil(28,0);                                                              // XP1- 15 HeS2Ls ��������� PTT �����������
	mb.Coil(2,0);                                                               // ��������� ������ �� ���� ��������� �����������  Mic2p
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	//delay(100);
}
void test_headset_dispatcher()
 {
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23])));                   // " ****** Test headset dispatcher start! ******"               ;
	myFile.println(buffer);                                                         // " ****** Test headset dispatcher start! ******"               ;
	file_print_date();
	myFile.println("");
	unsigned int regcount = 0;
	test_disp_off();                                                                // ��������� ���� � �������, �������� ����������
	test_disp_on();                                                                 // �������� ����������� �������, ��������� ���������
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 0));                                                                // ���������� ������� ������� 30 ��
	resistor(2, i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 1));                                                                // ���������� ������� ������� 30 ��
	mb.Coil(1,1);                                                               // ������ ������ �� ���� ��������� ���������� Mic1p
	UpdateRegs();                                                                   // ��������� �������
	delay(1000);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[13])));                   // "Signal headset dispatcher microphone 30mv     ON"            ;    
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset dispatcher microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 2));                                 // �������� ������� ������� �� ������ FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 3));                                 // �������� ������� ������� �� ������ FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40242,242,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 4));                                 // �������� ������� ������� �� ������ LineL     "Test headset dispatcher ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40243,243,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 5));                                 // �������� ������� ������� �� ������ LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40244,244,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 6));                                 // �������� ������� ������� �� ������ mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	measure_vol_min(analog_mag_phone,40245,245,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 7));                                 // �������� ������� ������� �� ������ mag phone "Test headset dispatcher ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40246,246,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 8));                                 // �������� ������� ������� �� ������ GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 9));                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 10));                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
//	myFile.println("");                                                             //
	mb.Coil(10,1);                                                              // ������ ����������� ������� �� ����� XP1 10 ��������� ��������� ����������
	mb.Coil(30,1);                                                              // XP1- 6  HeS1PTT   �������� PTT ����������
	mb.Coil(16,0);                                                              // ������ ��������� ���������
	mb.Coil(15,0);                                                              // ��� ��������� ���������
	mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	UpdateRegs();                                                                   // 
	delay(500);                                                                     //

	byte i53 = mb.Hreg(7);     
		if(bitRead(i53,6) == 0)                                                      // ��������  ��������� ��������� ����������
		  {
			regcount = mb.Hreg(182);                                          // ����� �������� ������ ��������� ��������� ����������          "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������       "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			mb.Hreg(182,regcount);                                            // ����� �������� ������ ��������� ��������� ����������
			mb.Coil(182,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			resistor(1, 255);                                                       // ���������� ������� ������� � �������� ��������e
			resistor(2, 255);                                                       // ���������� ������� ������� � �������� ��������e
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - ";  - Pass
			   }
		  }
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[14])));                   // "Microphone headset dispatcher signal          ON" 
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone dispatcher signal ON"  �������� ������ ����� �� ���� ��������� ����������
	delay(500);
	wdt_reset();
	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40227,227,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 11));                                // �������� ������� ������� �� ������ LineL     "Test headset dispatcher ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40229,229,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 12));                                // �������� ������� ������� �� ������ mag phone "Test headset dispatcher ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 13));                                 // �������� ������� ������� �� ������ FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 14));                                 // �������� ������� ������� �� ������ FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40243,243,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 15));                                 // �������� ������� ������� �� ������ LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_ggs,      40246,246,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 16));                                 // �������� ������� ������� �� ������ GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 17));                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 18));                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	wdt_reset();
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs   ��������� sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls   ���������  sensor ���������� ��������� ����������
	mb.Coil(15,0);                                                              // ��� ��������� ���������
	mb.Coil(10,0);                                                              // ������ ����������� ������� �� ����� XP1 10  (��������� �������� ����������)
	mb.Coil(30,0);                                                              // XP1- 6  HeS1PTT   ��������� PTT ����������
	mb.Coil(28,0);                                                              // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	mb.Coil(1,0);                                                               // ��������� ������ �� ���� ��������� ���������� Mic1p
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	//delay(100);
 }
void test_MTT()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[24])));                   // " ****** Test MTT start! ******"                              ; 
	myFile.println(buffer);                                                         // " ****** Test MTT start! ******"                              ; 
	file_print_date();
	myFile.println("");
	test_MTT_off();                                                                 // ��������� ���� � �������, �������� ����������
	test_MTT_on();                                                                  // �������� ����������� �������, ��������� ���������
//	myFile.println("");
	mb.Coil(25,0);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT �������� ������ ���� � "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command sensor ON MTT  send!         
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD ������ ������� DCD ������ ���� � "0"
	UpdateRegs();                                                                   // ��������� �������
	delay(1000);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command  HangUp MTT OFF send!"
	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������ ��������� �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 2));                                // �������� ������� ������� �� ������ FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 3));                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40252,252,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 4));                                // �������� ������� ������� �� ������ LineL     "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40253,253,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 5));                                // �������� ������� ������� �� ������ LineR     "Test MTT ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 6));                                // �������� ������� ������� �� ������ mag radio "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40255,255,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 7));                                // �������� ������� ������� �� ������ mag phone "Test MTT ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 8));                                // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 9));                                // �������� ������� ������� �� ������ GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 10));                                // �������� ������� ������� �� ������ GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";

	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 0));                                                               // ���������� ������� ������� 60 ��
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 1));                                                               // ���������� ������� ������� 60 ��
	mb.Coil(3,1);                                                               // �������� ������ �� ���� ��������� ������ Mic3p
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));                   // "Signal MTT microphone 30mv                    ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal MTT microphone 30mv                    ON"            ;
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������  +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 11));                                // �������� ������� ������� �� ������ FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 12));                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 13));                                // �������� ������� ������� �� ������ mag radio   "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 14));                                // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 15));                                // �������� ������� ������� �� ������ GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 16));                                // �������� ������� ������� �� ������ GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";
	// ++++++++++++++++++++++++++++++++++ ��������� ������� �������  ++++++++++++++++++++++++++++++++++++
	//measure_vol_max(analog_LineL,    40260,260,35);                                 // "Test MTT ** Signal LineL                                    ON  - ";  
	measure_vol_max(analog_LineR,    40261,261,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 17));                                 // "Test MTT ** Signal LineR                                    ON  - ";  
	measure_vol_max(analog_mag_phone,40262,262,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 18));                                 // �������� ������� ������� �� ������ mag phone  "Test MTT ** Signal Mag phone                                ON  - ";
	// +++++++++++++++++++++ �������� ������������ ������ ��� �� ������ HangUp  DCD ON +++++++++++++++++++++++++++++++++
	mb.Coil(3,0);                                                               // ��������� ������ �� ���� ��������� ������ Mic3p
	mb.Coil(6,1);                                                               // ���� RL5. ������ ���� Front L, Front R
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[35])));                   //   
	if (test_repeat == false) myFile.println(buffer);                               // "Signal FrontL, FrontR  ON                             - "
	measure_vol_min(analog_ggs,       40256,256,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 19));                    // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	mb.Coil(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;
	measure_vol_max(analog_ggs,      40259,259,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 20));                                //  �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS             On      
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD ON  �������� ������
	mb.Coil(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���. ��������� PTT MTT
	mb.Coil(25,1);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT ��������� ������ ���� � "1"
	mb.Coil(6,0);                                                               // ���� RL5. ��������� ���� Front L, Front R
	UpdateRegs();                                                                   // ��������� �������
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(200);
}
void test_tangR()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[36])));                   // " ****** Test tangenta ruchnaja start! ******"                ;
	myFile.println(buffer);                              //
	file_print_date();
	myFile.println("");
	mb.Coil(17,0);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	mb.Coil(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[37])));                   // "Command sensor OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[38])));                   // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[39])));                   // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 

	byte i50 = mb.Hreg(4);    

	if(bitRead(i50,3) != 0)                                                         // J8-11     XP7 2 sensor �������� ������               "Command sensor tangenta ruchnaja                            OFF - ";
		{
			regcount = mb.Hreg(274);                                          // ����� �������� ������ sensor �������� ������     "Command sensor tangenta ruchnaja                            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Command sensor tangenta ruchnaja                            OFF - ";
			mb.Hreg(274,regcount);                                            // ����� �������� ������ sensor �������� ������     "Command sensor tangenta ruchnaja                            OFF - ";
			mb.Coil(274,1);                                                     // ���������� ���� ������ sensor �������� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		}
	else
		{
		  if (test_repeat == false)
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta ruchnaja                            OFF - ";  - Pass
		  }
		}

	 UpdateRegs(); 
	  // 2)  ��������  �� ���������� J8-23     XP7 1 PTT1 �������� ������ CTS
		if(mb.Hreg(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� XP7 1 PTT1 �������� ������ CTS "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
		  {
			regcount = mb.Hreg(270);                                          // ����� �������� ������                                  "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(270,regcount);                                            // ����� �������� ������ 
			mb.Coil(270,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";  - Pass
		  }
		 }

	 // 3)  ��������  �� ���������� PTT2 �������� ������ (DSR)

		if(mb.Hreg(adr_reg_ind_DSR) != 0)                                       // ��������  �� ����������  PTT2 �������� ������ (DSR) "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
		  {
			regcount = mb.Hreg(271);                                          // ����� �������� ������  PTT  MTT (DSR)                "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(271,regcount);                                            // ����� �������� ������  PTT  MTT (DSR)                 "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			mb.Coil(271,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";  - Pass
		   }
		  }

	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[40])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	mb.Coil(17,1);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[41])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	mb.Coil(20,1);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[42])));                   // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //

	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	i50 = mb.Hreg(4);    

		if(bitRead(i50,3) == 0)                                          // J8-11     XP7 2 sensor �������� ������             "Command sensor tangenta ruchnaja                            ON  - ";
		  {
			regcount = mb.Hreg(275);                                          // ����� �������� ������ sensor �������� ������       "Command sensor tangenta ruchnaja                            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������    "Command sensor tangenta ruchnaja                            ON  - ";
			mb.Hreg(275,regcount);                                            // ����� �������� ������ sensor �������� ������
			mb.Coil(275,1);                                                     // ���������� ���� ������ sensor �������� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                         // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta ruchnaja                            ON  - "; - Pass
		   }
		  }
	 UpdateRegs(); 
	  // 2)  ��������  �� ���������� J8-23     XP7 1 PTT1 �������� ������ CTS
		if(mb.Hreg(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������� XP7 1 PTT1 �������� ������ CTS    "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
		  {
			regcount = mb.Hreg(272);                                          // ����� �������� ������ PTT  MTT (CTS)                      "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(272,regcount);                                            // ����� �������� ������ PTT  MTT (CTS)
			mb.Coil(272,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - "; - Pass
		   }
		  }

	 // 3)  ��������  �� ���������� PTT2 �������� ������ (DSR)

		if(mb.Hreg(adr_reg_ind_DSR) == 0)                                       // ��������  �� ����������  PTT2 �������� ������ (DSR)    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
		  {
			regcount = mb.Hreg(273);                                          // ����� �������� ������  PTT  MTT (DSR)                   "Command PTT2 tangenta ruchnaja (DCR)                        ON  - "; 
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(273,regcount);                                            // ����� �������� ������  PTT  MTT (DSR)                    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			mb.Coil(273,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";  - Pass
		   }
		  }
	mb.Coil(17,0);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	mb.Coil(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	UpdateRegs();                                                                   // ��������� �������
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
}
void test_tangN()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[43])));                   // " ****** Test tangenta nognaja start! ******"                 ;
	myFile.println(buffer);                                                         // "Command sensor OFF tangenta nognaja              send!"      ;
	file_print_date();
	myFile.println("");
	mb.Coil(13,0);                                                              // XP8 - 2   sensor �������� ������
	mb.Coil(14,0);                                                              // XP8 - 1   PTT �������� ������
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[44])));                   // "Command sensor OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[45])));                   // "Command PTT    OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //

//	byte i50 = regs_in[0];    
	byte i50 = mb.Hreg(4);    
//	byte i52 = mb.Hreg(40006);     
//	byte i53 = mb.Hreg(40007);     
	if(bitRead(i50,4) != 0)                                                         // J8-11     XP8 2 sensor ��������                  "Command sensor tangenta nognaja                             OFF - ";
		{
			regcount = mb.Hreg(276);                                          // ����� �������� ������ sensor �������� ������     "Command sensor tangenta nognaja                             OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Command sensor tangenta nognaja                             OFF - ";
			mb.Hreg(276,regcount);                                            // ����� �������� ������ sensor �������� ������     "Command sensor tangenta nognaja                             OFF - ";
			mb.Coil(276,1);                                                     // ���������� ���� ������ sensor �������� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		}
	else
		{
		  if (test_repeat == false)
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta nognaja                             OFF - "; - Pass
		  }
		}

	 UpdateRegs(); 
	  // 2)  ��������  �� ����������  XP8 1 PTT1 �������� ������ CTS
		if(mb.Hreg(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� XP8 1 PTT1 ��������   "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  {
			regcount = mb.Hreg(278);                                          // ����� �������� ������ 
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(278,regcount);                                            // ����� �������� ������                          "Command PTT tangenta nognaja (CTS)                          OFF - ";
			mb.Coil(278,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  }
		 }


	mb.Coil(13,1);                                                              // XP8 2 sensor �������� ������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[46])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	mb.Coil(14,1);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[47])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;


	UpdateRegs();                                                                   // ��������� �������
	delay(400);

	i50 = mb.Hreg(4);    
	//byte i52 = mb.Hreg(40006);     
	//byte i53 = mb.Hreg(40007);     


			if(bitRead(i50,4) == 0)                                                 // J8-11     XP7 2 sensor ��������                    "Command sensor tangenta nognaja                             ON  - ";
		  {
			regcount = mb.Hreg(277);                                          // ����� �������� ������ sensor �������� ������       "Command sensor tangenta nognaja                             ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������    "Command sensor tangenta nognaja                             ON  - ";
			mb.Hreg(277,regcount);                                            // ����� �������� ������ sensor �������� ������
			mb.Coil(277,1);                                                     // ���������� ���� ������ sensor �������� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                         // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta nognaja                             ON  - ";
		   }
		  }
	 UpdateRegs(); 
	  // 2)  ��������  �� ����������  XP8 1 PTT1 ��������  CTS
		if(mb.Hreg(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������� XP8 1         "Command PTT tangenta nognaja (CTS)                          ON  - ";
		  {
			regcount = mb.Hreg(279);                                          // ����� �������� ������                 "Command PTT tangenta nognaja (CTS)                          ON  - ";          
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(279,regcount);                                            // ����� �������� ������                  "Command PTT tangenta nognaja (CTS)                          ON  - ";
			mb.Coil(279,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT tangenta nognaja (CTS)                          ON  - ";
		   }
		  }

	mb.Coil(14,0);                                                              //   XP8 1 PTT ��������  
	mb.Coil(13,0);                                                              //   XP8 2 sensor ��������  
	UpdateRegs();                                                                   // ��������� �������
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
}
void test_mikrophon()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[54])));                   // " ****** Test mi�rophone start! ******"                       ;
	myFile.println(buffer);                                                         // " ****** Test mi�rophone start! ******"                       ;
	file_print_date();
	myFile.println("");
	mb.Coil(15,0);                                                              // XS1 - 5   PTT ��� CTS
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ����������� ���������

	mb.Coil(14,0);    // XP8 - 1   PTT �������� ������
	mb.Coil(17,0);    // J8-12     XP7 4 PTT2   ����. �.
	mb.Coil(18,0);    // XP1 - 20  HangUp  DCD
	mb.Coil(20,0);    // J8-23     XP7 1 PTT1 ����. �.
	mb.Coil(26,0);    // XP1- 17 HaSPTT    CTS DSR ���.  
	mb.Coil(28,0);    // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	mb.Coil(30,0);    // XP1- 6  HeS1PTT   CTS ���   ��� ����������

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));                   //"Command PTT    OFF microphone                    send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	wdt_reset();

	 // +++++++++++++++++++++++++++++++++++++++ ��������  �� ���������� ������� �  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
 
	byte i52 = mb.Hreg(6);     
//	byte i53 = mb.Hreg(40007);     

	//byte i52 = regs_in[2];    
			if(bitRead(i52,5) != 0)                                                 // XS1 - 6   sensor ���������� ���������
		  {
			regcount = mb.Hreg(207);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			mb.Hreg(207,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			mb.Coil(207,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone ��������  - Pass
			  }
		  }

	 UpdateRegs(); 
	 delay(500);
	 wdt_reset();
	  // 2)  ��������  �� ���������� PTT microphone
		if(mb.Hreg(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� "Test microphone PTT  (CTS)                                  OFF - ";
		  {
			regcount = mb.Hreg(264);                                          // ����� �������� ������       "Test microphone PTT  (CTS)                                  OFF - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(264,regcount);                                            // ����� �������� ������ 
			mb.Coil(264,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Test microphone PTT  (CTS)                                  OFF - ";  - Pass
		  }
		 }

	 // +++++++++++++++++++++++++++++++++++++++ ��������  �� ��������� �������  microphone ++++++++++++++++++++++++++++++++++++++++++++
	mb.Coil(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));                   // "Command sensor ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	wdt_reset();

	i52 = mb.Hreg(6);     
	//byte i53 = mb.Hreg(40007);     

	//i52 = regs_in[2];    

	  if(bitRead(i52,5) == 0)                                                 // XS1 - 6   sensor ���������� ���������
		  {
			regcount = mb.Hreg(217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			mb.Hreg(217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			mb.Coil(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
			}
		  }
	  // +++++++++++++++++++++++++++++++++++++++ ��������  �� ���������  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
	mb.Coil(15,1);                                                              // XS1 - 5   PTT ��� CTS
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ����������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[57])));                   // "Command PTT    ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	wdt_reset();

	i52 = mb.Hreg(6);     
	//byte i53 = mb.Hreg(40007);     
	//i52 = regs_in[2];    

	if(bitRead(i52,5) == 0)                                             // XS1 - 6   sensor ���������� ���������
		  {
			regcount = mb.Hreg(217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			mb.Hreg(217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			mb.Coil(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
			}
		  }

	 UpdateRegs(); 
	 delay(500);
	 wdt_reset();
	  // 2)  ��������  �� ���������  PTT microphone
		if(mb.Hreg(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������      "Test microphone PTT  (CTS)                                  ON  
		  {
			regcount = mb.Hreg(266);                                          // ����� �������� ������       "Test microphone PTT  (CTS)                                  ON  - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(266,regcount);                                            // ����� �������� ������ 
			mb.Coil(266,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Test microphone PTT  (CTS)                                  ON  - ";
		  }
		 }

	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������  �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40320,320,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 2));                                // �������� ������� ������� �� ������ FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40321,321,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 3));                                // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40322,322,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 4));                                // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40323,323,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 5));                                // �������� ������� ������� �� ������ LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40324,324,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 6));                                // �������� ������� ������� �� ������ mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40325,325,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 7));                                // �������� ������� ������� �� ������ mag phone "Test Microphone ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40326,326,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 8));                                // �������� ������� ������� �� ������ GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40327,327,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 9));                                // �������� ������� ������� �� ������ GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40328,328,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 10));                                // �������� ������� ������� �� ������ GG Radio2 "Test Microphone ** Signal GG Radio2     


		// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 0));                                                                // ���������� ������� ������� 60 ��
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 1));                                                                // ���������� ������� ������� 60 ��
	mb.Coil(9,1);                                                               // �������� ������ �� ���� ��������� ���� RL8 ���� �� ��������
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	UpdateRegs();                                                                   // ��������� �������
//	delay(1000);
	wdt_reset();
	measure_vol_max(analog_mag_phone,40298,298,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 11));                                // �������� ������� ������� �� ������ mag phone  "Test Microphone ** Signal Mag phone      
	measure_vol_max(analog_LineL,    40299,299,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 12));                                // �������� ������� ������� �� ������ "Test Microphone ** Signal LineL                      ON  - ";  
	
	measure_vol_min(analog_FrontL,    40320,320,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 13));                                // �������� ������� ������� �� ������ FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40321,321,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 14));                                // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineR,     40323,323,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 15));                                // �������� ������� ������� �� ������ LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40324,324,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 16));                                // �������� ������� ������� �� ������ mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       40326,326,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 17));                                // �������� ������� ������� �� ������ GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40327,327,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 18));                                // �������� ������� ������� �� ������ GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40328,328,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 19));                                // �������� ������� ������� �� ������ GG Radio2 "Test Microphone ** Signal GG Radio2     

	mb.Coil(9,0);                                                               // ��������� ������ �� ���� ��������� ���� RL8 ���� �� ��������
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ����������� ���������
	mb.Coil(15,0);                                                              // XS1 - 5   PTT ��� CTS
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(200);
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    

}

void testGGS()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	for (int k = 0; k < 20; k++)
	{
		por_buffer[k] = i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + k); //
		delay(20);
	}

	delay(100);
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[48])));                   // " ****** Test GGS start! ******"      ;
	myFile.println(buffer);                                                         // " ****** Test GGS start! ******"      ;
	file_print_date();
	myFile.println("");
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������  
	mb.Coil(6,0);                                                               // ���� RL5 ���� Front L, Front R
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 0));                                                               // ���������� ������� ������� 60 ��
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 1));                                                               // ���������� ������� ������� 60 ��
	//UpdateRegs();                                                                   // ��������� �������
	delay(500);
	UpdateRegs(); 
	delay(500);
	UpdateRegs(); 

	byte i50 = mb.Hreg(4);                                                 


		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = mb.Hreg(200);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Hreg(200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			mb.Coil(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ ��������  - Pass
			   }
		  }
		//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40280,280,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 2));                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40281,281,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 3));                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40282,282,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 4));                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 5));                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40284,284,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 6));                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40285,285,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 7));                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40286,286,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 8));                                // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40287,287,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 9));                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 10));                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	//----------------------------------------------------------------------------------------------------------------------------------------

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[49])));                   // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;
	mb.Coil(6,1);                                                               // ���� RL5 ���� Front L, Front R
	UpdateRegs();                                                                   // ��������� �������
	delay(500);

	measure_vol_max(analog_FrontL,    40290,290,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 11));                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    40291,291,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 12));                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	measure_vol_min(analog_LineL,     40282,282,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 13));                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 14));                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_max(analog_mag_phone, 40292,292,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 15)); //??                               // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_max(analog_ggs,       40289,289,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 16)); //??                               // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      ON  - ";
	measure_vol_min(analog_gg_radio1, 40287,287,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 17));                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 18));                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

	mb.Coil(25,0);                                                                          // XP1- 19 HaSs      sensor ����������� ������          
	UpdateRegs();                                                                               // ��������� �������
	delay(500);
	UpdateRegs(); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                               // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Command sensor ON  MTT                           send!"      ;

	measure_vol_max(analog_FrontL,    290,290,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 19));                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    291,291,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 20));                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	measure_vol_min(analog_LineL,     282,282,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 21));                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     283,283,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 22));                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
	//measure_vol_min(analog_mag_radio, 40284,284,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 23));                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_max(analog_mag_phone, 292,292,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 24));                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_min(analog_ggs,       286,286,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 25));                                // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 287,287,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 26));                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 288,288,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 27));                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

	mb.Coil(6,0);                                                               // ���� RL5 ���� Front L, Front R
	UpdateRegs();    
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
}
void test_GG_Radio1()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[50])));                   // " ****** Test Radio1 start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test Radio1 start! ******"                           ;
	file_print_date();
	myFile.println("");
	mb.Coil(4,0);                                                               // ���� RL3 ����  LFE  "���."
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 0));                                                               // ���������� ������� ������� 300 ��
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 1));                                                               // ���������� ������� ������� 300 ��
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	UpdateRegs(); 
	delay(500);
	wdt_reset();
	//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40300,300,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 2));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40301,301,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 3));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40302,302,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 4));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40303,303,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 5));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40304,304,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 6));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40305,305,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 7));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40306,306,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 8));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40307,307,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 9));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, 40308,308,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 10));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[51])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	mb.Coil(4,1);                                                               //  ���� RL3 ����  LFE  "���."
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	UpdateRegs();  
	wdt_reset();
//	Serial.println("test_GG_Radio1 - on ");
	measure_vol_min(analog_FrontL,    300,300,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 11));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    301,301,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 12));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     302,302,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 13));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     303,303,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 14));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
//	measure_vol_min(analog_mag_radio, 40304,304,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 15));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 305,305,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 16));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       306,306,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 17));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_max(analog_gg_radio1, 309,309,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 18));                               // �������� ������� ������� �� ������ "Test Radio1 ** Signal Radio1                                ON  - ";
	measure_vol_min(analog_gg_radio2, 308,308,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 19));                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.Coil(4,0);                                                               // ���� RL3 ����  LFE  "���."
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
}
void test_GG_Radio2()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[52])));                   // " ****** Test Radio2 start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test Radio2 start! ******"                           ;
	file_print_date();
	myFile.println("");
	mb.Coil(7,0);                                                               // ���� RL3 ����  LFE  "���."
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 0));                                                               // ���������� ������� ������� 300 ��
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 1));                                                               // ���������� ������� ������� 300 ��
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	UpdateRegs(); 
	delay(500);
	wdt_reset();
	//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40310,310,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 2));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40311,311,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 3));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40312,312,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 4));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40313,313,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 5));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40314,314,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 6));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40315,315,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 7));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40316,316,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 8));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40317,317,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 9));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, 40318,318,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 10));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	mb.Coil(7,1);                                                               //  ���� RL3 ����  LFE  "���."
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	UpdateRegs();  
	wdt_reset();
	//Serial.println("test_GG_Radio2 - on ");

	measure_vol_min(analog_FrontL,    40310,310,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 11));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40311,311,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 12));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40312,312,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 13));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40313,313,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 14));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	//measure_vol_min(analog_mag_radio, 40314,314,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 15));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40315,315,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 16));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40316,316,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 17));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40317,317,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 18));                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal Radio1                                ON  - ";
	measure_vol_max(analog_gg_radio2, 40319,319,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 19));                               // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";

	mb.Coil(7,0);                                                               // ���� RL6 ���� Center
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);    
}
void test_power()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[67])));                   // " ****** Test power start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test power start! ******"                           ;
	file_print_date();
	myFile.println("");

	//regBank.add(40293);                         // A���� ��������  ������ ADC1 ���������� 12/3 �����
	//regBank.add(40294);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio1
	//regBank.add(40295);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio2
	//regBank.add(40296);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� ���
	//regBank.add(40297);                         // A���� ��������  ������ ADC15 ���������� ���������� 3,6 ������

	//regBank.add(40493);                         // A���� ������ ��������� ADC1 ���������� 12/3 �����
	//regBank.add(40494);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio1
	//regBank.add(40495);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio2
	//regBank.add(40496);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� ���
	//regBank.add(40497);                         // A���� ������ ��������� ADC15 ���������� ���������� 3,6 ������

	measure_power();

	// �������� ���������� 12 ����� ����� ��������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[61])));                   // "Power Kamerton V  - "                                        ;
	if(mb.Hreg(493)*2.51/100 < 11 || mb.Hreg(493)*2.51/100 >13)
	{
		myFile.print(buffer);                               // 
		myFile.print(mb.Hreg(493)*2.51/100);
		myFile.println("V - error");
		mb.Coil(293,1); 
		regcount = mb.Hreg(293);
		regcount++;
		mb.Hreg(293,regcount); 
		mb.Coil(120,1);  
	}

	else
	{
		if (test_repeat == false)
			{
				myFile.print(buffer);                               // 
				myFile.print(mb.Hreg(493)*2.51/100);
				myFile.println("V - pass");
			}
	}

	// �������� ���������� 12 ����� ����� 1
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[63])));                   // "Power Radio1 V    - "                                        ;
	if(mb.Hreg(494)*2.51/100 < 11 || mb.Hreg(494)*2.51/100 >13)
	{
		myFile.print(buffer);                               // "Power Radio1 V    - "                                        ;
		myFile.print(mb.Hreg(494)*2.51/100);
		myFile.println("V - error");
		mb.Coil(294,1); 
		regcount = mb.Hreg(294);
		regcount++;
		mb.Hreg(294,regcount); 
		mb.Coil(120,1);  
	}

	else
	{
		if (test_repeat == false) 
			{
				myFile.print(buffer);                               // "Power Radio1 V    - "                                        ;
				myFile.print(mb.Hreg(494)*2.51/100);
				myFile.println("V - pass");
			}
	}

	// �������� ���������� 12 ����� ����� 2
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[64])));                   // "Power Radio2 V    - "                                        ;
	if(mb.Hreg(495)*2.51/100 < 11 || mb.Hreg(495)*2.51/100 >13)
	{
		myFile.print(buffer);                               // "Power Radio2 V    - "                                        ;
		myFile.print(mb.Hreg(495)*2.51/100);
		myFile.println("V - error");
		mb.Coil(295,1); 
		regcount = mb.Hreg(295);
		regcount++;
		mb.Hreg(295,regcount); 
		mb.Coil(120,1);  
	}

	else
	{
		if (test_repeat == false) 
			{
				myFile.print(buffer);                               // "Power Radio2 V    - "                                        ;
				myFile.print(mb.Hreg(495)*2.51/100);
				myFile.println("V - pass");
			}
	}

	// �������� ���������� 12 �����  ���
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[65])));                   // "Power GGS    V    - "                                        ;
	if(mb.Hreg(496)*2.51/100 < 11 || mb.Hreg(496)*2.51/100 >13)
	{
		myFile.print(buffer);                               // "Power GGS    V    - "                                        ;
		myFile.print(mb.Hreg(496)*2.51/100);
		myFile.println("V - error");
		mb.Coil(296,1); 
		regcount = mb.Hreg(296);
		regcount++;
		mb.Hreg(296,regcount); 
		mb.Coil(120,1);  
	}

	else
	{	
		if (test_repeat == false) 
		{
			myFile.print(buffer);                               // "Power GGS    V    - "                                        ;
			myFile.print(mb.Hreg(496)*2.51/100);
			myFile.println("V - pass");
		}
	}

	// �������� ���������� 3,6 ����� �� ���������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[66])));                   // "Power Led mic.V   - "  
	if(mb.Hreg(497)/100 < 2 || mb.Hreg(497)/100 >4)
	{
		myFile.print(buffer);                                 // "Power Led mic.V   - " 
		myFile.print(mb.Hreg(497)/100.0);
		myFile.println("V - error");
		mb.Coil(297,1); 
		regcount = mb.Hreg(297);
		regcount++;
		mb.Hreg(297,regcount); 
		mb.Coil(120,1);  
	}

	else
	{
		if (test_repeat == false) 
			{
				myFile.print(buffer);                                 // "Power Led mic.V   - " 
				myFile.print(mb.Hreg(497)/100.0);
				myFile.println("V - pass");
			}
	}
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);    
}
void test_video()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[68])));                   // " ****** Test Adjusting the brightness of the display! ******"; 
	myFile.println(buffer);                                                         // " ****** Test Adjusting the brightness of the display! ******"; 
	file_print_date();
	myFile.println("");
	mb.Hreg(61,100);                                                         // ������� ������� 100
	delay(300);
	regs_out[0]= 0x2B;                                                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0x84;                                                              // 
	regs_out[2]= mb.Hreg(61);                                                // ������� �������
	delay(300);
	regs_out[0]= 0x2B;                                                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                                                              // 
	regs_out[2]= 0x7F;                                                              // ������� �������
	measure_mks();                                                                  // �������� ������������ ���������
	
	mb.Hreg(62,mb.Hreg(5));                                          // �������� ������� ������� � ���������

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[69])));                   // !!!!! 
	if (mb.Hreg(62) != 50)                                                   //   
	{
		myFile.print(buffer);                                                       // 
		myFile.print(mb.Hreg(62));
		myFile.println(" - error");
		mb.Coil(329,1); 
		regcount = mb.Hreg(329);
		regcount++;
		mb.Hreg(329,regcount); 
		mb.Coil(120,1);  
		mb.Hreg(529,regcount); 
	}

	else
	{
	if (test_repeat == false) 
		{
			myFile.print(buffer);                                                   // 
			myFile.print(mb.Hreg(62));
			myFile.println(" - pass");
		}
	}

	delay(100);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[70])));                   // 
	if (mb.Hreg(63) < 18 || mb.Hreg(63) > 26)                         // �������� ��������� ������������ �������� �������
	{
		myFile.print(buffer);                                                       // 
		myFile.print(mb.Hreg(63));
		myFile.println(" - error");
		mb.Coil(269,1); 
		regcount = mb.Hreg(269);
		regcount++;
		mb.Hreg(269,regcount); 
		mb.Coil(120,1);  
		mb.Hreg(469,mb.Hreg(63));                   // �������� ������������ �������� � ��
	}

	else
	{
		if (test_repeat == false) 
			{
				myFile.print(buffer);                                               // 
				myFile.print(mb.Hreg(63));
				myFile.println(" - pass");
			}
	}

	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 196 �������� � �������� �����
	regs_out[2]= 0x7F;                              // 127 �������� � �������� �����
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);    
}
void set_video()
{
	delay(300);
	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0x84;                              // 
	regs_out[2]= mb.Hreg(61);                // ������� �������
	delay(300);
	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 
	regs_out[2]= 0x7F;                              // ������� �������
	measure_mks();                                  // �������� ������������ ���������
	mb.Hreg(62,mb.Hreg(5));          // �������� ������� ������� � ���������
	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 196 �������� � �������� �����
	regs_out[2]= 0x7F;                              // 127 �������� � �������� �����
	mb.Hreg(adr_control_command,0);  
}
void measure_mks()
{
  unsigned long duration = 0;
  unsigned long duration1 = 0;
  duration = pulseIn(A12, HIGH);
	  for (int imp = 0;imp < 10; imp++)
	  {
	   duration = pulseIn(A12, HIGH);
	   duration1 += duration;
	  }
	  duration = duration1/10;
  //Serial.println(duration);
 mb.Hreg(469,duration); 	 
 mb.Hreg(63,duration);                          // �������� ������������ �������� ������� � ���������
}

void test_instr_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[7])));                    // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"                   ;  
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls  ��������� ������ �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ���
	mb.Coil(1,0);                                                               // ���� RL0 ����
	mb.Coil(2,0);                                                               // ���� RL1 ����
	mb.Coil(3,0);                                                               // ���� RL2 ����
	mb.Coil(4,0);                                                               // ���� RL3 ����  LFE  "���."
	mb.Coil(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	mb.Coil(6,0);                                                               // ���� RL5 ����
	mb.Coil(7,0);                                                               // ���� RL6 ����
	mb.Coil(9,0);                                                               // ���� RL8 ���� �� ��������
	mb.Coil(10,0);                                                              // ���� RL9 XP1 10
	mb.Coil(28,0);                                                              // XP1- 15 HeS2Ls ��������� PTT �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[8])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!"
	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(1000);
	UpdateRegs(); 
	delay(100);
   
	byte i52 = mb.Hreg(6);     
//	byte i53 = mb.Hreg(40007);     


//	byte i52 = regs_in[2];    
	 
	  // 1)  �������� ������� �� ���������� ��������� ����������� 2 ����������
		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = mb.Hreg(203);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Hreg(203,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Coil(203,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 ��������  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = mb.Hreg(204);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			mb.Hreg(204,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			mb.Coil(204,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor ��������  - Pass
			  }
		  }

	 // 3)  �������� ������� �� ���������� ���������

		if(bitRead(i52,5) != 0)                                                     // ��������  ����� �� ���������� ���������
		  {
			regcount = mb.Hreg(207);                                          // ����� �������� ������ ������� ��������� 
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(207,regcount);                                            // ����� �������� ������ ������� ���������
			mb.Coil(207,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				if (test_repeat == false) myFile.print(buffer);                     // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone                   XS1 - 6                 OFF - ";   ��������  - Pass
			   }
		  }

		UpdateRegs(); 
		delay(100);
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_CTS) != 0)                                        // ��������� ��������� PTT �����������   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = mb.Hreg(220);                                          // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(220,regcount);                                            // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			mb.Coil(220,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        OFF - "  ��������  - Pass
		   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	   delay(100);
}
void test_instr_on()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[10])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor    send!"
	mb.Coil(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[11])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor 2  send!"
	mb.Coil(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor  ����. �.
	mb.Coil(16,1);                                                              // XS1 - 6   sensor ���
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������      
	mb.Coil(13,1);                                                              // XP8 - 2           sensor �������� ������
	mb.Coil(28,1);                                                              // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[12])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
	UpdateRegs();                                                                   // ��������� ������� ��������� ��������
	delay(1000);
	UpdateRegs(); 
	delay(100);
	wdt_reset();
 
	byte i52 = mb.Hreg(6);     

	  // 3)  �������� ������� �� ����������� ��������� ����������� 2 ����������
			if(bitRead(i52,1) == 0)                                                 // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = mb.Hreg(213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Hreg(213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			mb.Coil(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 �������  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = mb.Hreg(214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			mb.Hreg(214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			mb.Coil(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor �������  - Pass
			  }
		  }

		UpdateRegs(); 
		delay(100);
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_CTS)== 0)                                         // ��������� ��������� PTT �����������   CTS "Command PTT headset instructor (CTS)                        ON  - ";
		  {
			regcount = mb.Hreg(221);                                          // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(221,regcount);                                            // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        ON  - ";
			mb.Coil(221,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                       ON  - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        ON  - "  �������  - Pass
		   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	   delay(100);
}

void test_disp_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[16])));                   // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"     
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    ��������� ������ ��������� ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[15])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ���
	mb.Coil(1,0);                                                               // ���� RL0 ����
	mb.Coil(2,0);                                                               // ���� RL1 ����
	mb.Coil(3,0);                                                               // ���� RL2 ����
	mb.Coil(4,0);                                                               // ���� RL3 ����  LFE  "���."
	mb.Coil(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	mb.Coil(6,0);                                                               // ���� RL5 ����
	mb.Coil(7,0);                                                               // ���� RL6 ����
	mb.Coil(9,0);                                                               // ���� RL8 ���� �� ��������
	mb.Coil(10,0);                                                              // ���� RL9 XP1 10
	mb.Coil(30,0);                                                              // XP1- 6  HeS1PTT   ��������� PTT ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[17])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!""

	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(1000);
	UpdateRegs(); 
   
	byte i52 = mb.Hreg(6);     
	//byte i53 = mb.Hreg(40007);     
	//byte i52 = regs_in[2];    
	 
	  // 1)  �������� ������� �� ���������� ��������� ���������� 2 ����������
		if(bitRead(i52,3) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ���������� � 2 ����������
		  {
			regcount = mb.Hreg(205);                                          // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ���������� � 2 ����������
			mb.Hreg(205,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			mb.Coil(205,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� � 2 ���������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - " ��������  - Pass
			  }
		  }

		if(bitRead(i52,4) != 0)                                                     //"Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "  ����������� ��������� ����������
		  {
			regcount = mb.Hreg(206);                                          // ����� �������� ������ sensor ����������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������
			mb.Hreg(206,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������
			mb.Coil(206,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - " ��������  - Pass
			  }
		  }

	 // 3)  �������� ������� �� ���������� ���������

		if(bitRead(i52,5) != 0)                                                     // ��������  ����� �� ���������� ���������
		  {
			regcount = mb.Hreg(207);                                          // ����� �������� ������ ������� ��������� 
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(207,regcount);                                            // ����� �������� ������ ������� ���������
			mb.Coil(207,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor microphone                   XS1 - 6                 OFF - ";   ��������  - Pass
			   }
		  }

		UpdateRegs(); 
		delay(100);
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_CTS) != 0)                                        // ��������� ���������� PTT ����������   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = mb.Hreg(222);                                          // ����� ��������   ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(222,regcount);                                            // ����� ��������   ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			mb.Coil(222,1);                                                     // ���������� ����  ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        OFF - "  ��������  - Pass
		   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	   delay(100);
}
void test_disp_on()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[19])));                   // "Command sensor ON  headset dispatcher 2          send!"                   ;   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher 2          send!"                   ;    
	mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor ����������� ��������� ���������� 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20])));                   // "Command sensor ON  headset dispatcher            send!"      ;    
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher            send!"      ;    
	mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor  ����. �.
	mb.Coil(16,1);                                                              // XS1 - 6   sensor ���
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������      
	mb.Coil(13,1);                                                              // XP8 - 2           sensor �������� ������
	mb.Coil(30,1);                                                              // XP1- 6  HeS1PTT   �������� PTT ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
	UpdateRegs();                                                                   // ��������� ������� ��������� ��������
	delay(1000);
	UpdateRegs(); 
  
	byte i52 = mb.Hreg(6);     
	//byte i53 = mb.Hreg(40007);     
	//byte i52 = regs_in[2];    

	  // 3)  �������� ������� �� ����������� ��������� ���������� 2 ����������
		if(bitRead(i52,3) == 0)                                                 // XP1- 16 HeS2Rs    sensor ����������� ��������� ���������� � 2 ����������
		  {
			regcount = mb.Hreg(215);                                          // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ���������� � 2 ����������
			mb.Hreg(215,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			mb.Coil(215,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� � 2 ���������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  �������  - Pass
			  }
		  }

		if(bitRead(i52,4) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ���������� 
		  {
			regcount = mb.Hreg(216);                                          // ����� �������� ������    sensor ����������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������
			mb.Hreg(216,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� 
			mb.Coil(216,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� 
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - " �������  - Pass
			  }
		  }

		UpdateRegs(); 
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_CTS)== 0)                                         // ��������� ��������� PTT ����������   "Command PTT headset dispatcher (CTS)                        ON  - ";
		  {
			regcount = mb.Hreg(223);                                          // ����� �������� ������  ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(223,regcount);                                            // ����� �������� ������  ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        ON  - ";
			mb.Coil(223,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                       ON  - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        ON  - "  �������  - Pass
		   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	   delay(100);
}

void test_MTT_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	mb.Coil(25,1);                                                              // "Command sensor OFF MTT                           send! "     ;     
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[27])));                   // "Command sensor OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF MTT                           send! "     ;      
	mb.Coil(26,0);                                                              // XP1- 17 HaSPTT    CTS  ���. ��������� PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[28])));                   // "Command PTT    OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    OFF MTT                           send! "     ;
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                   // "Command        OFF HangUp MTT                    send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command        OFF HangUp MTT                    send! "     ;
	mb.Coil(16,0);                                                              // XS1 - 6   sensor ���
	mb.Coil(1,0);                                                               // ���� RL0 ����
	mb.Coil(2,0);                                                               // ���� RL1 ����
	mb.Coil(3,0);                                                               // ���� RL2 ����
	mb.Coil(4,0);                                                               // ���� RL3 ����  LFE  "���."
	mb.Coil(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	mb.Coil(6,0);                                                               // ���� RL5 ����
	mb.Coil(7,0);                                                               // ���� RL6 ����
	mb.Coil(9,0);                                                               // ���� RL8 ���� �� ��������
	mb.Coil(10,0);                                                              // ���� RL9 XP1 10
	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(1000);
	UpdateRegs(); 
	delay(100);
	byte i50 = mb.Hreg(4);    
	//byte i52 = mb.Hreg(40006);     
	//byte i53 = mb.Hreg(40007);     
	//byte i50 = regs_in[0];    
	wdt_reset();
		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = mb.Hreg(200);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Hreg(200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			mb.Coil(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  ������ ��������  - Pass
			   }
		  }
		   UpdateRegs(); 
		   delay(1000);
		   wdt_reset();
	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(mb.Hreg(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� CTS MTT
		  {
			regcount = mb.Hreg(263);                                          // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(263,regcount);                                            // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			mb.Coil(263,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));        // "Test MTT PTT    (CTS)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));    // "Test MTT PTT    (CTS)                                       OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Test MTT PTT    (CTS)                                       OFF - ";
			   }                   
		  }
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_DCD)!= 0)                                         // ��������� ��������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = mb.Hreg(267);                                          // ����� �������� ������ ���������� HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(267,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			mb.Coil(267,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));        // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));    // "Test MTT HangUp (DCD)                                       OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Test MTT HangUp (DCD)                                       OFF - ";
			   }             
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	   delay(100);
}
void test_MTT_on()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	delay(600);
	unsigned int regcount = 0;
	mb.Coil(25,0);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  MTT                           send!"      ;              
	mb.Coil(26,1);                                                              // XP1- 17 HaSPTT    CTS DSR ���. �������� PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[31])));                   // "Command PTT    ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    ON  MTT                           send!"      ;
	mb.Coil(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;

	UpdateRegs(); 
	delay(1000);
	UpdateRegs();
	wdt_reset();

	  // 1)  �������� ������� MTT �� ��������� 
	byte i50 = mb.Hreg(4);    
	//byte i52 = mb.Hreg(40006);     
	//byte i53 = mb.Hreg(40007);     
	//byte i50 = regs_in[0];    
		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = mb.Hreg(210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Hreg(210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			mb.Coil(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  ������ �������  - Pass
			   }
		  }

		delay(1000);
		UpdateRegs(); 

	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(mb.Hreg(adr_reg_ind_CTS) == 0)                                       // ��������  �� ��������� CTS MTT
		  {
			regcount = mb.Hreg(265);                                          // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(265,regcount);                                            // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			mb.Coil(265,1);                                                     // ���������� ���� ������
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));        // "Test MTT PTT    (CTS)                                       ON  - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));    // "Test MTT PTT    (CTS)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT PTT    (CTS)                                       ON  - " ������ �������  - Pass
			   }
		  }

	   if(mb.Hreg(adr_reg_ind_DCD)== 0)                                         // ��������� ��������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = mb.Hreg(268);                                          // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			mb.Hreg(268,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			mb.Coil(268,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			mb.Coil(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = mb.Hreg(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));        // "Test MTT HangUp (DCD)                                       ON  - ";  
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));    // "Test MTT HangUp (DCD)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT HangUp (DCD)                                       ON  - ";������ �������  - Pass
			   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	   delay(100);
}

void measure_vol_min(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
		mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
		int _istochnik          = istochnik;
		unsigned int _adr_count = adr_count;
		int _adr_flagErr        = adr_flagErr;
		unsigned int _porogV    = porogV;
		int regcount = 0;
		measure_volume(_istochnik);                                                 // �������� ������� ������� �� ������
		wdt_reset();
		switch (_adr_flagErr) 
		{
			case 230:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[30])));    // "Test headset instructor ** Signal FrontL                    OFF - ";
				break;
			case 231:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[31])));    // "Test headset instructor ** Signal FrontR                    OFF - ";
				break;
			case 232:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[32])));    // "Test headset instructor ** Signal LineL                     OFF - ";
				break;
			case 233:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[33])));    // "Test headset instructor ** Signal LineR                     OFF - ";
				break;
			case 234:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[34])));    // "Test headset instructor ** Signal mag radio                 OFF - ";
				break;
			case 235:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[35])));    // "Test headset instructor ** Signal mag phone                 OFF - ";
				break;
			case 236:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[36])));    // "Test headset instructor ** Signal GGS                       OFF - ";
				break;
			case 237:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[37])));    // "Test headset instructor ** Signal GG Radio1                 OFF - ";
				break;
			case 238:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[38])));    // "Test headset instructor ** Signal GG Radio2                 OFF - ";
				break;
			case 240:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[40])));    // "Test headset dispatcher ** Signal FrontL                    OFF - ";
				break;
			case 241:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[41])));    // "Test headset dispatcher ** Signal FrontR                    OFF - ";
				break;
			case 242:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[42])));    // "Test headset dispatcher ** Signal LineL                     OFF - ";
				break;
			case 243:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[43])));    // "Test headset dispatcher ** Signal LineR                     OFF - ";
				break;
			case 244:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[44])));    // "Test headset dispatcher ** Signal mag radio                 OFF - ";
				break;
			case 245:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[45])));    // "Test headset dispatcher ** Signal mag phone                 OFF - ";
				break;
			case 246:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[46])));    // "Test headset dispatcher ** Signal GGS                       OFF - ";
				break;
			case 247:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[47])));    // "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
				break;
			case 248:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[48])));    // "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
				break;
			case 250:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[50])));    //  "Test MTT ** Signal FrontL                                   OFF - ";
				break;
			case 251:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[51])));    // "Test MTT ** Signal FrontR                                   OFF - ";
				break;
			case 252:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[52])));    // "Test MTT ** Signal LineL                                    OFF - ";
				break;
			case 253:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[53])));    // "Test MTT ** Signal LineR                                    OFF - ";
				break;
			case 254:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[54])));    // "Test MTT ** Signal mag radio                                OFF - ";
				break;
			case 255:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[55])));    // "Test MTT ** Signal mag phone                                OFF - ";
				break;
			case 256:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[56])));    // "Test MTT ** Signal GGS                                      OFF - ";
				break;
			case 257:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[57])));    // "Test MTT ** Signal GG Radio1                                OFF - ";
				break;
			case 258:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[58])));    // "Test MTT ** Signal GG Radio2                                OFF - ";
				break;
			case 280:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[80])));    //  ������ "Test GGS ** Signal FrontL                                   OFF - ";
				break;
			case 281:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[81])));    // ������ "Test GGS ** Signal FrontR                                   OFF - ";
				break;
			case 282:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[82])));    // ������ "Test GGS ** Signal LineL                                    OFF - ";
				break;
			case 283:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[83])));    // ������ "Test GGS ** Signal LineR                                    OFF - ";
				break;
			case 284:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[84])));    // ������ "Test GGS ** Signal mag radio                                OFF - ";
				break;
			case 285:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[85])));    // ������ "Test GGS ** Signal mag phone                                OFF - ";
				break;
			case 286:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[86])));    // ������ "Test GGS ** Signal GGS                                      OFF - ";
				break;
			case 287:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[87])));    // ������ "Test GGS ** Signal GG Radio1                                OFF - ";
				break;
			case 288:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[88])));    // "Test GGS ** Signal GG Radio2                                OFF - ";
				break;
			case 300:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[100])));    //  "Test Radio1 ** Signal FrontL                                OFF - ";
				break;
			case 301:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[101])));    // "Test Radio1 ** Signal FrontR                                OFF - ";
				break;
			case 302:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[102])));    // "Test Radio1 ** Signal LineL                                 OFF - ";
				break;
			case 303:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[103])));    // "Test Radio1 ** Signal LineR                                 OFF - ";
				break;
			case 304:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[104])));    // "Test Radio1 ** Signal mag radio                             OFF - ";
				break;
			case 305:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[105])));    // "Test Radio1 ** Signal mag phone                             OFF - ";
				break;
			case 306:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[106])));    // "Test Radio1 ** Signal GGS                                   OFF - ";
				break;
			case 307:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[107])));    // "Test Radio1 ** Signal GG Radio1                             OFF - ";
				break;
			case 308:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[108])));    // "Test Radio1 ** Signal GG Radio2                             OFF - ";
				break;
			case 310:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[110])));    //  "Test Radio2 ** Signal FrontL                                OFF - ";
				break;
			case 311:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[111])));    // "Test Radio2 ** Signal FrontR                                OFF - ";
				break;
			case 312:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[112])));    // "Test Radio2 ** Signal LineL                                 OFF - ";
				break;
			case 313:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[113])));    // "Test Radio2 ** Signal LineR                                 OFF - ";
				break;
			case 314:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[114])));    // "Test Radio2 ** Signal mag radio                             OFF - ";
				break;
			case 315:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[115])));    // "Test Radio2 ** Signal mag phone                             OFF - ";
				break;
			case 316:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[116])));    // "Test Radio2 ** Signal GGS                                   OFF - ";
				break;
			case 317:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[117])));    // "Test Radio2 ** Signal GG Radio1                             OFF - ";
				break;
			case 318:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[118])));    // "Test Radio2 ** Signal GG Radio2                             OFF - ";
				break;
			case 320:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[120])));    //  "Test Microphone ** Signal FrontL                                   OFF - ";
				break;
			case 321:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[121])));    // "Test Microphone ** Signal FrontR                                   OFF - ";
				break;
			case 322:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[122])));    // "Test Microphone ** Signal LineL                                    OFF - ";
				break;
			case 323:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[123])));    // "Test Microphone ** Signal LineR                                    OFF - ";
				break;
			case 324:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[124])));    // "Test Microphone ** Signal mag radio                                OFF - ";
				break;
			case 325:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[125])));    // "Test Microphone ** Signal mag phone                                OFF - ";
				break;
			case 326:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[126])));    // "Test Microphone ** Signal GGS                                      OFF - ";
				break;
			case 327:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[127])));    // "Test Microphone ** Signal GG Radio1                                OFF - ";
				break;
			case 328:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[128])));    // "Test Microphone ** Signal GG Radio2                                OFF - ";
				break;

		}
		wdt_reset();
		
	/*	Serial.print("voltage - ");
		Serial.print(voltage);
		Serial.print(" voltage10 - ");
		Serial.print(voltage10);
		Serial.print(" porogV - ");
		Serial.println(_porogV);*/
		mb.Hreg(_adr_count+200,voltage10);                               // ����� ������ ������ ������ 
		if(voltage10 > _porogV)                                                      // ��������� ����������� ������
			{
				myFile.print(buffer); 
				regcount = mb.Hreg(_adr_count);                                  // ����� �������� ������ 
				regcount++;                                                          // ��������� ������� ������ ������ 
				mb.Hreg(_adr_count,regcount);                                    // ����� �������� ������ ������ 
				mb.Hreg(_adr_count+200,voltage10);                               // ����� ������ ������ ������ 
				mb.Coil(_adr_flagErr,1);                                         // ���������� ���� ������  ������ 
				regcount_err = mb.Hreg(adr_reg_count_err);                       // �������� ������ �������� ���� ������
				regcount_err++;                                                      // ��������� ������� ���� ������ 
				mb.Hreg(adr_reg_count_err,regcount_err);                         // ��������� ������ �������� ���� ������
				mb.Coil(120,1);                                                  // ���������� ����� ���� ������ 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));         // "    Error! - "; 
				myFile.print(buffer);                                                // "    Error! - "; 
				myFile.print(regcount);                                              // ��������� �������� ������
				myFile.print("  ");  
				myFile.print(voltage); 
				myFile.println(" V");
			}
		else
			{
				if (test_repeat == false)
				{
					myFile.print(buffer);                                           // ������������ ��������
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.print(buffer);                                           // "Pass";
					myFile.print("  ");  
					myFile.print(voltage); 
					myFile.println(" V");
				}
			}  
		wdt_reset();
		mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
}
void measure_vol_max(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	int _istochnik          = istochnik;
	unsigned int _adr_count = adr_count;
	int _adr_flagErr        = adr_flagErr;
	unsigned int _porogV    = porogV;
	int regcount            = 0;

	measure_volume(_istochnik);                                                 // �������� ������� ������� �� ������
	wdt_reset();
	switch (_adr_flagErr) 
		{
			case 224:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[24])));    // "Test headset instructor ** Signal LineL                     ON  - ";
				break;
			case 225:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[25])));    // "Test headset instructor ** Signal LineR                     ON  - "; 
				break;
			case 226:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[26])));    // "Test headset instructor ** Signal Mag phone                 ON  - ";
				break;
			case 227:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[27])));    // "Test headset dispatcher ** Signal LineL                     ON  - ";
				break;
			case 228:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[28])));    // "Test headset dispatcher ** Signal LineR                     ON  - "; 
				break;
			case 229:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[29])));    // "Test headset dispatcher ** Signal Mag phone                 ON  - ";
				break;
			case 259:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[59])));    // "Test MTT ** Signal GGS                                      ON  - ";
				break;
			case 260:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[60])));    // "Test MTT ** Signal LineL                                    ON  - ";
				break;
			case 261:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[61])));    // "Test MTT ** Signal LineR                                    ON  - ";  
				break;
			case 262:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[62])));    // "Test MTT ** Signal Mag phone                                ON  - ";
				break;
			case 289:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[89])));    // ������ "Test GGS ** Signal GGS                                      ON  - ";
				break;
			case 290:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[90])));    // ������ "Test GGS ** Signal FrontL                                   ON  - ";
				break;
			case 291:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[91])));    // ������ "Test GGS ** Signal FrontR                                   ON  - ";
				break;
			case 292:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[92])));    // ������ "Test GGS ** Signal mag phone                                ON  - ";
				break;
			case 298:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[98])));    // "Test Microphone ** Signal mag phone                         ON  - ";      
				break;
			case 299:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[99])));    // "Test Microphone ** Signal LineL                             ON  - ";  
				break;
			case 309:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[109])));   // "Test Radio1 ** Signal Radio1                                ON  - ";
				break;
			case 319:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[119])));   // "Test Radio1 ** Signal Radio2                                ON  - ";
				break;
		}
	wdt_reset();
	mb.Hreg(_adr_count+200,voltage10);                              // ����� ������ ������ ������ 
		if(voltage10 < _porogV)                                                     // ��������� ����������� ������
			{
				myFile.print(buffer); 
				regcount = mb.Hreg(_adr_count);                                 // ����� �������� ������ 
				regcount++;                                                         // ��������� ������� ������ ������ 
				mb.Hreg(_adr_count, regcount);                                  // ����� �������� ������ ������ 
				//mb.Hreg(_adr_count+200,voltage10);                              // ����� ������ ������ ������ 
				mb.Coil(_adr_flagErr,1);                                        // ���������� ���� ������  ������ 
				regcount_err = mb.Hreg(adr_reg_count_err);                       // �������� ������ �������� ���� ������
				regcount_err++;                                                      // ��������� ������� ���� ������ 
				mb.Hreg(adr_reg_count_err,regcount_err);                         // ��������� ������ �������� ���� ������
				mb.Coil(120,1);                                                 // ���������� ����� ���� ������ 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.print(regcount);                                             // ��������� �������� ������
				myFile.print("  ");  
				myFile.print(voltage); 
				myFile.println(" V");
			}
		else
			{
			if (test_repeat == false)
				{
					myFile.print(buffer);                                           // ������������ ��������
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.print(buffer);                                           // "Pass";
					myFile.print("  ");  
					myFile.print(voltage); 
					myFile.println(" V");
				}
			} 
		wdt_reset();
		mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
		delay(100);
}
void measure_volume(int analog)
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	volume1     = 0;
	 unsigned long	 volume_maxx = 0;
	 unsigned long   volume_minx = 0;
	 wdt_reset();
	 int stix=150;
	 for (int sti = 0;sti<= stix; sti++)
	  {
		volume_max  = 0;
		volume_min  = 1023;
		volume_fact = 0;
		int i;
		int i_stop = 150;
		
		for (i = 0;i<= i_stop; i++)
			{
				Array_volume[i] = analogRead(analog);               // ��������� ��������
			}
		for (i = 0; i<= i_stop; i++)
			{
				volume_max = max(volume_max, Array_volume[i]);
				volume_min = min(volume_min, Array_volume[i]);
			}
			   volume_maxx += volume_max;
			   volume_minx += volume_min;
	   }
		wdt_reset();

		volume_fact = (volume_maxx/stix) - (volume_minx/stix);
	//	volume_fact = (volume_max) - (volume_min);
		voltage = volume_fact * (5.0 / 1023.0);
		voltage10 = voltage * 100;
		//Serial.print("volume_max - ");
		//Serial.print((volume_maxx/stix) * (5.0 / 1023.0));
		//Serial.print("- volume_min - ");
		//Serial.print((volume_minx /stix) * (5.0 / 1023.0));
		//Serial.print(" = ");
		//Serial.println(((volume_maxx/stix) -(volume_minx /stix)) * (5.0 / 1023.0));
		//Serial.print("voltage - ");
		//Serial.println(voltage10);
		mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
		delay(100);
		//return voltage10;
}
void measure_volume_P(int analog)
{
		volume_fact = 0;
		volume_fact = analogRead(analog);               // ��������� ��������
		voltage = volume_fact * (5.0 / 1023.0);
		voltage10 = voltage * 100;

		//Serial.print("voltage - ");
		//Serial.println(voltage10);
}
void measure_power()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mb.Coil(21,0);                           // XP2-2     sensor "���."  
	mb.Coil(22,0);                           // XP5-3     sensor "��C."
	mb.Coil(23,0);                           // XP3-3     sensor "��-�����1."
	mb.Coil(24,0);                           // XP4-3     sensor "��-�����2."
	UpdateRegs();         
	delay(100);

	measure_volume_P(analog_tok);     
	mb.Hreg(400,voltage10);                     
	measure_volume_P(analog_12V);   
	mb.Hreg(493,voltage10);   
	measure_volume_P(analog_tok_x10);   
	mb.Hreg(402,voltage10);   

	mb.Coil(23,1);                           // XP3-3     sensor "��-�����1."
	UpdateRegs();         
	delay(200);
	wdt_reset();
	measure_volume_P(analog_14); 
	mb.Hreg(494,voltage10);   

	mb.Coil(23,0);                           // XP3-3     sensor "��-�����1."
	mb.Coil(24,1);                           // XP4-3     sensor "��-�����2."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	mb.Hreg(495,voltage10);   

	mb.Coil(24,0);                           // XP4-3     sensor "��-�����2."
	mb.Coil(22,1);                           // XP5-3     sensor "��C."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	mb.Hreg(496,voltage10);   
	wdt_reset();
	mb.Coil(22,0);                           // XP5-3     sensor "��C."
	UpdateRegs();         
	delay(100);

	measure_volume_P(analog_3_6);     
	mb.Hreg(497,voltage10);   
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
}

void i2c_test()
{ 
	/*
	
	Serial.println("--------  EEPROM Test  ---------");
	char somedata[] = "this data from the eeprom i2c"; // data to write
	i2c_eeprom_write_page(0x50, 0, (byte *)somedata, sizeof(somedata)); // write to EEPROM 
	delay(100); //add a small delay
	Serial.println("Written Done");    
	delay(10);
	Serial.print("Read EERPOM:");
	byte b = i2c_eeprom_read_byte(0x50, 0); // access the first address from the memory
	char addr=0; //first address
	
	while (b!=0) 
	{
	  Serial.print((char)b); //print content to serial port
	  if (b!=somedata[addr]){
	   e1=0;
	   break;
	   }      
	  addr++; //increase address
	  b = i2c_eeprom_read_byte(0x50, addr); //access an address from the memory
	}
	 Serial.println();
	 */
}

void i2c_test1()
{
	
	for( int n = 0; n<500; n++)
	{
		i2c_eeprom_write_byte(0x50, n,0);
		
	}

	for(unsigned int x=0;x<255;x++)
	{
		int  b = i2c_eeprom_read_byte(0x50, x); //access an address from the memory
		delay(10);
		Serial.print(x); //print content to serial port
		Serial.print(" - "); //print content to serial port
		Serial.println(b); //print content to serial port
	}
	
}

void test_RS232()
{

}

void default_mem_porog()  // ������ ��������� ��������� ������� ������
{
 byte por_buffer[30] ;
 // ---------------- porog_instruktor
 
  for (int k = 0; k < 19; k++)
  {
	por_buffer[k] = pgm_read_byte(pgm_get_far_address(porog_instruktor)+k);
	i2c_eeprom_write_byte(deviceaddress, adr_porog_instruktor+k,por_buffer[k]);
  }
 
  // ---------------- porog_dispatcher
 
  for (int k = 0; k < 19; k++)
  {
	por_buffer[k] = pgm_read_byte(pgm_get_far_address(porog_dispatcher)+k);
	i2c_eeprom_write_byte(deviceaddress, adr_porog_dispatcher+k,por_buffer[k]);
  }

  // ---------------- porog_MTT

  for (int k = 0; k < 21; k++)
  {
	por_buffer[k] = pgm_read_byte(pgm_get_far_address(porog_MTT)+k);
	i2c_eeprom_write_byte(deviceaddress, adr_porog_MTT+k,por_buffer[k]);
  }
 
// ---------------- porog_GGS
  for (int k = 0; k < 29; k++)
  {
	por_buffer[k] = pgm_read_byte(pgm_get_far_address(porog_GGS)+k);
	i2c_eeprom_write_byte(deviceaddress, adr_porog_GGS+k,por_buffer[k]);


	//Serial.print(k);
	//Serial.print(" - ");
	//byte b = i2c_eeprom_read_byte(deviceaddress, adr_porog_GGS+k); 
	//Serial.println(b); //print content to serial port

  }
  
  // ---------------- adr_porog_Radio1
  for (int k = 0; k < 20; k++)
  {
	por_buffer[k] = pgm_read_byte(pgm_get_far_address(porog_Radio1)+k);
	i2c_eeprom_write_byte(deviceaddress, adr_porog_Radio1+k,por_buffer[k]);
  }
 
 // ---------------- adr_porog_Radio2
  for (int k = 0; k < 20; k++)
  {
	por_buffer[k] = pgm_read_byte(pgm_get_far_address(porog_Radio2)+k);
	i2c_eeprom_write_byte(deviceaddress, adr_porog_Radio2+k,por_buffer[k]);
  }
 
  // ---------------- adr_porog_Microphone
   for (int k = 0; k < 20; k++)
  {
	por_buffer[k] = pgm_read_byte(pgm_get_far_address(porog_Microphone)+k);
	i2c_eeprom_write_byte(deviceaddress, adr_porog_Microphone+k,por_buffer[k]);
  }
 
	UpdateRegs();     
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
}

void set_mem_porog()
{
	/*
		��������� ������ ������� � EEPROM
		��������� ����� ������ 200
		��������� ����� ��������� 40130 
		����� ����� �� ����� ??? ����
		mb.Hreg(40127);  //  ����� ����� ��������� ��� �������� � �� ������� �������.
		mb.Hreg(40128);  //  ����� ����� ������ ��� �������� � �� ������� �������.
		mb.Hreg(40129);  //  ����� ����� ����� ������ ��� �������� � �� ������� �������.
	*/
	int _adr_reg  = mb.Hreg(127);              // ��������� ����� ����� ���������, 
	int _adr_mem  = mb.Hreg(128);              // ��������� ����� ����� ������
	int _step_mem = mb.Hreg(129);              // ����� ����� � ������ �������� ����������� �����
	int _u_porog  = 0;                               // ��������� �������� ����������� ��������.
	int i_k       = 0;                               // �������� ������ ����� ������

	for (int i = 0; i < _step_mem;i++)                            // ����������� ����� ������ � ��������.        
		{
			_u_porog = mb.Hreg(_adr_reg+i);
		   // ��������� _u_porog �� byte
			hi=highByte(_u_porog);
			low=lowByte(_u_porog);
		//	// ��� �� ��� hi,low ����� ��������� EEPROM
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k, low); 
			i_k++;
		}
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	delay(200);
}
void read_mem_porog()
{
	/*
		��������� ������ ������� ������ �� ������ � ��������
		mb.Hreg(40127);  //  ����� ����� ��������� ��� �������� � �� ������� �������.
		mb.Hreg(40128);  //  ����� ����� ������ ��� �������� � �� ������� �������.
		mb.Hreg(40129);  //  ����� ����� ����� ������ ��� �������� � �� ������� �������.
		��������� ����� ������ 200
		��������� ����� ��������� 40130 
		����� ����� �� ����� ??? ����
	*/
	int _adr_reg  = mb.Hreg(127);              // ��������� ����� ����� ���������, 
	int _adr_mem  = mb.Hreg(128);              // ��������� ����� ����� ������
	int _step_mem = mb.Hreg(129);              // ����� ����� � ������ �������� ����������� �����
	int _u_porog = 0;                                      // ��������� �������� ����������� ��������.
	int i_k = 0;                                           // �������� ������ ����� ������

	for (int i = 0; i < _step_mem;i++)                            // ����������� ����� ������ � ��������.        
		{

		  hi  = i2c_eeprom_read_byte(deviceaddress,_adr_mem+i_k);   // 
		  i_k++;
		  low = i2c_eeprom_read_byte(deviceaddress,_adr_mem+i_k);
		  i_k++;
		   _u_porog = (hi<<8) | low;                              // �������� ��� "��������� �����������"
		  mb.Hreg(_adr_reg+i,_u_porog);
		 }
	
	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	delay(200);
}

void set_mem_regBank(int adr_mem , int step_mem)
{
	int _adr_mem = adr_mem;
	int _step_mem = step_mem;
	for (int i = 0; i < _step_mem;i++)
		{
			i2c_eeprom_write_byte(deviceaddress, _adr_mem + i, mb.Hreg(130)+i);
		}
}

//void read_mem_porog()
//{
//	int n_test_mem ;
//	n_test_mem = mb.Hreg(40129); 
//
//	switch (n_test_mem)
//	{
//
//		case 1:                                                      // headset_instructor
//			    read_mem_regBank(adr_porog_instruktor , 19);
//				break;
//		case 2:
//				 read_mem_regBank(adr_porog_dispatcher , 19);        //headset_dispatcher			                                        // ����� ��������� ������                    
//				break;
//		case 3:
//				read_mem_regBank(adr_porog_MTT , 21);                //MTT                                                    	// ��������� ����������  �������
//				break;
//		case 4:
//				read_mem_regBank(adr_porog_Microphone, 20);          //mikrophon		              //
//				break;
//		case 5:
//				read_mem_regBank(adr_porog_GGS , 28);                //GGS			              //
//				break;
//        case 6:                                           
//				read_mem_regBank(adr_porog_Radio1 , 20);             //Radio1
//				break;
//        case 7:                                                                // 	
//				read_mem_regBank(adr_porog_Radio2 , 20);             //Radio2
//				break;
// 		default:
//			    break;
//		break;
//
//	}
//	wdt_reset();
//	mb.Hreg(40129,0);  
//	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
//	delay(200);
//}
void read_mem_regBank(int adr_mem , int step_mem)
{
	int _adr_mem = adr_mem;
	int _step_mem = step_mem;
	for (int i = 0; i < _step_mem;i++)
	{
	  mb.Hreg(130+i,i2c_eeprom_read_byte(deviceaddress,_adr_mem +i));   
	}
}

void readFileName()
{
	
	if (Serial2.available())                               // ���� ���-�� ���������? ���� ������ � ������?
		  {
			unsigned char overflowFlag = 0 ;               // ���� ���������� ������� ������
			unsigned char buffer = 0;                      // ���������� � ������ ������ ������

			while (Serial2.available())
				{
				  if (overflowFlag)                        // ���� ����� ���������� - ��������
					 Serial2.read();
				  else                                     // ������ ������ � �����, ������� ����������
					{
					if (bufferK == BUFFER_SIZEKF)           // ��������� ������ ������
						{
							overflowFlag = 1;              // ���������� ���� ���������� ������� ������
						}
							fileName_F[buffer] = Serial2.read(); 
						buffer++;
					}
				}

		   }

	 else 
		{
	
		}
	Serial.println(fileName_F);
	
//	wdt_reset();  // ����� ����������� ������� ��� ������� ����� � ��
}
void send_file_PC()
{
	delay(1000);
	read_Serial2();
 /* Serial.println("fileName");
 15110201.TXT*/

  Serial.println(fileName_F);
  myFile = sd.open(fileName_F);
 // myFile = sd.open(fileName);
 // if (myFile) 
 // {
 //   Serial.println(fileName_p);

 //   // read from the file until there's nothing else in it:
 //   while (myFile.available()) 
	//{
 //     Serial.print(myFile.read());
 //   }
 //   // close the file:
 //    myFile.close();
 // }
 // else 
 // {
 //   // if the file didn't open, print an error:
 //   Serial.println("error opening file");
 // }

  // if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  //{
	 //Serial.println("error opening file");                            // ���� ������  �������� �����
  //}
  //else
  //{
	   // read from the file until there's nothing else in it:
	while (myFile.available()) 
	{
	  Serial2.write(myFile.read());
	}
	// close the file:
   //  myFile.close();

   //}


	mb.Hreg(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
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

void setup_regModbus()
{

	// Set the Slave ID (1-247)
	mb.setSlaveId(1);  
   // ����������� � ��������� MODBUS ���������� Serial3    
	mb.addCoil(0);                           // ���� 
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


	mb.addCoil(118,0);                         // ���� ��������� ������������ ��������
	mb.addCoil(119,0);                         // 

	mb.addCoil(120,0);                         // ���� ��������� ������������� ����� ������
	mb.addCoil(122,0);                         // ���� ��������� �������� �����
	mb.addCoil(123,0);                         // ���� ��������� �������� �����
	mb.addCoil(124,0);                         // ���� ��������� ����� � ������� "��������"
	mb.addCoil(125,0);                         // ���� ��������� ������������� SD ������
	mb.addCoil(126,0);                         //  
	mb.addCoil(127,0);                         //  
	mb.addCoil(128,0);                         //  
	mb.addCoil(129,0);                         //  

	mb.addCoil(130,0);                         //  ���� ��������� ����� 0 - RS232, 1 - USB0
	mb.addCoil(131,0);                         //  


	mb.addCoil(200,0);                         // ���� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	mb.addCoil(201,0);                         // ���� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	mb.addCoil(202,0);                         // ���� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	mb.addCoil(203,0);                         // ���� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	mb.addCoil(204,0);                         // ���� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	mb.addCoil(205,0);                         // ���� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	mb.addCoil(206,0);                         // ���� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	mb.addCoil(207,0);                         // ���� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	mb.addCoil(208,0);                         // ���� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	mb.addCoil(209,0);                         // ���� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	mb.addCoil(210,0);                         // ���� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	mb.addCoil(211,0);                         // ���� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	mb.addCoil(212,0);                         // ���� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	mb.addCoil(213,0);                         // ���� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	mb.addCoil(214,0);                         // ���� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	mb.addCoil(215,0);                         // ���� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	mb.addCoil(216,0);                         // ���� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	mb.addCoil(217,0);                         // ���� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	mb.addCoil(218,0);                         // ���� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	mb.addCoil(219,0);                         // ���� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
	 
	mb.addCoil(220,0);                         // ���� ������ "Command PTT headset instructor (CTS)                        OFF - ";
	mb.addCoil(221,0);                         // ���� ������ "Command PTT headset instructor (CTS)                        ON  - ";
	mb.addCoil(222,0);                         // ���� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
	mb.addCoil(223,0);                         // ���� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
	mb.addCoil(224,0);                         // ���� ������ "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addCoil(225,0);                         // ���� ������ "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addCoil(226,0);                         // ���� ������ "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addCoil(227,0);                         // ���� ������ "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addCoil(228,0);                         // ���� ������ "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addCoil(229,0);                         // ���� ������ "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addCoil(230,0);                         // ���� ������ "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addCoil(231,0);                         // ���� ������ "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addCoil(232,0);                         // ���� ������ "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addCoil(233,0);                         // ���� ������ "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addCoil(234,0);                         // ���� ������ "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addCoil(235,0);                         // ���� ������ "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addCoil(236,0);                         // ���� ������ "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addCoil(237,0);                         // ���� ������ "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addCoil(238,0);                         // ���� ������ "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addCoil(239,0);                         // ���� ������  ADC0  ��� x1 

	mb.addCoil(240,0);                         // ���� ������ "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addCoil(241,0);                         // ���� ������ "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addCoil(242,0);                         // ���� ������ "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addCoil(243,0);                         // ���� ������ "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addCoil(244,0);                         // ���� ������ "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addCoil(245,0);                         // ���� ������ "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addCoil(246,0);                         // ���� ������ "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addCoil(247,0);                         // ���� ������ "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addCoil(248,0);                         // ���� ������ "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addCoil(249,0);                         // ���� ������ ADC2 ��� x10  

	mb.addCoil(250,0);                         // ���� ������ "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addCoil(251,0);                         // ���� ������ "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addCoil(252,0);                         // ���� ������ "Test MTT ** Signal LineL                                    OFF - ";
	mb.addCoil(253,0);                         // ���� ������ "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addCoil(254,0);                         // ���� ������ "Test MTT ** Signal mag radio                                OFF - ";
	mb.addCoil(255,0);                         // ���� ������ "Test MTT ** Signal mag phone                                OFF - ";
	mb.addCoil(256,0);                         // ���� ������ "Test MTT ** Signal GGS                                      OFF - ";
	mb.addCoil(257,0);                         // ���� ������ "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addCoil(258,0);                         // ���� ������ "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addCoil(259,0);                         // ���� ������ "Test MTT ** Signal GGS                                      ON  - ";

	mb.addCoil(260,0);                         // ���� ������ "Test MTT ** Signal LineL                                    ON  - ";
	mb.addCoil(261,0);                         // ���� ������ "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addCoil(262,0);                         // ���� ������ "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addCoil(263,0);                         // ���� ������ "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addCoil(264,0);                         // ���� ������ "Test microphone PTT  (CTS)                                  OFF - ";
	mb.addCoil(265,0);                         // ���� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addCoil(266,0);                         // ���� ������ "Test microphone PTT  (CTS)                                  ON  - ";
	mb.addCoil(267,0);                         // ���� ������ "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addCoil(268,0);                         // ���� ������ "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addCoil(269,0);                         // ���� ������ ������������ ����������� ������� 

	mb.addCoil(270,0);                         // ���� ������ "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addCoil(271,0);                         // ���� ������ "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addCoil(272,0);                         // ���� ������ "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addCoil(273,0);                         // ���� ������ "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addCoil(274,0);                         // ���� ������ "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addCoil(275,0);                         // ���� ������ "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addCoil(276,0);                         // ���� ������ "Command sensor tangenta nognaja                             OFF - ";
	mb.addCoil(277,0);                         // ���� ������ "Command sensor tangenta nognaja                             ON  - ";
	mb.addCoil(278,0);                         // ���� ������ "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addCoil(279,0);                         // ���� ������ "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addCoil(280,0);                         // ���� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addCoil(281,0);                         // ���� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addCoil(282,0);                         // ���� ������ "Test GGS ** Signal LineL                                    OFF - ";
	mb.addCoil(283,0);                         // ���� ������ "Test GGS ** Signal LineR                                    OFF - ";
	mb.addCoil(284,0);                         // ���� ������ "Test GGS ** Signal mag radio                                OFF - ";
	mb.addCoil(285,0);                         // ���� ������ "Test GGS ** Signal mag phone                                OFF - ";
	mb.addCoil(286,0);                         // ���� ������ "Test GGS ** Signal GGS                                      OFF - ";
	mb.addCoil(287,0);                         // ���� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addCoil(288,0);                         // ���� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addCoil(289,0);                         // ���� ������ "Test GGS ** Signal GGS                                      ON  - ";

	mb.addCoil(290,0);                         // ���� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addCoil(291,0);                         // ���� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addCoil(292,0);                         // ���� ������ "Test GGS ** Signal mag phone                                ON  - ";
	mb.addCoil(293,0);                         // ���� ������ ADC1 ���������� 12/3 �����
	mb.addCoil(294,0);                         // ���� ������ ADC14 ���������� 12/3 ����� Radio1
	mb.addCoil(295,0);                         // ���� ������ ADC14 ���������� 12/3 ����� Radio2
	mb.addCoil(296,0);                         // ���� ������ ADC14 ���������� 12/3 ����� ���
	mb.addCoil(297,0);                         // ���� ������ ADC15 ���������� ���������� 3,6 ������
	mb.addCoil(298,0);                         // ���� ������ "Test Microphone ** Signal mag phone                         ON  - ";      
	mb.addCoil(299,0);                         // ���� ������ "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addCoil(300,0);                         // ���� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addCoil(301,0);                         // ���� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addCoil(302,0);                         // ���� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addCoil(303,0);                         // ���� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addCoil(304,0);                         // ���� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addCoil(305,0);                         // ���� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addCoil(306,0);                         // ���� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addCoil(307,0);                         // ���� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addCoil(308,0);                         // ���� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addCoil(309,0);                         // ���� ������ "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addCoil(310,0);                         // ���� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addCoil(311,0);                         // ���� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addCoil(312,0);                         // ���� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addCoil(313,0);                         // ���� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addCoil(314,0);                         // ���� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addCoil(315,0);                         // ���� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addCoil(316,0);                         // ���� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addCoil(317,0);                         // ���� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addCoil(318,0);                         // ���� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addCoil(319,0);                         // ���� ������ "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addCoil(320,0);                         // ���� ������ "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addCoil(321,0);                         // ���� ������ "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addCoil(322,0);                         // ���� ������ "Test Microphone ** Signal LineL                             OFF - ";
	mb.addCoil(323,0);                         // ���� ������ "Test Microphone ** Signal LineR                             OFF - ";
	mb.addCoil(324,0);                         // ���� ������ "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addCoil(325,0);                         // ���� ������ "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addCoil(326,0);                         // ���� ������ "Test Microphone ** Signal GGS                               OFF - ";
	mb.addCoil(327,0);                         // ���� ������ "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addCoil(328,0);                         // ���� ������ "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addCoil(329,0);                         // ���� ������ ��� ������� �������

	mb.addCoil(330,0);                         // ���� ������ "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addCoil(331,0);                         // ���� ������ "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	mb.addCoil(332,0);                         // ���� ������ "Test GGS ** Signal mag radio   


	mb.addIsts(81,0);    // ����� ����a ��������� ��������� ������� CTS
	mb.addIsts(82,0);    // ����� ����a ��������� ��������� ������� DSR
	mb.addIsts(83,0);    // ����� ����a ��������� ��������� ������� DCD



	//mb.addHreg(40000);  // 

	mb.addHreg(1,0);  // �������� ������ � ����� 1
	mb.addHreg(2,0);  // �������� ������ � ����� 1
	mb.addHreg(3,0);  // �������� ������ � ����� 1
	mb.addHreg(4,0);  // �������� ������ � ����� 1
	mb.addHreg(5,0);  // �������� ������ � ����� 1
	mb.addHreg(6,0);  // �������� ������ � ����� 1
	mb.addHreg(7,0);  // �������� ������ � ����� 1
	mb.addHreg(8,0);  // 
	mb.addHreg(9,0);  // 




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
	mb.addHreg(48,4);  // ����� ��� ������ ����� �����������
	mb.addHreg(49,4);  // ����� ��� ������ ����� �����������
	mb.addHreg(50,5);  // ����� ������ ������ ����� �����������
	mb.addHreg(51,6);  // ����� ������� ������ ����� �����������
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


	mb.addHreg(200,0);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	mb.addHreg(201,0);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	mb.addHreg(202,0);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	mb.addHreg(203,0);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	mb.addHreg(204,0);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	mb.addHreg(205,0);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	mb.addHreg(206,0);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	mb.addHreg(207,0);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	mb.addHreg(208,0);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	mb.addHreg(209,0);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	mb.addHreg(210,0);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	mb.addHreg(211,0);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	mb.addHreg(212,0);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	mb.addHreg(213,0);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	mb.addHreg(214,0);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	mb.addHreg(215,0);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	mb.addHreg(216,0);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	mb.addHreg(217,0);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	mb.addHreg(218,0);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	mb.addHreg(219,0);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

	mb.addHreg(220,0);                         // A���� �������� ������ "Command PTT headset instructor (CTS)                        OFF - ";
	mb.addHreg(221,0);                         // A���� �������� ������ "Command PTT headset instructor (CTS)                        ON  - ";
	mb.addHreg(222,0);                         // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
	mb.addHreg(223,0);                         // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
	mb.addHreg(224,0);                         // A���� �������� ������ "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addHreg(225,0);                         // A���� �������� ������ "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addHreg(226,0);                         // A���� �������� ������ "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addHreg(227,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addHreg(228,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addHreg(229,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addHreg(230,0);                         // A���� �������� ������ "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addHreg(231,0);                         // A���� �������� ������ "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addHreg(232,0);                         // A���� �������� ������ "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addHreg(233,0);                         // A���� �������� ������ "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addHreg(234,0);                         // A���� �������� ������ "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addHreg(235,0);                         // A���� �������� ������ "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addHreg(236,0);                         // A���� �������� ������ "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addHreg(237,0);                         // A���� �������� ������ "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addHreg(238,0);                         // A���� �������� ������ "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addHreg(239,0);                         // A���� �������� ������ ADC0  ��� x1 

	mb.addHreg(240,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addHreg(241,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addHreg(242,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addHreg(243,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addHreg(244,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addHreg(245,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addHreg(246,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addHreg(247,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addHreg(248,0);                         // A���� �������� ������ "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addHreg(249,0);                         // A���� �������� ������ ADC2 ��� x10

	mb.addHreg(250,0);                         // A���� �������� ������ "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addHreg(251,0);                         // A���� �������� ������ "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addHreg(252,0);                         // A���� �������� ������ "Test MTT ** Signal LineL                                    OFF - ";
	mb.addHreg(253,0);                         // A���� �������� ������ "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addHreg(254,0);                         // A���� �������� ������ "Test MTT ** Signal mag radio                                OFF - ";
	mb.addHreg(255,0);                         // A���� �������� ������ "Test MTT ** Signal mag phone                                OFF - ";
	mb.addHreg(256,0);                         // A���� �������� ������ "Test MTT ** Signal GGS                                      OFF - ";
	mb.addHreg(257,0);                         // A���� �������� ������ "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addHreg(258,0);                         // A���� �������� ������ "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addHreg(259,0);                         // A���� �������� ������ "Test MTT ** Signal GGS                                      ON  - ";

	mb.addHreg(260,0);                         // A���� �������� ������ "Test MTT ** Signal LineL                                    ON  - ";
	mb.addHreg(261,0);                         // A���� �������� ������ "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addHreg(262,0);                         // A���� �������� ������ "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addHreg(263,0);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addHreg(264,0);                         // A���� �������� ������ "Test microphone PTT  (CTS)                                  OFF - ";
	mb.addHreg(265,0);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addHreg(266,0);                         // A���� �������� ������ "Test microphone PTT  (CTS)                                  ON  - ";
	mb.addHreg(267,0);                         // A���� �������� ������ "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addHreg(268,0);                         // A���� �������� ������ "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addHreg(269,0);                         // A���� �������� ������ ������������ ����������� �������

	mb.addHreg(270,0);                         // A���� �������� ������ "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addHreg(271,0);                         // A���� �������� ������ "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addHreg(272,0);                         // A���� �������� ������ "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addHreg(273,0);                         // A���� �������� ������ "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addHreg(274,0);                         // A���� �������� ������ "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addHreg(275,0);                         // A���� �������� ������ "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addHreg(276,0);                         // A���� �������� ������ "Command sensor tangenta nognaja                             OFF - ";
	mb.addHreg(277,0);                         // A���� �������� ������ "Command sensor tangenta nognaja                             ON  - ";
	mb.addHreg(278,0);                         // A���� �������� ������ "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addHreg(279,0);                         // A���� �������� ������ "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addHreg(280,0);                         // A���� �������� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addHreg(281,0);                         // A���� �������� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addHreg(282,0);                         // A���� �������� ������ "Test GGS ** Signal LineL                                    OFF - ";
	mb.addHreg(283,0);                         // A���� �������� ������ "Test GGS ** Signal LineR                                    OFF - ";
	mb.addHreg(284,0);                         // A���� �������� ������ "Test GGS ** Signal mag radio                                OFF - ";
	mb.addHreg(285,0);                         // A���� �������� ������ "Test GGS ** Signal mag phone                                OFF - ";
	mb.addHreg(286,0);                         // A���� �������� ������ "Test GGS ** Signal GGS                                      OFF - ";
	mb.addHreg(287,0);                         // A���� �������� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addHreg(288,0);                         // A���� �������� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addHreg(289,0);                         // A���� �������� ������ "Test GGS ** Signal GGS                                      ON  - ";

	mb.addHreg(290,0);                         // A���� �������� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addHreg(291,0);                         // A���� �������� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addHreg(292,0);                         // A���� �������� ������ "Test GGS ** Signal mag phone                                ON  - ";
	mb.addHreg(293,0);                         // A���� ��������  ������ ADC1 ���������� 12/3 �����
	mb.addHreg(294,0);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio1
	mb.addHreg(295,0);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio2
	mb.addHreg(296,0);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� ���
	mb.addHreg(297,0);                         // A���� ��������  ������ ADC15 ���������� ���������� 3,6 ������
	mb.addHreg(298,0);                         // A���� �������� ������ "Test Microphone ** Signal mag phone                         ON  - ";    
	mb.addHreg(299,0);                         // A���� �������� ������ "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addHreg(300,0);                         // A���� �������� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addHreg(301,0);                         // A���� �������� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addHreg(302,0);                         // A���� �������� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addHreg(303,0);                         // A���� �������� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addHreg(304,0);                         // A���� �������� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addHreg(305,0);                         // A���� �������� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addHreg(306,0);                         // A���� �������� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addHreg(307,0);                         // A���� �������� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(308,0);                         // A���� �������� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(309,0);                         // A���� �������� ������ "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addHreg(310,0);                         // A���� �������� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addHreg(311,0);                         // A���� �������� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addHreg(312,0);                         // A���� �������� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addHreg(313,0);                         // A���� �������� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addHreg(314,0);                         // A���� �������� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addHreg(315,0);                         // A���� �������� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addHreg(316,0);                         // A���� �������� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addHreg(317,0);                         // A���� �������� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(318,0);                         // A���� �������� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(319,0);                         // A���� �������� ������ "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addHreg(320,0);                         // A���� �������� ������ "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addHreg(321,0);                         // A���� �������� ������ "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addHreg(322,0);                         // A���� �������� ������ "Test Microphone ** Signal LineL                             OFF - ";
	mb.addHreg(323,0);                         // A���� �������� ������ "Test Microphone ** Signal LineR                             OFF - ";
	mb.addHreg(324,0);                         // A���� �������� ������ "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addHreg(325,0);                         // A���� �������� ������ "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addHreg(326,0);                         // A���� �������� ������ "Test Microphone ** Signal GGS                               OFF - ";
	mb.addHreg(327,0);                         // A���� �������� ������ "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addHreg(328,0);                         // A���� �������� ������ "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addHreg(329,0);                         // A���� �������� ������ ��� ����������� �������                             // 

	mb.addHreg(330,0);                         // A���� �������� ������ "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addHreg(331,0);                         // A���� �������� ������ "Test Radio2 ** Signal mag radio                             ON  - ";
	mb.addHreg(332,0);                         // A���� �������� ������ "Test GGS    ** Signal mag radio                             ON  - ";


	
	// ++++++++++++++++++++++ �������� �������� ������ ��� �������� ������� ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	mb.addHreg(400,0);                         // A���� ���������� ADC0  ��� x1 
	mb.addHreg(401,0);                         // A���� ���������� ADC1 ���������� 12/3 �����
	mb.addHreg(402,0);                         // A���� ���������� ADC2 ��� x10
	mb.addHreg(403,0);                         // A���� ���������� ADC14 ���������� 12/3 ����� Radio1
	mb.addHreg(404,0);                         // A���� ���������� ADC14 ���������� 12/3 ����� Radio2
	mb.addHreg(405,0);                         // A���� ���������� ADC14 ���������� 12/3 ����� ���
	mb.addHreg(406,0);                         // A���� ���������� ADC15 ���������� ���������� 3,6 ������
	mb.addHreg(407,0);                         // A���� 
	mb.addHreg(408,0);                         // A���� 
	mb.addHreg(409,0);                         // A����  

	mb.addHreg(410,0);                         // A���� �������� 
	mb.addHreg(411,0);                         // A���� ��������  
	mb.addHreg(412,0);                         // A���� ��������  
	mb.addHreg(413,0);                         // A���� ��������  
	mb.addHreg(414,0);                         // A���� ��������  
	mb.addHreg(415,0);                         // A���� ��������  
	mb.addHreg(416,0);                         // A���� ��������  
	mb.addHreg(417,0);                         // A���� ��������  
	mb.addHreg(418,0);                         // A���� ��������  
	mb.addHreg(419,0);                         // A���� ��������  

	mb.addHreg(420,0);                         // A����  ;
	mb.addHreg(421,0);                         // A����  ;
	mb.addHreg(422,0);                         // A����  ;
	mb.addHreg(423,0);                         // A����  ;
	mb.addHreg(424,0);                         // A���� ������ ��������� "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addHreg(425,0);                         // A���� ������ ��������� "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addHreg(426,0);                         // A���� ������ ��������� "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addHreg(427,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addHreg(428,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addHreg(429,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addHreg(430,0);                         // A���� ������ ��������� "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addHreg(431,0);                         // A���� ������ ��������� "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addHreg(432,0);                         // A���� ������ ��������� "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addHreg(433,0);                         // A���� ������ ��������� "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addHreg(434,0);                         // A���� ������ ��������� "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addHreg(435,0);                         // A���� ������ ��������� "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addHreg(436,0);                         // A���� ������ ��������� "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addHreg(437,0);                         // A���� ������ ��������� "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addHreg(438,0);                         // A���� ������ ��������� "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addHreg(439,0);                         // A���� ������ ��������� ADC0  ��� x1 

	mb.addHreg(440,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addHreg(441,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addHreg(442,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addHreg(443,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addHreg(444,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addHreg(445,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addHreg(446,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addHreg(447,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addHreg(448,0);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addHreg(449,0);                         // A���� ������ ��������� ADC2 ��� x10

	mb.addHreg(450,0);                         // A���� ������ ��������� "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addHreg(451,0);                         // A���� ������ ��������� "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addHreg(452,0);                         // A���� ������ ��������� "Test MTT ** Signal LineL                                    OFF - ";
	mb.addHreg(453,0);                         // A���� ������ ��������� "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addHreg(454,0);                         // A���� ������ ��������� "Test MTT ** Signal mag radio                                OFF - ";
	mb.addHreg(455,0);                         // A���� ������ ��������� "Test MTT ** Signal mag phone                                OFF - ";
	mb.addHreg(456,0);                         // A���� ������ ��������� "Test MTT ** Signal GGS                                      OFF - ";
	mb.addHreg(457,0);                         // A���� ������ ��������� "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addHreg(458,0);                         // A���� ������ ��������� "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addHreg(459,0);                         // A���� ������ ��������� "Test MTT ** Signal GGS                                      ON  - ";

	mb.addHreg(460,0);                         // A���� ������ ��������� "Test MTT ** Signal LineL                                    ON  - ";
	mb.addHreg(461,0);                         // A���� ������ ��������� "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addHreg(462,0);                         // A���� ������ ��������� "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addHreg(463,0);                         // A���� ������ ��������� "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addHreg(464,0);                         // 
	mb.addHreg(465,0);                         // A���� ������ ��������� "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addHreg(466,0);                         // 
	mb.addHreg(467,0);                         // A���� ������ ��������� "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addHreg(468,0);                         // A���� ������ ��������� "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addHreg(469,0);                         // ������������ �������� ����������� ������� �������

	mb.addHreg(470,0);                         // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addHreg(471,0);                         // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addHreg(472,0);                         // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addHreg(473,0);                         // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addHreg(474,0);                         // A���� ������ ��������� "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addHreg(475,0);                         // A���� ������ ��������� "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addHreg(476,0);                         // A���� ������ ��������� "Command sensor tangenta nognaja                             OFF - ";
	mb.addHreg(477,0);                         // A���� ������ ��������� "Command sensor tangenta nognaja                             ON  - ";
	mb.addHreg(478,0);                         // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addHreg(479,0);                         // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addHreg(480,0);                         // A���� ������ ��������� "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addHreg(481,0);                         // A���� ������ ��������� "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addHreg(482,0);                         // A���� ������ ��������� "Test GGS ** Signal LineL                                    OFF - ";
	mb.addHreg(483,0);                         // A���� ������ ��������� "Test GGS ** Signal LineR                                    OFF - ";
	mb.addHreg(484,0);                         // A���� ������ ��������� "Test GGS ** Signal mag radio                                OFF - ";
	mb.addHreg(485,0);                         // A���� ������ ��������� "Test GGS ** Signal mag phone                                OFF - ";
	mb.addHreg(486,0);                         // A���� ������ ��������� "Test GGS ** Signal GGS                                      OFF - ";
	mb.addHreg(487,0);                         // A���� ������ ��������� "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addHreg(488,0);                         // A���� ������ ��������� "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addHreg(489,0);                         // A���� ������ ��������� "Test GGS ** Signal GGS                                      ON  - ";

	mb.addHreg(490,0);                         // A���� ������ ��������� "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addHreg(491,0);                         // A���� ������ ��������� "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addHreg(492,0);                         // A���� ������ ��������� "Test GGS ** Signal mag phone                                ON  - ";
	mb.addHreg(493,0);                         // A���� ������ ��������� ADC1 ���������� 12/3 �����
	mb.addHreg(494,0);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio1
	mb.addHreg(495,0);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio2
	mb.addHreg(496,0);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� ���
	mb.addHreg(497,0);                         // A���� ������ ��������� ADC15 ���������� ���������� 3,6 ������
	mb.addHreg(498,0);                         // A���� ������ ��������� "Test Microphone ** Signal mag phone                         ON  - "; 
	mb.addHreg(499,0);                         // A���� ������ ��������� "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addHreg(500,0);                         // A���� ������ ��������� "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addHreg(501,0);                         // A���� ������ ��������� "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addHreg(502,0);                         // A���� ������ ��������� "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addHreg(503,0);                         // A���� ������ ��������� "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addHreg(504,0);                         // A���� ������ ��������� "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addHreg(505,0);                         // A���� ������ ��������� "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addHreg(506,0);                         // A���� ������ ��������� "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addHreg(507,0);                         // A���� ������ ��������� "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(508,0);                         // A���� ������ ��������� "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(509,0);                         // A���� ������ ��������� "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addHreg(510,0);                         // A���� ������ ��������� "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addHreg(511,0);                         // A���� ������ ��������� "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addHreg(512,0);                         // A���� ������ ��������� "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addHreg(513,0);                         // A���� ������ ��������� "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addHreg(514,0);                         // A���� ������ ��������� "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addHreg(515,0);                         // A���� ������ ��������� "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addHreg(516,0);                         // A���� ������ ��������� "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addHreg(517,0);                         // A���� ������ ��������� "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(518,0);                         // A���� ������ ��������� "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(519,0);                         // A���� ������ ��������� "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addHreg(520,0);                         // A���� ������ ��������� "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addHreg(521,0);                         // A���� ������ ��������� "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addHreg(522,0);                         // A���� ������ ��������� "Test Microphone ** Signal LineL                             OFF - ";
	mb.addHreg(523,0);                         // A���� ������ ��������� "Test Microphone ** Signal LineR                             OFF - ";
	mb.addHreg(524,0);                         // A���� ������ ��������� "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addHreg(525,0);                         // A���� ������ ��������� "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addHreg(526,0);                         // A���� ������ ��������� "Test Microphone ** Signal GGS                               OFF - ";
	mb.addHreg(527,0);                         // A���� ������ ��������� "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addHreg(528,0);                         // A���� ������ ��������� "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addHreg(529,0);                         // ��� ����������� ������� �������
	mb.addHreg(530,0);                         // A���� ������ ��������� "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addHreg(531,0);                         // A���� ������ ��������� "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	mb.addHreg(532,0);                         // A���� ������ ��������� "Test GGS ** Signal mag radio   


}
void test_system()
{
	//prer_Kmerton_On = 0;   
	////reg_Kamerton();
	//Serial.print(regs_in[0],HEX);
	//Serial.print("--");
	//////  Serial.println(regs_out[0],DEC);
	//Serial.print(regs_in[1],HEX);
	//Serial.print("--");
	////  Serial.println(regs_out[1],DEC);
	//Serial.print(regs_in[2],HEX);
	//Serial.print("--");
	//Serial.print(regs_in[3],HEX);

	//Serial.print("-   -");
	//Serial.print(mb.Hreg(10279),HEX);
	//Serial.print("--");
	//Serial.print(mb.Hreg(10278),HEX);
	//Serial.print("--");
	//Serial.print(mb.Hreg(10277),HEX);
	//Serial.print("--");
	//Serial.print(mb.Hreg(10276),HEX);
	//Serial.print("--");
	//Serial.print(mb.Hreg(10275),HEX);
	//Serial.print("--");
	//Serial.print(mb.Hreg(10274),HEX);
	//Serial.print("--");
	//Serial.print(mb.Hreg(10273),HEX);
	//Serial.print("--");
	//Serial.println(mb.Hreg(10272),HEX);

	//////  Serial.println(regs_out[2],DEC);*/

	//prer_Kmerton_On = 1;   
	////delay(1000);
}

void set_serial()
{
   wdt_disable(); //
   clear_serial3();
   delay(200);
// ����� ��� �����
	Serial.println("COM port find...");
	do
	{
	  if (Serial3.available() == 5) 
	  {
		//Read buffer
		inputByte_0 = Serial3.read();
		delay(100);    
		inputByte_1 = Serial3.read();
		delay(100);      
		inputByte_2 = Serial3.read();
		delay(100);      
		inputByte_3 = Serial3.read();
		delay(100);
		inputByte_4 = Serial3.read();   
	  }
	  //Check for start of Message
	  if(inputByte_0 == 16)
	  {       
		   //Detect Command type
		   switch (inputByte_1) 
		   {
			  case 127:
				 //Set PIN and value
				 switch (inputByte_2)
				{
				  case 4:
					if(inputByte_3 == 255)
					{
					//  digitalWrite(ledPin_13, HIGH); 
					  break;
					}
					else
					{
					 // digitalWrite(ledPin_13, LOW); 
					  break;
					}
				  break;
				} 
				break;
			  case 128:
				//Say hello
				Serial3.print("HELLO FROM KAMERTON ");
				portFound = true;
				Serial.println("COM port find OK!.");
				break;
			} 
			//Clear Message bytes
			inputByte_0 = 0;
			inputByte_1 = 0;
			inputByte_2 = 0;
			inputByte_3 = 0;
			inputByte_4 = 0;
	   }
	   Serial.print(".");
	   clear_serial3();
	   delay(500);
	   mcp_Analog.digitalWrite(Front_led_Red, blink_red); 
	   mcp_Analog.digitalWrite(Front_led_Blue, !blink_red); 
	   blink_red = !blink_red;
	   digitalWrite(ledPin13,!digitalRead(ledPin13));
	} while(portFound == false);
	//wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
	digitalWrite(ledPin13,LOW);
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
}
void clear_serial()
{
  if (Serial.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {

			while (Serial.available())
				{
					 Serial.read();
				}
		   }
}
void clear_serial2()
{
  if (Serial2.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {

			while (Serial2.available())
				{
					 Serial2.read();
				}
		   }
}

void clear_serial1()
{
  if (Serial1.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {

			while (Serial1.available())
				{
					 Serial1.read();
				}
		   }
}
void clear_serial3()
{
  if (Serial3.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {

			while (Serial3.available())
				{
					 Serial3.read();
				}
		   }
}

void set_SD()
{
		if (!sd.begin(chipSelect)) 
		{
			//Serial.println("initialization SD failed!");
			mb.Coil(125,false); 
		}
	else
		{
			  myFile = sd.open("example.txt", FILE_WRITE);
			  myFile.close();

			  // Check to see if the file exists:
			  if (sd.exists("example.txt")) 
			  {
				  mb.Coil(125,true); 
				  sd.remove("example.txt");
			   // Serial.println("example.txt exists.");
			  }
			  else 
			  {
			   // Serial.println("example.txt doesn't exist.");
				mb.Coil(125,false); 
			  }
			}

	UpdateRegs(); 
	delay(100);
	mb.Hreg(adr_control_command,0);  
}
void file_del_SD()
{
	if (!sd.begin(chipSelect)) 
		{
			//Serial.println("initialization SD failed!");
			//mb.Coil(125,false); 
		}
	else
		{
			  myFile = sd.open(fileName_F);
				//myFile = sd.open("example.txt", FILE_WRITE);
				//myFile.close();

			  // Check to see if the file exists:
			  if (sd.exists(fileName_F)) 
			  {
				 // mb.Coil(125,true); 
				  sd.remove(fileName_F);
			   // Serial.println("example.txt exists.");
			  }
			  else 
			  {
			   // Serial.println("example.txt doesn't exist.");
				//mb.Coil(125,false); 
			  }
			}

	UpdateRegs(); 
	delay(100);
	mb.Hreg(adr_control_command,0);  

}
//------------------------------------------------------------------------------

void setup()
{
	wdt_disable(); // ����������� ������ �� ������� �� ������� ���������� ��� bootloop
	Wire.begin();
	if (!RTC.begin())                               // ��������� ����� 
		{
			Serial.println("RTC failed");
			while(1);
		};
	setup_mcp();                                    // ��������� ����� ����������  
	mcp_Analog.digitalWrite(DTR, HIGH);             // ���������� ������ (������)���������� � ����������
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, HIGH); 
	Serial.begin(9600);                             // ����������� � USB ��
	Serial1.begin(115200);                          // ����������� � ��������� ������ ��������
	mb.config(&Serial3, 57600, SERIAL_8N1);                   // Config Modbus Serial (port, speed, byte format) 

	Serial2.begin(57600);                            // 
	Serial.println(" ");
	Serial.println(" ***** Start system  *****");
	Serial.println(" ");
	portFound = false;

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

//	set_serial();                                    // ����� ��� ����� ����������� � ����������

	//set_serial();                                    // ����� ��� ����� ����������� � ����������
	AD9850.reset();                                  //reset module
	delay(500);
	AD9850.powerDown();                              //set signal output to LOW
	delay(100);
	AD9850.set_frequency(0,0,1000);                   //set power=UP, phase=0, 1kHz frequency
	delay(1000); 

	// DateTime set_time = DateTime(15, 6, 15, 10, 51, 0); // ������� ������ � ������� � ������ "set_time"
	// RTC.adjust(set_time);                                // ������
	serial_print_date();
	Serial.println(" ");

	setup_resistor();                               // ��������� ��������� ���������

	setup_regModbus();                              // ��������� ��������� MODBUS

	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 196 �������� � �������� �����
	regs_out[2]= 0x7F;                              // 127 �������� � �������� �����

	//mb.Hreg(40004+buffer,Serial1.read());

	 mb.Coil(21,0);                              // XP2-2     sensor "���."  
	 mb.Coil(22,0);                              // XP5-3     sensor "��C."
	 mb.Coil(23,0);                              // XP3-3     sensor "��-�����1."
	 mb.Coil(24,0);                              // XP4-3     sensor "��-�����2."
//	 mb.Coil(8,0);                               // ��������� ������� ��������
	UpdateRegs();                                   // �������� ���������� � ���������

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
  //sd.ls (LS_R | LS_DATE | LS_SIZE);
 

	MsTimer2::set(30, flash_time);                   // 30ms ������ ������� ���������
	resistor(1, 200);                                // ���������� ������� �������
	resistor(2, 200);                                // ���������� ������� �������
	preob_num_str();
	list_file();                                     // ����� ������ ������ � ��� ����  
	default_mem_porog();
	prer_Kmerton_On = true;                          // ��������� ���������� �� ��������
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
//	logTime = micros();
	MsTimer2::start();                               // �������� ������ ����������
	Serial.println(" ");                             //
	Serial.println("System initialization OK!.");    // ���������� � ���������� ���������
	//wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
}

void loop()
{
	control_command();

//	delay(100);
	/*
	 Serial.print(regs_out[0],HEX);
	 Serial.print("--");
	 Serial.print(regs_out[1],HEX);
	 Serial.print("--");
	 Serial.print(regs_out[2],HEX);
	 Serial.print("    ");
	 */
	 //Serial.print(mb.Hreg(40004),HEX); 
	 //Serial.print("--");
	 //Serial.print(mb.Hreg(40005),HEX); 
	 //Serial.print("--");
	 //Serial.print(mb.Hreg(40006),HEX); 
	 //Serial.print("--");
	 //Serial.println(mb.Hreg(40007),HEX); 

	//Serial.print(	mb.Hreg(136),HEX);    // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	//Serial.print("--");
	//Serial.println(	mb.Hreg(137),HEX);    // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
 
}
