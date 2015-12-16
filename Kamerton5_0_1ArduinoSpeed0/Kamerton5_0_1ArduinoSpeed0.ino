/*

 Kamerton5_0_1ArduinoSpeed.ino
 VisualStudio
 
 Программа тестирования модуля "Камертон" (Базовый вариант)
 Версия:      - 5_0_1Speed
 Дата:        - 15.12.2015г.
 Организация: - ООО "Децима"
 Автор:       - Мосейчук А.В.
 Версия: Обновленная версия от 15.12.2015г. Новая плата с учетом добавления 
 высоковольтного  модуля для испытания на пробой.
 Замена библиотеки MODBUS

 Реализовано:
 -
 - прерывание 30мс,
 - передача/прием по СОМ порту,
 - подсчет контролных сумм, связь с Камертоном, 
 - Расширение MCP23017
 - модуль реле, 
 - чтение всех портов, 
 - подключен звуковой генератор
 - Подключена SD память
 - подключены часы, память, 
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
Новые настройки регистров MODBUS

mb.addCoil(1);   Добавить  регистр   1-1000
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
//****************** Регистры работы с модулем Аудио-1 ******************************************************

//byte regs_in[5];                                           // Регистры работы с платой Аудио-1
byte regs_out[4];                                            // Регистры работы с платой Аудио-1
byte regs_crc[1];                                            // Регистры работы с платой Аудио-1 контрольная сумма
byte regs_temp                 = 0;
byte regs_temp1                = 0;
byte Stop_Kam                  = 0;                          // Флаг индикации чтения инф. из Аудио-1
bool test_repeat               = true;                       // Флаг повторения теста
volatile bool prer_Kmerton_On  = true;                       // Флаг разрешение прерывания Аудио-1
volatile bool prer_Kmerton_Run = false;                      // Флаг разрешение прерывания Аудио-1
unsigned char bufferK;                                       // Счетчик количества принимаемых байт
#define BUFFER_SIZEK           64                            // Размер буфера Камертон не более 128 байт

//------------------------------------------------------------------------------------------------------------

#define  ledPin13  13                               // Назначение светодиодов на плате
#define  ledPin12  12                               // Назначение светодиодов на плате
#define  ledPin11  11                               // Назначение светодиодов на плате
#define  ledPin10  10                               // Назначение светодиодов на плате
#define  Front_led_Blue 14                          // Назначение светодиодов на передней панели
#define  Front_led_Red  15                          // Назначение светодиодов на передней панели
//--------------------------------------------------------------------------------------------------------

//  Порты управления платой Аудио -1
#define DTR  8                                      // DTR out выходной сигнал  сформировать 0 для старта
#define RTS  9                                      // RTS out выходной сигнал   
#define CTS  5                                      // CTS in  входной сигнал  флаг нажатия тангенты контролировать!!!!
#define DSR  6                                      // DSR in  входной сигнал  флаг нажатия "Связь - передача"
#define DCD  7                                      // DCD in  входной сигнал  флаг снятия трубки с ложемента 


//  Порты управления платой Arduino Nano
#define  kn1Nano   34                               // Назначение кнопок управления Nano генератор качения
#define  kn2Nano   36                               // Назначение кнопок управления Nano генератор 1000 гц
#define  kn3Nano   38                               // Назначение кнопок управления Nano генератор 2000 гц
#define  InNano12  40                               // Назначение входов - индикация генератор 1000 или 2000 гц
#define  InNano13  39                               // Назначение входов - индикация генератор качения 

//------------------------------------------------------------------------------------------------------------
MCP23017 mcp_Out1;                                  // Назначение портов расширения MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                  // Назначение портов расширения MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                // Назначение портов расширения MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------

//******************** Настройка звукового модуля *************************************************************
//AH_AD9850(int CLK, int FQUP, int BitData, int RESET);
AH_AD9850 AD9850(23, 25, 27, 29);
//-------------------------------------------------------------------------------------------------------------
//+++++++++++++++++++++++++++++ Внешняя память +++++++++++++++++++++++++++++++++++++++
int deviceaddress        = 80;                      // Адрес микросхемы памяти
unsigned int eeaddress   =  0;                      // Адрес ячейки памяти
byte hi;                                            // Старший байт для преобразования числа
byte low;                                           // Младший байт для преобразования числа

//+++++++++++++++++++++++ Настройка электронного резистора +++++++++++++++++++++++++++++++++++++
#define address_AD5252   0x2F                       // Адрес микросхемы AD5252  
#define control_word1    0x07                       // Байт инструкции резистор №1
#define control_word2    0x87                       // Байт инструкции резистор №2
byte resistance        = 0x00;                      // Сопротивление 0x00..0xFF - 0Ом..100кОм
//byte level_resist      = 0;                       // Байт считанных данных величины резистора
//-----------------------------------------------------------------------------------------------


#define FASTADC 1                                   // Ускорение считывания аналогового сигнала
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Текущее время 
const unsigned int adr_kontrol_day        PROGMEM      = 46; // адрес день
const unsigned int adr_kontrol_month      PROGMEM      = 47; // адрес месяц
const unsigned int adr_kontrol_year       PROGMEM      = 48; // адрес год
const unsigned int adr_kontrol_hour       PROGMEM      = 49; // адрес час
const unsigned int adr_kontrol_minute     PROGMEM      = 50; // адрес минута
const unsigned int adr_kontrol_second     PROGMEM      = 51; // адрес секунда

// Установка времени в контроллере
const unsigned int adr_set_kontrol_day    PROGMEM      = 52;   // адрес день
const unsigned int adr_set_kontrol_month  PROGMEM      = 53;   // адрес месяц
const unsigned int adr_set_kontrol_year   PROGMEM      = 54;   // адрес год
const unsigned int adr_set_kontrol_hour   PROGMEM      = 55;   // адрес час
const unsigned int adr_set_kontrol_minute PROGMEM      = 56;   // адрес минута

// Время старта теста
const unsigned int adr_Mic_Start_day      PROGMEM      = 96;   // адрес день
const unsigned int adr_Mic_Start_month    PROGMEM      = 97;   // адрес месяц
const unsigned int adr_Mic_Start_year     PROGMEM      = 98;   // адрес год
const unsigned int adr_Mic_Start_hour     PROGMEM      = 99;   // адрес час
const unsigned int adr_Mic_Start_minute   PROGMEM      = 100;  // адрес минута
const unsigned int adr_Mic_Start_second   PROGMEM      = 101;  // адрес секунда
// Время окончания теста
const unsigned int adr_Mic_Stop_day       PROGMEM       = 102; // адрес день
const unsigned int adr_Mic_Stop_month     PROGMEM       = 103; // адрес месяц
const unsigned int adr_Mic_Stop_year      PROGMEM       = 104; // адрес год
const unsigned int adr_Mic_Stop_hour      PROGMEM       = 105; // адрес час
const unsigned int adr_Mic_Stop_minute    PROGMEM       = 106; // адрес минута
const unsigned int adr_Mic_Stop_second    PROGMEM       = 107; // адрес секунда

// Продолжительность выполнения теста
const unsigned int adr_Time_Test_day      PROGMEM       = 108; // адрес день
const unsigned int adr_Time_Test_hour     PROGMEM       = 109; // адрес час
const unsigned int adr_Time_Test_minute   PROGMEM       = 110; // адрес минута
const unsigned int adr_Time_Test_second   PROGMEM       = 111; // адрес секунда
// Адрес текущего файла
const unsigned int adr_reg_temp_year      PROGMEM       = 112; // Регистр хранения переменной год  
const unsigned int adr_reg_temp_mon       PROGMEM       = 113; // Регистр хранения переменной месяц
const unsigned int adr_reg_temp_day       PROGMEM       = 114; // Регистр хранения переменной день 
const unsigned int adr_reg_file_name      PROGMEM       = 115; // Регистр хранения счетчик файлов  
const unsigned int adr_reg_file_tek       PROGMEM       = 116; // Регистр хранения счетчик файлов  

const unsigned int adr_control_command    PROGMEM       = 120; // Адрес передачи комманд на выполнение 
const unsigned int adr_reg_count_err      PROGMEM       = 121; // Адрес счетчика всех ошибок

const unsigned int adr_set_time           PROGMEM       = 36;  // адрес флаг установки
//-------------------------------------------------------------------------------------------------

const int adr_reg_ind_CTS                 PROGMEM       = 81;        // Адрес флагa индикации состояния сигнала CTS
const int adr_reg_ind_DSR                 PROGMEM       = 82;        // Адрес флагa индикации состояния сигнала DSR
const int adr_reg_ind_DCD                 PROGMEM       = 83;        // Адрес флагa индикации состояния сигнала DCD

//----------------------------------------------------------------------------------------------

//++++++++++++++++++++++ Работа с файлами +++++++++++++++++++++++++++++++++++++++
//#define chipSelect SS
#define chipSelect 49   // Основной
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
// созданы переменные, использующие функции библиотеки SD utility library functions: +++++++++++++++
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;


//++++++++++++++++++++ Назначение имени файла ++++++++++++++++++++++++++++++++++++++++++++
//const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "150101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13]            = FILE_BASE_NAME "00.KAM";
char fileName_p[13];
char fileName_F[13];
//------------------------------------------------------------------------------

char c;  // Для ввода символа с ком порта

// Serial output stream
ArduinoOutStream cout(Serial);
char bufferSerial2[128];  

//*********************Работа с именем файла ******************************

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

//------------------------- Уровни пороговых значений сигналов при тестировании устройств--------------------------------------
//++++++++++++++++++++++++++++ Заводские установки уровней порогов +++++++++++++++++++++++++++++++++++++

// Адреса внешней памяти для хранения порогов уровней измерения сигналов

const  int adr_porog_instruktor            = 0;                // 19 адресов 
const  int adr_porog_dispatcher            = 30;               // 19 адресов 
const  int adr_porog_MTT                   = 60;               // 21 адресов 
const  int adr_porog_GGS                   = 90;               // 29 адресов 
const  int adr_porog_Radio1                = 120;              // 20 адресов 
const  int adr_porog_Radio2                = 150;              // 20 адресов 
const  int adr_porog_Microphone            = 180;              //  
const  int adr_set_USB                     = 200;  
// end                                     = 160
//

//Новые пороги две ячейки, 2 байта
const  int adr_int_porog_instruktor            = 200;      // 19 адресов 
const  int adr_int_porog_dispatcher            = 250;      // 19 адресов 
const  int adr_int_porog_MTT                   = 300;      // 21 адресов 
const  int adr_int_porog_GGS                   = 350;      // 29 адресов 
const  int adr_int_porog_Radio1                = 420;      // 20 адресов 
const  int adr_int_porog_Radio2                = 460;      // 20 адресов 
const  int adr_int_porog_Microphone            = 500;      //
//end                                            550;


byte por_buffer[30] ;
int por_int_buffer[50] ;                 // Буфер хранения временной информации уровней порогов                      
//const byte porog_instruktor[]    PROGMEM  = {30,30,35,35,35,35,35,35,35,35,35,150,150,35,35,35,35,35,35,35,254};
const byte porog_instruktor[]    PROGMEM  = {
		 //++++++++++++++++  Test headset instructor ++++++++++++++++++++++++++++
25,			// 0                          // resistor(1, 30);     Установить уровень сигнала 30 мв
25,			// 1                          // resistor(2, 30);     Установить уровень сигнала 30 мв
15,			// 2                          // measure_vol_min(analog_FrontL,   40230,230,35);  уровень сигнала на выходе FrontL  
15,			// 3                          // measure_vol_min(analog_FrontR,   40231,231,35);  уровень сигнала на выходе FrontR
15,			// 4                          // measure_vol_min(analog_LineL,    40232,232,35);  уровень сигнала на выходе LineL 
15,			// 5                          // measure_vol_min(analog_LineR,    40233,233,35);  уровень сигнала на выходе LineR
15,			// 6                          // measure_vol_min(analog_mag_radio,40234,234,35);  уровень сигнала на выходе mag radio 
15,			// 7                          // measure_vol_min(analog_mag_phone,40235,235,35);  уровень сигнала на выходе mag phone
15,			// 8                          // measure_vol_min(analog_ggs,      40236,236,35);  уровень сигнала на выходе GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1,40237,237,35);  уровень сигнала на выходе GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2,40238,238,35);  уровень сигнала на выходе GG Radio2 
			//---------- Сигнал на вход подан ---------------------
101,      	// 11                         // measure_vol_max(analog_LineL,    40224,224,150); уровень сигнала на выходе LineL
80,		    // 12                         // measure_vol_max(analog_mag_phone,40226,226,150); уровень сигнала на выходе mag phone 
15, 		// 13                         // measure_vol_min(analog_FrontL,   40230,230,35);  уровень сигнала на выходе FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,   40231,231,35);  уровень сигнала на выходе FrontR 
15,			// 15                         // measure_vol_min(analog_LineR,    40233,233,35);  уровень сигнала на выходе LineR 
15,			// 16                         // measure_vol_min(analog_ggs,      40236,236,35);  уровень сигнала на выходе GGS 
15,			// 17                         // measure_vol_min(analog_gg_radio1,40237,237,35);  уровень сигнала на выходе GG Radio1
15  		// 18                         // measure_vol_min(analog_gg_radio2,40238,238,35);  уровень сигнала на выходе GG Radio2
};
  
const byte  porog_dispatcher[]    PROGMEM = {

		//	+++++++++++++Test headset dispatcher ++++++++++++++++++++++++++++++
40,			// 0                          // resistor(1, 30);   Установить уровень сигнала 30 мв
40,			// 1                          // resistor(2, 30);   Установить уровень сигнала 30 мв
15,			// 2                          // measure_vol_min(analog_FrontL,   40240,240,35);  уровень сигнала на выходе FrontL 
15,			// 3                          // measure_vol_min(analog_FrontR,   40241,241,35);  уровень сигнала на выходе FrontR
15,			// 4                          // measure_vol_min(analog_LineL,    40242,242,35);  уровень сигнала на выходе LineL
15,			// 5                          // measure_vol_min(analog_LineR,    40243,243,35);  уровень сигнала на выходе LineR
15,			// 6                          // measure_vol_min(analog_mag_radio,40244,244,35);  уровень сигнала на выходе mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone,40245,245,35);  уровень сигнала на выходе mag phone
15,			// 8                          // measure_vol_min(analog_ggs,      40246,246,35);  уровень сигнала на выходе GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1,40247,247,35);  уровень сигнала на выходе GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2,40248,248,35);  уровень сигнала на выходе GG Radio2 
			// ++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
100,	   	// 11                         // measure_vol_max(analog_LineL,    40227,227,200); уровень сигнала на выходе LineL
75,	        // 12                         // measure_vol_max(analog_mag_phone,40229,229,200); уровень сигнала на выходе mag phone
15,			// 13                         // measure_vol_min(analog_FrontL,   40240,240,35);  уровень сигнала на выходе FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,   40241,241,35);  уровень сигнала на выходе FrontR 
15,			// 15                         // measure_vol_min(analog_LineR,    40243,243,35);  уровень сигнала на выходе LineR 
15,			// 16                         // measure_vol_min(analog_ggs,      40246,246,35);  уровень сигнала на выходе GGS
15,			// 17                         // measure_vol_min(analog_gg_radio1,40247,247,35);  уровень сигнала на выходе GG Radio1
15			// 18                         // measure_vol_min(analog_gg_radio2,40248,248,35);  уровень сигнала на выходе GG Radio2
};

const byte  porog_MTT[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++++ Test MTT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
200,		// 0                          // resistor(1, 130);  Установить уровень сигнала 60 мв
200,		// 1                          // resistor(2, 130);  Установить уровень сигнала 60 мв
15,			// 2                          // measure_vol_min(analog_FrontL,    40250,250,35); уровень сигнала на выходе FrontL  
15,			// 3                          // measure_vol_min(analog_FrontR,    40251,251,35); уровень сигнала на выходе FrontR 
15,			// 4                          // measure_vol_min(analog_LineL,     40252,252,35); уровень сигнала на выходе LineL
15,			// 5                          // measure_vol_min(analog_LineR,     40253,253,35); уровень сигнала на выходе LineR
15,			// 6                          // measure_vol_min(analog_mag_radio, 40254,254,35); уровень сигнала на выходе mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone, 40255,255,35); уровень сигнала на выходе mag phone
15,			// 8                          // measure_vol_min(analog_ggs,       40256,256,35); уровень сигнала на выходе GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1, 40257,257,35); уровень сигнала на выходе GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2, 40258,258,35); уровень сигнала на выходе GG Radio2 
			//++++++++++++++++++++++++++++++++++ Сигнал подан на вход МТТ ++++++++++++++++++++++++++++++++++++++++++++
15,			// 11                         // measure_vol_min(analog_FrontL,    40250,250,35); уровень сигнала на выходе FrontL
15,			// 12                         // measure_vol_min(analog_FrontR,    40251,251,35); уровень сигнала на выходе FrontR
15,			// 13                         // measure_vol_min(analog_mag_radio, 40254,254,35); уровень сигнала на выходе mag radio
15,			// 14                         // measure_vol_min(analog_ggs,       40256,256,35); уровень сигнала на выходе GGS
15,			// 15                         // measure_vol_min(analog_gg_radio1, 40257,257,35); уровень сигнала на выходе GG Radio1
15,			// 16                         // measure_vol_min(analog_gg_radio2, 40258,258,35); уровень сигнала на выходе GG Radio2 
70,	    	// 17                         // measure_vol_max(analog_LineL,     40260,260,35);  "Test MTT ** Signal LineL 
55,	        // 18                         // measure_vol_max(analog_mag_phone, 40262,262, + 18)); 
15,         // 19                         // measure_vol_min(analog_ggs,       40256,256,por_buffer[19]);  // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
120			// 20                         // measure_vol_max(analog_ggs,       40259,259,  + 20));         //  Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS             On 
};




const byte  porog_Microphone[]    PROGMEM = {
		//+++++++++++++++++++++++++++++++++ Test Microphone +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
150,		// 0                          // resistor(1, 200);  Установить уровень сигнала 60 мв
150,		// 1                          // resistor(2, 200);  Установить уровень сигнала 60 мв
15,			// 2                          // measure_vol_min(analog_FrontL,    40320,320,35); уровень сигнала на выходе FrontL
15,			// 3                          // measure_vol_min(analog_FrontR,    40321,321,35); уровень сигнала на выходе FrontR 
15,			// 4                          // measure_vol_min(analog_LineL,     40322,322,35); уровень сигнала на выходе LineL
15,			// 5                          // measure_vol_min(analog_LineR,     40323,323,35); уровень сигнала на выходе LineR
15,			// 6                          // measure_vol_min(analog_mag_radio, 40324,324,35); уровень сигнала на выходе mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone, 40325,325,35); уровень сигнала на выходе mag phone
15,			// 8                          // measure_vol_min(analog_ggs,       40326,326,35); уровень сигнала на выходе GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1, 40327,327,35); уровень сигнала на выходе GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2, 40328,328,35); уровень сигнала на выходе GG Radio2
			//++++++++++++++++++++++++++++++++++ Сигнал подан на вход микрофона ++++++++++++++++++++++++
60,	    	// 11                         // measure_vol_max(analog_mag_phone, 40298,298,180);уровень сигнала на выходе mag phone
75,		    // 12                         // measure_vol_max(analog_LineL,     40299,299,180);уровень сигнала на выходе "Test Microphone ** Signal LineL 
15,			// 13                         // measure_vol_min(analog_FrontL,    40320,320,35); уровень сигнала на выходе FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,    40321,321,35); уровень сигнала на выходе FrontR
15,			// 15                         // measure_vol_min(analog_LineR,     40323,323,35); уровень сигнала на выходе LineR
15,			// 16                         // measure_vol_min(analog_mag_radio, 40324,324,35); уровень сигнала на выходе mag radio 
15,			// 17                         // measure_vol_min(analog_ggs,       40326,326,35); уровень сигнала на выходе GGS
15,			// 18                         // measure_vol_min(analog_gg_radio1, 40327,327,35); уровень сигнала на выходе GG Radio1
15          // 19                         // measure_vol_min(analog_gg_radio2, 40328,328,35); уровень сигнала на выходе GG Radio2
};

const byte  porog_GGS[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++ Test GGS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
230,		//  0                         // resistor(1, 200); Установить уровень сигнала xx мв
230,		//  1                         // resistor(2, 200); Установить уровень сигнала xx мв
				//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15, 		//  2                         // measure_vol_min(analog_FrontL,    40280,280,35); уровень сигнала на выходе "Test GGS ** Signal FrontL 
15, 		//  3                         // measure_vol_min(analog_FrontR,    40281,281,35); уровень сигнала на выходе "Test GGS ** Signal FrontR        
15, 		//  4                         // measure_vol_min(analog_LineL,     40282,282,35); уровень сигнала на выходе "Test GGS ** Signal LineL 
15, 		//  5                         // measure_vol_min(analog_LineR,     40283,283,35); уровень сигнала на выходе "Test GGS ** Signal LineR 
15, 		//  6                         // measure_vol_min(analog_mag_radio, 40284,284,35); уровень сигнала на выходе "Test GGS ** Signal mag radio
15, 		//  7                         // measure_vol_min(analog_mag_phone, 40285,285,35); уровень сигнала на выходе "Test GGS ** Signal mag phone 
15, 		//  8                         // measure_vol_min(analog_ggs,       40286,286,35); уровень сигнала на выходе "Test GGS ** Signal GGS    
15, 		//  9                         // measure_vol_min(analog_gg_radio1, 40287,287,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio1   
15, 		//  10                        // measure_vol_min(analog_gg_radio2, 40288,288,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio2
			   //++++++++++++++++++++++++++++++++++ Сигнал подан на вход GGS ++++++++++++++++++++++++
110,    	// 11    					  //measure_vol_max(analog_FrontL,    40290,290,40); уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
130,	   	// 12  						  //measure_vol_max(analog_FrontR,    40291,291,40); уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";
15,			// 13						  //measure_vol_min(analog_LineL,     40282,282,35); уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
15,			// 14						  //measure_vol_min(analog_LineR,     40283,283,35); уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
16,			// 15	                 	  //measure_vol_max(analog_mag_radio, 40332,332,35); уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
16,	    	// 16						  //measure_vol_max(analog_mag_phone, 40292,292,50); уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
130,	    // 17			     		  //measure_vol_max(analog_ggs,       40286,289,160); уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
15,			// 18						  //measure_vol_min(analog_gg_radio1, 40287,287,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
15			// 19						  //measure_vol_min(analog_gg_radio2, 40288,288,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

};

const byte  porog_Radio1[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio1 +++++++++++++++++++++++++++++++++++++++++++++++++++++

150,		// 0							//resistor(1, 250);  Установить уровень сигнала xx мв
150,		// 1							//resistor(2, 250);  Установить уровень сигнала xx мв
				 //+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15,			// 2							//measure_vol_min(analog_FrontL,    40300,300,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
15,			// 3							//measure_vol_min(analog_FrontR,    40301,301,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
15,			// 4							//measure_vol_min(analog_LineL,     40302,302,35); уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
15,			// 5							//measure_vol_min(analog_LineR,     40303,303,35); уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
15,			// 6							//measure_vol_min(analog_mag_radio, 40304,304,35); уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
15,			// 7							//measure_vol_min(analog_mag_phone, 40305,305,35); уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
15,			// 8							//measure_vol_min(analog_ggs,       40306,306,35); уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
15,			// 9							//measure_vol_min(analog_gg_radio1, 40307,307,35); уровень сигнала на выходе "Test Radio1 ** Signal GG Radio1                             OFF - ";
15,			// 10							//measure_vol_min(analog_gg_radio2, 40308,308,35); Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";
				  //++++++++++++++++++++++++++++++++++ Сигнал подан на вход  Radio1 ++++++++++++++++++++++++
15,			// 11							//measure_vol_min(analog_FrontL,    40300,300,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
15,			// 12							//measure_vol_min(analog_FrontR,    40301,301,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
15,			// 13							//measure_vol_min(analog_LineL,     40302,302,35); уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
15,			// 14							//measure_vol_min(analog_LineR,     40303,303,35); уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
40,			// 15							//measure_vol_max(analog_mag_radio, 40330,330,35); уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
15,			// 16							//measure_vol_min(analog_mag_phone, 40305,305,35); уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
15,			// 17							//measure_vol_min(analog_ggs,       40306,306,35); уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
220,		// 18							//measure_vol_max(analog_gg_radio1, 40309,309,250);уровень сигнала на выходе "Test Radio1 ** Signal Radio1                                ON  - ";
15			// 19							//measure_vol_min(analog_gg_radio2, 40308,308,35); уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2       
};

const byte  porog_Radio2[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio2 +++++++++++++++++++++++++++++++++++++++++++++++++++++

150,		// 0							//resistor(1, 250);  Установить уровень сигнала xx мв
150,		// 1							//resistor(2, 250);  Установить уровень сигнала xx мв
				 //+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15,			// 2							//measure_vol_min(analog_FrontL,    40310,310,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
15,			// 3							//measure_vol_min(analog_FrontR,    40311,311,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
15,			// 4							//measure_vol_min(analog_LineL,     40312,312,35); уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
15,			// 5							//measure_vol_min(analog_LineR,     40313,313,35); уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
15,			// 6 							//measure_vol_min(analog_mag_radio, 40314,314,35); уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
15,			// 7							//measure_vol_min(analog_mag_phone, 40315,315,35); уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
15,			// 8							//measure_vol_min(analog_ggs,       40316,316,35); уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
15,			// 9							//measure_vol_min(analog_gg_radio1, 40317,317,35); уровень сигнала на выходе "Test Radio2 ** Signal GG Radio1                             OFF - ";
15,			// 10							//measure_vol_min(analog_gg_radio2, 40318,318,35); уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";
					//++++++++++++++++++++++++++++++++++ Сигнал подан на вход  Radio2 ++++++++++++++++++++++++
15,			// 11							//measure_vol_min(analog_FrontL,    40310,310,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
15,			// 12							//measure_vol_min(analog_FrontR,    40311,311,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
15,			// 13							//measure_vol_min(analog_LineL,     40312,312,35); уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
15,			// 14							//measure_vol_min(analog_LineR,     40313,313,35); уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
40,			// 15							//measure_vol_max(analog_mag_radio, 40331,331,35); уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
15,			// 16							//measure_vol_min(analog_mag_phone, 40315,315,35); уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
15,			// 17							//measure_vol_min(analog_ggs,       40316,316,35); уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
15,			// 18							//measure_vol_min(analog_gg_radio1, 40317,317,35); уровень сигнала на выходе "Test Radio2 ** Signal Radio1                                ON  - ";
200			// 19							//measure_vol_max(analog_gg_radio2, 40319,319,250);уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2        
};





//---------------------------Тексты сообщений   ---------------------------------------------------
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



//++++++++++++++++++++++++++++++ Тексты ошибок ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
txt_message54,                                // " ****** Test miсrophone start! ******"                       ;
txt_message55,                                // "Signal miсrophone 30  mV                      ON"            ;
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














// ========================= Блок программ ============================================
void dateTime(uint16_t* date, uint16_t* time)                  // Программа записи времени и даты файла
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void serial_print_date()                           // Печать даты и времени    
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
//	DateTime set_time = DateTime(year, month, day, hour, minute, second); // Занести данные о времени в строку "set_time"
//	RTC.adjust(set_time);             
}

void flash_time()                                              // Программа обработчик прерывания 
{ 
		//prer_Kmerton_Run = true;
		////	digitalWrite(ledPin13,HIGH);
		//prer_Kamerton();
		//// mb.task();
	    //	digitalWrite(ledPin13,LOW);
		//prer_Kmerton_Run = false;
}
void prer_Kamerton()                                          // Произвести обмен информации с модулем Камертон
{
//	clear_serial1();
	sendPacketK ();  
	// Отправить информацию в модуль Камертон
	waiting_for_replyK();                                  // Получить подтверждение
}
void sendPacketK () 
{              // Программа передачи пакета в Камертон
	calculateCRC_Out();
	for (int i = 0; i <3; i++)
		{
			Serial1.write(regs_out[i]);
			mb.addHreg(1+i,regs_out[i]);
		}
}
void waiting_for_replyK()                                  // Чтение данных из Камертона
{
//	delayMicroseconds(5);
	//blink_red = !blink_red;
	//digitalWrite(ledPin13,!digitalRead(ledPin13));
	//  Уточнить задержку и применение Stop_Kam = 0; 
	if (Serial1.available())                               // есть что-то проверить? Есть данные в буфере?
		  {
			unsigned char overflowFlag = 0 ;               // Флаг превышения размера буфера
			unsigned char buffer = 0;                      // Установить в начало чтения буфера

			while (Serial1.available())
				{
				  if (overflowFlag)                        // Если буфер переполнен - очистить
					 Serial1.read();
				  else                                     // Размер буфера в норме, считать информацию
					{
					if (bufferK == BUFFER_SIZEK)           // Проверить размер буфера
						{
							overflowFlag = 1;              // Установить флаг превышения размера буфера
						}
						 mb.addHreg(4+buffer,Serial1.read());
						buffer++;
					}
				}
//			calculateCRC_In();
			mb.Coil(124,1);                               // Связь с "Камертон" установлена
		   }
	 else 
		{
			Stop_Kam = 0;                                 // Флаг отсутств. инф. из Камертона
			mb.Coil(124,0);                               // Флаг ошибки  связи с "Камертон"
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
void Stop_Kamerton ()                  //Если не приходит информация с Камертона - регистры обнулить
  {
	 for (unsigned char i = 0; i <4; i++)
	 mb.Hreg(4+i,0);
  }

void calculateCRC_Out()                // Вычисление контрольной суммы ниблов байта
{ 
  byte temp1, temp2, temp3, temp4, crc;
  temp1 = regs_out[1];                 // записать  
  temp1 = temp1&0xF0;                  // Наложить маску F0 на старший нибл 1 байта
  temp2 = temp1>>4;                    // Переместить старший нибл в младший
  temp3 = regs_out[2];                 // записать
  temp3 = temp3&0xF0;                  // Наложить маску F0 на старший нибл 2 байта
  temp3 = temp3>>4;                    // Переместить старший нибл в младший
  temp4 = regs_out[2];                 // записать
  temp4 = temp4&0x0F;                  // Наложить маску F0 на младший нибл 2 байта
  crc =  temp2 ^  temp3 ^  temp4  ;
  crc = crc&0x0F;                      // Наложить маску F0 на младший нибл 2 байта
  regs_out[1]= temp1 | crc;
}
void calculateCRC_In()                 // Вычисление контрольной суммы ниблов байта
{ 
	/*
  byte temp1,temp1H,temp1L, temp2,temp2H,temp2L, temp3,temp3H,temp3L, temp4, temp4H, crc_in;

  temp1 = regs_in[0];                  // записать  
  temp1 = temp1&0xF0;                  // Наложить маску F0 на старший нибл 1 байта
  temp1H = temp1>>4;                   // Переместить старший нибл в младший
  temp1 = regs_in[0];                  // записать 
  temp1L = temp1&0x0F;                 // Наложить маску 0F на младший нибл 1 байта

  temp2 = regs_in[1];                  // записать  
  temp2 = temp2&0xF0;                  // Наложить маску F0 на старший нибл 2 байта
  temp2H = temp2>>4;                   // Переместить старший нибл в младший
  temp2 = regs_in[1];                  // записать 
  temp2L = temp2&0x0F;                 // Наложить маску 0F на младший нибл 2 байта

  temp3 = regs_in[2];                  // записать  
  temp3 = temp3&0xF0;                  // Наложить маску F0 на старший нибл 3 байта
  temp3H = temp3>>4;                   // Переместить старший нибл в младший
  temp3 = regs_in[2];                  // записать 
  temp3L = temp3&0x0F;                 // Наложить маску 0F на младший нибл 3 байта

  temp4 = regs_in[3];                  // записать  
  temp4 = temp4&0xF0;                  // Наложить маску F0 на старший нибл 3 байта
  temp4H = temp4>>4;                   // Переместить старший нибл в младший
  crc_in =   temp1H ^  temp1L  ^   temp2H ^  temp2L  ^  temp3H ^  temp3L  ^  temp4H ;
  crc_in =  crc_in&0x0F;               // Наложить маску F0 на младший нибл 4 байта
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
void UpdateRegs()                                        // Обновить регистры
{
	//-----Первый байт ------------
	//-----Установить бит 0
	// while(prer_Kmerton_Run == true){}                  // Ждем окончания получения данных из Камертон
	 boolean set_rele ;
	 prer_Kmerton_On = false;                            // Запретить прерывание Камертон ??
	// reg_Kamerton();                                     // Записать данные из Камертон в    регистры 
		// Подпрограмма переноса данных из регистров на порты вывода
	  //-----Установить бит 0
	 set_rele = mb.Coil(1);
	 mcp_Out1.digitalWrite(0, set_rele);                 // Реле RL0 Звук  Звук Mic1p Диспетчер

	 //-----Установить бит 1
	  set_rele = mb.Coil(2);
	  mcp_Out1.digitalWrite(1, set_rele);               // Реле RL1 Звук Mic2p  Инструкор

	 //-----Установить бит 2
	  set_rele = mb.Coil(3);
	  mcp_Out1.digitalWrite(2, set_rele);               // Реле RL2 Звук Mic3p MTT
  
	 //-----Установить бит 3
	  set_rele = mb.Coil(4);
	  mcp_Out1.digitalWrite(3, set_rele);               // Реле RL3 Звук

	 //-----Установить бит 4                            // Реле RL4 XP1 12
	  set_rele = mb.Coil(5);
	  mcp_Out1.digitalWrite(4, set_rele);    

	 //-----Установить бит 5
	  set_rele = mb.Coil(6);                        // Реле RL5 Звук
	  mcp_Out1.digitalWrite(5, set_rele);              

	 //-----Установить бит 6	 
	  set_rele = mb.Coil(7);
	  mcp_Out1.digitalWrite(6, set_rele);              // Реле RL6 Звук

	 //-----Установить бит 7
	  set_rele = mb.Coil(8);
	  mcp_Out1.digitalWrite(7, set_rele);              // Реле RL7 Питание платы

	 //---- Второй байт----------
	 //-----Установить бит 8
	  set_rele = mb.Coil(9);                        // Реле RL8 Звук на микрофон
	  mcp_Out1.digitalWrite(8, set_rele);    

	 //-----Установить бит 9
	  set_rele = mb.Coil(10);
	  mcp_Out1.digitalWrite(9, set_rele);               // Реле RL9 XP1 10

	 //-----Установить бит 10                           // Реле RL10 Включение питания на высоковольтный модуль 
	  set_rele = mb.Coil(11);
	  mcp_Out1.digitalWrite(10, set_rele);    


	//-----Установить бит 11                            // Свободен 
	  set_rele = mb.Coil(12);
	  mcp_Out1.digitalWrite(11, set_rele);    

	 //-----Установить бит 12
	  set_rele = mb.Coil(13);
	  mcp_Out1.digitalWrite(12, set_rele);              // XP8 - 2   sensor Тангента ножная

	 //-----Установить бит 13
	  set_rele = mb.Coil(14);
	  mcp_Out1.digitalWrite(13, set_rele);              // XP8 - 1   PTT Тангента ножная

	 //-----Установить бит 14

	  set_rele = mb.Coil(15);
	  mcp_Out1.digitalWrite(14, set_rele);              // XS1 - 5   PTT Мик

	  //-----Установить бит 15
	  set_rele = mb.Coil(16);
	  mcp_Out1.digitalWrite(15, set_rele);              // XS1 - 6   sensor Мик

	  //  Test 3
	 //-----Первый байт ------------
	 //-----Установить бит 0

	  set_rele = mb.Coil(17);
	  mcp_Out2.digitalWrite(0, set_rele);                // J8-12     XP7 4 PTT2   Танг. р.

	 //-----Установить бит 1
	  set_rele = mb.Coil(18);
	  mcp_Out2.digitalWrite(1, set_rele);                // XP1 - 20  HangUp  DCD

	 //-----Установить бит 2
	  set_rele = mb.Coil(19);
	  mcp_Out2.digitalWrite(2, set_rele);                // J8-11     XP7 2 sensor  Танг. р.
  
	//-----Установить бит 3

	  set_rele = mb.Coil(20);
	  mcp_Out2.digitalWrite(3, set_rele);                 // J8-23     XP7 1 PTT1 Танг. р.

	 //-----Установить бит 4
	  set_rele = mb.Coil(21);
	  mcp_Out2.digitalWrite(4, set_rele);                 // XP2-2     sensor "Маг." 

	 //-----Установить бит 5

	  set_rele = mb.Coil(22);
	  mcp_Out2.digitalWrite(5, set_rele);                  // XP5-3     sensor "ГГC."

	 //-----Установить бит 6
	  set_rele = mb.Coil(23);
	  mcp_Out2.digitalWrite(6, set_rele);                  // XP3-3     sensor "ГГ-Радио1."

	 //-----Установить бит 7
	  set_rele = mb.Coil(24);
	  mcp_Out2.digitalWrite(7, set_rele);                  // XP4-3     sensor "ГГ-Радио2."

	  // Test 4
	//-----Первый байт ------------
	 //-----Установить бит 8
	  set_rele = mb.Coil(25);
	  mcp_Out2.digitalWrite(8, set_rele);                  // XP1- 19 HaSs      флаг подключения трубки  

	  //-----Установить бит 9
	  set_rele = mb.Coil(26);
	  mcp_Out2.digitalWrite(9, set_rele);                  // XP1- 17 HaSPTT    CTS DSR вкл.

	  //-----Установить бит 10
	  set_rele = mb.Coil(27);
	  mcp_Out2.digitalWrite(10, set_rele);                 // XP1- 16 HeS2Rs    флаг подключения гарнитуры инструктора с 2 наушниками

	  //-----Установить бит 11
	  set_rele = mb.Coil(28);
	  mcp_Out2.digitalWrite(11, set_rele);                 // XP1- 15 HeS2PTT   CTS вкл

	  //-----Установить бит 12
	  set_rele = mb.Coil(29);
	  mcp_Out2.digitalWrite(12, set_rele);                 // XP1- 13 HeS2Ls    флаг подключения гарнитуры инструктора 

	  //-----Установить бит 13
	  set_rele = mb.Coil(30);
	  mcp_Out2.digitalWrite(13, set_rele);                 // XP1- 6  HeS1PTT   CTS вкл

	  //-----Установить бит 14
	  set_rele = mb.Coil(31);
	  mcp_Out2.digitalWrite(14, set_rele);                 // XP1- 5  HeS1Rs    Флаг подкючения гарнитуры диспетчера с 2 наушниками

	  //-----Установить бит 15
	  set_rele = mb.Coil(32);
	  mcp_Out2.digitalWrite(15, set_rele);                 // XP1- 1  HeS1Ls    Флаг подкючения гарнитуры диспетчера

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
	int mwt = mb.Hreg(60);             // Адрес хранения величины сигнала
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
			//Wire.requestFrom(address_AD5252, 1, true);  // Считать состояние движка резистора 
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
	mb.Hreg(121,0);                                                             // Сбросить счетчик ошибок
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
		DateTime set_time = DateTime(year, month, day, hour, minute, second);   // Занести данные о времени в строку "set_time"
		RTC.adjust(set_time);                                                   // Записать время в контроллер часов  
		mb.Hreg(adr_set_time, 0);                                               // Записать в регистр признак окончания выполнения команды
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
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		  regBank.set(adr_reg_temp_day,day_temp);  
		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon);                             //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		  regBank.set(adr_reg_temp_mon,mon_temp); 
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year);                            //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		 regBank.set(adr_reg_temp_year,year_temp); 

		  b = i2c_eeprom_read_byte(0x50, adr_file_name_count);                             //access an address from the memory
		  regBank.set(adr_reg_file_name,b);                                                // Регистр  хранения переменной номер файла
		  */
}
void time_control() // Программа записи текущего времени в регистры для передачи в ПК
{
	DateTime now = RTC.now();
	mb.Hreg(adr_kontrol_day  , now.day());
	mb.Hreg(adr_kontrol_month, now.month());
	mb.Hreg(adr_kontrol_year, now.year());
	mb.Hreg(adr_kontrol_hour, now.hour());
	mb.Hreg(adr_kontrol_minute, now.minute());
	mb.Hreg(adr_kontrol_second, now.second());
}
void time_control_get()   // Тестовая программа проверки содержания регистров времени
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
void preob_num_str() // Программа формирования имени файла, состоящего из текущей даты и счетчика файлов
{
	DateTime now = RTC.now();
	day   = now.day();
	month = now.month();
	year  = now.year();
	int year_temp = year-2000;
	itoa (year_temp,str_year_file, 10);                                        // Преобразование даты год в строку ( 10 - десятичный формат) 

	if (month <10)
		{
		   itoa (0,str_mon_file0, 10);                                         //  Преобразование даты месяц  в строку ( 10 - десятичный формат) 
		   itoa (month,str_mon_file10, 10);                                    //  Преобразование числа в строку ( 10 - десятичный формат) 
		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);       // Сложение 2 строк
		}
	else
		{
		   itoa (month,str_mon_file, 10);                                      // Преобразование числа в строку ( 10 - десятичный формат) 
		}
	if (day <10)
		{
		   itoa (0,str_day_file0, 10);                                         // Преобразование числа в строку ( 10 - десятичный формат) 
		   itoa (day,str_day_file10, 10);                                      // Преобразование числа в строку ( 10 - десятичный формат) 
		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);       // Сложение 2 строк
		}
	else
		{
		itoa (day,str_day_file, 10);                                           // Преобразование числа в строку ( 10 - десятичный формат) 
		}
		 
	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // Сложение 2 строк
	sprintf(str2, "%s%s",str1, str_day_file);                                  // Сложение 2 строк
	sprintf(fileName, "%s%s", str2, "00.KAM");                                 // Получение имени файла в file_name
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
	Для вызова подпрограммы проверки необходимо записать номер проверки по адресу adr_control_command (40120) 
	Код проверки
	0 -  Выполнение команды окончено
	1 -  Отключить все сенсоры
	2 -  Включить все сенсоры
	3 -  Тест Инструктора
	4 -  Тест диспетчера
	5 -  Тест МТТ
	6 -  Тест Танг.р
	7 -  Тест Микрофон
	8 -  Тест ГГС
	9 -  Тест Радио 1
	10 - Тест Радио 2
	11 - Тест Магнитофон
	12 - Открыть файл
	13 - Закрыть файл
	14 - Записать время
	15 - Установить уровень сигнала
	16 - Reg_count_clear();			                                        // Сброс счетчиков ошибок                    
	17 - test_power();                                                    	// Проверить напряжение  питания
	18 - set_video();				 //
	19 - test_video();				 //
	20 - Записать уровни порогов заводские
	21 - Записать уровни порогов пользовательские
	22 - Получить уровни порогов пользовательские
	23 - Контроль имени файла
	*/
	UpdateRegs() ;

	int test_n = mb.Hreg(adr_control_command);                                  //адрес  40120
	if (test_n != 0)
	{
//	Serial.println(test_n);	
	switch (test_n)
	{
		case 1:
			// sensor_all_off();                                                      // Отключить все сенсоры
			 break;
		case 2:		
			// sensor_all_on();                                                       // Включить все сенсоры
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
			// test_mikrophon();                                                      // Тестирование микрофона
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
			//	Reg_count_clear();			                                        // Сброс счетчиков ошибок                    
				break;
		case 17:
			//	test_power();                                                    	// Проверить напряжение  питания
				break;
		case 18:
			//	set_video();				              //
				break;
		case 19:
			//	test_video();				              //
				break;
		case 20:                                           // Записать уровни порогов заводские
			//	default_mem_porog();
				break;
		case 21:                                           // 	21 - Записать уровни порогов пользовательские
			//	set_mem_porog();
				break;
		case 22:                                           // 22 - Получить уровни порогов пользовательские
			//	read_mem_porog();
				break;
		case 23:   
			//	controlFileName();                         // Контроль имени файла
				break;
		case 24:   
			//	 set_SD();                                 // Проверка SD памяти
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
	// Настройка расширителя портов

 
  mcp_Out1.begin(4);              //  Адрес (4) второго  расширителя портов
  mcp_Out1.pinMode(0, OUTPUT);    // Реле №0  Звук    
  mcp_Out1.pinMode(1, OUTPUT);    // Реле №1  Звук    
  mcp_Out1.pinMode(2, OUTPUT);    // Реле №2  Звук    
  mcp_Out1.pinMode(3, OUTPUT);    // Реле №3  Звук    
  mcp_Out1.pinMode(4, OUTPUT);    // Реле №4  Звук   XP1 10-12
  mcp_Out1.pinMode(5, OUTPUT);    // Реле №5  Звук    
  mcp_Out1.pinMode(6, OUTPUT);    // Реле №6  Звук   
  mcp_Out1.pinMode(7, OUTPUT);    // Реле №7  включить +12вольт  Питание платы Камертон
  
  mcp_Out1.pinMode(8, OUTPUT);    //  Реле №8 Звук на микрофон дифф.  
  mcp_Out1.pinMode(9, OUTPUT);    // Свободен J24 - 3    
  mcp_Out1.pinMode(10, OUTPUT);   // Свободен J24 - 2    
  mcp_Out1.pinMode(11, OUTPUT);   // Свободен J24 - 1   
  mcp_Out1.pinMode(12, OUTPUT);   // XP8 - 2  sensor     
  mcp_Out1.pinMode(13, OUTPUT);   // XP8 - 1  PTT       
  mcp_Out1.pinMode(14, OUTPUT);   // XS1 - 5   PTT      
  mcp_Out1.pinMode(15, OUTPUT);   // XS1 - 6 sensor      

	
  mcp_Out2.begin(6);              //  Адрес (6) второго  расширителя портов
  mcp_Out2.pinMode(0, OUTPUT);    // J8-12    XP7 4 PTT2    
  mcp_Out2.pinMode(1, OUTPUT);    // XP1 - 20  HandUp    
  mcp_Out2.pinMode(2, OUTPUT);    // J8-11    XP7 2 sensor
  mcp_Out2.pinMode(3, OUTPUT);    // J8-23    XP7 1 PTT1    
  mcp_Out2.pinMode(4, OUTPUT);    // XP2-2    sensor "Маг."    
  mcp_Out2.pinMode(5, OUTPUT);    // XP5-3    sensor "ГГC." 
  mcp_Out2.pinMode(6, OUTPUT);    // XP3-3    sensor "ГГ-Радио1."
  mcp_Out2.pinMode(7, OUTPUT);    // XP4-3    sensor "ГГ-Радио2."
  
  mcp_Out2.pinMode(8, OUTPUT);    // XP1- 19 HaSs
  mcp_Out2.pinMode(9, OUTPUT);    // XP1- 17 HaSPTT
  mcp_Out2.pinMode(10, OUTPUT);   // XP1- 16 HeS2Rs
  mcp_Out2.pinMode(11, OUTPUT);   // XP1- 15 HeS2PTT
  mcp_Out2.pinMode(12, OUTPUT);   // XP1- 13 HeS2Ls           
  mcp_Out2.pinMode(13, OUTPUT);   // XP1- 6  HeS1PTT            
  mcp_Out2.pinMode(14, OUTPUT);   // XP1- 5  HeS1Rs            
  mcp_Out2.pinMode(15, OUTPUT);   // XP1- 1  HeS1Ls          

 
  mcp_Analog.begin(5);            //  Адрес (5)  расширителя портов 
  mcp_Analog.pinMode(8, OUTPUT);  // DTR_D
  mcp_Analog.pinMode(9, OUTPUT);  // RTS_D
  mcp_Analog.pinMode(10, OUTPUT); // J15-2 Свободен
  mcp_Analog.pinMode(11, OUTPUT); // J15-3 Свободен
  mcp_Analog.pinMode(12, OUTPUT); // J15-4 Свободен
  mcp_Analog.pinMode(13, OUTPUT); // J15-5
  mcp_Analog.pinMode(14, OUTPUT); // J15-6
  mcp_Analog.pinMode(15, OUTPUT); // J15-7 
  
  mcp_Analog.pinMode(0, INPUT);   //  J22-1 Свободен
  mcp_Analog.pullUp(0, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(1, INPUT);   // J22-2 Свободен  Расширитель портов на ввод
  mcp_Analog.pullUp(1, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(2, INPUT);   // J22-3 Свободен Свободен Расширитель портов на ввод 
  mcp_Analog.pullUp(2, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(3, INPUT);   // J22-4 Свободен Расширитель портов на ввод
  mcp_Analog.pullUp(3, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(4, INPUT);   // J22-5 Свободен  Расширитель портов на ввод
  mcp_Analog.pullUp(4, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(5, INPUT);   //CTS Расширитель портов на ввод
  mcp_Analog.pullUp(5, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(6, INPUT);   // DSR Расширитель портов на ввод
  mcp_Analog.pullUp(6, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(7, INPUT);   //  DCD Расширитель портов на ввод
  mcp_Analog.pullUp(7, HIGH);     // Подключить внутренний резистор 100K к 5В.

}
void setup_regModbus()
{
	// Set the Slave ID (1-247)
	mb.setSlaveId(1);  
   // Подключение к протоколу MODBUS компьютера Serial3    

	mb.addCoil(1);                           // Реле RL0 Звук  MIC1P
	mb.addCoil(2);                           // Реле RL1 Звук  MIC2P
	mb.addCoil(3);                           // Реле RL2 Звук  MIC3P
	mb.addCoil(4);                           // Реле RL3 Звук  LFE  "Маг."
	mb.addCoil(5);                           // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
	mb.addCoil(6);                           // Реле RL5 Звук Front L, Front R
	mb.addCoil(7);                           // Реле RL6 Звук Center
	mb.addCoil(8);                           // Реле RL7 Питание платы
  
	mb.addCoil(9);                           // Реле RL8 Звук на микрофон
	mb.addCoil(10);                          // Реле RL9 XP1 10 Включение микрофона диспетчера
	mb.addCoil(11);                          // Реле RL10 Включение питания на высоковольтный модуль 
	mb.addCoil(12);                          // Свободен J24 - 1 
	mb.addCoil(13);                          // XP8 - 2   sensor Тангента ножная
	mb.addCoil(14);                          // XP8 - 1   PTT Тангента ножная
	mb.addCoil(15);                          // XS1 - 5   PTT Мик
	mb.addCoil(16);                          // XS1 - 6   sensor Мик
 
	mb.addCoil(17);                          // J8-12     XP7 4 PTT2   Танг. р.
	mb.addCoil(18);                          // XP1 - 20  HangUp  DCD
	mb.addCoil(19);                          // J8-11     XP7 2 sensor  Танг. р.
	mb.addCoil(20);                          // J8-23     XP7 1 PTT1 Танг. р.
	mb.addCoil(21);                          // XP2-2     sensor "Маг."  
	mb.addCoil(22);                          // XP5-3     sensor "ГГC."
	mb.addCoil(23);                          // XP3-3     sensor "ГГ-Радио1."
	mb.addCoil(24);                          // XP4-3     sensor "ГГ-Радио2."
 
	mb.addCoil(25);                          // XP1- 19 HaSs      sensor подключения трубки    MTT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	mb.addCoil(26);                          // XP1- 17 HaSPTT    CTS DSR вкл.  
	mb.addCoil(27);                          // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.addCoil(28);                          // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	mb.addCoil(29);                          // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	mb.addCoil(30);                          // XP1- 6  HeS1PTT   CTS вкл   РТТ Диспетчера
	mb.addCoil(31);                          // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.addCoil(32);                          // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера


	mb.addCoil(118);                         // Флаг индикации многоразовой проверки
	mb.addCoil(119);                         // 

	mb.addCoil(120);                         // Флаг индикации возникновения любой ошибки
	mb.addCoil(122);                         // Флаг индикации открытия файла
	mb.addCoil(123);                         // Флаг индикации закрытия файла
	mb.addCoil(124);                         // Флаг индикации связи с модулем "Камертон"
	mb.addCoil(125);                         // Флаг индикации инициализации SD памяти
	mb.addCoil(126);                         //  
	mb.addCoil(127);                         //  
	mb.addCoil(128);                         //  
	mb.addCoil(129);                         //  

	mb.addCoil(130);                         //  Флаг индикации порта 0 - RS232, 1 - USB0
	mb.addCoil(131);                         //  


	mb.addCoil(200);                         // Флаг ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	mb.addCoil(201);                         // Флаг ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	mb.addCoil(202);                         // Флаг ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	mb.addCoil(203);                         // Флаг ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	mb.addCoil(204);                         // Флаг ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	mb.addCoil(205);                         // Флаг ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	mb.addCoil(206);                         // Флаг ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	mb.addCoil(207);                         // Флаг ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
	mb.addCoil(208);                         // Флаг ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	mb.addCoil(209);                         // Флаг ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	mb.addCoil(210);                         // Флаг ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	mb.addCoil(211);                         // Флаг ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	mb.addCoil(212);                         // Флаг ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	mb.addCoil(213);                         // Флаг ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	mb.addCoil(214);                         // Флаг ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	mb.addCoil(215);                         // Флаг ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	mb.addCoil(216);                         // Флаг ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	mb.addCoil(217);                         // Флаг ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
	mb.addCoil(218);                         // Флаг ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	mb.addCoil(219);                         // Флаг ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
	 
	mb.addCoil(220);                         // Флаг ошибки "Command PTT headset instructor (CTS)                        OFF - ";
	mb.addCoil(221);                         // Флаг ошибки "Command PTT headset instructor (CTS)                        ON  - ";
	mb.addCoil(222);                         // Флаг ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
	mb.addCoil(223);                         // Флаг ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
	mb.addCoil(224);                         // Флаг ошибки "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addCoil(225);                         // Флаг ошибки "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addCoil(226);                         // Флаг ошибки "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addCoil(227);                         // Флаг ошибки "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addCoil(228);                         // Флаг ошибки "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addCoil(229);                         // Флаг ошибки "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addCoil(230);                         // Флаг ошибки "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addCoil(231);                         // Флаг ошибки "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addCoil(232);                         // Флаг ошибки "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addCoil(233);                         // Флаг ошибки "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addCoil(234);                         // Флаг ошибки "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addCoil(235);                         // Флаг ошибки "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addCoil(236);                         // Флаг ошибки "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addCoil(237);                         // Флаг ошибки "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addCoil(238);                         // Флаг ошибки "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addCoil(239);                         // Флаг ошибки  ADC0  ток x1 

	mb.addCoil(240);                         // Флаг ошибки "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addCoil(241);                         // Флаг ошибки "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addCoil(242);                         // Флаг ошибки "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addCoil(243);                         // Флаг ошибки "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addCoil(244);                         // Флаг ошибки "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addCoil(245);                         // Флаг ошибки "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addCoil(246);                         // Флаг ошибки "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addCoil(247);                         // Флаг ошибки "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addCoil(248);                         // Флаг ошибки "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addCoil(249);                         // Флаг ошибки ADC2 ток x10  

	mb.addCoil(250);                         // Флаг ошибки "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addCoil(251);                         // Флаг ошибки "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addCoil(252);                         // Флаг ошибки "Test MTT ** Signal LineL                                    OFF - ";
	mb.addCoil(253);                         // Флаг ошибки "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addCoil(254);                         // Флаг ошибки "Test MTT ** Signal mag radio                                OFF - ";
	mb.addCoil(255);                         // Флаг ошибки "Test MTT ** Signal mag phone                                OFF - ";
	mb.addCoil(256);                         // Флаг ошибки "Test MTT ** Signal GGS                                      OFF - ";
	mb.addCoil(257);                         // Флаг ошибки "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addCoil(258);                         // Флаг ошибки "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addCoil(259);                         // Флаг ошибки "Test MTT ** Signal GGS                                      ON  - ";

	mb.addCoil(260);                         // Флаг ошибки "Test MTT ** Signal LineL                                    ON  - ";
	mb.addCoil(261);                         // Флаг ошибки "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addCoil(262);                         // Флаг ошибки "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addCoil(263);                         // Флаг ошибки "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addCoil(264);                         // Флаг ошибки "Test microphone PTT  (CTS)                                  OFF - ";
	mb.addCoil(265);                         // Флаг ошибки "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addCoil(266);                         // Флаг ошибки "Test microphone PTT  (CTS)                                  ON  - ";
	mb.addCoil(267);                         // Флаг ошибки "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addCoil(268);                         // Флаг ошибки "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addCoil(269);                         // Флаг ошибки Длительность регулировки яркости 

	mb.addCoil(270);                         // Флаг ошибки "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addCoil(271);                         // Флаг ошибки "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addCoil(272);                         // Флаг ошибки "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addCoil(273);                         // Флаг ошибки "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addCoil(274);                         // Флаг ошибки "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addCoil(275);                         // Флаг ошибки "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addCoil(276);                         // Флаг ошибки "Command sensor tangenta nognaja                             OFF - ";
	mb.addCoil(277);                         // Флаг ошибки "Command sensor tangenta nognaja                             ON  - ";
	mb.addCoil(278);                         // Флаг ошибки "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addCoil(279);                         // Флаг ошибки "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addCoil(280);                         // Флаг ошибки "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addCoil(281);                         // Флаг ошибки "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addCoil(282);                         // Флаг ошибки "Test GGS ** Signal LineL                                    OFF - ";
	mb.addCoil(283);                         // Флаг ошибки "Test GGS ** Signal LineR                                    OFF - ";
	mb.addCoil(284);                         // Флаг ошибки "Test GGS ** Signal mag radio                                OFF - ";
	mb.addCoil(285);                         // Флаг ошибки "Test GGS ** Signal mag phone                                OFF - ";
	mb.addCoil(286);                         // Флаг ошибки "Test GGS ** Signal GGS                                      OFF - ";
	mb.addCoil(287);                         // Флаг ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addCoil(288);                         // Флаг ошибки "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addCoil(289);                         // Флаг ошибки "Test GGS ** Signal GGS                                      ON  - ";

	mb.addCoil(290);                         // Флаг ошибки "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addCoil(291);                         // Флаг ошибки "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addCoil(292);                         // Флаг ошибки "Test GGS ** Signal mag phone                                ON  - ";
	mb.addCoil(293);                         // Флаг ошибки ADC1 напряжение 12/3 вольт
	mb.addCoil(294);                         // Флаг ошибки ADC14 напряжение 12/3 вольт Radio1
	mb.addCoil(295);                         // Флаг ошибки ADC14 напряжение 12/3 вольт Radio2
	mb.addCoil(296);                         // Флаг ошибки ADC14 напряжение 12/3 вольт ГГС
	mb.addCoil(297);                         // Флаг ошибки ADC15 напряжение светодиода 3,6 вольта
	mb.addCoil(298);                         // Флаг ошибки "Test Microphone ** Signal mag phone                         ON  - ";      
	mb.addCoil(299);                         // Флаг ошибки "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addCoil(300);                         // Флаг ошибки "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addCoil(301);                         // Флаг ошибки "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addCoil(302);                         // Флаг ошибки "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addCoil(303);                         // Флаг ошибки "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addCoil(304);                         // Флаг ошибки "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addCoil(305);                         // Флаг ошибки "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addCoil(306);                         // Флаг ошибки "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addCoil(307);                         // Флаг ошибки "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addCoil(308);                         // Флаг ошибки "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addCoil(309);                         // Флаг ошибки "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addCoil(310);                         // Флаг ошибки "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addCoil(311);                         // Флаг ошибки "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addCoil(312);                         // Флаг ошибки "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addCoil(313);                         // Флаг ошибки "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addCoil(314);                         // Флаг ошибки "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addCoil(315);                         // Флаг ошибки "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addCoil(316);                         // Флаг ошибки "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addCoil(317);                         // Флаг ошибки "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addCoil(318);                         // Флаг ошибки "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addCoil(319);                         // Флаг ошибки "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addCoil(320);                         // Флаг ошибки "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addCoil(321);                         // Флаг ошибки "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addCoil(322);                         // Флаг ошибки "Test Microphone ** Signal LineL                             OFF - ";
	mb.addCoil(323);                         // Флаг ошибки "Test Microphone ** Signal LineR                             OFF - ";
	mb.addCoil(324);                         // Флаг ошибки "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addCoil(325);                         // Флаг ошибки "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addCoil(326);                         // Флаг ошибки "Test Microphone ** Signal GGS                               OFF - ";
	mb.addCoil(327);                         // Флаг ошибки "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addCoil(328);                         // Флаг ошибки "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addCoil(329);                         // Флаг ошибки Код яркости дисплея

	mb.addCoil(330);                         // Флаг ошибки "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addCoil(331);                         // Флаг ошибки "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	mb.addCoil(332);                         // Флаг ошибки "Test GGS ** Signal mag radio   


	mb.addIsts(81);    // Адрес флагa индикации состояния сигнала CTS
	mb.addIsts(82);    // Адрес флагa индикации состояния сигнала DSR
	mb.addIsts(83);    // Адрес флагa индикации состояния сигнала DCD

						 //Add Input registers 30001-30040 to the register bank


	//regBank.set(40004+buffer,Serial1.read());

	//mb.addHreg(40000);  // 
	mb.addHreg(1,1);  // Регистры обмена с Аудио 1
	mb.addHreg(2,2);  // Регистры обмена с Аудио 1
	mb.addHreg(3,3);  // Регистры обмена с Аудио 1
	mb.addHreg(4,4);  // Регистры обмена с Аудио 1
	mb.addHreg(5,5);  // Регистры обмена с Аудио 1
	mb.addHreg(6,6);  // Регистры обмена с Аудио 1
	mb.addHreg(7,7);  // Регистры обмена с Аудио 1
	mb.addHreg(8,8);  // 
	mb.addHreg(9,9);  // 




	mb.addHreg(10);  // №  Аудио 1
	mb.addHreg(11);  // №  Аудио 1
	mb.addHreg(12);  // №  Аудио 1
	mb.addHreg(13);  // №  Аудио 1
	mb.addHreg(14);  // 
	mb.addHreg(15);  // 
	mb.addHreg(16);  // 
	mb.addHreg(17);  // 
	mb.addHreg(18);  // 
	mb.addHreg(19);  // 

						 // Текущее время 
	mb.addHreg(46,1);  // адрес день модуля часов контроллера
	mb.addHreg(47,2);  // адрес месяц модуля часов контроллера
	mb.addHreg(48,3);  // адрес год модуля часов контроллера
	mb.addHreg(49,4);  // адрес час модуля часов контроллера
	mb.addHreg(50,5);  // адрес минута модуля часов контроллера
	mb.addHreg(51,6);  // адрес секунда модуля часов контроллера
							 // Текущее время 
	mb.addIreg(1,6);  // адрес день модуля часов контроллера
	mb.addIreg(2,5);  // адрес месяц модуля часов контроллера
	mb.addIreg(3,4);  // адрес год модуля часов контроллера
	mb.addIreg(4,3);  // адрес час модуля часов контроллера
	mb.addIreg(5,2);  // адрес минута модуля часов контроллера
	mb.addIreg(6,1);  // адрес секунда модуля часов контроллера
	mb.addIreg(7,2);  // адрес минута модуля часов контроллера
	mb.addIreg(8,1);  // адрес секунда модуля часов контроллера


//  mb.addIreg(SENSOR_IREG);
						 // Установка времени в контроллере
	mb.addHreg(52);  // адрес день
	mb.addHreg(53);  // адрес месяц
	mb.addHreg(54);  // адрес год
	mb.addHreg(55);  // адрес час
	mb.addHreg(56);  // адрес минута
	mb.addHreg(57);  // 
	mb.addHreg(58);  // 
	mb.addHreg(59);  // 
	
	mb.addHreg(60);  // Адрес хранения величины сигнала резисторами
	mb.addHreg(61);  // Адрес хранения величины яркости для управления
	mb.addHreg(62);  // Адрес хранения величины яркости для передачи в программу
	mb.addHreg(63);  // Адрес хранения длительности импульса яркости для передачи в программу ПК


	/*
	mb.addHreg(40061); // адрес счетчика ошибки
	mb.addHreg(40062); // адрес счетчика ошибки
	mb.addHreg(40063); // адрес счетчика ошибки
	mb.addHreg(40064); // адрес ошибки
	mb.addHreg(40065); // адрес ошибки
	mb.addHreg(40066); // адрес ошибки
	mb.addHreg(40067); // адрес ошибки
	mb.addHreg(40068); // адрес ошибки
	mb.addHreg(40069); // адрес ошибки
	mb.addHreg(40070); // адрес ошибки
	mb.addHreg(40071); // адрес ошибки

	mb.addHreg(40072); // адрес ошибки в %
	mb.addHreg(40073); // адрес ошибки в %
	mb.addHreg(40074); // адрес ошибки в %
	mb.addHreg(40075); // адрес ошибки %
	mb.addHreg(40076); // адрес ошибки %
	mb.addHreg(40077); // адрес ошибки %
	mb.addHreg(40078); // адрес ошибки %
	mb.addHreg(40079); // адрес ошибки %
	mb.addHreg(40080); // адрес ошибки %
	mb.addHreg(40081); // адрес ошибки %
	mb.addHreg(40082); // адрес ошибки %
	mb.addHreg(40083); // адрес ошибки %

	// Время ошибки на включение
	mb.addHreg(40084); // адрес день adr_Mic_On_day 
	mb.addHreg(40085); // адрес месяц adr_Mic_On_month  
	mb.addHreg(40086); // адрес год adr_Mic_On_year  
	mb.addHreg(40087); // адрес час adr_Mic_On_hour 
	mb.addHreg(40088); // адрес минута adr_Mic_On_minute 
	mb.addHreg(40089); // адрес секунда  adr_Mic_On_second    

	// Время ошибки на выключение
	mb.addHreg(40090); // адрес день adr_Mic_Off_day    
	mb.addHreg(40091); // адрес месяц  adr_Mic_Off_month 
	mb.addHreg(40092); // адрес год adr_Mic_Off_year  
	mb.addHreg(40093); // адрес час adr_Mic_Off_hour   
	mb.addHreg(40094); // адрес минута adr_Mic_Off_minute   
	mb.addHreg(40095); // адрес секунда adr_Mic_Off_second    
	*/
	// Время старта теста
	mb.addHreg(96);  // адрес день  adr_Mic_Start_day    
	mb.addHreg(97);  // адрес месяц adr_Mic_Start_month  
	mb.addHreg(98);  // адрес год adr_Mic_Start_year  
	mb.addHreg(99);  // адрес час adr_Mic_Start_hour 
	mb.addHreg(100);  // адрес минута adr_Mic_Start_minute 
	mb.addHreg(101);  // адрес секунда adr_Mic_Start_second  

	// Время окончания теста
	mb.addHreg(102);  // адрес день adr_Mic_Stop_day 
	mb.addHreg(103);  // адрес месяц adr_Mic_Stop_month 
	mb.addHreg(104);  // адрес год adr_Mic_Stop_year
	mb.addHreg(105);  // адрес час adr_Mic_Stop_hour 
	mb.addHreg(106);  // адрес минута adr_Mic_Stop_minute  
	mb.addHreg(107);  // адрес секунда adr_Mic_Stop_second 

	// Продолжительность выполнения теста
	mb.addHreg(108);  // адрес день adr_Time_Test_day 
	mb.addHreg(109);  // адрес час adr_Time_Test_hour 
	mb.addHreg(110);  // адрес минута adr_Time_Test_minute
	mb.addHreg(111);  // адрес секунда adr_Time_Test_second

	mb.addHreg(112);  // Адрес хранения переменной год 
	mb.addHreg(113);  // Адрес хранения переменной месяц 
	mb.addHreg(114);  // Адрес хранения переменной день
	mb.addHreg(115);  // Адрес хранения переменной счетчика последнего номера файла
	mb.addHreg(116);  // Адрес хранения переменной счетчика текущего номера файла

	mb.addHreg(120);  // adr_control_command Адрес передачи комманд на выполнение
	mb.addHreg(121);  // Адрес счетчика всех ошибок
	mb.addHreg(122);  //
	mb.addHreg(123);  //
	mb.addHreg(124);  //
	mb.addHreg(125);  //  
	mb.addHreg(126);  //  
	mb.addHreg(127);  //  Адрес блока регистров для передачи в ПК уровней порогов.
	mb.addHreg(128);  //  Адрес блока памяти для передачи в ПК уровней порогов.
	mb.addHreg(129);  //  Адрес длины блока памяти для передачи в ПК уровней порогов.

	mb.addHreg(130);  //  Регистры временного хранения для передачи уровней порогов 
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


	mb.addHreg(200);                         // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	mb.addHreg(201);                         // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	mb.addHreg(202);                         // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	mb.addHreg(203);                         // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	mb.addHreg(204);                         // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	mb.addHreg(205);                         // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	mb.addHreg(206);                         // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	mb.addHreg(207);                         // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
	mb.addHreg(208);                         // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	mb.addHreg(209);                         // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	mb.addHreg(210);                         // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	mb.addHreg(211);                         // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	mb.addHreg(212);                         // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	mb.addHreg(213);                         // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	mb.addHreg(214);                         // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	mb.addHreg(215);                         // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	mb.addHreg(216);                         // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	mb.addHreg(217);                         // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
	mb.addHreg(218);                         // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	mb.addHreg(219);                         // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

	mb.addHreg(220);                         // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        OFF - ";
	mb.addHreg(221);                         // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        ON  - ";
	mb.addHreg(222);                         // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
	mb.addHreg(223);                         // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
	mb.addHreg(224);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addHreg(225);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addHreg(226);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addHreg(227);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addHreg(228);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addHreg(229);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addHreg(230);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addHreg(231);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addHreg(232);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addHreg(233);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addHreg(234);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addHreg(235);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addHreg(236);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addHreg(237);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addHreg(238);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addHreg(239);                         // Aдрес счетчика ошибки ADC0  ток x1 

	mb.addHreg(240);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addHreg(241);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addHreg(242);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addHreg(243);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addHreg(244);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addHreg(245);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addHreg(246);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addHreg(247);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addHreg(248);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addHreg(249);                         // Aдрес счетчика ошибки ADC2 ток x10

	mb.addHreg(250);                         // Aдрес счетчика ошибки "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addHreg(251);                         // Aдрес счетчика ошибки "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addHreg(252);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineL                                    OFF - ";
	mb.addHreg(253);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addHreg(254);                         // Aдрес счетчика ошибки "Test MTT ** Signal mag radio                                OFF - ";
	mb.addHreg(255);                         // Aдрес счетчика ошибки "Test MTT ** Signal mag phone                                OFF - ";
	mb.addHreg(256);                         // Aдрес счетчика ошибки "Test MTT ** Signal GGS                                      OFF - ";
	mb.addHreg(257);                         // Aдрес счетчика ошибки "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addHreg(258);                         // Aдрес счетчика ошибки "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addHreg(259);                         // Aдрес счетчика ошибки "Test MTT ** Signal GGS                                      ON  - ";

	mb.addHreg(260);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineL                                    ON  - ";
	mb.addHreg(261);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addHreg(262);                         // Aдрес счетчика ошибки "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addHreg(263);                         // Aдрес счетчика ошибки "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addHreg(264);                         // Aдрес счетчика ошибки "Test microphone PTT  (CTS)                                  OFF - ";
	mb.addHreg(265);                         // Aдрес счетчика ошибки "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addHreg(266);                         // Aдрес счетчика ошибки "Test microphone PTT  (CTS)                                  ON  - ";
	mb.addHreg(267);                         // Aдрес счетчика ошибки "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addHreg(268);                         // Aдрес счетчика ошибки "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addHreg(269);                         // Aдрес счетчика ошибки Длительность регулировки яркости

	mb.addHreg(270);                         // Aдрес счетчика ошибки "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addHreg(271);                         // Aдрес счетчика ошибки "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addHreg(272);                         // Aдрес счетчика ошибки "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addHreg(273);                         // Aдрес счетчика ошибки "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addHreg(274);                         // Aдрес счетчика ошибки "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addHreg(275);                         // Aдрес счетчика ошибки "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addHreg(276);                         // Aдрес счетчика ошибки "Command sensor tangenta nognaja                             OFF - ";
	mb.addHreg(277);                         // Aдрес счетчика ошибки "Command sensor tangenta nognaja                             ON  - ";
	mb.addHreg(278);                         // Aдрес счетчика ошибки "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addHreg(279);                         // Aдрес счетчика ошибки "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addHreg(280);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addHreg(281);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addHreg(282);                         // Aдрес счетчика ошибки "Test GGS ** Signal LineL                                    OFF - ";
	mb.addHreg(283);                         // Aдрес счетчика ошибки "Test GGS ** Signal LineR                                    OFF - ";
	mb.addHreg(284);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag radio                                OFF - ";
	mb.addHreg(285);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag phone                                OFF - ";
	mb.addHreg(286);                         // Aдрес счетчика ошибки "Test GGS ** Signal GGS                                      OFF - ";
	mb.addHreg(287);                         // Aдрес счетчика ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addHreg(288);                         // Aдрес счетчика ошибки "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addHreg(289);                         // Aдрес счетчика ошибки "Test GGS ** Signal GGS                                      ON  - ";

	mb.addHreg(290);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addHreg(291);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addHreg(292);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag phone                                ON  - ";
	mb.addHreg(293);                         // Aдрес счетчика  ошибки ADC1 напряжение 12/3 вольт
	mb.addHreg(294);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio1
	mb.addHreg(295);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio2
	mb.addHreg(296);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт ГГС
	mb.addHreg(297);                         // Aдрес счетчика  ошибки ADC15 напряжение светодиода 3,6 вольта
	mb.addHreg(298);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag phone                         ON  - ";    
	mb.addHreg(299);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addHreg(300);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addHreg(301);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addHreg(302);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addHreg(303);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addHreg(304);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addHreg(305);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addHreg(306);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addHreg(307);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(308);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(309);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addHreg(310);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addHreg(311);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addHreg(312);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addHreg(313);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addHreg(314);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addHreg(315);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addHreg(316);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addHreg(317);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(318);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(319);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addHreg(320);                         // Aдрес счетчика ошибки "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addHreg(321);                         // Aдрес счетчика ошибки "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addHreg(322);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineL                             OFF - ";
	mb.addHreg(323);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineR                             OFF - ";
	mb.addHreg(324);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addHreg(325);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addHreg(326);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GGS                               OFF - ";
	mb.addHreg(327);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addHreg(328);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addHreg(329);                         // Aдрес счетчика ошибки Код регулировки яркости                             // 

	mb.addHreg(330);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addHreg(331);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag radio                             ON  - ";
	mb.addHreg(332);                         // Aдрес счетчика ошибки "Test GGS    ** Signal mag radio                             ON  - ";


	
	// ++++++++++++++++++++++ Регистры хранения данных при проверке модулей ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	mb.addHreg(400);                         // Aдрес напряжение ADC0  ток x1 
	mb.addHreg(401);                         // Aдрес напряжение ADC1 напряжение 12/3 вольт
	mb.addHreg(402);                         // Aдрес напряжение ADC2 ток x10
	mb.addHreg(403);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт Radio1
	mb.addHreg(404);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт Radio2
	mb.addHreg(405);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт ГГС
	mb.addHreg(406);                         // Aдрес напряжение ADC15 напряжение светодиода 3,6 вольта
	mb.addHreg(407);                         // Aдрес 
	mb.addHreg(408);                         // Aдрес 
	mb.addHreg(409);                         // Aдрес  

	mb.addHreg(410);                         // Aдрес счетчика 
	mb.addHreg(411);                         // Aдрес счетчика  
	mb.addHreg(412);                         // Aдрес счетчика  
	mb.addHreg(413);                         // Aдрес счетчика  
	mb.addHreg(414);                         // Aдрес счетчика  
	mb.addHreg(415);                         // Aдрес счетчика  
	mb.addHreg(416);                         // Aдрес счетчика  
	mb.addHreg(417);                         // Aдрес счетчика  
	mb.addHreg(418);                         // Aдрес счетчика  
	mb.addHreg(419);                         // Aдрес счетчика  

	mb.addHreg(420);                         // Aдрес  ;
	mb.addHreg(421);                         // Aдрес  ;
	mb.addHreg(422);                         // Aдрес  ;
	mb.addHreg(423);                         // Aдрес  ;
	mb.addHreg(424);                         // Aдрес данных измерения "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addHreg(425);                         // Aдрес данных измерения "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addHreg(426);                         // Aдрес данных измерения "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addHreg(427);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addHreg(428);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addHreg(429);                         // Aдрес данных измерения "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addHreg(430);                         // Aдрес данных измерения "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addHreg(431);                         // Aдрес данных измерения "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addHreg(432);                         // Aдрес данных измерения "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addHreg(433);                         // Aдрес данных измерения "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addHreg(434);                         // Aдрес данных измерения "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addHreg(435);                         // Aдрес данных измерения "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addHreg(436);                         // Aдрес данных измерения "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addHreg(437);                         // Aдрес данных измерения "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addHreg(438);                         // Aдрес данных измерения "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addHreg(439);                         // Aдрес данных измерения ADC0  ток x1 

	mb.addHreg(440);                         // Aдрес данных измерения "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addHreg(441);                         // Aдрес данных измерения "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addHreg(442);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addHreg(443);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addHreg(444);                         // Aдрес данных измерения "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addHreg(445);                         // Aдрес данных измерения "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addHreg(446);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addHreg(447);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addHreg(448);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addHreg(449);                         // Aдрес данных измерения ADC2 ток x10

	mb.addHreg(450);                         // Aдрес данных измерения "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addHreg(451);                         // Aдрес данных измерения "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addHreg(452);                         // Aдрес данных измерения "Test MTT ** Signal LineL                                    OFF - ";
	mb.addHreg(453);                         // Aдрес данных измерения "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addHreg(454);                         // Aдрес данных измерения "Test MTT ** Signal mag radio                                OFF - ";
	mb.addHreg(455);                         // Aдрес данных измерения "Test MTT ** Signal mag phone                                OFF - ";
	mb.addHreg(456);                         // Aдрес данных измерения "Test MTT ** Signal GGS                                      OFF - ";
	mb.addHreg(457);                         // Aдрес данных измерения "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addHreg(458);                         // Aдрес данных измерения "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addHreg(459);                         // Aдрес данных измерения "Test MTT ** Signal GGS                                      ON  - ";

	mb.addHreg(460);                         // Aдрес данных измерения "Test MTT ** Signal LineL                                    ON  - ";
	mb.addHreg(461);                         // Aдрес данных измерения "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addHreg(462);                         // Aдрес данных измерения "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addHreg(463);                         // Aдрес данных измерения "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addHreg(464);                         // 
	mb.addHreg(465);                         // Aдрес данных измерения "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addHreg(466);                         // 
	mb.addHreg(467);                         // Aдрес данных измерения "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addHreg(468);                         // Aдрес данных измерения "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addHreg(469);                         // Длительность импульса регулировки яркости дисплея

	mb.addHreg(470);                         // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addHreg(471);                         // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addHreg(472);                         // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addHreg(473);                         // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addHreg(474);                         // Aдрес данных измерения "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addHreg(475);                         // Aдрес данных измерения "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addHreg(476);                         // Aдрес данных измерения "Command sensor tangenta nognaja                             OFF - ";
	mb.addHreg(477);                         // Aдрес данных измерения "Command sensor tangenta nognaja                             ON  - ";
	mb.addHreg(478);                         // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addHreg(479);                         // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addHreg(480);                         // Aдрес данных измерения "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addHreg(481);                         // Aдрес данных измерения "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addHreg(482);                         // Aдрес данных измерения "Test GGS ** Signal LineL                                    OFF - ";
	mb.addHreg(483);                         // Aдрес данных измерения "Test GGS ** Signal LineR                                    OFF - ";
	mb.addHreg(484);                         // Aдрес данных измерения "Test GGS ** Signal mag radio                                OFF - ";
	mb.addHreg(485);                         // Aдрес данных измерения "Test GGS ** Signal mag phone                                OFF - ";
	mb.addHreg(486);                         // Aдрес данных измерения "Test GGS ** Signal GGS                                      OFF - ";
	mb.addHreg(487);                         // Aдрес данных измерения "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addHreg(488);                         // Aдрес данных измерения "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addHreg(489);                         // Aдрес данных измерения "Test GGS ** Signal GGS                                      ON  - ";

	mb.addHreg(490);                         // Aдрес данных измерения "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addHreg(491);                         // Aдрес данных измерения "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addHreg(492);                         // Aдрес данных измерения "Test GGS ** Signal mag phone                                ON  - ";
	mb.addHreg(493);                         // Aдрес данных измерения ADC1 напряжение 12/3 вольт
	mb.addHreg(494);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio1
	mb.addHreg(495);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio2
	mb.addHreg(496);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт ГГС
	mb.addHreg(497);                         // Aдрес данных измерения ADC15 напряжение светодиода 3,6 вольта
	mb.addHreg(498);                         // Aдрес данных измерения "Test Microphone ** Signal mag phone                         ON  - "; 
	mb.addHreg(499);                         // Aдрес данных измерения "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addHreg(500);                         // Aдрес данных измерения "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addHreg(501);                         // Aдрес данных измерения "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addHreg(502);                         // Aдрес данных измерения "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addHreg(503);                         // Aдрес данных измерения "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addHreg(504);                         // Aдрес данных измерения "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addHreg(505);                         // Aдрес данных измерения "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addHreg(506);                         // Aдрес данных измерения "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addHreg(507);                         // Aдрес данных измерения "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(508);                         // Aдрес данных измерения "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(509);                         // Aдрес данных измерения "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addHreg(510);                         // Aдрес данных измерения "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addHreg(511);                         // Aдрес данных измерения "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addHreg(512);                         // Aдрес данных измерения "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addHreg(513);                         // Aдрес данных измерения "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addHreg(514);                         // Aдрес данных измерения "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addHreg(515);                         // Aдрес данных измерения "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addHreg(516);                         // Aдрес данных измерения "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addHreg(517);                         // Aдрес данных измерения "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(518);                         // Aдрес данных измерения "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(519);                         // Aдрес данных измерения "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addHreg(520);                         // Aдрес данных измерения "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addHreg(521);                         // Aдрес данных измерения "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addHreg(522);                         // Aдрес данных измерения "Test Microphone ** Signal LineL                             OFF - ";
	mb.addHreg(523);                         // Aдрес данных измерения "Test Microphone ** Signal LineR                             OFF - ";
	mb.addHreg(524);                         // Aдрес данных измерения "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addHreg(525);                         // Aдрес данных измерения "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addHreg(526);                         // Aдрес данных измерения "Test Microphone ** Signal GGS                               OFF - ";
	mb.addHreg(527);                         // Aдрес данных измерения "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addHreg(528);                         // Aдрес данных измерения "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addHreg(529);                         // Код регулировки яркости дисплея
	mb.addHreg(530);                         // Aдрес данных измерения "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addHreg(531);                         // Aдрес данных измерения "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	mb.addHreg(532);                         // Aдрес данных измерения "Test GGS ** Signal mag radio   

//
//	
//	//regBank.set(40004+buffer,Serial1.read());
//
	mb.Coil(21,0);                              // XP2-2     sensor "Маг."  
	mb.Coil(22,0);                              // XP5-3     sensor "ГГC."
	mb.Coil(23,0);                              // XP3-3     sensor "ГГ-Радио1."
	mb.Coil(24,0);                              // XP4-3     sensor "ГГ-Радио2."

//	UpdateRegs();                                   // Обновить информацию в регистрах

	for (int i = 120; i <= 131; i++)                  // Очистить флаги ошибок
	{
	   mb.Coil(i,0);   
	}

	for (int i = 200; i <= 330; i++)                  // Очистить флаги ошибок
	{
	  mb.Coil(i,0);   
	}
	
	for (unsigned int i = 200; i <= 330; i++)         // Очистить флаги ошибок
	{
	   mb.Hreg(i,0);   
	}
		for (unsigned int i = 400; i <= 530; i++)     // Очистить флаги ошибок
	{
	   mb.Hreg(i,0);   
	} 

	mb.Hreg(120,0);                                  // 
	mb.Hreg(adr_reg_count_err,0);                    // Обнулить данные счетчика всех ошибок
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
	Serial.begin(9600);                                        // Подключение к USB ПК
	Serial1.begin(115200);                                     // Подключение к звуковому модулю Камертон
	Serial2.begin(38400);                                      // 
	mb.config(&Serial3, 19200, SERIAL_8N1);                   // Config Modbus Serial (port, speed, byte format) 

	Serial.println(" ");
	Serial.println(" ***** Start system  *****");
	Serial.println(" ");

	setup_regModbus();                                        // Настройка регистров MODBUS

	Wire.begin();
	if (!RTC.begin())                                         // Настройка часов 
		{
			Serial.println("RTC failed");
			while(1);
		};
	// DateTime set_time = DateTime(15, 6, 15, 10, 51, 0);    // Занести данные о времени в строку "set_time"
	// RTC.adjust(set_time);                                  // Записа
	serial_print_date();
	Serial.println(" ");
	MsTimer2::set(30, flash_time);                            // 30ms период таймера прерывани
	setup_mcp();
	mcp_Analog.digitalWrite(DTR, HIGH);                       // Разрешение вывода (обмена)информации с Камертоном
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, HIGH); 

	pinMode(ledPin13, OUTPUT);  
	pinMode(ledPin12, OUTPUT);  
	pinMode(ledPin11, OUTPUT);  
	pinMode(ledPin10, OUTPUT);  
	pinMode(kn1Nano, OUTPUT);                        // Назначение кнопок управления Nano генератор качения
	pinMode(kn2Nano, OUTPUT);                        // Назначение кнопок управления Nano генератор 1000 гц
	pinMode(kn3Nano, OUTPUT);                        // Назначение кнопок управления Nano генератор 2000 гц
	pinMode(InNano12, INPUT);                        // Назначение входов - индикация генератор 1000 или 2000 гц
	pinMode(InNano13, INPUT);                        // Назначение входов - индикация генератор качения 
 
	digitalWrite(kn1Nano, LOW);
	digitalWrite(kn2Nano, HIGH);
	digitalWrite(kn3Nano, HIGH);

	//********* Настройка звукового модуля **************************
	AD9850.reset();                                  //reset module
	delay(500);
	AD9850.powerDown();                              //set signal output to LOW
	delay(100);
	AD9850.set_frequency(0,0,1000);                  //set power=UP, phase=0, 1kHz frequency
	delay(1000); 
	//-------------------------------------------------------------------
	
	setup_resistor();                               // Начальные установки резистора

	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 196 Изменять в реальной схеме
	regs_out[2]= 0x7F;                              // 127 Изменять в реальной схеме

	#if FASTADC                                     // Ускорить считывание аналогового канала
	// set prescale to 16
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
	#endif
	Serial.println("Initializing SD card...");
	pinMode(49, OUTPUT);//    заменить 
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

	SdFile::dateTimeCallback(dateTime);             // Настройка времени записи файла
	// Serial.println("Files found on the card (name, date and size in bytes): ");
	// list all files in the card with date and size
	// sd.ls (LS_R | LS_DATE | LS_SIZE);
	preob_num_str();                                 // Записать начальное имя файла 
	list_file();                                     // Вывод списка файлов в СОМ порт  






	//MsTimer2::start();                                        // Включить таймер прерывания
	
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	Serial.println(" ");                                      //
	Serial.println(" ***** System initialization OK!. ****"); // Информация о завершении настройки

	//wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
}

void loop()
{
	mb.task();
 

}
