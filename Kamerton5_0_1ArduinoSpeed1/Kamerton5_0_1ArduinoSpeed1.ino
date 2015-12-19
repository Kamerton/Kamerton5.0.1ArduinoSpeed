/*

 Kamerton5_0_1ArduinoSpeed1.ino
 VisualStudio
 
 Программа тестирования модуля "Камертон" (Базовый вариант)
 Версия:      - 5.0.1
 Дата:        - 19.12.2015г.
 Организация: - ООО "Децима"
 Автор:       - Мосейчук А.В.
 Версия: Обновленная версия от  19.12.2015г.. Новая плата с учетом добавления 
 высоковольтного  модуля для испытания на пробой.
 Реализовано: Новый протокол MODBUS
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


#define  ledPin13  13                               // Назначение светодиодов на плате
#define  ledPin12  12                               // Назначение светодиодов на плате
#define  ledPin11  11                               // Назначение светодиодов на плате
#define  ledPin10  10                               // Назначение светодиодов на плате
#define  Front_led_Blue 14                          // Назначение светодиодов на передней панели
#define  Front_led_Red  15                          // Назначение светодиодов на передней панели

//+++++++++++++++++++++++++++++ Внешняя память +++++++++++++++++++++++++++++++++++++++
int deviceaddress        = 80;                      // Адрес микросхемы памяти
unsigned int eeaddress   =  0;                      // Адрес ячейки памяти
byte hi;                                            // Старший байт для преобразования числа
byte low;                                           // Младший байт для преобразования числа


//  Порты управления платой Камертон
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




//+++++++++++++++++++++++ Настройка электронного резистора +++++++++++++++++++++++++++++++++++++
#define address_AD5252   0x2F                       // Адрес микросхемы AD5252  
#define control_word1    0x07                       // Байт инструкции резистор №1
#define control_word2    0x87                       // Байт инструкции резистор №2
byte resistance        = 0x00;                      // Сопротивление 0x00..0xFF - 0Ом..100кОм
//byte level_resist      = 0;                       // Байт считанных данных величины резистора
//-----------------------------------------------------------------------------------------------
unsigned int volume1     = 0;                       //
unsigned int volume_max  = 0;                       //
unsigned int volume_min  = 0;                       //
unsigned int volume_fact = 0;                       //
unsigned int Array_volume[260];                     //
unsigned int Array_min[40];                         //
unsigned int Array_max[40];                         //
unsigned int volume_porog_D = 40;                   // Максимальная величина порога при проверке исправности FrontL,FrontR
unsigned int volume_porog_L = 200;                  // Минимальная величина порога при проверке исправности FrontL,FrontR
float voltage ;
//float voltage_test = 0.60;                        // порог величины синусоиды звука
unsigned int  voltage10 ;
unsigned long number_audio = 0;   


#define FASTADC 1                                   // Ускорение считывания аналогового сигнала
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//***************************** Назначение аналоговых входов   ****************************************
int analog_tok            = 0;       //   Измерение тока питания платы Камертон
int analog_12V            = 1;       //   Измерение напряжения питания 12в. платы Камертон
int analog_tok_x10        = 2;       //   Измерение тока питания платы Камертон х 10
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
int analog_13             = 13;       // Измерение напряжения питания  12в.на разъемах  платы Камертон
int analog_14             = 14;       // Измерение напряжения питания  12в.на разъемах  платы Камертон
int analog_3_6            = 15;       // Измерение напряжения питания 3,6в. на разъемах платы Камертон

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
MCP23017 mcp_Out1;                                  // Назначение портов расширения MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                  // Назначение портов расширения MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                // Назначение портов расширения MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------
const int adr_reg_ind_CTS      PROGMEM           = 81;        // Адрес флагa индикации состояния сигнала CTS
const int adr_reg_ind_DSR      PROGMEM           = 82;        // Адрес флагa индикации состояния сигнала DSR
const int adr_reg_ind_DCD      PROGMEM           = 83;        // Адрес флагa индикации состояния сигнала DCD

// **************** Адреса внешней памяти для хранения даты. Применяется приформировании имени файла *************
//const int adr_temp_day         PROGMEM           = 240;          // Адрес хранения переменной день
//const int adr_temp_mon         PROGMEM           = 241;          // Адрес хранения переменной месяц
//const int adr_temp_year        PROGMEM           = 242;          // Адрес хранения переменной год  
//const int adr_file_name_count  PROGMEM           = 243;          // Адрес хранения переменной счетчика номера файла
//------------------------------------------------------------------------------------------------------------------
int regcount_err        = 0;                                     // Переменная для хранения всех ошибок
	//	hi=highByte(n_str_electro);
	//	low=lowByte(n_str_electro);

//++++++++++++++++++++++ Работа с файлами +++++++++++++++++++++++++++++++++++++++
//#define chipSelect SS
#define chipSelect 49   // Основной
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


// созданы переменные, использующие функции библиотеки SD utility library functions: +++++++++++++++
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;


//++++++++++++++++++++ Назначение имени файла ++++++++++++++++++++++++++++++++++++++++++++
//const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "150101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13]            = FILE_BASE_NAME "00.TXT";
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

//+++++++++++++++++++++ Установки прерывания +++++++++++++++++++++++++++++++

unsigned int sampleCount1 = 0;

//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ModbusSerial mb;

//byte regs_in[5];                                    // Регистры работы с платой Камертон CPLL
byte regs_out[4];                                   // Регистры работы с платой Камертон
byte regs_crc[1];                                   // Регистры работы с платой Камертон контрольная сумма
byte regs_temp = 0;
byte regs_temp1 = 0;
byte Stop_Kam = 0;                                  // Флаг индикации чтения инф. из Камертона
bool prer_Kmerton_On = true;                        // Флаг разрешение прерывания Камертон
bool test_repeat     = true;                        // Флаг повторения теста
volatile bool prer_Kmerton_Run = false;             // Флаг разрешение прерывания Камертон
#define BUFFER_SIZEK 64                             // Размер буфера Камертон не более 128 байт
#define BUFFER_SIZEKF 128                           // Размер буфера Serial2 не более 128 байт
unsigned char bufferK;                              // Счетчик количества принимаемых байт

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
const unsigned int adr_Mic_Start_day      PROGMEM      = 96; // адрес день
const unsigned int adr_Mic_Start_month    PROGMEM      = 97; // адрес месяц
const unsigned int adr_Mic_Start_year     PROGMEM      = 98; // адрес год
const unsigned int adr_Mic_Start_hour     PROGMEM      = 99; // адрес час
const unsigned int adr_Mic_Start_minute   PROGMEM      = 100; // адрес минута
const unsigned int adr_Mic_Start_second   PROGMEM      = 101; // адрес секунда
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

const unsigned int adr_reg_temp_year      PROGMEM       = 112; // Регистр хранения переменной год  
const unsigned int adr_reg_temp_mon       PROGMEM       = 113; // Регистр хранения переменной месяц
const unsigned int adr_reg_temp_day       PROGMEM       = 114; // Регистр хранения переменной день 
const unsigned int adr_reg_file_name      PROGMEM       = 115; // Регистр хранения счетчик файлов  
const unsigned int adr_reg_file_tek       PROGMEM       = 116; // Регистр хранения счетчик файлов  

const unsigned int adr_control_command    PROGMEM       = 120; // Адрес передачи комманд на выполнение 
const unsigned int adr_reg_count_err      PROGMEM       = 121; // Адрес счетчика всех ошибок

const unsigned int adr_set_time           PROGMEM       = 36;    // адрес флаг установки

//------------------------- Уровни пороговых значений сигналов при тестировании устройств--------------------------------------
//++++++++++++++++++++++++++++ Заводские установки уровней порогов +++++++++++++++++++++++++++++++++++++

// Адреса внешней памяти для хранения порогов уровней измерения сигналов

const  int adr_porog_instruktor            = 0;     // 19 адресов 
const  int adr_porog_dispatcher            = 20;    // 19 адресов 
const  int adr_porog_MTT                   = 40;    // 21 адресов 
const  int adr_porog_GGS                   = 170;    // 29 адресов 
const  int adr_porog_Radio1                = 100;   // 20 адресов 
const  int adr_porog_Radio2                = 120;   // 20 адресов 
const  int adr_porog_Microphone            = 140;   //  
// end                                     = 160

//Новые пороги две ячейки, 2 байта
const  int adr_int_porog_instruktor            = 200;     // 19 адресов 
const  int adr_int_porog_dispatcher            = 250;    // 19 адресов 
const  int adr_int_porog_MTT                   = 300;    // 21 адресов 
const  int adr_int_porog_GGS                   = 350;    // 29 адресов 
const  int adr_int_porog_Radio1                = 420;   // 20 адресов 
const  int adr_int_porog_Radio2                = 460;   // 20 адресов 
const  int adr_int_porog_Microphone            = 500;   //
//end                                            550;


byte por_buffer[30] ;

//const byte porog_instruktor[]    PROGMEM  = {30,30,35,35,35,35,35,35,35,35,35,150,150,35,35,35,35,35,35,35,254};
const byte porog_instruktor[]    PROGMEM  = {
		 //++++++++++++++++  Test headset instructor ++++++++++++++++++++++++++++
100,			// 0                          // resistor(1, 30);     Установить уровень сигнала 30 мв
100,			// 1                          // resistor(2, 30);     Установить уровень сигнала 30 мв
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
160,    	// 11                         // measure_vol_max(analog_LineL,    40224,224,150); уровень сигнала на выходе LineL
160,		// 12                         // measure_vol_max(analog_mag_phone,40226,226,150); уровень сигнала на выходе mag phone 
15, 		// 13                         // measure_vol_min(analog_FrontL,   40230,230,35);  уровень сигнала на выходе FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,   40231,231,35);  уровень сигнала на выходе FrontR 
15,			// 15                         // measure_vol_min(analog_LineR,    40233,233,35);  уровень сигнала на выходе LineR 
15,			// 16                         // measure_vol_min(analog_ggs,      40236,236,35);  уровень сигнала на выходе GGS 
15,			// 17                         // measure_vol_min(analog_gg_radio1,40237,237,35);  уровень сигнала на выходе GG Radio1
15  		// 18                         // measure_vol_min(analog_gg_radio2,40238,238,35);  уровень сигнала на выходе GG Radio2
};
  
const byte  porog_dispatcher[]    PROGMEM = {

		//	+++++++++++++Test headset dispatcher ++++++++++++++++++++++++++++++
100,			// 0                          // resistor(1, 30);   Установить уровень сигнала 30 мв
100,			// 1                          // resistor(2, 30);   Установить уровень сигнала 30 мв
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
160,		// 11                         // measure_vol_max(analog_LineL,    40227,227,200); уровень сигнала на выходе LineL
160,		// 12                         // measure_vol_max(analog_mag_phone,40229,229,200); уровень сигнала на выходе mag phone
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
150,		// 17                         // measure_vol_max(analog_LineL,    40260,260,35);  "Test MTT ** Signal LineL 
150,		// 18                         // measure_vol_max(analog_LineR,    40261,261,35);  "Test MTT ** Signal LineR 
15,         // 19                         // measure_vol_min(analog_ggs,       40256,256,por_buffer[19]);                    // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
120			// 20                         // measure_vol_max(analog_mag_phone,40262,262,90);  уровень сигнала на выходе mag phone
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
150,		// 11                         // measure_vol_max(analog_mag_phone, 40298,298,180);уровень сигнала на выходе mag phone
150,		// 12                         // measure_vol_max(analog_LineL,     40299,299,180);уровень сигнала на выходе "Test Microphone ** Signal LineL 
15,			// 13                         // measure_vol_min(analog_FrontL,    40320,320,35); уровень сигнала на выходе FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,    40321,321,35); уровень сигнала на выходе FrontR
15,			// 15                         // measure_vol_min(analog_LineR,     40323,323,35); уровень сигнала на выходе LineR
15,			// 16                         // measure_vol_min(analog_mag_radio, 40324,324,35); уровень сигнала на выходе mag radio 
15,			// 17                         // measure_vol_min(analog_ggs,       40326,326,35); уровень сигнала на выходе GGS
15,			// 18                         // measure_vol_min(analog_gg_radio1, 40327,327,35); уровень сигнала на выходе GG Radio1
15,         // 19                         // measure_vol_min(analog_gg_radio2, 40328,328,35); уровень сигнала на выходе GG Radio2
};

const byte  porog_GGS[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++ Test GGS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
200,		//  0                         // resistor(1, 200); Установить уровень сигнала xx мв
200,		//  1                         // resistor(2, 200); Установить уровень сигнала xx мв
				//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
35, 		//  2                         // measure_vol_min(analog_FrontL,    40280,280,35); уровень сигнала на выходе "Test GGS ** Signal FrontL 
35, 		//  3                         // measure_vol_min(analog_FrontR,    40281,281,35); уровень сигнала на выходе "Test GGS ** Signal FrontR        
35, 		//  4                         // measure_vol_min(analog_LineL,     40282,282,35); уровень сигнала на выходе "Test GGS ** Signal LineL 
35, 		//  5                         // measure_vol_min(analog_LineR,     40283,283,35); уровень сигнала на выходе "Test GGS ** Signal LineR 
35, 		//  6                         // measure_vol_min(analog_mag_radio, 40284,284,35); уровень сигнала на выходе "Test GGS ** Signal mag radio
35, 		//  7                         // measure_vol_min(analog_mag_phone, 40285,285,35); уровень сигнала на выходе "Test GGS ** Signal mag phone 
35, 		//  8                         // measure_vol_min(analog_ggs,       40286,286,35); уровень сигнала на выходе "Test GGS ** Signal GGS    
35, 		//  9                         // measure_vol_min(analog_gg_radio1, 40287,287,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio1   
35, 		//  10                        // measure_vol_min(analog_gg_radio2, 40288,288,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio2
			   //++++++++++++++++++++++++++++++++++ Сигнал подан на вход GGS ++++++++++++++++++++++++
40,	    	// 11    					  //measure_vol_max(analog_FrontL,    40290,290,40); уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
40,	    	// 12  						  //measure_vol_max(analog_FrontR,    40291,291,40); уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";
35,			// 13						  //measure_vol_min(analog_LineL,     40282,282,35); уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
35,			// 14						  //measure_vol_min(analog_LineR,     40283,283,35); уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
50,	    	// 15						  //measure_vol_max(analog_mag_phone, 40292,292,50); уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
160,		// 16			     		  //measure_vol_max(analog_ggs,       40286,289,160); уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
35,			// 17						  //measure_vol_min(analog_gg_radio1, 40287,287,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
35,			// 18						  //measure_vol_min(analog_gg_radio2, 40288,288,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

40,			// 19						  //measure_vol_max(analog_FrontL,    40290,290,40); уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
40,			// 20 						  //measure_vol_max(analog_FrontR,    40291,291,40); уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";
35,			// 21					      //measure_vol_min(analog_LineL,     40282,282,35); уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
35,			// 22						  //measure_vol_min(analog_LineR,     40283,283,35); уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
35,			// 23						  //measure_vol_min(analog_mag_radio, 40284,284,35); уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
50,			// 24						  //measure_vol_max(analog_mag_phone, 40292,292,50); уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
35,			// 25						  //measure_vol_min(analog_ggs,       40286,286,35); уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
35,			// 26						  //measure_vol_min(analog_gg_radio1, 40287,287,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
35			// 27						  //measure_vol_min(analog_gg_radio2, 40288,288,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";
};

const byte  porog_Radio1[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio1 +++++++++++++++++++++++++++++++++++++++++++++++++++++

200,		// 0							//resistor(1, 250);  Установить уровень сигнала xx мв
200,		// 1							//resistor(2, 250);  Установить уровень сигнала xx мв
				 //+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
35,			// 2							//measure_vol_min(analog_FrontL,    40300,300,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
35,			// 3							//measure_vol_min(analog_FrontR,    40301,301,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
35,			// 4							//measure_vol_min(analog_LineL,     40302,302,35); уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
35,			// 5							//measure_vol_min(analog_LineR,     40303,303,35); уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
35,			// 6							//measure_vol_min(analog_mag_radio, 40304,304,35); уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
35,			// 7							//measure_vol_min(analog_mag_phone, 40305,305,35); уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
35,			// 8							//measure_vol_min(analog_ggs,       40306,306,35); уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
35,			// 9							//measure_vol_min(analog_gg_radio1, 40307,307,35); уровень сигнала на выходе "Test Radio1 ** Signal GG Radio1                             OFF - ";
35,			// 10							//measure_vol_min(analog_gg_radio2, 40308,308,35); Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";
				  //++++++++++++++++++++++++++++++++++ Сигнал подан на вход  Radio1 ++++++++++++++++++++++++
35,			// 11							//measure_vol_min(analog_FrontL,    40300,300,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
35,			// 12							//measure_vol_min(analog_FrontR,    40301,301,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
35,			// 13							//measure_vol_min(analog_LineL,     40302,302,35); уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
35,			// 14							//measure_vol_min(analog_LineR,     40303,303,35); уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
35,			// 15							//measure_vol_min(analog_mag_radio, 40304,304,35); уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
35,			// 16							//measure_vol_min(analog_mag_phone, 40305,305,35); уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
35,			// 17							//measure_vol_min(analog_ggs,       40306,306,35); уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
250,		// 18							//measure_vol_max(analog_gg_radio1, 40309,309,250);уровень сигнала на выходе "Test Radio1 ** Signal Radio1                                ON  - ";
35			// 19							//measure_vol_min(analog_gg_radio2, 40308,308,35); уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2       
};

const byte  porog_Radio2[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio2 +++++++++++++++++++++++++++++++++++++++++++++++++++++

200,		// 0							//resistor(1, 250);  Установить уровень сигнала xx мв
200,		// 1							//resistor(2, 250);  Установить уровень сигнала xx мв
				 //+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
35,			// 2							//measure_vol_min(analog_FrontL,    40310,310,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
35,			// 3							//measure_vol_min(analog_FrontR,    40311,311,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
35,			// 4							//measure_vol_min(analog_LineL,     40312,312,35); уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
35,			// 5							//measure_vol_min(analog_LineR,     40313,313,35); уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
35,			// 6 							//measure_vol_min(analog_mag_radio, 40314,314,35); уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
35,			// 7							//measure_vol_min(analog_mag_phone, 40315,315,35); уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
35,			// 8							//measure_vol_min(analog_ggs,       40316,316,35); уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
35,			// 9							//measure_vol_min(analog_gg_radio1, 40317,317,35); уровень сигнала на выходе "Test Radio2 ** Signal GG Radio1                             OFF - ";
35,			// 10							//measure_vol_min(analog_gg_radio2, 40318,318,35); уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";
					//++++++++++++++++++++++++++++++++++ Сигнал подан на вход  Radio2 ++++++++++++++++++++++++
35,			// 11							//measure_vol_min(analog_FrontL,    40310,310,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
35,			// 12							//measure_vol_min(analog_FrontR,    40311,311,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
35,			// 13							//measure_vol_min(analog_LineL,     40312,312,35); уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
35,			// 14							//measure_vol_min(analog_LineR,     40313,313,35); уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
35,			// 15							//measure_vol_min(analog_mag_radio, 40314,314,35); уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
35,			// 16							//measure_vol_min(analog_mag_phone, 40315,315,35); уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
35,			// 17							//measure_vol_min(analog_ggs,       40316,316,35); уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
35,			// 18							//measure_vol_min(analog_gg_radio1, 40317,317,35); уровень сигнала на выходе "Test Radio2 ** Signal Radio1                                ON  - ";
250			// 19							//measure_vol_max(analog_gg_radio2, 40319,319,250);уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2        
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
const char  txt_message55[]   PROGMEM            = "Signal miсrophone 30  mV                      ON"            ;
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




//++++++++++++++++++++++++++++++ Тексты ошибок ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
	if (Serial2.available())                             // есть что-то проверить? Есть данные в буфере?
		  {
			unsigned char overflowFlag = 0 ;               // Флаг превышения размера буфера
			unsigned char buffer_count = 0;                      // Установить в начало чтения буфера

			while (Serial2.available())
				{
				  if (overflowFlag)                        // Если буфер переполнен - очистить
					 Serial2.read();
				  else                                     // Размер буфера в норме, считать информацию
					{
					if (buffer_count == BUFFER_SIZEKF)           // Проверить размер буфера
						{
							overflowFlag = 1;              // Установить флаг превышения размера буфера
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
			mb.Hreg(1+i,regs_out[i]);
		}
}
void waiting_for_replyK()                                  // Чтение данных из Камертона
{
	delayMicroseconds(5);

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
						 mb.Hreg(4+buffer,Serial1.read());
						//regs_in[buffer] = Serial1.read(); 
						buffer++;
					}
				}
//			calculateCRC_In();
			mb.Coil(124,1);                              // Связь с "Камертон" установлена
		   }
	 else 
		{
			Stop_Kam = 0;                                    // Флаг отсутств. инф. из Камертона
			mb.Coil(124,0);                              // Флаг ошибки  связи с "Камертон"
		}

	  //if( mb.Hreg(40007) != regs_temp)
	  //{
		 //Serial.println(mb.Hreg(40004),BIN);
		 //Serial.println(mb.Hreg(40006),BIN);
		 //Serial.println(mb.Hreg(40007),BIN);
	  //}
   //   regs_temp = mb.Hreg(40007);

}
void Stop_Kamerton ()                  //Если не приходит информация с Камертона - регистры обнулить
  {
	 for (unsigned char i = 0; i <4; i++)
	mb.Hreg(4+i,0);
	// regs_in[i]=0;
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
		  mb.Hreg(adr_reg_temp_day,day_temp);  
		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon);                             //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		  mb.Hreg(adr_reg_temp_mon,mon_temp); 
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year);                            //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		 mb.Hreg(adr_reg_temp_year,year_temp); 

		  b = i2c_eeprom_read_byte(0x50, adr_file_name_count);                             //access an address from the memory
		  mb.Hreg(adr_reg_file_name,b);                                                // Регистр  хранения переменной номер файла
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

void file_print_date()  //программа  записи даты в файл
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
//void serial_print_date()                           // Печать даты и времени    
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
			//Wire.requestFrom(address_AD5252, 1, true);  // Считать состояние движка резистора 
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
		mb.Coil(122,1);                              // Флаг ошибки  открытия файла
	}
  }
 
  temp_file_name = ((fileName[BASE_NAME_SIZE]-48)*10) + (fileName[BASE_NAME_SIZE + 1]-48); // преобразование символьного номера файла в числа
 
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
		mb.Coil(122,1);                              // Флаг ошибки  открытия файла
	}
  }

 
  temp_file_name = ((fileName[BASE_NAME_SIZE]-48)*10) + (fileName[BASE_NAME_SIZE + 1]-48); // преобразование символьного номера файла в числа
  mb.Hreg(adr_reg_file_name,temp_file_name);      
//  i2c_eeprom_write_byte(0x50, adr_file_name_count,temp_file_name);                 // при смене даты счетчик номера файла сбросить в "0"

  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  {
	mb.Coil(122,1);                              // Флаг ошибки  открытия файла
  }
  else
  {
	Serial.print(fileName);
	Serial.println(F("  Open Ok!"));

	DateTime now = RTC.now();

	mb.Hreg(adr_Mic_Start_day , now.day());           // Время старта теста
	mb.Hreg(adr_Mic_Start_month, now.month());
	mb.Hreg(adr_Mic_Start_year, now.year());
	mb.Hreg(adr_Mic_Start_hour, now.hour());
	mb.Hreg(adr_Mic_Start_minute, now.minute());
	mb.Hreg(adr_Mic_Start_second, now.second());
	// Уточнить 			
	mb.Hreg(adr_Time_Test_day, 0); 
	mb.Hreg(adr_Time_Test_hour, 0); 
	mb.Hreg(adr_Time_Test_minute, 0); 
	mb.Hreg(adr_Time_Test_second, 0); 
	myFile.println ("");
	myFile.print ("Report of test module Audio-1 N ");
	byte y[4];                                //Чтение из памяти текущих данных счетчика 
		y[3]= mb.Hreg(10);
		y[2]= mb.Hreg(11);
		y[1]= mb.Hreg(12);
		y[0]= mb.Hreg(13);
		number_audio = (unsigned long&) y;       // Сложить восстановленные текущие данные в count_colwater_old
	myFile.print (number_audio);
	myFile.println ("");
	myFile.println ("");
	myFile.println ("");
	myFile.print ("Start test   ");
	file_print_date();
	myFile.println ("");
	mb.Coil(122,0);                              // Флаг индикации открытия файла                                   
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
			mb.Coil(123,0);                                  // Флаг закрытия файла
		}
	else 
		{
			Serial.println();
			Serial.print(fileName);
			Serial.println(" doesn't exist.");  
			mb.Coil(123,1);                              // Флаг ошибки  закрытия файла
		}
	mb.Hreg(adr_control_command,0);                                         // Завершить программу    
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
	sprintf(fileName, "%s%s", str2, "00.TXT");                                 // Получение имени файла в file_name
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
			 sensor_all_off();                                                      // Отключить все сенсоры
			 break;
		case 2:		
			 sensor_all_on();                                                       // Включить все сенсоры
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
			 test_mikrophon();                                                      // Тестирование микрофона
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
				Reg_count_clear();			                                        // Сброс счетчиков ошибок                    
				break;
		case 17:
				test_power();                                                    	// Проверить напряжение  питания
				break;
		case 18:
				set_video();				              //
				
				break;
		case 19:
				test_video();				              //
				break;
		case 20:                                           // Записать уровни порогов заводские
				default_mem_porog();
				break;
		case 21:                                           // 	21 - Записать уровни порогов пользовательские
				set_mem_porog();
				break;
		case 22:                                           // 22 - Получить уровни порогов пользовательские
				read_mem_porog();
				break;
		case 23:   
				controlFileName();                         // Контроль имени файла
				break;
		case 24:   
			//	 set_SD();                                 // Проверка SD памяти
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
	mb.Coil(8,1);                                                               // Включить питание Камертон
	UpdateRegs(); 
	delay(500);
	mb.Coil(5,0);                                                               // Микрофон инструктора отключить
	mb.Coil(10,0);                                                              // Микрофон диспетчера отключить
	mb.Coil(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	mb.Coil(14,0);                                                              // XP8 - 1   PTT     Тангента ножная
	mb.Coil(15,0);                                                              // XS1 - 5   PTT Мик CTS
	mb.Coil(16,0);                                                              // XS1 - 6   sensor подключения микрофона
 
	mb.Coil(17,0);                                                              // J8-12    XP7 4 PTT2 тангента ручная DSR
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	mb.Coil(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	mb.Coil(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR вкл.
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.Coil(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	mb.Coil(30,0);                                                              // XP1- 6  HeS1PTT   CTS вкл
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера

	UpdateRegs(); 
	delay(500);
	UpdateRegs(); 
/*	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];   */ 

	byte i50 = mb.Hreg(4);    
	byte i52 = mb.Hreg(6);     
	byte i53 = mb.Hreg(7);     




		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = mb.Hreg(0);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Hreg(200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			mb.Coil(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки отключен  - Pass
			   }
		  }
	
		if(bitRead(i50,3) != 0)                                                     // J8-11  тангента ручная                           "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
		  {
			regcount = mb.Hreg(201);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			mb.Hreg(201,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			mb.Coil(201,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));         // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));     // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja отключен  - Pass
				}
		  }

		if(bitRead(i50,4) != 0)                                                     // XP8 - 2   sensor Тангента ножная                  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
		  {
			regcount = mb.Hreg(202);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			mb.Hreg(202,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			mb.Coil(202,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));         // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));     // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";  отключен  - Pass
			  }
		  }

		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = mb.Hreg(203);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Hreg(203,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Coil(203,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 отключен  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = mb.Hreg(204);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			mb.Hreg(204,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(204,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor отключен  - Pass
			  }
		  }

		if(bitRead(i52,3) != 0)                                                     // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = mb.Hreg(205);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Hreg(205,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Coil(205,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 отключен  - Pass
			  }
		  }

		
		if(bitRead(i52,4) != 0)                                                     // XP1- 1  HeS1Ls   sensor подкючения гарнитуры диспетчера 
		  {
			regcount = mb.Hreg(206);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			mb.Hreg(206,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			mb.Coil(206,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
			  }
		  }

		if(bitRead(i52,5) != 0)                                                     // XS1 - 6   sensor отключения микрофона
		  {
			regcount = mb.Hreg(207);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			mb.Hreg(207,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			mb.Coil(207,1);                                                     // установить флаг ошибки sensor подключения микрофона
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone отключен  - Pass
			  }
		  }

		if(bitRead(i53,4) != 0)                                                     // Реле RL4 XP1 12  HeS2e   Выключение микрофона инструктора
		  {
			regcount = mb.Hreg(208);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			mb.Hreg(208,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			mb.Coil(208,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));         // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора отключен  - Pass
			  }
		  }

		if(bitRead(i53,6) != 0)                                                     // Реле RL9 XP1 10 Выключение микрофона диспетчера
		  {
			regcount = mb.Hreg(209);                                          // адрес счетчика ошибки Выключение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Выключение микрофона диспетчера
			mb.Hreg(209,regcount);                                            // адрес счетчика ошибки Выключение микрофона диспетчера
			mb.Coil(209,1);                                                     // установить флаг ошибки Выключение микрофона диспетчера
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));         // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));     // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. отключен  - Pass
			   }
		  }
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);

	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
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
	mb.Coil(8,1);                                                               // Включить питание Камертон
	UpdateRegs(); 
	delay(500);
	mb.Coil(5,1);                                                               // Микрофон инструктора включить
	mb.Coil(10,1);                                                              // Микрофон диспетчера включить
	mb.Coil(13,1);                                                              // XP8 - 2   sensor Тангента ножная
	mb.Coil(16,1);                                                              // XS1 - 6   sensor подключения микрофона
	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
	mb.Coil(25,0);                                                              // XP1- 19 HaSs      sensor подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	//mb.Coil(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	//mb.Coil(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	//mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	//mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
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

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = mb.Hreg(210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Hreg(210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			mb.Coil(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки включен  - Pass
			   }
		  }
	
		if(bitRead(i50,3) == 0)                                                     // J8-11  тангента ручная                           "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
		  {
			regcount = mb.Hreg(211);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Hreg(211,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Coil(211,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja включен  - Pass
				}
		  }

		if(bitRead(i50,4) == 0)                                                     // XP8 - 2   sensor Тангента ножная                  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
		  {
			regcount = mb.Hreg(212);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Hreg(212,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Coil(212,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  включен  - Pass
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
	mb.Coil(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
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

		if(bitRead(i52,1) == 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = mb.Hreg(40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Hreg(40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Coil(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 включен  - Pass
			  }
		  }

//--------------------------------------------------------------------------------
		Serial.println("Instr");
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.Coil(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	delay(500);
	UpdateRegs(); 
	delay(500);
	i52 = mb.Hreg(40006);     


		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = mb.Hreg(40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			mb.Hreg(40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor включен  - Pass
			  }
		  }
//-------------------------------------------------------------------------------
		Serial.println("Disp2");
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	delay(500);
	UpdateRegs(); 
	delay(500);

	i52 = mb.Hreg(40006);    

		if(bitRead(i52,3) == 0)                                                     // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = mb.Hreg(40215);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Hreg(40215,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Coil(215,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 включен  - Pass
			  }
		  }
		//--------------------------------------------------------------------------------------------------
		Serial.println("Disp");
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	delay(500);
	UpdateRegs(); 
	delay(500);

	i52 = mb.Hreg(40006);    
		
		if(bitRead(i52,4) == 0)                                                     // XP1- 1  HeS1Ls   sensor подкючения гарнитуры диспетчера 
		  {
			regcount = mb.Hreg(40216);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			mb.Hreg(40216,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			mb.Coil(216,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
			  }
		  }
//------------------------------------------------------------------------------------------------------------------------

*/
		if(bitRead(i52,5) == 0)                                                     // XS1 - 6   sensor включения микрофона
		  {
			regcount = mb.Hreg(217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			mb.Hreg(217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			mb.Coil(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass
			  }
		  }

		if(bitRead(i53,4) == 0)                                                     // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
		  {
			regcount = mb.Hreg(218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			mb.Hreg(218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			mb.Coil(218,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора включен  - Pass
			  }
		  }

		if(bitRead(i53,6) == 0)                                                     // Реле RL9 XP1 10 Выключение микрофона диспетчера
		  {
			regcount = mb.Hreg(219);                                          // адрес счетчика ошибки Включение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона диспетчера
			mb.Hreg(219,regcount);                                            // адрес счетчика ошибки Включение микрофона диспетчера
			mb.Coil(219,1);                                                     // установить флаг ошибки Включение микрофона диспетчера
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. включен  - Pass
			   }
		  }



	mb.Coil(5,0);                                                               // Микрофон инструктора отключить
	mb.Coil(10,0);                                                              // Микрофон диспетчера отключить




	/*
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3])));                    // " ****** Test sensor ON start! ******";    
	myFile.println(buffer);                                                         // " ****** Test sensor ON start! ******";    
	file_print_date();
	myFile.println();
	mb.Coil(8,1);                                                               // Включить питание Камертон
	UpdateRegs(); 
	delay(1000);
	//++++++++++++++++++++++++++++++++++++++++++ Начало проверки ++++++++++++++++++++++++++++++++++++++
	bool test_sens = true;
	mb.Coil(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
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
						if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 включен  - Pass
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
			regcount = mb.Hreg(40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Hreg(40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Coil(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.Coil(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor включен  - Pass
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
			regcount = mb.Hreg(40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			mb.Hreg(40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 включен  - Pass
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
			regcount = mb.Hreg(40215);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Hreg(40215,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Coil(215,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок                                            // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
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
			regcount = mb.Hreg(40216);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			mb.Hreg(40216,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			mb.Coil(216,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибокчетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	mb.Coil(25,0);                                                              // XP1- 19 HaSs      sensor подключения трубки        
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
							if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки включен  - Pass
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
			regcount = mb.Hreg(40210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Hreg(40210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			mb.Coil(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки        
	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja включен  - Pass
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
			regcount = mb.Hreg(40211);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Hreg(40211,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Coil(211,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	mb.Coil(13,1);                                                              // XP8 - 2   sensor Тангента ножная
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  включен  -
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
			regcount = mb.Hreg(40212);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Hreg(40212,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Coil(212,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	mb.Coil(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	mb.Coil(16,1);                                                              // XS1 - 6   sensor подключения микрофона
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
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
			regcount = mb.Hreg(40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			mb.Hreg(40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			mb.Coil(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }



	test_sens = true;                                                               // Флаг выполнения теста
	mb.Coil(16,0);                                                              // XS1 - 6   sensor подключения микрофона отключить
	mb.Coil(5,1);                                                               // Микрофон инструктора включить "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
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
							if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора включен  - Pass
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
			regcount = mb.Hreg(40218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			mb.Hreg(40218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(218,true);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(5,0);                                                               // Микрофон инструктора отключить
	regBank.set(10,1);                                                              // Микрофон диспетчера включить
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
							if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. включен  - Pass
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
			regcount = mb.Hreg(40219);                                          // адрес счетчика ошибки Включение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона диспетчера
			regBank.set(40219,regcount);                                            // адрес счетчика ошибки Включение микрофона диспетчера
			mb.Coil(219,1);                                                     // установить флаг ошибки Включение микрофона диспетчера
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Coil(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	mb.Coil(5,0);                                                               // Микрофон инструктора отключить
	mb.Coil(10,0);                                                              // Микрофон диспетчера отключить
	mb.Coil(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	mb.Coil(14,0);                                                              // XP8 - 1   PTT     Тангента ножная
	mb.Coil(15,0);                                                              // XS1 - 5   PTT Мик CTS
	mb.Coil(16,0);                                                              // XS1 - 6   sensor подключения микрофона
 
	mb.Coil(17,0);                                                              // J8-12    XP7 4 PTT2 тангента ручная DSR
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	mb.Coil(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	mb.Coil(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR вкл.
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.Coil(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	mb.Coil(30,0);                                                              // XP1- 6  HeS1PTT   CTS вкл
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	*/
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(500);
	UpdateRegs(); 
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
	//delay(100);
}

void set_rezistor()
{
	int mwt = mb.Hreg(60);             // Адрес хранения величины сигнала
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
	test_instr_off();                                                               // Отключить реле и сенсоры, прверить отключение
	test_instr_on();                                                                // Включить необходимые сенсоры, проверить состояние
	//myFile.println("");
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 0));                                                                // Установить уровень сигнала 30 мв
	resistor(2, i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 1));                                                                // Установить уровень сигнала 30 мв
	mb.Coil(2,1);                                                               // Подать сигнал на вход микрофона инструктора  Mic2p
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4])));                    // "Signal headset instructor microphone 30mv     ON"            ;   
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset instructor microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 2));                                 // Измерить уровень сигнала на выходе FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 3));                                 // Измерить уровень сигнала на выходе FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40232,232,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 4));                                 // Измерить уровень сигнала на выходе LineL     "Test headset instructor ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40233,233,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 5));                                 // Измерить уровень сигнала на выходе LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40234,234,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 6));                                 // Измерить уровень сигнала на выходе mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	measure_vol_min(analog_mag_phone,40235,235,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 7));                                 // Измерить уровень сигнала на выходе mag phone "Test headset instructor ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 8));                                 // Измерить уровень сигнала на выходе GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 9));                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 10));                                // Измерить уровень сигнала на выходе GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
												   //
	mb.Coil(5,1);                                                               // Подать управляющую команду на вывод 12 ХР1 HeS2e (Включить микрофон)
	mb.Coil(28,1);                                                              // XP1- 15 HeS2PTT Включить PTT инструктора
	mb.Coil(16,0);                                                              // Сенсор микрофона отключить
	mb.Coil(15,0);                                                              // РТТ микрофона отключить
	mb.Coil(29,1);                                                              // ВКЛ XP1- 13 HeS2Ls Кнопка  ВКЛ флаг подключения гарнитуры инструктора 
	UpdateRegs();                                                                   // 
	delay(500); 
	wdt_reset();    //
	byte i53 = mb.Hreg(7);                                                  // Получить текущее состояние Камертона
		if(bitRead(i53,4) == 0)                                                     // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
		  {
			regcount = mb.Hreg(218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			mb.Hreg(218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			mb.Coil(218,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // Микрофон инструктора включен  - Pass
			  }
		  }
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5])));                    // "Microphone headset instructor signal          ON"            ;  
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone headset instructor signal          ON"            ;    Звуковой сигнал подан на вход микрофона инструктора
	delay(500);
	wdt_reset();
	//+++++++++++++++++++++++++++ Проверить наличие сигнала на линиях LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40224,224,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 10));                                // Измерить уровень сигнала на выходе LineL      "Test headset instructor ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40226,226,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 12));                                // Измерить уровень сигнала на выходе mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 13));                                 // Измерить уровень сигнала на выходе FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 14));                                 // Измерить уровень сигнала на выходе FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40233,233,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 15));                                 // Измерить уровень сигнала на выходе LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 16));                                 // Измерить уровень сигнала на выходе GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 17));                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,i2c_eeprom_read_byte(deviceaddress,adr_porog_instruktor + 18));                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";
	wdt_reset();
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	mb.Coil(16,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	mb.Coil(15,0);                                                              // РТТ микрофона отключить
	mb.Coil(5,0);                                                               // Подать управляющую команду на вывод 12 ХР1 HeS2e (Выключить микрофон инструктора)
	mb.Coil(28,0);                                                              // XP1- 15 HeS2Ls Отключить PTT инструктора
	mb.Coil(2,0);                                                               // Выключить сигнал на вход микрофона инструктора  Mic2p
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
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
	test_disp_off();                                                                // Отключить реле и сенсоры, прверить отключение
	test_disp_on();                                                                 // Включить необходимые сенсоры, проверить состояние
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 0));                                                                // Установить уровень сигнала 30 мв
	resistor(2, i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 1));                                                                // Установить уровень сигнала 30 мв
	mb.Coil(1,1);                                                               // Подать сигнал на вход микрофона диспетчера Mic1p
	UpdateRegs();                                                                   // Выполнить команду
	delay(1000);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[13])));                   // "Signal headset dispatcher microphone 30mv     ON"            ;    
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset dispatcher microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 2));                                 // Измерить уровень сигнала на выходе FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 3));                                 // Измерить уровень сигнала на выходе FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40242,242,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 4));                                 // Измерить уровень сигнала на выходе LineL     "Test headset dispatcher ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40243,243,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 5));                                 // Измерить уровень сигнала на выходе LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40244,244,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 6));                                 // Измерить уровень сигнала на выходе mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	measure_vol_min(analog_mag_phone,40245,245,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 7));                                 // Измерить уровень сигнала на выходе mag phone "Test headset dispatcher ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40246,246,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 8));                                 // Измерить уровень сигнала на выходе GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 9));                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 10));                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
//	myFile.println("");                                                             //
	mb.Coil(10,1);                                                              // Подать управляющую команду на вывод XP1 10 Включение микрофона диспетчера
	mb.Coil(30,1);                                                              // XP1- 6  HeS1PTT   Включить PTT диспетчера
	mb.Coil(16,0);                                                              // Сенсор микрофона отключить
	mb.Coil(15,0);                                                              // РТТ микрофона отключить
	mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера

	UpdateRegs();                                                                   // 
	delay(500);                                                                     //

	byte i53 = mb.Hreg(7);     
		if(bitRead(i53,6) == 0)                                                      // Проверка  включения микрофона диспетчера
		  {
			regcount = mb.Hreg(182);                                          // адрес счетчика ошибки включения микрофона диспетчера          "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок включения микрофона диспетчера       "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			mb.Hreg(182,regcount);                                            // адрес счетчика ошибки включения микрофона диспетчера
			mb.Coil(182,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			resistor(1, 255);                                                       // Установить уровень сигнала в исходное состояниe
			resistor(2, 255);                                                       // Установить уровень сигнала в исходное состояниe
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone dispatcher signal ON"  Звуковой сигнал подан на вход микрофона диспетчера
	delay(500);
	wdt_reset();
	//+++++++++++++++++++++++++++ Проверить наличие сигнала на линиях LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40227,227,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 11));                                // Измерить уровень сигнала на выходе LineL     "Test headset dispatcher ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40229,229,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 12));                                // Измерить уровень сигнала на выходе mag phone "Test headset dispatcher ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 13));                                 // Измерить уровень сигнала на выходе FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 14));                                 // Измерить уровень сигнала на выходе FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40243,243,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 15));                                 // Измерить уровень сигнала на выходе LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_ggs,      40246,246,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 16));                                 // Измерить уровень сигнала на выходе GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 17));                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,i2c_eeprom_read_byte(deviceaddress,adr_porog_dispatcher + 18));                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	wdt_reset();
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs   Отключить sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls   Отключить  sensor подкючения гарнитуры диспетчера
	mb.Coil(15,0);                                                              // РТТ микрофона отключить
	mb.Coil(10,0);                                                              // Подать управляющую команду на вывод XP1 10  (Выключить микрофон диспетчера)
	mb.Coil(30,0);                                                              // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	mb.Coil(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	mb.Coil(1,0);                                                               // Отключить сигнал на вход микрофона диспетчера Mic1p
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
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
	test_MTT_off();                                                                 // Отключить реле и сенсоры, прверить отключение
	test_MTT_on();                                                                  // Включить необходимые сенсоры, проверить состояние
//	myFile.println("");
	mb.Coil(25,0);                                                              //  XP1- 19 HaSs  sensor подключения трубки    MTT включить должно быть в "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command sensor ON MTT  send!         
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD Трубку поднять DCD должно быть в "0"
	UpdateRegs();                                                                   // Выполнить команду
	delay(1000);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command  HangUp MTT OFF send!"
	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала динамиков на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 2));                                // Измерить уровень сигнала на выходе FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 3));                                // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40252,252,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 4));                                // Измерить уровень сигнала на выходе LineL     "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40253,253,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 5));                                // Измерить уровень сигнала на выходе LineR     "Test MTT ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 6));                                // Измерить уровень сигнала на выходе mag radio "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40255,255,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 7));                                // Измерить уровень сигнала на выходе mag phone "Test MTT ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 8));                                // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 9));                                // Измерить уровень сигнала на выходе GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 10));                                // Измерить уровень сигнала на выходе GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";

	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 0));                                                               // Установить уровень сигнала 60 мв
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 1));                                                               // Установить уровень сигнала 60 мв
	mb.Coil(3,1);                                                               // Включить сигнал на вход микрофона трубки Mic3p
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));                   // "Signal MTT microphone 30mv                    ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal MTT microphone 30mv                    ON"            ;
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях  +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 11));                                // Измерить уровень сигнала на выходе FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 12));                                // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 13));                                // Измерить уровень сигнала на выходе mag radio   "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 14));                                // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 15));                                // Измерить уровень сигнала на выходе GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 16));                                // Измерить уровень сигнала на выходе GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";
	// ++++++++++++++++++++++++++++++++++ Проверить наличие сигнала  ++++++++++++++++++++++++++++++++++++
	//measure_vol_max(analog_LineL,    40260,260,35);                                 // "Test MTT ** Signal LineL                                    ON  - ";  
	measure_vol_max(analog_LineR,    40261,261,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 17));                                 // "Test MTT ** Signal LineR                                    ON  - ";  
	measure_vol_max(analog_mag_phone,40262,262,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 18));                                 // Измерить уровень сигнала на выходе mag phone  "Test MTT ** Signal Mag phone                                ON  - ";
	// +++++++++++++++++++++ Проверка реагирования вывода ГГС на сигнал HangUp  DCD ON +++++++++++++++++++++++++++++++++
	mb.Coil(3,0);                                                               // Отключить сигнал на вход микрофона трубки Mic3p
	mb.Coil(6,1);                                                               // Реле RL5. Подать звук Front L, Front R
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[35])));                   //   
	if (test_repeat == false) myFile.println(buffer);                               // "Signal FrontL, FrontR  ON                             - "
	measure_vol_min(analog_ggs,       40256,256,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 19));                    // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	mb.Coil(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	wdt_reset();
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;
	measure_vol_max(analog_ggs,      40259,259,i2c_eeprom_read_byte(deviceaddress,adr_porog_MTT + 20));                                //  Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS             On      
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD ON  Положить трубку
	mb.Coil(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR вкл. Отключить PTT MTT
	mb.Coil(25,1);                                                              //  XP1- 19 HaSs  sensor подключения трубки    MTT отключить должно быть в "1"
	mb.Coil(6,0);                                                               // Реле RL5. Отключить звук Front L, Front R
	UpdateRegs();                                                                   // Выполнить команду
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
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
	mb.Coil(17,0);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	mb.Coil(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[37])));                   // "Command sensor OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[38])));                   // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[39])));                   // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 

	byte i50 = mb.Hreg(4);    

	if(bitRead(i50,3) != 0)                                                         // J8-11     XP7 2 sensor тангента ручная               "Command sensor tangenta ruchnaja                            OFF - ";
		{
			regcount = mb.Hreg(274);                                          // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta ruchnaja                            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Command sensor tangenta ruchnaja                            OFF - ";
			mb.Hreg(274,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta ruchnaja                            OFF - ";
			mb.Coil(274,1);                                                     // установить флаг ошибки sensor тангента ручная
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	  // 2)  Проверка  на отключение J8-23     XP7 1 PTT1 тангента ручная CTS
		if(mb.Hreg(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение XP7 1 PTT1 тангента ручная CTS "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
		  {
			regcount = mb.Hreg(270);                                          // адрес счетчика ошибки                                  "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(270,regcount);                                            // адрес счетчика ошибки 
			mb.Coil(270,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	 // 3)  Проверка  на отключение PTT2 тангента ручная (DSR)

		if(mb.Hreg(adr_reg_ind_DSR) != 0)                                       // Проверка  на отключение  PTT2 тангента ручная (DSR) "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
		  {
			regcount = mb.Hreg(271);                                          // адрес счетчика ошибки  PTT  MTT (DSR)                "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(271,regcount);                                            // адрес счетчика ошибки  PTT  MTT (DSR)                 "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			mb.Coil(271,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[40])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	mb.Coil(17,1);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[41])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	mb.Coil(20,1);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[42])));                   // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //

	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	i50 = mb.Hreg(4);    

		if(bitRead(i50,3) == 0)                                          // J8-11     XP7 2 sensor тангента ручная             "Command sensor tangenta ruchnaja                            ON  - ";
		  {
			regcount = mb.Hreg(275);                                          // адрес счетчика ошибки sensor тангента ручная       "Command sensor tangenta ruchnaja                            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная    "Command sensor tangenta ruchnaja                            ON  - ";
			mb.Hreg(275,regcount);                                            // адрес счетчика ошибки sensor тангента ручная
			mb.Coil(275,1);                                                     // установить флаг ошибки sensor тангента ручная
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	  // 2)  Проверка  на отключение J8-23     XP7 1 PTT1 тангента ручная CTS
		if(mb.Hreg(adr_reg_ind_CTS) == 0)                                       // Проверка  на отключение XP7 1 PTT1 тангента ручная CTS    "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
		  {
			regcount = mb.Hreg(272);                                          // адрес счетчика ошибки PTT  MTT (CTS)                      "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(272,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS)
			mb.Coil(272,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	 // 3)  Проверка  на отключение PTT2 тангента ручная (DSR)

		if(mb.Hreg(adr_reg_ind_DSR) == 0)                                       // Проверка  на отключение  PTT2 тангента ручная (DSR)    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
		  {
			regcount = mb.Hreg(273);                                          // адрес счетчика ошибки  PTT  MTT (DSR)                   "Command PTT2 tangenta ruchnaja (DCR)                        ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(273,regcount);                                            // адрес счетчика ошибки  PTT  MTT (DSR)                    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			mb.Coil(273,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	mb.Coil(17,0);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	mb.Coil(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	mb.Coil(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	UpdateRegs();                                                                   // Выполнить команду
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
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
	mb.Coil(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	mb.Coil(14,0);                                                              // XP8 - 1   PTT Тангента ножная
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[44])));                   // "Command sensor OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[45])));                   // "Command PTT    OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //

//	byte i50 = regs_in[0];    
	byte i50 = mb.Hreg(4);    
//	byte i52 = mb.Hreg(40006);     
//	byte i53 = mb.Hreg(40007);     
	if(bitRead(i50,4) != 0)                                                         // J8-11     XP8 2 sensor тангента                  "Command sensor tangenta nognaja                             OFF - ";
		{
			regcount = mb.Hreg(276);                                          // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta nognaja                             OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Command sensor tangenta nognaja                             OFF - ";
			mb.Hreg(276,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta nognaja                             OFF - ";
			mb.Coil(276,1);                                                     // установить флаг ошибки sensor тангента ручная
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	  // 2)  Проверка  на отключение  XP8 1 PTT1 тангента ножная CTS
		if(mb.Hreg(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение XP8 1 PTT1 тангента   "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  {
			regcount = mb.Hreg(278);                                          // адрес счетчика ошибки 
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(278,regcount);                                            // адрес счетчика ошибки                          "Command PTT tangenta nognaja (CTS)                          OFF - ";
			mb.Coil(278,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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


	mb.Coil(13,1);                                                              // XP8 2 sensor тангента ножная
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[46])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	mb.Coil(14,1);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[47])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;


	UpdateRegs();                                                                   // Выполнить команду
	delay(400);

	i50 = mb.Hreg(4);    
	//byte i52 = mb.Hreg(40006);     
	//byte i53 = mb.Hreg(40007);     


			if(bitRead(i50,4) == 0)                                                 // J8-11     XP7 2 sensor тангента                    "Command sensor tangenta nognaja                             ON  - ";
		  {
			regcount = mb.Hreg(277);                                          // адрес счетчика ошибки sensor тангента ручная       "Command sensor tangenta nognaja                             ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная    "Command sensor tangenta nognaja                             ON  - ";
			mb.Hreg(277,regcount);                                            // адрес счетчика ошибки sensor тангента ручная
			mb.Coil(277,1);                                                     // установить флаг ошибки sensor тангента ручная
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	  // 2)  Проверка  на отключение  XP8 1 PTT1 тангента  CTS
		if(mb.Hreg(adr_reg_ind_CTS) == 0)                                       // Проверка  на отключение XP8 1         "Command PTT tangenta nognaja (CTS)                          ON  - ";
		  {
			regcount = mb.Hreg(279);                                          // адрес счетчика ошибки                 "Command PTT tangenta nognaja (CTS)                          ON  - ";          
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(279,regcount);                                            // адрес счетчика ошибки                  "Command PTT tangenta nognaja (CTS)                          ON  - ";
			mb.Coil(279,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	mb.Coil(14,0);                                                              //   XP8 1 PTT тангента  
	mb.Coil(13,0);                                                              //   XP8 2 sensor тангента  
	UpdateRegs();                                                                   // Выполнить команду
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}
void test_mikrophon()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[54])));                   // " ****** Test miсrophone start! ******"                       ;
	myFile.println(buffer);                                                         // " ****** Test miсrophone start! ******"                       ;
	file_print_date();
	myFile.println("");
	mb.Coil(15,0);                                                              // XS1 - 5   PTT Мик CTS
	mb.Coil(16,0);                                                              // XS1 - 6   sensor подключения микрофона

	mb.Coil(14,0);    // XP8 - 1   PTT Тангента ножная
	mb.Coil(17,0);    // J8-12     XP7 4 PTT2   Танг. р.
	mb.Coil(18,0);    // XP1 - 20  HangUp  DCD
	mb.Coil(20,0);    // J8-23     XP7 1 PTT1 Танг. р.
	mb.Coil(26,0);    // XP1- 17 HaSPTT    CTS DSR вкл.  
	mb.Coil(28,0);    // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	mb.Coil(30,0);    // XP1- 6  HeS1PTT   CTS вкл   РТТ Диспетчера

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));                   //"Command PTT    OFF microphone                    send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	wdt_reset();

	 // +++++++++++++++++++++++++++++++++++++++ Проверка  на отключение сенсора и  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
 
	byte i52 = mb.Hreg(6);     
//	byte i53 = mb.Hreg(40007);     

	//byte i52 = regs_in[2];    
			if(bitRead(i52,5) != 0)                                                 // XS1 - 6   sensor отключения микрофона
		  {
			regcount = mb.Hreg(207);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			mb.Hreg(207,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			mb.Coil(207,1);                                                     // установить флаг ошибки sensor подключения микрофона
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone отключен  - Pass
			  }
		  }

	 UpdateRegs(); 
	 delay(500);
	 wdt_reset();
	  // 2)  Проверка  на отключение PTT microphone
		if(mb.Hreg(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение "Test microphone PTT  (CTS)                                  OFF - ";
		  {
			regcount = mb.Hreg(264);                                          // адрес счетчика ошибки       "Test microphone PTT  (CTS)                                  OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(264,regcount);                                            // адрес счетчика ошибки 
			mb.Coil(264,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	 // +++++++++++++++++++++++++++++++++++++++ Проверка  на включение сенсора  microphone ++++++++++++++++++++++++++++++++++++++++++++
	mb.Coil(16,1);                                                              // XS1 - 6   sensor подключения микрофона
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));                   // "Command sensor ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	wdt_reset();

	i52 = mb.Hreg(6);     
	//byte i53 = mb.Hreg(40007);     

	//i52 = regs_in[2];    

	  if(bitRead(i52,5) == 0)                                                 // XS1 - 6   sensor отключения микрофона
		  {
			regcount = mb.Hreg(217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			mb.Hreg(217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			mb.Coil(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
			}
		  }
	  // +++++++++++++++++++++++++++++++++++++++ Проверка  на включение  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
	mb.Coil(15,1);                                                              // XS1 - 5   PTT Мик CTS
	mb.Coil(16,0);                                                              // XS1 - 6   sensor подключения микрофона
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[57])));                   // "Command PTT    ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	wdt_reset();

	i52 = mb.Hreg(6);     
	//byte i53 = mb.Hreg(40007);     
	//i52 = regs_in[2];    

	if(bitRead(i52,5) == 0)                                             // XS1 - 6   sensor отключения микрофона
		  {
			regcount = mb.Hreg(217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			mb.Hreg(217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			mb.Coil(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
			}
		  }

	 UpdateRegs(); 
	 delay(500);
	 wdt_reset();
	  // 2)  Проверка  на включение  PTT microphone
		if(mb.Hreg(adr_reg_ind_CTS) == 0)                                       // Проверка  на включение      "Test microphone PTT  (CTS)                                  ON  
		  {
			regcount = mb.Hreg(266);                                          // адрес счетчика ошибки       "Test microphone PTT  (CTS)                                  ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(266,regcount);                                            // адрес счетчика ошибки 
			mb.Coil(266,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала  на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40320,320,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 2));                                // Измерить уровень сигнала на выходе FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40321,321,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 3));                                // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40322,322,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 4));                                // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40323,323,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 5));                                // Измерить уровень сигнала на выходе LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40324,324,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 6));                                // Измерить уровень сигнала на выходе mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40325,325,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 7));                                // Измерить уровень сигнала на выходе mag phone "Test Microphone ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40326,326,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 8));                                // Измерить уровень сигнала на выходе GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40327,327,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 9));                                // Измерить уровень сигнала на выходе GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40328,328,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 10));                                // Измерить уровень сигнала на выходе GG Radio2 "Test Microphone ** Signal GG Radio2     


		// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 0));                                                                // Установить уровень сигнала 60 мв
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 1));                                                                // Установить уровень сигнала 60 мв
	mb.Coil(9,1);                                                               // Включить сигнал на вход микрофона Реле RL8 Звук на микрофон
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	UpdateRegs();                                                                   // Выполнить команду
//	delay(1000);
	wdt_reset();
	measure_vol_max(analog_mag_phone,40298,298,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 11));                                // Измерить уровень сигнала на выходе mag phone  "Test Microphone ** Signal Mag phone      
	measure_vol_max(analog_LineL,    40299,299,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 12));                                // Измерить уровень сигнала на выходе "Test Microphone ** Signal LineL                      ON  - ";  
	
	measure_vol_min(analog_FrontL,    40320,320,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 13));                                // Измерить уровень сигнала на выходе FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40321,321,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 14));                                // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineR,     40323,323,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 15));                                // Измерить уровень сигнала на выходе LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40324,324,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 16));                                // Измерить уровень сигнала на выходе mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       40326,326,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 17));                                // Измерить уровень сигнала на выходе GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40327,327,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 18));                                // Измерить уровень сигнала на выходе GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40328,328,i2c_eeprom_read_byte(deviceaddress,adr_porog_Microphone + 19));                                // Измерить уровень сигнала на выходе GG Radio2 "Test Microphone ** Signal GG Radio2     

	mb.Coil(9,0);                                                               // Отключить сигнал на вход микрофона Реле RL8 Звук на микрофон
	mb.Coil(16,0);                                                              // XS1 - 6   sensor подключения микрофона
	mb.Coil(15,0);                                                              // XS1 - 5   PTT Мик CTS
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(200);
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    

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
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки  
	mb.Coil(6,0);                                                               // Реле RL5 Звук Front L, Front R
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 0));                                                               // Установить уровень сигнала 60 мв
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 1));                                                               // Установить уровень сигнала 60 мв
	//UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	UpdateRegs(); 
	delay(500);
	UpdateRegs(); 

	byte i50 = mb.Hreg(4);                                                 


		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = mb.Hreg(200);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Hreg(200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			mb.Coil(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки отключен  - Pass
			   }
		  }
		//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40280,280,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 2));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40281,281,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 3));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40282,282,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 4));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 5));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40284,284,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 6));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40285,285,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 7));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40286,286,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 8));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40287,287,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 9));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 10));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";
	//----------------------------------------------------------------------------------------------------------------------------------------

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[49])));                   // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;
	mb.Coil(6,1);                                                               // Реле RL5 Звук Front L, Front R
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);

	measure_vol_max(analog_FrontL,    40290,290,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 11));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    40291,291,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 12));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";
	measure_vol_min(analog_LineL,     40282,282,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 13));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 14));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_max(analog_mag_phone, 40292,292,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 15)); //??                               // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_max(analog_ggs,       40289,289,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 16)); //??                               // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      ON  - ";
	measure_vol_min(analog_gg_radio1, 40287,287,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 17));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 18));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

	mb.Coil(25,0);                                                                          // XP1- 19 HaSs      sensor подключения трубки          
	UpdateRegs();                                                                               // Выполнить команду
	delay(500);
	UpdateRegs(); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                               // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Command sensor ON  MTT                           send!"      ;

	measure_vol_max(analog_FrontL,    290,290,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 19));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    291,291,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 20));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";
	measure_vol_min(analog_LineL,     282,282,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 21));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     283,283,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 22));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
	//measure_vol_min(analog_mag_radio, 40284,284,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 23));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_max(analog_mag_phone, 292,292,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 24));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_min(analog_ggs,       286,286,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 25));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 287,287,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 26));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 288,288,i2c_eeprom_read_byte(deviceaddress,adr_porog_GGS + 27));                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

	mb.Coil(6,0);                                                               // Реле RL5 Звук Front L, Front R
	UpdateRegs();    
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
}
void test_GG_Radio1()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[50])));                   // " ****** Test Radio1 start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test Radio1 start! ******"                           ;
	file_print_date();
	myFile.println("");
	mb.Coil(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 0));                                                               // Установить уровень сигнала 300 мв
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 1));                                                               // Установить уровень сигнала 300 мв
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	UpdateRegs(); 
	delay(500);
	wdt_reset();
	//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40300,300,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 2));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40301,301,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 3));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40302,302,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 4));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40303,303,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 5));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40304,304,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 6));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40305,305,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 7));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40306,306,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 8));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40307,307,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 9));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, 40308,308,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 10));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[51])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	mb.Coil(4,1);                                                               //  Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	UpdateRegs();  
	wdt_reset();
//	Serial.println("test_GG_Radio1 - on ");
	measure_vol_min(analog_FrontL,    300,300,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 11));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    301,301,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 12));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     302,302,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 13));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     303,303,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 14));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
//	measure_vol_min(analog_mag_radio, 40304,304,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 15));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 305,305,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 16));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       306,306,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 17));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_max(analog_gg_radio1, 309,309,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 18));                               // Измерить уровень сигнала на выходе "Test Radio1 ** Signal Radio1                                ON  - ";
	measure_vol_min(analog_gg_radio2, 308,308,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio1 + 19));                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.Coil(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
}
void test_GG_Radio2()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[52])));                   // " ****** Test Radio2 start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test Radio2 start! ******"                           ;
	file_print_date();
	myFile.println("");
	mb.Coil(7,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	resistor(1,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 0));                                                               // Установить уровень сигнала 300 мв
	resistor(2,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 1));                                                               // Установить уровень сигнала 300 мв
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	UpdateRegs(); 
	delay(500);
	wdt_reset();
	//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40310,310,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 2));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40311,311,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 3));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40312,312,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 4));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40313,313,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 5));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40314,314,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 6));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40315,315,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 7));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40316,316,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 8));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40317,317,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 9));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, 40318,318,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 10));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	mb.Coil(7,1);                                                               //  Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	UpdateRegs();  
	wdt_reset();
	//Serial.println("test_GG_Radio2 - on ");

	measure_vol_min(analog_FrontL,    40310,310,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 11));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40311,311,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 12));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40312,312,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 13));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40313,313,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 14));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
	//measure_vol_min(analog_mag_radio, 40314,314,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 15));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40315,315,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 16));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40316,316,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 17));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40317,317,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 18));                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal Radio1                                ON  - ";
	measure_vol_max(analog_gg_radio2, 40319,319,i2c_eeprom_read_byte(deviceaddress,adr_porog_Radio2 + 19));                               // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";

	mb.Coil(7,0);                                                               // Реле RL6 Звук Center
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

	//regBank.add(40293);                         // Aдрес счетчика  ошибки ADC1 напряжение 12/3 вольт
	//regBank.add(40294);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio1
	//regBank.add(40295);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio2
	//regBank.add(40296);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт ГГС
	//regBank.add(40297);                         // Aдрес счетчика  ошибки ADC15 напряжение светодиода 3,6 вольта

	//regBank.add(40493);                         // Aдрес данных измерения ADC1 напряжение 12/3 вольт
	//regBank.add(40494);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio1
	//regBank.add(40495);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio2
	//regBank.add(40496);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт ГГС
	//regBank.add(40497);                         // Aдрес данных измерения ADC15 напряжение светодиода 3,6 вольта

	measure_power();

	// Проверка напряжения 12 вольт платы Камертон
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

	// Проверка напряжения 12 вольт Радио 1
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

	// Проверка напряжения 12 вольт Радио 2
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

	// Проверка напряжения 12 вольт  ГГС
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

	// Проверка напряжения 3,6 вольт на светодиоде микрофона
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
	mb.Hreg(61,100);                                                         // Уровень яркости 100
	delay(300);
	regs_out[0]= 0x2B;                                                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0x84;                                                              // 
	regs_out[2]= mb.Hreg(61);                                                // Уровень яркости
	delay(300);
	regs_out[0]= 0x2B;                                                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                                                              // 
	regs_out[2]= 0x7F;                                                              // Уровень яркости
	measure_mks();                                                                  // Измерить длительность импульсов
	
	mb.Hreg(62,mb.Hreg(5));                                          // Передать уровень яркости в программу

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
	if (mb.Hreg(63) < 18 || mb.Hreg(63) > 26)                         // Диапазон измерения длительности импульса яркости
	{
		myFile.print(buffer);                                                       // 
		myFile.print(mb.Hreg(63));
		myFile.println(" - error");
		mb.Coil(269,1); 
		regcount = mb.Hreg(269);
		regcount++;
		mb.Hreg(269,regcount); 
		mb.Coil(120,1);  
		mb.Hreg(469,mb.Hreg(63));                   // Передать длительность импульса в ПК
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

	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 196 Изменять в реальной схеме
	regs_out[2]= 0x7F;                              // 127 Изменять в реальной схеме
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	mb.Hreg(adr_control_command,0);    
}
void set_video()
{
	delay(300);
	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0x84;                              // 
	regs_out[2]= mb.Hreg(61);                // Уровень яркости
	delay(300);
	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 
	regs_out[2]= 0x7F;                              // Уровень яркости
	measure_mks();                                  // Измерить длительность импульсов
	mb.Hreg(62,mb.Hreg(5));          // Передать уровень яркости в программу
	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 196 Изменять в реальной схеме
	regs_out[2]= 0x7F;                              // 127 Изменять в реальной схеме
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
 mb.Hreg(63,duration);                          // Передать длительность импульса яркости в программу
}

void test_instr_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[7])));                    // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"                   ;  
	mb.Coil(29,0);                                                              // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	mb.Coil(27,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	mb.Coil(16,0);                                                              // XS1 - 6   sensor Мик
	mb.Coil(1,0);                                                               // Реле RL0 Звук
	mb.Coil(2,0);                                                               // Реле RL1 Звук
	mb.Coil(3,0);                                                               // Реле RL2 Звук
	mb.Coil(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	mb.Coil(5,0);                                                               // Реле RL4 XP1 12  HeS2e 
	mb.Coil(6,0);                                                               // Реле RL5 Звук
	mb.Coil(7,0);                                                               // Реле RL6 Звук
	mb.Coil(9,0);                                                               // Реле RL8 Звук на микрофон
	mb.Coil(10,0);                                                              // Реле RL9 XP1 10
	mb.Coil(28,0);                                                              // XP1- 15 HeS2Ls Отключить PTT инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[8])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!"
	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(1000);
	UpdateRegs(); 
	delay(100);
   
	byte i52 = mb.Hreg(6);     
//	byte i53 = mb.Hreg(40007);     


//	byte i52 = regs_in[2];    
	 
	  // 1)  Проверка сенсора на отключение гарнитуры инструктора 2 наушниками
		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = mb.Hreg(203);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Hreg(203,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Coil(203,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 отключен  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = mb.Hreg(204);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			mb.Hreg(204,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(204,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor отключен  - Pass
			  }
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i52,5) != 0)                                                     // Проверка  флага на отключение микрофона
		  {
			regcount = mb.Hreg(207);                                          // адрес счетчика ошибки сенсора микрофона 
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(207,regcount);                                            // адрес счетчика ошибки сенсора микрофона
			mb.Coil(207,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				if (test_repeat == false) myFile.print(buffer);                     // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone                   XS1 - 6                 OFF - ";   отключен  - Pass
			   }
		  }

		UpdateRegs(); 
		delay(100);
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_CTS) != 0)                                        // Проверить включение PTT инструктора   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = mb.Hreg(220);                                          // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(220,regcount);                                            // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			mb.Coil(220,1);                                                     // установить флаг ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        OFF - "  отключен  - Pass
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
	mb.Coil(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[11])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor 2  send!"
	mb.Coil(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor  Танг. р.
	mb.Coil(16,1);                                                              // XS1 - 6   sensor Мик
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки      
	mb.Coil(13,1);                                                              // XP8 - 2           sensor Тангента ножная
	mb.Coil(28,1);                                                              // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[12])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
	UpdateRegs();                                                                   // Выполнить команду включения сенсоров
	delay(1000);
	UpdateRegs(); 
	delay(100);
	wdt_reset();
 
	byte i52 = mb.Hreg(6);     

	  // 3)  Проверка сенсора на подключение гарнитуры инструктора 2 наушниками
			if(bitRead(i52,1) == 0)                                                 // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = mb.Hreg(213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Hreg(213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			mb.Coil(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 включен  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = mb.Hreg(214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			mb.Hreg(214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor включен  - Pass
			  }
		  }

		UpdateRegs(); 
		delay(100);
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_CTS)== 0)                                         // Проверить включение PTT инструктора   CTS "Command PTT headset instructor (CTS)                        ON  - ";
		  {
			regcount = mb.Hreg(221);                                          // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(221,regcount);                                            // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        ON  - ";
			mb.Coil(221,1);                                                     // установить флаг ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                       ON  - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        ON  - "  включен  - Pass
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
	mb.Coil(32,0);                                                              // XP1- 1  HeS1Ls    Отключить сенсор гарнитуры диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[15])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	mb.Coil(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(16,0);                                                              // XS1 - 6   sensor Мик
	mb.Coil(1,0);                                                               // Реле RL0 Звук
	mb.Coil(2,0);                                                               // Реле RL1 Звук
	mb.Coil(3,0);                                                               // Реле RL2 Звук
	mb.Coil(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	mb.Coil(5,0);                                                               // Реле RL4 XP1 12  HeS2e 
	mb.Coil(6,0);                                                               // Реле RL5 Звук
	mb.Coil(7,0);                                                               // Реле RL6 Звук
	mb.Coil(9,0);                                                               // Реле RL8 Звук на микрофон
	mb.Coil(10,0);                                                              // Реле RL9 XP1 10
	mb.Coil(30,0);                                                              // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[17])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!""

	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(1000);
	UpdateRegs(); 
   
	byte i52 = mb.Hreg(6);     
	//byte i53 = mb.Hreg(40007);     
	//byte i52 = regs_in[2];    
	 
	  // 1)  Проверка сенсора на отключение гарнитуры диспетчера 2 наушниками
		if(bitRead(i52,3) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = mb.Hreg(205);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера с 2 наушниками
			mb.Hreg(205,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			mb.Coil(205,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера с 2 наушниками 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - " отключен  - Pass
			  }
		  }

		if(bitRead(i52,4) != 0)                                                     //"Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "  подключения гарнитуры диспетчера
		  {
			regcount = mb.Hreg(206);                                          // адрес счетчика ошибки sensor подключения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера
			mb.Hreg(206,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры диспетчера
			mb.Coil(206,1);                                                     // установить флаг ошибки sensor подключения гарнитуры диспетчера
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - " отключен  - Pass
			  }
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i52,5) != 0)                                                     // Проверка  флага на отключение микрофона
		  {
			regcount = mb.Hreg(207);                                          // адрес счетчика ошибки сенсора микрофона 
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(207,regcount);                                            // адрес счетчика ошибки сенсора микрофона
			mb.Coil(207,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor microphone                   XS1 - 6                 OFF - ";   отключен  - Pass
			   }
		  }

		UpdateRegs(); 
		delay(100);
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_CTS) != 0)                                        // Проверить отключения PTT диспетчера   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = mb.Hreg(222);                                          // адрес счетчика   ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(222,regcount);                                            // адрес счетчика   ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			mb.Coil(222,1);                                                     // установить флаг  ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        OFF - "  отключен  - Pass
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
	mb.Coil(32,1);                                                              // XP1- 1  HeS1Ls    sensor подключения гарнитуры диспетчера 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20])));                   // "Command sensor ON  headset dispatcher            send!"      ;    
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher            send!"      ;    
	mb.Coil(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	mb.Coil(19,1);                                                              // J8-11     XP7 2 sensor  Танг. р.
	mb.Coil(16,1);                                                              // XS1 - 6   sensor Мик
	mb.Coil(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки      
	mb.Coil(13,1);                                                              // XP8 - 2           sensor Тангента ножная
	mb.Coil(30,1);                                                              // XP1- 6  HeS1PTT   Включить PTT диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
	UpdateRegs();                                                                   // Выполнить команду включения сенсоров
	delay(1000);
	UpdateRegs(); 
  
	byte i52 = mb.Hreg(6);     
	//byte i53 = mb.Hreg(40007);     
	//byte i52 = regs_in[2];    

	  // 3)  Проверка сенсора на подключение гарнитуры диспетчера 2 наушниками
		if(bitRead(i52,3) == 0)                                                 // XP1- 16 HeS2Rs    sensor подключения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = mb.Hreg(215);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера с 2 наушниками
			mb.Hreg(215,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			mb.Coil(215,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера с 2 наушниками 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  включен  - Pass
			  }
		  }

		if(bitRead(i52,4) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры диспетчера 
		  {
			regcount = mb.Hreg(216);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера
			mb.Hreg(216,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера 
			mb.Coil(216,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера 
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - " включен  - Pass
			  }
		  }

		UpdateRegs(); 
		wdt_reset();
	   if(mb.Hreg(adr_reg_ind_CTS)== 0)                                         // Проверить включение PTT диспетчера   "Command PTT headset dispatcher (CTS)                        ON  - ";
		  {
			regcount = mb.Hreg(223);                                          // адрес счетчика ошибки  отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(223,regcount);                                            // адрес счетчика ошибки  отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        ON  - ";
			mb.Coil(223,1);                                                     // установить флаг ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                       ON  - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        ON  - "  включен  - Pass
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
	mb.Coil(26,0);                                                              // XP1- 17 HaSPTT    CTS  вкл. Отключить PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[28])));                   // "Command PTT    OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    OFF MTT                           send! "     ;
	mb.Coil(18,0);                                                              // XP1 - 20  HangUp  DCD
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                   // "Command        OFF HangUp MTT                    send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command        OFF HangUp MTT                    send! "     ;
	mb.Coil(16,0);                                                              // XS1 - 6   sensor Мик
	mb.Coil(1,0);                                                               // Реле RL0 Звук
	mb.Coil(2,0);                                                               // Реле RL1 Звук
	mb.Coil(3,0);                                                               // Реле RL2 Звук
	mb.Coil(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	mb.Coil(5,0);                                                               // Реле RL4 XP1 12  HeS2e 
	mb.Coil(6,0);                                                               // Реле RL5 Звук
	mb.Coil(7,0);                                                               // Реле RL6 Звук
	mb.Coil(9,0);                                                               // Реле RL8 Звук на микрофон
	mb.Coil(10,0);                                                              // Реле RL9 XP1 10
	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(1000);
	UpdateRegs(); 
	delay(100);
	byte i50 = mb.Hreg(4);    
	//byte i52 = mb.Hreg(40006);     
	//byte i53 = mb.Hreg(40007);     
	//byte i50 = regs_in[0];    
	wdt_reset();
		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = mb.Hreg(200);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Hreg(200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			mb.Coil(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  трубки отключен  - Pass
			   }
		  }
		   UpdateRegs(); 
		   delay(1000);
		   wdt_reset();
	  // 2)  Проверка  на отключение PTT  MTT (CTS)
		if(mb.Hreg(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение CTS MTT
		  {
			regcount = mb.Hreg(263);                                          // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(263,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			mb.Coil(263,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));        // "Test MTT PTT    (CTS)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	   if(mb.Hreg(adr_reg_ind_DCD)!= 0)                                         // Проверить включение HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = mb.Hreg(267);                                          // адрес счетчика ошибки отключения HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(267,regcount);                                            // адрес счетчика ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			mb.Coil(267,1);                                                     // установить флаг ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));        // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	mb.Coil(25,0);                                                              //  XP1- 19 HaSs  sensor подключения трубки    MTT ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  MTT                           send!"      ;              
	mb.Coil(26,1);                                                              // XP1- 17 HaSPTT    CTS DSR вкл. включить PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[31])));                   // "Command PTT    ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    ON  MTT                           send!"      ;
	mb.Coil(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;

	UpdateRegs(); 
	delay(1000);
	UpdateRegs();
	wdt_reset();

	  // 1)  Проверка сенсора MTT на включение 
	byte i50 = mb.Hreg(4);    
	//byte i52 = mb.Hreg(40006);     
	//byte i53 = mb.Hreg(40007);     
	//byte i50 = regs_in[0];    
		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = mb.Hreg(210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Hreg(210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			mb.Coil(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  трубки включен  - Pass
			   }
		  }

		delay(1000);
		UpdateRegs(); 

	  // 2)  Проверка  на отключение PTT  MTT (CTS)
		if(mb.Hreg(adr_reg_ind_CTS) == 0)                                       // Проверка  на включение CTS MTT
		  {
			regcount = mb.Hreg(265);                                          // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(265,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			mb.Coil(265,1);                                                     // установить флаг ошибки
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));        // "Test MTT PTT    (CTS)                                       ON  - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));    // "Test MTT PTT    (CTS)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT PTT    (CTS)                                       ON  - " трубки включен  - Pass
			   }
		  }

	   if(mb.Hreg(adr_reg_ind_DCD)== 0)                                         // Проверить включение HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = mb.Hreg(268);                                          // адрес счетчика ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			mb.Hreg(268,regcount);                                            // адрес счетчика ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			mb.Coil(268,1);                                                     // установить флаг ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			mb.Coil(120,1);                                                     // установить общий флаг ошибки
			regcount_err = mb.Hreg(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			mb.Hreg(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));        // "Test MTT HangUp (DCD)                                       ON  - ";  
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));    // "Test MTT HangUp (DCD)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT HangUp (DCD)                                       ON  - ";трубки включен  - Pass
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
		measure_volume(_istochnik);                                                 // Измерить уровень сигнала на выходе
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
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[80])));    //  ошибки "Test GGS ** Signal FrontL                                   OFF - ";
				break;
			case 281:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[81])));    // ошибки "Test GGS ** Signal FrontR                                   OFF - ";
				break;
			case 282:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[82])));    // ошибки "Test GGS ** Signal LineL                                    OFF - ";
				break;
			case 283:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[83])));    // ошибки "Test GGS ** Signal LineR                                    OFF - ";
				break;
			case 284:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[84])));    // ошибки "Test GGS ** Signal mag radio                                OFF - ";
				break;
			case 285:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[85])));    // ошибки "Test GGS ** Signal mag phone                                OFF - ";
				break;
			case 286:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[86])));    // ошибки "Test GGS ** Signal GGS                                      OFF - ";
				break;
			case 287:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[87])));    // ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
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
		mb.Hreg(_adr_count+200,voltage10);                               // адрес данных ошибки канала 
		if(voltage10 > _porogV)                                                      // Проверить исправность канала
			{
				myFile.print(buffer); 
				regcount = mb.Hreg(_adr_count);                                  // адрес счетчика ошибки 
				regcount++;                                                          // увеличить счетчик ошибок канала 
				mb.Hreg(_adr_count,regcount);                                    // адрес счетчика ошибки канала 
				mb.Hreg(_adr_count+200,voltage10);                               // адрес данных ошибки канала 
				mb.Coil(_adr_flagErr,1);                                         // установить флаг ошибки  канала 
				regcount_err = mb.Hreg(adr_reg_count_err);                       // Получить данные счетчика всех ошибок
				regcount_err++;                                                      // увеличить счетчик всех ошибок 
				mb.Hreg(adr_reg_count_err,regcount_err);                         // Сохранить данные счетчика всех ошибок
				mb.Coil(120,1);                                                  // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));         // "    Error! - "; 
				myFile.print(buffer);                                                // "    Error! - "; 
				myFile.print(regcount);                                              // Показания счетчика ошибок
				myFile.print("  ");  
				myFile.print(voltage); 
				myFile.println(" V");
			}
		else
			{
				if (test_repeat == false)
				{
					myFile.print(buffer);                                           // Наименование проверки
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

	measure_volume(_istochnik);                                                 // Измерить уровень сигнала на выходе
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
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[89])));    // ошибки "Test GGS ** Signal GGS                                      ON  - ";
				break;
			case 290:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[90])));    // ошибки "Test GGS ** Signal FrontL                                   ON  - ";
				break;
			case 291:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[91])));    // ошибки "Test GGS ** Signal FrontR                                   ON  - ";
				break;
			case 292:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[92])));    // ошибки "Test GGS ** Signal mag phone                                ON  - ";
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
	mb.Hreg(_adr_count+200,voltage10);                              // адрес данных ошибки канала 
		if(voltage10 < _porogV)                                                     // Проверить исправность канала
			{
				myFile.print(buffer); 
				regcount = mb.Hreg(_adr_count);                                 // адрес счетчика ошибки 
				regcount++;                                                         // увеличить счетчик ошибок канала 
				mb.Hreg(_adr_count, regcount);                                  // адрес счетчика ошибки канала 
				//mb.Hreg(_adr_count+200,voltage10);                              // адрес данных ошибки канала 
				mb.Coil(_adr_flagErr,1);                                        // установить флаг ошибки  канала 
				regcount_err = mb.Hreg(adr_reg_count_err);                       // Получить данные счетчика всех ошибок
				regcount_err++;                                                      // увеличить счетчик всех ошибок 
				mb.Hreg(adr_reg_count_err,regcount_err);                         // Сохранить данные счетчика всех ошибок
				mb.Coil(120,1);                                                 // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.print(regcount);                                             // Показания счетчика ошибок
				myFile.print("  ");  
				myFile.print(voltage); 
				myFile.println(" V");
			}
		else
			{
			if (test_repeat == false)
				{
					myFile.print(buffer);                                           // Наименование проверки
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
				Array_volume[i] = analogRead(analog);               // считываем значение
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
		volume_fact = analogRead(analog);               // считываем значение
		voltage = volume_fact * (5.0 / 1023.0);
		voltage10 = voltage * 100;

		//Serial.print("voltage - ");
		//Serial.println(voltage10);
}
void measure_power()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mb.Coil(21,0);                           // XP2-2     sensor "Маг."  
	mb.Coil(22,0);                           // XP5-3     sensor "ГГC."
	mb.Coil(23,0);                           // XP3-3     sensor "ГГ-Радио1."
	mb.Coil(24,0);                           // XP4-3     sensor "ГГ-Радио2."
	UpdateRegs();         
	delay(100);

	measure_volume_P(analog_tok);     
	mb.Hreg(400,voltage10);                     
	measure_volume_P(analog_12V);   
	mb.Hreg(493,voltage10);   
	measure_volume_P(analog_tok_x10);   
	mb.Hreg(402,voltage10);   

	mb.Coil(23,1);                           // XP3-3     sensor "ГГ-Радио1."
	UpdateRegs();         
	delay(200);
	wdt_reset();
	measure_volume_P(analog_14); 
	mb.Hreg(494,voltage10);   

	mb.Coil(23,0);                           // XP3-3     sensor "ГГ-Радио1."
	mb.Coil(24,1);                           // XP4-3     sensor "ГГ-Радио2."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	mb.Hreg(495,voltage10);   

	mb.Coil(24,0);                           // XP4-3     sensor "ГГ-Радио2."
	mb.Coil(22,1);                           // XP5-3     sensor "ГГC."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	mb.Hreg(496,voltage10);   
	wdt_reset();
	mb.Coil(22,0);                           // XP5-3     sensor "ГГC."
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

void default_mem_porog()  // Запись заводских установок уровней порога
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
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}

void set_mem_porog()
{
	/*
		Программа записи порогов в EEPROM
		Стартовый адрес памяти 200
		Стартовый адрес регистров 40130 
		Длина блока не более ??? байт
		mb.Hreg(40127);  //  Адрес блока регистров для передачи в ПК уровней порогов.
		mb.Hreg(40128);  //  Адрес блока памяти для передачи в ПК уровней порогов.
		mb.Hreg(40129);  //  Адрес длины блока памяти для передачи в ПК уровней порогов.
	*/
	int _adr_reg  = mb.Hreg(127);              // Начальный адрес блока регистров, 
	int _adr_mem  = mb.Hreg(128);              // Начальный адрес блока памяти
	int _step_mem = mb.Hreg(129);              // Длина блока с учетом хранения двухбайтных чисел
	int _u_porog  = 0;                               // Временное хранения содержимого регистра.
	int i_k       = 0;                               // Смещение адреса блока памяти

	for (int i = 0; i < _step_mem;i++)                            // Копирование блока памяти в регистры.        
		{
			_u_porog = mb.Hreg(_adr_reg+i);
		   // разбираем _u_porog на byte
			hi=highByte(_u_porog);
			low=lowByte(_u_porog);
		//	// тут мы эти hi,low можем сохранить EEPROM
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k, low); 
			i_k++;
		}
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
	delay(200);
}
void read_mem_porog()
{
	/*
		Программа записи уровней порога из памяти в регистры
		mb.Hreg(40127);  //  Адрес блока регистров для передачи в ПК уровней порогов.
		mb.Hreg(40128);  //  Адрес блока памяти для передачи в ПК уровней порогов.
		mb.Hreg(40129);  //  Адрес длины блока памяти для передачи в ПК уровней порогов.
		Стартовый адрес памяти 200
		Стартовый адрес регистров 40130 
		Длина блока не более ??? байт
	*/
	int _adr_reg  = mb.Hreg(127);              // Начальный адрес блока регистров, 
	int _adr_mem  = mb.Hreg(128);              // Начальный адрес блока памяти
	int _step_mem = mb.Hreg(129);              // Длина блока с учетом хранения двухбайтных чисел
	int _u_porog = 0;                                      // Временное хранения содержимого регистра.
	int i_k = 0;                                           // Смещение адреса блока памяти

	for (int i = 0; i < _step_mem;i++)                            // Копирование блока памяти в регистры.        
		{

		  hi  = i2c_eeprom_read_byte(deviceaddress,_adr_mem+i_k);   // 
		  i_k++;
		  low = i2c_eeprom_read_byte(deviceaddress,_adr_mem+i_k);
		  i_k++;
		   _u_porog = (hi<<8) | low;                              // собираем как "настоящие программеры"
		  mb.Hreg(_adr_reg+i,_u_porog);
		 }
	
	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
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
//				 read_mem_regBank(adr_porog_dispatcher , 19);        //headset_dispatcher			                                        // Сброс счетчиков ошибок                    
//				break;
//		case 3:
//				read_mem_regBank(adr_porog_MTT , 21);                //MTT                                                    	// Проверить напряжение  питания
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
//	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
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
	
	if (Serial2.available())                               // есть что-то проверить? Есть данные в буфере?
		  {
			unsigned char overflowFlag = 0 ;               // Флаг превышения размера буфера
			unsigned char buffer = 0;                      // Установить в начало чтения буфера

			while (Serial2.available())
				{
				  if (overflowFlag)                        // Если буфер переполнен - очистить
					 Serial2.read();
				  else                                     // Размер буфера в норме, считать информацию
					{
					if (bufferK == BUFFER_SIZEKF)           // Проверить размер буфера
						{
							overflowFlag = 1;              // Установить флаг превышения размера буфера
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
	
//	wdt_reset();  // Сброс сторожевого таймера при наличии связи с ПК
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
	 //Serial.println("error opening file");                            // Флаг ошибки  открытия файла
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


	mb.Hreg(adr_control_command,0);                                             // Завершить программу    
	delay(100);
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
   // Подключение к протоколу MODBUS компьютера Serial3    
	mb.addCoil(0);                           // Реле 
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


	mb.addCoil(118,0);                         // Флаг индикации многоразовой проверки
	mb.addCoil(119,0);                         // 

	mb.addCoil(120,0);                         // Флаг индикации возникновения любой ошибки
	mb.addCoil(122,0);                         // Флаг индикации открытия файла
	mb.addCoil(123,0);                         // Флаг индикации закрытия файла
	mb.addCoil(124,0);                         // Флаг индикации связи с модулем "Камертон"
	mb.addCoil(125,0);                         // Флаг индикации инициализации SD памяти
	mb.addCoil(126,0);                         //  
	mb.addCoil(127,0);                         //  
	mb.addCoil(128,0);                         //  
	mb.addCoil(129,0);                         //  

	mb.addCoil(130,0);                         //  Флаг индикации порта 0 - RS232, 1 - USB0
	mb.addCoil(131,0);                         //  


	mb.addCoil(200,0);                         // Флаг ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	mb.addCoil(201,0);                         // Флаг ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	mb.addCoil(202,0);                         // Флаг ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	mb.addCoil(203,0);                         // Флаг ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	mb.addCoil(204,0);                         // Флаг ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	mb.addCoil(205,0);                         // Флаг ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	mb.addCoil(206,0);                         // Флаг ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	mb.addCoil(207,0);                         // Флаг ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
	mb.addCoil(208,0);                         // Флаг ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	mb.addCoil(209,0);                         // Флаг ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	mb.addCoil(210,0);                         // Флаг ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	mb.addCoil(211,0);                         // Флаг ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	mb.addCoil(212,0);                         // Флаг ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	mb.addCoil(213,0);                         // Флаг ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	mb.addCoil(214,0);                         // Флаг ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	mb.addCoil(215,0);                         // Флаг ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	mb.addCoil(216,0);                         // Флаг ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	mb.addCoil(217,0);                         // Флаг ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
	mb.addCoil(218,0);                         // Флаг ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	mb.addCoil(219,0);                         // Флаг ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
	 
	mb.addCoil(220,0);                         // Флаг ошибки "Command PTT headset instructor (CTS)                        OFF - ";
	mb.addCoil(221,0);                         // Флаг ошибки "Command PTT headset instructor (CTS)                        ON  - ";
	mb.addCoil(222,0);                         // Флаг ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
	mb.addCoil(223,0);                         // Флаг ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
	mb.addCoil(224,0);                         // Флаг ошибки "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addCoil(225,0);                         // Флаг ошибки "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addCoil(226,0);                         // Флаг ошибки "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addCoil(227,0);                         // Флаг ошибки "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addCoil(228,0);                         // Флаг ошибки "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addCoil(229,0);                         // Флаг ошибки "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addCoil(230,0);                         // Флаг ошибки "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addCoil(231,0);                         // Флаг ошибки "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addCoil(232,0);                         // Флаг ошибки "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addCoil(233,0);                         // Флаг ошибки "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addCoil(234,0);                         // Флаг ошибки "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addCoil(235,0);                         // Флаг ошибки "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addCoil(236,0);                         // Флаг ошибки "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addCoil(237,0);                         // Флаг ошибки "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addCoil(238,0);                         // Флаг ошибки "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addCoil(239,0);                         // Флаг ошибки  ADC0  ток x1 

	mb.addCoil(240,0);                         // Флаг ошибки "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addCoil(241,0);                         // Флаг ошибки "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addCoil(242,0);                         // Флаг ошибки "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addCoil(243,0);                         // Флаг ошибки "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addCoil(244,0);                         // Флаг ошибки "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addCoil(245,0);                         // Флаг ошибки "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addCoil(246,0);                         // Флаг ошибки "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addCoil(247,0);                         // Флаг ошибки "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addCoil(248,0);                         // Флаг ошибки "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addCoil(249,0);                         // Флаг ошибки ADC2 ток x10  

	mb.addCoil(250,0);                         // Флаг ошибки "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addCoil(251,0);                         // Флаг ошибки "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addCoil(252,0);                         // Флаг ошибки "Test MTT ** Signal LineL                                    OFF - ";
	mb.addCoil(253,0);                         // Флаг ошибки "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addCoil(254,0);                         // Флаг ошибки "Test MTT ** Signal mag radio                                OFF - ";
	mb.addCoil(255,0);                         // Флаг ошибки "Test MTT ** Signal mag phone                                OFF - ";
	mb.addCoil(256,0);                         // Флаг ошибки "Test MTT ** Signal GGS                                      OFF - ";
	mb.addCoil(257,0);                         // Флаг ошибки "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addCoil(258,0);                         // Флаг ошибки "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addCoil(259,0);                         // Флаг ошибки "Test MTT ** Signal GGS                                      ON  - ";

	mb.addCoil(260,0);                         // Флаг ошибки "Test MTT ** Signal LineL                                    ON  - ";
	mb.addCoil(261,0);                         // Флаг ошибки "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addCoil(262,0);                         // Флаг ошибки "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addCoil(263,0);                         // Флаг ошибки "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addCoil(264,0);                         // Флаг ошибки "Test microphone PTT  (CTS)                                  OFF - ";
	mb.addCoil(265,0);                         // Флаг ошибки "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addCoil(266,0);                         // Флаг ошибки "Test microphone PTT  (CTS)                                  ON  - ";
	mb.addCoil(267,0);                         // Флаг ошибки "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addCoil(268,0);                         // Флаг ошибки "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addCoil(269,0);                         // Флаг ошибки Длительность регулировки яркости 

	mb.addCoil(270,0);                         // Флаг ошибки "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addCoil(271,0);                         // Флаг ошибки "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addCoil(272,0);                         // Флаг ошибки "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addCoil(273,0);                         // Флаг ошибки "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addCoil(274,0);                         // Флаг ошибки "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addCoil(275,0);                         // Флаг ошибки "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addCoil(276,0);                         // Флаг ошибки "Command sensor tangenta nognaja                             OFF - ";
	mb.addCoil(277,0);                         // Флаг ошибки "Command sensor tangenta nognaja                             ON  - ";
	mb.addCoil(278,0);                         // Флаг ошибки "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addCoil(279,0);                         // Флаг ошибки "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addCoil(280,0);                         // Флаг ошибки "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addCoil(281,0);                         // Флаг ошибки "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addCoil(282,0);                         // Флаг ошибки "Test GGS ** Signal LineL                                    OFF - ";
	mb.addCoil(283,0);                         // Флаг ошибки "Test GGS ** Signal LineR                                    OFF - ";
	mb.addCoil(284,0);                         // Флаг ошибки "Test GGS ** Signal mag radio                                OFF - ";
	mb.addCoil(285,0);                         // Флаг ошибки "Test GGS ** Signal mag phone                                OFF - ";
	mb.addCoil(286,0);                         // Флаг ошибки "Test GGS ** Signal GGS                                      OFF - ";
	mb.addCoil(287,0);                         // Флаг ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addCoil(288,0);                         // Флаг ошибки "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addCoil(289,0);                         // Флаг ошибки "Test GGS ** Signal GGS                                      ON  - ";

	mb.addCoil(290,0);                         // Флаг ошибки "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addCoil(291,0);                         // Флаг ошибки "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addCoil(292,0);                         // Флаг ошибки "Test GGS ** Signal mag phone                                ON  - ";
	mb.addCoil(293,0);                         // Флаг ошибки ADC1 напряжение 12/3 вольт
	mb.addCoil(294,0);                         // Флаг ошибки ADC14 напряжение 12/3 вольт Radio1
	mb.addCoil(295,0);                         // Флаг ошибки ADC14 напряжение 12/3 вольт Radio2
	mb.addCoil(296,0);                         // Флаг ошибки ADC14 напряжение 12/3 вольт ГГС
	mb.addCoil(297,0);                         // Флаг ошибки ADC15 напряжение светодиода 3,6 вольта
	mb.addCoil(298,0);                         // Флаг ошибки "Test Microphone ** Signal mag phone                         ON  - ";      
	mb.addCoil(299,0);                         // Флаг ошибки "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addCoil(300,0);                         // Флаг ошибки "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addCoil(301,0);                         // Флаг ошибки "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addCoil(302,0);                         // Флаг ошибки "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addCoil(303,0);                         // Флаг ошибки "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addCoil(304,0);                         // Флаг ошибки "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addCoil(305,0);                         // Флаг ошибки "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addCoil(306,0);                         // Флаг ошибки "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addCoil(307,0);                         // Флаг ошибки "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addCoil(308,0);                         // Флаг ошибки "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addCoil(309,0);                         // Флаг ошибки "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addCoil(310,0);                         // Флаг ошибки "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addCoil(311,0);                         // Флаг ошибки "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addCoil(312,0);                         // Флаг ошибки "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addCoil(313,0);                         // Флаг ошибки "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addCoil(314,0);                         // Флаг ошибки "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addCoil(315,0);                         // Флаг ошибки "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addCoil(316,0);                         // Флаг ошибки "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addCoil(317,0);                         // Флаг ошибки "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addCoil(318,0);                         // Флаг ошибки "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addCoil(319,0);                         // Флаг ошибки "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addCoil(320,0);                         // Флаг ошибки "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addCoil(321,0);                         // Флаг ошибки "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addCoil(322,0);                         // Флаг ошибки "Test Microphone ** Signal LineL                             OFF - ";
	mb.addCoil(323,0);                         // Флаг ошибки "Test Microphone ** Signal LineR                             OFF - ";
	mb.addCoil(324,0);                         // Флаг ошибки "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addCoil(325,0);                         // Флаг ошибки "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addCoil(326,0);                         // Флаг ошибки "Test Microphone ** Signal GGS                               OFF - ";
	mb.addCoil(327,0);                         // Флаг ошибки "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addCoil(328,0);                         // Флаг ошибки "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addCoil(329,0);                         // Флаг ошибки Код яркости дисплея

	mb.addCoil(330,0);                         // Флаг ошибки "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addCoil(331,0);                         // Флаг ошибки "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	mb.addCoil(332,0);                         // Флаг ошибки "Test GGS ** Signal mag radio   


	mb.addIsts(81,0);    // Адрес флагa индикации состояния сигнала CTS
	mb.addIsts(82,0);    // Адрес флагa индикации состояния сигнала DSR
	mb.addIsts(83,0);    // Адрес флагa индикации состояния сигнала DCD



	//mb.addHreg(40000);  // 

	mb.addHreg(1,0);  // Регистры обмена с Аудио 1
	mb.addHreg(2,0);  // Регистры обмена с Аудио 1
	mb.addHreg(3,0);  // Регистры обмена с Аудио 1
	mb.addHreg(4,0);  // Регистры обмена с Аудио 1
	mb.addHreg(5,0);  // Регистры обмена с Аудио 1
	mb.addHreg(6,0);  // Регистры обмена с Аудио 1
	mb.addHreg(7,0);  // Регистры обмена с Аудио 1
	mb.addHreg(8,0);  // 
	mb.addHreg(9,0);  // 




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
	mb.addHreg(48,4);  // адрес год модуля часов контроллера
	mb.addHreg(49,4);  // адрес час модуля часов контроллера
	mb.addHreg(50,5);  // адрес минута модуля часов контроллера
	mb.addHreg(51,6);  // адрес секунда модуля часов контроллера
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


	mb.addHreg(200,0);                         // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	mb.addHreg(201,0);                         // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	mb.addHreg(202,0);                         // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	mb.addHreg(203,0);                         // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	mb.addHreg(204,0);                         // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	mb.addHreg(205,0);                         // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	mb.addHreg(206,0);                         // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	mb.addHreg(207,0);                         // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
	mb.addHreg(208,0);                         // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	mb.addHreg(209,0);                         // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	mb.addHreg(210,0);                         // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	mb.addHreg(211,0);                         // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	mb.addHreg(212,0);                         // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	mb.addHreg(213,0);                         // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	mb.addHreg(214,0);                         // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	mb.addHreg(215,0);                         // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	mb.addHreg(216,0);                         // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	mb.addHreg(217,0);                         // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
	mb.addHreg(218,0);                         // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	mb.addHreg(219,0);                         // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

	mb.addHreg(220,0);                         // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        OFF - ";
	mb.addHreg(221,0);                         // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        ON  - ";
	mb.addHreg(222,0);                         // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
	mb.addHreg(223,0);                         // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
	mb.addHreg(224,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addHreg(225,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addHreg(226,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addHreg(227,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addHreg(228,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addHreg(229,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addHreg(230,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addHreg(231,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addHreg(232,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addHreg(233,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addHreg(234,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addHreg(235,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addHreg(236,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addHreg(237,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addHreg(238,0);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addHreg(239,0);                         // Aдрес счетчика ошибки ADC0  ток x1 

	mb.addHreg(240,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addHreg(241,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addHreg(242,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addHreg(243,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addHreg(244,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addHreg(245,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addHreg(246,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addHreg(247,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addHreg(248,0);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addHreg(249,0);                         // Aдрес счетчика ошибки ADC2 ток x10

	mb.addHreg(250,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addHreg(251,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addHreg(252,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineL                                    OFF - ";
	mb.addHreg(253,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addHreg(254,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal mag radio                                OFF - ";
	mb.addHreg(255,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal mag phone                                OFF - ";
	mb.addHreg(256,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal GGS                                      OFF - ";
	mb.addHreg(257,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addHreg(258,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addHreg(259,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal GGS                                      ON  - ";

	mb.addHreg(260,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineL                                    ON  - ";
	mb.addHreg(261,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addHreg(262,0);                         // Aдрес счетчика ошибки "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addHreg(263,0);                         // Aдрес счетчика ошибки "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addHreg(264,0);                         // Aдрес счетчика ошибки "Test microphone PTT  (CTS)                                  OFF - ";
	mb.addHreg(265,0);                         // Aдрес счетчика ошибки "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addHreg(266,0);                         // Aдрес счетчика ошибки "Test microphone PTT  (CTS)                                  ON  - ";
	mb.addHreg(267,0);                         // Aдрес счетчика ошибки "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addHreg(268,0);                         // Aдрес счетчика ошибки "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addHreg(269,0);                         // Aдрес счетчика ошибки Длительность регулировки яркости

	mb.addHreg(270,0);                         // Aдрес счетчика ошибки "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addHreg(271,0);                         // Aдрес счетчика ошибки "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addHreg(272,0);                         // Aдрес счетчика ошибки "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addHreg(273,0);                         // Aдрес счетчика ошибки "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addHreg(274,0);                         // Aдрес счетчика ошибки "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addHreg(275,0);                         // Aдрес счетчика ошибки "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addHreg(276,0);                         // Aдрес счетчика ошибки "Command sensor tangenta nognaja                             OFF - ";
	mb.addHreg(277,0);                         // Aдрес счетчика ошибки "Command sensor tangenta nognaja                             ON  - ";
	mb.addHreg(278,0);                         // Aдрес счетчика ошибки "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addHreg(279,0);                         // Aдрес счетчика ошибки "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addHreg(280,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addHreg(281,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addHreg(282,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal LineL                                    OFF - ";
	mb.addHreg(283,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal LineR                                    OFF - ";
	mb.addHreg(284,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag radio                                OFF - ";
	mb.addHreg(285,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag phone                                OFF - ";
	mb.addHreg(286,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal GGS                                      OFF - ";
	mb.addHreg(287,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addHreg(288,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addHreg(289,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal GGS                                      ON  - ";

	mb.addHreg(290,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addHreg(291,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addHreg(292,0);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag phone                                ON  - ";
	mb.addHreg(293,0);                         // Aдрес счетчика  ошибки ADC1 напряжение 12/3 вольт
	mb.addHreg(294,0);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio1
	mb.addHreg(295,0);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio2
	mb.addHreg(296,0);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт ГГС
	mb.addHreg(297,0);                         // Aдрес счетчика  ошибки ADC15 напряжение светодиода 3,6 вольта
	mb.addHreg(298,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag phone                         ON  - ";    
	mb.addHreg(299,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addHreg(300,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addHreg(301,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addHreg(302,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addHreg(303,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addHreg(304,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addHreg(305,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addHreg(306,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addHreg(307,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(308,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(309,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addHreg(310,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addHreg(311,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addHreg(312,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addHreg(313,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addHreg(314,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addHreg(315,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addHreg(316,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addHreg(317,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(318,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(319,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addHreg(320,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addHreg(321,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addHreg(322,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineL                             OFF - ";
	mb.addHreg(323,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineR                             OFF - ";
	mb.addHreg(324,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addHreg(325,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addHreg(326,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GGS                               OFF - ";
	mb.addHreg(327,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addHreg(328,0);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addHreg(329,0);                         // Aдрес счетчика ошибки Код регулировки яркости                             // 

	mb.addHreg(330,0);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addHreg(331,0);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag radio                             ON  - ";
	mb.addHreg(332,0);                         // Aдрес счетчика ошибки "Test GGS    ** Signal mag radio                             ON  - ";


	
	// ++++++++++++++++++++++ Регистры хранения данных при проверке модулей ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	mb.addHreg(400,0);                         // Aдрес напряжение ADC0  ток x1 
	mb.addHreg(401,0);                         // Aдрес напряжение ADC1 напряжение 12/3 вольт
	mb.addHreg(402,0);                         // Aдрес напряжение ADC2 ток x10
	mb.addHreg(403,0);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт Radio1
	mb.addHreg(404,0);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт Radio2
	mb.addHreg(405,0);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт ГГС
	mb.addHreg(406,0);                         // Aдрес напряжение ADC15 напряжение светодиода 3,6 вольта
	mb.addHreg(407,0);                         // Aдрес 
	mb.addHreg(408,0);                         // Aдрес 
	mb.addHreg(409,0);                         // Aдрес  

	mb.addHreg(410,0);                         // Aдрес счетчика 
	mb.addHreg(411,0);                         // Aдрес счетчика  
	mb.addHreg(412,0);                         // Aдрес счетчика  
	mb.addHreg(413,0);                         // Aдрес счетчика  
	mb.addHreg(414,0);                         // Aдрес счетчика  
	mb.addHreg(415,0);                         // Aдрес счетчика  
	mb.addHreg(416,0);                         // Aдрес счетчика  
	mb.addHreg(417,0);                         // Aдрес счетчика  
	mb.addHreg(418,0);                         // Aдрес счетчика  
	mb.addHreg(419,0);                         // Aдрес счетчика  

	mb.addHreg(420,0);                         // Aдрес  ;
	mb.addHreg(421,0);                         // Aдрес  ;
	mb.addHreg(422,0);                         // Aдрес  ;
	mb.addHreg(423,0);                         // Aдрес  ;
	mb.addHreg(424,0);                         // Aдрес данных измерения "Test headset instructor ** Signal LineL                     ON  - ";
	mb.addHreg(425,0);                         // Aдрес данных измерения "Test headset instructor ** Signal LineR                     ON  - ";   
	mb.addHreg(426,0);                         // Aдрес данных измерения "Test headset instructor ** Signal Mag phone                 ON  - ";
	mb.addHreg(427,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                     ON  - ";
	mb.addHreg(428,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                     ON  - ";  
	mb.addHreg(429,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	mb.addHreg(430,0);                         // Aдрес данных измерения "Test headset instructor ** Signal FrontL                    OFF - ";
	mb.addHreg(431,0);                         // Aдрес данных измерения "Test headset instructor ** Signal FrontR                    OFF - ";
	mb.addHreg(432,0);                         // Aдрес данных измерения "Test headset instructor ** Signal LineL                     OFF - ";
	mb.addHreg(433,0);                         // Aдрес данных измерения "Test headset instructor ** Signal LineR                     OFF - ";
	mb.addHreg(434,0);                         // Aдрес данных измерения "Test headset instructor ** Signal mag radio                 OFF - "; 
	mb.addHreg(435,0);                         // Aдрес данных измерения "Test headset instructor ** Signal mag phone                 OFF - ";
	mb.addHreg(436,0);                         // Aдрес данных измерения "Test headset instructor ** Signal GGS                       OFF - ";
	mb.addHreg(437,0);                         // Aдрес данных измерения "Test headset instructor ** Signal GG Radio1                 OFF - ";
	mb.addHreg(438,0);                         // Aдрес данных измерения "Test headset instructor ** Signal GG Radio2                 OFF - ";
	mb.addHreg(439,0);                         // Aдрес данных измерения ADC0  ток x1 

	mb.addHreg(440,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal FrontL                    OFF - ";
	mb.addHreg(441,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal FrontR                    OFF - ";
	mb.addHreg(442,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                     OFF - "; 
	mb.addHreg(443,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                     OFF - ";
	mb.addHreg(444,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	mb.addHreg(445,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal mag phone                 OFF - ";
	mb.addHreg(446,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GGS                       OFF - "; 
	mb.addHreg(447,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	mb.addHreg(448,0);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	mb.addHreg(449,0);                         // Aдрес данных измерения ADC2 ток x10

	mb.addHreg(450,0);                         // Aдрес данных измерения "Test MTT ** Signal FrontL                                   OFF - ";
	mb.addHreg(451,0);                         // Aдрес данных измерения "Test MTT ** Signal FrontR                                   OFF - ";
	mb.addHreg(452,0);                         // Aдрес данных измерения "Test MTT ** Signal LineL                                    OFF - ";
	mb.addHreg(453,0);                         // Aдрес данных измерения "Test MTT ** Signal LineR                                    OFF - "; 
	mb.addHreg(454,0);                         // Aдрес данных измерения "Test MTT ** Signal mag radio                                OFF - ";
	mb.addHreg(455,0);                         // Aдрес данных измерения "Test MTT ** Signal mag phone                                OFF - ";
	mb.addHreg(456,0);                         // Aдрес данных измерения "Test MTT ** Signal GGS                                      OFF - ";
	mb.addHreg(457,0);                         // Aдрес данных измерения "Test MTT ** Signal GG Radio1                                OFF - ";
	mb.addHreg(458,0);                         // Aдрес данных измерения "Test MTT ** Signal GG Radio2                                OFF - "; 
	mb.addHreg(459,0);                         // Aдрес данных измерения "Test MTT ** Signal GGS                                      ON  - ";

	mb.addHreg(460,0);                         // Aдрес данных измерения "Test MTT ** Signal LineL                                    ON  - ";
	mb.addHreg(461,0);                         // Aдрес данных измерения "Test MTT ** Signal LineR                                    ON  - ";  
	mb.addHreg(462,0);                         // Aдрес данных измерения "Test MTT ** Signal Mag phone                                ON  - ";
	mb.addHreg(463,0);                         // Aдрес данных измерения "Test MTT PTT    (CTS)                                       OFF - ";
	mb.addHreg(464,0);                         // 
	mb.addHreg(465,0);                         // Aдрес данных измерения "Test MTT PTT    (CTS)                                       ON  - ";
	mb.addHreg(466,0);                         // 
	mb.addHreg(467,0);                         // Aдрес данных измерения "Test MTT HangUp (DCD)                                       OFF - ";
	mb.addHreg(468,0);                         // Aдрес данных измерения "Test MTT HangUp (DCD)                                       ON  - ";
	mb.addHreg(469,0);                         // Длительность импульса регулировки яркости дисплея

	mb.addHreg(470,0);                         // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	mb.addHreg(471,0);                         // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	mb.addHreg(472,0);                         // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	mb.addHreg(473,0);                         // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	mb.addHreg(474,0);                         // Aдрес данных измерения "Command sensor tangenta ruchnaja                            OFF - ";
	mb.addHreg(475,0);                         // Aдрес данных измерения "Command sensor tangenta ruchnaja                            ON  - ";
	mb.addHreg(476,0);                         // Aдрес данных измерения "Command sensor tangenta nognaja                             OFF - ";
	mb.addHreg(477,0);                         // Aдрес данных измерения "Command sensor tangenta nognaja                             ON  - ";
	mb.addHreg(478,0);                         // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                          OFF - ";
	mb.addHreg(479,0);                         // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                          ON  - ";

	mb.addHreg(480,0);                         // Aдрес данных измерения "Test GGS ** Signal FrontL                                   OFF - ";
	mb.addHreg(481,0);                         // Aдрес данных измерения "Test GGS ** Signal FrontR                                   OFF - ";
	mb.addHreg(482,0);                         // Aдрес данных измерения "Test GGS ** Signal LineL                                    OFF - ";
	mb.addHreg(483,0);                         // Aдрес данных измерения "Test GGS ** Signal LineR                                    OFF - ";
	mb.addHreg(484,0);                         // Aдрес данных измерения "Test GGS ** Signal mag radio                                OFF - ";
	mb.addHreg(485,0);                         // Aдрес данных измерения "Test GGS ** Signal mag phone                                OFF - ";
	mb.addHreg(486,0);                         // Aдрес данных измерения "Test GGS ** Signal GGS                                      OFF - ";
	mb.addHreg(487,0);                         // Aдрес данных измерения "Test GGS ** Signal GG Radio1                                OFF - ";
	mb.addHreg(488,0);                         // Aдрес данных измерения "Test GGS ** Signal GG Radio2                                OFF - ";
	mb.addHreg(489,0);                         // Aдрес данных измерения "Test GGS ** Signal GGS                                      ON  - ";

	mb.addHreg(490,0);                         // Aдрес данных измерения "Test GGS ** Signal FrontL                                   ON  - ";
	mb.addHreg(491,0);                         // Aдрес данных измерения "Test GGS ** Signal FrontR                                   ON  - ";
	mb.addHreg(492,0);                         // Aдрес данных измерения "Test GGS ** Signal mag phone                                ON  - ";
	mb.addHreg(493,0);                         // Aдрес данных измерения ADC1 напряжение 12/3 вольт
	mb.addHreg(494,0);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio1
	mb.addHreg(495,0);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio2
	mb.addHreg(496,0);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт ГГС
	mb.addHreg(497,0);                         // Aдрес данных измерения ADC15 напряжение светодиода 3,6 вольта
	mb.addHreg(498,0);                         // Aдрес данных измерения "Test Microphone ** Signal mag phone                         ON  - "; 
	mb.addHreg(499,0);                         // Aдрес данных измерения "Test Microphone ** Signal LineL                             ON  - ";   

	mb.addHreg(500,0);                         // Aдрес данных измерения "Test Radio1 ** Signal FrontL                                OFF - ";
	mb.addHreg(501,0);                         // Aдрес данных измерения "Test Radio1 ** Signal FrontR                                OFF - ";
	mb.addHreg(502,0);                         // Aдрес данных измерения "Test Radio1 ** Signal LineL                                 OFF - ";
	mb.addHreg(503,0);                         // Aдрес данных измерения "Test Radio1 ** Signal LineR                                 OFF - ";
	mb.addHreg(504,0);                         // Aдрес данных измерения "Test Radio1 ** Signal mag radio                             OFF - ";
	mb.addHreg(505,0);                         // Aдрес данных измерения "Test Radio1 ** Signal mag phone                             OFF - ";
	mb.addHreg(506,0);                         // Aдрес данных измерения "Test Radio1 ** Signal GGS                                   OFF - ";
	mb.addHreg(507,0);                         // Aдрес данных измерения "Test Radio1 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(508,0);                         // Aдрес данных измерения "Test Radio1 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(509,0);                         // Aдрес данных измерения "Test Radio1 ** Signal Radio1                                ON  - ";

	mb.addHreg(510,0);                         // Aдрес данных измерения "Test Radio2 ** Signal FrontL                                OFF - ";
	mb.addHreg(511,0);                         // Aдрес данных измерения "Test Radio2 ** Signal FrontR                                OFF - ";
	mb.addHreg(512,0);                         // Aдрес данных измерения "Test Radio2 ** Signal LineL                                 OFF - ";
	mb.addHreg(513,0);                         // Aдрес данных измерения "Test Radio2 ** Signal LineR                                 OFF - ";
	mb.addHreg(514,0);                         // Aдрес данных измерения "Test Radio2 ** Signal mag radio                             OFF - ";
	mb.addHreg(515,0);                         // Aдрес данных измерения "Test Radio2 ** Signal mag phone                             OFF - ";
	mb.addHreg(516,0);                         // Aдрес данных измерения "Test Radio2 ** Signal GGS                                   OFF - ";
	mb.addHreg(517,0);                         // Aдрес данных измерения "Test Radio2 ** Signal GG Radio1                             OFF - ";
	mb.addHreg(518,0);                         // Aдрес данных измерения "Test Radio2 ** Signal GG Radio2                             OFF - ";
	mb.addHreg(519,0);                         // Aдрес данных измерения "Test Radio2 ** Signal Radio2                                ON  - ";

	mb.addHreg(520,0);                         // Aдрес данных измерения "Test Microphone ** Signal FrontL                            OFF - ";
	mb.addHreg(521,0);                         // Aдрес данных измерения "Test Microphone ** Signal FrontR                            OFF - ";
	mb.addHreg(522,0);                         // Aдрес данных измерения "Test Microphone ** Signal LineL                             OFF - ";
	mb.addHreg(523,0);                         // Aдрес данных измерения "Test Microphone ** Signal LineR                             OFF - ";
	mb.addHreg(524,0);                         // Aдрес данных измерения "Test Microphone ** Signal mag radio                         OFF - ";
	mb.addHreg(525,0);                         // Aдрес данных измерения "Test Microphone ** Signal mag phone                         OFF - ";
	mb.addHreg(526,0);                         // Aдрес данных измерения "Test Microphone ** Signal GGS                               OFF - ";
	mb.addHreg(527,0);                         // Aдрес данных измерения "Test Microphone ** Signal GG Radio1                         OFF - ";
	mb.addHreg(528,0);                         // Aдрес данных измерения "Test Microphone ** Signal GG Radio2                         OFF - ";
	mb.addHreg(529,0);                         // Код регулировки яркости дисплея
	mb.addHreg(530,0);                         // Aдрес данных измерения "Test Radio1 ** Signal mag radio                             ON  - ";
	mb.addHreg(531,0);                         // Aдрес данных измерения "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	mb.addHreg(532,0);                         // Aдрес данных измерения "Test GGS ** Signal mag radio   


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
// Поиск ком порта
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
	//wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
	digitalWrite(ledPin13,LOW);
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
}
void clear_serial()
{
  if (Serial.available())                             // есть что-то проверить? Есть данные в буфере?
		  {

			while (Serial.available())
				{
					 Serial.read();
				}
		   }
}
void clear_serial2()
{
  if (Serial2.available())                             // есть что-то проверить? Есть данные в буфере?
		  {

			while (Serial2.available())
				{
					 Serial2.read();
				}
		   }
}

void clear_serial1()
{
  if (Serial1.available())                             // есть что-то проверить? Есть данные в буфере?
		  {

			while (Serial1.available())
				{
					 Serial1.read();
				}
		   }
}
void clear_serial3()
{
  if (Serial3.available())                             // есть что-то проверить? Есть данные в буфере?
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
	wdt_disable(); // бесполезная строка до которой не доходит выполнение при bootloop
	Wire.begin();
	if (!RTC.begin())                               // Настройка часов 
		{
			Serial.println("RTC failed");
			while(1);
		};
	setup_mcp();                                    // Настроить порты расширения  
	mcp_Analog.digitalWrite(DTR, HIGH);             // Разрешение вывода (обмена)информации с Камертоном
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, HIGH); 
	Serial.begin(9600);                             // Подключение к USB ПК
	Serial1.begin(115200);                          // Подключение к звуковому модулю Камертон
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
	pinMode(kn1Nano, OUTPUT);                        // Назначение кнопок управления Nano генератор качения
	pinMode(kn2Nano, OUTPUT);                        // Назначение кнопок управления Nano генератор 1000 гц
	pinMode(kn3Nano, OUTPUT);                        // Назначение кнопок управления Nano генератор 2000 гц
	pinMode(InNano12, INPUT);                        // Назначение входов - индикация генератор 1000 или 2000 гц
	pinMode(InNano13, INPUT);                        // Назначение входов - индикация генератор качения 
 
	digitalWrite(kn1Nano, LOW);
	digitalWrite(kn2Nano, HIGH);
	digitalWrite(kn3Nano, HIGH);

//	set_serial();                                    // Поиск СОМ порта подключения к компьютеру

	//set_serial();                                    // Поиск СОМ порта подключения к компьютеру
	AD9850.reset();                                  //reset module
	delay(500);
	AD9850.powerDown();                              //set signal output to LOW
	delay(100);
	AD9850.set_frequency(0,0,1000);                   //set power=UP, phase=0, 1kHz frequency
	delay(1000); 

	// DateTime set_time = DateTime(15, 6, 15, 10, 51, 0); // Занести данные о времени в строку "set_time"
	// RTC.adjust(set_time);                                // Записа
	serial_print_date();
	Serial.println(" ");

	setup_resistor();                               // Начальные установки резистора

	setup_regModbus();                              // Настройка регистров MODBUS

	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 196 Изменять в реальной схеме
	regs_out[2]= 0x7F;                              // 127 Изменять в реальной схеме

	//mb.Hreg(40004+buffer,Serial1.read());

	 mb.Coil(21,0);                              // XP2-2     sensor "Маг."  
	 mb.Coil(22,0);                              // XP5-3     sensor "ГГC."
	 mb.Coil(23,0);                              // XP3-3     sensor "ГГ-Радио1."
	 mb.Coil(24,0);                              // XP4-3     sensor "ГГ-Радио2."
//	 mb.Coil(8,0);                               // Выключить питание Камертон
	UpdateRegs();                                   // Обновить информацию в регистрах

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
  //sd.ls (LS_R | LS_DATE | LS_SIZE);
 

	MsTimer2::set(30, flash_time);                   // 30ms период таймера прерывани
	resistor(1, 200);                                // Установить уровень сигнала
	resistor(2, 200);                                // Установить уровень сигнала
	preob_num_str();
	list_file();                                     // Вывод списка файлов в СОМ порт  
	default_mem_porog();
	prer_Kmerton_On = true;                          // Разрешить прерывания на камертон
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
//	logTime = micros();
	MsTimer2::start();                               // Включить таймер прерывания
	Serial.println(" ");                             //
	Serial.println("System initialization OK!.");    // Информация о завершении настройки
	//wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
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

	//Serial.print(	mb.Hreg(136),HEX);    // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	//Serial.print("--");
	//Serial.println(	mb.Hreg(137),HEX);    // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
 
}
