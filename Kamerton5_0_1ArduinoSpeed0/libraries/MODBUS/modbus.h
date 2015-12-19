#include <stdint.h>
#ifndef _MODBUSPROTOCOL
#define _MODBUSPROTOCOL

//Maximum device list for network
#define DEVMAX		10
//Maximum control register que size
#define QUEMAX		10
//Maximum serial wait in micro seconds
#define SERIALMAXDELAY	300
#define SERIALBAUD		9600
//the total silence time needed to signify an EOM or SOM in RTU mode

//Modbus function codes
#define READ_COILS          	    0x01  //  00001-09999  Digital Outputs, A master device can read and write to these registers
                                          //  00001-09999  ÷ифровые выходы, ведущее устройство может писать в эти регистры
#define READ_DISCRETE_INPUT		    0x02  //  10001-19999  Digital Inputs, A master device can only read the values from these registers
                                          //  10001-19999  ¬едущее устройство может только читать значени€ из этих регистров
#define READ_REGISTERS       	    0x03  //  40001-49999  A master device can read and write to these registers
                                          //  40001-49999  ¬едущее устройство может читать и писать в эти регистры
#define READ_INPUT_REGISTER		    0x04  //  30001-39999  A master device can only read the values from these registers 
                                          //  30001-39999  ¬едущее устройство может только читать значени€ из этих регистров
#define WRITE_COIL                  0x05  //  00001-09999  Write Single Coil Force Single Coil
                                          //  00001-09999  «апись одного бита (Force Single Coil).
#define WRITE_REGISTER              0x06  //  40001-49999  Write Single Register Preset Single Register
                                          //  40001-49999  значени€ в один регистр хранени€ (Preset Single Register).
#define WRITE_MULTIPLE_COILS        0x0F  //  15 (0F hex) Write Multiple Coils Force Multiple Coils
                                          //    запись значений в несколько регистров флагов (Force Multiple Coils)
#define WRITE_MULTIPLE_REGISTERS    0x10  //  16 (10 Hex) Write Multiple Registers Preset Multiple Registers	



#define RTU 		0x01
#define ASCII		0x02

#define MASTER		0x01
#define SLAVE		0x02

#define DO			0x00
#define DI			0x01
#define AI			0x03
#define AO			0x04

#endif