#include <modbusSlave.h>
#include <modbus.h>
#include <modbusDevice.h>
#include <Arduino.h>

//#define  ledPin13  13       // Назначение светодиодов на плате
#define T35  5

modbusSlave::modbusSlave()
{
}
/*
Set the Serial Baud rate.
Reconfigure the UART for 8 data bits, no parity, and 1 stop bit.
and flush the serial port.
*/
void modbusSlave::setBaud(long baud)
{
	_baud = baud;
	//calculate the time perdiod for 3 characters for the given bps in ms.
	_frameDelay = 24000/_baud;

	Serial.begin(baud);

	// defaults to 8-bit, no parity, 1 stop bit
	//clear parity, stop bits, word length
//	UCSR0C = UCSR0C & B11000001;
//	UCSR0B = UCSR0B & B11111011; 

	//Set word length to 8 bits
//	UCSR0C = UCSR0C | B00000110;

	//No parity
//	UCSR0C = UCSR0C | B00000000;

	//1 Stop bit
//	UCSR0C = UCSR0C | B00000100;

	Serial.flush();
}

void modbusSlave::setSerial(byte serno, long baud) 
{
	 _baud = baud;
	 _serno = serno;
	//calculate the time perdiod for 3 characters for the given bps in ms.
	_frameDelay = 24000/_baud;

  switch( _serno ) 
  {
	#if defined(UBRR1H)
	  case 1:
		port = &Serial1;
		break;
	#endif

	#if defined(UBRR2H)
	  case 2:
		port = &Serial2;
		break;
	#endif

	#if defined(UBRR3H)
	  case 3:
		port = &Serial3;
		break;
	#endif
	  case 0:
	  default:
		port = &Serial;
		break;
  }
   u8lastRec = 0;
   port->begin(baud);
   port->flush();
 }

/*
Retrieve the serial baud rate
*/
long modbusSlave::getBaud(void)
{
	return(_baud);
}

/*
Generates the crc for the current message in the buffer.
*/

void modbusSlave::calcCrc(void)
{
	byte	CRCHi = 0xFF,
			CRCLo = 0x0FF,
			Index,
			msgLen,
			*msgPtr;

	msgLen = _len-2;
	msgPtr = _msg;

	while(msgLen--)
	{
		Index = CRCHi ^ *msgPtr++;
		CRCHi = CRCLo ^ _auchCRCHi[Index];
		CRCLo = _auchCRCLo[Index];
	}
	_crc = (CRCHi << 8) | CRCLo;
}

/*
  Checks the UART for query data
*/
void modbusSlave::checkSerial(void)
{
	//while there is more data in the UART than when last checked
	while(port->available()> _len)
		{
			//update the incoming query message length
			_len = port->available();
			//Wait for 3 bytewidths of data (SOM/EOM)
	        //delayMicroseconds(RTUFRAMETIME);
			delay(_frameDelay);
			//Check the UART again
		}
}

/*
Copies the contents of the UART to a buffer
*/
void modbusSlave::serialRx(void)
{
	byte i;

	//allocate memory for the incoming query message
	_msg = (byte*) malloc(_len);

		//copy the query byte for byte to the new buffer
		for (i=0 ; i < _len ; i++)
		{
			_msg[i] = port->read();
		}
}

/*
Generates a query reply message for Digital In/Out status update queries.
*/
void modbusSlave::getDigitalStatus(byte funcType, word startreg, word numregs)
{
	//************************* »сходный код *************************************
	//initialize the bit counter to 0
	byte bitn =0;
	
	//if the function is to read digital inputs then add 10001 to the start register
	// ≈сли функци€ €вл€етс€ чтение цифровых входов затем добавить 10001 до начального регистра
	//else add 1 to the start register
	if(funcType == READ_DISCRETE_INPUT)
		startreg += 10001;
	else
		startreg += 1;

	//determine the message length
	//определить длину сообщени€
	//for each group of 8 registers the message length increases by 1
	//дл€ каждой группы из 8 регистров длины сообщени€ увеличиваетс€ на 1
	_len = numregs/8;
	//if there is at least one incomplete byte's worth of data
	//≈сли там стоит по крайней мере одного неполного байта данных
	//then add 1 to the message length for the partial byte.
	//затем добавить 1 к длине сообщени€ дл€ частичного байта.
	if(numregs%8)
		_len++;
	//allow room for the Device ID byte, Function type byte, data byte count byte, and crc word
	//чтобы места дл€ байта идентификатор устройства, байт “ип функции, количество байт данных, байт, а слово CRC
	_len +=5;

	//allocate memory of the appropriate size for the message
	//выдел€ет пам€ть соответствующего размера дл€ сообщени€
	_msg = (byte *) malloc(_len);

	//write the slave device ID
	//написать ID ведомого устройства
	_msg[0] = _device->getId();
	//write the function type
	//написать функцию типа
	_msg[1] = funcType;
	//set the data byte count
	//установить количество байт данных
	_msg[2] = _len-5;

	//For the quantity of registers queried
	//ѕо количеству регистров запросы
	while(numregs--)
	{
		//if a value is found for the current register, set bit number bitn of msg[3]
		//else clear it
		if(_device->get(startreg))
			bitSet(_msg[3], bitn);
		else
			bitClear(_msg[3], bitn);
		//increment the bit index
		bitn++;
		//increment the register
		startreg++;
	}
	
	//generate the crc for the query reply and append it
	this->calcCrc();
	_msg[_len - 2] = _crc >> 8;
	_msg[_len - 1] = _crc & 0xFF;


//********************************************************************************
	/*  // –едактированный код


	//initialize the bit counter to 0
	//byte bitn =0;
	
	//if the function is to read digital inputs then add 10001 to the start register
	//else add 1 to the start register
	if(funcType == READ_DISCRETE_INPUT)
		startreg += 10001;
	else
		startreg += 1;


	uint8_t u8currentRegister, u8currentBit, u8bytesno, u8bitsno,  u8BufferSize;
  unsigned int u16currentCoil, u16coil;


	 // get the first and last coil from the message
  unsigned int u16StartCoil = word( _msg[2],  _msg[3] );
  unsigned int u16Coilno    = word( _msg[4],  _msg[5] );

  // put the number of bytes in the outcoming message
  u8bytesno = (uint8_t) (u16Coilno / 8);
  if (u16Coilno % 8 != 0) u8bytesno ++;
  _msg[ 2 ]  = u8bytesno;
  u8BufferSize  = 3;

  // read each coil from the register map and put its value inside the outcoming message  
  u8bitsno = 0;

  for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++) 
  {
	u16coil = u16StartCoil + u16currentCoil;
	u8currentRegister = (uint8_t) (u16coil / 16);
	u8currentBit = (uint8_t) (u16coil % 16);

   // bitWrite( _msg[ u8BufferSize ],  u8bitsno,  bitRead( regs[ u8currentRegister ], u8currentBit ) );
	  bitWrite( _msg[ u8BufferSize ],  u8bitsno,  _device->get(startreg + u16currentCoil));
	//   bitWrite( _msg[ u8BufferSize ],  u8bitsno,  1);
	u8bitsno ++;

	if (u8bitsno > 7)
	{
	  u8bitsno = 0;
	  u8BufferSize++;
	}
  } 


	//determine the message length
	//for each group of 8 registers the message length increases by 1
	_len = numregs/8;
	//if there is at least one incomplete byte's worth of data
	//then add 1 to the message length for the partial byte.
	if(numregs%8)
		_len++;
	//allow room for the Device ID byte, Function type byte, data byte count byte, and crc word
	_len +=5;

	//allocate memory of the appropriate size for the message
	_msg = (byte *) malloc(_len);

	//write the slave device ID
	_msg[0] = _device->getId();
	//write the function type
	_msg[1] = funcType;
	//set the data byte count
	_msg[2] = _len-5;

	//For the quantity of registers queried
	//while(numregs--)
	//{
	//	//if a value is found for the current register, set bit number bitn of msg[3]
	//	//else clear it
	//	if(_device->get(startreg))
	//		bitSet(_msg[3], bitn);
	//	else
	//		bitClear(_msg[3], bitn);
	//	//increment the bit index
	//	bitn++;
	//	//increment the register
	//	startreg++;
	//}
	
	//generate the crc for the query reply and append it
	this->calcCrc();
	_msg[_len - 2] = _crc >> 8;
	_msg[_len - 1] = _crc & 0xFF;





	*/
}
void modbusSlave::getImputRegister(byte funcType, word startreg, word numregs)
{
	word val;
	word i = 0;

	//if the function is to read analog inputs then add 30001 to the start register
	//else add 40001 to the start register
	if(funcType == READ_INPUT_REGISTER)
		startreg += 30001;
	else
		startreg += 40001;

	//calculate the query reply message length
	//for each register queried add 2 bytes
	_len = numregs * 2;
	//allow room for the Device ID byte, Function type byte, data byte count byte, and crc word
	_len += 5;

	//allocate memory for the query response
	_msg = (byte *) malloc(_len);

	//write the device ID
	_msg[0] = _device->getId();
	//write the function type
	_msg[1] = funcType;
	//set the data byte count
	_msg[2] = _len - 5;

	//for each register queried
	while(numregs--)
	{
		//retrieve the value from the register bank for the current register
		val = _device->get(startreg+i);
		//write the high byte of the register value
		_msg[3 + i * 2]  = val >> 8;
		//write the low byte of the register value
		_msg[4 + i * 2] = val & 0xFF;
		//increment the register
		i++;
	}

	//generate the crc for the query reply and append it
	this->calcCrc();
	_msg[_len - 2] = _crc >> 8;
	_msg[_len - 1] = _crc & 0xFF;
}
void modbusSlave::setStatus(byte funcType, word reg, word val)
{
	//Set the query response message length
	//Device ID byte, Function byte, Register byte, Value byte, CRC word
	_len = 8;
	//allocate memory for the message buffer.
	_msg = (byte *) malloc(_len);


	//write the device ID
	_msg[0] = _device->getId();
	//if the function type is a digital write
	if(funcType == WRITE_COIL)
	{
		//Add 1 to the register value and set it's value to val
		_device->set(reg + 1, val);
		//write the function type to the response message
		_msg[1] = WRITE_COIL;
	}
	else
	{
		//else add 40001 to the register and set it's value to val
		_device->set(reg + 40001, val);

		//write the function type of the response message
		_msg[1] = WRITE_REGISTER;
	}
	
	//write the register number high byte value
	_msg[2] = reg >> 8;
	//write the register number low byte value
	_msg[3] = reg & 0xFF;
	//write the control value's high byte
	_msg[4] = val >> 8;
	//write the control value's low byte
	_msg[5] = val & 0xFF;

	//calculate the crc for the query reply and append it.
	this->calcCrc();
	_msg[_len - 2]= _crc >> 8;
	_msg[_len - 1]= _crc & 0xFF;
}
void modbusSlave::setMULTIPLE_Status(byte funcType, word reg, word val)
{
	// int8_t Modbus::process_FC15( unsigned int *regs, uint8_t u8size )
  uint8_t u8frameByte, u8bitsno;
  //uint8_t u8currentRegister, u8currentBit;
  unsigned int u16currentCoil, u16coil;
  boolean bTemp;
  reg += 1;                                                  // ѕервый адрес регистров
   unsigned int u16StartCoil = word(  _msg[2],  _msg[3] );   // Ќомер первого регистра
  unsigned int u16Coilno = word(  _msg[4],  _msg[5] );       //  оличество регистров

  // read each coil from the register map and put its value inside the outcoming message  
  // читать каждый бит от регистра  и положить его значение внутри исход€щего сообщени€
 
  u8bitsno = 0;
  u8frameByte = 7;
  for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++) 
	  {

		u16coil = u16StartCoil + u16currentCoil;
		//u8currentRegister = (uint8_t) (u16coil / 16);
		//u8currentBit = (uint8_t) (u16coil % 16);

		bTemp = bitRead( _msg[ u8frameByte ], u8bitsno );

		_device->set( u16coil,  bTemp); 
		 u8bitsno ++;

		if (u8bitsno > 7) 
			{
			  u8bitsno = 0;
			  u8frameByte++;
			}
	  } 


	//Set the query response message length
	//Device ID byte, Function byte, Register byte, Value byte, CRC word
	_len = 8;
	//allocate memory for the message buffer.
	_msg = (byte *) malloc(_len);

	
	_msg[0] = _device->getId();  //write the device ID
		
	_msg[1] = WRITE_MULTIPLE_COILS;  //write the function type of the response message
	
	_msg[2] = reg >> 8; //write the register number high byte value
	
	_msg[3] = reg & 0xFF; //write the register number low byte value
	
	_msg[4] = val >> 8; 	//write the control value's high byte
	
	_msg[5] = val & 0xFF; //write the control value's low byte


	//calculate the crc for the query reply and append it.
	this->calcCrc();
	_msg[_len - 2]= _crc >> 8;
	_msg[_len - 1]= _crc & 0xFF;
	

}
void modbusSlave::setMULTIPLE_REGISTERS(byte funcType, word reg, word val)
{
	//  uint8_t u8func      = _msg[1];  // get the original FUNC code
	//  uint8_t u8StartAdd  = _msg[2] << 8 | _msg[3];
	  uint8_t u8regsno    = _msg[4] << 8 | _msg[5];
	  uint8_t i;
	  byte BYTE_CNT       = 6;
	  unsigned int temp;

	   for (i = 0; i < u8regsno; i++)
		   {
			temp = word(_msg[ (BYTE_CNT + 1) + i*2 ],	_msg[ (BYTE_CNT + 2) + i*2 ]);
			_device->set(reg + 40001 + i, temp);
		   }

	//Set the query response message length
	//Device ID byte, Function byte, Register byte, Value byte, CRC word
	_len = 8;
	//allocate memory for the message buffer.
	_msg = (byte *) malloc(_len);

	
	_msg[0] = _device->getId();  //write the device ID
		
	_msg[1] = WRITE_MULTIPLE_REGISTERS;  //write the function type of the response message
	
	_msg[2] = reg >> 8; //write the register number high byte value
	
	_msg[3] = reg & 0xFF; //write the register number low byte value
	
	_msg[4] = val >> 8; 	//write the control value's high byte
	
	_msg[5] = val & 0xFF; //write the control value's low byte

	//calculate the crc for the query reply and append it.
	this->calcCrc();
	_msg[_len - 2]= _crc >> 8;
	_msg[_len - 1]= _crc & 0xFF;

}

// uint8_t modbusSlave::run(void)
int modbusSlave::run(void)
//void modbusSlave::run(void)
{

//	byte deviceId;
	byte funcType;
	word field1;
	word field2;
	
	//initialize mesasge length
	_len = 0;

	//check for data in the recieve buffer
	this->checkSerial();

	//если нет ничего в буфере возврат
	if(_len == 0)
		{
			return 0;
		}

	 // проверить “35 после завершени€ кадра или еще  нет окончани€ кадра
  if (_len != u8lastRec) 
	  {
		u8lastRec = _len;
		u32time = millis() + T35;
		return 0;   //return 0;
	  }
  if (millis() < u32time) return 0;            // ≈сли интервал меньше заданного - возврат

  u8lastRec = 0;                                     //     
  //int8_t i8state = getRxBuffer();
 

  this->serialRx();

 

  int8_t i8state = _len;
  u8lastError = i8state;
  if (i8state < 7) return i8state;  //  return i8state;  

	//retrieve the query message from the serial uart
//	this->serialRx();
	
	//if the message id is not 255, and
	// and device id does not match bail
	if( (_msg[0] != 0xFF) && (_msg[0] != _device->getId()) )
		{
			return 0;
		}
	//calculate the checksum of the query message minus the checksum it came with.
	this->calcCrc();

	//if the checksum does not match, ignore the message
	if ( _crc != ((_msg[_len - 2] << 8) + _msg[_len - 1]))
		return 0;
	
	//copy the function type from the incoming query
	funcType = _msg[1];

	//copy field 1 from the incoming query
	field1	= (_msg[2] << 8) | _msg[3];

	//copy field 2 from the incoming query
	field2  = (_msg[4] << 8) | _msg[5];
	
	//free the allocated memory for the query message
	free(_msg);
	//reset the message length;
	_len = 0;

	//generate query response based on function type
	switch(funcType)
		{
			case READ_DISCRETE_INPUT:
				this->getDigitalStatus(funcType, field1, field2);
				break;
			case READ_COILS:
				this->getDigitalStatus(funcType, field1, field2);
				break;
			case READ_INPUT_REGISTER:
				this->getImputRegister(funcType, field1, field2);
				break;
			case READ_REGISTERS:
				this->getImputRegister(funcType, field1, field2);
				break;
			case WRITE_COIL:
				this->setStatus(funcType, field1, field2);
				break;
			case WRITE_REGISTER:
				this->setStatus(funcType, field1, field2);
				break;
			case WRITE_MULTIPLE_COILS:
				this->setMULTIPLE_Status(funcType, field1, field2);
				break;
			case WRITE_MULTIPLE_REGISTERS:
				this->setMULTIPLE_REGISTERS(funcType, field1, field2);
				break;
			default:
				return 0;
				break;
		}

	//if a reply was generated
	if(_len)
		{
			int i;
			//send the reply to the serial UART
			//Senguino doesn't support a bulk serial write command....
			for(i = 0 ; i < _len ; i++)
				port->write(_msg[i]);
			//free the allocated memory for the reply message
			free(_msg);
			//reset the message length
			_len = 0;
			port->flush();
		}
}
