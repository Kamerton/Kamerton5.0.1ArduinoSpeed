#include <modbusDevice.h>

modbusDevice::modbusDevice(void)
{
	_id = 0;
}

void modbusDevice::setId(byte id)
{
	_id=id;
}

byte modbusDevice::getId(void)
{
	return(_id);
}
