#include "cmsis_os.h"
#include "as5047p.h"

// **************************************************************************
// the defines

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

uint16_t AS5047P_readPosition(AS5047D_Handle handle){
	AS5047P_readSpi(handle, (uint16_t)0xFFFF);

	uint16_t data = 0;

	data = AS5047P_readSpi(handle, (uint16_t)0xFFFF);

	return data;

}


uint16_t AS5047P_readSpi(AS5047D_Handle handle, uint16_t command){
	HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);

	uint16_t buff = 0;
  //may want a delay between the GPIO operation and the SPI operation, (and after SPI operation too)
  //check setup and hold time
	HAL_SPI_TransmitReceive(handle->spiHandle, (uint16_t*)(&command), (uint16_t*)(&buff), 1, 500);

	HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);

	return buff;
}
