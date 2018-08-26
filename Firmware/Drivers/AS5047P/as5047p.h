#ifndef _AS5047P_H_
#define _AS5047P_H_

#include "stdbool.h"
#include "stdint.h"

// drivers
#include "stm32f4xx_hal.h"

// Port
typedef SPI_HandleTypeDef* SPI_Handle;
typedef GPIO_TypeDef* GPIO_Handle;
typedef uint16_t GPIO_Number_e;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _AS5047P_Obj_
{
  SPI_Handle       	spiHandle;                  //!< the handle for the serial peripheral interface
  GPIO_Handle      	nCSgpioHandle;              //!< the gpio handle that is connected to the aeat-6012-a06 nCS pin
  GPIO_Number_e    	nCSgpioNumber;               //!< the gpio number that is connected to the aeat-6012-a06 nCS pin
  float				encoder_angle;
  uint16_t			encoder_cnt;
} AS5047P_Obj;

typedef struct _AS5047P_Obj_ *AS5047D_Handle;

// **************************************************************************
// the globals

/* Volatile Register Table (Address)*/
#define NOP 0x0000
#define ERRFL 0x0001
#define PROG 0x0003
#define DIAAGC 0x3FFC
#define MAG 0x3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF


// **************************************************************************
// the function prototypes

extern uint16_t AS5047P_readPosition(AS5047D_Handle handle);
extern uint16_t AS5047P_readSpi(AS5047D_Handle handle, uint16_t command);
extern uint16_t AS5047P_readAngle(AS5047D_Handle handle);
extern bool AS5047P_setZeroPos(AS5047D_Handle handle);

#ifdef __cplusplus
}
#endif // extern "C"

#endif
