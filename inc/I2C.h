/*
 * I2C.h
 *
 *  Created on: 16 окт. 2017 г.
 *      Author: twist
 */

#ifndef I2C_H_
#define I2C_H_
#include "stm32f4xx_i2c.h"

//#define ENABLE_I2C_TIMEOUT

/*
 * TODO:
 * 1) MAKE DESCRIPTIONS GREAT AGAIN!!!
 * 2) Write a methods
 */

/*
 * ******************************************************************
 * method: I2C_start()
 * This function
 *
 * 0-returned value == correct
 * Errors:
 * 			*value == 1:	I2CBusy
 * 			*value == 2:
 * 			*value == 3:
 * 			*value == 4:
 * ******************************************************************
 */
int I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint16_t timeout){
	uint16_t timeout_ = timeout;

	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
#ifdef ENABLE_I2C_TIMEOUT
		if(!(timeout_--)) return 1;
#endif
	}
	timeout_ = timeout;
	I2C_GenerateSTART(I2Cx, ENABLE);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
#ifdef ENABLE_I2C_TIMEOUT
		if(!(timeout_--)) return 2;
#endif
	}
	timeout_ = timeout;
	I2C_Send7bitAddress(I2Cx, address, direction);

	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
#ifdef ENABLE_I2C_TIMEOUT
			if(!(timeout_--)) return 3;
#endif
		}
	} else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
#ifdef ENABLE_I2C_TIMEOUT
			if(!(timeout_--)) return 4;
#endif
		}
	}

	return 0;
}

/*
 * method: I2C_write()
 */
int I2C_write(I2C_TypeDef* I2Cx, uint8_t* data, uint16_t countOfBytes, uint16_t timeout){
	uint16_t timeout_ = timeout;
	int i;
	for(i = 0; i < countOfBytes;i++){
		I2C_SendData(I2Cx, data[i]);
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
#ifdef ENABLE_I2C_TIMEOUT
			if(!(timeout_--)) return 1;
#endif
		}
	}

	return 0;
}

void I2C_stop(I2C_TypeDef* I2Cx){
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

int I2C_read(I2C_TypeDef* I2Cx, uint8_t* data, uint16_t countOfBytes, uint16_t timeout){
	uint16_t timeout_ = timeout;
	int i;
	for (i = 0; i < countOfBytes; i++) {
		if (i != countOfBytes -1) {
			I2C_AcknowledgeConfig(I2Cx, ENABLE);
		} else {
			I2C_AcknowledgeConfig(I2Cx, ENABLE);
			I2C_GenerateSTOP(I2Cx, ENABLE);
		}
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
#ifdef ENABLE_I2C_TIMEOUT
			if (!(timeout_--)) return 1;
#endif
		}
		data[i] = I2C_ReceiveData(I2Cx);
	}
	return 0;
}


#endif /* I2C_H_ */



