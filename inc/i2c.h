/*
 * i2c.h
 *
 *  Created on: 11 нояб. 2017 г.
 *      Author: ivan
 */

#ifndef I2C_H_
#define I2C_H_
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Exported define ------------------------------------------------------------*/
/* Uncomment the line below if you will use the I2C peripheral as a Master */
/*  #define I2C_MASTER */
/* Uncomment the line below if you will use the I2C peripheral as a Slave */

/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */
/* Uncomment the line below if you will use the I2C peripheral in Fast Mode */
//#define FAST_I2C_MODE

/* Uncomment the line below if you will use the I2C peripheral in 10-bit addressing
   mode */
//#define I2C_10BITS_ADDRESS

/* Define I2C Speed --------------------------------------------------------*/
#ifdef FAST_I2C_MODE
 #define I2C_SPEED 340000
 #define I2C_DUTYCYCLE I2C_DutyCycle_16_9
#else /* STANDARD_I2C_MODE*/
 #define I2C_SPEED 100000
 #define I2C_DUTYCYCLE  I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/

/* Define Slave Address  ---------------------------------------------------*/
#ifdef I2C_10BITS_ADDRESS
 #define SLAVE_ADDRESS (uint16_t)0x0330
#else /* I2C_7BITS_ADDRESS */
 #define SLAVE_ADDRESS 0x7E//0x27

#endif /* I2C_10BITS_ADDRESS */

/* USER_TIMEOUT value for waiting loops. This timeout is just a guarantee that the
   application will not remain stuck if the I2C communication is corrupted.
   You may modify this timeout value depending on CPU frequency and application
   conditions (interrupts routines, number of data to transfer, speed, CPU
   frequency...). */
#define USER_TIMEOUT                  ((uint32_t)0x64) /* Waiting 1s */

/* I2Cx Communication boards Interface */
#define I2Cx_DMA                      DMA1
#define I2Cx_DMA_CHANNEL              DMA_Channel_1
#define I2Cx_DR_ADDRESS               ((uint32_t)0x40005410)//0x00//((uint32_t)0x40005410)//0x00//0x7E//
#define I2Cx_DMA_STREAM_TX            DMA1_Stream7
#define I2Cx_DMA_STREAM_RX            DMA1_Stream0
#define I2Cx_TX_DMA_TCFLAG            DMA_FLAG_TCIF7
#define I2Cx_TX_DMA_FEIFLAG           DMA_FLAG_FEIF7
#define I2Cx_TX_DMA_DMEIFLAG          DMA_FLAG_DMEIF7
#define I2Cx_TX_DMA_TEIFLAG           DMA_FLAG_TEIF7
#define I2Cx_TX_DMA_HTIFLAG           DMA_FLAG_HTIF7
#define I2Cx_RX_DMA_TCFLAG            DMA_FLAG_TCIF0
#define I2Cx_RX_DMA_FEIFLAG           DMA_FLAG_FEIF0
#define I2Cx_RX_DMA_DMEIFLAG          DMA_FLAG_DMEIF0
#define I2Cx_RX_DMA_TEIFLAG           DMA_FLAG_TEIF0
#define I2Cx_RX_DMA_HTIFLAG           DMA_FLAG_HTIF0
#define DMAx_CLK                      RCC_AHB1Periph_DMA1

#define I2Cx                          I2C1
#define I2Cx_CLK                      RCC_APB1Periph_I2C1
#define I2Cx_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SDA_PIN                  GPIO_Pin_9
#define I2Cx_SDA_GPIO_PORT            GPIOB
#define I2Cx_SDA_SOURCE               GPIO_PinSource9
#define I2Cx_SDA_AF                   GPIO_AF_I2C1

#define I2Cx_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2Cx_SCL_PIN                  GPIO_Pin_6
#define I2Cx_SCL_GPIO_PORT            GPIOB
#define I2Cx_SCL_SOURCE               GPIO_PinSource6
#define I2Cx_SCL_AF                   GPIO_AF_I2C1

#define TXBUFFERSIZE   2
#define RXBUFFERSIZE   TXBUFFERSIZE


#endif /* I2C_H_ */
