/*
 * main.h
 *
 *  Created on: 5 окт. 2017 г.
 *      Author: twist
 */

#ifndef MAIN_H_
#define MAIN_H_

//#include "I2C.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "commands.h"
#include "string.h"
#include "tm_stm32f4_i2c.h"
#include <stdio.h>
#include "i2c.h"

/**
 *******************************************************************************
 * If you want to use your own SysTick_Handler go to stm32f4xx_it.c and define
 * a constant with name USE_USER_SYSTICK_HANDLER
 *******************************************************************************
 */

#define USE_USER_SYSTICK_HANDLER

#define BAR_ADR					0xB8
#define BAR_ADR_W				0xB8
#define BAR_ADR_R				0xB9
#define CTRL_REG1				0x20
#define PRESS_OUT_XL			0x28
#define PRESS_OUT_L				0x29
#define PRESS_OUT_H				0x2A
#define TEMP_OUT_L				0x2B
#define TEMP_OUT_H				0x2C
#define I2C_MAX_TIMEOUT			0xA000
#define LCD_ADD     			0x27
#define LCD_ADD_W 				0x7E
#define LCD_ADD_R 				0x7F


#define READ_I2C_DATA_DELAY_MS	1000		//temperature and pressure read frequency

uint16_t delay_count = READ_I2C_DATA_DELAY_MS;
uint16_t i2c_delay =   READ_I2C_DATA_DELAY_MS;

typedef enum {
	MODE_BAR_INIT,
	MODE_ESP_INIT,
	MODE_SYS_TICK_INIT,
	MODE_ESP_START,
	MODE_TX_CMD_SEND,
	MODE_DATA_SEND,
	MODE_INIT_ESP_END,
	MODE_RECEIVE_DATA
} tx_Mode;

typedef enum {
	MODE_0,
	MODE_1,
	MODE_2,
	MODE_3,
	MODE_4,
	MODE_5,
	MODE_6,
	MODE_7,
	MODE_8
} i2c_Mode;
typedef enum {
	MODE_WAIT,
	MODE_TRANSFORM,
} tim4_Mode;

typedef enum{
	NONE,
	CLR,
	HOME
} LCD_Mode;

LCD_Mode lcd_mode;

//Extern values from main.c
char send_str[40] = {0};
uint8_t send_cnt = 0;
uint8_t str_tx_busy = 0;
uint8_t cmd_num = 0;
extern tx_Mode mode;
volatile i2c_Mode i2c_mode;
volatile tim4_Mode tim4_mode;
uint8_t transform=0;

/*
typedef enum {
	STATE_BAR_INI,				//Инициализация барометра
	STATE_ESP_INI,				//Инициализация ESP
	STATE_SYSCLK_INI,			//Инициализация системного таймера
	STATE_TX,					//Отправка команды/данных
	STATE_IDLE,					//Ожидание
	STATE_RX,					//Прием команды/данных
	STATE_ERR					//Ошибка отправки данных/команды
} Tx_state;*/

#define buffer_size 200

 char receive_buffer[buffer_size] = {0};
 uint8_t rec_bytes = 0;

uint8_t ap_delay		= 0;

#define send_buff_size 20
char send_buffer[send_buff_size] = {0};

uint8_t trans = 0;
volatile uint8_t clr = 0;


I2C_InitTypeDef  I2C_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;
uint8_t tx_buffer[70] = {
		0b00111100, 0b00111100, 0b00111100, 0b00101100 ,0b00101100,
		0b11001100, 0b00001100, 0b10001100, 0b00001100, 0b00011100,
		0b00001100, 0b01101100, 0b00001100, 0b11111100, 0,
};
uint8_t aTxBuffer[] = {
		0b00111100, 0b00111100
};
uint8_t transformed[256] = {0};
uint8_t size_tx_buff = 14;

uint8_t aRxBuffer [256];

//------------------------------------------------------
// USART2_IRQHandler()
//------------------------------------------------------
void USART2_IRQHandler(void){
	/*if (USART_GetITStatus(USART2, USART_IT_TXE)) {
		USART_ClearITPendingBit(USART2, USART_IT_TXE);

		if(send_str[send_cnt]){
			USART_SendData(USART2, send_str[send_cnt++]);
		} else {
			send_cnt = 0;
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
			memset(send_str, 0, 20);
			// disable flag str_tx_busy
			str_tx_busy = 0;
		}
	}*//*else if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		receive_buffer[rec_bytes] = USART_ReceiveData(USART2);
		if(!receive_buffer[rec_bytes++] || rec_bytes == buffer_size-1){
			receive_buffer[rec_bytes] = 0;
			rec_bytes = 0;
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
			clear_lcd();
			send_str_to_lcd(receive_buffer);
		}
		// at the end send all received symbols
		//send_char_to_lcd(receive_buffer[rec_bytes-1]);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	}*/

}

//------------------------------------------------------
// USART_Send()
//------------------------------------------------------
void USART_Send(){
	str_tx_busy = 1;
	USART_SendData(USART2, send_str[send_cnt++]);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

//------------------------------------------------------
// SysTick_Handler()
//------------------------------------------------------
void SysTick_Handler(void){
	if(delay_count) delay_count--;
	if(i2c_delay) i2c_delay--;
}


//------------------------------------------------------
// Delay_ms()
//------------------------------------------------------
void Delay_ms(uint16_t delay_temp){
	delay_count = delay_temp;
	while(delay_count);
}



//------------------------------------------------------
// ESPInit()
//------------------------------------------------------
void ESPInit(void){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin			=	GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode			=	GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd			=	GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed			=	GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType			=	GPIO_OType_PP;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART_InitTypeDef huart1;

	huart1.USART_BaudRate				=	115200;
	huart1.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
	huart1.USART_Mode					=	USART_Mode_Tx|USART_Mode_Rx;
	huart1.USART_Parity					=	USART_Parity_No;
	huart1.USART_StopBits				=	USART_StopBits_1;
	huart1.USART_WordLength				=	USART_WordLength_8b;

	USART_Init(USART2, &huart1);

	NVIC_EnableIRQ(USART2_IRQn);

	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	USART_Cmd(USART2, ENABLE);
	return;
}

//------------------------------------------------------
// LcdInit()
//------------------------------------------------------
void LCD_Init(void){

	TM_I2C_Init(I2C1, 50000);
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00111100);
	//Delay_ms(100);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00111100); // 2
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00111100); // 3
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00101100); // 4
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00101100); // 5
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b11001100); // 6
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00001100); // 7
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b10001100);
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00001100);
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00011100);
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00001100);
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b01101100);
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00001100);
	//Delay_ms(10);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b11111100); // 14
	//Delay_ms(10);
}

//------------------------------------------------------
// get_lcd_embd_h(char)
//------------------------------------------------------
char get_lcd_embd_h(char a){
	switch (a) {
	case '0' ... '9':
		return 0b0011;
	case 'A' ... 'O':
		return 0b0100;
	case 'P' ... 'Z':
		return 0b0101;
	case 'a' ... 'o':
		return 0b0110;
	case 'p' ... 'z':
		return 0b0111;
	case '.':
		return 0b0010;
	case ' ':
			return 0b0001;
	default:
		return 0b1101;
	}
}

//------------------------------------------------------
// get_lcd_embd_l(char)
//------------------------------------------------------
char get_lcd_embd_l(char a){
	switch(a){
	case '0' ... '9':
		return a-'0';
	case 'A' ... 'O':
		return a -'A' + 1;
	case 'P' ... 'Z':
		return a-'P';
	case 'a' ... 'o':
		return a -'a' + 1;
	case 'p' ... 'z':
		return a-'p';
	case '.':
		return 0b1110;
	case ' ':
		return 0b0000;
	default:
		return 0b1111;
	}
}


void send_char_to_lcd(char to_send){
	char h_part = (get_lcd_embd_h(to_send) << 4) + 0b1101;
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, h_part);
	char l_part = (get_lcd_embd_l(to_send) << 4) + 0b1101;
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, l_part);
}

void cursor_to_home(){
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00001100);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00111100);
}

void display_on(){
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00001100);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b11101100);
}


void clear_lcd(){
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00001100);
	TM_I2C_Write(I2C1, LCD_ADD_W, 0x00, 0b00011100);
}

void send_str_to_lcd(char *str){
	for(int i = 0; i < strlen(str); i++)
		send_char_to_lcd(str[i]);
}


void clr_send_s_lcd(char *str){
	clear_lcd();
	send_str_to_lcd(str);
}

//------------------------------------------------------
// USART_SendStr()
//------------------------------------------------------
void USART_SendStr(char *str){
	uint8_t i;
	for(i = 0; str[i] != 0; i++){
		while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
		USART_SendData(USART2, str[i]);
	}
}

//------------------------------------------------------
// ESPStart()
//------------------------------------------------------
int ESPStart(){
	char ans[20] = {0};
	switch (cmd_num++) {
		case 0:
			//clr_send_s_lcd(" AT CWMODE 3 ");
			SendDataUART2(ATCWMODE3);
			memcpy(send_str, ATCWMODE3, strlen(ATCWMODE3));
			//USART_Send();
			break;
		case 1:
			//clr_send_s_lcd(" AT CIPMODE 0 ");
			SendDataUART2(ATECIPMODE0);
			memcpy(send_str, ATECIPMODE0, strlen(ATECIPMODE0));
			//USART_Send();
			break;
		case 2:
			//clr_send_s_lcd(" AT CIPMUX 1 ");
			SendDataUART2(ATCIPMUX1);
			memcpy(send_str, ATCIPMUX1, strlen(ATCIPMUX1));
			//USART_Send();
			break;
		case 3:
			//clr_send_s_lcd(" AT CIPSERVER 1 88 ");
			SendDataUART2(ATCIPSERVER);
			memcpy(send_str, ATCIPSERVER, strlen(ATCIPSERVER));
			//USART_Send();
			//clr_send_s_lcd(" wait for data ");
			mode  = MODE_INIT_ESP_END;
			break;
		default:
			return 0;
			break;
	}
}

//------------------------------------------------------
// TimerInit()
//------------------------------------------------------
void TimerInit(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	RCC_ClocksTypeDef freqs;
	RCC_GetClocksFreq(&freqs);
	uint32_t TimerPrescaler = freqs.PCLK1_Frequency / 1000;

	TIM_TimeBaseInitTypeDef timer;
	TIM_TimeBaseStructInit(&timer);

	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode 	= TIM_CounterMode_Up;
	timer.TIM_Prescaler		= TimerPrescaler - 1;
	timer.TIM_Period		= 1000; //2000;
	TIM_TimeBaseInit(TIM2, &timer);

	NVIC_EnableIRQ(TIM2_IRQn);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	//NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);	//Возврат в режим Sleep после завершения обработчика прерывания
	//__WFI();	//Установка режима Sleep
}

uint8_t check_data(){
	if(1)
		return 1;
	else 0;
}

void generate_error(){
	memcpy(send_buffer, " ERROR ", send_buff_size);
}

void send_to_lcd(){
	int8_t sizeSendData = strlen(send_buffer);
	//memcpy(bufferDMAUSARTSend, send_buffer, sizeSendData);
	/**
	 * TODO: change dma
	 */
	DMA_Cmd(DMA1_Stream6, ENABLE);
}
//------------------------------------------------------
// TIM2_IRQHandler()
//------------------------------------------------------
void TIM2_IRQHandler(void){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if (mode == MODE_ESP_START && !str_tx_busy) {
			//i2c_delay = READ_I2C_DATA_DELAY_MS;
			if (cmd_num == 4) return;
			ESPStart();
		} if (mode == MODE_INIT_ESP_END){
			TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
			// закомментить, если будет что-то нечто новенькое...

			//USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);
			//DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, DISABLE);
			//DMA_Cmd(DMA1_Stream6, DISABLE);
			//DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | \
					DMA_FLAG_TEIF6 | DMA_FLAG_HTIF6);
			//DMAUSART2Init_Receive();
			//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		} /*if(mode == MODE_DATA_SEND){// && receive_data_status == 1){
			if(check_data()){
				transform_data();
			}else{
				generate_error();
			}
			send_to_lcd();
			//receive_data_status = 0;
		}*/

	}
}


// Размер буфера для получения данных по DMA, через UART2
static const uint8_t bufferSizeDMAUSARTSend = 20;

//#define bufferSizeDMAUSARTSend 100;

// Буфер для отправки данных по DMA, через UART2
char bufferDMAUSARTSend[20];

// Инициализация DMA (DMA1_Channel_4_Stream6) для USRT2 на отправку данных
void DMAUSART2Init(void)
{
    DMA_InitTypeDef DMA_Init_USART;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    DMA_Init_USART.DMA_Channel = DMA_Channel_4;
    DMA_Init_USART.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    DMA_Init_USART.DMA_Memory0BaseAddr = (uint32_t)bufferDMAUSARTSend;
    DMA_Init_USART.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_Init_USART.DMA_BufferSize = bufferSizeDMAUSARTSend;
    DMA_Init_USART.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_Init_USART.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_Init_USART.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_Init_USART.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_Init_USART.DMA_Mode = DMA_Mode_Normal;
    DMA_Init_USART.DMA_Priority = DMA_Priority_Medium;
    DMA_Init_USART.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_Init_USART.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_Init_USART.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_Init_USART.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream6, &DMA_Init_USART);

    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
}


// Прерывание DMA1_Stream6
void DMA1_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET)
    {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
    }
}


// Функция отправки данных через DMA
void SendDataUART2(char *SendData)
{
    int8_t sizeSendData = strlen(SendData);
    memcpy(bufferDMAUSARTSend, SendData, sizeSendData+1);
    DMA_Cmd(DMA1_Stream6, ENABLE);
}


static const uint8_t bufferSizeDMAUSARTReceive  = 100;

char bufferDMAUSARTReceive[100] = {0};

// Статус полученных данных
int8_t receiveDataStatus = 0;


void DMAUSART2Init_Receive(void)
{
    DMA_InitTypeDef DMA_Init_USART;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    DMA_Init_USART.DMA_Channel = DMA_Channel_4;
    DMA_Init_USART.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    DMA_Init_USART.DMA_Memory0BaseAddr = (uint32_t)bufferDMAUSARTReceive;
    DMA_Init_USART.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_Init_USART.DMA_BufferSize = bufferSizeDMAUSARTReceive-1;

    DMA_Init_USART.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_Init_USART.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_Init_USART.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_Init_USART.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_Init_USART.DMA_Mode = DMA_Mode_Circular;
    DMA_Init_USART.DMA_Priority = DMA_Priority_High;
    DMA_Init_USART.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_Init_USART.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_Init_USART.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_Init_USART.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream5, &DMA_Init_USART);

    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

    NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);

    DMA_Cmd(DMA1_Stream5, ENABLE);
}



// Функция очистка массива полученных данных
void ClearReceiveData()
{
    memset(&bufferDMAUSARTReceive, '\0', sizeof(bufferDMAUSARTReceive)+1);
    receiveDataStatus = 0;
}

// Прерывание DMA1_Stream5
void DMA1_Stream5_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5) == SET)
  {
    DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
  	lcd_mode = HOME;
    send_lcd_size(bufferDMAUSARTReceive, 20);
    //send_str_to_lcd(bufferDMAUSARTReceive);
    ClearReceiveData();
    DMA_Cmd(DMA1_Stream5, ENABLE);
  }
}

void send_lcd_size(char* str, int size){
	memcpy(transformed, str, size);
	transformed[size]=0;
	TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

void send_lcd(char* str){
	if(strlen(str)>200){
		memcpy(transformed, str, strlen(200));
		transformed[200] = 0;
	}
	else
		memcpy(transformed, str, sizeof(str)+1);
	TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

void Timer4Init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	RCC_ClocksTypeDef freqs;
	RCC_GetClocksFreq(&freqs);
	uint32_t TimerPrescaler = freqs.PCLK1_Frequency / 1000;

	TIM_TimeBaseInitTypeDef timer;
	TIM_TimeBaseStructInit(&timer);

	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode 	= TIM_CounterMode_Up;
	timer.TIM_Prescaler		= TimerPrescaler - 1;
	timer.TIM_Period		= 10; //2000;
	TIM_TimeBaseInit(TIM4, &timer);

	NVIC_EnableIRQ(TIM4_IRQn);
	tim4_mode = MODE_WAIT;
	transform=0;
	lcd_mode = NONE;
	//NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);	//Возврат в режим Sleep после завершения обработчика прерывания
	//__WFI();	//Установка режима Sleep
}



void TIM4_IRQHandler(){
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
			TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
			if(tim4_mode == MODE_WAIT){
				if(i2c_mode == MODE_0){
					tim4_mode = MODE_TRANSFORM;
					transform = 0;
					i2c_mode = MODE_1;
					if(lcd_mode == CLR){
						tx_buffer[0] = 0b00001100;
						tx_buffer[1]= 0b00011100;
						transform = 2;
					}
					if(lcd_mode == HOME){
						tx_buffer[0] = 0b00001100;
						tx_buffer[1]= 0b00111100;
						transform = 2;
					}
				}
			}
			if(tim4_mode == MODE_TRANSFORM){
				if(transform == strlen(transformed)){
					tx_buffer[strlen(transformed)*2] = 0;
					tim4_mode = MODE_WAIT;
					transform = 0;
					TIM_Cmd(TIM4, DISABLE);
					TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
					TIM_Cmd(TIM3, ENABLE);
					TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
					return;
				}
				tx_buffer[2*transform] = (get_lcd_embd_h(transformed[transform]) << 4) + 0b1101;
				tx_buffer[2*transform+1] = (get_lcd_embd_l(transformed[transform]) << 4) + 0b1101;
				transform++;
			}
	}
}



void Timer3Init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	RCC_ClocksTypeDef freqs;
	RCC_GetClocksFreq(&freqs);
	uint32_t TimerPrescaler = freqs.PCLK1_Frequency / 1000;

	TIM_TimeBaseInitTypeDef timer;
	TIM_TimeBaseStructInit(&timer);

	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode 	= TIM_CounterMode_Up;
	timer.TIM_Prescaler		= TimerPrescaler - 1;
	timer.TIM_Period		= 3; //2000;
	TIM_TimeBaseInit(TIM3, &timer);

	NVIC_EnableIRQ(TIM3_IRQn);

	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	//NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);	//Возврат в режим Sleep после завершения обработчика прерывания
	//__WFI();	//Установка режима Sleep
}


void TIM3_IRQHandler(){
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
			if(i2c_mode == MODE_1 || i2c_mode == MODE_0){
				if(trans == strlen(tx_buffer)){
					trans = 0;
					i2c_mode = MODE_0;
					TIM_Cmd(TIM3, DISABLE);
					TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
					return;
				}
				aTxBuffer[0] = 0x00;
				aTxBuffer[1] = tx_buffer[trans];
				I2C_GenerateSTART(I2Cx, ENABLE);
				i2c_mode = MODE_2;
			}
			if(i2c_mode == MODE_2){
				if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
					I2C_Send7bitAddress(I2Cx, SLAVE_ADDRESS, I2C_Direction_Transmitter);
					i2c_mode = MODE_3;
				}
			}
			if(i2c_mode == MODE_3){
				if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
					DMA_Cmd(I2Cx_DMA_STREAM_TX, ENABLE);
					I2C_DMACmd(I2Cx, ENABLE);
					i2c_mode = MODE_4;
				}
			}
			if(i2c_mode == MODE_4){
				if((DMA_GetFlagStatus(I2Cx_DMA_STREAM_TX,I2Cx_TX_DMA_TCFLAG) != RESET)){
					I2C_DMACmd(I2Cx, DISABLE);
					i2c_mode = MODE_5;
				}
			}
			if(i2c_mode == MODE_5){
				if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF)){
					I2C_GenerateSTOP(I2Cx, ENABLE);
					DMA_Cmd(I2Cx_DMA_STREAM_TX, DISABLE);
					DMA_ClearFlag(I2Cx_DMA_STREAM_TX, I2Cx_TX_DMA_TCFLAG | I2Cx_TX_DMA_FEIFLAG | I2Cx_TX_DMA_DMEIFLAG | \
					                                       I2Cx_TX_DMA_TEIFLAG | I2Cx_TX_DMA_HTIFLAG);
					i2c_mode = MODE_1;
					trans++;
				}
			}
		}
}

int i2c_init(void)
{

  I2C_Config();

  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;//0xA0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;//I2C_Ack_Enable;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

  /* I2C Initialize */
  I2C_Cmd(I2C1, ENABLE);
  I2C_Init(I2Cx, &I2C_InitStructure);

  trans = 0;
  i2c_mode = MODE_0;

}

void I2C1_EV_IRQHandler(void){
}



void DMA1_Stream0_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0) == SET)
  {
    DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
  }
}

void DMA1_Stream7_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7) == SET)
  {
    DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);
  }
}

void I2C_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  // RCC Configuration
  //I2C Peripheral clock enable
  RCC_APB1PeriphClockCmd(I2Cx_CLK, ENABLE);

  //SDA GPIO clock enable
  RCC_AHB1PeriphClockCmd(I2Cx_SDA_GPIO_CLK, ENABLE);

  //SCL GPIO clock enable
  RCC_AHB1PeriphClockCmd(I2Cx_SCL_GPIO_CLK, ENABLE);

  // Reset I2Cx IP
  RCC_APB1PeriphResetCmd(I2Cx_CLK, ENABLE);

  // Release reset signal of I2Cx IP
  RCC_APB1PeriphResetCmd(I2Cx_CLK, DISABLE);

  // Enable the DMA clock
  RCC_AHB1PeriphClockCmd(DMAx_CLK, ENABLE);

  // GPIO Configuration
  //Configure I2C SCL pin
  GPIO_InitStructure.GPIO_Pin = I2Cx_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;//GPIO_Speed_50MHz;//
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;//GPIO_PuPd_NOPULL;//
  GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

  //Configure I2C SDA pin
  GPIO_InitStructure.GPIO_Pin = I2Cx_SDA_PIN;
  GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);

  // Connect PXx to I2C_SCL
  GPIO_PinAFConfig(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_SOURCE, I2Cx_SCL_AF);

  // Connect PXx to I2C_SDA
  GPIO_PinAFConfig(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_SOURCE, I2Cx_SDA_AF);

  // DMA Configuration
  // Clear any pending flag on Tx Stream
  DMA_ClearFlag(I2Cx_DMA_STREAM_TX, I2Cx_TX_DMA_TCFLAG | I2Cx_TX_DMA_FEIFLAG | I2Cx_TX_DMA_DMEIFLAG | \
                                       I2Cx_TX_DMA_TEIFLAG | I2Cx_TX_DMA_HTIFLAG);

  // Clear any pending flag on Rx Stream
  DMA_ClearFlag(I2Cx_DMA_STREAM_RX, I2Cx_RX_DMA_TCFLAG | I2Cx_RX_DMA_FEIFLAG | I2Cx_RX_DMA_DMEIFLAG | \
                                       I2Cx_RX_DMA_TEIFLAG | I2Cx_RX_DMA_HTIFLAG);

  // Disable the I2C Tx DMA stream
  DMA_Cmd(I2Cx_DMA_STREAM_TX, DISABLE);
  // Configure the DMA stream for the I2C peripheral TX direction
  DMA_DeInit(I2Cx_DMA_STREAM_TX);

  // Disable the I2C Rx DMA stream
  DMA_Cmd(I2Cx_DMA_STREAM_RX, DISABLE);
  // Configure the DMA stream for the I2C peripheral RX direction
  DMA_DeInit(I2Cx_DMA_STREAM_RX);

  DMA_InitStructure.DMA_Channel = I2Cx_DMA_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = I2Cx_DR_ADDRESS;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;//
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  // Init DMA for Reception
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aRxBuffer;
   DMA_InitStructure.DMA_BufferSize = RXBUFFERSIZE;
   DMA_DeInit(I2Cx_DMA_STREAM_RX);
   DMA_Init(I2Cx_DMA_STREAM_RX, &DMA_InitStructure);

  //Init DMA for Transmission
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aTxBuffer;
   DMA_InitStructure.DMA_BufferSize = TXBUFFERSIZE;
   DMA_DeInit(I2Cx_DMA_STREAM_TX);
   DMA_Init(I2Cx_DMA_STREAM_TX, &DMA_InitStructure);

   // Configure I2C Filters
   I2C_AnalogFilterCmd(I2Cx,ENABLE);
   I2C_DigitalFilterConfig(I2Cx,0x0F);

  // I2C ENABLE
  //I2C_Cmd(I2Cx, ENABLE);
}


// uart dma
// http://forum.amperka.ru/threads/stm32f4discovery-uart-dma.3852/

// dma to i2c
// https://github.com/g4lvanix/STM32F4-workarea/blob/master/Project/STM32F4xx_StdPeriph_Examples/I2C/I2C_TwoBoards/I2C_DataExchangeDMA/main.c


#endif /* MAIN_H_ */
