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

typedef enum{
	CHECK,
	CLEAR,
	NOT_VALID,
	VALID,
	PRESS,
	TEMP,
	CHECK_SUM,
	CONNECTED,
	WAIT
} DMA_UART_Rec;

volatile DMA_UART_Rec rec_uart_mode = CHECK;

LCD_Mode lcd_mode;

volatile uint8_t packet_struct = 0;


//Extern values from main.c
char send_str[40] = {0};
uint8_t send_cnt = 0;
uint8_t str_tx_busy = 0;
uint8_t cmd_num = 0;
extern tx_Mode mode;
volatile i2c_Mode i2c_mode;
volatile tim4_Mode tim4_mode;
uint8_t transform=0;

uint8_t acknowledge_packet[4] = {0xff, 0x08, 0xff ^ 0x08, 0};

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
			//1
			SendDataUART2(ATCWMODE3);
			memcpy(send_str, ATCWMODE3, strlen(ATCWMODE3));
			break;
		case 1:
			SendDataUART2(ATECIPMODE0);
			memcpy(send_str, ATECIPMODE0, strlen(ATECIPMODE0));
			break;
		case 2:

			SendDataUART2(ATCIPMUX1);
			memcpy(send_str, ATCIPMUX1, strlen(ATCIPMUX1));

			break;
		case 3:

			//SendDataUART2(ATCIPSERVER);
			//memcpy(send_str, ATCIPSERVER, strlen(ATCIPSERVER));
			//mode  = MODE_INIT_ESP_END;


			SendDataUART2(MARAT);
			//memcpy(send_str, MARAT, strlen(MARAT));
			break;
		case 5:
			SendDataUART2(ATCIPSERVER);
			memcpy(send_str, ATCIPSERVER, strlen(ATCIPSERVER));
			mode  = MODE_INIT_ESP_END;
			break;
		case 4:

			SendDataUART2("AT+CIFSR\r\n");
			memcpy(send_str, ATCIPSERVER, strlen(ATCIPSERVER));
			//mode  = MODE_INIT_ESP_END;
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
	timer.TIM_Period		= 20000; //2000;
	TIM_TimeBaseInit(TIM2, &timer);

	NVIC_EnableIRQ(TIM2_IRQn);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	//NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);	//Возврат в режим Sleep после завершения обработчика прерывания
	//__WFI();	//Установка режима Sleep
}


//------------------------------------------------------
// TIM2_IRQHandler()
//------------------------------------------------------
void TIM2_IRQHandler(void){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if (mode == MODE_ESP_START && !str_tx_busy) {
			//i2c_delay = READ_I2C_DATA_DELAY_MS;
			if (cmd_num == 6) return;
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
static const uint8_t bufferSizeDMAUSARTSend = 30;

//#define bufferSizeDMAUSARTSend 100;

// Буфер для отправки данных по DMA, через UART2
char bufferDMAUSARTSend[30];

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


static const uint8_t bufferSizeDMAUSARTReceive  = 27;//17

char bufferDMAUSARTReceive[30] = {0};

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
    // TODO:
    //
    DMA_Init_USART.DMA_Mode =DMA_Mode_Circular;//DMA_Mode_Normal;//
    DMA_Init_USART.DMA_Priority = DMA_Priority_High;
    DMA_Init_USART.DMA_FIFOMode = DMA_FIFOMode_Disable;//DMA_FIFOMode_Enable;//
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
}

void generate_error(){
	memcpy(transformed, " Not valid data!", 16);
	rec_uart_mode = CLEAR;
	transformed[16]=0;
	TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

void Timer5Init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	RCC_ClocksTypeDef freqs;
	RCC_GetClocksFreq(&freqs);
	uint32_t TimerPrescaler = freqs.PCLK1_Frequency / 1000;

	TIM_TimeBaseInitTypeDef timer;
	TIM_TimeBaseStructInit(&timer);

	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode 	= TIM_CounterMode_Up;
	timer.TIM_Prescaler		= TimerPrescaler - 1;
	timer.TIM_Period		= 30; //2000;
	TIM_TimeBaseInit(TIM5, &timer);

	NVIC_EnableIRQ(TIM5_IRQn);
	//NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);	//Возврат в режим Sleep после завершения обработчика прерывания
	//__WFI();	//Установка режима Sleep
}

uint8_t i = 0;
uint8_t same = 0;
uint8_t same_con = 0;
volatile char checked[] = "IPD";
const uint8_t connected[] = "CONNECTED";
uint8_t con_check = 1;
uint8_t size_checked = 3;
volatile uint8_t size_press = 0;
volatile uint8_t size_temp = 0;
volatile uint8_t check_s = 0;

uint8_t settings_packet[5] = {0xff, 0x01, 0, 0, 0};
//volatile uint8_t packet_struct = 0;
// TODO:
volatile uint8_t type_packet = 1;

void send_settings_packet(){
	settings_packet[2] = packet_struct;
	settings_packet[3] = 0xff ^ 0x01 ^ packet_struct;
	//SendUART2(settings_packet);
}

uint8_t check_data(uint8_t *data){
	if(data[0] != 0xff)
		return 0;
	if(data[1] != 0x04) //???
			return 0;
	switch(packet_struct){
		case 0x11111101 ... 0x11111110:
			if((data[0] ^ data[1] ^ data[2] ^ data[3] ^ \
			   data[4] ^ data[5] ^ data[6] + 1) == data[8])
				return 1;
		    return 0;
		case 0x11111001 ... 0x11111010:
			if((data[0] ^ data[1] ^ data[2] ^ \
				data[3] ^ data[4] + 1) == data[8])
				return 1;
			return 0;
		case 0x11110101 ... 0x11110110:
		if((data[0] ^ data[1] ^ \
			data[2] ^ data[3] + 1) == data[8])
			return 1;
		return 0;
		default:
			return 0;
	}
}

void transformByRaw(){
	rec_uart_mode = WAIT;
	type_packet = 1;
	switch(type_packet){
		case(1):
				/*float temp = 0;
				float press = 0;
				if (transformed[0])
					temp  = 42.5 +
						(float) ((int16_t) ((transformed[1] << 8) | transformed[0])) / 480.;
				if (transformed[2])
					press = (float) (((int32_t) transformed[4] << 16)
							| ((uint16_t) transformed[3] << 8) | (transformed[2]))
								* 0.75006375541921 / 4096.;*/
		sprintf(transformed, "%.2f %.2f",
				42.5 + (float) ((int16_t) ((transformed[2] << 8) | transformed[3])) / 480.,
				(float) (((int32_t) transformed[4] << 16)
											| ((uint16_t) transformed[5] << 8) | (transformed[6]))
												* 0.75006375541921 / 4096.);
	 				//double press = 17.2;
			//double temp = 654.2;
			//sprintf(transformed, "%.2f %.2f", press, temp);

			break;
		case(2):
			//double press = 17.2;
			//sprintf(transformed, "%.2f", press);
			//sprintf(transformed, "%.15s", transformed);
			break;
		case(3):
			//double temp = 17.2;
			//sprintf(transformed, "%.2f", temp);
			//sprintf(transformed, "%.15s", transformed);
			break;
	}
	sprintf(transformed, "%.15s", transformed);
	rec_uart_mode = CLEAR;
}

void TIM5_IRQHandler(){
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
			if(rec_uart_mode == CHECK){
				if(bufferDMAUSARTReceive[i] == checked[same]){
					same++;
					if(same == size_checked){
						rec_uart_mode = VALID;
					}
				}else{
					same = 0;
				}
				if(bufferDMAUSARTReceive[i] == connected[same_con]){
					same_con++;
					if(same_con == con_check){
						rec_uart_mode = CONNECTED;
						rec_uart_mode = VALID;
					}
					}else{
						same_con = 0;
					}

				i++;
				if(bufferSizeDMAUSARTReceive == i)
					rec_uart_mode = NOT_VALID;
			}
			if(rec_uart_mode == VALID){
				/*char size = bufferDMAUSARTReceive[i];
				size_press  = size >> 4;
				size_temp = size & 11110000;
				memcpy(transformed, bufferDMAUSARTReceive[i+1], size_press);
				memcpy(transformed[size_press], " ", 1);
				memcpy(transformed, bufferDMAUSARTReceive[i+2+size_press], size_temp);*/
				//memcpy(transformed, bufferDMAUSARTReceive+6, bufferSizeDMAUSARTReceive-7);
				if (bufferSizeDMAUSARTReceive-i-7 >= 0){
					memcpy(transformed, bufferDMAUSARTReceive+i+6, bufferSizeDMAUSARTReceive-i-7);
					memcpy(transformed+bufferSizeDMAUSARTReceive-i-7, bufferDMAUSARTReceive, i-size_checked);
				}else{
					memcpy(transformed, bufferDMAUSARTReceive+i+6-bufferSizeDMAUSARTReceive, i);
				}//transformed[size_press + size_temp + 1]=0;
				//transformed[bufferSizeDMAUSARTReceive-i]=0;
				transformed[bufferSizeDMAUSARTReceive-7-size_checked] = 0;
				//rec_uart_mode = WAIT;
				//transformByRaw();

				rec_uart_mode = CLEAR;

				//SebdUART2(acknowledge_packet); // ACK
				//rec_uart_mode = CHECK_SUM;
			}
			if(rec_uart_mode == CHECK_SUM){
				check_s ^= bufferDMAUSARTReceive[i+1];
				i++;
				if(i == bufferSizeDMAUSARTReceive-1){
					if(check_s != bufferDMAUSARTReceive[i])
						rec_uart_mode = CLEAR;
					else
						rec_uart_mode = NOT_VALID;
					check_s = 0;
				}
			}
			if(rec_uart_mode == NOT_VALID){
				memcpy(transformed, " ERROOR not valid data ", 24);
				transformed[24]=0;
				rec_uart_mode = CLEAR;
			}
			if(rec_uart_mode == CONNECTED){
				send_settings_packet();
				memcpy(transformed, connected, 9);
				transformed[9]=0;
				rec_uart_mode = CLEAR;
			}
			if(rec_uart_mode == CLEAR){
				TIM_Cmd(TIM4, ENABLE);
				TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
				memset(&bufferDMAUSARTReceive, '\0', bufferSizeDMAUSARTReceive);
				//rec_uart_mode = CHECK;
				//TODO::
				rec_uart_mode = CHECK;
				//
				i = 0;
				same = 0;
				TIM_Cmd(TIM5, DISABLE);
				TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
				//DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5| DMA_FLAG_FEIF5|	DMA_FLAG_DMEIF5| DMA_FLAG_TEIF5| DMA_FLAG_HTIF5);
				//USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
				//DMA_DeInit(DMA1_Stream5);

				//TODO: попробовать
				//USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);
				//DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, DISABLE);
				//DMA_Cmd(DMA1_Stream6, DISABLE);
				//DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | \
									DMA_FLAG_TEIF6 | DMA_FLAG_HTIF6);
				// ЗАКОНЧИТЬ попробовать

				DMA_Cmd(DMA1_Stream5, ENABLE);
				USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
				DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
				DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | \
													DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);

				//DMAUSART2Init_Receive();

			}

	}
}

// Прерывание DMA1_Stream5
void DMA1_Stream5_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5) == SET)
  {
    DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
    if(rec_uart_mode == CHECK){
		lcd_mode = CLR;
		type_packet = 1;
		memcpy(checked, "IPD,", 4);
		size_checked = 3;
		//TODO:::!!!!
		rec_uart_mode = CHECK;
		// !!!
		//send_lcd_size(bufferDMAUSARTReceive, 20);
		TIM_Cmd(TIM5, ENABLE);
		TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    }
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
						transform = 1;
					}
					if(lcd_mode == HOME){
						tx_buffer[0] = 0b00001100;
						tx_buffer[1]= 0b00111100;
						transform = 1;
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
				tx_buffer[2*transform] = (get_lcd_embd_h(transformed[transform - (lcd_mode != NONE)]) << 4) + 0b1101;
				tx_buffer[2*transform+1] = (get_lcd_embd_l(transformed[transform - (lcd_mode != NONE)]) << 4) + 0b1101;
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

void ButEXTI_Init(void)
{
	// Clock for GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Clock for SYSCFG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	// GPIOA initialization as an input from user button (GPIOA0)
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	// Selects the GPIOA pin 0 used as external interrupt source
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	// External interrupt settings
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
	// Nested vectored interrupt settings
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	// EXTI0_IRQn has Most important interrupt
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);

}

uint8_t request_packet[4] = {0xff, 0x02, 0xff ^ 0x02, 0};

void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0))
	{
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
		SendDataUART2(request_packet);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}
// uart dma
// http://forum.amperka.ru/threads/stm32f4discovery-uart-dma.3852/

// dma to i2c
// https://github.com/g4lvanix/STM32F4-workarea/blob/master/Project/STM32F4xx_StdPeriph_Examples/I2C/I2C_TwoBoards/I2C_DataExchangeDMA/main.c



#endif /* MAIN_H_ */
