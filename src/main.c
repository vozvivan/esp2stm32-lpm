/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f401_discovery.h"
#include "main.h"
#include "i2c_ex.h"

tx_Mode mode = MODE_BAR_INIT;


int main(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD, &gpio);
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	SysTick_Config(SystemCoreClock/1000); //3

	//nit_LCD();
	i2c_init();
	Timer5Init();
	Timer3Init();
	Timer4Init();
	send_lcd_size("_asd_vanya_boom_boom_boom_sd_asd_lkg_asdlk", strlen("_asd_vanya_boom_boom_boom_sd_asd_lkg_asdlk"));
	//NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);	//Возврат в режим Sleep после завершения обработчика прерывания
	//__WFI();
	//Delay_ms(1000);
	//while(1);
	/*i2c_init();
	uint8_t buff[] = {0b00111100, 0b00111100, 0b00111100, 0b00101100 ,0b00101100,
			0b11001100, 0b00001100, 0b10001100, 0b00001100, 0b00011100,
			0b00001100, 0b01101100, 0b00001100, 0b11111100};
	for(int i = 0; i < 14; i++)
		if(!i2c_send(buff[i]))
					i--;
	uint8_t buff_s[] = {(get_lcd_embd_h('l') << 4) + 0b1101, (get_lcd_embd_l('l') << 4) + 0b1101, \
			(get_lcd_embd_h('s') << 4) + 0b1101, (get_lcd_embd_l('s') << 4) + 0b1101
	};
	for(int i = 0; i < 4; i++)
		if(!i2c_send(buff_s[i]))
			i--;*/
	//LCD_Init();
	//display_on();
	int i = 0;
	//send_char_to_lcd('1');
	//send_char_to_lcd('2');
	mode = MODE_ESP_INIT;
	ESPInit(); //2
	DMAUSART2Init_Receive();
	//send_char_to_lcd('3');
	DMAUSART2Init();
	mode = MODE_ESP_START;

	TimerInit();
	NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);	//Возврат в режим Sleep после завершения обработчика прерывания
	__WFI();

	//while(1);
	//send_char_to_lcd(' ');
	//I2C_DMACmd(I2Cx, ENABLE);
	/*while(1);



	for(;;){
		if (i == 20){
			//cursor_to_home();
			//clear_lcd();
			i=0;
		}else i++;
		//cursor_to_home();
		clear_lcd();
		//Delay_ms(100);
		send_char_to_lcd('d');
		send_char_to_lcd('d');
		send_char_to_lcd('a');
		send_char_to_lcd('s');
		send_char_to_lcd('h');
		send_char_to_lcd('a');
		send_char_to_lcd(' ');
		Delay_ms(100);
		GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);


	}*/
}
