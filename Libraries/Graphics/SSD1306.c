/*********************************************************************
 Editor: Lauren
 E-Mail: Lauren.pan@dfrobot.com
 Date:   2012.11.21
 Description
 Improve the library to work with Oled 2864 and Arduino Uno
 Modify the display() function

 This is a library for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98
 Oled 2864 from DFRobot
 ------> http://www.dfrobot.com/index.php?route=product/product&product_id=802#.UKyOjE09ha0

 You can also pick up an OLED 2864 Display Module in DFRobot
 ------>
 The OLED 2864 Display Module uses IIC to comunicate and this 
 provides great convenience for using.

 These displays use SPI to communicate, 4 or 5 pins are required to  
 interface

 Adafruit invests time and resources providing this open source code, 
 please support Adafruit and open-source hardware by purchasing 
 products from Adafruit!

 Written by Limor Fried/Ladyada  for Adafruit Industries.  
 BSD license, check license.txt for more information
 All text above, and the splash screen must be included in any redistribution
 *********************************************************************/

#include <stdlib.h>

//#include "Adafruit_GFX.h"

#include "SSD1306.h"

//#include "glcdfont.c"

// a 5x7 font table
extern uint8_t font[];

// the memory buffer for the LCD

uint8_t bufferX[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] =
{
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ######    #####  ###  ####  ######  #####        ###   ######  ######  ####    #####  ###         ##         ###         #######
	// ########   ####  ###  ####   #####  ####          ###   #####  ######  ######   ####  ###         ##         ###         #######
	// #########   ###  ###  ####    ####  ####  ######  ####   ####  ######  #######   ###  ##########  #########  ##########  #######
	// ##########   ##  ###  ####     ###  ####  ######  #####    ##  ######  ########   ##  ##########  #########  ##########  #######
	// ###########   #  ###  ####      ##  ####  ######  ####         ######  #########   #  ######      #########  ######      #######
	// ############     ###  ####  #    #  ####  ######  ###          ######  ##########     ######      #########  ######      #######
	// ############     ###  ####  ##      ####  ######  ###  ######  ######  ##########     ##########  #########  ##########  #######
	// ###########   #  ###  ####  ###     ####  ######  ###  ######  ######  #########   #  ##########  #########  ##########  #######
	// #########    ##  ###  ####  ####    ####  ######  ###  ######  ######  #######    ##  ##########  #########  ##########  #######
	// ########    ###  ###  ####  #####   ####          ###          ##          ##    ###  ###         #########  ###         #######
	// #######    ####  ###  ####  ######  #####        #####         ##          #    ####  ###         #########  ###         #######
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// ################################################################################################################################
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #      #####       ####      ############      ###          ###   #####        ###   ##############    ###           ###       #
	// #       #####      ####     ##############     ####         ###    #####       ###   ##############    ###           ###       #
	// #        #####     ####    ################    #####        ###     #####      ###   ##############    ###     #     ###       #
	// #         ######   ####    ####         ###    ######       ###       ####     ###             ####    ###    ###    ###       #
	// #          ######  ####    ####         ###    ########     ###        ####    ###             ####    ###   ####    ###       #
	// #            ##### ####    ####         ###    #########    ###         ####   ###             ####    ###   #####   ###       #
	// #             #########    ####         ###    #### #####   ###     ##############       ##########    ###  #######  ###       #
	// #              ########    ####         ###    ####  #####  ###     ##############       ##########    ### ######### ###       #
	// #              ########    ####         ###    ####   ##### ###    ####        ###       ##########    #######  #### ###       #
	// #             #### ####    ####         ###    ####    ########    ####        ###             ####    ######    #######       #
	// #            ####  ####    ####         ###    ####     #######    ####        ###             ####    ######     ######       #
	// #          #####   ####    ####         ###    ####      ######    ####        ###             ####    #####      ######       #
	// #         #####    ####    ################    ####       #####     ##############   ##############    ####        #####       #
	// #        #####     ####     ##############     ####        ####     ##############   ##############    ###          ####       #
	// #       #####      ####      ############      ####         ###      #############   ##############    ###           ###       #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// #                                                                                                                              #
	// ################################################################################################################################
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xFD, 0xF9, 0xF1, 0xE3, 0xC7, 0x0F, 0x1F, 0x3F, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x01, 0xC3, 0x87, 0x0F, 0x1F, 0x3F, 0x7F, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x01, 0xF9, 0xF9, 0xF9, 0xF9, 0xF9, 0xF9, 0x01, 0x03, 0xFF, 0xFF, 0xFD, 0x39, 0x11, 0x83, 0x87, 0x8F, 0x8F, 0x9F, 0x9F, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xFD, 0xF9, 0xF1, 0xE3, 0xC7, 0x0F, 0x1F, 0x3F, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xF9, 0xF9, 0xF9, 0x99, 0x99, 0x99, 0x99, 0x01, 0x01, 0xFF, 0xFF, 0xF9, 0xF9, 0xF9, 0xF9, 0xF9, 0xF9, 0xF9, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xF9, 0xF9, 0xF9, 0x99, 0x99, 0x99, 0x99, 0x01, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0xF3, 0xF1, 0xF1, 0xF8, 0xFC, 0xFE, 0xFF, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF0, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF0, 0xF8, 0xFF, 0xFF, 0xFF, 0xF8, 0xF0, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF0, 0xF0, 0xFF, 0xFF, 0xF3, 0xF3, 0xF3, 0xF3, 0xF0, 0xF0, 0xF3, 0xF3, 0xF3, 0xF3, 0xFF, 0xF7, 0xF3, 0xF1, 0xF1, 0xF8, 0xFC, 0xFE, 0xFF, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF3, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF,
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x60, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x20, 0x60, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x87, 0xCF, 0xFF, 0xFE, 0x7C, 0x38, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x0F, 0x1E, 0x3E, 0x7C, 0xF8, 0xF0, 0xE0, 0xC0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF8, 0xF8, 0xF9, 0x1B, 0x1F, 0x1F, 0x1E, 0x1C, 0x18, 0x18, 0x18, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xF0, 0xF8, 0x3E, 0x1F, 0x1F, 0x3F, 0x7C, 0xF8, 0xF0, 0xC0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x0C, 0x0E, 0x0F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x0F, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0F, 0x0F, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
	0xFF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF,
};



//----------------------------------------------------------------------------------
//   Target specific initialization of SPI interface in hal_spi_config.c
//----------------------------------------------------------------------------------

void SSD_SPI_start(void)
{
	SPI_change_mode(LCD_SPI, TRUE);

	GPIO_ResetBits(DISP_CS_PORT, DISP_CS_PIN); //CS;
}

void SSD_SPI_end(void)
{
	GPIO_SetBits(DISP_CS_PORT, DISP_CS_PIN); //CS
}













// the most basic function, set a single pixel
void drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) || (x >= get_width()) || (y < 0) || (y >= get_height()))
		return;

	// check rotation, move pixel around if necessary
	switch (getRotation())
	{
	case 1:
		swap(x, y)
		;
		x = WIDTH - x - 1;
		break;
	case 2:
		x = WIDTH - x - 1;
		y = HEIGHT - y - 1;
		break;
	case 3:
		swap(x, y)
		;
		y = HEIGHT - y - 1;
		break;
	}

	// x is which column
	if (color == WHITE)
		bufferX[x + (y>>3) * SSD1306_LCDWIDTH] |= 1 << ((y % 8));
	else
		bufferX[x + (y>>3) * SSD1306_LCDWIDTH] &= ~(1 << ((y % 8)));
}

void SSD1306_begin(uint8_t vccstate, uint8_t flip_v)
{
	// set pin directions

   /// Control_SPI_outputs_Write(1);
	// VDD (3.3V) goes high at start, lets just chill for a ms
	Delay(1);
	// bring reset low
	GPIO_ResetBits(DISP_EN_PORT, DISP_EN_PIN);
	
	// wait 10ms
	Delay(10);
	// bring out of reset
	GPIO_SetBits(DISP_EN_PORT, DISP_EN_PIN);
	// turn on VCC (9V?)
#if defined SSD1306_128_32
	// Init sequence for 128x32 OLED module
	ssd1306_command(SSD1306_DISPLAYOFF);// 0xAE
	ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);// 0xD5
	ssd1306_command(0x80);// the suggested ratio 0x80
	ssd1306_command(SSD1306_SETMULTIPLEX);// 0xA8
	ssd1306_command(0x1F);
	ssd1306_command(SSD1306_SETDISPLAYOFFSET);// 0xD3
	ssd1306_command(0x0);// no offset
	ssd1306_command(SSD1306_SETSTARTLINE | 0x0);// line #0
	ssd1306_command(SSD1306_CHARGEPUMP);// 0x8D
	if (vccstate == SSD1306_EXTERNALVCC)
	{	ssd1306_command(0x10);}
	else
	{	ssd1306_command(0x14);}
	ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
	ssd1306_command(0x00);// 0x0 act like ks0108
	ssd1306_command(SSD1306_SEGREMAP | 0x1);
	ssd1306_command(SSD1306_COMSCANDEC);
	ssd1306_command(SSD1306_SETCOMPINS);// 0xDA
	ssd1306_command(0x02);
	ssd1306_command(SSD1306_SETCONTRAST);// 0x81
	ssd1306_command(0x8F);
	ssd1306_command(SSD1306_SETPRECHARGE);// 0xd9
	if (vccstate == SSD1306_EXTERNALVCC)
	{	ssd1306_command(0x22);}
	else
	{	ssd1306_command(0xF1);}
	ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
	ssd1306_command(0x40);
	ssd1306_command(SSD1306_DISPLAYALLON_RESUME);// 0xA4
	ssd1306_command(SSD1306_NORMALDISPLAY);// 0xA6
#endif

#if defined SSD1306_128_64
	// Init sequence for 128x64 OLED module
	ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
	ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
	ssd1306_command(0xF0);                                  // the suggested ratio 0x80
	ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
	ssd1306_command(0x3F);
	ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
	ssd1306_command(0x08);// (0x00);//              //was   0x0);                 // no offset

	ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
	ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
	if (vccstate == SSD1306_EXTERNALVCC)
		ssd1306_command(0x10);
	else
		ssd1306_command(0x14);
	ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
	ssd1306_command(0x00);                                  // 0x0 act like ks0108
	ssd1306_command(SSD1306_SEGREMAP | 0x0);
//	ssd1306_command(SSD1306_SEGREMAP | 0x1);
	if (flip_v)
		ssd1306_command(SSD1306_COMSCANDEC);
	else
		ssd1306_command(SSD1306_COMSCANINC);

	ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
	ssd1306_command(0x12);

	ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81

	if (vccstate == SSD1306_EXTERNALVCC)
	{
		ssd1306_command(159);
	}
	else
	{
	ssd1306_command(207);
//		ssd1306_command(1);
  }
	ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
	if (vccstate == SSD1306_EXTERNALVCC)
	{
		ssd1306_command(0x22);
	}
	else
	{
		ssd1306_command(0xF1);
	}
	ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
	ssd1306_command(0x40);											//check this?
	ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
	ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6
#endif

	//Delay(100);
	ssd1306_command(SSD1306_DISPLAYON);                 //--turn on oled panel
	/*
	 */
}

void display_off()
{
	uint8_t tmp;

	ssd1306_command(SSD1306_DISPLAYOFF);
	// bring reset low

	GPIO_ResetBits(GPIOC, GPIO_Pin_15);		//LED_EN
	//GPIO_ResetBits(DISP_CS_PORT, DISP_CS_PIN); //CS;
	GPIO_SetBits(DISP_CS_PORT, DISP_CS_PIN); //CS;
	GPIO_ResetBits(DISP_DC_PORT, DISP_DC_PIN); //D/C
	//OLED_RESET_Write(0);
	// wait for previous writes to complete
//	do
//	{
//		tmp = SPI_OLED_ReadTxStatus();
//		tmp &= SPI_OLED_STS_SPI_DONE;
//	} while (tmp != SPI_OLED_STS_SPI_DONE);
}

void display_on()
{
    // bring out of reset
	//OLED_RESET_Write(1);
	//ssd1306_command(SSD1306_DISPLAYON);

	GPIO_SetBits(GPIOC, GPIO_Pin_15);		//LED_EN
	GPIO_SetBits(DISP_CS_PORT, DISP_CS_PIN); //CS;
	//GPIO_ResetBits(DISP_DC_PORT, DISP_DC_PIN); //D/C

    ssd1306_command(SSD1306_DISPLAYON);
}

void invertDisplay(uint8_t i)
{
	if (i)
	{
		ssd1306_command(SSD1306_INVERTDISPLAY);
	}
	else
	{
		ssd1306_command(SSD1306_NORMALDISPLAY);
	}
}

void ssd1306_command(uint8_t c)
{
	uint8_t tmp;
	uint16_t cnt =0;

	// SPI

	SSD_SPI_start(); //CS
	GPIO_ResetBits(DISP_DC_PORT, DISP_DC_PIN); //D/C

//	for (cnt=0; cnt<100; cnt++)
//	{
//		;
//	}
	send_spi(LCD_SPI, c);
//	for (cnt=0; cnt<100; cnt++)
//	{
//		;
//	}
	SSD_SPI_end(); //CS
//	for (cnt=0; cnt<100; cnt++)
//	{
//		;
//	}
}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void startscrollright(uint8_t start, uint8_t stop)
{
	ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(0XFF);
	ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void startscrollleft(uint8_t start, uint8_t stop)
{
	ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(0XFF);
	ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void startscrolldiagright(uint8_t start, uint8_t stop)
{
	ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
	ssd1306_command(0X00);
	ssd1306_command(SSD1306_LCDHEIGHT);
	ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void startscrolldiagleft(uint8_t start, uint8_t stop)
{
	ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
	ssd1306_command(0X00);
	ssd1306_command(SSD1306_LCDHEIGHT);
	ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

void stopscroll(void)
{
	ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
}

void ssd1306_data(uint8_t c)
{
	uint8_t tmp;

	// SPI

//	GPIO_SetBits(DISP_DC_PORT, DISP_DC_PIN); //D/C
//	SSD_SPI_start(); // CS
	send_spi(LCD_SPI, c);
//	SSD_SPI_end();	 // CS
}

void fill(unsigned char dat)
{
	unsigned char i, j;

	ssd1306_command(0x00);                 //set lower column address
	ssd1306_command(0x10);                 //set higher column address
	ssd1306_command(0xB0);                 //set page address

	for (j = 0; j < 8; j++)
	{
		ssd1306_command(0xB0 + j);                 //set page address
		ssd1306_command(0x00);                 //set lower column address
		ssd1306_command(0x10);                 //set higher column address
		for (i = 0; i < 128; i++)
		{
			ssd1306_data(dat);
		}
	}
}

void display(void)
{
	uint16_t i,cnt=0;
	
	ssd1306_command(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
	ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
	ssd1306_command(SSD1306_SETSTARTLINE | 0x0); // line #0

	// SPI
	GPIO_SetBits(DISP_DC_PORT, DISP_DC_PIN); //D/C
	SSD_SPI_start(); //CS

//	for (cnt=0; cnt<100; cnt++)
//	{
//		;
//	}

	for (i = 0; i < (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT/ 8 ); i++)
	{
		ssd1306_data(bufferX[i]);
	}
	// i wonder why we have to do this (check datasheet)
	if (SSD1306_LCDHEIGHT == 32)
	{
		for (i = 0; i < (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8); i++)
		{
			ssd1306_data(0);
		}
	}

//	for (cnt=0; cnt<100; cnt++)
//	{
//		;
//	}
	SSD_SPI_end(); //CS
//	for (cnt=0; cnt<100; cnt++)
//	{
//		;
//	}
}

// clear everything
void clearDisplay(void)
{
	memset(bufferX, 0, (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8));
}

