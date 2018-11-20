// 
//  Image data for lightninge
// 
#include "lightning_icon.h" 

const uint8_t lightningeBitmaps[] =
{
	0x07, 0xC0, //      #####     
	0x1F, 0xF0, //    #########   
	0x3E, 0x38, //   #####   ###  
	0x7E, 0x7C, //  ######  ##### 
	0x7C, 0x7C, //  #####   ##### 
	0xFC, 0xFE, // ######  #######
	0xF9, 0xFE, // #####  ########
	0xF8, 0x3E, // #####     #####
	0xFF, 0x3E, // ########  #####
	0xFE, 0x7E, // #######  ######
	0x7E, 0xFC, //  ###### ###### 
	0x7D, 0xFC, //  ##### ####### 
	0x3F, 0xF8, //   ###########  
	0x1F, 0xF0, //    #########   
	0x07, 0xC0, //      #####     
};

// Bitmap sizes for lightninge
const uint8_t lightningeWidthPixels = 15;
const uint8_t lightningeHeightPixels = 15;


// Image struct containing data //
const IMAGE_INFO lightningeINFO =
{
	15, 	//image width pixels
	15, 	//image height pixels
	lightningeBitmaps,
};

