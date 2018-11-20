// 
//  Image data for earth
// 



#include "earth.h"
const uint8_t earth[] =
{
	0x0F, 0xE0, //     #######    
	0x3F, 0xF8, //   ###########  
	0x7E, 0xFC, //  ###### ###### 
	0x7E, 0xFC, //  ###### ###### 
	0xFE, 0xFE, // ####### #######
	0xFE, 0xFE, // ####### #######
	0xFE, 0xFE, // ####### #######
	0xC0, 0x06, // ##           ##
	0xFF, 0xFE, // ###############
	0xF0, 0x1E, // ####       ####
	0xFF, 0xFE, // ###############
	0x7C, 0x7C, //  #####   ##### 
	0x7F, 0xFC, //  ############# 
	0x3F, 0xF8, //   ###########  
	0x0F, 0xE0, //     #######    
};

// Bitmap sizes for earth
const uint8_t earthWidthPixels = 15;
const uint8_t earthHeightPixels = 15;


// Image struct containing data //
const IMAGE_INFO earthINFO = 
{
	15, 	//image width pixels
	15, 	//image height pixels
	earth,
};
