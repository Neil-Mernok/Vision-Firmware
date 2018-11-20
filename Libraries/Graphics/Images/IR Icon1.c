// 
//  Image data for iRIcon1
// 



#include "IR Icon1.h"
const uint8_t iRIcon1[] =
{
	0x10, //    #   
	0x10, //    #   
	0x00, //        
	0x10, //    #   
	0x10, //    #   
	0x00, //        
	0x7C, //  ##### 
	0xC6, // ##   ##
	0x82, // #     #
	0x82, // #     #
	0x82, // #     #
	0xFE, // #######
};

// Bitmap sizes for iRIcon1
const uint8_t iRIcon1WidthPixels = 7;
const uint8_t iRIcon1HeightPixels = 12;


// Image struct containing data //
const IMAGE_INFO iRIcon1INFO = 
{
	7, 	//image width pixels
	12, 	//image height pixels
	iRIcon1,
};