/***********************************
 This is a graphics core library, for use with monochrome displays. 

BSD license
Kobus Goosen, June 2014
 ****************************************/

#ifndef GFX_H
#define GFX_H

//#include "project.h"
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef bool boolean;
//typedef int16 int16_t;
//typedef uint16 uint16_t;
//typedef int8 int8_t;
//typedef uint8 uint8_t;
//typedef int8 boolean;
//typedef uint32 uint32_t;
//typedef int32 int32_t;
    

//#define swap(a, b) { int16_t t = a; a = b; b = t; }
#define abs(x) ((x)>0?(x):-(x)) 
    
// struct definitions
    
    
    
    // struct for holding size and offset of characters in a font. 
    typedef struct 
    {
        uint8_t width;
        uint16_t offset;
    } FONT_CHAR_INFO;
        
    // struct for containing a font information. 
    typedef struct 
    {
        int font_height;
        char start_char;
        char end_char;
        FONT_CHAR_INFO* characters;
        const uint8_t* bitmaps;    
    } FONT_INFO;
    
    // struct to contain bitmap data 
    typedef struct 
    {
        int W;
        int H;
        const uint8_t* bitmapB;    
    } IMAGE_INFO;
    
    
    
    
    
    
    
    
    ///////////////////////////////////////////////////////////////////////////////////
    // Kobus's functions
    int print_text(char* text);
    void print_centered(char* text);
    void print_TextXY(char* text);
    void Setfont(const FONT_INFO *F);
    uint8_t get_font_h(void);
    void drawImage(const IMAGE_INFO *I, int16_t x, int16_t y, uint16_t color);
    void drawCharf(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg);
    uint8_t get_char_width(char c);
    int get_string_width(char* s);
        
    
    ///////////////////////////////////////////////////////////////////////////////////
    
  // this must be defined by the subclass
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  // void invertDisplay(boolean i);

  // these are 'generic' drawing functions, so we can share them!
   void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
		uint16_t color);
   void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
   void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
   void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
   void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
   void fillScreen(uint16_t color);

  void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
  void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);

  void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
  void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);

  void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
  void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);

  void write(uint8_t);

  void setCursor(int16_t x, int16_t y);
  void setTextColor(uint16_t c);
  void setTextColorBG(uint16_t c, uint16_t bg);
  void setTextSize(uint8_t s);
  void setTextWrap(boolean w);

  int16_t get_height(void);
  int16_t get_width(void);

  void setRotation(uint8_t r);
  uint8_t getRotation(void);

    extern int16_t  WIDTH, HEIGHT;   // this is the 'raw' display w/h - never changes
    extern int16_t  _width, _height; // dependent on rotation
    extern int16_t  cursor_x, cursor_y;
    extern uint16_t textcolor, textbgcolor;
    extern uint8_t  textsize;
    extern uint8_t  rotation;
    extern boolean  wrap; // If set, 'wrap' text at right edge of display
    
    /////////////////////////////////////////////////////////////
    extern FONT_INFO Font;
    /////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif
#endif
