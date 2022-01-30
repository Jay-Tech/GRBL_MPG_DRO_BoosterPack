/*
 * graphics.h
 *
 *  Created on: Mar 19, 2012
 *      Author: RobG
 *
 * Modified by Terje Io:
 *  Removed original font handling functions and replaced them with generic functions for freedom of choice
 *  See http://www.mikrocontroller.net/topic/99701#865331
 *
 *  Implemented drawImageMono for images generated by TIs tool
 *  Switched to use RGB colors
 *  Changed to call driver layer via weakly defined functions
 *  Added jpg rendering
 *
 */

#ifndef __GRAPHICS__H_
#define __GRAPHICS__H_

#include <stdint.h>
#include <stdbool.h>

#include "colorRGB.h"
#include "../fonts/font.h"

typedef enum {
    Orientation_Vertical,
    Orientation_Horizontal,
    Orientation_VerticalRotated,
    Orientation_HorizontalRotated
} orientation_t;

typedef enum {
   Align_Left,
   Align_Right,
   Align_Center,
} align_t;

#define DISPLAY_ON true
#define DISPLAY_OFF false

typedef struct {
    uint16_t Width;
    uint16_t Height;
    orientation_t Orientation;
} lcd_display_t;

// NOTE: struct defines for colorRGB565 is not correct for little endian uint16_t, display format is big endian
typedef union {
    uint16_t value;
    struct {
        uint16_t B :5,
                 G :6,
                 R :5;
    };
    struct {
        uint8_t lowByte;
        uint8_t highByte;
    };
} colorRGB565;

typedef struct {
    lcd_display_t display;
    void (*systickCallback)(void);
    void (*touchIRQHandler)(void);
} lcd_driver_t;

void initGraphics (void);
lcd_display_t *getDisplayDescriptor (void);
void setOrientation (orientation_t orientation);
void displayOn (bool on);
void setColor (RGBColor_t color);
void setBackgroundColor (RGBColor_t color);
void delay (uint16_t ms);
bool setSysTickCallback (void (*fn)(void));
//
void clearScreen (bool blackWhite);
void drawPixel (uint16_t x, uint16_t y);
void drawLine (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void drawRect (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void drawCircle (uint16_t x, uint16_t y, uint16_t radius);
//
void fillRect (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void fillCircle (uint16_t x, uint16_t y, uint16_t radius);
//
void drawImage (uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t *data);
void drawImageLut (uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data, uint32_t *lut);
void drawImageMono (uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data);
void drawJPG (const uint16_t x, const uint16_t y, const uint8_t *data, const uint16_t length);
//
uint32_t getPixel (uint16_t x, uint16_t y);
void getPixels (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint32_t *data);

/* new font functions by Terje Io - 2015-08-02 */

uint8_t getFontWidth (Font *font);
uint8_t getFontHeight (Font *font);
uint8_t getCharWidth (Font *font, char c);
uint16_t getStringWidth (Font *font, const char *string);
uint8_t drawChar (Font *font, uint16_t x, uint16_t y, char c, bool opaque);
void drawString (Font *font, uint16_t x, uint16_t y, const char *string, bool opaque);
bool drawStringAligned (Font *font, uint16_t x, uint16_t y, const char *string, align_t align, uint16_t maxWidth, bool opaque);

extern void lcd_panelInit (lcd_driver_t *driver);
extern void lcd_setArea (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
extern void lcd_displayOn (bool on);
extern void lcd_changeOrientation (orientation_t orientation);
extern uint16_t lcd_readID (void);

extern void lcd_driverInit (lcd_driver_t *driver);
extern void lcd_delayms (uint16_t ms);
extern uint32_t lcd_systicks (void);
extern void lcd_writeData (uint8_t data);
extern void lcd_writePixel (colorRGB565 color, uint32_t count);
extern void lcd_writePixels (uint16_t *pixels, uint32_t length);
extern void lcd_writeCommand (uint8_t command);
extern void lcd_readDataBegin (uint8_t command);
extern uint8_t lcd_readData (void);
extern void lcd_readDataEnd (void);
extern void lcd_touchIRQHandler (void);
extern bool lcd_touchIsPenDown (void);
extern uint16_t lcd_touchGetPosition (bool xpos, uint8_t samples);

#endif // __GRAPHICS__H_
