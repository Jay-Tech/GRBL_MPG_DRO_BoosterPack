/*
    created with FontEditor written by H. Reddmann
    HaReddmann at t-online dot de

    Template by Terje Io

    File Name           : font23x16.h
    Date                : 02.07.2018
    Font size in bytes  : 0x1346, 4934
    Font width          : 23
    Font height         : 16
    Font first char     : 0x21
    Font last char      : 0xFF
    Font bits per pixel : 1
    Font is compressed  : false

    The font data are defined as

    struct _FONT_ {
     // common shared fields
       uint16_t   font_Size_in_Bytes_over_all_included_Size_it_self;
       uint8_t    font_Width_in_Pixel_for_fixed_drawing;
       uint8_t    font_Height_in_Pixel_for_all_Characters;
       uint8_t    font_Bits_per_Pixels;
                    // if MSB are set then font is a compressed font
       uint8_t    font_First_Char;
       uint8_t    font_Last_Char;
       uint8_t    font_Char_Widths[font_Last_Char - font_First_Char +1];
                    // for each character the separate width in pixels,
                    // characters < 128 have an implicit virtual right empty row
                    // characters with font_Char_Widths[] == 0 are undefined

     // if compressed font then additional fields
       uint8_t    font_Byte_Padding;
                    // each Char in the table are aligned in size to this value
       uint8_t    font_RLE_Table[3];
                    // Run Length Encoding Table for compression
       uint8_t    font_Char_Size_in_Bytes[font_Last_Char - font_First_Char +1];
                    // for each char the size in (bytes / font_Byte_Padding) are stored,
                    // this get us the table to seek to the right beginning of each char
                    // in the font_data[].

     // for compressed and uncompressed fonts
       uint8_t    font_data[];
                    // bit field of all characters
    }
*/

#ifndef __font23x16_h__
#define __font23x16_h__

#ifndef __font_h__
#include "font.h"
#endif

#define font23x16_width 23
#define font23x16_height 16

#define font_23x16 (Font*)font_23x16_data

const uint8_t font_23x16_data[] = {
    0x13, 0x46, 0x17, 0x10, 0x01, 0x21, 0xFF,
    0x02, 0x06, 0x0C, 0x0A, 0x16, 0x0A, 0x04, 0x06, 0x06, 0x0A, 0x0A, 0x04, 0x0A, 0x02, 0x08, 0x0A,
    0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x02, 0x04, 0x08, 0x0A, 0x08, 0x0A, 0x0C,
    0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x06, 0x09, 0x0A, 0x0A, 0x0E, 0x0A, 0x0A, 0x0A,
    0x0B, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0E, 0x0A, 0x0A, 0x0A, 0x06, 0x08, 0x06, 0x0A, 0x08, 0x04,
    0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x08, 0x0A, 0x0A, 0x06, 0x09, 0x09, 0x06, 0x0E, 0x0A, 0x0A, 0x0A,
    0x0A, 0x09, 0x0A, 0x08, 0x09, 0x0A, 0x0E, 0x0A, 0x0A, 0x0A, 0x0A, 0x02, 0x0A, 0x0C, 0x05, 0x0E,
    0x00, 0x06, 0x0A, 0x0A, 0x10, 0x0C, 0x0C, 0x0C, 0x22, 0x0C, 0x08, 0x13, 0x00, 0x0C, 0x00, 0x00,
    0x06, 0x06, 0x0A, 0x0A, 0x08, 0x0A, 0x12, 0x0E, 0x14, 0x0C, 0x08, 0x14, 0x00, 0x0C, 0x0C, 0x00,
    0x04, 0x0C, 0x0E, 0x0C, 0x0C, 0x04, 0x07, 0x0A, 0x12, 0x07, 0x0E, 0x0C, 0x0A, 0x12, 0x0A, 0x0A,
    0x0C, 0x09, 0x09, 0x06, 0x0E, 0x0E, 0x04, 0x08, 0x12, 0x12, 0x0E, 0x12, 0x12, 0x12, 0x0C, 0x0C,
    0x0C, 0x0C, 0x0E, 0x0C, 0x0C, 0x14, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x08, 0x08, 0x08, 0x0A, 0x0C,
    0x0E, 0x0C, 0x0C, 0x0C, 0x0E, 0x0C, 0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
    0x0C, 0x0C, 0x0E, 0x0C, 0x0C, 0x14, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x08, 0x08, 0x08, 0x08, 0x0E,
    0x0E, 0x0C, 0x0C, 0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
    0xFF, 0x33, 0xFF, 0x33, 0x3F, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x3F, 0x00,
    0x30, 0x0C, 0x30, 0x0C, 0xFC, 0x3F, 0xFC, 0x3F, 0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C,
    0xFC, 0x3F, 0xFC, 0x3F, 0x30, 0x0C, 0x30, 0x0C, 0x70, 0x0C, 0xF8, 0x0C, 0xDC, 0x0C, 0xCC, 0x0C,
    0xFF, 0x3F, 0xFF, 0x3F, 0xCC, 0x0C, 0xCC, 0x0E, 0xCC, 0x07, 0x8C, 0x03, 0x3C, 0x00, 0x7E, 0x00,
    0xE7, 0x00, 0xC3, 0x20, 0xC3, 0x30, 0xE7, 0x38, 0x7E, 0x1C, 0x3C, 0x0E, 0x00, 0x07, 0x80, 0x03,
    0xC0, 0x01, 0xE0, 0x00, 0x70, 0x00, 0x38, 0x00, 0x1C, 0x0F, 0x8E, 0x1F, 0xC7, 0x39, 0xC3, 0x30,
    0xC1, 0x30, 0xC0, 0x39, 0x80, 0x1F, 0x00, 0x0F, 0x3C, 0x0F, 0xFE, 0x1F, 0xE7, 0x39, 0xC3, 0x30,
    0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC7, 0x38, 0xCE, 0x1F, 0xCC, 0x0F, 0x30, 0x00, 0x38, 0x00,
    0x1F, 0x00, 0x0F, 0x00, 0xF0, 0x03, 0xF8, 0x07, 0x1C, 0x0E, 0x0E, 0x1C, 0x07, 0x38, 0x03, 0x30,
    0x03, 0x30, 0x07, 0x38, 0x0E, 0x1C, 0x1C, 0x0E, 0xF8, 0x07, 0xF0, 0x03, 0x30, 0x03, 0x30, 0x03,
    0xE0, 0x01, 0xC0, 0x00, 0xFC, 0x0F, 0xFC, 0x0F, 0xC0, 0x00, 0xE0, 0x01, 0x30, 0x03, 0x30, 0x03,
    0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xFC, 0x0F, 0xFC, 0x0F, 0xC0, 0x00, 0xC0, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0x00, 0xC0, 0x00, 0xE0, 0x00, 0x7C, 0x00, 0x3C, 0x00, 0x00, 0xC0, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x00, 0x30, 0x00, 0xE0, 0x00, 0xF8, 0x00, 0x1E, 0x80, 0x07, 0xE0, 0x01, 0x78, 0x00,
    0x1F, 0x00, 0x07, 0x00, 0xFC, 0x0F, 0xFE, 0x1F, 0x07, 0x3F, 0x83, 0x33, 0xC3, 0x31, 0xE3, 0x30,
    0x73, 0x30, 0x3F, 0x38, 0xFE, 0x1F, 0xFC, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x30, 0x0C, 0x30,
    0xFF, 0x3F, 0xFF, 0x3F, 0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x30, 0x3E, 0x38,
    0x07, 0x3C, 0x03, 0x3E, 0x03, 0x37, 0x83, 0x33, 0xC3, 0x31, 0xE7, 0x30, 0x7E, 0x30, 0x3C, 0x30,
    0x1C, 0x0E, 0x1E, 0x1E, 0x07, 0x38, 0x03, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC7, 0x38,
    0xFE, 0x1F, 0x3C, 0x0F, 0xC0, 0x03, 0xE0, 0x03, 0x70, 0x03, 0x38, 0x03, 0x1C, 0x03, 0x0E, 0x03,
    0x07, 0x03, 0xFF, 0x3F, 0xFF, 0x3F, 0x00, 0x03, 0x7F, 0x0E, 0x7F, 0x1E, 0x63, 0x38, 0x63, 0x30,
    0x63, 0x30, 0x63, 0x30, 0x63, 0x30, 0xE3, 0x38, 0xC3, 0x1F, 0x83, 0x0F, 0xFC, 0x0F, 0xFE, 0x1F,
    0xC7, 0x39, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC7, 0x39, 0x8E, 0x1F, 0x0C, 0x0F,
    0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x3E, 0x83, 0x3F, 0xE3, 0x01, 0x7B, 0x00,
    0x1F, 0x00, 0x0F, 0x00, 0x3C, 0x0F, 0xFE, 0x1F, 0xE7, 0x39, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30,
    0xC3, 0x30, 0xE7, 0x39, 0xFE, 0x1F, 0x3C, 0x0F, 0x3C, 0x0C, 0x7E, 0x1C, 0xE7, 0x38, 0xC3, 0x30,
    0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xE7, 0x38, 0xFE, 0x1F, 0xFC, 0x0F, 0x30, 0x30, 0x30, 0x30,
    0x00, 0xC0, 0x00, 0xE0, 0x30, 0x7C, 0x30, 0x3C, 0xC0, 0x00, 0xE0, 0x01, 0xF0, 0x03, 0x38, 0x07,
    0x1C, 0x0E, 0x0E, 0x1C, 0x07, 0x38, 0x03, 0x30, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03,
    0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x03, 0x30, 0x07, 0x38,
    0x0E, 0x1C, 0x1C, 0x0E, 0x38, 0x07, 0xF0, 0x03, 0xE0, 0x01, 0xC0, 0x00, 0x3C, 0x00, 0x3E, 0x00,
    0x07, 0x00, 0x03, 0x00, 0x03, 0x33, 0x83, 0x33, 0xC3, 0x01, 0xE7, 0x00, 0x7E, 0x00, 0x3C, 0x00,
    0xFC, 0x3F, 0xFE, 0x7F, 0x07, 0xE0, 0x03, 0xC0, 0xC3, 0xC3, 0xE3, 0xC7, 0x73, 0xCE, 0x33, 0xCC,
    0x33, 0xCC, 0x77, 0xCE, 0xFE, 0xC7, 0xFC, 0xC3, 0xFC, 0x3F, 0xFE, 0x3F, 0x87, 0x01, 0x83, 0x01,
    0x83, 0x01, 0x83, 0x01, 0x83, 0x01, 0x87, 0x01, 0xFE, 0x3F, 0xFC, 0x3F, 0xFF, 0x3F, 0xFF, 0x3F,
    0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xFE, 0x1F, 0x3C, 0x0F,
    0xFC, 0x0F, 0xFE, 0x1F, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30,
    0x1E, 0x1E, 0x1C, 0x0E, 0xFF, 0x3F, 0xFF, 0x3F, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30,
    0x03, 0x30, 0x03, 0x30, 0xFE, 0x1F, 0xFC, 0x0F, 0xFF, 0x3F, 0xFF, 0x3F, 0xC3, 0x30, 0xC3, 0x30,
    0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0x03, 0x30, 0x03, 0x30, 0xFF, 0x3F, 0xFF, 0x3F,
    0xC3, 0x00, 0xC3, 0x00, 0xC3, 0x00, 0xC3, 0x00, 0xC3, 0x00, 0xC3, 0x00, 0x03, 0x00, 0x03, 0x00,
    0xFC, 0x0F, 0xFE, 0x1F, 0x07, 0x38, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x83, 0x31, 0x87, 0x39,
    0x9E, 0x1F, 0x9C, 0x0F, 0xFF, 0x3F, 0xFF, 0x3F, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0xFF, 0x3F, 0xFF, 0x3F, 0x03, 0x30, 0x03, 0x30, 0xFF, 0x3F, 0xFF, 0x3F,
    0x03, 0x30, 0x03, 0x30, 0x00, 0x0F, 0x00, 0x1F, 0x00, 0x38, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30,
    0x03, 0x38, 0xFF, 0x1F, 0xFF, 0x0F, 0xFF, 0x3F, 0xFF, 0x3F, 0xC0, 0x00, 0xE0, 0x01, 0xF0, 0x03,
    0x38, 0x07, 0x1C, 0x0E, 0x0E, 0x1C, 0x07, 0x38, 0x03, 0x30, 0xFF, 0x3F, 0xFF, 0x3F, 0x00, 0x30,
    0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0xFF, 0x3F,
    0xFF, 0x3F, 0x0E, 0x00, 0x1C, 0x00, 0x38, 0x00, 0x70, 0x00, 0xE0, 0x00, 0xE0, 0x00, 0x70, 0x00,
    0x38, 0x00, 0x1C, 0x00, 0x0E, 0x00, 0xFF, 0x3F, 0xFF, 0x3F, 0xFF, 0x3F, 0xFF, 0x3F, 0x38, 0x00,
    0x70, 0x00, 0xE0, 0x00, 0xC0, 0x01, 0x80, 0x03, 0x00, 0x07, 0xFF, 0x3F, 0xFF, 0x3F, 0xFC, 0x0F,
    0xFE, 0x1F, 0x07, 0x38, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x07, 0x38, 0xFE, 0x1F,
    0xFC, 0x0F, 0xFF, 0x3F, 0xFF, 0x3F, 0x83, 0x01, 0x83, 0x01, 0x83, 0x01, 0x83, 0x01, 0x83, 0x01,
    0xC7, 0x01, 0xFE, 0x00, 0x7C, 0x00, 0xFC, 0x0F, 0xFE, 0x1F, 0x07, 0x38, 0x03, 0x30, 0x03, 0x30,
    0x03, 0x30, 0x03, 0x30, 0x07, 0x78, 0xFE, 0xFF, 0xFC, 0xCF, 0x00, 0xC0, 0xFF, 0x3F, 0xFF, 0x3F,
    0x83, 0x01, 0x83, 0x01, 0x83, 0x01, 0x83, 0x01, 0x83, 0x01, 0xC7, 0x03, 0xFE, 0x3F, 0x3C, 0x3E,
    0x3C, 0x0C, 0x7E, 0x1C, 0xE7, 0x38, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC7, 0x38,
    0x8E, 0x1F, 0x0C, 0x0F, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0xFF, 0x3F, 0xFF, 0x3F,
    0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0xFF, 0x0F, 0xFF, 0x1F, 0x00, 0x38, 0x00, 0x30,
    0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x38, 0xFF, 0x1F, 0xFF, 0x0F, 0x3F, 0x00, 0x7F, 0x00,
    0xE0, 0x03, 0xC0, 0x07, 0x00, 0x3C, 0x00, 0x3C, 0xC0, 0x07, 0xE0, 0x03, 0x7F, 0x00, 0x3F, 0x00,
    0xFF, 0x03, 0xFF, 0x07, 0x00, 0x3C, 0x00, 0x3C, 0xC0, 0x07, 0xE0, 0x03, 0x3C, 0x00, 0x3C, 0x00,
    0xE0, 0x03, 0xC0, 0x07, 0x00, 0x3C, 0x00, 0x3C, 0xFF, 0x07, 0xFF, 0x03, 0x0F, 0x3C, 0x1F, 0x3E,
    0x38, 0x07, 0xF0, 0x03, 0xE0, 0x01, 0xE0, 0x01, 0xF0, 0x03, 0x38, 0x07, 0x1F, 0x3E, 0x0F, 0x3C,
    0x3F, 0x00, 0x7F, 0x00, 0xE0, 0x00, 0xC0, 0x01, 0x80, 0x3F, 0x80, 0x3F, 0xC0, 0x01, 0xE0, 0x00,
    0x7F, 0x00, 0x3F, 0x00, 0x03, 0x3C, 0x03, 0x3E, 0x03, 0x37, 0x83, 0x33, 0xC3, 0x31, 0xE3, 0x30,
    0x73, 0x30, 0x3B, 0x30, 0x1F, 0x30, 0x0F, 0x30, 0xFF, 0x3F, 0xFF, 0x3F, 0x03, 0x30, 0x03, 0x30,
    0x03, 0x30, 0x03, 0x30, 0x07, 0x00, 0x1F, 0x00, 0x78, 0x00, 0xE0, 0x01, 0x80, 0x07, 0x00, 0x1E,
    0x00, 0xF8, 0x00, 0xE0, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0xFF, 0x3F, 0xFF, 0x3F,
    0x30, 0x00, 0x38, 0x00, 0x1C, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x07, 0x00, 0x0E, 0x00, 0x1C, 0x00,
    0x38, 0x00, 0x30, 0x00, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0,
    0x00, 0xC0, 0x00, 0xC0, 0x03, 0x00, 0x07, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x00, 0x0C, 0x00, 0x1E,
    0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x70, 0x33, 0xE0, 0x3F, 0xC0, 0x3F,
    0xFF, 0x3F, 0xFF, 0x3F, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x70, 0x38,
    0xE0, 0x1F, 0xC0, 0x0F, 0xC0, 0x0F, 0xE0, 0x1F, 0x70, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x70, 0x38, 0xE0, 0x1C, 0xC0, 0x0C, 0xC0, 0x0F, 0xE0, 0x1F, 0x70, 0x38, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0xFF, 0x3F, 0xFF, 0x3F, 0xC0, 0x0F, 0xE0, 0x1F,
    0x70, 0x3B, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x70, 0x33, 0xE0, 0x33, 0xC0, 0x03,
    0x30, 0x00, 0x30, 0x00, 0xFC, 0x3F, 0xFE, 0x3F, 0x37, 0x00, 0x33, 0x00, 0x33, 0x00, 0x33, 0x00,
    0xC0, 0x03, 0xE0, 0xC7, 0x70, 0xCE, 0x30, 0xCC, 0x30, 0xCC, 0x30, 0xCC, 0x30, 0xCC, 0x30, 0xEC,
    0xF0, 0x7F, 0xF0, 0x3F, 0xFF, 0x3F, 0xFF, 0x3F, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
    0x30, 0x00, 0x70, 0x00, 0xE0, 0x3F, 0xC0, 0x3F, 0x30, 0x30, 0x30, 0x30, 0xF3, 0x3F, 0xF3, 0x3F,
    0x00, 0x30, 0x00, 0x30, 0x00, 0x3C, 0x00, 0x7C, 0x00, 0xE0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0,
    0x30, 0xE0, 0xF3, 0x7F, 0xF3, 0x3F, 0xFF, 0x3F, 0xFF, 0x3F, 0x00, 0x03, 0x00, 0x03, 0x80, 0x07,
    0xC0, 0x0F, 0xE0, 0x1C, 0x70, 0x38, 0x30, 0x30, 0x03, 0x30, 0x03, 0x30, 0xFF, 0x3F, 0xFF, 0x3F,
    0x00, 0x30, 0x00, 0x30, 0xF0, 0x3F, 0xF0, 0x3F, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x70, 0x00,
    0xE0, 0x3F, 0xE0, 0x3F, 0x70, 0x00, 0x30, 0x00, 0x30, 0x00, 0x70, 0x00, 0xE0, 0x3F, 0xC0, 0x3F,
    0xF0, 0x3F, 0xF0, 0x3F, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x70, 0x00,
    0xE0, 0x3F, 0xC0, 0x3F, 0xC0, 0x0F, 0xE0, 0x1F, 0x70, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x70, 0x38, 0xE0, 0x1F, 0xC0, 0x0F, 0xF0, 0xFF, 0xF0, 0xFF, 0x30, 0x0C, 0x30, 0x0C,
    0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C, 0x70, 0x0E, 0xE0, 0x07, 0xC0, 0x03, 0xC0, 0x03, 0xE0, 0x07,
    0x70, 0x0E, 0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C, 0xF0, 0xFF, 0xF0, 0xFF,
    0xF0, 0x3F, 0xF0, 0x3F, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x70, 0x00, 0xE0, 0x01,
    0xC0, 0x01, 0xC0, 0x30, 0xE0, 0x31, 0x70, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33,
    0x30, 0x37, 0x30, 0x1E, 0x30, 0x0C, 0x30, 0x00, 0x30, 0x00, 0xFC, 0x0F, 0xFC, 0x1F, 0x30, 0x38,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0xF0, 0x0F, 0xF0, 0x1F, 0x00, 0x38, 0x00, 0x30, 0x00, 0x30,
    0x00, 0x30, 0x00, 0x30, 0xF0, 0x3F, 0xF0, 0x3F, 0xF0, 0x03, 0xF0, 0x07, 0x00, 0x0E, 0x00, 0x1C,
    0x00, 0x38, 0x00, 0x38, 0x00, 0x1C, 0x00, 0x0E, 0xF0, 0x07, 0xF0, 0x03, 0xF0, 0x0F, 0xF0, 0x1F,
    0x00, 0x38, 0x00, 0x30, 0x00, 0x30, 0x00, 0x38, 0xC0, 0x1F, 0xC0, 0x1F, 0x00, 0x38, 0x00, 0x30,
    0x00, 0x30, 0x00, 0x38, 0xF0, 0x1F, 0xF0, 0x0F, 0x30, 0x30, 0x70, 0x38, 0xE0, 0x1C, 0xC0, 0x0F,
    0x80, 0x07, 0x80, 0x07, 0xC0, 0x0F, 0xE0, 0x1C, 0x70, 0x38, 0x30, 0x30, 0xF0, 0x03, 0xF0, 0x07,
    0x00, 0xCE, 0x00, 0xCC, 0x00, 0xCC, 0x00, 0xCC, 0x00, 0xCC, 0x00, 0xEE, 0xF0, 0x7F, 0xF0, 0x3F,
    0x30, 0x30, 0x30, 0x38, 0x30, 0x3C, 0x30, 0x3E, 0x30, 0x37, 0xB0, 0x33, 0xF0, 0x31, 0xF0, 0x30,
    0x70, 0x30, 0x30, 0x30, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xE0, 0x01, 0xFC, 0x0F, 0x3E, 0x1F,
    0x07, 0x38, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0xFF, 0x3F, 0xFF, 0x3F, 0x03, 0x30, 0x03, 0x30,
    0x03, 0x30, 0x07, 0x38, 0x3E, 0x1F, 0xFC, 0x0F, 0xE0, 0x01, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00,
    0x0C, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x03, 0x00, 0x03, 0x00, 0x07, 0x00, 0x0E, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x03, 0x00, 0xFC, 0x7F, 0x04, 0x40, 0x04, 0x40, 0x04, 0x40,
    0xFC, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x30, 0x03, 0x30, 0x03, 0xFC, 0x0F, 0xFE, 0x1F, 0x37, 0x3B,
    0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x03, 0x30, 0x07, 0x38, 0x1E, 0x1E, 0x1C, 0x0E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xC0, 0x00, 0xE0, 0x00, 0x7C, 0x00, 0x3C, 0x00, 0xC0, 0x00, 0xC0, 0x30, 0xC0,
    0x30, 0xE0, 0xFC, 0x7F, 0xFE, 0x3F, 0x37, 0x00, 0x33, 0x00, 0x33, 0x00, 0x33, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xC0, 0x00, 0xE0, 0x00, 0x7C, 0x00, 0x3C, 0x00, 0xC0, 0x00, 0xE0, 0x00, 0x7C,
    0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0xFF, 0xFF,
    0xFF, 0xFF, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x0C,
    0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C, 0xFF, 0xFF, 0xFF, 0xFF, 0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C,
    0x30, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x38, 0x00, 0x1C, 0x00, 0x0E, 0x00, 0x07, 0x00,
    0x07, 0x00, 0x0E, 0x00, 0x1C, 0x00, 0x38, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00,
    0x7E, 0x00, 0xE7, 0x00, 0xC3, 0x00, 0xC3, 0x30, 0xE7, 0x38, 0x7E, 0x1C, 0x3C, 0x0E, 0x00, 0x07,
    0x80, 0x03, 0xC0, 0x01, 0xE0, 0x00, 0x70, 0x00, 0x38, 0x00, 0x1C, 0x0F, 0x8E, 0x1F, 0xC7, 0x39,
    0xC3, 0x30, 0xC0, 0x30, 0xC0, 0x39, 0x80, 0x1F, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,
    0x80, 0x1F, 0xC0, 0x39, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x39, 0x80, 0x1F, 0x00, 0x0F, 0x00, 0x00,
    0x00, 0x00, 0xC3, 0x30, 0xE7, 0x31, 0xBE, 0x33, 0x3C, 0x33, 0x38, 0x33, 0x38, 0x33, 0x3C, 0x33,
    0x3E, 0x37, 0x37, 0x1E, 0x33, 0x0C, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xE0, 0x01, 0xF0, 0x03,
    0x38, 0x07, 0x1C, 0x0E, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x0F, 0xFE, 0x1F, 0x07, 0x38,
    0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x03, 0x30, 0x07, 0x38, 0xFE, 0x1F, 0xFF, 0x3F, 0xC3, 0x30,
    0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0x03, 0x30, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00,
    0x33, 0x30, 0x37, 0x38, 0x3E, 0x3C, 0x3C, 0x3E, 0x38, 0x37, 0xB8, 0x33, 0xFC, 0x31, 0xFE, 0x30,
    0x77, 0x30, 0x33, 0x30, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x3E, 0x00, 0x07, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x38, 0x00, 0x1F, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3C, 0x00, 0x3E, 0x00, 0x07, 0x00, 0x03, 0x00, 0x3C, 0x00, 0x3E, 0x00, 0x07, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x38, 0x00, 0x1F, 0x00, 0x0F, 0x00, 0x30, 0x00, 0x38, 0x00,
    0x1F, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x03, 0xF0, 0x03, 0xF0, 0x03, 0xF0, 0x03,
    0xF0, 0x03, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xC0, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0C, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x03, 0x00, 0x03, 0x00, 0x07, 0x00, 0x0E, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0x0E, 0x00, 0x1C, 0x00, 0x38, 0x00, 0x38, 0x00, 0x1C, 0x00, 0x0E, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC1, 0x30, 0xE3, 0x31, 0x76, 0x33, 0x3C, 0x33, 0x38, 0x33, 0x38, 0x33,
    0x3C, 0x33, 0x36, 0x37, 0x33, 0x1E, 0x31, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x1C, 0x0E,
    0x38, 0x07, 0xF0, 0x03, 0xE0, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xE0, 0x1F,
    0x70, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x70, 0x38, 0xE0, 0x1F, 0xE0, 0x1F,
    0x70, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0xE0, 0x33, 0xC0, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x31, 0x30, 0x33, 0x38, 0x36, 0x3C, 0x3C, 0x3E, 0x38, 0x37, 0xB8, 0x33,
    0xFC, 0x31, 0xF6, 0x30, 0x73, 0x30, 0x31, 0x30, 0x00, 0x00, 0x00, 0x00, 0xF3, 0x00, 0xF3, 0x01,
    0x80, 0x03, 0x00, 0x07, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x07, 0x80, 0x03, 0xF3, 0x01, 0xF3, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xCC, 0xFF, 0xCC, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x03, 0xF8, 0x07,
    0x1C, 0x0E, 0x0C, 0x0C, 0xFF, 0x3F, 0xFF, 0x3F, 0x0C, 0x0C, 0x1C, 0x0E, 0x38, 0x07, 0x30, 0x03,
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x30, 0xC0, 0x30, 0xFC, 0x3F, 0xFE, 0x3F, 0xC7, 0x30, 0xC3, 0x30,
    0xC3, 0x30, 0xC3, 0x30, 0x03, 0x30, 0x07, 0x38, 0x1E, 0x1E, 0x1C, 0x0E, 0x0C, 0x30, 0xFC, 0x3F,
    0xF8, 0x1F, 0x38, 0x1C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x38, 0x1C, 0xF8, 0x1F,
    0xFC, 0x3F, 0x0C, 0x30, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x19, 0x9F, 0x19, 0xB8, 0x19, 0xF0, 0x19,
    0xE0, 0xFF, 0xE0, 0xFF, 0xF0, 0x19, 0xB8, 0x19, 0x9F, 0x19, 0x8F, 0x19, 0x00, 0x00, 0x00, 0x00,
    0x3F, 0x3F, 0x3F, 0x3F, 0x00, 0x00, 0xE0, 0x21, 0x3C, 0x63, 0x7E, 0x66, 0xE6, 0x6C, 0x86, 0x79,
    0x06, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0xF8, 0x1F, 0x1C, 0x38,
    0x0E, 0x70, 0x07, 0xE0, 0xC3, 0xC3, 0xE3, 0xC7, 0x73, 0xCE, 0x33, 0xCC, 0x33, 0xCC, 0x33, 0xCC,
    0x03, 0xE0, 0x0E, 0x70, 0x1C, 0x38, 0xF8, 0x1F, 0xF0, 0x0F, 0x00, 0x00, 0xC4, 0x19, 0xE6, 0x1B,
    0x66, 0x1B, 0x66, 0x1B, 0x7E, 0x1B, 0xFC, 0x19, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xE0, 0x01,
    0xF0, 0x03, 0x38, 0x07, 0x1C, 0x0E, 0x0C, 0x0C, 0xC0, 0x00, 0xE0, 0x01, 0xF0, 0x03, 0x38, 0x07,
    0x1C, 0x0E, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x0F, 0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0xF8, 0x1F, 0x1C, 0x38, 0x0E, 0x70, 0xF7, 0xEF, 0xF3, 0xCF,
    0x33, 0xC3, 0x33, 0xC3, 0x33, 0xC3, 0x33, 0xC3, 0xE3, 0xCF, 0xC7, 0xEC, 0x0E, 0x70, 0x1C, 0x38,
    0xF8, 0x1F, 0xF0, 0x0F, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00,
    0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x7E, 0x00,
    0xE7, 0x00, 0xC3, 0x00, 0xC3, 0x00, 0xE7, 0x00, 0x7E, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0xFF, 0x33, 0xFF, 0x33, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0xC6, 0x00, 0xE2, 0x00,
    0xB2, 0x00, 0x9E, 0x00, 0x8C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0xD6, 0x00,
    0x92, 0x00, 0x92, 0x00, 0xD6, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0E, 0x00,
    0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xF0, 0xFF, 0x00, 0x38, 0x00, 0x30,
    0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x38, 0xF0, 0x1F, 0xF0, 0x3F, 0x00, 0x70, 0x00, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x7E, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x3F,
    0xFF, 0x3F, 0x03, 0x00, 0x03, 0x00, 0xFF, 0x3F, 0xFF, 0x3F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xCC, 0x00, 0xFC,
    0x00, 0x78, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0xFF, 0xCF, 0xFF,
    0xC3, 0xFF, 0xC3, 0xFF, 0xF3, 0xCC, 0xF3, 0xCC, 0x33, 0xCC, 0x33, 0xCC, 0x03, 0xFF, 0x03, 0xFF,
    0xCF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    0xCF, 0xFF, 0xCF, 0xFF, 0xC3, 0xFF, 0xC3, 0xFF, 0xF3, 0xCC, 0xF3, 0xCC, 0x33, 0xCC, 0x33, 0xCC,
    0x03, 0xFF, 0x03, 0xFF, 0xCF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
    0x0C, 0x0C, 0x1C, 0x0E, 0x38, 0x07, 0xF0, 0x03, 0xE0, 0x01, 0xC0, 0x00, 0x0C, 0x0C, 0x1C, 0x0E,
    0x38, 0x07, 0x70, 0x03, 0xE0, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    0xCF, 0xFF, 0xCF, 0xFF, 0xC3, 0xFF, 0xC3, 0xFF, 0xF3, 0xCC, 0xF3, 0xCC, 0x33, 0xCC, 0x33, 0xCC,
    0x03, 0xFF, 0x03, 0xFF, 0xCF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0xFF, 0xCF, 0xFF, 0xC3, 0xFF, 0xC3, 0xFF, 0xF3, 0xCC, 0xF3, 0xCC,
    0x33, 0xCC, 0x33, 0xCC, 0x03, 0xFF, 0x03, 0xFF, 0xCF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0xFF, 0xCF, 0xFF, 0xC3, 0xFF, 0xC3, 0xFF,
    0xF3, 0xCC, 0xF3, 0xCC, 0x33, 0xCC, 0x33, 0xCC, 0x03, 0xFF, 0x03, 0xFF, 0xCF, 0xFF, 0xCF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x7E, 0x00, 0xE7, 0x80, 0xC3,
    0xCC, 0xC1, 0xCC, 0xC0, 0x00, 0xC0, 0x00, 0xE0, 0x00, 0x7C, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x3F, 0xE1, 0x3F, 0x73, 0x06, 0x37, 0x06, 0x36, 0x06, 0x34, 0x06, 0x30, 0x06, 0x70, 0x06,
    0xE0, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0xE0, 0x3F, 0x70, 0x06, 0x30, 0x06,
    0x34, 0x06, 0x36, 0x06, 0x37, 0x06, 0x73, 0x06, 0xE1, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x3F, 0xE0, 0x3F, 0x74, 0x06, 0x36, 0x06, 0x33, 0x06, 0x33, 0x06, 0x36, 0x06, 0x74, 0x06,
    0xE0, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xCC, 0x3F, 0xEE, 0x3F, 0x77, 0x06, 0x33, 0x06,
    0x33, 0x06, 0x37, 0x06, 0x3E, 0x06, 0x7C, 0x06, 0xEC, 0x3F, 0xCE, 0x3F, 0x07, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0xE3, 0x3F, 0x73, 0x06, 0x30, 0x06, 0x30, 0x06, 0x30, 0x06,
    0x30, 0x06, 0x73, 0x06, 0xE3, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3F, 0xE0, 0x3F,
    0x72, 0x06, 0x37, 0x06, 0x35, 0x06, 0x35, 0x06, 0x37, 0x06, 0x72, 0x06, 0xE0, 0x3F, 0xC0, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0xFC, 0x3F, 0xFE, 0x3F, 0x07, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0xFF, 0x3F, 0xFF, 0x3F, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30, 0xC3, 0x30,
    0xC3, 0x30, 0xC3, 0x30, 0x03, 0x30, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0xFE, 0x01,
    0x87, 0xC3, 0x03, 0xC3, 0x03, 0xCF, 0x03, 0xDF, 0x03, 0x7B, 0x87, 0x33, 0xCE, 0x01, 0xCC, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, 0xF1, 0x3F, 0x33, 0x33, 0x37, 0x33, 0x36, 0x33, 0x34, 0x33,
    0x30, 0x33, 0x30, 0x33, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, 0xF0, 0x3F,
    0x30, 0x33, 0x30, 0x33, 0x34, 0x33, 0x36, 0x33, 0x37, 0x33, 0x33, 0x33, 0x31, 0x30, 0x30, 0x30,
    0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, 0xF4, 0x3F, 0x36, 0x33, 0x37, 0x33, 0x33, 0x33, 0x33, 0x33,
    0x37, 0x33, 0x36, 0x33, 0x34, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, 0xF3, 0x3F,
    0x33, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x33, 0x33, 0x33, 0x30, 0x30, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x31, 0x30, 0x33, 0x30, 0xF7, 0x3F, 0xF6, 0x3F, 0x34, 0x30, 0x30, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x34, 0x30, 0xF6, 0x3F, 0xF7, 0x3F, 0x33, 0x30, 0x31, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x34, 0x30, 0x36, 0x30, 0xF3, 0x3F, 0xF3, 0x3F, 0x36, 0x30, 0x34, 0x30,
    0x03, 0x00, 0x03, 0x00, 0x30, 0x30, 0x30, 0x30, 0xF0, 0x3F, 0xF0, 0x3F, 0x30, 0x30, 0x30, 0x30,
    0x03, 0x00, 0x03, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xFF, 0x3F, 0xFF, 0x3F, 0xC3, 0x30, 0xC3, 0x30,
    0xC3, 0x30, 0xC3, 0x30, 0x03, 0x30, 0x07, 0x38, 0xFE, 0x1F, 0xFC, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0xFC, 0x3F, 0xFE, 0x3F, 0xE7, 0x00, 0xC3, 0x01, 0x83, 0x03, 0x07, 0x07, 0x0E, 0x0E, 0x0C, 0x1C,
    0xFC, 0x3F, 0xFE, 0x3F, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xE1, 0x1F,
    0x73, 0x38, 0x37, 0x30, 0x36, 0x30, 0x34, 0x30, 0x30, 0x30, 0x70, 0x38, 0xE0, 0x1F, 0xC0, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xE0, 0x1F, 0x70, 0x38, 0x30, 0x30, 0x34, 0x30, 0x36, 0x30,
    0x37, 0x30, 0x73, 0x38, 0xE1, 0x1F, 0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xE0, 0x1F,
    0x74, 0x38, 0x36, 0x30, 0x33, 0x30, 0x33, 0x30, 0x36, 0x30, 0x74, 0x38, 0xE0, 0x1F, 0xC0, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0xCC, 0x0F, 0xEE, 0x1F, 0x77, 0x38, 0x33, 0x30, 0x33, 0x30, 0x37, 0x30,
    0x3E, 0x30, 0x7C, 0x38, 0xEC, 0x1F, 0xCE, 0x0F, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x0F, 0xE3, 0x1F, 0x73, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x73, 0x38,
    0xE3, 0x1F, 0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x1C, 0x0E, 0x38, 0x07, 0xF0, 0x03,
    0xE0, 0x01, 0xE0, 0x01, 0xF0, 0x03, 0x38, 0x07, 0x1C, 0x0E, 0x0C, 0x0C, 0x00, 0x30, 0x00, 0x38,
    0xFC, 0x1F, 0xFE, 0x1F, 0x07, 0x3F, 0x83, 0x33, 0xC3, 0x31, 0xE3, 0x30, 0x73, 0x30, 0x3F, 0x38,
    0xFE, 0x1F, 0xFE, 0x0F, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0xF0, 0x1F,
    0x03, 0x38, 0x07, 0x30, 0x0E, 0x30, 0x0C, 0x30, 0x00, 0x30, 0x00, 0x38, 0xF0, 0x1F, 0xF0, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0xF0, 0x1F, 0x00, 0x38, 0x00, 0x30, 0x0C, 0x30, 0x0E, 0x30,
    0x07, 0x30, 0x03, 0x38, 0xF0, 0x1F, 0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0xF0, 0x1F,
    0x04, 0x38, 0x06, 0x30, 0x03, 0x30, 0x03, 0x30, 0x06, 0x30, 0x04, 0x38, 0xF0, 0x1F, 0xF0, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0xF3, 0x0F, 0xF3, 0x1F, 0x00, 0x38, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30,
    0x00, 0x30, 0x00, 0x38, 0xF3, 0x1F, 0xF3, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0xF0, 0x01,
    0x80, 0x03, 0x00, 0x07, 0x0C, 0x3E, 0x0E, 0x3E, 0x07, 0x07, 0x83, 0x03, 0xF0, 0x01, 0xF0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0x3F, 0xFF, 0x3F, 0x18, 0x06, 0x18, 0x06, 0x18, 0x06, 0x18, 0x06,
    0x18, 0x06, 0x38, 0x07, 0xF0, 0x03, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x3F, 0xFE, 0x3F,
    0x07, 0x00, 0x03, 0x00, 0xC3, 0x30, 0xC3, 0x30, 0xE7, 0x30, 0xFE, 0x39, 0xBC, 0x1F, 0x00, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x01, 0x1E, 0x33, 0x37, 0x37, 0x33, 0x36, 0x33, 0x34, 0x33,
    0x30, 0x33, 0x70, 0x33, 0xE0, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x1E,
    0x30, 0x37, 0x30, 0x33, 0x34, 0x33, 0x36, 0x33, 0x37, 0x33, 0x73, 0x33, 0xC1, 0x3F, 0xC0, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x1E, 0x34, 0x37, 0x36, 0x33, 0x33, 0x33, 0x33, 0x33,
    0x36, 0x33, 0x74, 0x33, 0xE0, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x0E, 0x1E,
    0x37, 0x37, 0x33, 0x33, 0x33, 0x33, 0x37, 0x33, 0x3E, 0x33, 0x7C, 0x33, 0xEC, 0x3F, 0xCE, 0x3F,
    0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x1E, 0x33, 0x37, 0x33, 0x33,
    0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x73, 0x33, 0xE3, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0C, 0x00, 0x1E, 0x30, 0x37, 0x32, 0x33, 0x37, 0x33, 0x35, 0x33, 0x37, 0x33, 0x72, 0x33,
    0xE0, 0x3F, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x1E, 0x30, 0x37, 0x30, 0x33,
    0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x70, 0x33, 0xE0, 0x1F, 0xE0, 0x1F, 0x70, 0x3B, 0x30, 0x33,
    0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0xE0, 0x33, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00,
    0xFC, 0x00, 0xFE, 0x01, 0x87, 0xC3, 0x03, 0xC7, 0x03, 0xCF, 0x03, 0xFF, 0x03, 0x7B, 0x87, 0x33,
    0xCE, 0x01, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xE1, 0x1F, 0x73, 0x3B, 0x37, 0x33,
    0x36, 0x33, 0x34, 0x33, 0x30, 0x33, 0x30, 0x33, 0xE0, 0x03, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x0F, 0xE0, 0x1F, 0x70, 0x3B, 0x30, 0x33, 0x34, 0x33, 0x36, 0x33, 0x37, 0x33, 0x33, 0x33,
    0xE1, 0x03, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xE4, 0x1F, 0x76, 0x3B, 0x37, 0x33,
    0x33, 0x33, 0x33, 0x33, 0x37, 0x33, 0x36, 0x33, 0xE4, 0x03, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x0F, 0xE3, 0x1F, 0x73, 0x3B, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x30, 0x33, 0x33, 0x33,
    0xE3, 0x03, 0xC0, 0x03, 0x00, 0x00, 0x01, 0x00, 0x33, 0x30, 0x36, 0x30, 0xF4, 0x3F, 0xF0, 0x3F,
    0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0xF4, 0x3F, 0xF6, 0x3F,
    0x03, 0x30, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x34, 0x30, 0x36, 0x30, 0xF3, 0x3F, 0xF3, 0x3F,
    0x06, 0x30, 0x04, 0x30, 0x00, 0x00, 0x00, 0x00, 0x33, 0x30, 0x33, 0x30, 0xF0, 0x3F, 0xF0, 0x3F,
    0x03, 0x30, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x1F, 0xC0, 0x39, 0xC0, 0x30,
    0xCC, 0x30, 0xCC, 0x30, 0xCC, 0x30, 0xCC, 0x30, 0xFF, 0x3F, 0xFF, 0x3F, 0x0C, 0x00, 0x0C, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFC, 0x3F, 0xFC, 0x3F, 0x33, 0x00, 0x33, 0x00, 0x33, 0x00, 0x37, 0x00,
    0x3E, 0x00, 0x7C, 0x00, 0xEC, 0x3F, 0xCE, 0x3F, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x0F, 0xE1, 0x1F, 0x73, 0x38, 0x37, 0x30, 0x36, 0x30, 0x34, 0x30, 0x30, 0x30, 0x70, 0x38,
    0xE0, 0x1F, 0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xE0, 0x1F, 0x70, 0x38, 0x30, 0x30,
    0x34, 0x30, 0x36, 0x30, 0x37, 0x30, 0x73, 0x38, 0xE1, 0x1F, 0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x0F, 0xE0, 0x1F, 0x74, 0x38, 0x36, 0x30, 0x33, 0x30, 0x33, 0x30, 0x36, 0x30, 0x74, 0x38,
    0xE0, 0x1F, 0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xCC, 0x0F, 0xEE, 0x1F, 0x77, 0x38, 0x33, 0x30,
    0x33, 0x30, 0x37, 0x30, 0x3E, 0x30, 0x7C, 0x38, 0xEC, 0x1F, 0xCC, 0x0F, 0x07, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xE3, 0x1F, 0x73, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x30, 0x30, 0x73, 0x38, 0xE3, 0x1F, 0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0xC0, 0x00,
    0xC0, 0x00, 0xC0, 0x00, 0xCC, 0x0C, 0xCC, 0x0C, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00,
    0x00, 0x00, 0x00, 0x40, 0xC0, 0x6F, 0xE0, 0x7F, 0x70, 0x3C, 0x30, 0x3E, 0x30, 0x37, 0xB0, 0x33,
    0xF0, 0x31, 0xF0, 0x38, 0xF8, 0x1F, 0xDC, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0xF0, 0x1F,
    0x03, 0x38, 0x07, 0x30, 0x0E, 0x30, 0x0C, 0x30, 0x00, 0x30, 0x00, 0x30, 0xF0, 0x3F, 0xF0, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0xF0, 0x1F, 0x00, 0x38, 0x00, 0x30, 0x0C, 0x30, 0x0E, 0x30,
    0x07, 0x30, 0x03, 0x30, 0xF0, 0x3F, 0xF0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x0F, 0xF0, 0x1F,
    0x04, 0x38, 0x06, 0x30, 0x03, 0x30, 0x03, 0x30, 0x06, 0x30, 0x04, 0x30, 0xF0, 0x3F, 0xF0, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0xF3, 0x0F, 0xF3, 0x1F, 0x00, 0x38, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30,
    0x00, 0x30, 0x00, 0x30, 0xF3, 0x3F, 0xF3, 0x3F, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x03, 0xF0, 0x07,
    0x00, 0xCE, 0x00, 0xCC, 0x0C, 0xCC, 0x0E, 0xCC, 0x07, 0xCC, 0x03, 0xEE, 0xF0, 0x7F, 0xF0, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C, 0x30, 0x0C,
    0x30, 0x0C, 0x70, 0x0E, 0xE0, 0x07, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF3, 0x03, 0xF3, 0x07,
    0x00, 0xCE, 0x00, 0xCC, 0x00, 0xCC, 0x00, 0xCC, 0x00, 0xCC, 0x00, 0xEE, 0xF3, 0x7F, 0xF3, 0x3F,

};

#endif
