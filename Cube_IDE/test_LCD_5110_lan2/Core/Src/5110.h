/*
 * 5110.h
 *
 *  Created on: Jul 25, 2020
 *      Author: Dat UTC
 */
#include "main.h"
#ifndef SRC_5110_H_
#define SRC_5110_H_



#endif /* SRC_5110_H_ */

#define BL_pin                                                                           0
#define RST_pin                                                                          5
#define CE_pin                                                                           6
#define DC_pin                                                                           7
#define SDO_pin                                                                          8
#define SCK_pin                                                                          9

#define PCD8544_set_Y_addr                                                               0x40
#define PCD8544_set_X_addr                                                               0x80

#define PCD8544_set_temp                                                                 0x04
#define PCD8544_set_bias                                                                 0x10
#define PCD8544_set_VOP                                                                  0x80

#define PCD8544_power_down                                                               0x04
#define PCD8544_entry_mode                                                               0x02
#define PCD8544_extended_instruction                                                     0x01

#define PCD8544_display_blank                                                            0x00
#define PCD8544_display_normal                                                           0x04
#define PCD8544_display_all_on                                                           0x01
#define PCD8544_display_inverted                                                         0x05

#define PCD8544_function_set                                                             0x20
#define PCD8544_display_control                                                          0x08

#define CMD                                                                              0
#define DAT                                                                              1

#define X_max                                                                            84
#define Y_max                                                                            48
#define Rows                                                                             6

#define BLACK                                                                            0
#define WHITE                                                                            1
#define PIXEL_XOR                                                                        2

#define ON                                                                               1
#define OFF                                                                              0

#define NO                                                                               0
#define YES                                                                              1

#define buffer_size                                                                      504


unsigned char PCD8544_buffer[X_max][Rows];


void setup_LCD_GPIOs();
void PCD8544_write(unsigned char type, unsigned char value);
void PCD8544_reset();
void PCD8544_init();
void PCD8544_backlight_state(unsigned char value);
void PCD8544_set_contrast(unsigned char value);
void PCD8544_set_cursor(unsigned char x_pos, unsigned char y_pos);
void PCD8544_print_char(unsigned char ch, unsigned char colour);
void PCD8544_print_custom_char(unsigned char *map);
void PCD8544_fill(unsigned char bufr);
void PCD8544_clear_buffer(unsigned char colour);
void PCD8544_clear_screen(unsigned char colour);
void PCD8544_print_image(const unsigned char *bmp);
void PCD8544_print_string(unsigned char x_pos, unsigned char y_pos, unsigned char *ch, unsigned char colour);
void print_char(unsigned char x_pos, unsigned char y_pos, unsigned char ch, unsigned char colour);
void print_string(unsigned char x_pos, unsigned char y_pos, unsigned char *ch, unsigned char colour);
void print_chr(unsigned char x_pos, unsigned char y_pos, signed int value, unsigned char colour);
void print_int(unsigned char x_pos, unsigned char y_pos, signed long value, unsigned char colour);
void print_decimal(unsigned char x_pos, unsigned char y_pos, unsigned int value, unsigned char points, unsigned char colour);
void print_float(unsigned char x_pos, unsigned char y_pos, float value, unsigned char points, unsigned char colour);
void Draw_Pixel(unsigned char x_pos, unsigned char y_pos, unsigned char colour);
void Draw_Line(signed int x1, signed int y1, signed int x2, signed int y2, unsigned char colour);
void Draw_Rectangle(signed int x1, signed int y1, signed int x2, signed int y2, unsigned char fill, unsigned char colour);
void Draw_Circle(signed int xc, signed int yc, signed int radius, unsigned char fill, unsigned char colour);
static const unsigned char font[][5] =
{
     {0x00, 0x00, 0x00, 0x00, 0x00} // 20
    ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
    ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
    ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
    ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
    ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
    ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
    ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
    ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
    ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
    ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
    ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
    ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
    ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
    ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
    ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
    ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
    ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
    ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
    ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
    ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
    ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
    ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
    ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
    ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
    ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
    ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
    ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
    ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
    ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
    ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
    ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
    ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
    ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
    ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
    ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
    ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
    ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
    ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
    ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
    ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
    ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
    ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
    ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
    ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
    ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
    ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
    ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
    ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
    ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
    ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
    ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
    ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
    ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
    ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
    ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
    ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
    ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
    ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
    ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
    ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ?
    ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
    ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
    ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
    ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
    ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
    ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
    ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
    ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
    ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
    ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
    ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
    ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
    ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
    ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
    ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
    ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
    ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
    ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
    ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
    ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
    ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
    ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
    ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
    ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
    ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
    ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
    ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
    ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
    ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
    ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
    ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
    ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
    ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
    ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
    ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};

//unsigned char const mikro_bmp[504] = {
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0, 255, 255,   0, 255, 255, 255,   7,  71,  99,  99,  99,
// 99,  99,  99,  99, 103,   7, 143, 255, 255,  63,  63,  31,
// 15,  15,   7,   7,   7,   3,   3,   3,   3,   3,   3,   3,
//  3,   3,   7,   7,   7,  15,  15,  31,  31,  63, 255, 255,
//255, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0, 255, 255,   0, 255, 255, 255,  14,  12,  12,  28,  60,
// 60,  60,  60,  60, 252, 254, 255,   3,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0, 248, 254, 254, 254, 252,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,
// 15, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0, 255, 255,   0, 255, 255, 255,  34, 102,   6,   6,  30,
// 14,   6,   6,  34, 255, 255, 255,   0,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   1,   3,   3,   3,   3,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0, 255, 255,   0, 255, 255, 255, 196, 254, 222, 198, 198,
//198, 198, 198, 198, 255, 255, 255,   0,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0, 254, 255, 255, 255, 255,
//255,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
//  7, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0, 255, 255,   0, 255, 255, 255,  16,  24,  24,  24,  24,
// 24,  24,  24,  24,  27,  63, 255, 224, 128,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,  15,  63, 127,  63,  63,
// 15,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 128,
//240, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
//  0, 255, 255,   0, 255, 255, 255, 224, 227, 227, 227, 227,
//227, 227, 227, 227, 227, 227, 227, 227, 231, 238, 252, 248,
//248, 240, 240, 224, 224, 224, 224, 224, 224, 224, 224, 224,
//224, 224, 224, 224, 224, 240, 240, 248, 252, 254, 255, 255,
//255, 255, 255, 255, 127, 128, 255, 255,   0,   0,   0,   0,
//  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0
//};
