#include "lvgl/lvgl.h"

/*******************************************************************************
 * Size: 8 px
 * Bpp: 1
 * Opts: 
 ******************************************************************************/

#ifndef DISP_THUMB_8_7F
#define DISP_THUMB_8_7F 1
#endif

#if DISP_THUMB_8_7F

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t gylph_bitmap[] = {
    /* U+20 " " */

    /* U+21 "!" */
    0xe8,

    /* U+22 "\"" */
    0xb4,

    /* U+23 "#" */
    0xbe, 0xfa,

    /* U+24 "$" */
    0x52, 0xb4,

    /* U+25 "%" */
    0xa5, 0x4a,

    /* U+26 "&" */
    0x5d, 0x56,

    /* U+27 "'" */
    0xc0,

    /* U+28 "(" */
    0x6a, 0x40,

    /* U+29 ")" */
    0x95, 0x80,

    /* U+2A "*" */
    0xaa, 0x80,

    /* U+2B "+" */
    0x5d, 0x0,

    /* U+2C "," */
    0x60,

    /* U+2D "-" */
    0xe0,

    /* U+2E "." */
    0x80,

    /* U+2F "/" */
    0x25, 0x48,

    /* U+30 "0" */
    0xf6, 0xde,

    /* U+31 "1" */
    0x75, 0x40,

    /* U+32 "2" */
    0xe7, 0xce,

    /* U+33 "3" */
    0xe7, 0x9e,

    /* U+34 "4" */
    0xb7, 0x92,

    /* U+35 "5" */
    0xf3, 0x9e,

    /* U+36 "6" */
    0xf3, 0xde,

    /* U+37 "7" */
    0xe4, 0x92,

    /* U+38 "8" */
    0xf7, 0xde,

    /* U+39 "9" */
    0xf7, 0x9e,

    /* U+3A ":" */
    0xa0,

    /* U+3B ";" */
    0x46,

    /* U+3C "<" */
    0x2a, 0x22,

    /* U+3D "=" */
    0xe3, 0x80,

    /* U+3E ">" */
    0x88, 0xa8,

    /* U+3F "?" */
    0xe5, 0x4,

    /* U+40 "@" */
    0xf7, 0xce,

    /* U+41 "A" */
    0x57, 0xda,

    /* U+42 "B" */
    0xd7, 0x5c,

    /* U+43 "C" */
    0x72, 0x46,

    /* U+44 "D" */
    0x96, 0xdc,

    /* U+45 "E" */
    0xf3, 0xce,

    /* U+46 "F" */
    0xf3, 0xc8,

    /* U+47 "G" */
    0x73, 0xd6,

    /* U+48 "H" */
    0xb7, 0xda,

    /* U+49 "I" */
    0xe9, 0x2e,

    /* U+4A "J" */
    0x24, 0xd4,

    /* U+4B "K" */
    0xb7, 0x5a,

    /* U+4C "L" */
    0x92, 0x4e,

    /* U+4D "M" */
    0xbf, 0xda,

    /* U+4E "N" */
    0xbf, 0xfa,

    /* U+4F "O" */
    0x56, 0xd4,

    /* U+50 "P" */
    0xd7, 0x48,

    /* U+51 "Q" */
    0x56, 0xf6,

    /* U+52 "R" */
    0x97, 0xea,

    /* U+53 "S" */
    0x71, 0x1c,

    /* U+54 "T" */
    0xe9, 0x24,

    /* U+55 "U" */
    0xb6, 0xd6,

    /* U+56 "V" */
    0xb6, 0xa4,

    /* U+57 "W" */
    0xb7, 0xfa,

    /* U+58 "X" */
    0xb5, 0x5a,

    /* U+59 "Y" */
    0xb5, 0x24,

    /* U+5A "Z" */
    0xe5, 0x6e,

    /* U+5B "[" */
    0xea, 0xc0,

    /* U+5C "\\" */
    0x91, 0x12,

    /* U+5D "]" */
    0xd5, 0xc0,

    /* U+5E "^" */
    0x44,

    /* U+5F "_" */
    0xe0,

    /* U+60 "`" */
    0x90,

    /* U+61 "a" */
    0xce, 0xf0,

    /* U+62 "b" */
    0x92, 0xdc,

    /* U+63 "c" */
    0x72, 0x30,

    /* U+64 "d" */
    0x2e, 0xd6,

    /* U+65 "e" */
    0x77, 0x30,

    /* U+66 "f" */
    0x2f, 0xa4,

    /* U+67 "g" */
    0x7f, 0x94,

    /* U+68 "h" */
    0x92, 0xda,

    /* U+69 "i" */
    0xb8,

    /* U+6A "j" */
    0x20, 0x9a, 0x80,

    /* U+6B "k" */
    0x97, 0x6a,

    /* U+6C "l" */
    0xc9, 0x2e,

    /* U+6D "m" */
    0xff, 0xd0,

    /* U+6E "n" */
    0x96, 0xd0,

    /* U+6F "o" */
    0x56, 0xa0,

    /* U+70 "p" */
    0x96, 0xe8,

    /* U+71 "q" */
    0x56, 0xb2,

    /* U+72 "r" */
    0x72, 0x40,

    /* U+73 "s" */
    0x7d, 0xe0,

    /* U+74 "t" */
    0x5d, 0x26,

    /* U+75 "u" */
    0xb6, 0xb0,

    /* U+76 "v" */
    0xb7, 0xa0,

    /* U+77 "w" */
    0xbf, 0xf0,

    /* U+78 "x" */
    0xa9, 0x50,

    /* U+79 "y" */
    0xb5, 0xb4,

    /* U+7A "z" */
    0xef, 0x70,

    /* U+7B "{" */
    0x6b, 0x26,

    /* U+7C "|" */
    0xfc,

    /* U+7D "}" */
    0xc9, 0xac,

    /* U+7E "~" */
    0x78
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_h = 0, .box_w = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 64, .box_h = 0, .box_w = 0, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 0, .adv_w = 64, .box_h = 5, .box_w = 1, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 64, .box_h = 2, .box_w = 3, .ofs_x = 0, .ofs_y = 3},
    {.bitmap_index = 2, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 4, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 6, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 8, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 10, .adv_w = 64, .box_h = 2, .box_w = 1, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 11, .adv_w = 64, .box_h = 5, .box_w = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 13, .adv_w = 64, .box_h = 5, .box_w = 2, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 15, .adv_w = 64, .box_h = 3, .box_w = 3, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 17, .adv_w = 64, .box_h = 3, .box_w = 3, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 19, .adv_w = 64, .box_h = 2, .box_w = 2, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 20, .adv_w = 64, .box_h = 1, .box_w = 3, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 21, .adv_w = 64, .box_h = 1, .box_w = 1, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 22, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 24, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 26, .adv_w = 64, .box_h = 5, .box_w = 2, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 28, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 30, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 32, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 34, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 36, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 38, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 40, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 42, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 44, .adv_w = 64, .box_h = 3, .box_w = 1, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 45, .adv_w = 64, .box_h = 4, .box_w = 2, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 46, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 48, .adv_w = 64, .box_h = 3, .box_w = 3, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 50, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 52, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 54, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 56, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 58, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 60, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 62, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 64, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 66, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 68, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 70, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 72, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 74, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 76, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 78, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 80, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 82, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 84, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 86, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 88, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 90, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 92, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 94, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 96, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 98, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 100, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 102, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 104, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 106, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 108, .adv_w = 64, .box_h = 5, .box_w = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 110, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 112, .adv_w = 64, .box_h = 5, .box_w = 2, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 114, .adv_w = 64, .box_h = 2, .box_w = 3, .ofs_x = 0, .ofs_y = 3},
    {.bitmap_index = 115, .adv_w = 64, .box_h = 1, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 116, .adv_w = 64, .box_h = 2, .box_w = 2, .ofs_x = 0, .ofs_y = 3},
    {.bitmap_index = 117, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 119, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 121, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 123, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 125, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 127, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 129, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 131, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 133, .adv_w = 64, .box_h = 5, .box_w = 1, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 134, .adv_w = 64, .box_h = 6, .box_w = 3, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 137, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 139, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 141, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 143, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 145, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 147, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 149, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 151, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 153, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 155, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 157, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 159, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 161, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 163, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 165, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = -1},
    {.bitmap_index = 167, .adv_w = 64, .box_h = 4, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 169, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 171, .adv_w = 64, .box_h = 6, .box_w = 1, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 172, .adv_w = 64, .box_h = 5, .box_w = 3, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 174, .adv_w = 64, .box_h = 2, .box_w = 3, .ofs_x = 0, .ofs_y = 3}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY,
        .glyph_id_start = 1, .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

/*Store all the custom data of the font*/
static lv_font_fmt_txt_dsc_t font_dsc = {
    .glyph_bitmap = gylph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .cmap_num = 1,
    .bpp = 1,

    .kern_scale = 0,
    .kern_dsc = NULL,
    .kern_classes = 0,
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
lv_font_t DISP_Thumb_8_7F = {
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .line_height = 6,          /*The maximum line height required by the font*/
    .base_line = 1,             /*Baseline measured from the bottom of the line*/
};

#endif /*#if DISP_THUMB_8_7F*/

