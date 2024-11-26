#ifndef __BSP_LCD_FONT_H__
#define __BSP_LCD_FONT_H__ 	   
#include <stdint.h>



#define ASC2_1206
#define ASC2_1608
//#define ASC2_2412
//#define ASC2_3216
//#define ASC2_4824
//#define ASC2_6432
#if !defined(ASC2_1206) && !defined(ASC2_1608) && !defined(ASC2_2412)  && !defined(ASC2_3216) && !defined(ASC2_4824) && !defined(ASC2_6432)
#error "There is no any define ASC2_1608 && ASC2_2412 && ASC2_2416 && ASC2_3216 !"
#endif
#ifdef ASC2_1206
extern const unsigned char asc2_1206[];
#endif
#ifdef ASC2_1608
extern const uint8_t asc2_1608[];
#endif
#ifdef ASC2_2412
extern const uint8_t asc2_2412[];
#endif
#ifdef ASC2_3216
extern const uint8_t asc2_3216[];
#endif
#ifdef ASC2_4824
extern const uint8_t asc2_4824[];
#endif
#ifdef ASC2_6432
extern const uint8_t asc2_6432[];
#endif


#endif
