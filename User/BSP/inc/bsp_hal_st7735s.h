#ifndef __BSP_HAL_ST7735S_H__
#define __BSP_HAL_ST7735S_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
/* Private macros ------------------------------------------------------------*/
#define ST7735S_INVERT_COLORS 0
#define LCD_ST7735S           1  //1£∫ST7735S   0£∫ST7735
/* ∆¡ƒª∑ΩœÚ…Ë÷√ */
#define LCD_CLEAR_SEND_NUMBER    512
#define ST7735S_XSTART   0
#define ST7735S_YSTART   0
#define LCD_PORTRAIT             0       /* ∫·∆¡ */
#define LCD_PORTRAIT_INVERTED    1       /* ∫·∆¡µπ÷√ */
#define LCD_LANDSCAPE            2       /*  ˙∆¡ */
#define LCD_LANDSCAPE_INVERTED   3       /*  ˙∆¡µπ÷√ */
#define CONFIG_LCD_DISPLAY_ORIENTATION   LCD_PORTRAIT
#if LCD_ST7735S == 1
#define LCD_W 132
#define LCD_H 132
#else
#define LCD_W 162
#define LCD_H 132
#endif
#if   ( CONFIG_LCD_DISPLAY_ORIENTATION == LCD_PORTRAIT )
   #define ST7735S_WIDTH    LCD_W
   #define ST7735S_HEIGHT   LCD_H
#elif (CONFIG_LCD_DISPLAY_ORIENTATION == LCD_PORTRAIT_INVERTED)
   #define ST7735S_WIDTH    LCD_W
   #define ST7735S_HEIGHT   LCD_H
#elif (CONFIG_LCD_DISPLAY_ORIENTATION == LCD_LANDSCAPE)
   #define ST7735S_WIDTH    LCD_H
   #define ST7735S_HEIGHT   LCD_W
#elif (CONFIG_LCD_DISPLAY_ORIENTATION == LCD_LANDSCAPE_INVERTED)
   #define ST7735S_WIDTH    LCD_H
   #define ST7735S_HEIGHT   LCD_W
#else
   #define ST7735S_WIDTH    LCD_W
   #define ST7735S_HEIGHT   LCD_H
#endif
//POINT_COLOR
#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE             0x001F
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40
#define BRRED            0XFC07
#define GRAY             0X8430
#define GRAY175          0XAD75
#define GRAY151          0X94B2
#define GRAY187          0XBDD7
#define GRAY240          0XF79E
#define GRAY0            0xEF7D //ª“…´0 
#define GRAY1            0x8410 //ª“…´1      
#define GRAY2            0x4208 //ª“…´2 
// Delay between some initialisation commands
#define TFT_INIT_DELAY      0x80

#define TFT_NOP             0x00
#define TFT_SWRST           0x01

#define TFT_CASET           0x2A
#define TFT_PASET           0x2B
#define TFT_RAMWR           0x2C

#define TFT_RAMRD           0x2E
#define TFT_IDXRD           0x00

#define TFT_MADCTL          0x36
#define TFT_MAD_MY          0x80
#define TFT_MAD_MX          0x40
#define TFT_MAD_MV          0x20
#define TFT_MAD_ML          0x10
#define TFT_MAD_BGR         0x08
#define TFT_MAD_MH          0x04
#define TFT_MAD_RGB         0x00

#define TFT_INVOFF          0x20
#define TFT_INVON           0x21

// ST7735 specific commands used in init
#define ST7735_NOP          0x00
#define ST7735_SWRESET      0x01
#define ST7735_RDDID        0x04
#define ST7735_RDDST        0x09

#define ST7735_SLPIN        0x10
#define ST7735_SLPOUT       0x11
#define ST7735_PTLON        0x12
#define ST7735_NORON        0x13

#define ST7735_INVOFF       0x20
#define ST7735_INVON        0x21
#define ST7735_DISPOFF      0x28
#define ST7735_DISPON       0x29
#define ST7735_CASET        0x2A
#define ST7735_RASET        0x2B
#define ST7735_RAMWR        0x2C
#define ST7735_RAMRD        0x2E

#define ST7735_PTLAR        0x30
#define ST7735_VSCRDEF      0x33
#define ST7735_COLMOD       0x3A
#define ST7735_MADCTL       0x36
#define ST7735_VSCRSADD     0x37

#define ST7735_FRMCTR1      0xB1
#define ST7735_FRMCTR2      0xB2
#define ST7735_FRMCTR3      0xB3
#define ST7735_INVCTR       0xB4
#define ST7735_DISSET5      0xB6

#define ST7735_PWCTR1       0xC0
#define ST7735_PWCTR2       0xC1
#define ST7735_PWCTR3       0xC2
#define ST7735_PWCTR4       0xC3
#define ST7735_PWCTR5       0xC4
#define ST7735_VMCTR1       0xC5

#define ST7735_RDID1        0xDA
#define ST7735_RDID2        0xDB
#define ST7735_RDID3        0xDC
#define ST7735_RDID4        0xDD

#define ST7735_PWCTR6       0xFC

#define ST7735_GMCTRP1      0xE0
#define ST7735_GMCTRN1      0xE1

#define ST77XX_MADCTL_MY    0x80
#define ST77XX_MADCTL_MX    0x40
#define ST77XX_MADCTL_MV    0x20
#define ST77XX_MADCTL_ML    0x10
#define ST77XX_MADCTL_RGB   0x00
#define ST77XX_MADCTL_BGR   0x08
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern uint16_t BACK_COLOR;
extern uint16_t FORE_COLOR;
/* Exported functions --------------------------------------------------------*/
int st7735s_init(void);
void st7735s_flush(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t *color);
void lcd_display_on(void);
void lcd_display_off(void);
void lcd_set_color(uint16_t back, uint16_t fore);
void lcd_clear(uint16_t color);
void lcd_draw_point(uint16_t x, uint16_t y);
void lcd_fill(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color);
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r);
void lcd_show_char(uint16_t x, uint16_t y, uint8_t data, uint32_t size);
int32_t lcd_show_string(uint16_t x, uint16_t y, uint32_t size, char *str);
void lcd_show_num(uint16_t x, uint16_t y,uint32_t size,uint32_t num,uint8_t len,uint8_t mode);
void lcd_show_snum(uint16_t x, uint16_t y,uint32_t size,int32_t num,uint8_t len,uint8_t mode);
void lcd_Display_Picture(uint16_t x, uint16_t y,uint16_t PicWidth,uint16_t PicHeight,const uint8_t *qImage);
void lcd_Display_monochrome(uint16_t x, uint16_t y,uint16_t PicWidth,uint16_t PicHeight,const uint8_t *qImage);
void LCD_ShowString_BufMode(uint16_t x, uint16_t y, uint8_t *str,uint8_t size,uint8_t mode);











#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  // __BSP_HAL_GPIO_H__

