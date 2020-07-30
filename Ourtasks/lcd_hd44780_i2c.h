/**
 * Copyright Nikita Bulaev 2017
 *
 * STM32 HAL libriary for LCD display based on HITACHI HD44780U chip.
 *
 * ===========================================================================
 * WARNING!
 *
 * YOU MUST INCLUDE CORRECT STM32 HAL LIB HEAR. THIS LIB WAS TESTED ON STM32F3
 * PLEASE, INCLUDE CORRECT ONE!
 * ===========================================================================
 *
 * THIS SOFTWARE IS PROVIDED "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LCD_HD44780_I2C_H
#define LCD_HD44780_I2C_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "LcdTask.h"

#define LCD_BIT_RS                 ((uint8_t)0x01U)
#define LCD_BIT_RW                 ((uint8_t)0x02U)
#define LCD_BIT_E                  ((uint8_t)0x04U)
#define LCD_BIT_BACKIGHT_ON        ((uint8_t)0x08U)
#define LCD_BIT_BACKIGHT_OFF       ((uint8_t)0x00U)

#define LCD_MODE_4BITS             ((uint8_t)0x02U)
#define LCD_BIT_1LINE              ((uint8_t)0x00U)
#define LCD_BIT_2LINE              ((uint8_t)0x08U)
#define LCD_BIT_4LINE              LCD_BIT_2LINE
#define LCD_BIT_5x8DOTS            ((uint8_t)0x00U)
#define LCD_BIT_5x10DOTS           ((uint8_t)0x04U)
#define LCD_BIT_SETCGRAMADDR       ((uint8_t)0x40U)
#define LCD_BIT_SETDDRAMADDR       ((uint8_t)0x80U)

#define LCD_BIT_DISPLAY_CONTROL    ((uint8_t)0x08U)
#define LCD_BIT_DISPLAY_ON         ((uint8_t)0x04U)
#define LCD_BIT_CURSOR_ON          ((uint8_t)0x02U)
#define LCD_BIT_CURSOR_OFF         ((uint8_t)0x00U)
#define LCD_BIT_BLINK_ON           ((uint8_t)0x01U)
#define LCD_BIT_BLINK_OFF          ((uint8_t)0x00U)

#define LCD_BIT_DISP_CLEAR         ((uint8_t)0x01U)
#define LCD_BIT_CURSOR_HOME        ((uint8_t)0x02U)

#define LCD_BIT_ENTRY_MODE         ((uint8_t)0x04U)
#define LCD_BIT_CURSOR_DIR_RIGHT   ((uint8_t)0x02U)
#define LCD_BIT_CURSOR_DIR_LEFT    ((uint8_t)0x00U)
#define LCD_BIT_DISPLAY_SHIFT      ((uint8_t)0x01U)

// TODO: Update commands with this defines
#define LCD_BIT_CURSOR_SHIFT_MODE  ((uint8_t)0x10U)
#define LCD_BIT_CURSOR_DISP_SHIFT  ((uint8_t)0x08U)
#define LCD_BIT_CURSOR_MOVE        ((uint8_t)0x00U)
#define LCD_BIT_CURSOR_SHIFT_DIR_R ((uint8_t)0x40U)
#define LCD_BIT_CURSOR_SHIFT_DIR_L ((uint8_t)0x00U)


/* Function defines */
#define lcdBacklightOn(p1)           lcdBacklight(p1,LCD_BIT_BACKIGHT_ON)
#define lcdBacklightOff(p1)          lcdBacklight(p1,LCD_BIT_BACKIGHT_OFF)
#define lcdAutoscrollOn(p1)          lcdCommand(p1,LCD_DISPLAY_SHIFT, LCD_PARAM_SET)
#define lcdAutoscrollOff(p1)         lcdCommand(p1,LCD_DISPLAY_SHIFT, LCD_PARAM_UNSET)
#define lcdDisplayClear(p1)          lcdCommand(p1,LCD_CLEAR, LCD_PARAM_SET)
#define lcdDisplayOn(p1)             lcdCommand(p1,LCD_DISPLAY, LCD_PARAM_SET)
#define lcdDisplayOff(p1)            lcdCommand(p1,LCD_DISPLAY, LCD_PARAM_UNSET)
#define lcdCursorOn(p1)              lcdCommand(p1,LCD_CURSOR, LCD_PARAM_SET)
#define lcdCursorOff(p1)             lcdCommand(p1,LCD_CURSOR, LCD_PARAM_UNSET)
#define lcdBlinkOn(p1)               lcdCommand(p1,LCD_CURSOR_BLINK, LCD_PARAM_SET)
#define lcdBlinkOff(p1)              lcdCommand(p1,LCD_CURSOR_BLINK, LCD_PARAM_UNSET)
#define lcdCursorDirToRight(p1)      lcdCommand(p1,LCD_CURSOR_DIR_RIGHT, LCD_PARAM_SET)
#define lcdCursorDirToLeft(p1)       lcdCommand(p1,LCD_CURSOR_DIR_LEFT, LCD_PARAM_SET)
#define lcdCursorHome(p1)            lcdCommand(p1,LCD_CURSOR_HOME, LCD_PARAM_SET)

#ifndef bool
typedef enum {
    false,
    true
} bool;
#endif

typedef enum {
    LCD_PARAM_UNSET = 0,
    LCD_PARAM_SET
} LCDParamsActions;

typedef enum {
    LCD_BACKLIGHT = 0,
    LCD_DISPLAY,
    LCD_CLEAR,
    LCD_CURSOR,
    LCD_CURSOR_BLINK,
    LCD_CURSOR_HOME,
    LCD_CURSOR_DIR_LEFT,
    LCD_CURSOR_DIR_RIGHT,
    LCD_DISPLAY_SHIFT
} LCDCommands;


struct LCDPARAMS* lcdInit(struct LCDI2C_UNIT* punit);
bool lcdCommand          (struct LCDPARAMS* p1,LCDCommands command, LCDParamsActions action);
bool lcdBacklight        (struct LCDPARAMS* p1,uint8_t command);
bool lcdSetCursorPosition(struct LCDPARAMS* p1,uint8_t line, uint8_t row);
bool lcdPrintStr         (struct LCDPARAMS* p1,uint8_t * data, uint8_t length);
bool lcdPrintChar        (struct LCDPARAMS* p1,uint8_t data);
bool lcdLoadCustomChar   (struct LCDPARAMS* p1,uint8_t cell, uint8_t * charMap);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
