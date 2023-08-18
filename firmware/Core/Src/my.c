/*
 * USB keyboard controller for ZX Spectrum
 * Copyright (c) 2023 Aleksey Morozov aleksey.f.morozov@gmail.com aleksey.f.morozov@yandex.ru
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include "my.h"

extern UART_HandleTypeDef huart1;
extern USBH_HandleTypeDef hUsbHostFS;

#define ARRAY_SIZE(A) (sizeof(A) / sizeof(A[0]))
#define BITS_PER_BYTE 8
#define USB_SHIFTS_COUNT 8
#define BSRR_RESET 16

/* ZX Spectrum keyboard
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐ ┌───────┐
 * │ 1 ! │ 2 @ │ 3 # │ 4 $ │ 5 % │ 6 & │ 7 ` │ 8 ( │ 9 ) │ 0 _ │ │ RESET │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤ ├───────┤
 * │ Q<= │ W<> │ E=> │ R < │ T > │  Y  │  U  │  I  │ O ; │ P " │ │ MAGIC │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤ └───────┘
 * │  A  │  S  │  D  │  F  │  G  │ H ^ │ J - │ K + │ L = │ ENT │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
 * │ CAP │ Z : │  X  │ C ? │ V / │ B * │ N , │ M . │ SYM │ SPC │
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 */

#define ZX_A_SHIFT 3
#define ZX_A_MASK 7
#define ZX_D_MASK 7
#define ZX(A, D) ((((A) & ZX_A_MASK) << ZX_A_SHIFT) | ((D) & ZX_D_MASK))
#define ZXM_CAP (1 << 7)
#define ZXM_SYM (1 << 6)
#define ZX_GET_ADDRESS(KEY) (((KEY) >> ZX_A_SHIFT) & ZX_A_MASK)
#define ZX_GET_DATA(KEY) ((KEY) & ZX_D_MASK)

#define ZX_1 ZX(3, 0)
#define ZX_2 ZX(3, 1)
#define ZX_3 ZX(3, 2)
#define ZX_4 ZX(3, 3)
#define ZX_5 ZX(3, 4)
#define ZX_6 ZX(4, 4)
#define ZX_7 ZX(4, 3)
#define ZX_8 ZX(4, 2)
#define ZX_9 ZX(4, 1)
#define ZX_0 ZX(4, 0)

#define ZX_Q ZX(2, 0)
#define ZX_W ZX(2, 1)
#define ZX_E ZX(2, 2)
#define ZX_R ZX(2, 3)
#define ZX_T ZX(2, 4)
#define ZX_Y ZX(5, 4)
#define ZX_U ZX(5, 3)
#define ZX_I ZX(5, 2)
#define ZX_O ZX(5, 1)
#define ZX_P ZX(5, 0)

#define ZX_A ZX(1, 0)
#define ZX_S ZX(1, 1)
#define ZX_D ZX(1, 2)
#define ZX_F ZX(1, 3)
#define ZX_G ZX(1, 4)
#define ZX_H ZX(6, 4)
#define ZX_J ZX(6, 3)
#define ZX_K ZX(6, 2)
#define ZX_L ZX(6, 1)
#define ZX_ENTER ZX(6, 0)

#define ZX_CAPS ZX(0, 0) /* Caps shift */
#define ZX_Z ZX(0, 1)
#define ZX_X ZX(0, 2)
#define ZX_C ZX(0, 3)
#define ZX_V ZX(0, 4)
#define ZX_B ZX(7, 4)
#define ZX_N ZX(7, 3)
#define ZX_M ZX(7, 2)
#define ZX_SYM ZX(7, 1) /* Symbol shift */
#define ZX_SPACE ZX(7, 0)

#define ZX_EDIT (ZX_1 | ZXM_CAP)
#define ZX_CAPSL (ZX_2 | ZXM_CAP) /* Caps lock */
#define ZX_TRUVI (ZX_3 | ZXM_CAP) /* True video */
#define ZX_INVVI (ZX_4 | ZXM_CAP) /* Inverse video */
#define ZX_LEFT (ZX_5 | ZXM_CAP)
#define ZX_DOWN (ZX_6 | ZXM_CAP)
#define ZX_UP (ZX_7 | ZXM_CAP)
#define ZX_RIGHT (ZX_8 | ZXM_CAP)
#define ZX_GRAPH (ZX_9 | ZXM_CAP)
#define ZX_DEL (ZX_0 | ZXM_CAP) /* Delete */
#define ZX_BREAK (ZX_SPACE | ZXM_CAP)
#define ZX_EXTMO (ZX_SYM | ZXM_CAP) /* Ext mode */

#define ZX_GRAVE (ZX_7 | ZXM_SYM) /* ` Back quote */
#define ZX_OPEN (ZX_8 | ZXM_SYM)  /* ( Open or left parenthesis */
#define ZX_CLOSE (ZX_9 | ZXM_SYM) /* ) Close or right parenthesis */

#define ZX_LT (ZX_R | ZXM_SYM)    /* < Less then */
#define ZX_GT (ZX_T | ZXM_SYM)    /* > Greater then */
#define ZX_SEMIC (ZX_O | ZXM_SYM) /* ; Semicolon */
#define ZX_QUOTE (ZX_P | ZXM_SYM) /* " Quote */

#define ZX_MINUS (ZX_J | ZXM_SYM) /* - Minus */
#define ZX_PLUS (ZX_K | ZXM_SYM)  /* + Plus */
#define ZX_EQUAL (ZX_L | ZXM_SYM) /* = Equal */

#define ZX_COLON (ZX_Z | ZXM_SYM) /* : Colon */
#define ZX_SLASH (ZX_V | ZXM_SYM) /* / Divide */
#define ZX_MUL (ZX_B | ZXM_SYM)   /* * Multiply */
#define ZX_COMMA (ZX_N | ZXM_SYM) /* , Comma */
#define ZX_DOT (ZX_M | ZXM_SYM)   /* . Dot */

#define ZX_RESET ZX(0, 5)
#define ZX_MAGIC ZX(0, 6)
#define ZX_CURJO ZX(1, 5)
#define ZX_SINJO ZX(1, 6)

#define NONE 0xFF
#define STD_KEYS_OFFSET (USB_SHIFTS_COUNT - KEY_A)

static const uint8_t usb_to_zx[] = {
    ZX_SYM,   ZX_CAPS,  ZX_EXTMO, ZX_0,     /* LCTRL, LSHIFT, LALT, LGUI */
    ZX_CAPS,  ZX_SYM,   ZX_EXTMO, ZX_0,     /* RCTRL, RSHIFT, RALT, RGUI */
    ZX_A,     ZX_B,     ZX_C,     ZX_D,     /* 04  A B C D */
    ZX_E,     ZX_F,     ZX_G,     ZX_H,     /* 08  E F G H */
    ZX_I,     ZX_J,     ZX_K,     ZX_L,     /* 0C  I J K L */
    ZX_M,     ZX_N,     ZX_O,     ZX_P,     /* 10  M N O P */
    ZX_Q,     ZX_R,     ZX_S,     ZX_T,     /* 14  Q R S T */
    ZX_U,     ZX_V,     ZX_W,     ZX_X,     /* 18  U V W X */
    ZX_Y,     ZX_Z,     ZX_1,     ZX_2,     /* 1B  Y Z 1 2 */
    ZX_3,     ZX_4,     ZX_5,     ZX_6,     /* 20  3 4 5 6 */
    ZX_7,     ZX_8,     ZX_9,     ZX_0,     /* 24  7 8 9 0 */
    ZX_ENTER, ZX_BREAK, ZX_DEL,   ZX_EDIT,  /* 28  Enter ESC BKSPC TAB */
    ZX_SPACE, ZX_MINUS, ZX_EQUAL, ZX_OPEN,  /* 28  Space - = [ */
    ZX_CLOSE, ZX_COLON, NONE,     ZX_SEMIC, /* 30  ] \ ?  ; */
    ZX_QUOTE, ZX_GRAVE, ZX_COMMA, ZX_DOT,   /* 34  " ` , . */
    ZX_SLASH, ZX_CAPSL, ZX_TRUVI, ZX_INVVI, /* 38  / CAPS F1 F2 */
    ZX_GRAPH, NONE,     ZX_CURJO, ZX_SINJO, /* 3C  F3 F4 F5 F6 */
    NONE,     NONE,     NONE,     ZX_MAGIC, /* 40  F7 F8 F9 F10 */
    NONE,     ZX_RESET, ZX_PLUS,  NONE,     /* 44  F11 F12 PRSCR SCROLL */
    ZX_PLUS,  NONE,     NONE,     NONE,     /* 48  PAUSE INSERT HOME PGUP */
    ZX_DEL,   NONE,     NONE,     ZX_RIGHT, /* 4C  DEL END PGDN RIGHT */
    ZX_LEFT,  ZX_DOWN,  ZX_UP,    NONE,     /* 50  LEFT DOWN UP NUM */
    ZX_SLASH, ZX_MUL,   ZX_MINUS, ZX_PLUS,  /* 54  g/ g* g- g+ */
    ZX_ENTER, ZX_1,     ZX_2,     ZX_3,     /* 58  gENTER g1 g2 g3 */
    ZX_4,     ZX_5,     ZX_6,     ZX_7,     /* 58  g4 g5 g6 g7 */
    ZX_8,     ZX_9,     ZX_0,     ZX_DOT,   /* 60  g8 g9 g0 g. */
    0, ZX_0                                 /* 64  ? APP */
};

static uint8_t zx_prepared_a[0x100] = {[0 ... 0xFF] = 0xFF};
static uint8_t zx_prepared_b[0x100] = {[0 ... 0xFF] = 0xFF};
static volatile uint8_t* zx_prepared = zx_prepared_a;
static uint8_t sinclair_joystic = false;

void DebugOutput(const char *format, ...) {
    assert(format != NULL);
    char buf[128];
    va_list args;
    va_start(args, format);
    const int result = vsnprintf(buf, sizeof(buf), format, args);
    assert(result >= 0);
    va_end(args);
    if (result > 0) {
        const size_t length = result < sizeof(buf) - 1 ? (size_t)result : sizeof(buf) - 1;
        HAL_StatusTypeDef result2 = HAL_UART_Transmit(&huart1, (const uint8_t *)buf, length, HAL_MAX_DELAY);
        assert(result2 == HAL_OK);
    }
}

void MyInit() {
    DebugOutput("\r\nZX USB Keyboard, version 15-Аug-2023, (c) 2023 Aleksey Morozov aleksey.f.morozov@gmail.com aleksey.f.morozov@yandex.ru\r\n");
}

static inline uint8_t ZxMatrixGet(uint8_t *zx_matix, uint8_t zx_key) {
    return zx_matix[ZX_GET_ADDRESS(zx_key)] & (1 << ZX_GET_DATA(zx_key));
}

static inline void ZxMatrixSet(uint8_t *zx_matrix, uint8_t zx_key) {
    zx_matrix[ZX_GET_ADDRESS(zx_key)] |= 1 << ZX_GET_DATA(zx_key);
}

static void ZxMatrixSetUsb(uint8_t *zx_matrix, uint8_t usb_key) {
    if (sinclair_joystic) {
        static const uint8_t usb_to_zx_joystick[] = { ZX_7, ZX_6, ZX_8, ZX_9 };
        unsigned i = usb_key - (STD_KEYS_OFFSET + KEY_RIGHTARROW);
        if (i < ARRAY_SIZE(usb_to_zx_joystick)) {
            ZxMatrixSet(zx_matrix, usb_to_zx_joystick[i]);
            return;
        }
    }
    if (usb_key < ARRAY_SIZE(usb_to_zx)) {
        const uint8_t zx_key = usb_to_zx[usb_key];
        if (zx_key != NONE) {
            ZxMatrixSet(zx_matrix, zx_key);
            if (zx_key & ZXM_CAP)
                ZxMatrixSet(zx_matrix, ZX_CAPS);
            if (zx_key & ZXM_SYM)
                ZxMatrixSet(zx_matrix, ZX_SYM);
            return;
        }
    }
    DebugOutput("Unknown key %02X\r\n", usb_key);
}

void MyIdle() {
    /* Keyboard connected? */
    if (hUsbHostFS.pActiveClass != USBH_HID_CLASS)
        return;

    /* Get key from USB keyboard */
    HID_KEYBD_Info_TypeDef *info = USBH_HID_GetKeybdInfo(&hUsbHostFS);
    if (info == NULL)
        return;

    /* ZX keyboard matrix calculation */
    uint8_t zx_matrix[8] = {0};
    unsigned i;
    const uint8_t *info_shifts = info->keys - USB_SHIFTS_COUNT;
    for (i = 0; i < USB_SHIFTS_COUNT; i++)
        if (info_shifts[i])
            ZxMatrixSetUsb(zx_matrix, i);
    for (i = 0; i < ARRAY_SIZE(info->keys); i++)
        if (info->keys[i] >= KEY_A)
            ZxMatrixSetUsb(zx_matrix, STD_KEYS_OFFSET + info->keys[i]);

    /* Modes */
    if (ZxMatrixGet(zx_matrix, ZX_CURJO))
        sinclair_joystic = false;
    else if (ZxMatrixGet(zx_matrix, ZX_SINJO))
        sinclair_joystic = true;

    /* Precompute data for the interrupt handler */
    uint8_t *a = zx_prepared != zx_prepared_a ? zx_prepared_a : zx_prepared_b;
    uint8_t *o = a;
    for (i = 0; i < ARRAY_SIZE(zx_prepared_a) - 1; i++) {
        unsigned j, p = 0, z = i;
        for (j = 0; j < BITS_PER_BYTE; j++) {
            if ((z & 1) == 0)
                p |= zx_matrix[j];
            z >>= 1;
        }
        *o++ = ~p;
    }
    zx_prepared = a;

    /* Onboard led */
    GPIOC->BSRR = a[0] != 0xFF ? (GPIO_PIN_13 << BSRR_RESET) : GPIO_PIN_13;

    /* Reset key */
    GPIOB->BSRR = ZxMatrixGet(zx_matrix, ZX_RESET) ? (GPIO_PIN_8 << BSRR_RESET) : GPIO_PIN_8;

    /* Magic key */
    if (ZxMatrixGet(zx_matrix, ZX_MAGIC)) {
        /* Disable IRQ */
        __disable_irq();
        /* Wait for M1 raise */
        while ((GPIOA->IDR & GPIO_PIN_6) == 0);
        while ((GPIOA->IDR & GPIO_PIN_6) != 0);
        /* Press MAGIC */
        GPIOB->BSRR = GPIO_PIN_9 << BSRR_RESET;
        /* Delay */
        volatile unsigned delay = 0;
        while (delay < 2)
            delay++;
        /* Release MAGIC */
        GPIOB->BSRR = GPIO_PIN_9;
        /* Enable IRQ */
        __enable_irq();
    }
}

void EXTI9_5_IRQHandler() {
    GPIOA->ODR = zx_prepared[GPIOB->IDR & 0xFF];
    __HAL_GPIO_EXTI_CLEAR_IT(0xFFFF);
}
