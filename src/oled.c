#include "oled.h"
#include "stm32g0xx.h"
#include "font6x8.h"
#include <string.h>

#ifndef OLED_SH1106
#define OLED_SH1106 0          /* 1 for 1.3" SH1106 modules (2-pixel column offset) */
#endif
#define OLED_ADDR   0x3C
#define OLED_W      128
#define OLED_H      64
#define OLED_PAGES  (OLED_H / 8)

#define SCL_PIN 3u              /* PB3 */
#define SDA_PIN 4u              /* PB4 */

static uint8_t  fb[OLED_PAGES][OLED_W];
static uint8_t  dirty;          /* bit per page */
static bool     present;
static uint32_t next_probe;

/* ---------------- bit-banged I2C, open drain, ~200-300 kHz at 16 MHz ---------------- */
static inline void i2c_delay(void) { for (volatile int i = 0; i < 4; i++) {} }
static inline void scl_hi(void) { GPIOB->BSRR = 1u << SCL_PIN; }
static inline void scl_lo(void) { GPIOB->BRR  = 1u << SCL_PIN; }
static inline void sda_hi(void) { GPIOB->BSRR = 1u << SDA_PIN; }
static inline void sda_lo(void) { GPIOB->BRR  = 1u << SDA_PIN; }
static inline bool sda_rd(void) { return (GPIOB->IDR >> SDA_PIN) & 1u; }

static bool scl_wait_hi(void)   /* honour clock stretching with a timeout */
{
    for (int i = 0; i < 2000; i++)
        if ((GPIOB->IDR >> SCL_PIN) & 1u) return true;
    return false;
}

static void i2c_gpio_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
    scl_hi(); sda_hi();
    GPIOB->OTYPER  |= (1u << SCL_PIN) | (1u << SDA_PIN);            /* open drain */
    GPIOB->OSPEEDR |= (1u << (SCL_PIN * 2)) | (1u << (SDA_PIN * 2)); /* medium    */
    GPIOB->PUPDR    = (GPIOB->PUPDR & ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2))))
                    | (1u << (SCL_PIN * 2)) | (1u << (SDA_PIN * 2)); /* weak pull-ups as backup */
    GPIOB->MODER    = (GPIOB->MODER & ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2))))
                    | (1u << (SCL_PIN * 2)) | (1u << (SDA_PIN * 2)); /* output    */
}

static bool i2c_start(void)
{
    sda_hi(); scl_hi(); i2c_delay();
    if (!scl_wait_hi() || !sda_rd()) return false;     /* bus stuck / missing */
    sda_lo(); i2c_delay();
    scl_lo(); i2c_delay();
    return true;
}

static void i2c_stop(void)
{
    sda_lo(); i2c_delay();
    scl_hi(); i2c_delay(); scl_wait_hi();
    sda_hi(); i2c_delay();
}

static bool i2c_write_byte(uint8_t b)        /* returns true on ACK */
{
    for (int i = 0; i < 8; i++) {
        if (b & 0x80) sda_hi(); else sda_lo();
        b <<= 1;
        i2c_delay();
        scl_hi(); i2c_delay();
        if (!scl_wait_hi()) return false;
        scl_lo();
    }
    sda_hi(); i2c_delay();
    scl_hi(); i2c_delay();
    if (!scl_wait_hi()) return false;
    bool ack = !sda_rd();
    scl_lo(); i2c_delay();
    return ack;
}

static bool i2c_write(uint8_t control, const uint8_t *data, int len)
{
    if (!i2c_start()) return false;
    bool ok = i2c_write_byte(OLED_ADDR << 1) && i2c_write_byte(control);
    for (int i = 0; ok && i < len; i++) ok = i2c_write_byte(data[i]);
    i2c_stop();
    return ok;
}

static bool cmd(uint8_t c) { return i2c_write(0x00, &c, 1); }

/* ---------------- SSD1306 ---------------- */
static bool oled_hw_init(void)
{
    static const uint8_t seq[] = {
        0xAE,           /* display off */
        0xD5, 0x80,     /* clock */
        0xA8, 0x3F,     /* multiplex 64 */
        0xD3, 0x00,     /* display offset */
        0x40,           /* start line 0 */
        0x8D, 0x14,     /* charge pump on */
        0x20, 0x02,     /* page addressing mode */
        0xA1,           /* segment remap */
        0xC8,           /* COM scan dec */
        0xDA, 0x12,     /* COM pins */
        0x81, 0x7F,     /* contrast */
        0xD9, 0xF1,     /* precharge */
        0xDB, 0x40,     /* VCOM detect */
        0xA4,           /* resume RAM content */
        0xA6,           /* normal (not inverted) */
        0x2E,           /* no scroll */
        0xAF,           /* display on */
    };
    for (unsigned i = 0; i < sizeof seq; i++)
        if (!cmd(seq[i])) return false;
    return true;
}

static bool push_page(int p)
{
    int col = OLED_SH1106 ? 2 : 0;
    if (!cmd(0xB0 | p)) return false;
    if (!cmd(0x00 | (col & 0x0F))) return false;
    if (!cmd(0x10 | (col >> 4))) return false;
    return i2c_write(0x40, fb[p], OLED_W);
}

void oled_init(void)
{
    i2c_gpio_init();
    memset(fb, 0, sizeof fb);
    present = oled_hw_init();
    if (present) dirty = 0xFF;
    next_probe = 0;
}

bool oled_present(void) { return present; }

void oled_poll(uint32_t now_ms)
{
    if (!present) {
        if ((int32_t)(now_ms - next_probe) >= 0) {
            next_probe = now_ms + 2000;
            present = oled_hw_init();
            if (present) dirty = 0xFF;
        }
        return;
    }
    if (!dirty) return;
    for (int p = 0; p < OLED_PAGES; p++) {
        if (dirty & (1u << p)) {
            if (!push_page(p)) {          /* lost the display: fall back to probing */
                present = false;
                next_probe = now_ms + 2000;
                return;
            }
            dirty &= ~(1u << p);
            return;                       /* one page per call keeps the main loop responsive */
        }
    }
}

/* ---------------- drawing ---------------- */
void oled_clear(void) { memset(fb, 0, sizeof fb); dirty = 0xFF; }
void oled_flush(void) { dirty = 0xFF; }

static inline void set_px(int x, int y)
{
    if ((unsigned)x < OLED_W && (unsigned)y < OLED_H)
        fb[y >> 3][x] |= (uint8_t)(1u << (y & 7));
}

void oled_text(int x, int y8, int scale, const char *s)
{
    if (scale < 1) scale = 1;
    for (; *s; s++) {
        unsigned c = (unsigned char)*s;
        if (c < 0x20 || c > 0x7E) c = '?';
        const uint8_t *g = font6x8[c - 0x20];
        if (scale == 1) {
            if (y8 >= 0 && y8 < OLED_PAGES)
                for (int cx = 0; cx < 6; cx++)
                    if ((unsigned)(x + cx) < OLED_W) fb[y8][x + cx] |= g[cx];
        } else {
            for (int cx = 0; cx < 6; cx++)
                for (int cy = 0; cy < 8; cy++)
                    if (g[cx] & (1u << cy))
                        for (int dx = 0; dx < scale; dx++)
                            for (int dy = 0; dy < scale; dy++)
                                set_px(x + cx * scale + dx, y8 * 8 + cy * scale + dy);
        }
        x += 6 * scale;
    }
    dirty = 0xFF;
}

void oled_hline(int x0, int x1, int y)
{
    for (int x = x0; x <= x1; x++) set_px(x, y);
    dirty = 0xFF;
}

void oled_rect(int x, int y, int w, int h, bool fill)
{
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++)
            if (fill || yy == y || yy == y + h - 1 || xx == x || xx == x + w - 1)
                set_px(xx, yy);
    dirty = 0xFF;
}

const uint8_t *oled_framebuffer(void) { return &fb[0][0]; }
