#ifndef OLED_H_
#define OLED_H_
#include <stdbool.h>
#include <stdint.h>

/* 128x64 SSD1306 (or SH1106 with OLED_SH1106=1) on I2C, bit-banged on
 * PB3 (SCL) / PB4 (SDA) = the EBB42 v1.2 "I2C" header.  Optional: if no
 * display answers, everything is a no-op and we re-probe periodically.   */
void oled_init(void);
bool oled_present(void);
void oled_poll(uint32_t now_ms);     /* call often: pushes one dirty page, re-probes */

void oled_clear(void);
/* Draw text at pixel x, page row y8 (0..7), scale 1..4 (glyph = 6*scale x 8*scale) */
void oled_text(int x, int y8, int scale, const char *s);
void oled_hline(int x0, int x1, int y);
void oled_rect(int x, int y, int w, int h, bool fill);
void oled_flush(void);               /* mark whole frame dirty */
const uint8_t *oled_framebuffer(void); /* 8 pages x 128 columns */
#endif
