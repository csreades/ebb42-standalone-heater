/* 128x64 screen layout for the heater controller */
#include "ui.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

void ui_render(const ui_state_t *s)
{
    char buf[24];
    oled_clear();

    /* Row 0: setpoint left, status right, rule underneath */
    int s10 = (int)(s->target * 10.0f + 0.5f);
    snprintf(buf, sizeof buf, "SET %d.%dC", s10 / 10, s10 % 10);
    oled_text(0, 0, 1, buf);
    const char *st = s->fault ? "FAULT" : !s->enabled ? "OFF" : s->at_temp ? "READY" : "HEAT";
    oled_text(128 - 6 * (int)strlen(st), 0, 1, st);
    oled_hline(0, 127, 9);

    if (s->fault) {
        oled_text(19, 2, 3, "FAULT");                       /* 5 chars * 18 px */
        oled_text(64 - 3 * (int)strlen(s->fault_short), 6, 1, s->fault_short);
        oled_text(64 - 3 * 12, 7, 1, "OK TO RESET");
        return;
    }

    /* Big temperature, scale 4 (24x32 px per char), rows 2..5 */
    int t10 = (int)(s->temp * 10.0f + (s->temp >= 0 ? 0.5f : -0.5f));
    snprintf(buf, sizeof buf, "%d.%d", t10 / 10, (t10 < 0 ? -t10 : t10) % 10);
    int w = 24 * (int)strlen(buf);
    int x = (128 - w - 14) / 2;
    if (x < 0) x = 0;
    oled_text(x, 2, 4, buf);
    if (x + w + 14 <= 128) oled_text(x + w + 2, 2, 2, "C");   /* unit, if room */
    else                   oled_text(x + w + 1, 2, 1, "C");   /* 3-digit temps  */

    /* Power bar, rows 6..7 */
    unsigned pct = s->power_pct > 100 ? 100 : s->power_pct;
    oled_rect(0, 53, 96, 8, false);
    if (pct) oled_rect(0, 53, (int)(96u * pct / 100u), 8, true);
    if (s->enabled) snprintf(buf, sizeof buf, "%3u%%", pct);
    else            snprintf(buf, sizeof buf, "OFF");
    oled_text(128 - 6 * (int)strlen(buf), 7, 1, buf);
}
