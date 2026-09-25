#ifndef UI_H_
#define UI_H_
#include <stdbool.h>
typedef struct {
    float       temp;
    float       target;
    unsigned    power_pct;
    bool        enabled;
    bool        at_temp;
    bool        fault;
    const char *fault_short;
} ui_state_t;
void ui_render(const ui_state_t *s);   /* draws into the OLED framebuffer */
#endif
