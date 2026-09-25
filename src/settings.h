#ifndef SETTINGS_H_
#define SETTINGS_H_
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t  setpoint_c10;      /* setpoint in 0.1 C */
    uint8_t  enabled;           /* heater enabled at boot */
    uint8_t  pad;
} settings_t;

bool settings_load(settings_t *out);          /* false if none stored */
bool settings_save(const settings_t *in);     /* erases + programs last flash page */
#endif
