/*
 * EBB42 v1.2 standalone heater controller
 *
 * Board : BIGTREETECH EBB42 CAN v1.2  (STM32G0B1CBT6)
 * Clock : HSI16, SYSCLK = 16 MHz (no crystal dependency)
 * Heater: PB13  -> heater MOSFET, active high (Klipper: heater_pin PB13)
 * Sensor: PA3   -> ADC1_IN3, 4.7k pull-up to 3.3 V
 *         (Klipper: sensor_pin PA3, sensor_type EPCOS 100K B57560G104F)
 * LED   : PA13  -> blue "Status" LED via 200R to GND, active high.
 *         PA13 is also SWDIO; using it as a GPIO disables SWD (DFU still works).
 *         Slow blink = heating, solid = at temperature, fast blink = fault,
 *         short blip every 2 s = heater disabled (standby).
 * OLED  : optional 128x64 SSD1306 on the I2C header (PB3 SCL, PB4 SDA).
 * Keys  : optional buttons to GND on the Endstop header:
 *         Stop1/PB5 = setpoint up, Stop2/PB6 = setpoint down,
 *         Stop3/PB7 = heater on/off, or reset when faulted.
 * USB   : CDC serial telemetry once a second. Commands: '+'/'-' setpoint,
 *         'e' toggle heater, 'r' reset, 'b' reboot to DFU bootloader.
 *
 * Safety (all latch the heater OFF until reset / power cycle):
 *   - min/max temperature sanity (open / shorted thermistor)
 *   - overshoot cut-off
 *   - "verify heater": heater on but temperature not rising -> fault
 *   - independent watchdog
 */
#include "stm32g0xx.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "usb.h"
#include "oled.h"
#include "ui.h"
#include "settings.h"

/* ------------------------------------------------------------------ */
/* User configuration                                                  */
/* ------------------------------------------------------------------ */
#define DEFAULT_TARGET_C     80.0f
#define SETPOINT_MIN_C       20.0f
#define SETPOINT_MAX_C       100.0f
#define SETPOINT_STEP_C      1.0f
#define CONTROL_MODE_PID     1        /* 1 = PID, 0 = bang-bang            */
#define MAX_POWER            1.0f     /* 0.0 .. 1.0 cap on heater duty     */
#define BANGBANG_HYST_C      1.0f

/* PID gains, Klipper convention (output on a 0..255 scale). These are the
 * Klipper example values for a typical hotend cartridge heater.           */
#define PID_KP               22.2f
#define PID_KI               1.08f
#define PID_KD               114.0f

/* Thermistor: EPCOS 100K B57560G104F, Klipper's three calibration points  */
#define THERM_PULLUP_OHMS    4700.0f
#define THERM_T1  25.0f
#define THERM_R1  100000.0f
#define THERM_T2  150.0f
#define THERM_R2  1641.9f
#define THERM_T3  250.0f
#define THERM_R3  226.15f

/* Safety limits */
#define MIN_TEMP_C           0.0f     /* below  -> open thermistor / fault */
#define MAX_TEMP_C           130.0f   /* above  -> short / runaway / fault */
#define OVERSHOOT_FAULT_C    25.0f    /* target + this -> fault            */
#define VERIFY_HYST_C        5.0f     /* Klipper verify_heater: hysteresis */
#define VERIFY_GAIN_C        1.5f     /* must rise this much ...           */
#define VERIFY_TIME_MS       45000u   /* ... within this time              */
#define VERIFY_MAX_ERROR     120.0f   /* Klipper max_error, degC*s below target-hyst */
#define CONTROL_TIMEOUT_MS   1000u    /* SysTick forces heater off if the control
                                         loop stops updating (Klipper max_duration) */
#define ADC_RANGE_MIN        5.0f     /* raw ADC sanity window, Klipper adc_range */
#define ADC_RANGE_MAX        4090.0f
#define ADC_RANGE_COUNT      4u       /* consecutive out-of-range samples -> fault */

/* Timing */
#define SAMPLE_PERIOD_MS     10u
#define CONTROL_PERIOD_MS    250u
#define UI_PERIOD_MS         250u
#define PWM_WINDOW_MS        100u     /* heater time-proportioning window  */
#define STARTUP_SETTLE_MS    1000u
#define SETTINGS_SAVE_DELAY_MS 3000u
#define ADC_OVERSAMPLE       16u

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static volatile uint32_t g_ms;              /* SysTick millisecond counter */
static volatile uint8_t  g_heater_duty;     /* 0..PWM_WINDOW_MS            */
static volatile bool     g_fault_latched;
static volatile uint32_t g_control_stamp;   /* millis() of last control update */

static float       g_target = DEFAULT_TARGET_C;
static bool        g_enabled = true;
static float       g_temp;                  /* filtered temperature        */
static float       g_adc_raw;
static const char *g_state = "STARTUP";
static const char *g_fault_short = "";      /* short text for the OLED     */
static uint32_t    g_settings_dirty_at;
static bool        g_settings_dirty;

static float adc_to_temp(float adc);
static float adc_read_avg(void);
static void  ui_update(void);
static void  control_reset(void);
static void  verify_reset(void);

/* ------------------------------------------------------------------ */
/* Pins                                                                */
/* ------------------------------------------------------------------ */
#define HEATER_PIN      13u                      /* PB13 */
#define HEATER_ON()     (GPIOB->BSRR = (1u << HEATER_PIN))
#define HEATER_OFF()    (GPIOB->BRR  = (1u << HEATER_PIN))

#define LED_PIN         13u                      /* PA13 */
#define LED_ON()        (GPIOA->BSRR = (1u << LED_PIN))
#define LED_OFF()       (GPIOA->BRR  = (1u << LED_PIN))

#define BTN_UP_PIN      5u                       /* PB5, Endstop1 */
#define BTN_DOWN_PIN    6u                       /* PB6, Endstop2 */
#define BTN_OK_PIN      7u                       /* PB7, Endstop3 */

/* LED pattern: period in ms and on-time in ms, driven from SysTick */
static volatile uint16_t g_led_period = 1000, g_led_on = 500;
static void led_pattern_heating(void) { g_led_period = 1000; g_led_on = 500; }
static void led_pattern_at_temp(void) { g_led_period = 1000; g_led_on = 1000; }
static void led_pattern_fault(void)   { g_led_period = 200;  g_led_on = 100; }
static void led_pattern_standby(void) { g_led_period = 2000; g_led_on = 60; }

void SysTick_Handler(void)
{
    uint32_t now = ++g_ms;
    if ((now % g_led_period) < g_led_on) LED_ON(); else LED_OFF();
    if (g_fault_latched || (now - g_control_stamp) > CONTROL_TIMEOUT_MS) {
        HEATER_OFF();               /* latched fault, or control loop stalled */
        return;
    }
    if ((now % PWM_WINDOW_MS) < g_heater_duty)
        HEATER_ON();
    else
        HEATER_OFF();
}

static inline uint32_t millis(void) { return g_ms; }

static void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) { __NOP(); }
}

static void set_heater_power(float power)   /* 0.0 .. 1.0 */
{
    if (power < 0.0f) power = 0.0f;
    if (power > MAX_POWER) power = MAX_POWER;
    g_heater_duty = (uint8_t)(power * (float)PWM_WINDOW_MS + 0.5f);
}

/* ------------------------------------------------------------------ */
/* Buttons (active low, debounced, auto-repeat)                        */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  pin;
    uint8_t  cnt;
    bool     stable;
    uint32_t next_rep;
} btn_t;

static btn_t btn_up = { .pin = BTN_UP_PIN }, btn_down = { .pin = BTN_DOWN_PIN }, btn_ok = { .pin = BTN_OK_PIN };

enum { BTN_NONE = 0, BTN_CLICK, BTN_REPEAT };

/* call every SAMPLE_PERIOD_MS */
static int btn_update(btn_t *b, uint32_t now)
{
    bool pressed = !((GPIOB->IDR >> b->pin) & 1u);
    if (pressed == b->stable) {
        b->cnt = 0;
    } else if (++b->cnt >= 3) {
        b->cnt = 0;
        b->stable = pressed;
        if (pressed) {
            b->next_rep = now + 600;
            return BTN_CLICK;
        }
    }
    if (b->stable && (int32_t)(now - b->next_rep) >= 0) {
        b->next_rep = now + 120;
        return BTN_REPEAT;
    }
    return BTN_NONE;
}

/* ------------------------------------------------------------------ */
/* Setpoint / enable handling                                           */
/* ------------------------------------------------------------------ */
static void settings_touch(void)
{
    g_settings_dirty = true;
    g_settings_dirty_at = millis();
}

static void set_target(float t)
{
    if (t < SETPOINT_MIN_C) t = SETPOINT_MIN_C;
    if (t > SETPOINT_MAX_C) t = SETPOINT_MAX_C;
    if (t != g_target) {
        g_target = t;
        verify_reset();
        settings_touch();
    }
}

static void set_enabled(bool en)
{
    if (en == g_enabled) return;
    g_enabled = en;
    control_reset();
    verify_reset();
    set_heater_power(0.0f);
    settings_touch();
}

static void settings_flush_if_due(uint32_t now)
{
    if (g_settings_dirty && (now - g_settings_dirty_at) >= SETTINGS_SAVE_DELAY_MS) {
        settings_t s = { .setpoint_c10 = (int16_t)(g_target * 10.0f + 0.5f),
                         .enabled = g_enabled ? 1 : 0 };
        settings_save(&s);
        g_settings_dirty = false;
    }
}

/* ------------------------------------------------------------------ */
/* USB telemetry / commands                                            */
/* ------------------------------------------------------------------ */
static void handle_usb_commands(void)
{
    int c = usb_read_char();
    switch (c) {
    case 'b': case 'B': usb_request_bootloader(); break;
    case 'r': case 'R': NVIC_SystemReset(); break;
    case '+': case '=': set_target(g_target + SETPOINT_STEP_C); break;
    case '-': case '_': set_target(g_target - SETPOINT_STEP_C); break;
    case 'e': case 'E': set_enabled(!g_enabled); break;
    default: break;
    }
}

static void telemetry(void)
{
    char line[112];
    int t100 = (int)(g_temp * 100.0f);
    int s10  = (int)(g_target * 10.0f + 0.5f);
    snprintf(line, sizeof line,
             "t=%lus temp=%d.%02dC raw=%d duty=%u%% target=%d.%d en=%d oled=%d state=%s\r\n",
             (unsigned long)(millis() / 1000u), t100 / 100, (t100 < 0 ? -t100 : t100) % 100,
             (int)(g_adc_raw + 0.5f), (unsigned)g_heater_duty,
             s10 / 10, s10 % 10, g_enabled ? 1 : 0, oled_present() ? 1 : 0, g_state);
    usb_write(line);
}

/* ------------------------------------------------------------------ */
/* Fault latch                                                          */
/* ------------------------------------------------------------------ */
static void fault(const char *reason, const char *short_text)
{
    g_fault_latched = true;
    g_heater_duty = 0;
    HEATER_OFF();
    g_state = reason;
    g_fault_short = short_text;
    led_pattern_fault();
    /* Latch forever, keep the watchdog happy so we don't reboot into a
     * heating state. Keep USB + display alive so the fault can be read.  */
    uint32_t last = millis(), last_ui = millis(), last_btn = millis();
    for (;;) {
        IWDG->KR = 0xAAAAu;
        HEATER_OFF();
        usb_task();
        handle_usb_commands();
        oled_poll(millis());
        uint32_t now = millis();
        if ((now - last_btn) >= SAMPLE_PERIOD_MS) {
            last_btn = now;
            if (btn_update(&btn_ok, now) == BTN_CLICK) NVIC_SystemReset();
            btn_update(&btn_up, now);
            btn_update(&btn_down, now);
        }
        if ((now - last) >= 1000u) {
            last = now;
            g_adc_raw = adc_read_avg();
            g_temp = adc_to_temp(g_adc_raw);
            telemetry();
        }
        if ((now - last_ui) >= UI_PERIOD_MS) {
            last_ui = now;
            ui_update();
        }
    }
}

/* ------------------------------------------------------------------ */
/* Clocks / GPIO / ADC / watchdog                                      */
/* ------------------------------------------------------------------ */
void SystemInit(void)
{
    usb_check_bootloader_request();

    /* The ROM DFU bootloader may leave the clock tree reconfigured.
     * Force a clean HSI16 @ 16 MHz SYSCLK.                              */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CR &= ~RCC_CR_HSIDIV;                     /* HSISYS = HSI16 / 1 */
    RCC->CFGR = 0;                                 /* SW=HSISYS, no div  */
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0) {}
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);
    SCB->VTOR = FLASH_BASE;
}

static void gpio_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    (void)RCC->IOPENR;

    /* Heater PB13: drive low first, then make it a push-pull output   */
    HEATER_OFF();
    GPIOB->OTYPER  &= ~(1u << HEATER_PIN);
    GPIOB->OSPEEDR &= ~(3u << (HEATER_PIN * 2));
    GPIOB->PUPDR   &= ~(3u << (HEATER_PIN * 2));
    GPIOB->MODER    = (GPIOB->MODER & ~(3u << (HEATER_PIN * 2)))
                    | (1u << (HEATER_PIN * 2));

    /* Thermistor PA3: analog                                           */
    GPIOA->PUPDR &= ~(3u << (3 * 2));
    GPIOA->MODER |=  (3u << (3 * 2));

    /* Status LED PA13: push-pull output (takes the pin away from SWD) */
    LED_OFF();
    GPIOA->PUPDR   &= ~(3u << (LED_PIN * 2));
    GPIOA->OTYPER  &= ~(1u << LED_PIN);
    GPIOA->MODER    = (GPIOA->MODER & ~(3u << (LED_PIN * 2)))
                    | (1u << (LED_PIN * 2));

    /* Buttons PB5/PB6/PB7: inputs with pull-up (board also has 10k)   */
    uint32_t mask2 = (3u << (BTN_UP_PIN * 2)) | (3u << (BTN_DOWN_PIN * 2)) | (3u << (BTN_OK_PIN * 2));
    uint32_t pu    = (1u << (BTN_UP_PIN * 2)) | (1u << (BTN_DOWN_PIN * 2)) | (1u << (BTN_OK_PIN * 2));
    GPIOB->MODER &= ~mask2;
    GPIOB->PUPDR  = (GPIOB->PUPDR & ~mask2) | pu;
}

static void adc_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_ADCEN;
    (void)RCC->APBENR2;

    ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;              /* async clock = SYSCLK */
    ADC1_COMMON->CCR &= ~ADC_CCR_PRESC;

    ADC1->CR = ADC_CR_ADVREGEN;                    /* regulator on, 20 us */
    delay_ms(2);

    ADC1->CR |= ADC_CR_ADCAL;                      /* calibrate           */
    while (ADC1->CR & ADC_CR_ADCAL) {}

    ADC1->CFGR1 = 0;                               /* 12-bit, single, SW  */
    ADC1->SMPR  = ADC_SMPR_SMP1;                   /* 160.5 ADC cycles    */
    ADC1->ISR   = ADC_ISR_CCRDY;
    ADC1->CHSELR = ADC_CHSELR_CHSEL3;              /* PA3 = IN3           */
    while (!(ADC1->ISR & ADC_ISR_CCRDY)) {}

    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
}

static uint32_t adc_read_raw(void)
{
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC)) {}
    return ADC1->DR;
}

static float adc_read_avg(void)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < ADC_OVERSAMPLE; i++)
        sum += adc_read_raw();
    return (float)sum / (float)ADC_OVERSAMPLE;
}

static void iwdg_init(void)
{
    /* LSI ~32 kHz / 32 = 1 kHz tick, reload 2000 -> ~2 s timeout       */
    IWDG->KR  = 0xCCCCu;
    IWDG->KR  = 0x5555u;
    IWDG->PR  = 3u;
    IWDG->RLR = 2000u;
    while (IWDG->SR) {}
    IWDG->KR  = 0xAAAAu;
}

/* ------------------------------------------------------------------ */
/* Thermistor (Steinhart-Hart from three points, as Klipper does)      */
/* ------------------------------------------------------------------ */
static float sh_c1, sh_c2, sh_c3;

static void thermistor_init(void)
{
    float inv_t1 = 1.0f / (THERM_T1 + 273.15f);
    float inv_t2 = 1.0f / (THERM_T2 + 273.15f);
    float inv_t3 = 1.0f / (THERM_T3 + 273.15f);
    float ln_r1 = logf(THERM_R1), ln_r2 = logf(THERM_R2), ln_r3 = logf(THERM_R3);
    float ln3_r1 = ln_r1 * ln_r1 * ln_r1;
    float ln3_r2 = ln_r2 * ln_r2 * ln_r2;
    float ln3_r3 = ln_r3 * ln_r3 * ln_r3;

    float inv_t12 = inv_t1 - inv_t2, inv_t13 = inv_t1 - inv_t3;
    float ln_r12 = ln_r1 - ln_r2,    ln_r13 = ln_r1 - ln_r3;
    float ln3_r12 = ln3_r1 - ln3_r2, ln3_r13 = ln3_r1 - ln3_r3;

    sh_c3 = (inv_t12 - inv_t13 * ln_r12 / ln_r13)
          / (ln3_r12 - ln3_r13 * ln_r12 / ln_r13);
    sh_c2 = (inv_t12 - sh_c3 * ln3_r12) / ln_r12;
    sh_c1 = inv_t1 - sh_c2 * ln_r1 - sh_c3 * ln3_r1;
}

static float adc_to_temp(float adc)
{
    if (adc >= 4090.0f) return -300.0f;           /* open thermistor  */
    if (adc <= 5.0f)    return  999.0f;           /* shorted          */
    float r    = THERM_PULLUP_OHMS * adc / (4095.0f - adc);
    float ln_r = logf(r);
    float inv_t = sh_c1 + sh_c2 * ln_r + sh_c3 * ln_r * ln_r * ln_r;
    return 1.0f / inv_t - 273.15f;
}

/* ------------------------------------------------------------------ */
/* Control                                                              */
/* ------------------------------------------------------------------ */
#if CONTROL_MODE_PID
#define PID_PARAM_BASE 255.0f
static float pid_prev_temp, pid_prev_integ;
static bool  pid_first = true;

static void control_reset(void) { pid_first = true; pid_prev_integ = 0.0f; }

static float control_update(float temp, float dt)
{
    if (pid_first) { pid_prev_temp = temp; pid_first = false; }

    float integ_max = MAX_POWER * PID_PARAM_BASE / PID_KI;
    float err   = g_target - temp;
    float deriv = (temp - pid_prev_temp) / dt;
    float integ = pid_prev_integ + err * dt;
    if (integ < 0.0f)      integ = 0.0f;
    if (integ > integ_max) integ = integ_max;

    float co = PID_KP * err + PID_KI * integ - PID_KD * deriv;
    float bounded = co;
    if (bounded < 0.0f) bounded = 0.0f;
    if (bounded > MAX_POWER * PID_PARAM_BASE) bounded = MAX_POWER * PID_PARAM_BASE;
    if (co == bounded)                 /* anti-windup: only integrate when unsaturated */
        pid_prev_integ = integ;
    pid_prev_temp = temp;
    return bounded / PID_PARAM_BASE;
}
#else
static bool bb_heating;
static void control_reset(void) { bb_heating = false; }

static float control_update(float temp, float dt)
{
    (void)dt;
    if (bb_heating && temp >= g_target + BANGBANG_HYST_C)
        bb_heating = false;
    else if (!bb_heating && temp <= g_target - BANGBANG_HYST_C)
        bb_heating = true;
    return bb_heating ? MAX_POWER : 0.0f;
}
#endif

/* Klipper verify_heater (klippy/extras/verify_heater.py), same algorithm:
 *  - while approaching the target the temperature must rise VERIFY_GAIN_C
 *    every VERIFY_TIME_MS, otherwise "not heating at expected rate";
 *  - once at temperature, time spent more than VERIFY_HYST_C below target
 *    accumulates as degC*s; above VERIFY_MAX_ERROR -> "not maintaining".
 * Catches an unplugged heater, dead MOSFET, missing 24 V, and a thermistor
 * that has fallen off the heated part.                                   */
static bool     vh_approaching, vh_starting;
static float    vh_goal_temp, vh_error, vh_last_target = -1000.0f;
static uint32_t vh_goal_time;

static void verify_reset(void)
{
    vh_approaching = vh_starting = false;
    vh_error = 0.0f;
    vh_last_target = -1000.0f;              /* forces a fresh approach check */
}

static void verify_heater(float temp, uint32_t now, float dt)
{
    float target = g_target;
    if (temp >= target - VERIFY_HYST_C) {
        vh_approaching = vh_starting = false;
        if (temp <= target + VERIFY_HYST_C) vh_error = 0.0f;
        vh_last_target = target;
        return;
    }
    vh_error += ((target - VERIFY_HYST_C) - temp) * dt;
    if (!vh_approaching) {
        if (target != vh_last_target) {
            vh_approaching = vh_starting = true;
            vh_goal_temp = temp + VERIFY_GAIN_C;
            vh_goal_time = now + VERIFY_TIME_MS;
        } else if (vh_error >= VERIFY_MAX_ERROR) {
            fault("FAULT_HEATER_NOT_MAINTAINING_TEMP", "TEMP DROPPED");
        }
    } else if (temp >= vh_goal_temp) {
        vh_starting = false;
        vh_error = 0.0f;
        vh_goal_temp = temp + VERIFY_GAIN_C;
        vh_goal_time = now + VERIFY_TIME_MS;
    } else if ((int32_t)(now - vh_goal_time) >= 0) {
        fault("FAULT_HEATER_NOT_HEATING", "NOT HEATING");
    } else if (vh_starting) {
        if (temp + VERIFY_GAIN_C < vh_goal_temp) vh_goal_temp = temp + VERIFY_GAIN_C;
    }
    vh_last_target = target;
}

/* Klipper adc_range: raw sample must stay inside the window, a few
 * consecutive misses is a sensor fault regardless of the filtered value.  */
static uint32_t adc_range_misses;

static void check_adc_range(float raw)
{
    if (raw <= ADC_RANGE_MIN || raw >= ADC_RANGE_MAX) {
        if (++adc_range_misses >= ADC_RANGE_COUNT)
            fault(raw >= ADC_RANGE_MAX ? "FAULT_ADC_OPEN" : "FAULT_ADC_SHORT",
                  raw >= ADC_RANGE_MAX ? "SENSOR OPEN" : "SENSOR SHORT");
    } else {
        adc_range_misses = 0;
    }
}

static void check_limits(float temp)
{
    if (!isfinite(temp))                        fault("FAULT_SENSOR_NAN", "SENSOR ERROR");
    if (temp < MIN_TEMP_C)                      fault("FAULT_SENSOR_OPEN_OR_TOO_COLD", "SENSOR OPEN");
    if (temp > MAX_TEMP_C)                      fault("FAULT_SENSOR_SHORT_OR_TOO_HOT", "SENSOR SHORT");
    if (temp > g_target + OVERSHOOT_FAULT_C)    fault("FAULT_OVERSHOOT", "OVERSHOOT");
}

/* ------------------------------------------------------------------ */
/* OLED user interface                                                  */
/* ------------------------------------------------------------------ */
static void ui_update(void)
{
    if (!oled_present()) return;
    ui_state_t st = {
        .temp        = g_temp,
        .target      = g_target,
        .power_pct   = ((unsigned)g_heater_duty * 100u) / PWM_WINDOW_MS,
        .enabled     = g_enabled,
        .at_temp     = g_temp >= g_target - VERIFY_HYST_C,
        .fault       = g_fault_latched,
        .fault_short = g_fault_short,
    };
    ui_render(&st);
}

/* ------------------------------------------------------------------ */
int main(void)
{
    gpio_init();
    SysTick_Config(16000000u / 1000u);
    __enable_irq();

    thermistor_init();
    adc_init();
    usb_init();
    oled_init();
    iwdg_init();

    settings_t saved;
    if (settings_load(&saved)) {
        g_target  = (float)saved.setpoint_c10 / 10.0f;
        if (g_target < SETPOINT_MIN_C) g_target = SETPOINT_MIN_C;
        if (g_target > SETPOINT_MAX_C) g_target = SETPOINT_MAX_C;
        g_enabled = saved.enabled != 0;
    }

    /* Let the ADC filter settle before trusting readings              */
    float temp_filt = adc_to_temp(adc_read_avg());
    uint32_t t0 = millis();
    while ((millis() - t0) < STARTUP_SETTLE_MS) {
        g_adc_raw = adc_read_avg();
        temp_filt += 0.25f * (adc_to_temp(g_adc_raw) - temp_filt);
        g_temp = temp_filt;
        IWDG->KR = 0xAAAAu;
        usb_task();
        oled_poll(millis());
        delay_ms(SAMPLE_PERIOD_MS);
    }
    check_limits(temp_filt);

    uint32_t last_sample  = millis();
    uint32_t last_control = millis();
    uint32_t last_report  = millis();
    uint32_t last_ui      = millis();
    ui_update();

    for (;;) {
        uint32_t now = millis();

        if ((now - last_sample) >= SAMPLE_PERIOD_MS) {
            last_sample += SAMPLE_PERIOD_MS;
            g_adc_raw = adc_read_avg();
            check_adc_range(g_adc_raw);
            temp_filt += 0.25f * (adc_to_temp(g_adc_raw) - temp_filt);
            g_temp = temp_filt;
            check_limits(temp_filt);

            int e;
            if ((e = btn_update(&btn_up, now)) != BTN_NONE)   set_target(g_target + SETPOINT_STEP_C);
            if ((e = btn_update(&btn_down, now)) != BTN_NONE) set_target(g_target - SETPOINT_STEP_C);
            if (btn_update(&btn_ok, now) == BTN_CLICK)        set_enabled(!g_enabled);
        }

        if ((now - last_control) >= CONTROL_PERIOD_MS) {
            float dt = (float)(now - last_control) / 1000.0f;
            last_control = now;
            g_control_stamp = now;              /* keeps SysTick's heater gate open */
            if (g_enabled) {
                set_heater_power(control_update(temp_filt, dt));
                verify_heater(temp_filt, now, dt);
                if (temp_filt >= g_target - VERIFY_HYST_C) {
                    g_state = "AT_TEMP";
                    led_pattern_at_temp();
                } else {
                    g_state = "HEATING";
                    led_pattern_heating();
                }
            } else {
                set_heater_power(0.0f);
                g_state = "STANDBY";
                led_pattern_standby();
            }
        }

        if ((now - last_ui) >= UI_PERIOD_MS) {
            last_ui = now;
            ui_update();
        }

        if ((now - last_report) >= 1000u) {
            last_report = now;
            telemetry();
        }

        settings_flush_if_due(now);
        oled_poll(now);
        usb_task();
        handle_usb_commands();
        IWDG->KR = 0xAAAAu;
        __WFI();            /* wake on next SysTick or USB interrupt */
    }
}
