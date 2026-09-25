/* USB CDC telemetry / command channel for the EBB42 heater controller */
#include "stm32g0xx.h"
#include "tusb.h"
#include "usb.h"
#include <stdio.h>
#include <string.h>

/* Set by usb_request_bootloader(), consumed very early in SystemInit() */
__attribute__((section(".noinit"))) uint32_t g_boot_magic;
#define BOOT_MAGIC 0xB007C0DEu

void USB_UCPD1_2_IRQHandler(void)
{
    tud_int_handler(0);
}

void usb_init(void)
{
    /* HSI48 on, trimmed to the USB SOF by the CRS */
    RCC->CR |= RCC_CR_HSI48ON;
    while (!(RCC->CR & RCC_CR_HSI48RDY)) {}

    RCC->APBENR1 |= RCC_APBENR1_CRSEN | RCC_APBENR1_PWREN | RCC_APBENR1_USBEN;
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
    (void)RCC->APBENR1;

    /* CRS_CFGR reset value already selects USB SOF sync, reload 47999 */
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    /* USB kernel clock = HSI48 */
    RCC->CCIPR2 &= ~RCC_CCIPR2_USBSEL;

    /* VDDUSB supply valid, release UCPD dead-battery pull-downs */
    PWR->CR2 |= PWR_CR2_USV;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_UCPD1_STROBE | SYSCFG_CFGR1_UCPD2_STROBE;

    /* PA11/PA12 are dedicated USB pins on G0B1; leave GPIO in analog/reset. */

    tusb_rhport_init_t rh = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL };
    tusb_rhport_init(0, &rh);
}

void usb_task(void)
{
    tud_task();
}

void usb_write(const char *s)
{
    size_t len = strlen(s);
    if (tud_cdc_write_available() >= len) {
        tud_cdc_write(s, (uint32_t)len);
        tud_cdc_write_flush();
    }
}

int usb_read_char(void)
{
    if (tud_cdc_available()) {
        uint8_t c;
        if (tud_cdc_read(&c, 1) == 1) return c;
    }
    return -1;
}

void usb_request_bootloader(void)
{
    usb_write("Rebooting into DFU bootloader...\r\n");
    for (volatile uint32_t i = 0; i < 200000; i++) { tud_task(); }
    g_boot_magic = BOOT_MAGIC;
    NVIC_SystemReset();
}

/* Called from SystemInit() before anything else touches the hardware */
void usb_check_bootloader_request(void)
{
    if (g_boot_magic != BOOT_MAGIC) return;
    g_boot_magic = 0;
    const uint32_t *sysmem = (const uint32_t *)0x1FFF0000u;   /* system memory */
    __set_MSP(sysmem[0]);
    ((void (*)(void))sysmem[1])();
    for (;;) {}
}
