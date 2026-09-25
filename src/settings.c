/* Tiny settings store in the last 2 KB flash page.
 * Records are 8 bytes (one flash double word) appended until the page is
 * full, then the page is erased and we start again. Last valid record wins. */
#include "settings.h"
#include "stm32g0xx.h"
#include <string.h>

#define PAGE_SIZE      2048u
#define REC_MAGIC      0xA5u

typedef union {
    struct { uint8_t magic; uint8_t enabled; int16_t setpoint_c10; uint32_t check; } f;
    uint64_t raw;
} rec_t;

static uint32_t page_addr(void)
{
    uint32_t size = FLASH_SIZE;            /* from FLASHSIZE_BASE, bytes */
    return FLASH_BASE + size - PAGE_SIZE;
}

static uint32_t checksum(const rec_t *r)
{
    return 0x5EED0000u ^ (uint32_t)r->f.magic ^ ((uint32_t)r->f.enabled << 8)
         ^ ((uint32_t)(uint16_t)r->f.setpoint_c10 << 16);
}

static const rec_t *last_record(void)
{
    const rec_t *base = (const rec_t *)page_addr();
    const rec_t *found = 0;
    for (unsigned i = 0; i < PAGE_SIZE / sizeof(rec_t); i++) {
        if (base[i].raw == ~0ull) break;                   /* erased: end of log */
        if (base[i].f.magic == REC_MAGIC && base[i].f.check == checksum(&base[i]))
            found = &base[i];
    }
    return found;
}

bool settings_load(settings_t *out)
{
    const rec_t *r = last_record();
    if (!r) return false;
    out->setpoint_c10 = r->f.setpoint_c10;
    out->enabled = r->f.enabled;
    out->pad = 0;
    return true;
}

/* ---- low level flash ---- */
static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
}
static void flash_lock(void) { FLASH->CR |= FLASH_CR_LOCK; }

static void flash_wait(void)
{
    while (FLASH->SR & (FLASH_SR_BSY1 | FLASH_SR_CFGBSY)) {}
}

static void flash_clear_errors(void)
{
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR
              | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR
              | FLASH_SR_FASTERR | FLASH_SR_OPTVERR;
}

/* Bank layout exactly as ST's HAL derives it (FLASH_BANK_NB / FLASH_BANK_SIZE) */
static uint32_t bank_count(void)
{
    uint32_t sales = (*(volatile uint32_t *)PACKAGE_BASE) & (0x3u << 24);
    uint32_t dual  = FLASH->OPTR & FLASH_OPTR_DUAL_BANK;
    if (sales == 0u) return 1;
    if (sales == (0x1u << 24) && dual == 0u) return 1;
    return 2;
}

static bool flash_erase_page_at(uint32_t addr)
{
    uint32_t size  = FLASH_SIZE;
    uint32_t bsize = (bank_count() == 1) ? size : size / 2;
    uint32_t off   = addr - FLASH_BASE;
    uint32_t bank2 = (off >= bsize);
    uint32_t page  = (bank2 ? off - bsize : off) / PAGE_SIZE;

    flash_wait();
    flash_clear_errors();
    uint32_t cr = FLASH->CR & ~(FLASH_CR_PNB | FLASH_CR_BKER | FLASH_CR_PG);
    if (bank2) cr |= FLASH_CR_BKER;
    FLASH->CR = cr | (page << FLASH_CR_PNB_Pos) | FLASH_CR_PER | FLASH_CR_STRT;
    flash_wait();
    FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_BKER | FLASH_CR_PNB);
    bool ok = !(FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGSERR));
    /* verify */
    for (uint32_t a = addr; ok && a < addr + PAGE_SIZE; a += 4)
        if (*(volatile uint32_t *)a != 0xFFFFFFFFu) ok = false;
    return ok;
}

static bool flash_program_dword(uint32_t addr, uint64_t v)
{
    flash_wait();
    flash_clear_errors();
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint32_t *)addr       = (uint32_t)v;
    __ISB();
    *(volatile uint32_t *)(addr + 4) = (uint32_t)(v >> 32);
    flash_wait();
    FLASH->CR &= ~FLASH_CR_PG;
    return *(volatile uint64_t *)addr == v;
}

bool settings_save(const settings_t *in)
{
    rec_t r;
    r.raw = ~0ull;
    r.f.magic = REC_MAGIC;
    r.f.enabled = in->enabled;
    r.f.setpoint_c10 = in->setpoint_c10;
    r.f.check = checksum(&r);

    const rec_t *cur = last_record();
    if (cur && cur->raw == r.raw) return true;         /* unchanged */

    uint32_t base = page_addr();
    const rec_t *slots = (const rec_t *)base;
    unsigned i;
    for (i = 0; i < PAGE_SIZE / sizeof(rec_t); i++)
        if (slots[i].raw == ~0ull) break;

    __disable_irq();
    flash_unlock();
    bool ok = true;
    if (i >= PAGE_SIZE / sizeof(rec_t)) {              /* page full: wrap */
        ok = flash_erase_page_at(base);
        i = 0;
    }
    if (ok) ok = flash_program_dword(base + i * sizeof(rec_t), r.raw);
    flash_lock();
    __enable_irq();
    return ok;
}
