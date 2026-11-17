/*
 * ROM access log: which parts of the ROM the game reads, and how.
 *
 * Enabled with SNESRECOMP_ROMLOG=<file>. LakeSnes is left untouched: the
 * log wraps the emulated CPU's read/write handlers and sees translated code
 * through bus.c. Kinds per ROM byte:
 *   fetch   the CPU executes it (opcode or operand, just behind PC)
 *   data    the CPU reads it as data             first reader = PB:PC
 *   dma     a DMA reads it (seen at the $420B write)  reader = B-bus
 *                                                 register and VRAM/CGRAM
 *                                                 address at the start
 *   hdma    an HDMA table in ROM (walked at the $420C write)
 *   native  translated C code reads it (bus_read8)
 * The table is written at exit (binary, merged by tools/rom_catalog.py).
 * Only addresses and readers are recorded, never ROM contents.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "snes.h"
#include "cpu.h"
#include "dma.h"
#include "ppu.h"

extern Snes *snesrecomp_get_snes(void);

enum { K_FETCH = 1, K_DATA = 2, K_DMA = 4, K_HDMA = 8, K_NATIVE = 16 };

static uint32_t s_size;
static uint8_t  *s_kind;
static uint32_t *s_first[4];          /* fetch, data, dma, hdma */
static char      s_path[512];
static CpuReadHandler  s_read;
static CpuWriteHandler s_write;

bool g_romlog_on = false;

/* LoROM: the ROM offset of a CPU address, or -1 when it is not ROM. */
static int32_t rom_off(uint32_t adr) {
    uint8_t bank = (uint8_t)(adr >> 16);
    uint16_t a = (uint16_t)adr;
    if (((bank >= 0x70 && bank < 0x7E) || bank >= 0xF0) && a < 0x8000) return -1;  /* SRAM */
    if (bank == 0x7E || bank == 0x7F) return -1;                                    /* WRAM */
    bank &= 0x7F;
    if (a < 0x8000 && bank < 0x40) return -1;
    return (int32_t)((((uint32_t)bank << 15) | (a & 0x7FFF)) & (s_size - 1));
}

static void mark(int32_t off, int k, uint32_t who) {
    if (off < 0 || (uint32_t)off >= s_size) return;
    uint8_t bit = (uint8_t)(1u << k);
    if (!(s_kind[off] & bit)) {
        s_kind[off] |= bit;
        if (k < 4) s_first[k][off] = who;
    }
}

static uint8_t romlog_cpuRead(void *mem, uint32_t adr) {
    int32_t off = rom_off(adr);
    if (off >= 0) {
        Cpu *c = ((Snes *)mem)->cpu;
        uint16_t behind = (uint16_t)(c->pc - (uint16_t)adr);
        if ((uint8_t)(adr >> 16) == c->k && behind >= 1 && behind <= 4)
            mark(off, 0, 0);                                   /* opcode / operand */
        else
            mark(off, 1, ((uint32_t)c->k << 16) | c->pc);
    }
    return s_read(mem, adr);
}

static const int k_unit[8] = { 1, 2, 2, 4, 4, 4, 2, 4 };   /* bytes per transfer unit */

/* Before a $420B/$420C write takes effect: mark what each started channel
 * will read from the ROM. */
static void note_dma_start(uint16_t reg, uint8_t val) {
    Snes *sn = snesrecomp_get_snes();
    if (!sn || !sn->dma) return;
    for (int i = 0; i < 8; i++) {
        if (!(val & (1 << i))) continue;
        DmaChannel *ch = &sn->dma->channel[i];
        if (ch->fromB) continue;
        uint32_t base = (uint32_t)ch->aBank << 16;
        if (reg == 0x420B) {
            uint32_t n = ch->size ? ch->size : 0x10000;
            uint16_t at = (ch->bAdr == 0x18 || ch->bAdr == 0x19) ? sn->ppu->vramPointer
                        : (ch->bAdr == 0x22 ? sn->ppu->cgramPointer : 0);
            uint32_t who = ((uint32_t)ch->bAdr << 16) | at;
            uint16_t a = ch->aAdr;
            if (ch->fixed) n = 1;                            /* the same byte over and over */
            for (uint32_t j = 0; j < n; j++) {
                mark(rom_off(base | a), 2, who);
                if (!ch->fixed) a = (uint16_t)(ch->decrement ? a - 1 : a + 1);
            }
        } else {                                               /* HDMA table */
            uint16_t a = ch->aAdr;
            uint32_t who = (uint32_t)ch->bAdr << 16;
            int unit = ch->indirect ? 2 : k_unit[ch->mode & 7];
            for (int guard = 0; guard < 4096; guard++) {
                int32_t off = rom_off(base | a);
                if (off < 0) break;                            /* table in RAM */
                uint8_t c = sn->cart->rom[off];
                mark(off, 3, who);
                a++;
                if (c == 0) break;
                int units = ch->indirect ? 1 : ((c & 0x80) ? (c & 0x7F) : 1);
                for (int j = 0; j < units * unit; j++, a++) mark(rom_off(base | a), 3, who);
            }
        }
    }
}

void romlog_note_write(uint32_t adr, uint8_t val) {
    uint16_t a = (uint16_t)adr;
    uint8_t bank = (uint8_t)(adr >> 16);
    if ((a == 0x420B || a == 0x420C) && (bank < 0x40 || (bank >= 0x80 && bank < 0xC0)))
        note_dma_start(a, val);
}

void romlog_note_native(uint32_t adr) {
    int32_t off = rom_off(adr);
    if (off >= 0 && (uint32_t)off < s_size) s_kind[off] |= K_NATIVE;
}

static void romlog_cpuWrite(void *mem, uint32_t adr, uint8_t val) {
    romlog_note_write(adr, val);
    s_write(mem, adr, val);
}

/* Binary dump, merged across runs by tools/rom_catalog.py:
 *   "MPRL" u32 size, u8 kind[size], u32 first[4][size] (little endian). */
static void romlog_dump(void) {
    FILE *f = fopen(s_path, "wb");
    if (!f) return;
    fwrite("MPRL", 1, 4, f);
    fwrite(&s_size, 4, 1, f);
    fwrite(s_kind, 1, s_size, f);
    for (int k = 0; k < 4; k++) fwrite(s_first[k], 4, s_size, f);
    fclose(f);
    fprintf(stderr, "romlog: written %s\n", s_path);
}

void romlog_init(uint32_t rom_size) {
    const char *p = getenv("SNESRECOMP_ROMLOG");
    Snes *sn = snesrecomp_get_snes();
    if (!p || !*p || s_kind || !sn || !sn->cpu) return;
    snprintf(s_path, sizeof s_path, "%s", p);
    s_size = rom_size;
    s_kind = calloc(rom_size, 1);
    for (int k = 0; k < 4; k++) s_first[k] = calloc(rom_size, sizeof(uint32_t));
    s_read = sn->cpu->read;
    s_write = sn->cpu->write;
    sn->cpu->read = romlog_cpuRead;
    sn->cpu->write = romlog_cpuWrite;
    g_romlog_on = true;
    atexit(romlog_dump);
    fprintf(stderr, "romlog: logging ROM reads to %s\n", s_path);
}
