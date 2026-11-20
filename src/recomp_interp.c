/*
 * Interpreter fallback for not-yet-recompiled code.
 *
 * snesrecomp's model is "the recompiled native code IS the CPU" — the
 * LakeSnes CPU is normally never driven. That works only as far as code
 * has actually been hand-recompiled; any JSR/JSL into an un-recompiled
 * routine would otherwise be silently skipped.
 *
 * This module bridges the gap: when the dispatch table has no native
 * function for an address, we run the ORIGINAL 65816 code for that
 * subroutine on the real LakeSnes CPU, against the same hardware/WRAM the
 * recompiled code uses. Recompiled and interpreted code interoperate
 * freely because both ultimately read/write the same bus.
 *
 * Key design point: we do NOT advance SNES master-cycle timing while
 * interpreting. The normal LakeSnes CPU memory handlers (snes_cpuRead/
 * Write/Idle) call snes_runCycles(), which steps the PPU scanline renderer
 * and fires NMI/IRQ at vblank. snesrecomp drives the PPU and NMI manually
 * (see snesrecomp_end_frame / the recompiled NMI dispatch), so letting the
 * CPU advance timing here would double-render lines and fire spurious
 * interrupts mid-subroutine. Instead we temporarily swap in untimed memory
 * handlers that route through bus_read8/bus_write8 — exactly the same
 * untimed bus the recompiled functions use (including the DMA 2-step fix).
 */

#include "snesrecomp/func_table.h"
#include "snesrecomp/cpu.h"
#include "snesrecomp/bus.h"

#include "snes.h"
#include "cpu.h"
#include "apu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern Snes *snesrecomp_get_snes(void);

static bool s_interp_enabled = true;
static bool s_interp_force   = false;

void recomp_interp_set_enabled(bool enabled) { s_interp_enabled = enabled; }
bool recomp_interp_enabled(void) { return s_interp_enabled; }

/* Force mode: dispatch always interprets the original ROM, ignoring any
 * recompiled native function registered for the address. Lets the genuine
 * game logic drive the menus/gameplay while the recomp grows incrementally.
 * The recompiled frame "shells" (boot, NMI/main alternation, brightness)
 * still run in C; only the dispatched bodies are interpreted. */
void recomp_interp_set_force(bool force) { s_interp_force = force; }
bool recomp_interp_force(void) { return s_interp_force; }

/* --- Untimed memory handlers (route through the recomp bus) --- */

/* Advance the SPC700/APU when interpreted code touches the APU I/O ports
 * ($2140-$2143). Untimed interpretation never steps the APU, so code that
 * polls these ports for an SPC handshake (audio-driver upload, per-frame
 * sound-engine sync) would spin forever. Letting the APU catch up on each
 * port access lets those handshakes converge. The APU is independent of
 * PPU/CPU timing, so advancing it here doesn't disturb the manual frame
 * model. */
static void apu_catchup_if_port(Snes *snes, uint32_t adr) {
    uint16_t a = (uint16_t)adr;
    if (a >= 0x2140 && a <= 0x2143 && snes->apu) {
        apu_runCycles(snes->apu, 64);
    }
}

/* Advance the PPU H/V scan position when interpreted code polls the H/V-blank
 * status register ($4212/HVBJOY). Untimed interpretation never steps the PPU,
 * so hPos/vPos stay frozen and a "wait for HBlank/VBlank" spin-loop would never
 * exit. We bump the position directly (NOT via snes_runCycle, which would render
 * scanlines and fire NMI) so the blank flags cycle and the wait completes.
 * $4212: bit6 HBlank = (hPos<4 || hPos>=1096), bit7 VBlank = inVblank. */
static void ppu_status_advance(Snes *snes, uint32_t adr) {
    if ((uint16_t)adr != 0x4212) return;
    snes->hPos += 192;
    if (snes->hPos >= 1364) {
        snes->hPos -= 1364;
        snes->vPos++;
        if (snes->vPos >= 262) snes->vPos = 0;
        snes->inVblank = (snes->vPos >= 225);
    }
}

static uint8_t interp_read(void *mem, uint32_t adr) {
    Snes *snes = (Snes *)mem;
    if (snes) {
        apu_catchup_if_port(snes, adr);
        ppu_status_advance(snes, adr);
    }
    return bus_read8((uint8_t)(adr >> 16), (uint16_t)adr);
}

/* Optional debug: log interpreted writes to a watched address with the PC.
 *   SMK_WATCH_REG=<hex>   — match low 16 bits, I/O-capable banks only ($00-3F/$80-BF)
 *   SMK_WATCH_ADDR=<hex>  — match the full 24-bit address exactly (e.g. WRAM 7E9F40) */
static long s_watch_reg  = -2;
static long s_watch_addr = -2;

static void interp_write(void *mem, uint32_t adr, uint8_t val) {
    Snes *snes = (Snes *)mem;
    if (s_watch_reg == -2) {
        const char *w = getenv("SMK_WATCH_REG");
        s_watch_reg = w ? strtol(w, NULL, 16) : -1;
    }
    if (s_watch_addr == -2) {
        const char *w = getenv("SMK_WATCH_ADDR");
        s_watch_addr = w ? strtol(w, NULL, 16) : -1;
    }
    if (snes && snes->cpu) {
        /* I/O register watch: $21xx etc. are registers only in banks with bit6
         * clear ($00-3F / $80-BF); in $7E/$7F the same low address is WRAM. */
        if (s_watch_reg >= 0 && (uint16_t)adr == (uint16_t)s_watch_reg &&
            (((adr >> 16) & 0x40) == 0)) {
            fprintf(stderr, "WR $%04X <- %02X  pc=%02X:%04X\n",
                    (uint16_t)adr, val, snes->cpu->k, snes->cpu->pc);
        }
        if (s_watch_addr >= 0 && (adr & 0xFFFFFF) == (uint32_t)s_watch_addr) {
            fprintf(stderr, "WR $%06X <- %02X  pc=%02X:%04X\n",
                    adr & 0xFFFFFF, val, snes->cpu->k, snes->cpu->pc);
        }
    }
    if (snes) apu_catchup_if_port(snes, adr);
    bus_write8((uint8_t)(adr >> 16), (uint16_t)adr, val);
}

static void interp_idle(void *mem, bool waiting) {
    (void)mem;
    (void)waiting;
}

/* --- Register state sync between g_cpu and the LakeSnes CPU --- */

static void sync_to_lake(Cpu *c) {
    c->a  = g_cpu.C;
    c->x  = g_cpu.X;
    c->y  = g_cpu.Y;
    c->sp = g_cpu.S;
    c->dp = g_cpu.DP;
    c->db = g_cpu.DB;
    c->k  = g_cpu.PB;
    c->c  = g_cpu.flag_C;
    c->z  = g_cpu.flag_Z;
    c->i  = g_cpu.flag_I;
    c->d  = g_cpu.flag_D;
    c->xf = g_cpu.flag_X;
    c->mf = g_cpu.flag_M;
    c->v  = g_cpu.flag_V;
    c->n  = g_cpu.flag_N;
    c->e  = g_cpu.flag_E;
}

static void sync_from_lake(Cpu *c) {
    g_cpu.C      = c->a;
    g_cpu.X      = c->x;
    g_cpu.Y      = c->y;
    g_cpu.S      = c->sp;
    g_cpu.DP     = c->dp;
    g_cpu.DB     = c->db;
    g_cpu.PB     = c->k;
    g_cpu.flag_C = c->c;
    g_cpu.flag_Z = c->z;
    g_cpu.flag_I = c->i;
    g_cpu.flag_D = c->d;
    g_cpu.flag_X = c->xf;
    g_cpu.flag_M = c->mf;
    g_cpu.flag_V = c->v;
    g_cpu.flag_N = c->n;
    g_cpu.flag_E = c->e;
}

/* Safety cap: number of opcodes to interpret before bailing out. A correct
 * per-frame handler executes well under this; a routine that spins waiting
 * for NMI (which we drive externally) would otherwise loop forever. */
#define INTERP_MAX_OPS 20000000L

static unsigned long s_interp_calls = 0;   /* lets validation see a fallback */

void recomp_interp_call(uint32_t snes_addr, bool is_long) {
    s_interp_calls++;
    Snes *snes = snesrecomp_get_snes();
    if (!snes || !snes->cpu) return;
    Cpu *c = snes->cpu;

    /* Save and swap in untimed memory handlers */
    void            *saved_mem   = c->mem;
    CpuReadHandler   saved_read  = c->read;
    CpuWriteHandler  saved_write = c->write;
    CpuIdleHandler   saved_idle  = c->idle;
    c->mem   = snes;
    c->read  = interp_read;
    c->write = interp_write;
    c->idle  = interp_idle;

    sync_to_lake(c);

    /* No interrupts/halt may interfere: the recomp drives NMI itself, and we
     * never advance timing here so none would fire anyway. */
    c->waiting     = false;
    c->stopped     = false;
    c->nmiWanted   = false;
    c->intWanted   = false;
    c->irqWanted   = false;
    c->resetWanted = false;

    uint16_t start_sp = c->sp;

    /* Push a sentinel return frame. The values don't matter — completion is
     * detected purely by the stack pointer climbing back to start_sp when the
     * routine's terminating RTL/RTS pulls these bytes. JSL pushes a 24-bit
     * return (bank + 16-bit PC); JSR pushes a 16-bit return. */
    if (is_long) {
        bus_write8(0x00, c->sp, 0x00);  /* return bank */
        c->sp--;
    }
    bus_write8(0x00, c->sp, 0xFF);       /* return PC high */
    c->sp--;
    bus_write8(0x00, c->sp, 0xFF);       /* return PC low */
    c->sp--;

    c->k  = (snes_addr >> 16) & 0xFF;
    c->pc = (uint16_t)snes_addr;

    /* Optional control-flow trace: record (k:pc) before each opcode in a ring
     * buffer and dump the tail on abort, to locate runaway loops. */
    static int trace_on = -1;
    if (trace_on < 0) trace_on = getenv("SMK_INTERP_TRACE") ? 1 : 0;
    enum { TRC = 64 };
    static uint32_t trc[TRC];
    int trci = 0;

    /* Optional exec trace: when PC reaches SMK_TRACE_EXEC=<24-bit hex>, log the
     * caller's return address off the stack (works for routines reached via
     * indirect dispatch that static xref can't find). */
    static long trace_exec = -2;
    if (trace_exec == -2) {
        const char *te = getenv("SMK_TRACE_EXEC");
        trace_exec = te ? strtol(te, NULL, 16) : -1;
    }

    long ops = 0;
    bool aborted = false;
    /* Run until the stack unwinds back to (or past) the entry level. Using a
     * signed difference handles the exact-return case (== 0) and any overshoot
     * (< 0) without a separate check. */
    /* DMA-register guard across interrupts taken DURING an interpreted routine.
     *
     * snesrecomp doesn't advance PPU timing in interp, but interpreted writes to
     * $4200/$4207-$420A can still latch irqWanted, so an IRQ/NMI may be serviced
     * mid-routine. The handler can re-enter game code that programs DMA channel 0
     * (e.g. SMK's OBJ-palette CGRAM upload) — which on real hardware is harmless
     * because the interrupt fires at a different cycle, but here lands in the
     * MIDDLE of another routine's DMA-flush loop and leaves the channel's B-bus
     * address pointing at CGRAM. The resumed flush then DMAs VRAM tile data into
     * CGRAM, corrupting the Mode-7 race palette.
     *
     * Rather than suppress the interrupt (which would drop its legitimate work),
     * snapshot the DMA channel state when an interrupt is taken and restore it
     * once the handler's RTI unwinds the stack back. The interrupt's own DMAs
     * already executed (writing CGRAM/VRAM) before the restore, so they stick;
     * only the channel *registers* are rewound so the interrupted loop resumes
     * with the setup it had. */
    DmaChannel saved_dma[8];
    bool dma_guard_pending = false;
    uint16_t dma_guard_sp = 0;

    while ((int16_t)(start_sp - c->sp) > 0) {
        if (c->intWanted && !dma_guard_pending) {
            memcpy(saved_dma, snes->dma->channel, sizeof(saved_dma));
            dma_guard_sp = c->sp;       /* RTI will return SP to here */
            dma_guard_pending = true;
        }
        if (trace_on) { trc[trci % TRC] = ((uint32_t)c->k << 16) | c->pc; trci++; }
        if (trace_exec >= 0 && (((uint32_t)c->k << 16) | c->pc) == (uint32_t)trace_exec) {
            uint16_t sp = c->sp;
            uint16_t ret = bus_read8(0x00, (sp + 1) & 0xFFFF) |
                           (bus_read8(0x00, (sp + 2) & 0xFFFF) << 8);
            fprintf(stderr, "TRACE_EXEC $%06lX hit: sp=%04X ret=%04X (caller~%04X) entry-routine=$%06X\n",
                    (long)trace_exec, sp, ret, (uint16_t)(ret - 2), snes_addr);
        }
        cpu_runOpcode(c);
        /* Once the interrupt handler's RTI has unwound the stack back to (or
         * above) the pre-interrupt level, restore the DMA channel registers the
         * interrupted routine had programmed. */
        if (dma_guard_pending && (int16_t)(c->sp - dma_guard_sp) >= 0) {
            memcpy(snes->dma->channel, saved_dma, sizeof(saved_dma));
            dma_guard_pending = false;
        }
        if (c->waiting || c->stopped) {
            fprintf(stderr, "recomp_interp: $%06X executed WAI/STP — aborting "
                            "(this routine likely waits on NMI)\n", snes_addr);
            aborted = true;
            break;
        }
        if (++ops > INTERP_MAX_OPS) {
            fprintf(stderr, "recomp_interp: $%06X exceeded %ld-op cap "
                            "(sp=%04X start=%04X pc=%02X:%04X) — aborting\n",
                    snes_addr, (long)INTERP_MAX_OPS, c->sp, start_sp, c->k, c->pc);
            aborted = true;
            if (trace_on) {
                fprintf(stderr, "  trace (last %d opcodes, oldest first):\n", TRC);
                int start = (trci > TRC) ? (trci - TRC) : 0;
                for (int i = start; i < trci; i++) {
                    fprintf(stderr, " %06X", trc[i % TRC]);
                    if ((i - start) % 12 == 11) fprintf(stderr, "\n");
                }
                fprintf(stderr, "\n");
            }
            break;
        }
    }

    /* On abort, the routine left an unbalanced stack. Restore the entry stack
     * pointer so the bailout doesn't leak frames into subsequent calls (which
     * would otherwise drift the stack down until it corrupts WRAM/overflows). */
    if (aborted) c->sp = start_sp;

    /* SNESRECOMP_INTERP_STATS=1 — one line per interpreted routine: how many
     * opcodes it actually ran and where it stopped. A routine that "returns"
     * after a handful of ops has almost certainly tripped the stack-unwind
     * completion check early rather than genuinely finishing. */
    if (getenv("SNESRECOMP_INTERP_STATS"))
        fprintf(stderr, "[INTERP] $%06X ran %ld ops, end pc=%02X:%04X sp=%04X (start_sp=%04X)%s\n",
                snes_addr, ops, c->k, c->pc, c->sp, start_sp, aborted ? " ABORTED" : "");

    sync_from_lake(c);

    /* Restore the original (timed) memory handlers */
    c->mem   = saved_mem;
    c->read  = saved_read;
    c->write = saved_write;
    c->idle  = saved_idle;
}

/* ===== Phase-1 timed-recomp interception ==================================
 *
 * Unlike recomp_interp_call (above), which runs ROM on the LakeSnes CPU with
 * UNTIMED handlers for the old manual-frame model, this path runs inside the
 * TIMED frame loop (snes_runFrame, real PPU/APU/NMI timing — same as
 * real-frame mode). The CPU calls g_cpuRecompHook at every opcode fetch; when
 * PB:PC matches a registered entry we run the recompiled native body in place
 * of the ROM subroutine and advance PB:PC past it via a simulated RTS/RTL.
 *
 * Cycles "skipped" by running the native body instantly are reabsorbed by the
 * genuine code's next vblank spin-wait (which keeps stepping the timed CPU
 * until the PPU actually reaches vblank), so ordinary (non-raster-timed)
 * routines need no per-function cycle accounting. */

extern bool (*g_cpuRecompHook)(Cpu*);

typedef struct { uint32_t addr; bool is_long; bool framed; } TimedIntercept;
static TimedIntercept s_intercepts[1024];
static int            s_intercept_count = 0;
static unsigned long  s_intercept_hits  = 0;

void recomp_timed_add_intercept(uint32_t snes_addr, bool is_long) {
    for (int i = 0; i < s_intercept_count; i++)
        if (s_intercepts[i].addr == snes_addr) return;   /* dedupe */
    if (s_intercept_count < (int)(sizeof(s_intercepts) / sizeof(s_intercepts[0]))) {
        s_intercepts[s_intercept_count].addr    = snes_addr;
        s_intercepts[s_intercept_count].is_long = is_long;
        s_intercepts[s_intercept_count].framed  = false;
        s_intercept_count++;
    } else {
        fprintf(stderr, "recomp: intercept table full, $%06X stays on the ROM\n", snes_addr);
    }
}

void recomp_timed_add_framed_intercept(uint32_t snes_addr, bool is_long) {
    recomp_timed_add_intercept(snes_addr, is_long);
    for (int i = 0; i < s_intercept_count; i++)
        if (s_intercepts[i].addr == snes_addr) s_intercepts[i].framed = true;
}

unsigned long recomp_timed_intercept_hits(void) { return s_intercept_hits; }

/* Depth of native bodies currently running inside the timed loop. */
static int s_timed_depth = 0;

bool recomp_timed_spin(int master_cycles) {
    Snes *sn = snesrecomp_get_snes();
    if (!s_timed_depth || !sn) return false;
    snes_runCycles(sn, master_cycles);
    return true;
}

/* Redirect protocol: an intercepted function whose original ends in a JMP/JML
 * (not RTS/RTL) calls recomp_set_redirect(target) instead of returning; the
 * hook then sets PB:PC to the target (no stack pop), matching the jump. */
static uint32_t s_redirect = 0xFFFFFFFFu;   /* sentinel = no redirect */
void recomp_set_redirect(uint32_t snes_addr) { s_redirect = snes_addr; }

/* --- Call-target profiler (Phase-3 target finding) -----------------------
 * When enabled, tallies the target of every direct JSR/JSL the timed CPU
 * executes, so the hottest leaves (best recompilation targets) can be ranked.
 * Indirect dispatch (JSR ($table,x)) is not target-resolvable here and is
 * skipped — leaves are virtually always called directly. */
/* Records each direct JSR/JSL target with the M/X flags the callee enters with
 * (= the caller's current flags; JSR/JSL preserve P), and a multi-flag marker so
 * the auto-generator can skip per-(M,X) variant targets. */
typedef struct { uint32_t addr; unsigned long count; uint8_t m, x, multi, jsl; } CallTally;
static CallTally s_tally[8192];
static int       s_tally_n  = 0;
static bool      s_profile  = false;

void recomp_timed_profile_enable(void) { s_profile = true; }

static void tally_call(uint32_t target, uint8_t m, uint8_t x, uint8_t jsl) {
    for (int i = 0; i < s_tally_n; i++)
        if (s_tally[i].addr == target) {
            s_tally[i].count++;
            if (s_tally[i].m != m || s_tally[i].x != x) s_tally[i].multi = 1;
            if (i > 0) {  /* move-to-front: keep hot targets cheap to find */
                CallTally t = s_tally[i]; s_tally[i] = s_tally[i - 1]; s_tally[i - 1] = t;
            }
            return;
        }
    if (s_tally_n < (int)(sizeof(s_tally) / sizeof(s_tally[0]))) {
        s_tally[s_tally_n].addr = target;
        s_tally[s_tally_n].count = 1;
        s_tally[s_tally_n].m = m; s_tally[s_tally_n].x = x; s_tally[s_tally_n].multi = 0;
        s_tally[s_tally_n].jsl = jsl;
        s_tally_n++;
    }
}

void recomp_timed_profile_dump(int top) {
    fprintf(stderr, "=== timed-recomp call profile: %d distinct JSR/JSL targets ===\n", s_tally_n);
    fprintf(stderr, "# addr   count    P(entry) call flags\n");
    for (int k = 0; k < top; k++) {
        int best = -1; unsigned long bc = 0;
        for (int i = 0; i < s_tally_n; i++)
            if (s_tally[i].count > bc) { bc = s_tally[i].count; best = i; }
        if (best < 0) break;
        CallTally *t = &s_tally[best];
        uint8_t P = (uint8_t)((t->m ? 0x20 : 0) | (t->x ? 0x10 : 0));
        fprintf(stderr, "PROF %06X %-8lu %02X %s %s%s\n", t->addr, t->count, P, t->jsl ? "JSL" : "JSR",
                t->multi ? "MULTI-MX " : "",
                func_table_lookup(t->addr) ? "recompiled" : "");
        t->count = 0;  /* consume so the next pass finds the next-highest */
    }
}

/* --- Lockstep validation ---------------------------------------------------
 *
 * Checks a recompiled function against the genuine routine on every call,
 * without letting it change the run. At each entry: save the machine, run the
 * native body, keep what it produced, restore, and let the ROM routine run
 * for real. When the ROM routine returns (PB:PC = return address, S back to
 * its pre-call value), compare WRAM, VRAM, CGRAM and OAM with what the native
 * body produced.
 *
 * Intercepting instead (recomp_timed_add_intercept) is not a usable test: the
 * native body costs zero cycles, so later work lands on different frames and a
 * correct function still produces a different snapshot. Here execution stays
 * genuine, so any difference is the function's own.
 *
 * The last value written to each PPU/CPU I/O register ($2100-$21FF,
 * $4200-$43FF) is compared too, since a register write (INIDISP brightness,
 * a screen enable) leaves no trace in memory. Data ports ($2104, $2118/9,
 * $2122, $2180) and the joypad strobe $4016 are left out: their effect is the
 * memory they fill, which is compared anyway.
 *
 * Bytes just below the entry stack pointer are not compared: the ROM routine
 * pushes there and a native body does not. Registers are compared but reported
 * separately (a hand-port often leaves A/X/Y different where no caller looks).
 * A native body that exits through recomp_set_redirect is counted as skipped.
 */
/* ---- Coroutines for frame-driving native bodies -------------------------
 *
 * A native body that waits for frames (a menu loop, a fade, a pause screen)
 * cannot be checked as one call: the genuine routine spans many frames, with
 * NMIs and input in between. So each native body under check runs on a
 * stack of its own. When it calls a registered frame driver
 * (recomp_timed_add_frame_driver) it is suspended there instead; the genuine
 * routine runs until it enters a frame driver too, and the machine is
 * compared at that point. The genuine frame driver then runs for real, and
 * once it returns the native body resumes on that machine state, with the
 * registers the driver returned, up to its next frame or its end. Each piece
 * between two frames is checked like a whole call of an ordinary routine.
 * Frames are paired in order: the k-th frame the native body waits for with
 * the k-th frame driver the genuine routine enters (the stack depth cannot
 * pair them: native bodies do not mirror the ROM's PHA/PHP pushes).
 */
#ifdef _WIN32
#include <windows.h>
typedef struct { LPVOID fiber, ret; } CoCtx;   /* ret: who switched in */
static LPVOID s_main_fiber = NULL;
#else
#include <ucontext.h>
typedef struct { ucontext_t uc, ret; char *stack; } CoCtx;   /* ret: who switched in */
#endif
#define CO_STACK (1 << 20)

typedef struct { uint32_t addr; bool is_long; } FrameDriver;
static FrameDriver s_fd[16];
static int         s_fd_n = 0;

void recomp_timed_add_frame_driver(uint32_t snes_addr, bool is_long) {
    for (int i = 0; i < s_fd_n; i++)
        if (s_fd[i].addr == snes_addr) return;
    if (s_fd_n < (int)(sizeof(s_fd) / sizeof(s_fd[0]))) {
        s_fd[s_fd_n].addr = snes_addr;
        s_fd[s_fd_n].is_long = is_long;
        s_fd_n++;
    }
}
static const FrameDriver *frame_driver(uint32_t addr) {
    for (int i = 0; i < s_fd_n; i++)
        if (s_fd[i].addr == addr) return &s_fd[i];
    return NULL;
}
bool recomp_is_frame_driver(uint32_t snes_addr) { return frame_driver(snes_addr) != NULL; }

typedef struct {
    uint32_t addr; bool is_long;
    unsigned long calls, mem_fail, reg_fail, skipped, mx_fail;
    unsigned long irq;   /* calls an NMI/IRQ handler interrupted */
    unsigned long frames;   /* frames waited for inside checked calls */
    unsigned long irqdep;   /* calls not comparable: an interrupt changed WRAM the native body read */
    char first[200];
    char first_reg[160];
} Validate;
static Validate s_val[256];
static int      s_val_n = 0;

typedef struct {
    int idx; uint8_t k; uint16_t pc, sp, entry_sp;
    uint8_t *ram; uint16_t *vram, *cgram, *oam; uint8_t *hoam;
    uint16_t a, x, y, dp; uint8_t db, c, z, n, v, mf, xf;
    uint8_t nio[0x300], nio_set[0x300];   /* native body's register writes */
    uint8_t gio[0x300], gio_set[0x300];   /* genuine routine's */
    uint8_t irq_hit;                      /* an interrupt ran mid-routine */
    uint8_t *irq_mask;                    /* WRAM bytes its handler wrote */
    uint8_t *irq_vmask;                   /* VRAM/CGRAM/OAM words it changed: */
                                          /* bits 0-$7FFF VRAM, then $100 CGRAM, */
                                          /* $100 OAM, $20 high OAM */
    /* Frame-driving bodies (see the coroutine note above). */
    int phase;                            /* PH_* below */
    snes_func_t fn;
    CoCtx co;
    bool co_yielded;                      /* stopped at a frame driver (else ended) */
    uint32_t fd_addr;                     /* the driver it stopped at */
    bool sync;                            /* stopped at a ROM sync point, not a driver */
    bool sync_met;                        /* the genuine routine reached spin_pc: compared */
    uint32_t spin_pc;                     /* where the ROM's wait begins (compare there) */
    uint32_t sync_pc;                     /* where it ends (the body goes on there) */
    uint16_t native_s;                    /* its S there (native view) */
    uint8_t ret_k; uint16_t ret_pc, ret_sp;   /* where the genuine driver returns */
    unsigned frames;                      /* frames passed so far */
    uint8_t *read_mask;                   /* WRAM the native body read (bit per byte) */
} Pending;
enum { PH_FINAL,      /* native body ended: compare at the routine's return */
       PH_SEG,        /* native waits at a frame driver: compare when the ROM enters one */
       PH_FRAME,      /* genuine frame driver running: resume the native body when it returns */
       PH_SYNC,       /* native waits at a ROM sync point: compare when the ROM reaches it */
       PH_DONE };     /* decided early (failed or skipped): nothing more to compare */
#define VAL_DEPTH 8
static Pending s_pend[VAL_DEPTH];
static int     s_pend_n = 0;
static uint8_t *s_state_buf = NULL;
static Pending *s_native_cap = NULL;          /* set while a native body runs */
static CpuWriteHandler s_genuine_write = NULL; /* CPU write handler we wrap */
/* An NMI/IRQ handler taken while a genuine routine is under check: its
 * writes land in the genuine run but can never happen in the native body,
 * which runs between frames. Track them so the compare can leave them out. */
static bool     s_irq_active = false;
static uint16_t s_irq_sp;
static int      s_irq_base;   /* routines pending when it fired: s_pend[0..base) */
#define IRQ_V_BITS (0x8000 + 0x100 + 0x100 + 0x20)
static uint16_t s_irq_vram[0x8000], s_irq_cgram[0x100], s_irq_oam[0x100];
static uint8_t  s_irq_hoam[0x20];
static int wram_offset(uint32_t adr) {
    uint8_t bank = (uint8_t)(adr >> 16); uint16_t a = (uint16_t)adr;
    if (bank == 0x7E || bank == 0x7F) return ((bank - 0x7E) << 16) | a;
    if ((bank < 0x40 || (bank >= 0x80 && bank < 0xC0)) && a < 0x2000) return a;
    return -1;
}

static int io_index(uint32_t adr) {
    uint8_t bank = (uint8_t)(adr >> 16);
    uint16_t a = (uint16_t)adr;
    if (bank >= 0x40 && bank < 0x80) return -1;
    if (bank >= 0xC0) return -1;
    if (a == 0x2104 || a == 0x2118 || a == 0x2119 || a == 0x2122 || a == 0x2180) return -1;
    if (a >= 0x2100 && a <= 0x21FF) return a - 0x2100;
    if (a >= 0x4200 && a <= 0x43FF) return 0x100 + (a - 0x4200);
    return -1;
}

void recomp_validate_note_write(uint32_t adr, uint8_t val) {
    int i;
    if (!s_native_cap || (i = io_index(adr)) < 0) return;
    s_native_cap->nio[i] = val;
    s_native_cap->nio_set[i] = 1;
}

static void genuine_write(void *mem, uint32_t adr, uint8_t val) {
    int i = io_index(adr);
    /* Routines entered inside the handler see its writes as their own; the
     * ones it interrupted get them masked instead. */
    int first = s_irq_active ? s_irq_base : 0;
    if (i >= 0)
        for (int k = first; k < s_pend_n; k++) { s_pend[k].gio[i] = val; s_pend[k].gio_set[i] = 1; }
    if (s_irq_active) {
        int w = wram_offset(adr);
        if (w >= 0)
            for (int k = 0; k < s_irq_base && k < s_pend_n; k++)
                s_pend[k].irq_mask[w >> 3] |= (uint8_t)(1 << (w & 7));
    }
    s_genuine_write(mem, adr, val);
}
extern bool g_recomp_validating, g_recomp_frame_touched;

void recomp_timed_add_validate(uint32_t snes_addr, bool is_long) {
    for (int i = 0; i < s_val_n; i++)
        if (s_val[i].addr == snes_addr) return;
    if (s_val_n < (int)(sizeof(s_val) / sizeof(s_val[0]))) {
        memset(&s_val[s_val_n], 0, sizeof(Validate));
        s_val[s_val_n].addr = snes_addr;
        s_val[s_val_n].is_long = is_long;
        s_val_n++;
    }
}

void recomp_timed_validate_report(void) {
    for (int i = 0; i < s_val_n; i++) {
        Validate *v = &s_val[i];
        fprintf(stderr, "VALIDATE %06X calls=%lu mem_fail=%lu reg_fail=%lu mx_fail=%lu skipped=%lu irq=%lu frames=%lu irqdep=%lu%s%s%s%s\n",
                v->addr, v->calls, v->mem_fail, v->reg_fail, v->mx_fail, v->skipped, v->irq, v->frames, v->irqdep,
                v->first[0] ? " first: " : "", v->first,
                v->first_reg[0] ? " | reg: " : "", v->first_reg);
    }
}

/* WRAM/VRAM/CGRAM/OAM/I/O differences between the live (genuine) machine and
 * what the native body produced; the first one goes to buf. */
static int validate_diff(Pending *p, char *buf, size_t bufsz) {
    Snes *sn = snesrecomp_get_snes();
    int n = 0;
    buf[0] = 0;
    for (int i = 0; i < 0x20000; i++) {
        if (i <= p->entry_sp && i > p->entry_sp - 0x100) continue;   /* stack scratch */
        if (p->irq_hit && (p->irq_mask[i >> 3] & (1 << (i & 7)))) continue;  /* handler wrote it */
        if (sn->ram[i] != p->ram[i] && n++ == 0)
            snprintf(buf, bufsz, "wram $%05X rom=%02X native=%02X", i, sn->ram[i], p->ram[i]);
    }
    /* An interrupted call: what the handler wrote (WRAM bytes, video words,
     * I/O registers) is left out; everything else is compared as usual. */
#define VMASKED(b) (p->irq_hit && (p->irq_vmask[(b) >> 3] & (1 << ((b) & 7))))
    for (int i = 0; i < 0x8000; i++)
        if (!VMASKED(i) && sn->ppu->vram[i] != p->vram[i] && n++ == 0)
            snprintf(buf, bufsz, "vram word $%04X rom=%04X native=%04X", i, sn->ppu->vram[i], p->vram[i]);
    for (int i = 0; i < 0x100; i++)
        if (!VMASKED(0x8000 + i) && sn->ppu->cgram[i] != p->cgram[i] && n++ == 0)
            snprintf(buf, bufsz, "cgram $%02X rom=%04X native=%04X", i, sn->ppu->cgram[i], p->cgram[i]);
    for (int i = 0; i < 0x100; i++)
        if (!VMASKED(0x8100 + i) && sn->ppu->oam[i] != p->oam[i] && n++ == 0)
            snprintf(buf, bufsz, "oam word $%02X rom=%04X native=%04X", i, sn->ppu->oam[i], p->oam[i]);
    for (int i = 0; i < 0x20; i++)
        if (!VMASKED(0x8200 + i) && sn->ppu->highOam[i] != p->hoam[i] && n++ == 0)
            snprintf(buf, bufsz, "high oam $%02X rom=%02X native=%02X", i, sn->ppu->highOam[i], p->hoam[i]);
    for (int i = 0; i < 0x300; i++) {
        if (p->nio_set[i] == p->gio_set[i] && (!p->nio_set[i] || p->nio[i] == p->gio[i])) continue;
        unsigned reg = i < 0x100 ? 0x2100 + i : 0x4200 + (i - 0x100);
        if (n++ == 0)
            snprintf(buf, bufsz, "io $%04X rom=%s%02X native=%s%02X", reg,
                     p->gio_set[i] ? "" : "unwritten/", p->gio[i],
                     p->nio_set[i] ? "" : "unwritten/", p->nio[i]);
    }
#undef VMASKED
    return n;
}

static void record_fail(Pending *p, const char *what, int n) {
    Snes *sn = snesrecomp_get_snes();
    Validate *v = &s_val[p->idx];
    v->mem_fail++;
    if (!v->first[0]) {
        if (p->frames)
            snprintf(v->first, sizeof(v->first), "call %lu, frame %u (after %u frames in it): %s (%d differ)",
                     v->calls, sn->frames, p->frames, what, n);
        else
            snprintf(v->first, sizeof(v->first), "call %lu, frame %u: %s (%d differ)", v->calls, sn->frames, what, n);
    }
}

/* Did an interrupt that landed in the genuine call write WRAM the native body
 * read? Then the genuine routine may have acted on the handler's value (a
 * mouse button, a frame counter) that the native body, run beforehand,
 * could not see: the call is not comparable. */
static bool irq_read_overlap(const Pending *p) {
    if (!p->irq_hit) return false;
    for (int i = 0; i < 0x20000 / 8; i++)
        if (p->irq_mask[i] & p->read_mask[i]) return true;
    return false;
}

static void validate_compare(Cpu *c, Pending *p) {
    Validate *v = &s_val[p->idx];
    char buf[200];
    int n = validate_diff(p, buf, sizeof(buf));
    if (n && irq_read_overlap(p)) { v->irqdep++; return; }
    if (p->irq_hit) v->irq++;
    v->calls++;
    if (n) {
        record_fail(p, buf, n);
    } else if (c->a != p->a || c->x != p->x || c->y != p->y || c->db != p->db || c->dp != p->dp ||
               c->c != p->c || c->z != p->z || c->n != p->n || c->v != p->v ||
               c->mf != p->mf || c->xf != p->xf) {
        v->reg_fail++;
        if (c->mf != p->mf || c->xf != p->xf) v->mx_fail++;   /* caller runs at the wrong width */
        if (!v->first_reg[0])
            snprintf(v->first_reg, sizeof(v->first_reg),
                     "rom A=%04X X=%04X Y=%04X C%d Z%d N%d V%d M%d X%d / native A=%04X X=%04X Y=%04X C%d Z%d N%d V%d M%d X%d",
                     c->a, c->x, c->y, c->c, c->z, c->n, c->v, c->mf, c->xf,
                     p->a, p->x, p->y, p->c, p->z, p->n, p->v, p->mf, p->xf);
    }
}

/* The native body of p, on its own stack: started, or resumed after a frame. */
static Pending *s_co_cur = NULL;
extern uint8_t *g_bus_read_mark;   /* bus.c */

#ifdef _WIN32
static void WINAPI co_entry(LPVOID arg) {
    Pending *p = (Pending *)arg;
    p->fn();
    p->co_yielded = false;
    SwitchToFiber(p->co.ret);
}
#else
static void co_entry(void) {
    Pending *p = s_co_cur;
    p->fn();
    p->co_yielded = false;
    swapcontext(&p->co.uc, &p->co.ret);
}
#endif

static void co_start(Pending *p) {
#ifdef _WIN32
    if (!s_main_fiber) s_main_fiber = ConvertThreadToFiber(NULL);
    if (p->co.fiber) DeleteFiber(p->co.fiber);
    p->co.fiber = CreateFiber(CO_STACK, co_entry, p);
#else
    if (!p->co.stack) p->co.stack = (char *)malloc(CO_STACK);
    getcontext(&p->co.uc);
    p->co.uc.uc_stack.ss_sp = p->co.stack;
    p->co.uc.uc_stack.ss_size = CO_STACK;
    p->co.uc.uc_link = NULL;
    makecontext(&p->co.uc, co_entry, 0);
#endif
}

static void co_switch_in(Pending *p) {
    s_co_cur = p;
#ifdef _WIN32
    p->co.ret = GetCurrentFiber();
    SwitchToFiber(p->co.fiber);
#else
    swapcontext(&p->co.ret, &p->co.uc);
#endif
    s_co_cur = NULL;
}

/* Called by func_table dispatch while a native body under check calls addr.
 * A registered frame driver suspends the body (true: the caller then
 * continues as if the driver had returned, with what the genuine one
 * returned). */
static bool intercept_frame_call(uint32_t snes_addr);
bool recomp_validate_frame_call(uint32_t snes_addr) {
    Pending *p = s_co_cur;
    if (!p) return intercept_frame_call(snes_addr);
    if (!frame_driver(snes_addr)) return false;
    p->co_yielded = true;
    p->fd_addr = snes_addr;
    p->native_s = g_cpu.S;
#ifdef _WIN32
    SwitchToFiber(p->co.ret);
#else
    swapcontext(&p->co.uc, &p->co.ret);
#endif
    return true;
}

/* Run (resume = false: start) the native body on the live machine until it
 * ends or waits for a frame; keep what it produced in p and put the machine
 * back. False if the piece cannot be checked: it exits by a jump, presents a
 * frame itself, or falls back to the interpreter for a callee. */
static bool run_native(Cpu *c, Pending *p, bool resume) {
    Snes *sn = snesrecomp_get_snes();
    int size = snes_saveState(sn, s_state_buf);
    s_native_cap = p;
    unsigned long interp_before = s_interp_calls;
    sync_from_lake(c);
    if (resume) g_cpu.S = p->native_s;          /* its call_jsl/call_jsr undo their own frame */
    else co_start(p);
    s_redirect = 0xFFFFFFFFu;
    s_timed_depth++;
    g_recomp_validating = true;
    g_recomp_frame_touched = false;
    p->co_yielded = false;
    p->sync = false;
    g_bus_read_mark = p->read_mask;
    co_switch_in(p);
    g_bus_read_mark = NULL;
    g_recomp_validating = false;
    s_native_cap = NULL;
    s_timed_depth--;
    sync_to_lake(c);
    bool ok = s_redirect == 0xFFFFFFFFu && !g_recomp_frame_touched && s_interp_calls == interp_before;
    if (!ok && getenv("SNESRECOMP_VALIDATE_WHY"))
        fprintf(stderr, "WHY $%06X skipped: redirect %06X, frame touched %d, interpreter calls %lu\n",
                s_val[p->idx].addr, s_redirect == 0xFFFFFFFFu ? 0 : s_redirect, g_recomp_frame_touched,
                s_interp_calls - interp_before);
    s_redirect = 0xFFFFFFFFu;
    memcpy(p->ram, sn->ram, 0x20000);
    memcpy(p->vram, sn->ppu->vram, 0x10000);
    memcpy(p->cgram, sn->ppu->cgram, 0x200);
    memcpy(p->oam, sn->ppu->oam, 0x200);
    memcpy(p->hoam, sn->ppu->highOam, 0x20);
    p->a = c->a; p->x = c->x; p->y = c->y; p->dp = c->dp; p->db = c->db;
    p->c = c->c; p->z = c->z; p->n = c->n; p->v = c->v; p->mf = c->mf; p->xf = c->xf;
    /* A body that ended on another stack (LDA saved; TCS; RTS: leave a game
     * loop) returns where that stack says, as the intercept's simulated
     * RTS/RTL takes it: the genuine routine is compared there. */
    if (!p->co_yielded && c->sp != p->entry_sp) {
        uint16_t sp = c->sp;
        uint16_t ret = (uint16_t)(p->ram[(uint16_t)(sp + 1)] | (p->ram[(uint16_t)(sp + 2)] << 8));
        bool is_long = s_val[p->idx].is_long;
        p->pc = (uint16_t)(ret + 1);
        p->k  = is_long ? p->ram[(uint16_t)(sp + 3)] : c->k;
        p->sp = (uint16_t)(sp + (is_long ? 3 : 2));
    }
    if (!snes_loadState(sn, s_state_buf, size)) {
        fprintf(stderr, "validate: state restore failed (size %d)\\n", size);
        abort();
    }
    return ok;
}

/* A fresh piece starts from the machine as it is now: forget what the last
 * one saw (interrupts, register writes). */
static void reset_piece(Pending *p) {
    p->irq_hit = 0;
    memset(p->read_mask, 0, 0x20000 / 8);
    memset(p->irq_mask, 0, 0x20000 / 8);
    memset(p->irq_vmask, 0, (IRQ_V_BITS + 7) / 8);
    memset(p->nio_set, 0, sizeof(p->nio_set));
    memset(p->gio_set, 0, sizeof(p->gio_set));
}

/* Frame bookkeeping for routines under check, at each genuine opcode. */
static void validate_frames(Cpu *c, uint32_t pc) {
    const FrameDriver *fd = frame_driver(pc);
    if (fd && !s_irq_active) {
        /* The genuine code enters a frame driver: for every routine it runs
         * inside, this is the next frame. */
        for (int k = 0; k < s_pend_n; k++) {
            Pending *p = &s_pend[k];
            if (c->sp >= p->entry_sp || (p->phase != PH_SEG && p->phase != PH_FINAL && p->phase != PH_SYNC)) continue;
            Validate *v = &s_val[p->idx];
            /* A body waiting at a sync point lets the ROM run up to it,
             * frames included (the ROM loop it waits in may take them). */
            if (p->phase == PH_SYNC) { p->frames++; continue; }
            if (p->phase == PH_FINAL) {
                v->calls++;
                record_fail(p, "the ROM waits for a frame here, the native body had already returned", 1);
                p->phase = PH_DONE;
                continue;
            }
            char buf[200], what[260];
            int n = validate_diff(p, buf, sizeof(buf));
            if (getenv("SNESRECOMP_VALIDATE_FTRACE"))
                fprintf(stderr, "FTRACE $%06X frame %u: ROM driver $%06X from $%02X%04X, native $%06X, %d differ\n",
                        s_val[p->idx].addr, p->frames, pc, fd->is_long ? bus_read8(0x00, (c->sp + 3) & 0xFFFF) : c->k,
                        (unsigned)(bus_read8(0x00, (c->sp + 1) & 0xFFFF) | (bus_read8(0x00, (c->sp + 2) & 0xFFFF) << 8)),
                        p->fd_addr, n);
            if (p->fd_addr != pc) {
                snprintf(what, sizeof(what), "ROM frame driver $%06X, native $%06X", pc, p->fd_addr);
                v->calls++; record_fail(p, what, 1); p->phase = PH_DONE;
            } else if (n && irq_read_overlap(p)) {
                v->irqdep++; p->phase = PH_DONE;
            } else if (n) {
                snprintf(what, sizeof(what), "before frame driver $%06X: %s", pc, buf);
                v->calls++; record_fail(p, what, n); p->phase = PH_DONE;
            } else {
                uint16_t sp = c->sp;
                uint16_t ret = (uint16_t)(bus_read8(0x00, (sp + 1) & 0xFFFF) | (bus_read8(0x00, (sp + 2) & 0xFFFF) << 8));
                p->ret_pc = (uint16_t)(ret + 1);
                p->ret_k  = fd->is_long ? bus_read8(0x00, (sp + 3) & 0xFFFF) : c->k;
                p->ret_sp = (uint16_t)(sp + (fd->is_long ? 3 : 2));
                p->phase = PH_FRAME;
            }
        }
        return;
    }
    /* A genuine frame driver returned: those routines' native bodies go on
     * from here, on the machine the frame left. */
    for (int k = 0; k < s_pend_n; k++) {
        Pending *p = &s_pend[k];
        if (p->phase != PH_FRAME || c->k != p->ret_k || c->pc != p->ret_pc || c->sp != p->ret_sp) continue;
        p->frames++;
        s_val[p->idx].frames++;
        reset_piece(p);
        if (!run_native(c, p, true)) { s_val[p->idx].skipped++; p->phase = PH_DONE; continue; }
        p->phase = p->co_yielded ? (p->sync ? PH_SYNC : PH_SEG) : PH_FINAL;
    }
}

/* Called from the timed hook at every opcode. True = pc was a validated entry
 * (the genuine routine now runs; the hook must not intercept it). */
static bool validate_hook(Cpu *c) {
    /* A native body under check can fall back to the interpreter, which comes
     * back through this hook. Nothing it runs is genuine, and a nested check
     * would overwrite the saved state the outer one restores from. */
    if (g_recomp_validating) return false;
    uint32_t pc = ((uint32_t)c->k << 16) | (uint16_t)c->pc;

    /* Interrupt bookkeeping: the handler is entered at its vector with the
     * 4-byte interrupt frame pushed, and is done once RTI lifts the stack
     * back above that point. */
    if (s_pend_n > 0) {
        if (s_irq_active && c->sp > s_irq_sp) {
            /* Handler done: whatever video memory it changed is its own. */
            Snes *sv = snesrecomp_get_snes();
            for (int k = 0; k < s_irq_base && k < s_pend_n; k++) {
                uint8_t *m = s_pend[k].irq_vmask;
                for (int i = 0; i < 0x8000; i++)
                    if (sv->ppu->vram[i] != s_irq_vram[i]) m[i >> 3] |= (uint8_t)(1 << (i & 7));
                for (int i = 0; i < 0x100; i++) {
                    int b = 0x8000 + i;
                    if (sv->ppu->cgram[i] != s_irq_cgram[i]) m[b >> 3] |= (uint8_t)(1 << (b & 7));
                    b = 0x8100 + i;
                    if (sv->ppu->oam[i] != s_irq_oam[i]) m[b >> 3] |= (uint8_t)(1 << (b & 7));
                }
                for (int i = 0; i < 0x20; i++) {
                    int b = 0x8200 + i;
                    if (sv->ppu->highOam[i] != s_irq_hoam[i]) m[b >> 3] |= (uint8_t)(1 << (b & 7));
                }
            }
            s_irq_active = false;
        }
        if (!s_irq_active && c->k == 0 &&
            (c->pc == bus_read16(0x00, 0xFFEA) || c->pc == bus_read16(0x00, 0xFFEE))) {
            s_irq_active = true;
            s_irq_sp = c->sp;
            s_irq_base = s_pend_n;
            /* The CPU pushed its 4-byte interrupt frame (PB, PC, P) itself,
             * not through the write handler; it can land above a routine's
             * entry stack pointer when the interrupt comes right at its
             * return. Those bytes belong to the interrupt too. */
            for (int k = 0; k < s_pend_n; k++)
                for (int b = 1; b <= 4; b++) {
                    uint16_t w = (uint16_t)(c->sp + b);
                    s_pend[k].irq_mask[w >> 3] |= (uint8_t)(1 << (w & 7));
                }
            Snes *sv = snesrecomp_get_snes();
            memcpy(s_irq_vram, sv->ppu->vram, sizeof(s_irq_vram));
            memcpy(s_irq_cgram, sv->ppu->cgram, sizeof(s_irq_cgram));
            memcpy(s_irq_oam, sv->ppu->oam, sizeof(s_irq_oam));
            memcpy(s_irq_hoam, sv->ppu->highOam, sizeof(s_irq_hoam));
            for (int k = 0; k < s_pend_n; k++) s_pend[k].irq_hit = 1;
        }
    } else {
        s_irq_active = false;
    }

    /* A pending routine returned? Innermost first. If the stack has unwound
     * past its return frame without coming back through it (it popped its own
     * return address, as jump-table dispatchers do), there is no comparison
     * point: drop it as skipped. */
    for (int k = 0; k < s_pend_n; k++) {
        /* A native body waiting at a ROM sync point: the genuine routine has
         * got there. Compare, and let the body go on from this machine. */
        Pending *p = &s_pend[k];
        /* (By address only, inside the routine: a native body need not keep
         * the ROM's stack bytes, PHP/PHB, at the same depth.) */
        if (p->phase != PH_SYNC || c->sp > p->entry_sp) continue;
        Validate *v = &s_val[p->idx];
        if (!p->sync_met) {
            /* Compare where the ROM's wait begins: the body's state then. */
            if (pc != p->spin_pc) continue;
            char buf[200], what[260];
            int n = validate_diff(p, buf, sizeof(buf));
            if (n && irq_read_overlap(p)) { v->irqdep++; p->phase = PH_DONE; continue; }
            if (n) {
                snprintf(what, sizeof(what), "at the wait at $%06X: %s", pc, buf);
                v->calls++; record_fail(p, what, n); p->phase = PH_DONE; continue;
            }
            p->sync_met = true;
            reset_piece(p);
            continue;
        }
        /* The ROM's wait is over: the body goes on from the live machine. */
        if (pc != p->sync_pc) continue;
        reset_piece(p);
        if (!run_native(c, p, true)) { v->skipped++; p->phase = PH_DONE; continue; }
        p->phase = p->co_yielded ? (p->sync ? PH_SYNC : PH_SEG) : PH_FINAL;
    }
    if (s_fd_n && s_pend_n > 0) validate_frames(c, pc);
    while (s_pend_n > 0) {
        Pending *p = &s_pend[s_pend_n - 1];
        bool back = c->k == p->k && c->pc == p->pc && c->sp == p->sp;
        if (!back && c->sp <= p->sp) break;
        if (p->phase == PH_DONE) {
            /* already counted */
        } else if (!back || p->phase == PH_FRAME) {
            if (getenv("SNESRECOMP_VALIDATE_WHY"))
                fprintf(stderr, "WHY $%06X skipped: %s\n", s_val[p->idx].addr,
                        !back ? "the stack unwound past its return" : "returned inside a frame driver");
            s_val[p->idx].skipped++;
        } else if (p->phase == PH_SEG || p->phase == PH_SYNC) {
            s_val[p->idx].calls++;
            record_fail(p, p->phase == PH_SEG ? "the native body waits for a frame, the ROM routine returned"
                                              : "the native body waits at a sync point, the ROM routine returned", 1);
        } else {
            validate_compare(c, p);
        }
        s_pend_n--;
        if (s_pend_n == 0 && c->write == genuine_write) c->write = s_genuine_write;
    }

    int idx = -1;
    for (int i = 0; i < s_val_n; i++)
        if (s_val[i].addr == pc) { idx = i; break; }
    if (idx < 0 || s_pend_n >= VAL_DEPTH) return false;
    /* A loop that branches back to its own entry (BRA to the routine's first
     * instruction) passes the entry again at the same stack depth: that is
     * not a call, the pending check of the routine covers it. */
    for (int k = 0; k < s_pend_n; k++)
        if (s_pend[k].idx == idx && s_pend[k].entry_sp == c->sp) return false;
    /* Each check copies the whole machine twice, so a routine called 100k
     * times would take minutes. SNESRECOMP_VALIDATE_MAX (default 300) caps the
     * checked calls per function; later calls just run genuine. */
    static long s_max = -1;
    if (s_max < 0) { const char *e = getenv("SNESRECOMP_VALIDATE_MAX"); s_max = e ? atol(e) : 300; }
    if ((long)(s_val[idx].calls + s_val[idx].skipped) >= s_max) return false;
    snes_func_t fn = func_table_lookup(pc);
    if (!fn) return false;

    Snes *sn = snesrecomp_get_snes();
    Validate *v = &s_val[idx];
    Pending *p = &s_pend[s_pend_n];
    if (!s_state_buf) s_state_buf = (uint8_t *)malloc(1 << 20);
    if (!p->ram) {
        p->ram = (uint8_t *)malloc(0x20000);
        p->vram = (uint16_t *)malloc(0x10000);
        p->cgram = (uint16_t *)malloc(0x200);
        p->oam = (uint16_t *)malloc(0x200);
        p->hoam = (uint8_t *)malloc(0x20);
        p->irq_mask = (uint8_t *)malloc(0x20000 / 8);
        p->irq_vmask = (uint8_t *)malloc((IRQ_V_BITS + 7) / 8);
        p->read_mask = (uint8_t *)malloc(0x20000 / 8);
    }
    memset(p->read_mask, 0, 0x20000 / 8);
    p->irq_hit = 0;
    memset(p->irq_mask, 0, 0x20000 / 8);
    memset(p->irq_vmask, 0, (IRQ_V_BITS + 7) / 8);

    /* Where the genuine routine will come back to. */
    uint16_t sp = c->sp;
    uint16_t ret = (uint16_t)(bus_read8(0x00, (sp + 1) & 0xFFFF) | (bus_read8(0x00, (sp + 2) & 0xFFFF) << 8));
    p->idx = idx;
    p->entry_sp = sp;
    p->pc = (uint16_t)(ret + 1);
    p->k  = v->is_long ? bus_read8(0x00, (sp + 3) & 0xFFFF) : c->k;
    p->sp = (uint16_t)(sp + (v->is_long ? 3 : 2));

    memset(p->nio_set, 0, sizeof(p->nio_set));
    memset(p->gio_set, 0, sizeof(p->gio_set));
    p->fn = fn;
    p->frames = 0;
    p->phase = PH_FINAL;
    if (!run_native(c, p, false)) { v->skipped++; return true; }
    p->phase = p->co_yielded ? (p->sync ? PH_SYNC : PH_SEG) : PH_FINAL;
    s_pend_n++;
    if (c->write != genuine_write) { s_genuine_write = c->write; c->write = genuine_write; }
    return true;
}

/* ---- Frame-driving native bodies in the timed loop -----------------------
 *
 * An intercepted body marked framed (recomp_timed_add_framed_intercept) runs
 * on a stack of its own. When it calls a registered frame driver it is
 * suspended, and the timed CPU runs the genuine driver instead, as if the
 * body had executed the JSL: its registers go to the CPU and the return frame
 * is pushed where the body's S says. The return address is the driver's own
 * entry (as a JSL there would push it); when the CPU comes back to it at that
 * stack depth the hook resumes the body, with the registers the driver
 * returned, before anything runs there. */
typedef struct {
    CoCtx co;
    snes_func_t fn;
    const TimedIntercept *e;
    bool yielded;
    uint32_t fd_addr;
    bool sync;                    /* waits in a ROM loop (spin_pc) up to sync_pc */
    uint32_t spin_pc, sync_pc;
    uint16_t native_s;
    uint8_t land_k; uint16_t land_pc, land_sp;
} InterceptCo;
#define ICO_DEPTH 8
static InterceptCo  s_ico[ICO_DEPTH];
static int          s_ico_n = 0;
static InterceptCo *s_ico_cur = NULL;

#ifdef _WIN32
static void WINAPI ico_entry(LPVOID arg) {
    InterceptCo *t = (InterceptCo *)arg;
    t->fn();
    t->yielded = false;
    SwitchToFiber(t->co.ret);
}
#else
static void ico_entry(void) {
    InterceptCo *t = s_ico_cur;
    t->fn();
    t->yielded = false;
    swapcontext(&t->co.uc, &t->co.ret);
}
#endif

static bool intercept_frame_call(uint32_t snes_addr) {
    InterceptCo *t = s_ico_cur;
    if (!t || !frame_driver(snes_addr)) return false;
    t->yielded = true;
    t->sync = false;
    t->fd_addr = snes_addr;
    t->native_s = g_cpu.S;
#ifdef _WIN32
    SwitchToFiber(t->co.ret);
#else
    swapcontext(&t->co.uc, &t->co.ret);
#endif
    return true;
}

static void ico_run(InterceptCo *t) {
    InterceptCo *outer = s_ico_cur;
    s_ico_cur = t;
    t->yielded = false;
#ifdef _WIN32
    t->co.ret = GetCurrentFiber();
    SwitchToFiber(t->co.fiber);
#else
    swapcontext(&t->co.ret, &t->co.uc);
#endif
    s_ico_cur = outer;
}

static void intercept_finish(Cpu *c, const TimedIntercept *e);

/* After a framed body ran (started or resumed): send the CPU into the frame
 * driver it waits for, or finish the call. */
static void ico_after(Cpu *c, InterceptCo *t) {
    sync_to_lake(c);
    if (!t->yielded) {
        s_ico_n--;
        intercept_finish(c, t->e);
        return;
    }
    if (t->sync) {
        /* Run the ROM's own wait loop; come back where it falls through. */
        c->sp = t->native_s;
        c->k = (uint8_t)(t->spin_pc >> 16);
        c->pc = (uint16_t)t->spin_pc;
        t->land_k = (uint8_t)(t->sync_pc >> 16);
        t->land_pc = (uint16_t)t->sync_pc;
        t->land_sp = t->native_s;
        return;
    }
    const FrameDriver *fd = frame_driver(t->fd_addr);
    uint16_t sp = t->native_s;
    uint16_t ret = (uint16_t)(t->fd_addr - 1);
    if (fd->is_long) {
        bus_write8(0x00, (uint16_t)(sp + 3), (uint8_t)(t->fd_addr >> 16));
        bus_write8(0x00, (uint16_t)(sp + 2), (uint8_t)(ret >> 8));
        bus_write8(0x00, (uint16_t)(sp + 1), (uint8_t)ret);
    } else {
        bus_write8(0x00, (uint16_t)(sp + 2), (uint8_t)(ret >> 8));
        bus_write8(0x00, (uint16_t)(sp + 1), (uint8_t)ret);
    }
    t->land_k = (uint8_t)(t->fd_addr >> 16);
    t->land_pc = (uint16_t)t->fd_addr;
    t->land_sp = (uint16_t)(sp + (fd->is_long ? 3 : 2));
    c->sp = sp;
    c->k = (uint8_t)(t->fd_addr >> 16);
    c->pc = (uint16_t)t->fd_addr;
}

/* A native body at a loop the ROM leaves only when an interrupt has run
 * (LDA busy : BNE -). Under check the body waits there until the genuine
 * routine reaches sync_pc (the instruction after its loop) at the same stack,
 * and is compared there; in the timed loop the CPU runs the ROM's loop at
 * spin_pc and the body goes on when it falls through to sync_pc (by address
 * inside the routine under check; in the timed loop at the body's stack, the
 * ROM loop running on it). False
 * when neither runs the body: the caller then waits some other way. */
bool recomp_rom_wait(uint32_t spin_pc, uint32_t sync_pc) {
    Pending *p = s_co_cur;
    if (p) {
        p->co_yielded = true;
        p->sync = true;
        /* Waiting from the routine's own first instruction: nothing to
         * compare before it, the ROM runs the whole routine. */
        p->sync_met = spin_pc == s_val[p->idx].addr;
        p->spin_pc = spin_pc;
        p->sync_pc = sync_pc;
        p->native_s = g_cpu.S;
#ifdef _WIN32
        SwitchToFiber(p->co.ret);
#else
        swapcontext(&p->co.uc, &p->co.ret);
#endif
        return true;
    }
    InterceptCo *t = s_ico_cur;
    if (!t) return false;
    t->yielded = true;
    t->sync = true;
    t->spin_pc = spin_pc;
    t->sync_pc = sync_pc;
    t->native_s = g_cpu.S;
#ifdef _WIN32
    SwitchToFiber(t->co.ret);
#else
    swapcontext(&t->co.uc, &t->co.ret);
#endif
    return true;
}

/* A suspended body whose frame driver just returned? Resume it. */
/* Can the native body that runs now wait for a frame (a validation
 * coroutine or a framed intercept)? Otherwise a frame-driver call would
 * present the frame from C; such a body should hand over to the ROM. */
bool recomp_frame_wait_possible(void) { return s_co_cur != NULL || s_ico_cur != NULL; }

/* May a native body call the C version of snes_addr? Under lockstep
 * validation yes (that is how callers check their callees' paths); in the
 * timed loop only if it is an intercept, i.e. verified on its own. */
bool recomp_timed_trusted(uint32_t snes_addr) {
    if (s_val_n || s_intercept_count == 0) return true;   /* validating, or no timed loop */
    for (int i = 0; i < s_intercept_count; i++)
        if (s_intercepts[i].addr == snes_addr) return true;
    return false;
}

static bool ico_resume(Cpu *c) {
    for (int i = s_ico_n - 1; i >= 0; i--) {
        InterceptCo *t = &s_ico[i];
        if (!t->yielded || c->k != t->land_k || c->pc != t->land_pc || c->sp != t->land_sp) continue;
        if (i != s_ico_n - 1) {
            fprintf(stderr, "recomp: framed body $%06X resumed out of order\n", t->e->addr);
            abort();
        }
        sync_from_lake(c);
        g_cpu.S = t->native_s;
        s_timed_depth++;
        ico_run(t);
        s_timed_depth--;
        ico_after(c, t);
        return true;
    }
    return false;
}

static bool timed_recomp_hook(Cpu *c) {
    if (s_profile) {
        uint8_t op = bus_read8(c->k, c->pc);
        if (op == 0x20) {  /* JSR abs (near) */
            uint32_t t = ((uint32_t)c->k << 16)
                       | bus_read8(c->k, c->pc + 1)
                       | ((uint32_t)bus_read8(c->k, c->pc + 2) << 8);
            tally_call(t, c->mf, c->xf, 0);
        } else if (op == 0x22) {  /* JSL long */
            uint32_t t = bus_read8(c->k, c->pc + 1)
                       | ((uint32_t)bus_read8(c->k, c->pc + 2) << 8)
                       | ((uint32_t)bus_read8(c->k, c->pc + 3) << 16);
            tally_call(t, c->mf, c->xf, 1);
        }
    }

    if (s_ico_n && ico_resume(c)) return true;
    if (s_val_n && validate_hook(c)) return false;   /* genuine routine runs */

    uint32_t pc = ((uint32_t)c->k << 16) | (uint16_t)c->pc;
    const TimedIntercept *e = NULL;
    for (int i = 0; i < s_intercept_count; i++)
        if (s_intercepts[i].addr == pc) { e = &s_intercepts[i]; break; }
    if (!e) return false;
    snes_func_t fn = func_table_lookup(pc);
    if (!fn) return false;

    if (getenv("SMK_RECOMP_DEBUG")) {
        Snes *sn = snesrecomp_get_snes();
        uint16_t sp = c->sp;
        uint16_t ret = bus_read8(0x00, (sp + 1) & 0xFFFF) | (bus_read8(0x00, (sp + 2) & 0xFFFF) << 8);
        fprintf(stderr, "[RECOMP] hit $%06X frame=%u db=%02X sp=%04X ret=%04X\n",
                pc, sn ? sn->frames : 0, c->db, sp, ret);
    }

    sync_from_lake(c);   /* LakeSnes CPU regs -> g_cpu (native body reads these) */
    s_redirect = 0xFFFFFFFFu;
    if (e->framed && s_fd_n && s_ico_n < ICO_DEPTH) {
        InterceptCo *t = &s_ico[s_ico_n++];
        t->fn = fn;
        t->e = e;
#ifdef _WIN32
        if (!s_main_fiber) s_main_fiber = ConvertThreadToFiber(NULL);
        if (t->co.fiber) DeleteFiber(t->co.fiber);
        t->co.fiber = CreateFiber(CO_STACK, ico_entry, t);
#else
        if (!t->co.stack) t->co.stack = (char *)malloc(CO_STACK);
        getcontext(&t->co.uc);
        t->co.uc.uc_stack.ss_sp = t->co.stack;
        t->co.uc.uc_stack.ss_size = CO_STACK;
        t->co.uc.uc_link = NULL;
        makecontext(&t->co.uc, ico_entry, 0);
#endif
        s_timed_depth++;
        ico_run(t);
        s_timed_depth--;
        ico_after(c, t);
        return true;
    }
    s_timed_depth++;
    fn();                /* run recompiled native body (g_cpu + recomp bus) */
    s_timed_depth--;
    sync_to_lake(c);     /* g_cpu -> LakeSnes CPU regs */
    intercept_finish(c, e);
    return true;
}

/* The end of an intercepted call: a redirect, or the simulated RTS/RTL. */
static void intercept_finish(Cpu *c, const TimedIntercept *e) {

    /* JMP/JML exit: the function redirected instead of returning. Set PB:PC to
     * the target without popping the stack (matches the original jump). */
    if (s_redirect != 0xFFFFFFFFu) {
        c->pc = (uint16_t)s_redirect;
        c->k  = (uint8_t)(s_redirect >> 16);
        s_redirect = 0xFFFFFFFFu;
        s_intercept_hits++;
        return;
    }

    /* Optional cycle accounting: advance the timed clock by the original
     * routine's non-DMA instruction cycles, which running the native body
     * instantly skips. SMK_RECOMP_CYCLES=N (diagnostic/tuning while the
     * per-function model is calibrated). */
    {
        static long extra = -2;
        if (extra == -2) { const char *e = getenv("SMK_RECOMP_CYCLES"); extra = e ? atol(e) : 0; }
        if (extra > 0) { Snes *sn = snesrecomp_get_snes(); if (sn) snes_runCycles(sn, (int)extra); }
    }

    /* Simulate the routine's terminating RTS (near) / RTL (long), using the
     * return frame the calling JSR/JSL pushed. Stack lives in bank $00. */
    uint16_t sp = c->sp;
    uint16_t lo = bus_read8(0x00, (sp + 1) & 0xFFFF);
    uint16_t hi = bus_read8(0x00, (sp + 2) & 0xFFFF);
    c->pc = (uint16_t)((lo | (hi << 8)) + 1);
    if (e->is_long) {
        c->k  = (uint8_t)bus_read8(0x00, (sp + 3) & 0xFFFF);
        c->sp = (uint16_t)((sp + 3) & 0xFFFF);
    } else {
        c->sp = (uint16_t)((sp + 2) & 0xFFFF);
    }
    s_intercept_hits++;
}

void recomp_timed_recomp_enable(void)  { g_cpuRecompHook = timed_recomp_hook; }
void recomp_timed_recomp_disable(void) { g_cpuRecompHook = NULL; }
