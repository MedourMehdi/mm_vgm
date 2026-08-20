/*
 * mm_vgm.c — Atari ST/FreeMiNT SN76489 → YM2149 VGM/VGZ player
 * Medour Mehdi - 2026
 * Architecture: Motorola 68000 / Atari ST-TT-Falcon
 * 
 * STF-focused professional release:
 *   • Pure m68k assembly parser — no C tick at all.
 *   • BATCHED WINDOW timing with exact 44.1 kHz fractional scheduling.
 *   • ISR saves/restores only d3-d6/a0-a3 (8 regs).
 *   • SN noise → YM noise + simultaneous tone/noise mix on channel C.
 *   • Channel-C volume precedence: noise active → noise volume; noise silent
 *     → restore tone-2 volume.  Tone C is never killed in the mixer.
 *   • PERIODIC NOISE (FB=0, NF=3, tone-2 silent) → pure YM tone C buzz.
 *   • WHITE NOISE NF=2 (tone-2 silent) → YM noise period 31 + 250 Hz body
 *     tone on channel C.  This adds the missing low-end thump that the YM
 *     noise generator cannot produce (YM floor ≈ 4 kHz vs SN NF=2 ≈ 1.7 kHz).
 *   • Loop support, FM-only detection, header-derived SN clock.
 *   • Unknown VGM commands safely skipped.
 *   • GEM/AES windowed UI with proper wind_update / graf_mouse locking,
 *     vertical scrollbar support, window resizing with minimum size guard,
 *     strict list clipping between header and sticky status, and correct
 *     VDI text baseline handling.
 *   • Incremental redraw — only changed rows / status / list area repainted.
 *   • M3U playlist support (CLI load only; no directory scan for .m3u).
 *
 * Compile:
 *   m68k-atari-mint-gcc -O2 -m68000 mm_vgm.c -o playvgm.ttp -lz
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <zlib.h>
#include <mint/osbind.h>

#include <gem.h>   /* AES/GEM bindings: appl_init, wind_update, form_dial... */

#define YM_SEL  *(volatile uint8_t *)0xFFFF8800
#define YM_DAT  *(volatile uint8_t *)0xFFFF8802

#define MFP_TACR  (*(volatile uint8_t *)0xFFFFFA19)
#define MFP_TADR  (*(volatile uint8_t *)0xFFFFFA1F)
#define MFP_IERA  (*(volatile uint8_t *)0xFFFFFA07)
#define MFP_IMRA  (*(volatile uint8_t *)0xFFFFFA13)
#define MFP_IPRA  (*(volatile uint8_t *)0xFFFFFA0B)
#define TIMER_A_VEC  (*(volatile uint32_t *)0x0134)

#define TACR_DIV10        2
#define TADR_VALUE        251
#define TIMER_BASE_SAMPLES 45

#define CONTERM (*(volatile uint8_t *)0x484)
static uint8_t old_conterm;

typedef struct {
    uint16_t sn_tone[4];    /*   0  */
    uint8_t  sn_vol[4];     /*   8  */
    uint8_t  latched;       /*  12  */
    uint8_t  mixer;         /*  13  */
    uint8_t  noise_ym[4];   /*  14  */
    uint8_t  lut_vol[16];   /*  18  */
    uint8_t  skip_len[256]; /*  34  */
} parser_state_t;

parser_state_t  parser_state;
uint16_t        lut_tone[1024];

volatile uint8_t  *vgm_ptr;
volatile uint8_t  *vgm_loop_ptr;
volatile uint32_t  wait_samples = 0;
volatile uint16_t  timer_frac = 0;
volatile uint16_t  timer_window = TIMER_BASE_SAMPLES;
volatile uint8_t   vgm_playing  = 0;
volatile uint8_t  *vgm_end_ptr = 0;
uint32_t           sn_clock_hz = 0;

uint32_t old_timer_a_vec;
uint8_t  old_tacr, old_tadr, old_iera, old_imra;

static uint32_t vgm_data_offset = 0;   /* remembered for seamless loop */
int loop_mode = 0;
int autoplay_mode = 0;

static int aes_apid = -1;   /* -1 = no AES, >= 0 = AES present */

/* ── GEM / VDI globals (windowed mode) ─────────────────────────────────── */
static short phys_handle;
static short vdi_handle;
static short wi_handle;
static short xdesk, ydesk, wdesk, hdesk;   /* desktop work area */
static short xwork, ywork, wwork, hwork;     /* window work area */
static short work_in[11];
static short work_out[57];
static short gem_font_w, gem_font_h, gem_font_wbox, gem_font_hbox;

static int gem_selected = 0;
static int gem_playing = -1;
static char gem_status[64] = "Ready.";

static uint8_t gem_old_conterm = 0;

/* ── Scrollbar / layout state ──────────────────────────────────────────── */
static int scroll_y = 0;          /* pixel offset into virtual document */
static int doc_h = 0;               /* total virtual height of file list */
static int line_height = 0;         /* gem_font_hbox, cached */
static int header_lines = 5;        /* title + loop/auto + keys1 + keys2 + separator */
static int header_h = 0;            /* header_lines * line_height */
static int16_t gem_min_w = 300;     /* minimum outer width  (set at open) */
static int16_t gem_min_h = 200;     /* minimum outer height (set at open) */

/* ── Forward declarations ──────────────────────────────────────────────── */
void stop_vgm(void);
int  start_vgm(const char *filename);

__asm__(
    "   .text\n"
    "   .global _vgm_parser_tick\n"
    "_vgm_parser_tick:\n"
    "   tst.b   _vgm_playing\n"
    "   beq     .Lpt_ret\n"
    "   movea.l _vgm_ptr, a0\n"
    "   cmpa.l  _vgm_end_ptr, a0\n"
    "   bhs     .Lpt_stop\n"
    "   movea.l #0xFFFF8800, a1\n"
    "   lea     _parser_state, a2\n"
    "   lea     _lut_tone, a3\n"
    "   move.w  _timer_window, d6\n"

    ".Lpt_window:\n"
    "   tst.b   _vgm_playing\n"
    "   beq     .Lpt_exit\n"
    "   move.l  _wait_samples, d5\n"
    "   cmp.l   d6, d5\n"
    "   blo     .Lpt_consume\n"
    "   sub.l   d6, d5\n"
    "   move.l  d5, _wait_samples\n"
    "   bra     .Lpt_exit\n"
    ".Lpt_consume:\n"
    "   sub.l   d5, d6\n"
    "   clr.l   _wait_samples\n"

    ".Lpt_loop:\n"
    "   cmpa.l  _vgm_end_ptr, a0\n"
    "   bhs     .Lpt_stop\n"
    "   moveq   #0, d4\n"
    "   move.b  (a0)+, d4\n"
    "   cmp.b   #0x50, d4\n"
    "   beq     .Lpt_sn\n"
    "   cmp.b   #0x61, d4\n"
    "   beq     .Lpt_waitn\n"
    "   cmp.b   #0x62, d4\n"
    "   beq     .Lpt_w735\n"
    "   cmp.b   #0x63, d4\n"
    "   beq     .Lpt_w882\n"
    "   cmp.b   #0x66, d4\n"
    "   beq     .Lpt_end\n"
    "   cmp.b   #0x67, d4\n"
    "   beq     .Lpt_dblock\n"
    "   cmp.b   #0xA0, d4\n"
    "   beq     .Lpt_ay\n"
    "   move.b  d4, d5\n"
    "   and.b   #0xF0, d5\n"
    "   cmp.b   #0x70, d5\n"
    "   beq     .Lpt_waitshort\n"
    "   cmp.b   #0x80, d5\n"
    "   beq     .Lpt_ymwait\n"
    "   moveq   #0, d5\n"
    "   move.b  d4, d5\n"
    "   lea     34(a2), a3\n"
    "   move.b  0(a3,d5.w), d5\n"
    "   beq.s   .Lpt_skip_1\n"
    "   adda.l  d5, a0\n"
    "   lea     _lut_tone, a3\n"
    "   bra     .Lpt_loop\n"
    ".Lpt_skip_1:\n"
    "   addq.l  #1, a0\n"
    "   lea     _lut_tone, a3\n"
    "   bra     .Lpt_loop\n"

    /* 0x50 SN76489 write */
    ".Lpt_sn:\n"
    "   move.b  (a0)+, d4\n"
    "   btst    #7, d4\n"
    "   beq     .Lpt_sn_data\n"
    "   move.b  d4, d5\n"
    "   and.b   #0x70, d5\n"
    "   move.b  d5, 12(a2)\n"
    "   bra     .Lpt_sn_decode\n"
    ".Lpt_sn_data:\n"
    "   move.b  12(a2), d5\n"
    ".Lpt_sn_decode:\n"
    "   moveq   #0, d3\n"
    "   move.b  d5, d3\n"
    "   lsr.b   #5, d3\n"
    "   and.w   #3, d3\n"
    "   btst    #4, d5\n"
    "   bne     .Lpt_vol\n"
    "   add.w   d3, d3\n"
    "   cmpi.w  #6, d3\n"
    "   beq     .Lpt_noise_update\n"
    "   move.w  0(a2,d3.w), d5\n"
    "   btst    #7, d4\n"
    "   beq     .Lpt_tone_hi\n"
    "   and.w   #0x3F0, d5\n"
    "   and.b   #0x0F, d4\n"
    "   or.w    d4, d5\n"
    "   bra     .Lpt_tone_st\n"
    ".Lpt_tone_hi:\n"
    "   and.w   #0x00F, d5\n"
    "   and.b   #0x3F, d4\n"
    "   lsl.w   #4, d4\n"
    "   or.w    d4, d5\n"
    ".Lpt_tone_st:\n"
    "   move.w  d5, 0(a2,d3.w)\n"
    "   move.w  d5, d4\n"
    "   add.w   d4, d4\n"
    "   move.w  0(a3,d4.w), d5\n"
    "   move.b  d3, (a1)\n"
    "   move.b  d5, 2(a1)\n"
    "   addq.b  #1, d3\n"
    "   move.b  d3, (a1)\n"
    "   lsr.w   #8, d5\n"
    "   and.b   #0x0F, d5\n"
    "   move.b  d5, 2(a1)\n"
    "   bra     .Lpt_loop\n"

    ".Lpt_noise_update:\n"
    "   and.b   #0x07, d4\n"
    "   moveq   #0, d5\n"
    "   move.b  d4, d5\n"
    "   move.w  d5, 6(a2)\n"
    "   bra     .Lpt_noise\n"

    /* noise channel (chan 3) */
    ".Lpt_noise:\n"
    "   move.b  d5, d4\n"
    "   and.w   #3, d4\n"
    "   cmp.b   #3, d4\n"
    "   beq     .Lpt_noise_t2\n"
    "   btst    #2, d5\n"
    "   beq     .Lpt_noise_per\n"
    "   move.b  14(a2,d4.w), d5\n"
    "   cmpi.b  #2, d4\n"
    "   bne     .Lpt_noise_wr_and_mixer\n"
    "   move.b  10(a2), d4\n"
    "   cmpi.b  #15, d4\n"
    "   bne     .Lpt_noise_wr_and_mixer\n"
    "   move.b  #4, (a1)\n"
    "   move.b  #0xF4, 2(a1)\n"
    "   move.b  #5, (a1)\n"
    "   move.b  #0x01, 2(a1)\n"
    "   bra     .Lpt_noise_wr_and_mixer\n"

    ".Lpt_noise_per:\n"
    "   moveq   #1, d5\n"
    "   bra     .Lpt_noise_wr_and_mixer\n"

    ".Lpt_noise_t2:\n"
    "   move.b  6(a2), d4\n"
    "   btst    #2, d4\n"
    "   beq     .Lpt_noise_t2_per\n"
    "   moveq   #0, d5\n"
    "   move.w  4(a2), d5\n"
    "   add.w   d5, d5\n"
    "   move.w  0(a3,d5.w), d5\n"
    "   beq     .Lpt_noise_per\n"
    "   cmp.b   #31, d5\n"
    "   bls     .Lpt_noise_wr_and_mixer\n"
    "   moveq   #31, d5\n"
    "   bra     .Lpt_noise_wr_and_mixer\n"

    ".Lpt_noise_t2_per:\n"
    "   move.b  10(a2), d4\n"
    "   cmpi.b  #15, d4\n"
    "   bne     .Lpt_noise_per\n"
    "   move.b  #0x38, 13(a2)\n"
    "   move.b  #7, (a1)\n"
    "   move.b  #0x38, 2(a1)\n"
    "   bra     .Lpt_loop\n"

    ".Lpt_noise_wr_and_mixer:\n"
    "   move.b  #6, (a1)\n"
    "   move.b  d5, 2(a1)\n"
    "   move.b  11(a2), d4\n"
    "   cmpi.b  #15, d4\n"
    "   beq     .Lpt_loop\n"
    "   move.b  #0x18, 13(a2)\n"
    "   move.b  #7, (a1)\n"
    "   move.b  #0x18, 2(a1)\n"
    "   bra     .Lpt_loop\n"

    /* volume (SN ch0-3) */
    ".Lpt_vol:\n"
    "   move.b  d4, d5\n"
    "   and.b   #0x0F, d5\n"
    "   move.b  d5, 8(a2,d3.w)\n"
    "   cmpi.b  #2, d3\n"
    "   beq     .Lpt_vol_ch2\n"
    "   cmpi.b  #3, d3\n"
    "   beq     .Lpt_vol_ch3\n"
    "   move.b  18(a2,d5.w), d5\n"
    "   move.b  d3, d4\n"
    "   addq.b  #8, d4\n"
    "   move.b  d4, (a1)\n"
    "   move.b  d5, 2(a1)\n"
    "   bra     .Lpt_loop\n"

    ".Lpt_vol_ch2:\n"
    "   move.b  11(a2), d4\n"
    "   cmpi.b  #15, d4\n"
    "   bne     .Lpt_loop\n"
    "   move.b  18(a2,d5.w), d5\n"
    "   move.b  #10, (a1)\n"
    "   move.b  d5, 2(a1)\n"
    "   bra     .Lpt_loop\n"

    ".Lpt_vol_ch3:\n"
    "   move.b  d5, d6\n"
    "   move.b  18(a2,d5.w), d5\n"
    "   move.b  #10, (a1)\n"
    "   move.b  d5, 2(a1)\n"
    "   tst.b   d5\n"
    "   beq     .Lpt_nvol_silent\n"
    "   move.b  6(a2), d4\n"
    "   and.b   #0x07, d4\n"
    "   cmpi.b  #0x03, d4\n"
    "   bne     .Lpt_nvol_noise\n"
    "   move.b  10(a2), d4\n"
    "   cmpi.b  #15, d4\n"
    "   bne     .Lpt_nvol_noise\n"
    "   move.b  #0x38, 13(a2)\n"
    "   bra     .Lpt_mix_wr\n"
    ".Lpt_nvol_noise:\n"
    "   move.b  #0x18, 13(a2)\n"
    "   bra     .Lpt_mix_wr\n"
    ".Lpt_nvol_silent:\n"
    "   move.b  10(a2), d4\n"
    "   cmpi.b  #15, d4\n"
    "   beq     .Lpt_mix_tone\n"
    "   move.b  18(a2,d4.w), d5\n"
    "   move.b  #10, (a1)\n"
    "   move.b  d5, 2(a1)\n"
    ".Lpt_mix_tone:\n"
    "   move.b  #0x38, 13(a2)\n"
    ".Lpt_mix_wr:\n"
    "   move.b  #7, (a1)\n"
    "   move.b  13(a2), 2(a1)\n"
    "   bra     .Lpt_loop\n"

    /* waits, end, data block, AY passthrough */
    ".Lpt_waitn:\n"
    "   moveq   #0, d5\n"
    "   move.b  (a0)+, d5\n"
    "   moveq   #0, d4\n"
    "   move.b  (a0)+, d4\n"
    "   lsl.w   #8, d4\n"
    "   or.w    d4, d5\n"
    "   move.l  d5, _wait_samples\n"
    "   bra     .Lpt_window\n"
    ".Lpt_w735:\n"
    "   move.l  #735, d5\n"
    "   move.l  d5, _wait_samples\n"
    "   bra     .Lpt_window\n"
    ".Lpt_w882:\n"
    "   move.l  #882, d5\n"
    "   move.l  d5, _wait_samples\n"
    "   bra     .Lpt_window\n"
    ".Lpt_waitshort:\n"
    "   and.b   #0x0F, d4\n"
    "   addq.b  #1, d4\n"
    "   moveq   #0, d5\n"
    "   move.b  d4, d5\n"
    "   move.l  d5, _wait_samples\n"
    "   bra     .Lpt_window\n"
    ".Lpt_ymwait:\n"
    "   and.b   #0x0F, d4\n"
    "   moveq   #0, d5\n"
    "   move.b  d4, d5\n"
    "   move.l  d5, _wait_samples\n"
    "   bra     .Lpt_window\n"
    ".Lpt_end:\n"
    "   move.l  _vgm_loop_ptr, d5\n"
    "   beq.s   .Lpt_stop\n"
    "   movea.l d5, a0\n"
    "   bra     .Lpt_loop\n"
    ".Lpt_stop:\n"
    "   clr.b   _vgm_playing\n"
    "   bra     .Lpt_exit\n"
    ".Lpt_dblock:\n"
    "   addq.l  #2, a0\n"
    "   moveq   #0, d5\n"
    "   move.b  (a0)+, d5\n"
    "   moveq   #0, d4\n"
    "   move.b  (a0)+, d4\n"
    "   lsl.w   #8, d4\n"
    "   or.l    d4, d5\n"
    "   moveq   #0, d4\n"
    "   move.b  (a0)+, d4\n"
    "   swap    d4\n"
    "   or.l    d4, d5\n"
    "   moveq   #0, d4\n"
    "   move.b  (a0)+, d4\n"
    "   swap    d4\n"
    "   lsl.l   #8, d4\n"
    "   or.l    d4, d5\n"
    "   adda.l  d5, a0\n"
    "   bra     .Lpt_loop\n"
    ".Lpt_ay:\n"
    "   move.b  (a0)+, d4\n"
    "   and.b   #0x0F, d4\n"
    "   move.b  (a0)+, d5\n"
    "   move.b  d4, (a1)\n"
    "   move.b  d5, 2(a1)\n"
    "   bra     .Lpt_loop\n"
    ".Lpt_exit:\n"
    "   move.l  a0, _vgm_ptr\n"
    ".Lpt_ret:\n"
    "   rts\n"
);


__asm__(
    "   .text\n"
    "   .global timer_a_isr\n"
    "timer_a_isr:\n"
    "   movem.l d3-d6/a0-a3, -(sp)\n"
    "   moveq   #45, d6\n"
    "   move.w  _timer_frac, d5\n"
    "   addi.w  #165, d5\n"
    "   cmpi.w  #4096, d5\n"
    "   blo.s   .Lta_frac_ok\n"
    "   subi.w  #4096, d5\n"
    "   addq.w  #1, d6\n"
    ".Lta_frac_ok:\n"
    "   move.w  d5, _timer_frac\n"
    "   move.w  d6, _timer_window\n"
    "   jsr     _vgm_parser_tick\n"
    "   bclr    #5, 0xFFFFFA0F.w\n"
    "   movem.l (sp)+, d3-d6/a0-a3\n"
    "   rte\n"
);
extern void timer_a_isr(void) __asm__("timer_a_isr");

/* ══════════════════════════════════════════════════════════════════════════
 * C SUPPORT CODE (Playback Management)
 * ══════════════════════════════════════════════════════════════════════════
 */

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void init_tables(void)
{
    memset(&parser_state, 0, sizeof(parser_state));
    parser_state.mixer = 0x38;

    for (int i = 0; i < 1024; i++) {
        uint32_t n = (i == 0) ? 1UL : (uint32_t)i;
        uint32_t m = (n * 4000000UL) / sn_clock_hz;
        if (m < 1UL)    m = 1UL;
        if (m > 4095UL) m = 4095UL;
        lut_tone[i] = (uint16_t)m;
    }

    static const uint8_t vol_map[16] =
        { 15,14,14,13,12,11,10,10, 9, 8, 7, 6, 6, 5, 4, 0 };
    memcpy(parser_state.lut_vol, vol_map, 16);

    uint32_t r;
    r = (32UL * 2000000UL) / sn_clock_hz;
    if (r < 1)  r = 1;  if (r > 31) r = 31;
    parser_state.noise_ym[0] = (uint8_t)r;

    r = (64UL * 2000000UL) / sn_clock_hz;
    if (r < 1)  r = 1;  if (r > 31) r = 31;
    parser_state.noise_ym[1] = (uint8_t)r;

    r = (128UL * 2000000UL) / sn_clock_hz;
    if (r < 1)  r = 1;  if (r > 31) r = 31;
    parser_state.noise_ym[2] = (uint8_t)r;
    parser_state.noise_ym[3] = 0;

    uint8_t *s = parser_state.skip_len;
    s[0x30] = 1;    s[0x3F] = 1;    s[0x4F] = 1;    s[0x50] = 1;
    for (int i = 0x51; i <= 0x5F; i++) s[i] = 2;
    s[0x61] = 2;    s[0x68] = 11;
    s[0x90] = 4;    s[0x91] = 4;    s[0x92] = 5;
    s[0x93] = 10;   s[0x94] = 1;    s[0x95] = 4;
    for (int i = 0xA0; i <= 0xBF; i++) s[i] = 2;
    for (int i = 0xC0; i <= 0xCF; i++) s[i] = 2;
    for (int i = 0xD0; i <= 0xDF; i++) s[i] = 3;
    for (int i = 0xE0; i <= 0xFF; i++) s[i] = 4;
}

static uint32_t get_uncompressed_size(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    if (!f) return 0;
    uint8_t magic[2];
    fread(magic, 1, 2, f);
    uint32_t size = 0;
    if (magic[0] == 0x1F && magic[1] == 0x8B) {
        fseek(f, -4, SEEK_END);
        uint8_t footer[4];
        fread(footer, 1, 4, f);
        size = (uint32_t)footer[0] | ((uint32_t)footer[1] << 8)
             | ((uint32_t)footer[2] << 16) | ((uint32_t)footer[3] << 24);
    } else {
        fseek(f, 0, SEEK_END);
        size = (uint32_t)ftell(f);
    }
    fclose(f);
    return size;
}

static void init_hardware(void)
{
    void *ssp = (void *)Super(0);

    old_timer_a_vec = TIMER_A_VEC;
    old_tacr = MFP_TACR;
    old_tadr = MFP_TADR;
    old_iera = MFP_IERA;
    old_imra = MFP_IMRA;

    YM_SEL = 7;  YM_DAT = 0x38;
    YM_SEL = 8;  YM_DAT = 0;
    YM_SEL = 9;  YM_DAT = 0;
    YM_SEL = 10; YM_DAT = 0;

    MFP_TACR = 0;
    TIMER_A_VEC = (uint32_t)timer_a_isr;
    MFP_IPRA &= ~0x20;
    MFP_TADR = TADR_VALUE;
    MFP_TACR = TACR_DIV10;
    MFP_IERA |= 0x20;
    MFP_IMRA |= 0x20;

    Super(ssp);
}

static void restore_hardware(void)
{
    void *ssp = (void *)Super(0);

    MFP_TACR = 0;
    TIMER_A_VEC = old_timer_a_vec;
    MFP_IERA = old_iera;
    MFP_IMRA = old_imra;
    MFP_TACR = old_tacr;
    MFP_TADR = old_tadr;

    YM_SEL = 8;  YM_DAT = 0;
    YM_SEL = 9;  YM_DAT = 0;
    YM_SEL = 10; YM_DAT = 0;
    YM_SEL = 7;  YM_DAT = 0x38;

    Super(ssp);
}

/* ── Interactive UI Globals ──────────────────────────────────────────────── */

#define MAX_FILES 256
char file_list[MAX_FILES][64];
int num_files = 0;
uint8_t *vgm_buffer = NULL;

void scan_files(void)
{
    char dta[44];
    Fsetdta(dta);
    num_files = 0;

    if (Fsfirst("*.vgm", 0) == 0) {
        do {
            snprintf(file_list[num_files], 64, "%s", &dta[30]);
            num_files++;
        } while (Fsnext() == 0 && num_files < MAX_FILES);
    }
    if (Fsfirst("*.vgz", 0) == 0 && num_files < MAX_FILES) {
        do {
            snprintf(file_list[num_files], 64, "%s", &dta[30]);
            num_files++;
        } while (Fsnext() == 0 && num_files < MAX_FILES);
    }
}

/* Load M3U playlist from CLI.  Lines starting with # are ignored. */
static int load_m3u(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    if (!f) return -1;

    num_files = 0;
    char line[128];
    while (fgets(line, sizeof(line), f) && num_files < MAX_FILES) {
        int len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
            line[--len] = '\0';

        if (len == 0 || line[0] == '#') continue;

        char *src = line;
        if ((unsigned char)src[0] == 0xEF && (unsigned char)src[1] == 0xBB && (unsigned char)src[2] == 0xBF)
            src += 3;

        while (*src == ' ' || *src == '\t') src++;
        char *end = src + strlen(src) - 1;
        while (end > src && (*end == ' ' || *end == '\t')) *end-- = '\0';

        if (*src == '\0') continue;

        snprintf(file_list[num_files], 64, "%s", src);
        num_files++;
    }
    fclose(f);
    return (num_files > 0) ? 0 : -1;
}


/* ══════════════════════════════════════════════════════════════════════════
 * GEM VDI / AES WINDOW INTERFACE
 * ══════════════════════════════════════════════════════════════════════════
 */

static int gem_update_scrollbars(void)
{
    int view_h = hwork - header_h - line_height;
    int v_size, v_pos;
    int changed = 0;
    static int last_v_size = -1, last_v_pos = -1;

    if (doc_h > view_h && view_h > 0) {
        v_size = (view_h * 1000) / doc_h;
        if (v_size < 20) v_size = 20;
        int old_scroll = scroll_y;
        if (scroll_y < 0) scroll_y = 0;
        if (scroll_y > doc_h - view_h) scroll_y = doc_h - view_h;
        if (scroll_y != old_scroll) changed = 1;
        v_pos = (scroll_y * 1000) / (doc_h - view_h);
    } else {
        v_size = 1000;
        v_pos = 0;
        if (scroll_y != 0) {
            scroll_y = 0;
            changed = 1;
        }
    }

    if (v_size != last_v_size) {
        wind_set(wi_handle, WF_VSLSIZE, v_size, 0, 0, 0);
        last_v_size = v_size;
        changed = 1;
    }
    if (v_pos != last_v_pos) {
        wind_set(wi_handle, WF_VSLIDE,  v_pos,  0, 0, 0);
        last_v_pos = v_pos;
        changed = 1;
    }
    return changed;
}

static void gem_open_vwork(void)
{
    int16_t i;
    for (i = 0; i < 10; i++) work_in[i] = 1;
    work_in[10] = 2;
    vdi_handle = phys_handle;
    v_opnvwk(work_in, &vdi_handle, work_out);

    vswr_mode(vdi_handle, MD_TRANS);
    vsf_interior(vdi_handle, 1);
    vsf_perimeter(vdi_handle, 0);
}

static void gem_open_window(void)
{
    int16_t wi_kind = NAME | CLOSER | MOVER | SIZER | UPARROW | DNARROW | VSLIDE;
    int16_t win_w = 440;
    int16_t win_h = 420;
    int16_t win_x = xdesk + (wdesk - win_w) / 2;
    int16_t win_y = ydesk + 20;
    int16_t dummy_x, dummy_y;

    wi_handle = wind_create(wi_kind, xdesk, ydesk, wdesk, hdesk);
    wind_set_str(wi_handle, WF_NAME, " mm_vgm ");

    wind_open(wi_handle, win_x, win_y, win_w, win_h);

    wind_get(wi_handle, WF_WORKXYWH, &xwork, &ywork, &wwork, &hwork);

    line_height = gem_font_hbox;
    header_h    = header_lines * line_height;
    doc_h       = num_files * line_height;

    {
        int16_t min_work_w = 300;
        int16_t min_work_h = header_h + line_height + line_height;
        wind_calc(WC_BORDER, wi_kind, 0, 0, min_work_w, min_work_h,
                  &dummy_x, &dummy_y, &gem_min_w, &gem_min_h);
    }

    gem_update_scrollbars();
}

static void gem_draw_ui_rect(int16_t rx, int16_t ry, int16_t rw, int16_t rh)
{
    int16_t clip_pxy[4];
    int16_t ty;
    char buf[80];
    int16_t wx = xwork, wy = ywork, ww = wwork, wh = hwork;
    int16_t list_top    = wy + header_h;
    int16_t list_bottom = wy + wh - line_height;
    int16_t list_view_h = list_bottom - list_top;
    int16_t i;

    clip_pxy[0] = rx;  clip_pxy[1] = ry;
    clip_pxy[2] = rx + rw - 1;
    clip_pxy[3] = ry + rh - 1;
    vs_clip(vdi_handle, 1, clip_pxy);

    vsf_color(vdi_handle, 0);
    {
        int16_t bg[4] = { rx, ry, rx + rw - 1, ry + rh - 1 };
        v_bar(vdi_handle, bg);
    }

    ty = wy + line_height;
    if (ty >= ry && ty <= ry + rh) {
        vst_color(vdi_handle, 1);
        v_gtext(vdi_handle, wx + 8, ty, (char *)"--- mm_vgm v0.2 GEM Player ---");
    }

    ty += line_height;
    if (ty >= ry && ty <= ry + rh) {
        sprintf(buf, "Loop:%s  Auto:%s",
                loop_mode ? "ON" : "OFF", autoplay_mode ? "ON" : "OFF");
        v_gtext(vdi_handle, wx + 8, ty, buf);
    }

    ty += line_height;
    if (ty >= ry && ty <= ry + rh) {
        v_gtext(vdi_handle, wx + 8, ty,
                (char *)"UP/DOWN=Select  ENTER=Play");
    }

    ty += line_height;
    if (ty >= ry && ty <= ry + rh) {
        v_gtext(vdi_handle, wx + 8, ty,
                (char *)"SPACE=Stop      Q=Quit");
    }

    if (num_files == 0) {
        if (list_view_h > 0) {
            int16_t msg_y = list_top + line_height;
            if (msg_y >= ry && msg_y <= ry + rh)
                v_gtext(vdi_handle, wx + 8, msg_y,
                        (char *)"No .vgm or .vgz files found.");
        }
    } else if (list_view_h > 0) {
        for (i = 0; i < num_files; i++) {
            int16_t row_virt_y = i * line_height;
            int16_t row_scr_y  = list_top + row_virt_y - scroll_y;

            if (row_scr_y + 2 < list_top) continue;
            if (row_scr_y - line_height > list_bottom) break;
            if (row_scr_y - line_height > ry + rh) break;
            if (row_scr_y < ry) continue;

            if (i == gem_selected) {
                vsf_color(vdi_handle, 1);
                {
                    int16_t bar[4] = { wx + 4,
                                       row_scr_y - line_height + 2,
                                       wx + ww - 4,
                                       row_scr_y + 2 };
                    v_bar(vdi_handle, bar);
                }
                vst_color(vdi_handle, 0);
            } else {
                vst_color(vdi_handle, 1);
            }

            sprintf(buf, "%c %-30s", (i == gem_playing) ? '>' : ' ', file_list[i]);
            v_gtext(vdi_handle, wx + 8, row_scr_y, buf);
        }
    }

    {
        int16_t status_baseline = wy + wh - 2;
        if (status_baseline >= ry && status_baseline <= ry + rh) {
            vsf_color(vdi_handle, 0);
            {
                int16_t status_bg[4] = { wx + 4, status_baseline - line_height + 2,
                                         wx + ww - 4, status_baseline + 2 };
                v_bar(vdi_handle, status_bg);
            }
            vst_color(vdi_handle, 1);
            v_gtext(vdi_handle, wx + 8, status_baseline, gem_status);
        }
    }

    vs_clip(vdi_handle, 0, clip_pxy);
}

static void gem_do_redraw(int16_t xc, int16_t yc, int16_t wc, int16_t hc)
{
    GRECT rect, clip_rect;

    clip_rect.g_x = xc;  clip_rect.g_y = yc;
    clip_rect.g_w = wc;  clip_rect.g_h = hc;

    wind_update(BEG_UPDATE);
    graf_mouse(M_OFF, NULL);

    wind_get(wi_handle, WF_FIRSTXYWH, &rect.g_x, &rect.g_y, &rect.g_w, &rect.g_h);
    while (rect.g_w != 0 && rect.g_h != 0) {
        if (rc_intersect(&clip_rect, &rect)) {
            gem_draw_ui_rect(rect.g_x, rect.g_y, rect.g_w, rect.g_h);
        }
        wind_get(wi_handle, WF_NEXTXYWH, &rect.g_x, &rect.g_y, &rect.g_w, &rect.g_h);
    }

    graf_mouse(M_ON, NULL);
    wind_update(END_UPDATE);
}

/* ── Incremental redraw helpers ─────────────────────────────────────────── */

static void gem_inval_rows(int old_sel, int new_sel, int old_play, int new_play)
{
    int16_t wx = xwork, wy = ywork, ww = wwork, wh = hwork;
    int16_t list_top    = wy + header_h;
    int16_t list_bottom = wy + wh - line_height;
    int16_t rows[4];
    int n = 0;
    int i;

    if (old_sel >= 0 && old_sel != new_sel) rows[n++] = old_sel;
    if (new_sel >= 0) rows[n++] = new_sel;
    if (old_play >= 0 && old_play != new_play && old_play != old_sel && old_play != new_sel)
        rows[n++] = old_play;
    if (new_play >= 0 && new_play != old_play && new_play != old_sel && new_play != new_sel)
        rows[n++] = new_play;

    for (i = 0; i < n; i++) {
        int16_t row = rows[i];
        int16_t row_scr_y = list_top + row * line_height - scroll_y;
        int16_t y1 = row_scr_y - line_height + 2;
        int16_t y2 = row_scr_y + 2;

        if (y2 < list_top || y1 > list_bottom) continue;
        if (y2 > list_bottom) y2 = list_bottom;

        gem_do_redraw(wx + 4, y1, ww - 8, y2 - y1 + 1);
    }
}

static void gem_inval_status(void)
{
    int16_t wy = ywork, wh = hwork;
    int16_t y1 = wy + wh - line_height;
    gem_do_redraw(xwork + 4, y1, wwork - 8, line_height);
}

static void gem_inval_list(void)
{
    int16_t wy = ywork, wh = hwork;
    int16_t list_top = wy + header_h;
    int16_t list_h   = wh - header_h - line_height;
    if (list_h > 0)
        gem_do_redraw(xwork + 4, list_top - line_height + 2,
                      wwork - 8, list_h + line_height - 2);
}

static void gem_force_redraw(void)
{
    gem_do_redraw(xwork, ywork, wwork, hwork);
}

static void gem_main_loop(void)
{
    int16_t msg[8];
    int16_t mx, my, mb, kstat, key, br;
    int quit = 0;
    int16_t ev_mask, timer_ms;

    gem_force_redraw();

    while (!quit) {
        if (gem_playing != -1) {
            ev_mask  = MU_KEYBD | MU_MESAG | MU_TIMER;
            timer_ms = 200;
        } else {
            ev_mask  = MU_KEYBD | MU_MESAG;
            timer_ms = 0;
        }

        int16_t ev = evnt_multi(ev_mask,
                                0, 0, 0,
                                0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0,
                                msg, timer_ms,
                                &mx, &my, &mb, &kstat, &key, &br);

        if (ev & MU_MESAG) {
            if (msg[3] != wi_handle) continue;

            switch (msg[0]) {
            case WM_REDRAW:
                gem_do_redraw(msg[4], msg[5], msg[6], msg[7]);
                break;
            case WM_CLOSED:
                quit = 1;
                break;
            case WM_MOVED:
                wind_set(wi_handle, WF_CURRXYWH, msg[4], msg[5], msg[6], msg[7]);
                wind_get(wi_handle, WF_WORKXYWH, &xwork, &ywork, &wwork, &hwork);
                gem_update_scrollbars();
                gem_force_redraw();
                break;
            case WM_SIZED:
                if (msg[6] < gem_min_w) msg[6] = gem_min_w;
                if (msg[7] < gem_min_h) msg[7] = gem_min_h;
                wind_set(wi_handle, WF_CURRXYWH, msg[4], msg[5], msg[6], msg[7]);
                wind_get(wi_handle, WF_WORKXYWH, &xwork, &ywork, &wwork, &hwork);
                gem_update_scrollbars();
                gem_force_redraw();
                break;

            case WM_ARROWED:
                {
                    int old_scroll = scroll_y;
                    switch (msg[4]) {
                    case WA_UPLINE:
                        scroll_y -= line_height;
                        break;
                    case WA_DNLINE:
                        scroll_y += line_height;
                        break;
                    case WA_UPPAGE:
                        scroll_y -= (hwork - header_h - line_height);
                        break;
                    case WA_DNPAGE:
                        scroll_y += (hwork - header_h - line_height);
                        break;
                    }
                    if (gem_update_scrollbars()) {
                        if (old_scroll != scroll_y)
                            gem_inval_list();
                        else
                            gem_force_redraw();
                    }
                }
                break;

            case WM_VSLID:
                {
                    int old_scroll = scroll_y;
                    int view_h = hwork - header_h - line_height;
                    if (doc_h > view_h && view_h > 0) {
                        scroll_y = (msg[4] * (doc_h - view_h)) / 1000;
                        if (scroll_y < 0) scroll_y = 0;
                        if (scroll_y > doc_h - view_h) scroll_y = doc_h - view_h;
                    } else {
                        scroll_y = 0;
                    }
                    wind_set(wi_handle, WF_VSLIDE, msg[4], 0, 0, 0);
                    if (old_scroll != scroll_y)
                        gem_inval_list();
                    else
                        gem_force_redraw();
                }
                break;
            }
        }

        if (ev & MU_KEYBD) {
            char ascii = key & 0xFF;
            char scan  = (key >> 8) & 0xFF;
            int needs_redraw = 0;
            int old_scroll = scroll_y;
            int old_sel = gem_selected;
            int old_play = gem_playing;

            if (ascii == 'q' || ascii == 'Q' || ascii == 27) {
                quit = 1;
            } else if (scan == 0x48 && gem_selected > 0) {
                gem_selected--;
                if (gem_selected * line_height < scroll_y)
                    scroll_y = gem_selected * line_height;
                needs_redraw = 1;
            } else if (scan == 0x50 && gem_selected < num_files - 1) {
                gem_selected++;
                {
                    int view_h = hwork - header_h - line_height;
                    int sel_bottom = (gem_selected + 1) * line_height;
                    if (sel_bottom > scroll_y + view_h)
                        scroll_y = sel_bottom - view_h;
                }
                needs_redraw = 1;
            } else if (ascii == 'l' || ascii == 'L') {
                loop_mode = !loop_mode;
                if (loop_mode) autoplay_mode = 0;
                sprintf(gem_status, "Loop %s", loop_mode ? "ON" : "OFF");
                gem_inval_status();
            } else if (ascii == 'a' || ascii == 'A') {
                autoplay_mode = !autoplay_mode;
                if (autoplay_mode) loop_mode = 0;
                sprintf(gem_status, "Autoplay %s", autoplay_mode ? "ON" : "OFF");
                gem_inval_status();
            } else if (ascii == 13) {
                int res = start_vgm(file_list[gem_selected]);
                if (res == 0) {
                    gem_playing = gem_selected;
                    sprintf(gem_status, "Playing: %s", file_list[gem_playing]);
                } else {
                    gem_playing = -1;
                    sprintf(gem_status, "Err %d: %s", res, file_list[gem_selected]);
                }
                needs_redraw = 1;
            } else if (ascii == 32) {
                if (gem_playing != -1) {
                    stop_vgm();
                    gem_playing = -1;
                    sprintf(gem_status, "Stopped.");
                    gem_inval_status();
                }
            }

            if (needs_redraw) {
                if (old_scroll != scroll_y) {
                    gem_update_scrollbars();
                    gem_inval_list();
                } else {
                    gem_inval_rows(old_sel, gem_selected, old_play, gem_playing);
                    gem_inval_status();
                }
            }
        }

        if (ev & MU_TIMER) {
            if (gem_playing != -1 && !vgm_playing) {
                int old_scroll = scroll_y;
                int old_play = gem_playing;
                int old_sel  = gem_selected;

                if (loop_mode) {
                    vgm_ptr = vgm_loop_ptr ? vgm_loop_ptr : vgm_buffer + vgm_data_offset;
                    timer_frac = 0; timer_window = 45; vgm_playing = 1;
                    sprintf(gem_status, "Loop: %s", file_list[gem_playing]);
                } else if (autoplay_mode) {
                    int next = (gem_playing + 1) % num_files;
                    if (start_vgm(file_list[next]) == 0) {
                        gem_playing = next;
                        gem_selected = next;
                        {
                            int view_h = hwork - header_h - line_height;
                            int sel_bottom = (gem_selected + 1) * line_height;
                            if (sel_bottom > scroll_y + view_h)
                                scroll_y = sel_bottom - view_h;
                            if (gem_selected * line_height < scroll_y)
                                scroll_y = gem_selected * line_height;
                        }
                        sprintf(gem_status, "Playing: %s", file_list[gem_playing]);
                    } else {
                        stop_vgm(); gem_playing = -1;
                        sprintf(gem_status, "Autoplay err");
                    }
                } else {
                    stop_vgm(); gem_playing = -1;
                    sprintf(gem_status, "Playback finished.");
                }

                gem_update_scrollbars();
                if (old_scroll != scroll_y)
                    gem_inval_list();
                else
                    gem_inval_rows(old_sel, gem_selected, old_play, gem_playing);
                gem_inval_status();
            }
        }
    }
}

static int main_gem(int argc, char **argv)
{
    aes_apid = appl_init();
    if (aes_apid < 0) return -1;

    phys_handle = graf_handle(&gem_font_w, &gem_font_h, &gem_font_wbox, &gem_font_hbox);
    wind_get(0, WF_WORKXYWH, &xdesk, &ydesk, &wdesk, &hdesk);
    gem_open_vwork();

    if (argc >= 2) {
        const char *arg = argv[1];
        int len = strlen(arg);
        if (len > 4 && (strcmp(arg + len - 4, ".m3u") == 0 ||
                        strcmp(arg + len - 4, ".M3U") == 0)) {
            if (load_m3u(arg) < 0) {
                scan_files();
            }
        } else {
            scan_files();
        }
    } else {
        scan_files();
    }

    gem_selected = 0;
    gem_playing = -1;
    strcpy(gem_status, "Ready.");
    scroll_y = 0;

    gem_open_window();

    {
        void *ssp = (void *)Super(0);
        gem_old_conterm = CONTERM;
        CONTERM &= ~0x01;
        Super(ssp);
    }

    if (argc >= 2) {
        const char *arg = argv[1];
        int len = strlen(arg);
        int is_m3u = (len > 4 && (strcmp(arg + len - 4, ".m3u") == 0 ||
                                  strcmp(arg + len - 4, ".M3U") == 0));
        if (!is_m3u) {
            char filename[128];
            if (argc == 2) {
                strncpy(filename, argv[1], 127);
                filename[127] = '\0';
            } else {
                int pos = 0;
                for (int i = 1; i < argc && pos < 127; i++) {
                    if (i > 1) filename[pos++] = ' ';
                    int l = strlen(argv[i]);
                    if (pos + l > 127) l = 127 - pos;
                    memcpy(filename + pos, argv[i], l);
                    pos += l;
                }
                filename[pos] = '\0';
            }
            int res = start_vgm(filename);
            if (res == 0) {
                gem_playing = 0;
                sprintf(gem_status, "Playing: %s", filename);
            } else {
                sprintf(gem_status, "Error code %d", res);
            }
        }
    }

    gem_main_loop();

    stop_vgm();
    wind_close(wi_handle);
    wind_delete(wi_handle);
    v_clsvwk(vdi_handle);

    {
        void *ssp = (void *)Super(0);
        CONTERM = gem_old_conterm;
        Super(ssp);
    }

    {
        int16_t x, y, w, h;
        wind_get(0, WF_WORKXYWH, &x, &y, &w, &h);
        form_dial(FMD_FINISH, 0, 0, 0, 0, x, y, w, h);
    }

    appl_exit();
    return 0;
}


/* ══════════════════════════════════════════════════════════════════════════
 * PURE TOS / TTP UI LOOP
 * ══════════════════════════════════════════════════════════════════════════
 */

void draw_menu(int selected, int playing, const char *status)
{
    printf("\033E");
    printf("\033f");
    printf("--- mm_vgm v0.2 Interactive Player ---\n");
    printf("UP/DOWN: Select | ENTER: Play | SPACE: Stop | Q: Quit\n");
    printf("Loop:%s Auto:%s\n",
        loop_mode ? "ON" : "OFF",
        autoplay_mode ? "ON" : "OFF");
    printf("---------------------------------------\n");

    if (num_files == 0) {
        printf("No .vgm or .vgz files found.\n");
        return;
    }

    int start_idx = selected - 10;
    if (start_idx < 0) start_idx = 0;
    if (start_idx > num_files - 20 && num_files > 20) start_idx = num_files - 20;

    for (int i = start_idx; i < start_idx + 20 && i < num_files; i++) {
        if (i == selected) printf("\033p");

        if (i == playing) printf(" > ");
        else printf("   ");

        printf("%-30s", file_list[i]);

        if (i == selected) printf("\033q");
        printf("\n");
    }
    printf("---------------------------------------\n");
    if (status) printf("%s\n", status);
}

void stop_vgm(void)
{
    vgm_playing = 0;
    restore_hardware();
    if (vgm_buffer) {
        free(vgm_buffer);
        vgm_buffer = NULL;
    }
}

int start_vgm(const char *filename)
{
    stop_vgm();

    uint32_t target_size = get_uncompressed_size(filename);
    if (target_size == 0) return -1;

    vgm_buffer = (uint8_t *)malloc(target_size);
    if (!vgm_buffer) return -2;

    gzFile gf = gzopen(filename, "rb");
    if (!gf) { free(vgm_buffer); vgm_buffer = NULL; return -3; }

    int bytes_read = gzread(gf, vgm_buffer, target_size);
    gzclose(gf);

    if (bytes_read < 0x40 || memcmp(vgm_buffer, "Vgm ", 4) != 0) {
        free(vgm_buffer); vgm_buffer = NULL; return -4;
    }

    uint32_t ym2413_clock = read_le32(&vgm_buffer[0x10]);
    uint32_t ym2612_clock = read_le32(&vgm_buffer[0x2C]);
    sn_clock_hz = read_le32(&vgm_buffer[0x0C]) & 0x7FFFFFFFUL;

    if (sn_clock_hz == 0) {
        free(vgm_buffer); vgm_buffer = NULL;
        if (ym2612_clock != 0) return -5;
        if (ym2413_clock != 0) return -7;
        return -6;
    }

    uint32_t version = read_le32(&vgm_buffer[0x08]);
    uint32_t data_offset = 0x40;
    if (version >= 0x00000150) {
        uint32_t rel = read_le32(&vgm_buffer[0x34]);
        if (rel != 0) {
            if (rel > (uint32_t)bytes_read - 0x34UL) {
                free(vgm_buffer); vgm_buffer = NULL; return -8;
            }
            data_offset = rel + 0x34UL;
        }
    }

    if (data_offset >= (uint32_t)bytes_read) {
        free(vgm_buffer); vgm_buffer = NULL; return -8;
    }

    uint32_t loop_ofs = read_le32(&vgm_buffer[0x1C]);
    vgm_loop_ptr = NULL;
    if (loop_ofs != 0 && loop_ofs <= (uint32_t)bytes_read - 0x1CUL) {
        uint32_t lp = 0x1CUL + loop_ofs;
        if (lp < (uint32_t)bytes_read)
            vgm_loop_ptr = vgm_buffer + lp;
    }

    init_tables();
    vgm_ptr = vgm_buffer + data_offset;
    vgm_end_ptr = vgm_buffer + bytes_read;
    timer_frac = 0;
    timer_window = 45;
    vgm_playing = 1;

    vgm_data_offset = data_offset;

    init_hardware();
    return 0;
}

static int main_tos(int argc, char **argv)
{
    void *old_ssp = (void *)Super(0);

    if (argc >= 2) {
        printf("\033E");
        printf("\n--- mm_vgm v0.2 CLI Mode ---\n");

        char filename[128];
        if (argc == 2) {
            strncpy(filename, argv[1], 127);
            filename[127] = '\0';
        } else {
            int pos = 0;
            for (int i = 1; i < argc && pos < 127; i++) {
                if (i > 1) filename[pos++] = ' ';
                int len = strlen(argv[i]);
                if (pos + len > 127) len = 127 - pos;
                memcpy(filename + pos, argv[i], len);
                pos += len;
            }
            filename[pos] = '\0';
        }

        int res = start_vgm(filename);
        if (res < 0) {
            printf("[ERR] Could not load \"%s\" (code %d)\nPress any key...\n",
                   filename, res);
            Cconin();
        } else {
            printf("[INFO] Playing \"%s\"...\nPress any key to stop.\n", filename);
            while (vgm_playing) { if (Cconis()) break; }
            if (Cconis()) Cconin();
        }
        stop_vgm();
        Super(old_ssp);
        return 0;
    }

    scan_files();
    int selected = 0;
    int playing = -1;
    char status[64] = "Ready.";

    draw_menu(selected, playing, status);

    old_conterm = CONTERM;
    CONTERM = old_conterm & ~0x01;
    while (1) {
        if (Cconis()) {
            long key = Cconin();
            char ascii = key & 0xFF;
            char scan = (key >> 16) & 0xFF;

            if (ascii == 'q' || ascii == 'Q' || ascii == 27) {
                break;
            }
            else if (scan == 0x48) {
                if (selected > 0) {
                    selected--;
                    draw_menu(selected, playing, status);
                }
            }
            else if (scan == 0x50) {
                if (selected < num_files - 1) {
                    selected++;
                    draw_menu(selected, playing, status);
                }
            }
            else if (ascii == 'l' || ascii == 'L') {
                loop_mode = !loop_mode;
                if (loop_mode) autoplay_mode = 0;
                sprintf(status, "Loop %s", loop_mode ? "ON" : "OFF");
                draw_menu(selected, playing, status);
            }
            else if (ascii == 'a' || ascii == 'A') {
                autoplay_mode = !autoplay_mode;
                if (autoplay_mode) loop_mode = 0;
                sprintf(status, "Autoplay %s", autoplay_mode ? "ON" : "OFF");
                draw_menu(selected, playing, status);
            }
            else if (ascii == 13 || ascii == 32) {
                if (playing != -1 && ascii == 32) {
                    stop_vgm();
                    playing = -1;
                    sprintf(status, "Stopped.");
                    draw_menu(selected, playing, status);
                } else if (ascii == 13) {
                    sprintf(status, "Loading %s...", file_list[selected]);
                    draw_menu(selected, playing, status);

                    int res = start_vgm(file_list[selected]);
                    if (res == 0) {
                        playing = selected;
                        sprintf(status, "Playing: %s", file_list[playing]);
                    } else {
                        playing = -1;
                        sprintf(status, "Error code %d on %s", res, file_list[selected]);
                    }
                    draw_menu(selected, playing, status);
                }
            }
        }

        if (playing != -1 && !vgm_playing) {
            if (loop_mode) {
                vgm_ptr = vgm_buffer + vgm_data_offset;
                if (vgm_loop_ptr) vgm_ptr = vgm_loop_ptr;
                timer_frac = 0;
                timer_window = 45;
                vgm_playing = 1;
                sprintf(status, "Loop: %s", file_list[playing]);
            }
            else if (autoplay_mode) {
                int next = playing + 1;
                if (next >= num_files) next = 0;
                int res = start_vgm(file_list[next]);
                if (res == 0) {
                    playing = next;
                    selected = next;
                    sprintf(status, "Playing: %s", file_list[playing]);
                } else {
                    stop_vgm();
                    playing = -1;
                    sprintf(status, "Autoplay err %d", res);
                }
            }
            else {
                stop_vgm();
                playing = -1;
                sprintf(status, "Playback finished.");
            }
            draw_menu(selected, playing, status);
        }
    }

    stop_vgm();
    CONTERM = old_conterm;
    printf("\033e");
    printf("\033E");
    Super(old_ssp);
    return 0;
}


/* ══════════════════════════════════════════════════════════════════════════
 * ENTRY POINT
 * ══════════════════════════════════════════════════════════════════════════
 */
int main(int argc, char **argv)
{
    aes_apid = appl_init();
    if (aes_apid >= 0) {
        appl_exit();
        return main_gem(argc, argv);
    }
    return main_tos(argc, argv);
}
