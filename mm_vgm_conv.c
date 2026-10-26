
/*
 * mm_vgm2ym.c — Atari ST/FreeMiNT batch VGM/VGZ -> YM6 converter
 * Medour Mehdi - 2026
 * Architecture: Motorola 68000 / Atari ST-TT-Falcon
 *
 * Compile (build as .prg/.app: it opens a GEM window):
 *   m68k-atari-mint-gcc -O2 -m68000 mm_vgm2ym.c -o vgm2ym.prg -lz
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <zlib.h>
#include <mint/osbind.h>

#include <gem.h>

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 1 — VGM (SN76489) -> YM2149 register translation engine
 * ══════════════════════════════════════════════════════════════════════════
 */

typedef struct {
    uint16_t tone[4];
    uint8_t  vol[4];
    uint8_t  latched;
    uint8_t  mixer;
} conv_state_t;

static uint8_t  lut_vol[16];
static uint8_t  noise_ym[4];
static uint16_t lut_tone[1024];
static uint8_t  skip_len[256];
static uint8_t  ym_regs[16];

static void init_skip_table(void)
{
    memset(skip_len, 0, sizeof(skip_len));
    for (int i = 0x30; i <= 0x3F; i++) skip_len[i] = 1;
    for (int i = 0x40; i <= 0x4E; i++) skip_len[i] = 2;
    skip_len[0x4F] = 1; skip_len[0x50] = 1;
    for (int i = 0x51; i <= 0x5F; i++) skip_len[i] = 2;
    skip_len[0x61] = 2; skip_len[0x64] = 3; skip_len[0x68] = 11;
    skip_len[0x90] = 4; skip_len[0x91] = 4; skip_len[0x92] = 5;
    skip_len[0x93] = 10; skip_len[0x94] = 1; skip_len[0x95] = 4;
    for (int i = 0xA0; i <= 0xBF; i++) skip_len[i] = 2;
    for (int i = 0xC0; i <= 0xCF; i++) skip_len[i] = 3;
    for (int i = 0xD0; i <= 0xDF; i++) skip_len[i] = 3;
    for (int i = 0xE0; i <= 0xFF; i++) skip_len[i] = 4;
}

static void init_conv_tables(uint32_t sn_clock_hz)
{
    for (int i = 0; i < 1024; i++) {
        uint32_t n = (i == 0) ? 1UL : (uint32_t)i;
        uint32_t m = (n * 4000000UL) / sn_clock_hz;
        if (m < 1UL)    m = 1UL;
        if (m > 4095UL) m = 4095UL;
        lut_tone[i] = (uint16_t)m;
    }

    static const uint8_t vol_map[16] =
        { 15,14,14,13,12,11,10,10, 9, 8, 7, 6, 6, 5, 4, 0 };
    memcpy(lut_vol, vol_map, 16);

    uint32_t r;
    r = (32UL * 2000000UL) / sn_clock_hz;  if (r < 1) r = 1; if (r > 31) r = 31;
    noise_ym[0] = (uint8_t)r;
    r = (64UL * 2000000UL) / sn_clock_hz;  if (r < 1) r = 1; if (r > 31) r = 31;
    noise_ym[1] = (uint8_t)r;
    r = (128UL * 2000000UL) / sn_clock_hz; if (r < 1) r = 1; if (r > 31) r = 31;
    noise_ym[2] = (uint8_t)r;
    noise_ym[3] = 0;
}

static void ym_write(int reg, uint8_t val) { ym_regs[reg & 0x0F] = val; }

static void write_noise_and_mixer(conv_state_t *st, uint8_t period)
{
    ym_write(6, period);
    if (st->vol[3] == 15) return;
    st->mixer = 0x18;
    ym_write(7, st->mixer);
}

static void update_noise(conv_state_t *st)
{
    uint8_t nc = (uint8_t)(st->tone[3] & 0x07);
    uint8_t nf = nc & 0x03;

    if (nf == 3) {
        if (nc & 0x04) {
            uint16_t ymp = lut_tone[st->tone[2] & 0x3FF];
            uint8_t period = (ymp == 0) ? 1 : (ymp > 31 ? 31 : (uint8_t)ymp);
            write_noise_and_mixer(st, period);
        } else if (st->vol[2] == 15) {
            st->mixer = 0x38;
            ym_write(7, st->mixer);
        } else {
            write_noise_and_mixer(st, 1);
        }
        return;
    }

    uint8_t period;
    if (!(nc & 0x04)) {
        period = 1;
    } else {
        period = noise_ym[nf];
        if (nf == 2 && st->vol[2] == 15) {
            ym_write(4, 0xF4);
            ym_write(5, 0x01);
        }
    }
    write_noise_and_mixer(st, period);
}

static void process_volume(conv_state_t *st, int chan, uint8_t snvol)
{
    st->vol[chan] = snvol;
    uint8_t ymvol = lut_vol[snvol];

    if (chan == 0 || chan == 1) {
        ym_write(8 + chan, ymvol);
        return;
    }
    if (chan == 2) {
        if (st->vol[3] != 15) return;
        ym_write(10, ymvol);
        return;
    }

    ym_write(10, ymvol);
    if (ymvol != 0) {
        uint8_t nc = st->tone[3] & 0x07;
        st->mixer = (nc == 0x03) ? (st->vol[2] == 15 ? 0x38 : 0x18) : 0x18;
        ym_write(7, st->mixer);
        return;
    }
    if (st->vol[2] != 15) ym_write(10, lut_vol[st->vol[2]]);
    st->mixer = 0x38;
    ym_write(7, st->mixer);
}

static void process_sn_byte(conv_state_t *st, uint8_t d4)
{
    uint8_t sel;
    if (d4 & 0x80) {
        sel = d4 & 0x70;
        st->latched = sel;
    } else {
        sel = st->latched;
    }

    int chan = (sel >> 5) & 3;

    if (sel & 0x10) {
        process_volume(st, chan, d4 & 0x0F);
        return;
    }
    if (chan == 3) {
        st->tone[3] = d4 & 0x07;
        update_noise(st);
        return;
    }

    uint16_t period = st->tone[chan];
    if (d4 & 0x80) {
        period = (period & 0x3F0) | (d4 & 0x0F);
    } else {
        period = (period & 0x000F) | ((uint16_t)(d4 & 0x3F) << 4);
    }
    st->tone[chan] = period;

    uint16_t ymp = lut_tone[period & 0x3FF];
    ym_write(chan * 2,     (uint8_t)(ymp & 0xFF));
    ym_write(chan * 2 + 1, (uint8_t)((ymp >> 8) & 0x0F));
}

#define FRAME_PERIOD_SAMPLES 882u

static uint32_t wait_62 = 735;   /* 0x62 = 1/60 s, may be overridden by 0x64 */
static uint32_t wait_63 = 882;   /* 0x63 = 1/50 s, may be overridden by 0x64 */

static uint8_t  *frame_buf = NULL;
static uint32_t  frame_cap = 0;
static uint32_t  frame_n   = 0;
static uint32_t  frame_rem = FRAME_PERIOD_SAMPLES;

static int frame_push(void)
{
    if (frame_n >= frame_cap) {
        uint32_t newcap = frame_cap ? frame_cap * 2 : 4096;
        uint8_t *nb = (uint8_t *)realloc(frame_buf, (size_t)newcap * 16);
        if (!nb) return -1;
        frame_buf = nb;
        frame_cap = newcap;
    }
    uint8_t *row = frame_buf + (size_t)frame_n * 16;
    memcpy(row, ym_regs, 14);
    row[14] = 0;
    row[15] = 0;
    frame_n++;
    return 0;
}

static int emit_wait(uint32_t wait_samples)
{
    while (wait_samples > 0) {
        if (wait_samples < frame_rem) {
            frame_rem -= wait_samples;
            wait_samples = 0;
        } else {
            wait_samples -= frame_rem;
            frame_rem = FRAME_PERIOD_SAMPLES;
            if (frame_push() < 0) return -1;
        }
    }
    return 0;
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int parse_and_convert(uint8_t *buf, uint32_t size, uint32_t data_offset,
                              uint32_t loop_abs_offset, uint32_t *out_loop_frame)
{
    uint8_t *p   = buf + data_offset;
    uint8_t *end = buf + size;
    uint8_t *loop_ptr = (loop_abs_offset && loop_abs_offset < size) ? buf + loop_abs_offset : NULL;
    int have_loop_frame = 0;

    conv_state_t st;
    memset(&st, 0, sizeof(st));
    st.mixer = 0x38;
    memset(ym_regs, 0, sizeof(ym_regs));
    ym_regs[7] = 0x38;

    frame_rem = FRAME_PERIOD_SAMPLES;
    frame_n   = 0;
    wait_62   = 735;
    wait_63   = 882;
    *out_loop_frame = 0;

    while (p < end) {
        if (loop_ptr && !have_loop_frame && p == loop_ptr) {
            *out_loop_frame = frame_n;
            have_loop_frame = 1;
        }

        uint8_t op = *p++;

        if (op == 0x50) {
            if (p >= end) break;
            process_sn_byte(&st, *p++);
        } else if (op == 0x61) {
            if (p + 2 > end) break;
            uint16_t w = (uint16_t)(p[0] | (p[1] << 8));
            p += 2;
            if (emit_wait(w) < 0) return -1;
        } else if (op == 0x62) {
            if (emit_wait(wait_62) < 0) return -1;
        } else if (op == 0x63) {
            if (emit_wait(wait_63) < 0) return -1;
        } else if (op == 0x64) {
            if (p + 3 > end) break;
            {
                uint32_t n = (uint32_t)(p[1] | (p[2] << 8));
                if (p[0] == 0x62) wait_62 = n;
                else if (p[0] == 0x63) wait_63 = n;
            }
            p += 3;
        } else if (op == 0x66) {
            break;
        } else if (op == 0x67) {
            if (p + 6 > end) break;
            p += 1;
            p += 1;
            uint32_t dsize = read_le32(p);
            p += 4;
            if (p + dsize > end) break;
            p += dsize;
        } else if (op == 0xA0) {
            if (p + 2 > end) break;
            if (!(p[0] & 0x80))            /* bit 7 = second AY chip: ignore */
                ym_regs[p[0] & 0x0F] = p[1];
            p += 2;
        } else if ((op & 0xF0) == 0x70) {
            if (emit_wait((op & 0x0F) + 1) < 0) return -1;
        } else if ((op & 0xF0) == 0x80) {
            if (emit_wait(op & 0x0F) < 0) return -1;
        } else {
            uint8_t skip = skip_len[op];
            if (p + skip > end) break;
            p += skip;
        }
    }

    /* flush the last partial frame so trailing register writes are kept */
    if (frame_rem != FRAME_PERIOD_SAMPLES && frame_push() < 0) return -1;

    return (int)frame_n;
}

static int utf16_units(const uint8_t *src, const uint8_t *limit)
{
    int n = 0;
    while (src + n * 2 + 1 < limit && !(src[n*2] == 0 && src[n*2+1] == 0)) n++;
    return n;
}

static void ascii_from_utf16le(const uint8_t *src, int units, char *dst, int dstsz)
{
    int n = units < dstsz - 1 ? units : dstsz - 1;
    int i;
    for (i = 0; i < n; i++) {
        uint16_t c = (uint16_t)(src[i*2] | (src[i*2+1] << 8));
        dst[i] = (c >= 32 && c < 127) ? (char)c : '?';
    }
    dst[i] = '\0';
}

static void extract_gd3(uint8_t *buf, uint32_t size,
                         char *title, int tsz, char *author, int asz,
                         char *comment, int csz)
{
    title[0] = author[0] = comment[0] = '\0';
    if (size < 0x18) return;

    uint32_t gd3_rel = read_le32(&buf[0x14]);
    if (gd3_rel == 0) return;
    uint32_t gd3_abs = 0x14 + gd3_rel;
    if (gd3_abs + 12 > size) return;
    if (memcmp(&buf[gd3_abs], "Gd3 ", 4) != 0) return;

    uint32_t gd3_len = read_le32(&buf[gd3_abs + 8]);
    uint8_t *p = &buf[gd3_abs + 12];
    uint8_t *lim = p + gd3_len;
    if (lim > buf + size) lim = buf + size;

    for (int idx = 0; idx < 11 && p < lim; idx++) {
        int units = utf16_units(p, lim);
        if (idx == 0)       ascii_from_utf16le(p, units, title,   tsz);
        else if (idx == 6)  ascii_from_utf16le(p, units, author,  asz);
        else if (idx == 10) ascii_from_utf16le(p, units, comment, csz);
        p += (units + 1) * 2;
    }
}

static uint8_t *build_ym6_buffer(uint32_t nb_frames, uint32_t loop_frame,
                                  const char *title, const char *author, const char *comment,
                                  size_t *out_len)
{
    size_t cap = 4 + 8 + 4+4+2+4+2+4+2
               + strlen(title) + 1 + strlen(author) + 1 + strlen(comment) + 1
               + (size_t)nb_frames * 16 + 4 + 64;
    uint8_t *buf = (uint8_t *)malloc(cap);
    if (!buf) return NULL;
    size_t p = 0;

    memcpy(buf + p, "YM6!", 4); p += 4;
    memcpy(buf + p, "LeOnArD!", 8); p += 8;
#define PUT32(v) do { uint32_t _v=(v); buf[p++]=(uint8_t)(_v>>24); buf[p++]=(uint8_t)(_v>>16); buf[p++]=(uint8_t)(_v>>8); buf[p++]=(uint8_t)_v; } while(0)
#define PUT16(v) do { uint16_t _v=(v); buf[p++]=(uint8_t)(_v>>8); buf[p++]=(uint8_t)_v; } while(0)
    PUT32(nb_frames);
    PUT32(0x00000001UL);
    PUT16(0);
    PUT32(2000000UL);
    PUT16(50);
    PUT32(loop_frame);
    PUT16(0);
#undef PUT32
#undef PUT16
    size_t tl = strlen(title), al = strlen(author), cl = strlen(comment);
    memcpy(buf + p, title, tl);   p += tl;   buf[p++] = 0;
    memcpy(buf + p, author, al);  p += al;   buf[p++] = 0;
    memcpy(buf + p, comment, cl); p += cl;   buf[p++] = 0;

    for (int reg = 0; reg < 16; reg++)
        for (uint32_t i = 0; i < nb_frames; i++)
            buf[p++] = frame_buf[(size_t)i * 16 + reg];

    memcpy(buf + p, "End!", 4); p += 4;
    *out_len = p;
    return buf;
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 1b — inline LZH ("-lh1-") codec
 * ══════════════════════════════════════════════════════════════════════════
 */

#define LZH_N          4096
#define LZH_F          60
#define LZH_THRESHOLD  2
#define LZH_N_CHAR     (256 - LZH_THRESHOLD + LZH_F)
#define LZH_T          (LZH_N_CHAR * 2 - 1)
#define LZH_R          (LZH_T - 1)
#define LZH_MAX_FREQ   0x8000
#define LZH_N_MASK     (LZH_N - 1)

static const unsigned char LZH_D_CODE_SRC[] =
"0000000000000000000000000000000011111111111111112222222222222222"
"3333333333333333444444445555555566666666777777778888888899999999"
"::::::::;;;;;;;;<<<<====>>>>????@@@@AAAABBBBCCCCDDDDEEEEFFFFGGGG"
"HHIIJJKKLLMMNNOOPPQQRRSSTTUUVVWWXXYYZZ[[\\\\]]^^__`abcdefghijklmno";
static const unsigned char LZH_D_LEN_SRC[] =
"3333333333333333333333333333333344444444444444444444444444444444"
"4444444444444444555555555555555555555555555555555555555555555555"
"5555555555555555666666666666666666666666666666666666666666666666"
"7777777777777777777777777777777777777777777777778888888888888888";

static int lzh_c_code[64], lzh_c_len[64];
static int lzh_tables_ready = 0;

static void lzh_build_tables(void)
{
    int d_code[256], d_len[256];
    for (int i = 0; i < 256; i++) { d_code[i] = LZH_D_CODE_SRC[i] - '0'; d_len[i] = LZH_D_LEN_SRC[i] - '0'; }
    for (int v = 0; v < 64; v++) lzh_c_len[v] = -1;
    for (int i = 0; i < 256; i++) {
        int v = d_code[i];
        if (lzh_c_len[v] == -1) {
            int L = d_len[i];
            lzh_c_len[v] = L;
            lzh_c_code[v] = (unsigned)i >> (8 - L);
        }
    }
    lzh_tables_ready = 1;
}

typedef struct {
    int freq[LZH_T + 1];
    int prnt[LZH_T + LZH_N_CHAR];
    int son[LZH_T];
} lzh_huff_t;

static void lzh_start_huff(lzh_huff_t *h)
{
    int i, j;
    for (i = 0; i < LZH_N_CHAR; i++) { h->freq[i] = 1; h->son[i] = i + LZH_T; h->prnt[i + LZH_T] = i; }
    i = 0; j = LZH_N_CHAR;
    while (j <= LZH_R) {
        h->freq[j] = h->freq[i] + h->freq[i + 1];
        h->son[j] = i;
        h->prnt[i] = h->prnt[i + 1] = j;
        i += 2; j++;
    }
    h->freq[LZH_T] = 0xffff;
    h->prnt[LZH_R] = 0;
}

static void lzh_reconst(lzh_huff_t *h)
{
    int i, j, k, f, l;
    j = 0;
    for (i = 0; i < LZH_T; i++) {
        if (h->son[i] >= LZH_T) { h->freq[j] = (h->freq[i] + 1) / 2; h->son[j] = h->son[i]; j++; }
    }
    i = 0; j = LZH_N_CHAR;
    while (j < LZH_T) {
        k = i + 1;
        f = h->freq[j] = h->freq[i] + h->freq[k];
        for (k = j - 1; f < h->freq[k]; k--) ;
        k++;
        l = (j - k) * (int)sizeof(int);
        memmove(&h->freq[k + 1], &h->freq[k], (size_t)l);
        h->freq[k] = f;
        memmove(&h->son[k + 1], &h->son[k], (size_t)l);
        h->son[k] = i;
        i += 2; j++;
    }
    for (i = 0; i < LZH_T; i++) {
        if ((k = h->son[i]) >= LZH_T) h->prnt[k] = i;
        else { h->prnt[k] = i; h->prnt[k + 1] = i; }
    }
}

static void lzh_update(lzh_huff_t *h, int c)
{
    int i, j, k, l;
    if (h->freq[LZH_R] == LZH_MAX_FREQ) lzh_reconst(h);
    c = h->prnt[c + LZH_T];
    do {
        k = ++h->freq[c];
        l = c + 1;
        if (k > h->freq[l]) {
            while (k > h->freq[++l]) ;
            l--;
            h->freq[c] = h->freq[l];
            h->freq[l] = k;
            i = h->son[c];
            h->prnt[i] = l;
            if (i < LZH_T) h->prnt[i + 1] = l;
            j = h->son[l];
            h->son[l] = i;
            h->prnt[j] = c;
            if (j < LZH_T) h->prnt[j + 1] = c;
            h->son[c] = j;
            c = l;
        }
        c = h->prnt[c];
    } while (c != 0);
}

typedef struct { uint8_t *buf; size_t len, cap; uint32_t acc; int nbits; } lzh_bw_t;

static void lzh_bw_init(lzh_bw_t *bw) { memset(bw, 0, sizeof(*bw)); }
static void lzh_bw_push(lzh_bw_t *bw, uint8_t b)
{
    if (bw->len >= bw->cap) { bw->cap = bw->cap ? bw->cap * 2 : 4096; bw->buf = (uint8_t *)realloc(bw->buf, bw->cap); }
    bw->buf[bw->len++] = b;
}
static void lzh_putbit(lzh_bw_t *bw, int bit)
{
    bw->acc = (bw->acc << 1) | (unsigned)(bit & 1);
    if (++bw->nbits == 8) { lzh_bw_push(bw, (uint8_t)bw->acc); bw->acc = 0; bw->nbits = 0; }
}
static void lzh_putbits(lzh_bw_t *bw, unsigned code, int len)
{
    for (int i = len - 1; i >= 0; i--) lzh_putbit(bw, (int)((code >> i) & 1));
}
static void lzh_bw_flush(lzh_bw_t *bw)
{
    if (bw->nbits > 0) { bw->acc <<= (8 - bw->nbits); lzh_bw_push(bw, (uint8_t)bw->acc); bw->acc = 0; bw->nbits = 0; }
}

static void lzh_encode_char(lzh_huff_t *h, lzh_bw_t *bw, int ch)
{
    int c = h->prnt[ch + LZH_T];
    int bits[24], nbits = 0;
    while (c != LZH_R) {
        int p = h->prnt[c];
        bits[nbits++] = (h->son[p] == c) ? 0 : 1;
        c = p;
    }
    for (int i = nbits - 1; i >= 0; i--) lzh_putbit(bw, bits[i]);
    lzh_update(h, ch);
}

static void lzh_encode_position(lzh_bw_t *bw, int p)
{
    int high6 = (p >> 6) & 0x3F;
    int low6  = p & 0x3F;
    lzh_putbits(bw, (unsigned)lzh_c_code[high6], lzh_c_len[high6]);
    lzh_putbits(bw, (unsigned)low6, 6);
}

#define LZH_HASH_BITS  15
#define LZH_HASH_SIZE  (1 << LZH_HASH_BITS)
#define LZH_MAX_CHAIN  48

static unsigned lzh_hash3(const uint8_t *p)
{
    unsigned hv = (unsigned)p[0] | ((unsigned)p[1] << 8) | ((unsigned)p[2] << 16);
    hv = (hv * 2654435761u) >> (32 - LZH_HASH_BITS);
    return hv & (LZH_HASH_SIZE - 1);
}

static uint8_t *lzh_compress(const uint8_t *in, size_t n, size_t *out_len)
{
    if (!lzh_tables_ready) lzh_build_tables();

    lzh_huff_t *h = (lzh_huff_t *)malloc(sizeof(lzh_huff_t));
    lzh_bw_t bw; lzh_bw_init(&bw);
    if (!h) return NULL;
    lzh_start_huff(h);

    int32_t *head = (int32_t *)malloc(sizeof(int32_t) * LZH_HASH_SIZE);
    int32_t *prev = (int32_t *)malloc(sizeof(int32_t) * (n ? n : 1));
    if (!head || !prev) { free(h); free(head); free(prev); return NULL; }
    for (int i = 0; i < LZH_HASH_SIZE; i++) head[i] = -1;

    size_t pos = 0;
    while (pos < n) {
        size_t max_match = LZH_F; if (max_match > n - pos) max_match = n - pos;
        size_t win_start = (pos > LZH_N) ? pos - LZH_N : 0;
        size_t best_len = 0, best_dist = 0;

        if (max_match >= 3) {
            unsigned hv = lzh_hash3(in + pos);
            int32_t cand = head[hv];
            int chain = 0;
            while (cand >= 0 && (size_t)cand >= win_start && chain < LZH_MAX_CHAIN) {
                size_t L = 0;
                while (L < max_match && in[(size_t)cand + L] == in[pos + L]) L++;
                if (L > best_len) {
                    best_len = L; best_dist = pos - (size_t)cand;
                    if (L >= max_match) break;
                }
                cand = prev[cand];
                chain++;
            }
        }

        if (best_len <= LZH_THRESHOLD) {
            lzh_encode_char(h, &bw, in[pos]);
            if (pos + 3 <= n) { unsigned hv = lzh_hash3(in + pos); prev[pos] = head[hv]; head[hv] = (int32_t)pos; }
            pos += 1;
        } else {
            int ch = 255 - LZH_THRESHOLD + (int)best_len;
            lzh_encode_char(h, &bw, ch);
            int P = (int)(best_dist - 1) & LZH_N_MASK;
            lzh_encode_position(&bw, P);
            size_t end = pos + best_len;
            for (; pos < end; pos++)
                if (pos + 3 <= n) { unsigned hv = lzh_hash3(in + pos); prev[pos] = head[hv]; head[hv] = (int32_t)pos; }
        }
    }
    free(h); free(head); free(prev);
    lzh_bw_flush(&bw);
    *out_len = bw.len;
    return bw.buf;
}

static uint16_t lzh_crc16_arc(const uint8_t *buf, size_t n)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static uint8_t *lzh_pack_archive(const uint8_t *data, size_t n, const char *entry_name, size_t *out_len)
{
    size_t clen;
    uint8_t *comp = lzh_compress(data, n, &clen);
    if (!comp) return NULL;
    uint16_t crc = lzh_crc16_arc(data, n);

    size_t fnlen = strlen(entry_name);
    if (fnlen > 200) fnlen = 200;

    size_t hdr_cap = 22 + fnlen + 2;
    uint8_t *hdr = (uint8_t *)malloc(hdr_cap);
    if (!hdr) { free(comp); return NULL; }

    memcpy(hdr + 2, "-lh1-", 5);
    hdr[7]  = (uint8_t)(clen & 0xFF);       hdr[8]  = (uint8_t)((clen >> 8) & 0xFF);
    hdr[9]  = (uint8_t)((clen >> 16) & 0xFF); hdr[10] = (uint8_t)((clen >> 24) & 0xFF);
    hdr[11] = (uint8_t)(n & 0xFF);           hdr[12] = (uint8_t)((n >> 8) & 0xFF);
    hdr[13] = (uint8_t)((n >> 16) & 0xFF);   hdr[14] = (uint8_t)((n >> 24) & 0xFF);
    hdr[15] = 0; hdr[16] = 0;
    hdr[17] = 0x21; hdr[18] = 0x00;
    hdr[19] = 0x20;
    hdr[20] = 0x00;
    hdr[21] = (uint8_t)fnlen;
    memcpy(hdr + 22, entry_name, fnlen);
    size_t p = 22 + fnlen;
    hdr[p++] = (uint8_t)(crc & 0xFF);
    hdr[p++] = (uint8_t)((crc >> 8) & 0xFF);

    int total_header = (int)p;
    hdr[0] = (uint8_t)(total_header - 2);
    unsigned sum = 0;
    for (int i = 2; i < total_header; i++) sum += hdr[i];
    hdr[1] = (uint8_t)(sum & 0xFF);

    uint8_t *out = (uint8_t *)malloc((size_t)total_header + clen);
    if (!out) { free(hdr); free(comp); return NULL; }
    memcpy(out, hdr, (size_t)total_header);
    memcpy(out + total_header, comp, clen);
    free(hdr); free(comp);

    *out_len = (size_t)total_header + clen;
    return out;
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 1c — SNDH export (uncompressed)
 * ══════════════════════════════════════════════════════════════════════════
 */
static const uint8_t sndh_driver_blob[138] = {
    0x48, 0xe7, 0xff, 0xfe, 0x61, 0x00, 0x00, 0x02, 0x20, 0x5f, 0x41, 0xe8,
    0x00, 0x7a, 0x42, 0x90, 0x4c, 0xdf, 0x7f, 0xff, 0x4e, 0x75, 0x48, 0xe7,
    0xff, 0xfe, 0x28, 0x7c, 0xff, 0xff, 0x88, 0x00, 0x74, 0x08, 0x18, 0x82,
    0x42, 0x2c, 0x00, 0x02, 0x52, 0x02, 0x0c, 0x02, 0x00, 0x0b, 0x6d, 0x00,
    0xff, 0xf2, 0x4c, 0xdf, 0x7f, 0xff, 0x4e, 0x75, 0x48, 0xe7, 0xff, 0xfe,
    0x61, 0x00, 0x00, 0x02, 0x24, 0x5f, 0x45, 0xea, 0x00, 0x42, 0x20, 0x12,
    0x47, 0xea, 0x00, 0x04, 0xb0, 0x93, 0x6d, 0x00, 0x00, 0x04, 0x70, 0x00,
    0x43, 0xea, 0x00, 0x08, 0x22, 0x00, 0xc2, 0xfc, 0x00, 0x0e, 0xd3, 0xc1,
    0x28, 0x7c, 0xff, 0xff, 0x88, 0x00, 0x74, 0x00, 0x18, 0x82, 0x19, 0x59,
    0x00, 0x02, 0x52, 0x02, 0x0c, 0x02, 0x00, 0x0d, 0x6d, 0x00, 0xff, 0xf2,
    0x52, 0x80, 0x24, 0x80, 0x4c, 0xdf, 0x7f, 0xff, 0x4e, 0x75, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#define SNDH_DRIVER_INIT_OFS   0
#define SNDH_DRIVER_EXIT_OFS   22
#define SNDH_DRIVER_PLAY_OFS   56
#define SNDH_DRIVER_NBFRAMES_OFS 134

static void sndh_put32be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

static uint8_t *build_sndh_buffer(uint32_t nb_frames, const char *title,
                                   const char *author, size_t *out_len)
{
    char tags[512];
    size_t tp = 0;
    unsigned secs;

    /* the 68k driver indexes frames with mulu.w: keep the frame count 16-bit */
    if (nb_frames > 65535UL) nb_frames = 65535UL;
    secs = (nb_frames + 49) / 50;
    if (secs == 0) secs = 1;
    if (secs > 65535) secs = 65535;

    memcpy(tags + tp, "SNDH", 4); tp += 4;
#define TAGSTR(name, val) do { \
        memcpy(tags + tp, name, 4); tp += 4; \
        size_t l = strlen(val); if (tp + l + 1 > sizeof(tags)) l = sizeof(tags) - tp - 1; \
        memcpy(tags + tp, val, l); tp += l; tags[tp++] = 0; \
    } while (0)
    TAGSTR("TITL", title);
    TAGSTR("COMM", author);
    TAGSTR("CONV", "mm_vgm2ym");
    memcpy(tags + tp, "##01", 4); tp += 4;
    tags[tp++] = 0;
#undef TAGSTR
    memcpy(tags + tp, "TC50", 4); tp += 4;
    tags[tp++] = 0;
    /* tags start at file offset 12 (even): tp parity == file offset parity */
    if (tp & 1) tags[tp++] = 0;                 /* TIME words on an even address */
    memcpy(tags + tp, "TIME", 4); tp += 4;
    tags[tp++] = (uint8_t)(secs >> 8);          /* one big-endian word per subtune */
    tags[tp++] = (uint8_t)secs;
    memcpy(tags + tp, "HDNS", 4); tp += 4;      /* HDNS must be on an even boundary */

    size_t header_len = 12 + tp;                /* even by construction */
    size_t driver_ofs = header_len;
    size_t data_ofs = driver_ofs + sizeof(sndh_driver_blob);
    size_t data_len = (size_t)nb_frames * 14;
    size_t total = data_ofs + data_len;

    uint8_t *buf = (uint8_t *)malloc(total);
    if (!buf) return NULL;
    memset(buf, 0, total);

#define PATCH_BRA(off, target) do { \
        uint32_t disp = (uint32_t)((target) - ((off) + 2)); \
        buf[off]   = 0x60; buf[(off)+1] = 0x00; \
        buf[(off)+2] = (uint8_t)(disp >> 8); buf[(off)+3] = (uint8_t)disp; \
    } while (0)
    PATCH_BRA(0, driver_ofs + SNDH_DRIVER_INIT_OFS);
    PATCH_BRA(4, driver_ofs + SNDH_DRIVER_EXIT_OFS);
    PATCH_BRA(8, driver_ofs + SNDH_DRIVER_PLAY_OFS);
#undef PATCH_BRA

    memcpy(buf + 12, tags, tp);
    memcpy(buf + driver_ofs, sndh_driver_blob, sizeof(sndh_driver_blob));
    sndh_put32be(buf + driver_ofs + SNDH_DRIVER_NBFRAMES_OFS, nb_frames);

    for (uint32_t i = 0; i < nb_frames; i++) {
        uint8_t *row = buf + data_ofs + (size_t)i * 14;
        memcpy(row, frame_buf + (size_t)i * 16, 14);
        row[7] |= 0xC0;   /* I/O ports A/B must stay outputs on the ST */
    }

    *out_len = total;
    return buf;
}

static int use_lha = 0;
static int output_sndh = 0;

static uint32_t get_uncompressed_size(const char *filename)
{
    FILE *f = fopen(filename, "rb");
    if (!f) return 0;
    uint8_t magic[2] = {0, 0};
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

static int convert_one(const char *src_name, const char *dest_dir_in, char *status, int statussz)
{
    uint32_t target_size = get_uncompressed_size(src_name);
    if (target_size == 0) { snprintf(status, statussz, "%s: cannot read", src_name); return -1; }

    uint8_t *buf = (uint8_t *)malloc(target_size);
    if (!buf) { snprintf(status, statussz, "%s: out of memory", src_name); return -2; }

    gzFile gf = gzopen(src_name, "rb");
    if (!gf) { free(buf); snprintf(status, statussz, "%s: cannot open", src_name); return -3; }
    int bytes_read = gzread(gf, buf, target_size);
    gzclose(gf);

    if (bytes_read < 0x40 || memcmp(buf, "Vgm ", 4) != 0) {
        free(buf); snprintf(status, statussz, "%s: not a VGM", src_name); return -4;
    }

    uint32_t sn_clock = read_le32(&buf[0x0C]) & 0x3FFFFFFFUL;
    if (sn_clock == 0) {
        free(buf); snprintf(status, statussz, "%s: no SN76489 stream", src_name); return -5;
    }

    uint32_t version = read_le32(&buf[0x08]);
    uint32_t data_offset = 0x40;
    if (version >= 0x00000150UL) {
        uint32_t rel = read_le32(&buf[0x34]);
        if (rel != 0) data_offset = rel + 0x34UL;
    }
    if (data_offset >= (uint32_t)bytes_read) {
        free(buf); snprintf(status, statussz, "%s: bad data offset", src_name); return -6;
    }

    uint32_t loop_ofs = read_le32(&buf[0x1C]);
    uint32_t loop_abs = loop_ofs ? (0x1CUL + loop_ofs) : 0;

    init_conv_tables(sn_clock);

    uint32_t loop_frame = 0;
    int nframes = parse_and_convert(buf, (uint32_t)bytes_read, data_offset, loop_abs, &loop_frame);

    if (nframes <= 0) {
        free(buf);
        free(frame_buf); frame_buf = NULL; frame_cap = 0;
        snprintf(status, statussz, "%s: no frames produced", src_name);
        return -7;
    }

    char title[64], author[64], comment[128];
    extract_gd3(buf, (uint32_t)bytes_read, title, sizeof(title), author, sizeof(author),
                comment, sizeof(comment));
    if (title[0]  == '\0') { strncpy(title, src_name, sizeof(title)-1); title[sizeof(title)-1] = '\0'; }
    if (author[0] == '\0') strcpy(author, "Unknown");
    if (comment[0]== '\0') strcpy(comment, "Converted from VGM by mm_vgm2ym");

    char base[64];
    strncpy(base, src_name, sizeof(base)-1); base[sizeof(base)-1] = '\0';
    char *dot = strrchr(base, '.');
    if (dot) *dot = '\0';

    char out_path[192];
    const char *out_ext = output_sndh ? "sndh" : "ym";
    if (dest_dir_in[0] == '\0' || strcmp(dest_dir_in, ".") == 0)
        snprintf(out_path, sizeof(out_path), "%s.%s", base, out_ext);
    else
    {
        size_t dl = strlen(dest_dir_in);
        const char *sep = (dl > 0 && dest_dir_in[dl - 1] == '\\') ? "" : "\\";
        snprintf(out_path, sizeof(out_path), "%s%s%s.%s", dest_dir_in, sep, base, out_ext);
    }

    if (output_sndh) {
        size_t sndh_len;
        uint8_t *sndh_buf = build_sndh_buffer((uint32_t)nframes, title, author, &sndh_len);

        free(buf);
        free(frame_buf); frame_buf = NULL; frame_cap = 0;

        if (!sndh_buf) { snprintf(status, statussz, "%s: out of memory building SNDH", src_name); return -10; }

        FILE *f = fopen(out_path, "wb");
        int wok = f && (fwrite(sndh_buf, 1, sndh_len, f) == sndh_len);
        if (f) fclose(f);
        free(sndh_buf);

        if (!wok) { snprintf(status, statussz, "%s: write failed", out_path); return -11; }
        snprintf(status, statussz, "%s (%d frames, %.1fs)", out_path, nframes, nframes / 50.0);
        return 0;
    }

    size_t ym_len;
    uint8_t *ym_buf = build_ym6_buffer((uint32_t)nframes, loop_frame, title, author, comment, &ym_len);

    free(buf);
    free(frame_buf); frame_buf = NULL; frame_cap = 0;

    if (!ym_buf) { snprintf(status, statussz, "%s: out of memory building YM6", src_name); return -8; }

    char entry_name[80];
    snprintf(entry_name, sizeof(entry_name), "%s.ym", base);

    const uint8_t *final_buf = ym_buf;
    size_t final_len = ym_len;
    uint8_t *packed_buf = NULL;

    if (use_lha) {
        size_t packed_len;
        packed_buf = lzh_pack_archive(ym_buf, ym_len, entry_name, &packed_len);
        if (packed_buf) { final_buf = packed_buf; final_len = packed_len; }
    }

    FILE *f = fopen(out_path, "wb");
    int wok = f && (fwrite(final_buf, 1, final_len, f) == final_len);
    if (f) fclose(f);

    if (use_lha && packed_buf) {
        snprintf(status, statussz, "%s (%d frames, %.0f%% packed)", out_path, nframes,
                 100.0 * (double)final_len / (double)ym_len);
    } else if (use_lha) {
        snprintf(status, statussz, "%s (%d frames, packing failed, kept uncompressed)", out_path, nframes);
    } else {
        snprintf(status, statussz, "%s (%d frames)", out_path, nframes);
    }

    free(ym_buf);
    free(packed_buf);

    if (!wok) { snprintf(status, statussz, "%s: write failed", out_path); return -9; }
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 2 — shared state
 * ══════════════════════════════════════════════════════════════════════════
 */

#define MAX_FILES 256
static char    file_list[MAX_FILES][64];
static uint8_t file_selected[MAX_FILES];
static int     num_files = 0;
static char    dest_dir[128] = ".";

static int aes_apid = -1;

static void scan_files(void)
{
    static char dta[44];
    void *old_dta = (void *)Fgetdta();
    Fsetdta((void *)dta);
    num_files = 0;

    if (Fsfirst("*.vgm", 0) == 0) {
        do {
            snprintf(file_list[num_files], 64, "%s", &dta[30]);
            file_selected[num_files] = 0;
            num_files++;
        } while (Fsnext() == 0 && num_files < MAX_FILES);
    }
    if (num_files < MAX_FILES && Fsfirst("*.vgz", 0) == 0) {
        do {
            snprintf(file_list[num_files], 64, "%s", &dta[30]);
            file_selected[num_files] = 0;
            num_files++;
        } while (Fsnext() == 0 && num_files < MAX_FILES);
    }
    Fsetdta(old_dta);
}

static void select_all(void)  { for (int i = 0; i < num_files; i++) file_selected[i] = 1; }
static void select_none(void) { for (int i = 0; i < num_files; i++) file_selected[i] = 0; }
static int  count_selected(void)
{
    int n = 0;
    for (int i = 0; i < num_files; i++) if (file_selected[i]) n++;
    return n;
}

static void run_conversion(char *status, int statussz, void (*progress)(const char *))
{
    int ok = 0, fail = 0;
    char line[160];

    if (aes_apid >= 0) graf_mouse(BUSY_BEE, NULL);

    for (int i = 0; i < num_files; i++) {
        if (!file_selected[i]) continue;
        if (progress) {
            snprintf(line, sizeof(line), "Converting %s...", file_list[i]);
            progress(line);
        }
        int r = convert_one(file_list[i], dest_dir, line, sizeof(line));
        if (r == 0) ok++; else fail++;
        if (progress) progress(line);
    }

    if (aes_apid >= 0) graf_mouse(ARROW, NULL);

    if (ok + fail == 0)
        snprintf(status, statussz, "Nothing selected.");
    else
        snprintf(status, statussz, "Done: %d converted, %d failed.", ok, fail);
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 3 — GEM/AES windowed UI
 * ══════════════════════════════════════════════════════════════════════════
 */

#ifndef ARROW
#define ARROW    0
#endif
#ifndef BUSY_BEE
#define BUSY_BEE 2
#endif

static short phys_handle, vdi_handle, wi_handle;
static short xdesk, ydesk, wdesk, hdesk;
static short xwork, ywork, wwork, hwork;
static short work_in[11], work_out[57];
static short gem_font_w, gem_font_h, gem_font_wbox, gem_font_hbox;

static int  gem_cursor = 0;
static char gem_status[80] = "Ready.";

static int scroll_y = 0, doc_h = 0, line_height = 0;

/* Chrome size for the FULL layout (exactly as the schema).
 * header: 9 rows, footer: 5 rows. */
static int hdr_rows_full = 9;
static int ftr_rows_full = 5;

/* Medium and minimal fallbacks. */
static int hdr_rows_med  = 5;
static int ftr_rows_med  = 3;
static int hdr_rows_min  = 2;
static int ftr_rows_min  = 2;

/* Currently chosen chrome size (in pixels). */
static int header_h = 0;
static int footer_h = 0;
static int layout_mode = 0;   /* 0=tiny, 1=minimal, 2=medium, 3=full */

static int16_t gem_min_w = 260, gem_min_h = 100;

static int choose_dest_dir(void)
{
    char path[128];
    char fname[64] = "";
    short button = 0;

    if (aes_apid < 0) return -1;

    snprintf(path, sizeof(path), "%s\\*.*", dest_dir);
    fsel_input(path, fname, &button);
    if (button != 1) return -1;

    char *slash = strrchr(path, '\\');
    if (slash) {
        if (slash == path || slash[-1] == ':') slash[1] = '\0';   /* root: keep the backslash */
        else *slash = '\0';
    }
    strncpy(dest_dir, path, sizeof(dest_dir) - 1);
    dest_dir[sizeof(dest_dir) - 1] = '\0';
    return 0;
}

static int gem_update_scrollbars(void)
{
    int lh = line_height > 0 ? line_height : 16;
    int view_h = hwork - header_h - footer_h;
    int max_scroll;
    int v_size, v_pos;

    if (view_h < lh) view_h = lh;

    max_scroll = doc_h - view_h;
    if (max_scroll < 0) max_scroll = 0;

    if (scroll_y < 0) scroll_y = 0;
    if (scroll_y > max_scroll) scroll_y = max_scroll;
    scroll_y = (scroll_y / lh) * lh;
    if (scroll_y > max_scroll) scroll_y = (max_scroll / lh) * lh;

    if (doc_h > view_h) {
        v_size = (view_h * 1000) / doc_h;
        if (v_size < 20) v_size = 20;
        v_pos = (scroll_y * 1000) / (doc_h - view_h);
    } else {
        v_size = 1000;
        v_pos = 0;
    }

    wind_set(wi_handle, WF_VSLSIZE, v_size, 0, 0, 0);
    wind_set(wi_handle, WF_VSLIDE,  v_pos,  0, 0, 0);
    return 0;
}

/* Choose the biggest layout that still leaves at least 3 file rows visible. */
static void gem_recompute_layout(void)
{
    int lh = line_height > 0 ? line_height : 16;
    int avail_rows;
    int min_list_rows = 3;

    if (hwork <= 0) hwork = lh;
    avail_rows = hwork / lh;

    if (avail_rows >= hdr_rows_full + ftr_rows_full + min_list_rows) {
        layout_mode = 3;
        header_h = hdr_rows_full * lh;
        footer_h = ftr_rows_full * lh;
    } else if (avail_rows >= hdr_rows_med + ftr_rows_med + min_list_rows) {
        layout_mode = 2;
        header_h = hdr_rows_med * lh;
        footer_h = ftr_rows_med * lh;
    } else if (avail_rows >= hdr_rows_min + ftr_rows_min + min_list_rows) {
        layout_mode = 1;
        header_h = hdr_rows_min * lh;
        footer_h = ftr_rows_min * lh;
    } else {
        layout_mode = 0;
        header_h = lh;
        footer_h = lh;
    }

    doc_h = num_files * lh;
    gem_update_scrollbars();
}

static void gem_open_vwork(void)
{
    for (int i = 0; i < 10; i++) work_in[i] = 1;
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
    int16_t win_w = 420, win_h = 300;
    int16_t win_x, win_y;

    if (win_w > wdesk - 20) win_w = wdesk - 20;
    if (win_h > hdesk - 20) win_h = hdesk - 20;
    if (win_w < 260) win_w = 260;
    if (win_h < 100) win_h = 100;

    win_x = xdesk + (wdesk - win_w) / 2;
    win_y = ydesk + 10;
    if (win_y + win_h > ydesk + hdesk) win_y = ydesk + hdesk - win_h;
    if (win_y < ydesk) win_y = ydesk;

    wi_handle = wind_create(wi_kind, xdesk, ydesk, wdesk, hdesk);
    wind_set_str(wi_handle, WF_NAME, " mm_vgm2ym ");
    wind_open(wi_handle, win_x, win_y, win_w, win_h);
    wind_get(wi_handle, WF_WORKXYWH, &xwork, &ywork, &wwork, &hwork);

    line_height = gem_font_hbox;
    if (line_height < 8) line_height = 16;

    gem_min_w = 260;
    gem_min_h = 100;

    gem_recompute_layout();
}

/* ---- The full-layout header, exactly as the schema ------------------- */
static void draw_header_full(int wx, int wy, int lh, int *out_ty)
{
    char buf[200];
    int ty = wy + lh - 4;

    v_gtext(vdi_handle, wx + 4, ty, (char *)"=====================================");
    ty += lh;
    v_gtext(vdi_handle, wx + 4, ty, (char *)"mm_vgm2ym v1.0 | VGM->YM6 Converter");
    ty += lh;
    v_gtext(vdi_handle, wx + 4, ty, (char *)"=====================================");
    ty += lh;

    /* "[D]est : %-7s [F]mt : %s"  ->  matches "[D]est : .       [F]mt : .YM" */
    snprintf(buf, sizeof(buf), "[D]est : %-7s [F]mt : %s",
             dest_dir, output_sndh ? ".SNDH" : ".YM");
    v_gtext(vdi_handle, wx + 4, ty, buf);
    ty += lh;

    snprintf(buf, sizeof(buf), "[L]HA  : %s", use_lha ? "ON" : "OFF");
    v_gtext(vdi_handle, wx + 4, ty, buf);
    ty += lh;

    v_gtext(vdi_handle, wx + 4, ty, (char *)"-------------------------------------");
    ty += lh;

    snprintf(buf, sizeof(buf), "%d file(s) found, %d selected",
             num_files, count_selected());
    v_gtext(vdi_handle, wx + 4, ty, buf);
    ty += lh;

    v_gtext(vdi_handle, wx + 4, ty, (char *)"UP/DN=Move SPACE=Tgl A=All N=None");
    ty += lh;

    /* blank line before file list */
    ty += lh;

    *out_ty = ty;
}

/* ---- The full-layout footer, exactly as the schema ------------------- */
static void draw_footer_full(int wx, int fy, int lh)
{
    char buf[200];

    /* blank line */
    fy += lh;

    v_gtext(vdi_handle, wx + 4, fy, (char *)"-------------------------------------");
    fy += lh;

    v_gtext(vdi_handle, wx + 4, fy, (char *)"[ENTER/C]=Convert   [Q]=Quit");
    fy += lh;

    snprintf(buf, sizeof(buf), "%s", gem_status);
    v_gtext(vdi_handle, wx + 4, fy, buf);
    fy += lh;

    v_gtext(vdi_handle, wx + 4, fy, (char *)"=====================================");
}

/* ---- Medium header --------------------------------------------------- */
static void draw_header_medium(int wx, int wy, int lh)
{
    char buf[200];
    int ty = wy + lh - 4;

    v_gtext(vdi_handle, wx + 4, ty, (char *)"=====================================");
    ty += lh;
    v_gtext(vdi_handle, wx + 4, ty, (char *)"mm_vgm2ym v1.0 | VGM->YM6 Converter");
    ty += lh;

    snprintf(buf, sizeof(buf), "[D]est:%-8s [F]mt:%-5s [L]HA:%s",
             dest_dir, output_sndh ? ".SNDH" : ".YM", use_lha ? "ON" : "OFF");
    v_gtext(vdi_handle, wx + 4, ty, buf);
    ty += lh;

    snprintf(buf, sizeof(buf), "%d found, %d sel  SPACE=tgl A=All N=None",
             num_files, count_selected());
    v_gtext(vdi_handle, wx + 4, ty, buf);
    ty += lh;
    /* blank */
}

static void draw_footer_medium(int wx, int fy, int lh)
{
    char buf[200];

    v_gtext(vdi_handle, wx + 4, fy, (char *)"[ENTER/C]=Conv  [Q]=Quit");
    fy += lh;
    snprintf(buf, sizeof(buf), "%s", gem_status);
    v_gtext(vdi_handle, wx + 4, fy, buf);
    fy += lh;
    v_gtext(vdi_handle, wx + 4, fy, (char *)"=====================================");
}

/* ---- Minimal header / footer ---------------------------------------- */
static void draw_header_minimal(int wx, int wy, int lh)
{
    char buf[200];
    int ty = wy + lh - 4;

    v_gtext(vdi_handle, wx + 4, ty, (char *)"mm_vgm2ym | VGM->YM6 Converter");
    ty += lh;
    snprintf(buf, sizeof(buf), "Dest:%s Fmt:%s LHA:%s",
             dest_dir, output_sndh ? ".SNDH" : ".YM", use_lha ? "ON" : "OFF");
    v_gtext(vdi_handle, wx + 4, ty, buf);
}

static void draw_footer_minimal(int wx, int fy, int lh)
{
    char buf[200];

    snprintf(buf, sizeof(buf), "%s", gem_status);
    v_gtext(vdi_handle, wx + 4, fy, buf);
    fy += lh;
    v_gtext(vdi_handle, wx + 4, fy, (char *)"C=Conv Q=Quit");
}

static void gem_draw_ui(void)
{
    char buf[200];
    int lh = line_height > 0 ? line_height : 16;
    int wx = xwork, wy = ywork, ww = wwork, wh = hwork;
    int list_top = wy + header_h;
    int list_bottom = wy + wh - footer_h;
    int list_h = list_bottom - list_top;

    vsf_color(vdi_handle, 0);
    {
        int16_t bg[4];
        bg[0] = wx; bg[1] = wy; bg[2] = wx + ww - 1; bg[3] = wy + wh - 1;
        v_bar(vdi_handle, bg);
    }
    vst_color(vdi_handle, 1);

    /* ── Header ──────────────────────────────────────────────── */
    if (layout_mode == 3) {
        int dummy;
        draw_header_full(wx, wy, lh, &dummy);
    } else if (layout_mode == 2) {
        draw_header_medium(wx, wy, lh);
    } else if (layout_mode == 1) {
        draw_header_minimal(wx, wy, lh);
    } else {
        draw_header_minimal(wx, wy, lh);
    }

    /* ── File list ───────────────────────────────────────────── */
    if (list_h >= lh) {
        if (num_files == 0) {
            vst_color(vdi_handle, 1);
            v_gtext(vdi_handle, wx + 4, list_top + lh - 4,
                    (char *)"  No .vgm or .vgz files found.");
        } else {
            for (int i = 0; i < num_files; i++) {
                int cell_top = list_top + i * lh - scroll_y;
                int cell_bottom = cell_top + lh;
                int baseline;

                if (cell_bottom <= list_top) continue;
                if (cell_top >= list_bottom) break;

                baseline = cell_top + lh - 4;

                /* Format: "> [ ] 01 - NAME" for cursor row,
                 *         "  [ ] 01 - NAME" for others. */
                snprintf(buf, sizeof(buf), "%c [%c] %02d - %s",
                         (i == gem_cursor) ? '>' : ' ',
                         file_selected[i] ? 'x' : ' ',
                         i + 1, file_list[i]);

                if (i == gem_cursor) {
                    int16_t bar[4];
                    vsf_color(vdi_handle, 1);
                    bar[0] = wx + 2;
                    bar[1] = cell_top + 2;
                    bar[2] = wx + ww - 2;
                    bar[3] = cell_bottom - 2;
                    v_bar(vdi_handle, bar);
                    vst_color(vdi_handle, 0);
                } else {
                    vst_color(vdi_handle, 1);
                }
                v_gtext(vdi_handle, wx + 4, baseline, buf);
            }
            vst_color(vdi_handle, 1);
        }
    }

    /* ── Footer ──────────────────────────────────────────────── */
    if (layout_mode == 3) {
        /* The full footer in the schema starts with a blank line; that
         * blank is drawn as the first row of the footer band. */
        draw_footer_full(wx, list_bottom, lh);
    } else if (layout_mode == 2) {
        draw_footer_medium(wx, list_bottom, lh);
    } else if (layout_mode == 1) {
        draw_footer_minimal(wx, list_bottom + lh - 4, lh);
    } else {
        draw_footer_minimal(wx, list_bottom + lh - 4, lh);
    }
}

static void gem_force_redraw(void)
{
    GRECT rect;
    int16_t clip[4];
    wind_update(BEG_UPDATE);
    graf_mouse(M_OFF, NULL);
    wind_get(wi_handle, WF_FIRSTXYWH, &rect.g_x, &rect.g_y, &rect.g_w, &rect.g_h);
    while (rect.g_w > 0 && rect.g_h > 0) {
        clip[0] = rect.g_x;
        clip[1] = rect.g_y;
        clip[2] = rect.g_x + rect.g_w - 1;
        clip[3] = rect.g_y + rect.g_h - 1;
        vs_clip(vdi_handle, 1, clip);
        gem_draw_ui();
        wind_get(wi_handle, WF_NEXTXYWH, &rect.g_x, &rect.g_y, &rect.g_w, &rect.g_h);
    }
    vs_clip(vdi_handle, 0, clip);
    graf_mouse(M_ON, NULL);
    wind_update(END_UPDATE);
}

static void gem_set_status(const char *s)
{
    strncpy(gem_status, s, sizeof(gem_status) - 1);
    gem_status[sizeof(gem_status) - 1] = '\0';
    gem_force_redraw();
}

static void gem_main_loop(void)
{
    int16_t msg[8];
    int16_t mx, my, mb, kstat, key, br;
    int quit = 0;

    gem_force_redraw();

    while (!quit) {
        int16_t ev = evnt_multi(MU_KEYBD | MU_MESAG,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 msg, 0, &mx, &my, &mb, &kstat, &key, &br);

        if (ev & MU_MESAG) {
            if (msg[3] != wi_handle) continue;
            switch (msg[0]) {
            case WM_REDRAW:
                gem_force_redraw();
                break;
            case WM_TOPPED:
                wind_set(wi_handle, WF_TOP, 0, 0, 0, 0);
                break;
            case WM_CLOSED:
                quit = 1;
                break;
            case WM_MOVED:
                wind_set(wi_handle, WF_CURRXYWH, msg[4], msg[5], msg[6], msg[7]);
                wind_get(wi_handle, WF_WORKXYWH, &xwork, &ywork, &wwork, &hwork);
                gem_recompute_layout();
                gem_force_redraw();
                break;
            case WM_SIZED:
                if (msg[4] < xdesk) msg[4] = xdesk;
                if (msg[5] < ydesk) msg[5] = ydesk;
                if (msg[6] < gem_min_w) msg[6] = gem_min_w;
                if (msg[7] < gem_min_h) msg[7] = gem_min_h;
                if (msg[4] + msg[6] > xdesk + wdesk) msg[4] = xdesk + wdesk - msg[6];
                if (msg[5] + msg[7] > ydesk + hdesk) msg[5] = ydesk + hdesk - msg[7];
                wind_set(wi_handle, WF_CURRXYWH, msg[4], msg[5], msg[6], msg[7]);
                wind_get(wi_handle, WF_WORKXYWH, &xwork, &ywork, &wwork, &hwork);
                gem_recompute_layout();
                gem_force_redraw();
                break;
            case WM_ARROWED:
                switch (msg[4]) {
                case WA_UPLINE: scroll_y -= line_height; break;
                case WA_DNLINE: scroll_y += line_height; break;
                case WA_UPPAGE: scroll_y -= (hwork - header_h - footer_h); break;
                case WA_DNPAGE: scroll_y += (hwork - header_h - footer_h); break;
                }
                gem_update_scrollbars();
                gem_force_redraw();
                break;
            case WM_VSLID:
                {
                    int view_h = hwork - header_h - footer_h;
                    if (view_h <= 0) view_h = line_height;
                    if (doc_h > view_h)
                        scroll_y = (msg[4] * (doc_h - view_h)) / 1000;
                    else
                        scroll_y = 0;
                    gem_update_scrollbars();
                    gem_force_redraw();
                }
                break;
            }
        }

        if (ev & MU_KEYBD) {
            char ascii = key & 0xFF;
            char scan  = (key >> 8) & 0xFF;

            if (ascii == 'q' || ascii == 'Q' || ascii == 27) {
                quit = 1;
            } else if (scan == 0x48) {
                if (gem_cursor > 0) {
                    gem_cursor--;
                    int cursor_top = gem_cursor * line_height;
                    if (cursor_top < scroll_y) scroll_y = cursor_top;
                    gem_update_scrollbars();
                    gem_force_redraw();
                }
            } else if (scan == 0x50) {
                if (gem_cursor < num_files - 1) {
                    gem_cursor++;
                    int view_h = hwork - header_h - footer_h;
                    if (view_h <= 0) view_h = line_height;
                    int cursor_bottom = (gem_cursor + 1) * line_height;
                    if (cursor_bottom > scroll_y + view_h)
                        scroll_y = cursor_bottom - view_h;
                    gem_update_scrollbars();
                    gem_force_redraw();
                }
            } else if (ascii == ' ') {
                if (num_files > 0 && gem_cursor >= 0 && gem_cursor < num_files) {
                    file_selected[gem_cursor] = !file_selected[gem_cursor];
                    gem_force_redraw();
                }
            } else if (ascii == 'a' || ascii == 'A') {
                select_all();
                strcpy(gem_status, "All files selected.");
                gem_force_redraw();
            } else if (ascii == 'n' || ascii == 'N') {
                select_none();
                strcpy(gem_status, "Selection cleared.");
                gem_force_redraw();
            } else if (ascii == 'd' || ascii == 'D') {
                if (choose_dest_dir() == 0)
                    snprintf(gem_status, sizeof(gem_status), "Destination: %s", dest_dir);
                else
                    strcpy(gem_status, "Destination unchanged.");
                gem_force_redraw();
            } else if (ascii == 'l' || ascii == 'L') {
                if(output_sndh == 0) {
                    use_lha = !use_lha;
                }
                snprintf(gem_status, sizeof(gem_status), "LHA pack %s", use_lha ? "ON" : "OFF");
                gem_force_redraw();
            } else if (ascii == 'f' || ascii == 'F') {
                output_sndh = !output_sndh;
                snprintf(gem_status, sizeof(gem_status), "Output format: %s", output_sndh ? ".SNDH" : ".YM");
                if(output_sndh && use_lha) {
                    use_lha = 0;
                    strcat(gem_status, " (LHA disabled for SNDH)");
                }
                gem_force_redraw();
            } else if (ascii == 13 || ascii == 'c' || ascii == 'C') {
                run_conversion(gem_status, sizeof(gem_status), gem_set_status);
                gem_force_redraw();
            }
        }
    }
}

static int main_gem(int argc, char **argv)
{
    (void)argc; (void)argv;

    phys_handle = graf_handle(&gem_font_w, &gem_font_h, &gem_font_wbox, &gem_font_hbox);
    wind_get(0, WF_WORKXYWH, &xdesk, &ydesk, &wdesk, &hdesk);
    gem_open_vwork();

    scan_files();
    gem_cursor = 0;
    scroll_y = 0;
    strcpy(gem_status, "Ready.");

    gem_open_window();
    graf_mouse(ARROW, NULL);
    gem_main_loop();

    wind_close(wi_handle);
    wind_delete(wi_handle);
    v_clsvwk(vdi_handle);

    {
        int16_t x, y, w, h;
        wind_get(0, WF_WORKXYWH, &x, &y, &w, &h);
        form_dial(FMD_FINISH, 0, 0, 0, 0, x, y, w, h);
    }

    appl_exit();
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 4 — Plain TOS console UI, same schema
 * ══════════════════════════════════════════════════════════════════════════
 */

static void tos_progress(const char *line)
{
    printf("%s\n", line);
}

static void draw_tos_menu(int cursor, const char *status)
{
    const int max_show = 12;
    int start = 0;

    if (num_files > max_show) {
        start = cursor - max_show / 2;
        if (start < 0) start = 0;
        if (start + max_show > num_files) start = num_files - max_show;
    }

    printf("\033E\033f");

    printf("=====================================\n");
    printf("mm_vgm2ym v1.0 | VGM->YM6 Converter\n");
    printf("=====================================\n");
    printf("[D]est : %-7s [F]mt : %s\n",
           dest_dir, output_sndh ? ".SNDH" : ".YM");
    printf("[L]HA  : %s\n", use_lha ? "ON" : "OFF");
    printf("-------------------------------------\n");
    printf("%d file(s) found, %d selected\n", num_files, count_selected());
    printf("UP/DN=Move SPACE=Tgl A=All N=None\n");
    printf("\n");

    if (num_files == 0) {
        printf("  No .vgm or .vgz files found.\n");
    } else {
        for (int i = start; i < start + max_show && i < num_files; i++) {
            if (i == cursor) printf("\033p");
            printf("%c [%c] %02d - %s",
                   (i == cursor) ? '>' : ' ',
                   file_selected[i] ? 'x' : ' ',
                   i + 1, file_list[i]);
            if (i == cursor) printf("\033q");
            printf("\n");
        }
    }

    printf("\n");
    printf("-------------------------------------\n");
    printf("[ENTER/C]=Convert   [Q]=Quit\n");
    printf("%s\n", status ? status : "Ready.");
    printf("=====================================\n");
}

static int main_tos(int argc, char **argv)
{
    (void)argc; (void)argv;
    scan_files();
    int cursor = 0;
    char status[160] = "Ready.";

    draw_tos_menu(cursor, status);

    while (1) {
        long key = Cconin();
        char ascii = key & 0xFF;
        char scan  = (key >> 16) & 0xFF;

        if (ascii == 'q' || ascii == 'Q' || ascii == 27) {
            break;
        } else if (scan == 0x48) {
            if (cursor > 0) cursor--;
            draw_tos_menu(cursor, status);
        } else if (scan == 0x50) {
            if (cursor < num_files - 1) cursor++;
            draw_tos_menu(cursor, status);
        } else if (ascii == ' ') {
            if (num_files > 0) file_selected[cursor] = !file_selected[cursor];
            draw_tos_menu(cursor, status);
        } else if (ascii == 'a' || ascii == 'A') {
            select_all();
            strcpy(status, "All files selected.");
            draw_tos_menu(cursor, status);
        } else if (ascii == 'n' || ascii == 'N') {
            select_none();
            strcpy(status, "Selection cleared.");
            draw_tos_menu(cursor, status);
        } else if (ascii == 'd' || ascii == 'D') {
            printf("\nEnter destination path (blank = current dir): ");
            fflush(stdout);
            {
                char line[128];
                if (fgets(line, sizeof(line), stdin)) {
                    int len = strlen(line);
                    while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) line[--len] = '\0';
                    if (len > 0) {
                        strncpy(dest_dir, line, sizeof(dest_dir) - 1);
                        dest_dir[sizeof(dest_dir) - 1] = '\0';
                    } else {
                        strcpy(dest_dir, ".");
                    }
                }
            }
            snprintf(status, sizeof(status), "Destination: %s", dest_dir);
            draw_tos_menu(cursor, status);
        } else if (ascii == 'l' || ascii == 'L') {
            use_lha = !use_lha;
            snprintf(status, sizeof(status), "LHA pack %s", use_lha ? "ON" : "OFF");
            draw_tos_menu(cursor, status);
        } else if (ascii == 'f' || ascii == 'F') {
            output_sndh = !output_sndh;
            snprintf(status, sizeof(status), "Output format: %s", output_sndh ? ".SNDH" : ".YM");
            draw_tos_menu(cursor, status);
        } else if (ascii == 13 || ascii == 'c' || ascii == 'C') {
            printf("\n");
            run_conversion(status, sizeof(status), tos_progress);
            draw_tos_menu(cursor, status);
        }
    }
    return 0;
}

/* ══════════════════════════════════════════════════════════════════════════
 * ENTRY POINT
 * ══════════════════════════════════════════════════════════════════════════
 */
int main(int argc, char **argv)
{
    init_skip_table();

    aes_apid = appl_init();
    if (aes_apid >= 0) {
        return main_gem(argc, argv);
    }
    return main_tos(argc, argv);
}