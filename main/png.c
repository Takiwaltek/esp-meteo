#include "png.h"
#include <string.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "rom/miniz.h"

static const char *TAG = "png";

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }

bool png_decode_rgb565(const uint8_t *png, size_t len, uint16_t *out, int max_w, int max_h, int *ow, int *oh)
{
    static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (len < 33 || memcmp(png, sig, 8)) { ESP_LOGE(TAG, "signature invalide"); return false; }

    uint32_t w = 0, h = 0; uint8_t depth = 0, ctype = 0, interlace = 0;
    uint8_t pal[256][3]; memset(pal, 0, sizeof pal);
    uint8_t *idat = heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
    size_t idat_len = 0;
    uint8_t *raw = NULL;
    bool ok = false;
    if (!idat) return false;

    for (size_t pos = 8; pos + 12 <= len;) {
        uint32_t clen = be32(png + pos);
        const uint8_t *type = png + pos + 4, *data = png + pos + 8;
        if (pos + 12 + (size_t)clen > len) break;
        if (!memcmp(type, "IHDR", 4) && clen >= 13) {
            w = be32(data); h = be32(data + 4); depth = data[8]; ctype = data[9]; interlace = data[12];
        } else if (!memcmp(type, "PLTE", 4)) {
            for (uint32_t i = 0; i < clen / 3 && i < 256; i++) memcpy(pal[i], data + 3 * i, 3);
        } else if (!memcmp(type, "IDAT", 4)) {
            memcpy(idat + idat_len, data, clen); idat_len += clen;
        } else if (!memcmp(type, "IEND", 4)) break;
        pos += 12 + clen;
    }
    int bpp;
    switch (ctype) { case 0: case 3: bpp = 1; break; case 2: bpp = 3; break; case 6: bpp = 4; break; default: bpp = 0; }
    if (!bpp || depth != 8 || interlace || !w || !h || (int)w > max_w || (int)h > max_h) {
        ESP_LOGE(TAG, "format non supporté (type %d, prof. %d, %lux%lu)", ctype, depth, (unsigned long)w, (unsigned long)h);
        goto end;
    }
    size_t stride = w * bpp, raw_len = (stride + 1) * h;
    raw = heap_caps_malloc(raw_len, MALLOC_CAP_SPIRAM);
    if (!raw) goto end;

    size_t in_len = idat_len, out_len = raw_len;
    tinfl_decompressor d; tinfl_init(&d);
    tinfl_status st = tinfl_decompress(&d, idat, &in_len, raw, raw, &out_len,
                                       TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    if (st != TINFL_STATUS_DONE || out_len != raw_len) { ESP_LOGE(TAG, "inflate: %d", st); goto end; }

    for (uint32_t y = 0; y < h; y++) {
        uint8_t *row = raw + y * (stride + 1);
        uint8_t f = row[0]; row++;
        const uint8_t *prev = y ? row - (stride + 1) : NULL;
        for (size_t i = 0; i < stride; i++) {
            int a = i >= (size_t)bpp ? row[i - bpp] : 0;
            int b = prev ? prev[i] : 0;
            int c = (prev && i >= (size_t)bpp) ? prev[i - bpp] : 0;
            int v = row[i];
            switch (f) {
            case 1: v += a; break;
            case 2: v += b; break;
            case 3: v += (a + b) >> 1; break;
            case 4: { int p = a + b - c, pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
                      v += (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c); break; }
            default: break;
            }
            row[i] = (uint8_t)v;
        }
        uint16_t *o = out + y * w;
        for (uint32_t x = 0; x < w; x++) {
            const uint8_t *p = row + x * bpp;
            if (ctype == 0) o[x] = rgb565(p[0], p[0], p[0]);
            else if (ctype == 3) o[x] = rgb565(pal[p[0]][0], pal[p[0]][1], pal[p[0]][2]);
            else o[x] = rgb565(p[0], p[1], p[2]);
        }
    }
    *ow = w; *oh = h; ok = true;
end:
    free(idat); free(raw);
    return ok;
}
