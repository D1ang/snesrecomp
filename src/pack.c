/*
 * Asset pack instead of the ROM file.
 *
 * snesrecomp_load_rom() given a directory with a pack.idx (written by the
 * game's asset extractor) builds the ROM image from the pack's files:
 *
 *   size <ROM size in bytes>
 *   <ROM offset, hex> <length> <file, relative to the directory>
 *   ...
 *
 * Bytes no file covers stay zero. Which bytes the pack has is kept, so the
 * ROM access logger (romlog.c) can report every read outside the pack: the
 * parts of the game the pack does not cover yet.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static uint8_t *s_have;
static uint32_t s_have_size;

const uint8_t *pack_have(uint32_t *size) {
    if (size) *size = s_have_size;
    return s_have;
}

/* path is a pack when path/pack.idx opens. */
bool pack_load(const char *path, uint8_t **out, long *out_size) {
    char idx[1024];
    snprintf(idx, sizeof idx, "%s/pack.idx", path);
    FILE *f = fopen(idx, "r");
    if (!f) return false;
    char line[1200];
    long size = 0;
    if (!fgets(line, sizeof line, f) || sscanf(line, "size %ld", &size) != 1 || size <= 0 || size > (16 << 20)) {
        fprintf(stderr, "snesrecomp: %s: no size line\n", idx);
        fclose(f);
        return false;
    }
    uint8_t *data = calloc((size_t)size, 1);
    s_have = calloc((size_t)size, 1);
    s_have_size = (uint32_t)size;
    long files = 0, bytes = 0;
    while (fgets(line, sizeof line, f)) {
        unsigned long off, len;
        char name[1024];
        if (sscanf(line, "%lx %lu %1023s", &off, &len, name) != 3) continue;
        if (off + len > (unsigned long)size) continue;
        char p[2100];
        snprintf(p, sizeof p, "%s/%s", path, name);
        FILE *b = fopen(p, "rb");
        if (!b) { fprintf(stderr, "snesrecomp: pack file missing: %s\n", p); continue; }
        size_t got = fread(data + off, 1, len, b);
        fclose(b);
        memset(s_have + off, 1, got);
        files++;
        bytes += (long)got;
    }
    fclose(f);
    fprintf(stderr, "snesrecomp: ROM image from the pack %s: %ld files, %ld of %ld bytes\n",
            path, files, bytes, size);
    *out = data;
    *out_size = size;
    return true;
}
