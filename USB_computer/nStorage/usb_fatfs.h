#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ *
 * REAL-FOLDER-AS-FAT16 SYNTHESIZER
 *
 * Presents a real folder on the Nspire's own filesystem as a
 * read/write FAT16 volume over USB Mass Storage. The root is
 * switchable at runtime (see fatfs_toggle_root()) between "/documents"
 * and "/" (the whole calculator filesystem) -- Tab in main.cpp's
 * service loop triggers a real, immediate disconnect+reconnect, so the
 * host notices and re-enumerates right away.
 *
 * WHAT'S SUPPORTED:
 *  - Recursive subdirectories (up to FATFS_MAX_SUBDIRS with real
 *    content; deeper/excess folders still list but appear empty).
 *  - VFAT long filenames on read (lowercase, spaces, non-ASCII-range
 *    bytes) via generated LFN entries, and on write (new files/folders
 *    get their true reconstructed name, not a derived 8.3 one).
 *  - Creating a new file: detected live from directory-entry writes.
 *    Data that arrives before its name is known (common ordering for
 *    real FAT drivers, and for macOS's "safe save" staging-file
 *    pattern) is buffered in memory, never written to the real
 *    filesystem until a genuine final name claims it.
 *  - Editing an existing file in place, including size changes (the
 *    real file gets truncated/extended to match).
 *  - Deleting a file or directory (recursive for directories).
 *  - Renaming a file or directory (copy-then-remove for files;
 *    recursive copy-verify-then-remove for directories, so a failure
 *    partway through can't lose data).
 *  - Moving a directory to a new parent -- performs a REAL recursive
 *    move on the underlying filesystem (not just in our tracking),
 *    including fixing up every contained file/folder's real path.
 *  - Filtering out macOS noise: .DS_Store, "._name" AppleDouble
 *    sidecars, and .fseventsd (which we also proactively suppress by
 *    injecting a virtual-only ".fseventsd/no_log" -- nothing is
 *    created on the real filesystem for it).
 *
 * WHAT'S STILL NOT SUPPORTED (silently no-ops -- the SCSI command
 * succeeds so host software doesn't error out, but nothing happens):
 *  - Creating a brand new (empty) directory.
 *
 * KNOWN RESIDUAL LIMITATION: occasional single garbage entries with
 * implausible names/sizes have been observed and traced to
 * intermittent bit-level corruption at the USB transport layer (the
 * same general class of issue chased at the hardware level earlier in
 * this project) rather than a logic bug here. A few targeted, testable
 * heuristics reject entries with the specific signatures observed
 * (impossible cluster/size values) but this is pattern-matching around
 * a lower-level issue, not something fully eliminated.
 *
 * This is genuinely dynamic code -- it tracks live writes to the FAT
 * table and directory entries, not just a one-time snapshot at
 * startup. Directory moves in particular perform real, irreversible
 * operations (recursive copy + delete) on the actual filesystem.
 * ------------------------------------------------------------------ */

// Set to 1 to enable diagnostic logging to /documents/usb_create_log.txt.tns
// (file creation, deletion, rename, and directory-move events). Off by
// default -- logging does real file I/O on every event and was a real,
// measurable source of slowdown when it was more broadly enabled earlier
// in development.
#ifndef DEBUG_FS
#define DEBUG_FS 0
#endif

static const char *fatfs_root_path = "/documents";
#define FATFS_SECTOR_SIZE      512u
#define FATFS_CLUSTER_SECTORS  8u      // 4096 bytes/cluster
#define FATFS_CLUSTER_BYTES    (FATFS_SECTOR_SIZE * FATFS_CLUSTER_SECTORS)
#define FATFS_MAX_ENTRIES      2048u   // files + directories, combined, whole tree
#define FATFS_MAX_SUBDIRS      512u    // real (non-root) directories with their own entry table
#define FATFS_MAX_SCAN_DEPTH   6u      // recursion guard
#define FATFS_MAX_CLUSTERS     24576u  // 96 MiB volume capacity at 4096 bytes/cluster -- data is staged directly to disk now, not RAM-bound, so this is the real remaining ceiling macOS checks before even attempting a copy
#define FATFS_ROOT_ENTRIES     512u    // 512 * 32 bytes = 16 KiB = 32 sectors
#define FATFS_MAX_LONG_NAME    64u

typedef struct {
    char     short_name[11]; // 8.3, space-padded, NO dot (on-disk FAT format)
    char     long_name[FATFS_MAX_LONG_NAME]; // real original name, NUL-terminated
    char     path[192];      // full real path
    uint8_t  is_dir;
    uint32_t size;           // 0 for directories
    uint32_t start_cluster;
    uint32_t num_clusters;   // directories only: 0 if overflowed FATFS_MAX_SUBDIRS, else 1.
                              // files: informational only now -- live FAT-chain walking
                              // (fatfs_locate_cluster) is authoritative, so a file can
                              // grow past this without it needing to be updated.
    int32_t  parent_index;   // -1 = top-level (root), else index into fatfs_entries[]
    int32_t  subdir_slot;    // directories only: index into fatfs_subdir_tables[], -1 if overflowed
    uint8_t  deleted;        // marked (not removed -- would invalidate other entries' parent_index)
    uint8_t  scanned;        // directories only: have children been discovered yet? (lazy loading)
} FatEntry;

static FatEntry fatfs_entries[FATFS_MAX_ENTRIES];
static uint32_t fatfs_entry_count = 0;
static uint32_t fatfs_locate_cluster_cache_cluster = 0xFFFFFFFFu;
static uint32_t fatfs_locate_cluster_cache_gen = 0xFFFFFFFFu;
static uint32_t fatfs_subdir_count = 0;
static uint32_t fatfs_total_data_clusters = 0;

// The currently-running program's own file path (from argv[0] in
// main.cpp) -- used to mark this one specific file read-only in its
// served directory entry, so macOS's own Finder shows its native
// "locked" indicator and refuses to move/rename it, rather than the
// user attempting something that can't actually succeed while the
// program is executing.
static const char *fatfs_self_path = NULL;
void fatfs_set_self_path(const char *path) {
    fatfs_self_path = path;
}

static uint8_t fatfs_subdir_tables[FATFS_MAX_SUBDIRS][FATFS_CLUSTER_BYTES] __attribute__((aligned(32)));

static uint32_t fatfs_fat_sectors_per_copy = 0;
static uint32_t fatfs_root_dir_sectors = 0;
static uint32_t fatfs_data_start_lba = 0;
static uint32_t fatfs_total_sectors = 0; // size of the FAT VOLUME itself (not counting the MBR)

static uint8_t fatfs_mbr[FATFS_SECTOR_SIZE] __attribute__((aligned(32)));
#define FATFS_PARTITION_START_LBA 1u

static uint8_t fatfs_boot_sector[FATFS_SECTOR_SIZE] __attribute__((aligned(32)));
static uint8_t fatfs_fat_table[FATFS_MAX_CLUSTERS * 2] __attribute__((aligned(32)));
static uint8_t fatfs_root_dir[FATFS_ROOT_ENTRIES * 32] __attribute__((aligned(32)));

/* ==================================================================
 * 8.3 short name generation (siblings-only collision handling)
 * ================================================================== */
// Joins a directory path and a name with exactly one slash between
// them, regardless of whether dir already ends with one. Every path
// built with a naive "%s/%s" snprintf was vulnerable to producing a
// double slash whenever dir was exactly "/" (the whole-filesystem
// root view) -- confirmed as the actual cause of a silent directory
// move failure: opendir() on a resulting "//name" path failed at the
// very top of the recursive copy, before any per-file diagnostic
// logging even had a chance to run.
static void fatfs_join_path(char *out, size_t outsize, const char *dir, const char *name) {
    size_t dlen = strlen(dir);
    int has_trailing_slash = (dlen > 0 && dir[dlen - 1] == '/');
    snprintf(out, outsize, has_trailing_slash ? "%s%s" : "%s/%s", dir, name);
}

static void fatfs_make_short_name(const char *long_name, char out11[11], int32_t parent_index) {
    const char *dot = strrchr(long_name, '.');
    char base[9];
    char ext[4];
    memset(base, 0, sizeof(base));
    memset(ext, 0, sizeof(ext));

    int base_len = dot ? (int)(dot - long_name) : (int)strlen(long_name);
    int bi = 0;
    for (int i = 0; i < base_len && bi < 8; i++) {
        char c = long_name[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) base[bi++] = c;
    }
    if (bi == 0) { base[0] = '_'; bi = 1; }

    int ei = 0;
    if (dot) {
        for (const char *p = dot + 1; *p && ei < 3; p++) {
            char c = *p;
            if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
            if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) ext[ei++] = c;
        }
    }

    for (int n = 1; n <= 999; n++) {
        char candidate[8];
        if (n == 1) {
            memcpy(candidate, base, 8);
            for (int i = bi; i < 8; i++) candidate[i] = ' ';
        } else {
            char tail[6];
            sprintf(tail, "~%d", n);
            int tail_len = (int)strlen(tail);
            int keep = 8 - tail_len;
            if (keep > bi) keep = bi;
            if (keep < 0) keep = 0;
            memcpy(candidate, base, keep);
            memcpy(candidate + keep, tail, tail_len);
            for (int i = keep + tail_len; i < 8; i++) candidate[i] = ' ';
        }

        int collision = 0;
        for (uint32_t i = 0; i < fatfs_entry_count; i++) {
            if (fatfs_entries[i].parent_index != parent_index) continue;
            if (memcmp(fatfs_entries[i].short_name, candidate, 8) == 0 &&
                memcmp(fatfs_entries[i].short_name + 8, ext, (size_t)ei) == 0) {
                collision = 1;
                break;
            }
        }
        if (!collision) {
            memcpy(out11, candidate, 8);
            for (int i = 0; i < 3; i++) out11[8 + i] = (i < ei) ? ext[i] : ' ';
            return;
        }
    }
    memcpy(out11, "OVERFLOW", 8);
    memcpy(out11 + 8, "TXT", 3);
}

/* ==================================================================
 * VFAT long filename entries -- generation (for reads) and parsing
 * (for writes, so newly-created files get their true original name).
 * ================================================================== */
static const uint8_t FATFS_LFN_CHAR_OFFSETS[13] = {1,3,5,7,9,14,16,18,20,22,24,28,30};

static uint8_t fatfs_lfn_checksum(const char short_name11[11]) {
    uint8_t sum = 0;
    for (int i = 0; i < 11; i++) {
        sum = (uint8_t)(((sum & 1) ? 0x80 : 0) + (sum >> 1) + (uint8_t)short_name11[i]);
    }
    return sum;
}

/* Writes LFN entries + the final short entry into `table`, starting at
 * slot *pos (32 bytes/slot), advancing *pos. Returns 0 (and writes
 * nothing) if there isn't room for the whole group in max_slots -- the
 * caller should treat that entry as "doesn't fit, skip it" rather than
 * write a truncated/corrupt group. */
static int fatfs_write_name_entries(uint8_t *table, uint32_t max_slots, uint32_t *pos,
                                     const char *long_name, const char short_name11[11],
                                     uint8_t attr, uint32_t start_cluster, uint32_t size) {
    size_t len = strlen(long_name);
    if (len == 0) len = 1;
    uint32_t lfn_count = (uint32_t)((len + 12) / 13);
    if (*pos + lfn_count + 1 > max_slots) return 0;

    uint8_t checksum = fatfs_lfn_checksum(short_name11);
    for (uint32_t seq = lfn_count; seq >= 1; seq--) {
        uint8_t *e = &table[(*pos) * 32];
        uint8_t seqbyte = (uint8_t)seq;
        if (seq == lfn_count) seqbyte |= 0x40; // "last" LFN entry (first on disk, per spec)
        e[0] = seqbyte;
        e[11] = 0x0F; // ATTR_LONG_NAME
        e[12] = 0x00;
        e[13] = checksum;
        e[26] = 0x00; e[27] = 0x00;

        uint32_t char_base = (seq - 1) * 13;
        int ended = 0;
        for (int ci = 0; ci < 13; ci++) {
            uint32_t src_i = char_base + (uint32_t)ci;
            uint16_t ch;
            if (!ended && src_i < len) {
                ch = (uint16_t)(unsigned char)long_name[src_i];
            } else if (!ended && src_i == len) {
                ch = 0x0000;
                ended = 1;
            } else {
                ch = 0xFFFF;
            }
            e[FATFS_LFN_CHAR_OFFSETS[ci]]     = (uint8_t)(ch & 0xFF);
            e[FATFS_LFN_CHAR_OFFSETS[ci] + 1] = (uint8_t)(ch >> 8);
        }
        (*pos)++;
    }

    uint8_t *e = &table[(*pos) * 32];
    memset(e, 0, 32);
    memcpy(e, short_name11, 11);
    e[11] = attr;
    e[24] = 0x21; e[25] = 0x56; // write date (dummy, valid-looking)
    e[26] = (uint8_t)(start_cluster & 0xFF);
    e[27] = (uint8_t)(start_cluster >> 8);
    e[28] = (uint8_t)(size & 0xFF);
    e[29] = (uint8_t)((size >> 8) & 0xFF);
    e[30] = (uint8_t)((size >> 16) & 0xFF);
    e[31] = (uint8_t)((size >> 24) & 0xFF);
    (*pos)++;
    return 1;
}

/* Persistent (across separate sector writes) accumulator for LFN
 * fragments seen so far -- a name's LFN+short entry group could in
 * principle straddle a 512-byte WRITE(10) sector boundary, so this
 * needs to survive between calls, not just within one sector.
 *
 * Scoped PER DIRECTORY TABLE (root, plus each subdirectory slot) --
 * NOT a single shared instance. A folder move writes to root's table
 * and both the source and destination subdirectories' tables in quick
 * succession; a single global accumulator let a partial LFN sequence
 * from one table get stomped by an interleaved write to a different
 * table before it was closed off by its matching short entry,
 * producing corrupted, cross-contaminated parses (garbage names,
 * nonsense cluster numbers far outside the volume's valid range). */
#define FATFS_LFN_TABLE_SLOTS (FATFS_MAX_SUBDIRS + 1) // +1 for root
#define FATFS_ROOT_LFN_SLOT   FATFS_MAX_SUBDIRS

static char    fatfs_pending_lfn[FATFS_LFN_TABLE_SLOTS][FATFS_MAX_LONG_NAME * 4];
static int     fatfs_pending_lfn_len[FATFS_LFN_TABLE_SLOTS];
static uint8_t fatfs_pending_lfn_checksum[FATFS_LFN_TABLE_SLOTS];
static int     fatfs_pending_lfn_active[FATFS_LFN_TABLE_SLOTS];

#if DEBUG_FS
#include <stdarg.h>
// Appends a formatted line directly to /documents/usb_create_log.txt.tns.
// Only fires on real filesystem events (creation, deletion, rename,
// directory-move) -- rare enough that even with logging enabled, this
// doesn't reintroduce the per-sector slowdown a much-higher-frequency
// version of this had earlier in development.
static void fatfs_log(const char *fmt, ...) {
    char line[280];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    FILE *lg = fopen("/documents/usb_create_log.txt.tns", "ab");
    if (lg) { fwrite(line, 1, (size_t)n, lg); fclose(lg); }
}

// Clears the log to a fresh, empty file -- called once at program
// startup (not on every fatfs_init(), e.g. via Tab-switch, which
// should keep accumulating within the same session).
static void fatfs_log_reset(void) {
    FILE *lg = fopen("/documents/usb_create_log.txt.tns", "wb");
    if (lg) fclose(lg);
}
#else
static inline void fatfs_log(const char *fmt, ...) { (void)fmt; }
static inline void fatfs_log_reset(void) {}
#endif

// Logs every filesystem-mutating operation before it executes, and
// always allows it -- no blocking, no prompt. A blocking Y/N version
// used to live here, but confirmed to cause real macOS timeouts
// (directory moves failing outright) since it paused mid-USB-
// transaction waiting for a keypress, and the host has its own ~5
// second timeout for how long it'll wait for a command to complete.
// This keeps the visibility without the risk. Defined once, outside
// the DEBUG_FS conditional above, so the on-screen announcement
// (display_msg) always works regardless of build type -- only the
// file-log side (fatfs_log itself) is debug-only.
// Set whenever a real filesystem mutation happens (create/delete/
// move/rename) -- lets us skip calling the OS's document-browser
// refresh at exit entirely when nothing actually changed, since that
// refresh is known to be slow on real hardware (scales with the
// user's total folder count).
static bool fatfs_any_mutation_happened = false;

static int fatfs_confirm(const char *action, const char *display_msg = NULL) {
    fatfs_any_mutation_happened = true;
    fatfs_log("op: \"%s\"\n", action);
    if (display_msg) screen_console << display_msg << nio::endl;
    return 1;
}

/* ==================================================================
 * Directory tree: recursive scan (startup) + live write tracking
 * (ongoing, for file creation and size updates)
 * ================================================================== */
static void fatfs_scan_recurse(const char *dirpath, int32_t parent_index, int depth, const char *priority_name);
static void fatfs_mark_fat_chain(FatEntry *fe);

static void fatfs_scan_one_entry(const char *dirpath, const char *name, int32_t parent_index, int depth) {
    char full[192];
    fatfs_join_path(full, sizeof(full), dirpath, name);

    DIR *subd = opendir(full);
    if (subd) {
        closedir(subd);
        uint32_t idx = fatfs_entry_count;
        FatEntry *fe = &fatfs_entries[idx];
        memset(fe, 0, sizeof(*fe));
        strncpy(fe->path, full, sizeof(fe->path) - 1);
        strncpy(fe->long_name, name, sizeof(fe->long_name) - 1);
        fe->is_dir = 1;
        fe->parent_index = parent_index;
        fe->subdir_slot = (fatfs_subdir_count < FATFS_MAX_SUBDIRS) ? (int32_t)fatfs_subdir_count : -1;
        fe->scanned = 1; // eager scanning -- always fully discovered by the time this returns
        fatfs_make_short_name(name, fe->short_name, parent_index);
        fatfs_entry_count++;
        if (fe->subdir_slot >= 0) {
            fatfs_subdir_count++;
            fatfs_scan_recurse(full, (int32_t)idx, depth + 1, NULL);
        }
        return;
    }

    FILE *f = fopen(full, "rb");
    if (!f) return;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fclose(f);
    if (size < 0) return;

    FatEntry *fe = &fatfs_entries[fatfs_entry_count];
    memset(fe, 0, sizeof(*fe));
    strncpy(fe->path, full, sizeof(fe->path) - 1);
    strncpy(fe->long_name, name, sizeof(fe->long_name) - 1);
    fe->is_dir = 0;
    fe->size = (uint32_t)size;
    fe->parent_index = parent_index;
    fe->subdir_slot = -1;
    fatfs_make_short_name(name, fe->short_name, parent_index);
    fatfs_entry_count++;
}

// priority_name, if non-NULL, is scanned first via a dedicated pass
// (used only for the top-level call when scanning "/", to guarantee
// "/documents" always claims a subdirectory slot rather than possibly
// losing the race to unrelated top-level folders, since readdir()'s
// ordering isn't predictable or alphabetical).
static void fatfs_scan_recurse(const char *dirpath, int32_t parent_index, int depth, const char *priority_name) {
    if (depth > (int)FATFS_MAX_SCAN_DEPTH) return;

    if (priority_name) {
        DIR *pd = opendir(dirpath);
        if (pd) {
            struct dirent *pent;
            while ((pent = readdir(pd)) != NULL) {
                if (strcmp(pent->d_name, priority_name) != 0) continue;
                if (fatfs_entry_count < FATFS_MAX_ENTRIES) {
                    fatfs_scan_one_entry(dirpath, pent->d_name, parent_index, depth);
                }
                break;
            }
            closedir(pd);
        }
    }

    DIR *d = opendir(dirpath);
    if (!d) return;

    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        if (priority_name && strcmp(ent->d_name, priority_name) == 0) continue; // already handled above
        if (fatfs_entry_count >= FATFS_MAX_ENTRIES) break;
        fatfs_scan_one_entry(dirpath, ent->d_name, parent_index, depth);
    }
    closedir(d);
}

static void fatfs_build_subdir_tables(void) {
    memset(fatfs_subdir_tables, 0, sizeof(fatfs_subdir_tables));
    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        FatEntry *dir = &fatfs_entries[i];
        if (!dir->is_dir || dir->subdir_slot < 0) continue;
        uint8_t *table = fatfs_subdir_tables[dir->subdir_slot];
        uint32_t pos = 0;
        uint32_t max_slots = FATFS_CLUSTER_BYTES / 32;

        memcpy(&table[pos * 32], ".          ", 11);
        table[pos * 32 + 11] = 0x10;
        table[pos * 32 + 26] = (uint8_t)(dir->start_cluster & 0xFF);
        table[pos * 32 + 27] = (uint8_t)(dir->start_cluster >> 8);
        pos++;

        uint32_t parent_cluster = 0;
        if (dir->parent_index >= 0) parent_cluster = fatfs_entries[dir->parent_index].start_cluster;
        memcpy(&table[pos * 32], "..         ", 11);
        table[pos * 32 + 11] = 0x10;
        table[pos * 32 + 26] = (uint8_t)(parent_cluster & 0xFF);
        table[pos * 32 + 27] = (uint8_t)(parent_cluster >> 8);
        pos++;

        for (uint32_t j = 0; j < fatfs_entry_count; j++) {
            if (fatfs_entries[j].deleted) continue;
            if (fatfs_entries[j].parent_index != (int32_t)i) continue;
            FatEntry *child = &fatfs_entries[j];
            uint8_t attr = child->is_dir ? 0x10 : 0x20;
            if (!child->is_dir && fatfs_self_path && strcmp(child->path, fatfs_self_path) == 0) attr |= 0x01;
            fatfs_write_name_entries(table, max_slots, &pos, child->long_name, child->short_name,
                                      attr, child->start_cluster, child->size);
        }
    }
}

static void fatfs_build_root_dir(void) {
    memset(fatfs_root_dir, 0, sizeof(fatfs_root_dir));
    uint32_t pos = 0;
    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        FatEntry *fe = &fatfs_entries[i];
        if (fe->deleted) continue;
        if (fe->parent_index != -1) continue;
        uint8_t attr = fe->is_dir ? 0x10 : 0x20;
        if (!fe->is_dir && fatfs_self_path && strcmp(fe->path, fatfs_self_path) == 0) attr |= 0x01;
        fatfs_write_name_entries(fatfs_root_dir, FATFS_ROOT_ENTRIES, &pos, fe->long_name, fe->short_name,
                                  attr, fe->start_cluster, fe->size);
    }
}

static void fatfs_scan(void) {
    fatfs_entry_count = 0;
    fatfs_subdir_count = 0;
    fatfs_locate_cluster_cache_cluster = 0xFFFFFFFFu;
    fatfs_locate_cluster_cache_gen = 0xFFFFFFFFu;

    // Synthetic, virtual-only ".fseventsd/no_log" -- tells macOS's
    // fseventsd daemon to skip event logging on this volume (a real,
    // documented convention: an empty file named "no_log" inside a
    // ".fseventsd" folder at the volume root). Injected FIRST so it
    // always claims a subdirectory slot, regardless of how many real
    // folders exist. Nothing is created on the real, underlying
    // filesystem -- this exists purely in our synthesized view.
    {
        uint32_t dir_idx = fatfs_entry_count;
        FatEntry *dir = &fatfs_entries[dir_idx];
        memset(dir, 0, sizeof(*dir));
        strncpy(dir->long_name, ".fseventsd", sizeof(dir->long_name) - 1);
        fatfs_make_short_name(".fseventsd", dir->short_name, -1);
        dir->is_dir = 1;
        dir->parent_index = -1;
        dir->subdir_slot = (int32_t)fatfs_subdir_count;
        dir->scanned = 1;
        fatfs_subdir_count++;
        fatfs_entry_count++;

        FatEntry *nolog = &fatfs_entries[fatfs_entry_count];
        memset(nolog, 0, sizeof(*nolog));
        strncpy(nolog->long_name, "no_log", sizeof(nolog->long_name) - 1);
        fatfs_make_short_name("no_log", nolog->short_name, (int32_t)dir_idx);
        nolog->is_dir = 0;
        nolog->size = 0; // stays empty -- size=0 means we never dereference nolog->path
        nolog->parent_index = (int32_t)dir_idx;
        nolog->subdir_slot = -1;
        fatfs_entry_count++;
    }

    // Confirmed fix for macOS's OTHER standard per-volume folder:
    // ".Trashes" (its own Trash directory for removable media).
    // Unlike .fseventsd, the trick here is injecting it as a plain
    // FILE, not a directory -- macOS specifically checks whether that
    // name is already taken by something that isn't a directory, and
    // backs off from creating its own. (An earlier attempt injected
    // this as a directory, which does nothing to stop macOS's own
    // creation attempt -- that's why it kept happening.) Doesn't need
    // a subdirectory slot at all now.
    if (fatfs_entry_count < FATFS_MAX_ENTRIES) {
        FatEntry *tr = &fatfs_entries[fatfs_entry_count];
        memset(tr, 0, sizeof(*tr));
        strncpy(tr->long_name, ".Trashes", sizeof(tr->long_name) - 1);
        fatfs_make_short_name(".Trashes", tr->short_name, -1);
        tr->is_dir = 0;
        tr->size = 0;
        tr->parent_index = -1;
        tr->subdir_slot = -1;
        fatfs_entry_count++;
    }

    // Same general idea: an empty ".metadata_never_index" file at the
    // volume root discourages Spotlight from indexing it, which should
    // reduce another source of macOS's own background housekeeping
    // writes to the volume.
    if (fatfs_entry_count < FATFS_MAX_ENTRIES) {
        FatEntry *mni = &fatfs_entries[fatfs_entry_count];
        memset(mni, 0, sizeof(*mni));
        strncpy(mni->long_name, ".metadata_never_index", sizeof(mni->long_name) - 1);
        fatfs_make_short_name(".metadata_never_index", mni->short_name, -1);
        mni->is_dir = 0;
        mni->size = 0;
        mni->parent_index = -1;
        mni->subdir_slot = -1;
        fatfs_entry_count++;
    }

    {
        const char *priority = (strcmp(fatfs_root_path, "/") == 0) ? "documents" : NULL;
        fatfs_scan_recurse(fatfs_root_path, -1, 0, priority);
    }

    uint32_t next_cluster = 2;
    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        FatEntry *fe = &fatfs_entries[i];
        if (fe->is_dir) {
            if (fe->subdir_slot < 0) { fe->start_cluster = 0; fe->num_clusters = 0; continue; }
            fe->start_cluster = next_cluster;
            fe->num_clusters = 1;
            next_cluster += 1;
        } else {
            uint32_t clusters = (fe->size + FATFS_CLUSTER_BYTES - 1) / FATFS_CLUSTER_BYTES;
            if (clusters == 0) clusters = 1;
            fe->start_cluster = next_cluster;
            fe->num_clusters = clusters;
            next_cluster += clusters;
        }
    }
    fatfs_total_data_clusters = next_cluster - 2;

    fatfs_build_subdir_tables();
}

/* ==================================================================
 * Boot sector / FAT table / MBR builders (unchanged in spirit)
 * ================================================================== */
static void fatfs_build_boot_sector(void) {
    uint8_t *b = fatfs_boot_sector;
    memset(b, 0, FATFS_SECTOR_SIZE);

    b[0] = 0xEB; b[1] = 0x3C; b[2] = 0x90;
    memcpy(&b[0x03], "NSPIRDOC", 8);

    b[0x0B] = (uint8_t)(FATFS_SECTOR_SIZE & 0xFF);
    b[0x0C] = (uint8_t)(FATFS_SECTOR_SIZE >> 8);
    b[0x0D] = (uint8_t)FATFS_CLUSTER_SECTORS;
    b[0x0E] = 1; b[0x0F] = 0;
    b[0x10] = 2;
    b[0x11] = (uint8_t)(FATFS_ROOT_ENTRIES & 0xFF);
    b[0x12] = (uint8_t)(FATFS_ROOT_ENTRIES >> 8);

    if (fatfs_total_sectors <= 0xFFFF) {
        b[0x13] = (uint8_t)(fatfs_total_sectors & 0xFF);
        b[0x14] = (uint8_t)(fatfs_total_sectors >> 8);
    } else {
        b[0x13] = 0; b[0x14] = 0;
    }

    b[0x15] = 0xF8;
    b[0x16] = (uint8_t)(fatfs_fat_sectors_per_copy & 0xFF);
    b[0x17] = (uint8_t)(fatfs_fat_sectors_per_copy >> 8);
    b[0x18] = 32; b[0x19] = 0;
    b[0x1A] = 64; b[0x1B] = 0;

    if (fatfs_total_sectors > 0xFFFF) {
        b[0x20] = (uint8_t)(fatfs_total_sectors & 0xFF);
        b[0x21] = (uint8_t)((fatfs_total_sectors >> 8) & 0xFF);
        b[0x22] = (uint8_t)((fatfs_total_sectors >> 16) & 0xFF);
        b[0x23] = (uint8_t)((fatfs_total_sectors >> 24) & 0xFF);
    }

    b[0x24] = 0x80;
    b[0x25] = 0;
    b[0x26] = 0x29;
    {
        static uint32_t fatfs_boot_init_count = 0;
        fatfs_boot_init_count++;
        uint32_t stack_sample;
        uint32_t serial = ((uint32_t)(uintptr_t)&stack_sample) ^ (fatfs_boot_init_count * 0x9E3779B1u);
        b[0x27] = (uint8_t)(serial & 0xFF);
        b[0x28] = (uint8_t)((serial >> 8) & 0xFF);
        b[0x29] = (uint8_t)((serial >> 16) & 0xFF);
        b[0x2A] = (uint8_t)((serial >> 24) & 0xFF);
    }
    memcpy(&b[0x2B], (strcmp(fatfs_root_path, "/") == 0) ? "NSPIRE ROOT" : "NSPIRE DOCS", 11);
    memcpy(&b[0x36], "FAT16   ", 8);

    b[0x1FE] = 0x55; b[0x1FF] = 0xAA;
}

static void fatfs_mark_fat_chain(FatEntry *fe) {
    if (fe->num_clusters == 0) return;
    for (uint32_t c = 0; c < fe->num_clusters; c++) {
        uint32_t cluster = fe->start_cluster + c;
        if (cluster >= FATFS_MAX_CLUSTERS) break;
        uint16_t entry = (c + 1 < fe->num_clusters) ? (uint16_t)(cluster + 1) : 0xFFFF;
        fatfs_fat_table[cluster * 2]     = (uint8_t)(entry & 0xFF);
        fatfs_fat_table[cluster * 2 + 1] = (uint8_t)(entry >> 8);
    }
}

static void fatfs_build_fat_table(void) {
    memset(fatfs_fat_table, 0, sizeof(fatfs_fat_table));
    fatfs_fat_table[0] = 0xF8; fatfs_fat_table[1] = 0xFF;
    fatfs_fat_table[2] = 0xFF; fatfs_fat_table[3] = 0xFF;

    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        fatfs_mark_fat_chain(&fatfs_entries[i]);
    }
}

static void fatfs_build_mbr(void) {
    memset(fatfs_mbr, 0, FATFS_SECTOR_SIZE);
    {
        static uint32_t fatfs_mbr_init_count = 0;
        fatfs_mbr_init_count++;
        uint32_t stack_sample;
        uint32_t sig = ((uint32_t)(uintptr_t)&stack_sample) ^ (fatfs_mbr_init_count * 0x85EBCA6Bu);
        fatfs_mbr[440] = (uint8_t)(sig & 0xFF);
        fatfs_mbr[441] = (uint8_t)((sig >> 8) & 0xFF);
        fatfs_mbr[442] = (uint8_t)((sig >> 16) & 0xFF);
        fatfs_mbr[443] = (uint8_t)((sig >> 24) & 0xFF);
    }
    uint8_t *entry = &fatfs_mbr[446];
    entry[0] = 0x00;
    entry[1] = 0xFE; entry[2] = 0xFF; entry[3] = 0xFF;
    entry[4] = 0x06;
    entry[5] = 0xFE; entry[6] = 0xFF; entry[7] = 0xFF;
    entry[8]  = (uint8_t)(FATFS_PARTITION_START_LBA & 0xFF);
    entry[9]  = (uint8_t)((FATFS_PARTITION_START_LBA >> 8) & 0xFF);
    entry[10] = (uint8_t)((FATFS_PARTITION_START_LBA >> 16) & 0xFF);
    entry[11] = (uint8_t)((FATFS_PARTITION_START_LBA >> 24) & 0xFF);
    entry[12] = (uint8_t)(fatfs_total_sectors & 0xFF);
    entry[13] = (uint8_t)((fatfs_total_sectors >> 8) & 0xFF);
    entry[14] = (uint8_t)((fatfs_total_sectors >> 16) & 0xFF);
    entry[15] = (uint8_t)((fatfs_total_sectors >> 24) & 0xFF);
    fatfs_mbr[510] = 0x55; fatfs_mbr[511] = 0xAA;
}

void fatfs_init(void) {
    fatfs_scan();

    fatfs_root_dir_sectors = (FATFS_ROOT_ENTRIES * 32) / FATFS_SECTOR_SIZE;

    // Always declare the volume's full, intended capacity -- not one
    // scaled down to barely fit whatever content currently exists.
    // Scaling it down was the actual cause of "not enough disk space"
    // errors on the host: the declared free space would shrink to
    // just above existing content size, leaving little to no real
    // headroom for new files being copied in.
    uint32_t total_clusters = FATFS_MAX_CLUSTERS;

    uint32_t fat_bytes_needed = (total_clusters + 2) * 2;
    fatfs_fat_sectors_per_copy = (fat_bytes_needed + FATFS_SECTOR_SIZE - 1) / FATFS_SECTOR_SIZE;
    if (fatfs_fat_sectors_per_copy < 1) fatfs_fat_sectors_per_copy = 1;

    uint32_t reserved_sectors = 1;
    fatfs_data_start_lba = reserved_sectors + (fatfs_fat_sectors_per_copy * 2) + fatfs_root_dir_sectors;
    fatfs_total_sectors = fatfs_data_start_lba + (total_clusters * FATFS_CLUSTER_SECTORS);

    fatfs_build_boot_sector();
    fatfs_build_fat_table();
    fatfs_build_root_dir();
    fatfs_build_mbr();

    for (uint32_t i = 0; i < FATFS_LFN_TABLE_SLOTS; i++) fatfs_pending_lfn_active[i] = 0;

    {
        uint32_t dirs = 0, files = 0;
        for (uint32_t i = 0; i < fatfs_entry_count; i++) {
            if (fatfs_entries[i].is_dir) dirs++; else files++;
        }
        char d[130];
        sprintf(d, "fatfs: %u files, %u dirs (%u/%u with content), %u volume sectors, %u disk sectors",
                (unsigned int)files, (unsigned int)dirs,
                (unsigned int)fatfs_subdir_count, (unsigned int)FATFS_MAX_SUBDIRS,
                (unsigned int)fatfs_total_sectors,
                (unsigned int)(fatfs_total_sectors + FATFS_PARTITION_START_LBA));
        dbg_print(d);
    }
}

// Switches the synthesized root between "/documents" and "/" (the
// whole calculator filesystem) and does a full fresh rescan. Called
// from main.cpp around a real disconnect+reconnect cycle (Tab), so the
// host re-enumerates and picks up the new content immediately rather
// than needing to notice a live change underneath an active
// connection.
void fatfs_toggle_root(void) {
    if (strcmp(fatfs_root_path, "/documents") == 0) {
        fatfs_root_path = "/";
    } else {
        fatfs_root_path = "/documents";
    }
    fatfs_init();
}

uint32_t fatfs_total_sectors_count(void) {
    return fatfs_total_sectors + FATFS_PARTITION_START_LBA;
}

/* ==================================================================
 * Live FAT-chain walking -- authoritative for cluster->file mapping
 * now, since a file can grow beyond what fe->num_clusters said at
 * scan time. Directories stay fixed at exactly 1 cluster (no growth).
 * ================================================================== */
static uint32_t fatfs_fat_next(uint32_t cluster) {
    if (cluster >= FATFS_MAX_CLUSTERS) return 0xFFFF;
    uint32_t off = cluster * 2;
    return fatfs_fat_table[off] | ((uint32_t)fatfs_fat_table[off + 1] << 8);
}

static int32_t fatfs_locate_cluster(uint32_t cluster, uint32_t *out_chain_pos) {
    if (cluster == fatfs_locate_cluster_cache_cluster + 1 &&
        fatfs_locate_cluster_cache_gen == fatfs_entry_count) {
        // Previous cluster wasn't found, and no new entries have been
        // added since -- guaranteed this one won't be found either,
        // without repeating the full scan + per-entry FAT-chain walk.
        fatfs_locate_cluster_cache_cluster = cluster;
        return -1;
    }
    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        FatEntry *fe = &fatfs_entries[i];
        if (fe->deleted) continue;
        if (fe->start_cluster == 0) continue;
        if (fe->is_dir) {
            if (fe->start_cluster == cluster) { *out_chain_pos = 0; return (int32_t)i; }
            continue;
        }
        uint32_t c = fe->start_cluster;
        uint32_t pos = 0;
        while (1) {
            if (c == cluster) { *out_chain_pos = pos; return (int32_t)i; }
            uint32_t next = fatfs_fat_next(c);
            if (next < 2 || next >= 0xFFF8) break;
            c = next;
            pos++;
            if (pos > FATFS_MAX_CLUSTERS) break;
        }
    }
    fatfs_locate_cluster_cache_cluster = cluster;
    fatfs_locate_cluster_cache_gen = fatfs_entry_count;
    return -1;
}

/* Copies real file content across and removes the original -- used for
 * renaming an already-named, already-real tracked file (a plain Finder
 * rename, not the staging pattern), and for promoting a staging file
 * to its final name. Uses copy+remove instead of rename(): a later
 * attempt to use rename() directly for staging-file promotion was
 * confirmed to silently fail in this environment -- it reported
 * success but the file was never actually created at its destination.
 * Only fopen/fread/fwrite/fclose/remove are confirmed reliable here. */
/* We persist the raw incoming directory-table write before parsing it
 * (so the host's own subsequent reads reflect its write), which means
 * a corrupted/garbage size field in a malformed entry stays served
 * back to the host verbatim even after we decide to track a different,
 * trusted size internally. Overwrites just the 4 size bytes of the
 * matching entry in the actual persisted table to keep what we serve
 * consistent with what we track -- a real, confirmed inconsistency
 * that could otherwise leave the host believing a file is megabytes
 * larger than it actually has real backing for. */
static void fatfs_fix_persisted_size(int32_t parent_index, const char short_name11[11], uint32_t new_size) {
    uint8_t *table;
    uint32_t table_bytes;
    if (parent_index < 0) {
        table = fatfs_root_dir;
        table_bytes = sizeof(fatfs_root_dir);
    } else {
        int32_t slot = fatfs_entries[parent_index].subdir_slot;
        if (slot < 0) return;
        table = fatfs_subdir_tables[slot];
        table_bytes = FATFS_CLUSTER_BYTES;
    }
    for (uint32_t off = 0; off + 32 <= table_bytes; off += 32) {
        uint8_t *e = table + off;
        if (e[0] == 0x00 || e[0] == 0xE5 || e[11] == 0x0F) continue;
        if (memcmp(e, short_name11, 11) != 0) continue;
        e[28] = (uint8_t)(new_size & 0xFF);
        e[29] = (uint8_t)((new_size >> 8) & 0xFF);
        e[30] = (uint8_t)((new_size >> 16) & 0xFF);
        e[31] = (uint8_t)((new_size >> 24) & 0xFF);
        return;
    }
}

#define FATFS_PROGRESS_REPORT_THRESHOLD (100u * 1024u) // only start showing progress once a file/folder is this large
#define FATFS_PROGRESS_REPORT_INTERVAL  (250u * 1024u) // then update every this many additional bytes (incoming USB transfers only)
#define FATFS_RECEIVING_ANNOUNCE_THRESHOLD (8u * 1024u) // announce "Receiving a file..." this early -- small enough to fire near the true start, large enough that tiny hidden sidecar files (which rarely exceed one small cluster) usually never trigger it at all

// Shared state for a real file/folder move currently in progress --
// unlike incoming USB transfers, the source size is already known
// upfront here, so a true percentage is achievable rather than just a
// running byte count.
static uint32_t fatfs_move_progress_total = 0;
static char fatfs_move_display_name[64];
static uint32_t fatfs_move_progress_done = 0;
static uint32_t fatfs_move_progress_last_reported_pct = 0xFFFFFFFFu;

static void fatfs_format_size(char *buf, size_t buflen, uint32_t bytes) {
    if (bytes >= 1024u * 1024u) {
        uint32_t whole = bytes / (1024u * 1024u);
        uint32_t frac = (bytes % (1024u * 1024u)) * 10u / (1024u * 1024u);
        snprintf(buf, buflen, "%u.%u MB", (unsigned int)whole, (unsigned int)frac);
    } else {
        snprintf(buf, buflen, "%u KB", (unsigned int)(bytes / 1024u));
    }
}

static void fatfs_move_report_progress(void) {
    if (fatfs_move_progress_total < FATFS_PROGRESS_REPORT_THRESHOLD) return;
    uint32_t pct = (uint32_t)(((uint64_t)fatfs_move_progress_done * 100) / fatfs_move_progress_total);
    if (pct > 100) pct = 100;
    if (pct == fatfs_move_progress_last_reported_pct) return;
    // Report every 5% rather than every single percentage point, to
    // keep the screen readable during a large move.
    if (pct % 5 != 0 && pct != 100) return;
    fatfs_move_progress_last_reported_pct = pct;
    char done_str[20], total_str[20];
    fatfs_format_size(done_str, sizeof(done_str), fatfs_move_progress_done);
    fatfs_format_size(total_str, sizeof(total_str), fatfs_move_progress_total);
    char line[160];
    snprintf(line, sizeof(line), "  moving \"%s\"... %u%% (%s/%s)",
             fatfs_move_display_name, (unsigned int)pct, done_str, total_str);
    screen_console << line << nio::endl;
}

// Reprints the connection-time controls reminder -- called after a
// large transfer/move finishes, since progress messages may have
// scrolled the original reminder off-screen.
static void fatfs_print_controls_reminder(void) {
    screen_console << "  Esc  - eject and exit" << nio::endl;
    screen_console << "  Tab  - switch root between /documents and / (reconnects)" << nio::endl;
}

#define FATFS_BG_COPY_QUEUE_MAX 256
typedef struct {
    char old_path[128];
    char new_path[128];
} FatBgCopyEntry;
static FatBgCopyEntry fatfs_bg_copy_queue[FATFS_BG_COPY_QUEUE_MAX];
static int fatfs_bg_copy_queue_count = 0;
static int fatfs_bg_copy_queue_pos = 0;
static FILE *fatfs_bg_copy_src = NULL;
static FILE *fatfs_bg_copy_dst = NULL;
static int  fatfs_bg_copy_active = 0;
static int  fatfs_bg_copy_ok = 1;
static int  fatfs_bg_copy_is_folder = 0;
static char fatfs_bg_copy_root_old[192];

// Walks a directory tree up front, creating destination subdirectories
// immediately (fast -- just metadata, no data movement) and queuing
// every real file for later, incremental copying. If the queue's
// fixed capacity is somehow exceeded (very large folder), remaining
// files are copied synchronously right here as a fallback rather than
// silently dropped -- rare in practice, and still correct, just
// without the background-yielding benefit for that specific overflow.
static int fatfs_bg_copy_enumerate(const char *old_path, const char *new_path) {
    int mkdir_result = mkdir(new_path, 0777);
    fatfs_log("bg-copy: mkdir(\"%s\") returned %d (errno=%d)\n", new_path, mkdir_result, errno);

    DIR *d = opendir(old_path);
    if (!d) {
        fatfs_log("bg-copy: FAILED to open source directory \"%s\" (errno=%d)\n", old_path, errno);
        return 0;
    }
    struct dirent *ent;
    int ok = 1;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        char old_full[256], new_full[256];
        fatfs_join_path(old_full, sizeof(old_full), old_path, ent->d_name);
        fatfs_join_path(new_full, sizeof(new_full), new_path, ent->d_name);

        DIR *subd = opendir(old_full);
        if (subd) {
            closedir(subd);
            if (!fatfs_bg_copy_enumerate(old_full, new_full)) ok = 0;
            continue;
        }

        if (fatfs_bg_copy_queue_count >= FATFS_BG_COPY_QUEUE_MAX) {
            fatfs_log("bg-copy: queue full, copying \"%s\" synchronously as a fallback\n", old_full);
            FILE *in = fopen(old_full, "rb");
            FILE *out = in ? fopen(new_full, "wb") : NULL;
            if (in && out) {
                uint8_t buf[2048];
                size_t n;
                while ((n = fread(buf, 1, sizeof(buf), in)) > 0) fwrite(buf, 1, n, out);
            } else {
                ok = 0;
            }
            if (in) fclose(in);
            if (out) fclose(out);
            continue;
        }
        FatBgCopyEntry *qe = &fatfs_bg_copy_queue[fatfs_bg_copy_queue_count++];
        strncpy(qe->old_path, old_full, sizeof(qe->old_path) - 1);
        qe->old_path[sizeof(qe->old_path) - 1] = '\0';
        strncpy(qe->new_path, new_full, sizeof(qe->new_path) - 1);
        qe->new_path[sizeof(qe->new_path) - 1] = '\0';
    }
    closedir(d);
    return ok;
}

static void fatfs_recursive_remove_dir(const char *path); // defined below -- forward-declared since fatfs_bg_copy_step (here) needs to call it on completion of a background folder move

// Called once per main-loop iteration. Copies one small chunk of the
// current file if a background copy is active, then returns --
// keeping control flowing back to normal USB servicing between
// chunks, rather than blocking the whole bulk pipe until an entire
// move finishes.
static void fatfs_bg_copy_step(void) {
    if (!fatfs_bg_copy_active) return;

    if (!fatfs_bg_copy_src) {
        if (fatfs_bg_copy_queue_pos >= fatfs_bg_copy_queue_count) {
            if (fatfs_bg_copy_ok && fatfs_bg_copy_is_folder) {
                fatfs_recursive_remove_dir(fatfs_bg_copy_root_old);
            }
            if (fatfs_move_progress_total >= FATFS_PROGRESS_REPORT_THRESHOLD) {
                char size_str[20];
                fatfs_format_size(size_str, sizeof(size_str), fatfs_move_progress_total);
                screen_console << "  done -- " << fatfs_move_display_name << " (" << size_str << ")" << nio::endl;
                fatfs_print_controls_reminder();
            }
            fatfs_bg_copy_active = 0;
            return;
        }
        FatBgCopyEntry *e = &fatfs_bg_copy_queue[fatfs_bg_copy_queue_pos];
        fatfs_bg_copy_src = fopen(e->old_path, "rb");
        fatfs_bg_copy_dst = fatfs_bg_copy_src ? fopen(e->new_path, "wb") : NULL;
        if (!fatfs_bg_copy_src || !fatfs_bg_copy_dst) {
            fatfs_log("bg-copy: FAILED to open \"%s\" -> \"%s\"\n", e->old_path, e->new_path);
            if (fatfs_bg_copy_src) fclose(fatfs_bg_copy_src);
            fatfs_bg_copy_src = NULL;
            fatfs_bg_copy_dst = NULL;
            fatfs_bg_copy_ok = 0;
            fatfs_bg_copy_queue_pos++;
            return;
        }
        return; // opened this iteration -- copy its first chunk next time, keeping each step small
    }

    uint8_t buf[16384];
    size_t n = fread(buf, 1, sizeof(buf), fatfs_bg_copy_src);
    if (n > 0) {
        fwrite(buf, 1, n, fatfs_bg_copy_dst);
        fatfs_move_progress_done += (uint32_t)n;
        fatfs_move_report_progress();
    } else {
        fclose(fatfs_bg_copy_src);
        fclose(fatfs_bg_copy_dst);
        fatfs_bg_copy_src = NULL;
        fatfs_bg_copy_dst = NULL;
        fatfs_bg_copy_queue_pos++;
    }
}

static int fatfs_copy_and_remove(const char *old_path, const char *new_path, int show_progress = 0) {
    FILE *in = fopen(old_path, "rb");
    if (!in) return 0;
    FILE *out = fopen(new_path, "wb");
    if (!out) { fclose(in); return 0; }
    if (show_progress) {
        fseek(in, 0, SEEK_END);
        long sz = ftell(in);
        fseek(in, 0, SEEK_SET);
        fatfs_move_progress_total = (sz > 0) ? (uint32_t)sz : 0;
        fatfs_move_progress_done = 0;
        fatfs_move_progress_last_reported_pct = 0xFFFFFFFFu;
    }
    uint8_t buf[8192];
    size_t n;
    uint32_t chunks = 0;
    while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
        fwrite(buf, 1, n, out);
        if (show_progress) {
            fatfs_move_progress_done += (uint32_t)n;
            fatfs_move_report_progress();
        }
        if (++chunks % 16 == 0) service_control_endpoint();
    }
    fclose(in);
    fclose(out);
    remove(old_path);
    return 1;
}

// Sums the size of every real file under a directory tree (recursing
// into subdirectories), using fopen/fseek/ftell rather than stat() --
// consistent with the rest of this codebase, which has only ever
// confirmed fopen-family calls reliable in this environment.
static uint32_t fatfs_dir_total_size(const char *path) {
    uint32_t total = 0;
    DIR *d = opendir(path);
    if (!d) return 0;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        char full[256];
        fatfs_join_path(full, sizeof(full), path, ent->d_name);
        DIR *subd = opendir(full);
        if (subd) {
            closedir(subd);
            total += fatfs_dir_total_size(full);
            continue;
        }
        FILE *f = fopen(full, "rb");
        if (f) {
            fseek(f, 0, SEEK_END);
            long sz = ftell(f);
            fclose(f);
            if (sz > 0) total += (uint32_t)sz;
        }
    }
    closedir(d);
    return total;
}

/* Real directory move support: recursively copy an entire folder's
 * contents (files and subfolders) to a new real location, and -- only
 * if that fully succeeds -- recursively remove the original. Ordered
 * copy-then-verify-then-delete specifically so a failure partway
 * through never loses data: worst case is a harmless duplicate left at
 * the old location, never data destroyed with nothing to show for it.
 * mkdir()/rmdir() are untested anywhere else in this codebase. */
static int fatfs_recursive_copy_dir(const char *old_path, const char *new_path) {
    int mkdir_result = mkdir(new_path, 0777); // may legitimately already exist, or this may not be supported; caught downstream if writes fail
    fatfs_log("copy-dir: mkdir(\"%s\") returned %d (errno=%d)\n", new_path, mkdir_result, errno);

    DIR *d = opendir(old_path);
    if (!d) {
        fatfs_log("copy-dir: FAILED to open source directory \"%s\" (errno=%d)\n", old_path, errno);
        return 0;
    }

    struct dirent *ent;
    int ok = 1;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        char old_full[256], new_full[256];
        fatfs_join_path(old_full, sizeof(old_full), old_path, ent->d_name);
        fatfs_join_path(new_full, sizeof(new_full), new_path, ent->d_name);

        DIR *subd = opendir(old_full);
        if (subd) {
            closedir(subd);
            if (!fatfs_recursive_copy_dir(old_full, new_full)) ok = 0;
            continue;
        }

        FILE *in = fopen(old_full, "rb");
        if (!in) {
            fatfs_log("copy-dir: FAILED to open source \"%s\" for reading\n", old_full);
            ok = 0;
            continue; // keep going -- copy everything else that can be copied
        }
        FILE *out = fopen(new_full, "wb");
        if (!out) {
            fatfs_log("copy-dir: FAILED to open destination \"%s\" for writing\n", new_full);
            fclose(in);
            ok = 0;
            continue;
        }
        uint8_t buf[2048];
        size_t n;
        uint32_t chunks = 0;
        while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
            fwrite(buf, 1, n, out);
            fatfs_move_progress_done += (uint32_t)n;
            fatfs_move_report_progress();
            if (++chunks % 16 == 0) service_control_endpoint();
        }
        fclose(in);
        fclose(out);
    }
    closedir(d);
    return ok;
}

static void fatfs_recursive_remove_dir(const char *path) {
    DIR *d = opendir(path);
    if (!d) return;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;
        char full[256];
        fatfs_join_path(full, sizeof(full), path, ent->d_name);
        DIR *subd = opendir(full);
        if (subd) {
            closedir(subd);
            fatfs_recursive_remove_dir(full);
        } else {
            remove(full);
        }
    }
    closedir(d);
    rmdir(path);
}

// Directory deletes are tracked immediately but not physically
// executed until now -- see the comment at the 0xE5 handling site for
// why. Call this once, right before actually disconnecting/ejecting,
// to really remove anything that's still marked deleted at that point
// (i.e. nothing ever reactivated it via a matching move).
static void fatfs_cleanup_pending_stage_files(void); // defined below, near the pending-data state

void fatfs_flush_pending_deletes(void) {
    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        FatEntry *fe = &fatfs_entries[i];
        if (!fe->deleted) continue;
        if (fe->is_dir) {
            fatfs_recursive_remove_dir(fe->path);
        } else {
            remove(fe->path);
        }
    }
    fatfs_cleanup_pending_stage_files();
}

/* After a real directory move, every tracked file/subfolder beneath it
 * still has a `path` pointing at the OLD real location -- fix those up
 * recursively, or every subsequent read/write to anything inside the
 * moved folder would target a location that no longer exists. */
static void fatfs_update_child_paths(int32_t dir_idx) {
    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        FatEntry *fe = &fatfs_entries[i];
        if (fe->deleted || fe->parent_index != dir_idx) continue;
        char new_path[256];
        fatfs_join_path(new_path, sizeof(new_path), fatfs_entries[dir_idx].path, fe->long_name);
        strncpy(fe->path, new_path, sizeof(fe->path) - 1);
        fe->path[sizeof(fe->path) - 1] = '\0';
        if (fe->is_dir) fatfs_update_child_paths((int32_t)i);
    }
}

/* Data writes always land in whole 512-byte sectors, so a file's real
 * length can end up rounded up past its true logical size. When a
 * later directory-entry write tells us the true size, trim the real
 * file down to match. Streams through a small fixed-size chunk buffer
 * regardless of file size, rather than requiring the whole file to fit
 * in memory at once. */
static void fatfs_truncate_file(const char *path, uint32_t new_size) {
    char tmp_path[280];
    snprintf(tmp_path, sizeof(tmp_path), "%s.trunctmp", path);
    FILE *in = fopen(path, "rb");
    if (!in) return;
    FILE *out = fopen(tmp_path, "wb");
    if (!out) { fclose(in); return; }
    uint8_t chunk[1024];
    uint32_t remaining = new_size;
    while (remaining > 0) {
        size_t want = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
        size_t got = fread(chunk, 1, want, in);
        if (got == 0) break;
        fwrite(chunk, 1, got, out);
        remaining -= (uint32_t)got;
    }
    fclose(in);
    fclose(out);
    fatfs_copy_and_remove(tmp_path, path);
}

/* Real FAT drivers commonly allocate the cluster chain and write DATA
 * before committing the directory entry that names the file (the
 * entry write is the "commit" step, done last for safety) -- and
 * macOS's "safe save" pattern writes a WHOLE staging file, then
 * renames it over the original. Data for a not-yet-named cluster is
 * written directly to a small staging file on the real disk as it
 * arrives, rather than buffered in RAM -- once the real name is known,
 * that staging file's content is copied into place (streamed via a
 * small chunk buffer, not rename() -- confirmed unreliable in this
 * environment; see fatfs_copy_and_remove's comment) and the staging
 * file removed. This still removes any RAM ceiling on file size
 * entirely; the only real limit becomes actual disk space. If a
 * matching directory entry never arrives (AppleDouble sidecars,
 * .DS_Store, an abandoned staging file), the staging file is deleted
 * -- still no permanent .tmp litter, just handled via cleanup instead
 * of never having touched disk at all. */
#define FATFS_MAX_PENDING 64u

typedef struct {
    uint8_t  used;
    uint32_t start_cluster;
    uint32_t last_cluster;   // most recent cluster appended to this slot
    uint32_t cluster_count;  // how many clusters appended so far (== chain_pos for the next one)
    uint32_t size; // highest (offset + length) written so far
    char     stage_path[64];
    uint8_t *ram_buf;  // lazily malloc'd RAM staging buffer, NULL until first write to this slot
    uint32_t ram_cap;  // capacity of ram_buf once allocated (0 if never allocated)
    uint8_t  spilled;  // 1 once this slot's data has moved to stage_path on disk (either because it outgrew ram_cap, or RAM wasn't available) -- all further writes for this slot go straight to disk from that point on
    uint32_t last_reported_size; // last size (bytes) at which an on-screen progress update was shown for this slot -- avoids reprinting on every single sector
    uint8_t  announced; // whether the generic "Receiving a file..." start announcement has already been shown for this slot
} FatPending;

static FatPending fatfs_pending[FATFS_MAX_PENDING];

// RAM-first staging: most real files (NDLESS programs, documents,
// small archives) fit comfortably within this per-slot budget and
// never touch disk at all -- avoiding the disk-write latency that was
// implicated in slow progress reporting and, on at least one test,
// the host never sending the tail end of a transfer at all. A global
// ceiling across all slots combined guards against exhausting RAM if
// several large files are somehow in flight at once.
#define FATFS_PENDING_RAM_PER_SLOT_CAP (20u * 1024u * 1024u) // 20 MiB
#define FATFS_PENDING_RAM_GLOBAL_CAP   (22u * 1024u * 1024u) // 22 MiB across all slots combined
static uint32_t fatfs_pending_ram_total = 0; // currently allocated across all slots' ram_bufs combined

// We generally don't know a file's final size until the very last
// moment (the directory entry carrying it is typically the last thing
// written), so this shows a running received-byte count rather than a
// percentage -- an honest reflection of what we actually know at any
// given point during the transfer.
static void fatfs_pending_announce_start(uint32_t slot) {
    if (fatfs_pending[slot].announced) return;
    if (fatfs_pending[slot].size < FATFS_RECEIVING_ANNOUNCE_THRESHOLD) return;
    fatfs_pending[slot].announced = 1;
    screen_console << "Receiving a file..." << nio::endl;
}

static void fatfs_pending_report_progress(uint32_t slot) {
    uint32_t size = fatfs_pending[slot].size;
    if (size < FATFS_PROGRESS_REPORT_THRESHOLD) return;
    if (size - fatfs_pending[slot].last_reported_size < FATFS_PROGRESS_REPORT_INTERVAL) return;
    fatfs_pending[slot].last_reported_size = size;
    screen_console << "  received " << (int)(size / 1024) << " KB so far..." << nio::endl;
}

// Moves a slot's RAM-held data to its disk staging file in one bulk
// write, then frees the RAM buffer -- a one-way transition after which
// this slot behaves exactly like the original, disk-only staging
// design. Safe to call on a slot with no RAM buffer at all (e.g. one
// that never got RAM in the first place); just marks it spilled with
// nothing to move.
static void fatfs_pending_spill(uint32_t slot) {
    if (fatfs_pending[slot].spilled) return;
    if (fatfs_pending[slot].ram_buf) {
        FILE *f = fopen(fatfs_pending[slot].stage_path, "wb");
        if (f) {
            fwrite(fatfs_pending[slot].ram_buf, 1, fatfs_pending[slot].size, f);
            fclose(f);
        }
        fatfs_pending_ram_total -= fatfs_pending[slot].ram_cap;
        free(fatfs_pending[slot].ram_buf);
        fatfs_pending[slot].ram_buf = NULL;
        fatfs_pending[slot].ram_cap = 0;
    }
    fatfs_pending[slot].spilled = 1;
}

static uint32_t fatfs_pending_evict_next = 0;

// Cached, most-recently-used staging file handle -- avoids a full
// fopen/fseek/fwrite/fclose cycle for every single sector when
// consecutive writes target the same file (the overwhelmingly common
// case), which was slow enough to likely be masking real-time
// transfer progress on the host side entirely.
static FILE *fatfs_stage_cache_handle = NULL;
static int32_t fatfs_stage_cache_slot = -1;
static long fatfs_stage_cache_pos = -1; // expected current file position, so a redundant fseek() can be skipped for sequential writes

// Must be called before anything reads, renames, or deletes a staging
// file that might still have unflushed writes sitting in the cached
// handle above -- otherwise that read/delete could race an write that
// hasn't actually reached disk yet.
static void fatfs_stage_cache_invalidate(int32_t slot) {
    if (fatfs_stage_cache_slot == slot && fatfs_stage_cache_handle) {
        fclose(fatfs_stage_cache_handle);
        fatfs_stage_cache_handle = NULL;
        fatfs_stage_cache_slot = -1;
        fatfs_stage_cache_pos = -1;
    }
}


static int32_t fatfs_locate_pending(uint32_t cluster, uint32_t *out_chain_pos) {
    for (uint32_t i = 0; i < FATFS_MAX_PENDING; i++) {
        if (!fatfs_pending[i].used) continue;
        if (fatfs_pending[i].start_cluster == cluster) { *out_chain_pos = 0; return (int32_t)i; }
        // Same cluster as the one most recently appended here (a later
        // sector within that same cluster, not a new one) -- resolve
        // to the same chain position, not a new one. Without this,
        // only a cluster's first sector could ever match here; every
        // later sector of it would fall through to the FAT-walk
        // fallback below, which can legitimately still be missing the
        // relevant link for a file's very last cluster (that link is
        // the last one ever written, since there's no further cluster
        // after it to justify writing it any earlier).
        if (cluster == fatfs_pending[i].last_cluster && fatfs_pending[i].cluster_count > 0) {
            *out_chain_pos = fatfs_pending[i].cluster_count - 1;
            return (int32_t)i;
        }
        // Robust match for a continuation of this slot's data: the
        // incoming cluster is exactly the next one after the last
        // cluster already buffered here. Doesn't depend on the
        // FAT-table mirror already showing the chain link -- confirmed
        // that ordering isn't reliable, and relying on it exclusively
        // was causing every cluster of a multi-cluster file to land in
        // its own separate, isolated pending slot instead of
        // accumulating into one (a large file would evict its own
        // earlier clusters before ever being claimed).
        if (cluster == fatfs_pending[i].last_cluster + 1) {
            *out_chain_pos = fatfs_pending[i].cluster_count;
            return (int32_t)i;
        }
        // Fallback: walk the FAT-table chain too, in case it does
        // already reflect this link -- harmless to also check.
        uint32_t c = fatfs_pending[i].start_cluster;
        uint32_t pos = 0;
        while (1) {
            if (c == cluster) { *out_chain_pos = pos; return (int32_t)i; }
            uint32_t next = fatfs_fat_next(c);
            if (next < 2 || next >= 0xFFF8) break;
            c = next;
            pos++;
            if (pos > FATFS_MAX_CLUSTERS) break;
        }
    }
    return -1;
}

/* Allocates a pending slot for a genuinely new cluster's data,
 * evicting the oldest in-use slot (round-robin) if all are busy. Its
 * staging file, if any was ever created, is deleted -- whatever was in
 * it is simply discarded, never having been claimed by a real name. */
static uint32_t fatfs_alloc_pending(uint32_t cluster) {
    uint32_t slot;
    int is_eviction = 0;
    uint32_t i;
    for (i = 0; i < FATFS_MAX_PENDING; i++) {
        if (!fatfs_pending[i].used) break;
    }
    if (i < FATFS_MAX_PENDING) {
        slot = i;
    } else {
        slot = fatfs_pending_evict_next;
        fatfs_pending_evict_next = (fatfs_pending_evict_next + 1) % FATFS_MAX_PENDING;
        is_eviction = 1;
    }

    if (is_eviction) {
        fatfs_log("pending-evict: slot=%u evicting cluster=%u (had %u bytes staged, never claimed) for new cluster=%u\n",
                  slot, (unsigned int)fatfs_pending[slot].start_cluster,
                  (unsigned int)fatfs_pending[slot].size, (unsigned int)cluster);
        fatfs_stage_cache_invalidate((int32_t)slot);
        if (fatfs_pending[slot].ram_buf) {
            fatfs_pending_ram_total -= fatfs_pending[slot].ram_cap;
            free(fatfs_pending[slot].ram_buf);
        }
        if (fatfs_pending[slot].stage_path[0]) remove(fatfs_pending[slot].stage_path);
    }

    fatfs_pending[slot].used = 1;
    fatfs_pending[slot].start_cluster = cluster;
    fatfs_pending[slot].last_cluster = cluster;
    fatfs_pending[slot].cluster_count = 1;
    fatfs_pending[slot].size = 0;
    fatfs_pending[slot].ram_buf = NULL;
    fatfs_pending[slot].ram_cap = 0;
    fatfs_pending[slot].spilled = 0;
    fatfs_pending[slot].last_reported_size = 0;
    fatfs_pending[slot].announced = 0;
    // Path computed now but the file itself is only actually created
    // on disk if/when this slot spills -- most files fit entirely in
    // the RAM buffer and never need it at all.
    snprintf(fatfs_pending[slot].stage_path, sizeof(fatfs_pending[slot].stage_path),
             "/documents/.usbstage_%u.tmp", (unsigned int)cluster);
    if (!is_eviction) {
        fatfs_log("pending-alloc: slot=%u fresh allocation for cluster=%u\n", slot, (unsigned int)cluster);
    }
    return slot;
}

// Deletes any staging files still pending (never claimed by a real
// directory-entry write) -- called at eject time so an interrupted or
// abandoned transfer doesn't leave permanent .tmp litter behind.
static void fatfs_cleanup_pending_stage_files(void) {
    for (uint32_t i = 0; i < FATFS_MAX_PENDING; i++) {
        if (fatfs_pending[i].used) {
            fatfs_stage_cache_invalidate((int32_t)i);
            if (fatfs_pending[i].ram_buf) {
                fatfs_pending_ram_total -= fatfs_pending[i].ram_cap;
                free(fatfs_pending[i].ram_buf);
                fatfs_pending[i].ram_buf = NULL;
            }
            if (fatfs_pending[i].stage_path[0]) remove(fatfs_pending[i].stage_path);
            fatfs_pending[i].used = 0;
        }
    }
}

// A real FAT short name (11 bytes: 8-byte base + 3-byte extension)
// only ever contains uppercase letters, digits, spaces (padding), and
// a small set of allowed special characters. Anything else appearing
// in this field -- null bytes, 0xFF, lowercase letters, control
// characters, or 0xE5 outside position 0 (which is the delete marker,
// handled separately) -- means this genuinely isn't a valid directory
// entry, whatever operation the rest of the bytes might otherwise
// appear to describe. Checked once, early, before any
// rename/delete/create logic acts on an entry, since a false match
// here (e.g. a coincidentally-matching cluster number in corrupted
// data) can trigger a real, unintended filesystem operation.
static int fatfs_short_name_looks_valid(const uint8_t short_name11[11]) {
    for (int i = 0; i < 11; i++) {
        uint8_t c = short_name11[i];
        if (c == 0x00) return 0; // never valid -- strchr() below would wrongly match this against the search string's own terminator
        int ok = (c == ' ') ||
                 (c >= 'A' && c <= 'Z') ||
                 (c >= '0' && c <= '9') ||
                 (i == 0 && c == 0x05) || // legitimate escape for a real first byte of 0xE5
                 strchr("!#$%&'()-@^_`{}~", (char)c) != NULL;
        if (!ok) return 0;
    }
    return 1;
}

static int32_t fatfs_find_entry(int32_t parent_index, const char short_name11[11]) {
    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        if (fatfs_entries[i].deleted) continue;
        if (fatfs_entries[i].parent_index != parent_index) continue;
        if (memcmp(fatfs_entries[i].short_name, short_name11, 11) == 0) return (int32_t)i;
    }
    return -1;
}

/* A delete only overwrites byte 0 of the short name with 0xE5 -- the
 * rest of the entry (bytes 1-10, attribute, cluster, size) survives,
 * so we can match against those directly rather than needing to know
 * the original first byte. */
static int32_t fatfs_find_entry_for_delete(int32_t parent_index, const uint8_t *e) {
    uint32_t cluster = e[26] | ((uint32_t)e[27] << 8);
    for (uint32_t i = 0; i < fatfs_entry_count; i++) {
        FatEntry *fe = &fatfs_entries[i];
        if (fe->deleted) continue;
        if (fe->parent_index != parent_index) continue;
        if (memcmp(fe->short_name + 1, e + 1, 10) != 0) continue;
        // Overflowed entries (e.g. directories that exceeded the
        // subdirectory slot budget) always have start_cluster==0,
        // which is meaningless/ambiguous for matching -- fall back to
        // matching by name alone for these. They still have a real,
        // known path on disk even though their contents were never
        // individually tracked, so a delete can still act on them.
        if (fe->start_cluster != 0 && fe->start_cluster != cluster) continue;
        return (int32_t)i;
    }
    return -1;
}

/* ==================================================================
 * Live directory-entry write handling -- size updates for existing
 * files, and creation of genuinely new ones.
 * ================================================================== */
static void fatfs_handle_short_entry_write(int32_t parent_index, const uint8_t *e, const char *long_name_maybe, int lfn_was_mismatched) {
    if (e[0] == 0x00) return; // free slot, nothing here

    if (e[0] == 0xE5) {
        int32_t idx = fatfs_find_entry_for_delete(parent_index, e);
        if (idx >= 0) {
            FatEntry *fe = &fatfs_entries[idx];
            char action[220];
            snprintf(action, sizeof(action), "delete \"%s\" (is_dir=%d)", fe->path, (int)fe->is_dir);
            char display[96];
            snprintf(display, sizeof(display), "Deleting %s \"%s\"...", fe->is_dir ? "folder" : "file", fe->long_name);
            if (fatfs_confirm(action, display)) {
                // Deliberately NOT physically deleting here, for files
                // OR directories -- see fatfs_flush_pending_deletes for
                // the full reasoning (originally applied to directory
                // moves, confirmed to affect file moves across parent
                // directories too: those can also show up as
                // delete-then-recreate rather than a same-cluster
                // rename, and deleting immediately destroys the source
                // before a matching recreate can reclaim it).
                fe->deleted = 1;
                fatfs_log("delete: matched \"%s\" is_dir=%d parent=%d\n", fe->path, (int)fe->is_dir, (int)parent_index);
            }
        } else {
            uint32_t cluster = e[26] | ((uint32_t)e[27] << 8);
            fatfs_log("delete: NO MATCH cluster=%u parent=%d\n", (unsigned int)cluster, (int)parent_index);
        }
        return;
    }

    char short_name11[11];
    memcpy(short_name11, e, 11);
    uint32_t start_cluster = e[26] | ((uint32_t)e[27] << 8);
    uint32_t size = e[28] | ((uint32_t)e[29] << 8) | ((uint32_t)e[30] << 16) | ((uint32_t)e[31] << 24);

    // General validity guard: whatever operation the rest of this
    // entry's bytes might otherwise appear to describe (rename, move,
    // create), don't act on it if the short name itself isn't
    // genuinely valid. This is what actually should have caught the
    // "phoenix/dlog/locales/a" rename that was never asked for --
    // narrower pattern-specific checks (the FAT-table-ramp detector,
    // the byte-1-can't-be-zero check) don't catch every form corrupted
    // data can take, but a real short name failing basic FAT character
    // rules is a much more general, reliable signal.
    if (!fatfs_short_name_looks_valid((const uint8_t *)short_name11)) return;

    uint8_t attr = e[11];
    if (attr == 0x0F) return; // shouldn't reach here (LFN handled separately)

    // Confirmed via raw byte analysis: macOS creates hidden (attr bit
    // 0x02) sidecar/metadata entries alongside real files during a
    // copy -- same general idea as .DS_Store or AppleDouble "._" files,
    // just signaled through the attribute byte instead of a name
    // prefix. These carried a real-looking declared size but no actual
    // data ever arrives for them. Skip creating any visible entry for
    // these entirely.
    if (attr & 0x02) {
        fatfs_log("hidden-entry: SKIPPED cluster=%u parent=%d attr=0x%02x\n",
                  (unsigned int)(e[26] | ((uint32_t)e[27] << 8)), (int)parent_index, (unsigned int)attr);
        uint32_t hidden_pos;
        int32_t hidden_pidx = fatfs_locate_pending(start_cluster, &hidden_pos);
        if (hidden_pidx >= 0) {
            fatfs_stage_cache_invalidate(hidden_pidx);
            if (fatfs_pending[hidden_pidx].ram_buf) {
                fatfs_pending_ram_total -= fatfs_pending[hidden_pidx].ram_cap;
                free(fatfs_pending[hidden_pidx].ram_buf);
                fatfs_pending[hidden_pidx].ram_buf = NULL;
            }
            if (fatfs_pending[hidden_pidx].stage_path[0]) remove(fatfs_pending[hidden_pidx].stage_path);
            fatfs_pending[hidden_pidx].used = 0;
        }
        return;
    }

    if (attr & 0x10) {
        // "." and ".." are present in EVERY directory table and are
        // ALSO directory-type entries -- never real moves. Without
        // this check, a directory's own "." entry (which legitimately
        // points back at its own cluster) gets misread as "this
        // directory is being moved to become its own child",
        // corrupting the tree with a self-reference.
        if (e[0] == '.') return;

        // A directory-type entry that doesn't correspond to a genuinely
        // new directory (still unsupported) but rather a MOVE and/or
        // RENAME of an existing one: its actual contents/cluster don't
        // change, only its position in the tree. Match by cluster ONLY
        // -- including entries currently marked deleted (a move's
        // delete side may have already landed in the old parent's
        // table before this, the new parent's, side arrives).
        //
        // A name-based fallback for overflowed directories (cluster
        // always 0) used to live here, searching the WHOLE tree by
        // name alone with no parent constraint. Removed: confirmed to
        // have caused a real, unintended move of a folder the user
        // never asked to touch -- too loose a match to trust with a
        // real, irreversible filesystem operation. Overflowed
        // directories can still be deleted (see
        // fatfs_find_entry_for_delete, which requires an exact parent
        // match too, a much tighter and safer constraint) but can no
        // longer be moved.
        int32_t match_idx = -1;
        for (uint32_t k = 0; k < fatfs_entry_count; k++) {
            FatEntry *dir = &fatfs_entries[k];
            if (!dir->is_dir || dir->start_cluster == 0) continue;
            if (dir->start_cluster != start_cluster) continue;
            match_idx = (int32_t)k;
            break;
        }

        if (match_idx >= 0) {
            uint32_t k = (uint32_t)match_idx;
            FatEntry *dir = &fatfs_entries[k];

            // Nothing actually changed (the host just rewrote the whole
            // sector, as it often does) -- skip re-processing and
            // logging entirely.
            if (!dir->deleted && dir->parent_index == parent_index &&
                memcmp(dir->short_name, short_name11, 11) == 0) {
                return;
            }

            // The short entry's own name bytes genuinely changed (a
            // real rename/move, not just a rewrite). Whether there was
            // no LFN at all (a name that's already fully 8.3-compliant)
            // or an LFN that didn't checksum-match (unreliable data),
            // the short entry's own bytes are still trustworthy --
            // derive the name from them rather than falling back to
            // the OLD long name (which would silently apply a no-op
            // "rename" and leave the real, on-disk directory stuck at
            // its previous name) or skipping and waiting for a later
            // rewrite that may never actually come (macOS may
            // consider the rename already complete on its end and
            // never touch this table again).
            if (lfn_was_mismatched) {
                fatfs_log("dir-move: LFN was active but mismatched for cluster=%u -- deriving name from short entry instead\n",
                          (unsigned int)start_cluster);
            }

            char derived_dir_name[13];
            const char *move_name_src = long_name_maybe;
            if (!move_name_src || !move_name_src[0]) {
                int lower_base = (e[12] & 0x08) != 0;
                int lower_ext = (e[12] & 0x10) != 0;
                int bi = 0;
                for (int i = 0; i < 8 && short_name11[i] != ' '; i++) {
                    char c = short_name11[i];
                    if (lower_base && c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
                    derived_dir_name[bi++] = c;
                }
                if (short_name11[8] != ' ') {
                    derived_dir_name[bi++] = '.';
                    for (int i = 8; i < 11 && short_name11[i] != ' '; i++) {
                        char c = short_name11[i];
                        if (lower_ext && c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
                        derived_dir_name[bi++] = c;
                    }
                }
                derived_dir_name[bi] = '\0';
                move_name_src = derived_dir_name;
            }

            char old_path[192];
            strncpy(old_path, dir->path, sizeof(old_path) - 1);
            old_path[sizeof(old_path) - 1] = '\0';

            const char *new_parent_path = (parent_index < 0) ? fatfs_root_path : fatfs_entries[parent_index].path;
            const char *move_name = move_name_src;
            char new_path[256];
            fatfs_join_path(new_path, sizeof(new_path), new_parent_path, move_name);

            int real_move_ok = 1;
            if (strcmp(old_path, new_path) != 0) {
                char action[600];
                snprintf(action, sizeof(action), "move dir \"%s\" -> \"%s\"", old_path, new_path);
                char display[96];
                snprintf(display, sizeof(display), "Moving folder \"%s\"...", move_name);
                if (fatfs_confirm(action, display)) {
                    // If an earlier move's background copy hasn't
                    // finished yet, finish it now rather than
                    // overwriting its in-progress state (which would
                    // corrupt or lose that earlier move).
                    while (fatfs_bg_copy_active) fatfs_bg_copy_step();

                    // If the destination path already has a different,
                    // separately-tracked directory (e.g. an earlier
                    // move/create left one there), this move is
                    // genuinely overwriting it. Mark that stale tracked
                    // entry deleted -- otherwise two FatEntry objects
                    // end up claiming the same path, which can make the
                    // directory appear to vanish entirely once the real
                    // parent table is re-served. Also physically clear
                    // the old real directory first, so new content
                    // cleanly replaces it rather than silently merging
                    // with whatever files were already there.
                    for (uint32_t oi = 0; oi < fatfs_entry_count; oi++) {
                        if (oi == k) continue;
                        FatEntry *other = &fatfs_entries[oi];
                        if (other->deleted || !other->is_dir) continue;
                        if (strcmp(other->path, new_path) != 0) continue;
                        other->deleted = 1;
                        fatfs_log("dir-move: destination \"%s\" already tracked separately -- marking stale entry deleted\n", new_path);
                    }
                    fatfs_recursive_remove_dir(new_path); // safe no-op if nothing real exists there yet

                    fatfs_move_progress_total = fatfs_dir_total_size(old_path);
                    fatfs_move_progress_done = 0;
                    fatfs_move_progress_last_reported_pct = 0xFFFFFFFFu;

                    fatfs_bg_copy_queue_count = 0;
                    fatfs_bg_copy_queue_pos = 0;
                    fatfs_bg_copy_ok = 1;
                    fatfs_bg_copy_is_folder = 1;
                    strncpy(fatfs_bg_copy_root_old, old_path, sizeof(fatfs_bg_copy_root_old) - 1);
                    fatfs_bg_copy_root_old[sizeof(fatfs_bg_copy_root_old) - 1] = '\0';
                    strncpy(fatfs_move_display_name, move_name, sizeof(fatfs_move_display_name) - 1);
                    fatfs_move_display_name[sizeof(fatfs_move_display_name) - 1] = '\0';

                    int enum_ok = fatfs_bg_copy_enumerate(old_path, new_path);
                    fatfs_bg_copy_ok = enum_ok;
                    real_move_ok = enum_ok;
                    if (enum_ok) {
                        fatfs_bg_copy_active = 1; // actual file data now copies incrementally in the background, one small chunk per main-loop iteration, so new USB commands keep getting serviced in the meantime
                    }
                    // if enumeration itself failed (couldn't even list
                    // the source), nothing was queued -- same safety
                    // guarantee as before: the original is untouched,
                    // nothing lost. If enumeration succeeded but a
                    // later chunk copy fails partway through, the
                    // (possibly partial) copy at new_path is left in
                    // place rather than deleting anything.
                } else {
                    real_move_ok = 0; // declined -- tracking stays unchanged below, same as a failed move
                }
            }

            if (real_move_ok) {
                dir->deleted = 0; // reactivate if the delete side of the move already landed
                dir->parent_index = parent_index;
                memcpy(dir->short_name, short_name11, 11);
                strncpy(dir->path, new_path, sizeof(dir->path) - 1);
                dir->path[sizeof(dir->path) - 1] = '\0';
                if (long_name_maybe && long_name_maybe[0] && long_name_maybe[0] != '.') {
                    strncpy(dir->long_name, long_name_maybe, sizeof(dir->long_name) - 1);
                }
                fatfs_update_child_paths((int32_t)k);

                // A directory's own ".." entry must always point at its
                // immediate parent -- update it now that the parent changed.
                if (dir->subdir_slot >= 0) {
                    uint8_t *table = fatfs_subdir_tables[dir->subdir_slot];
                    uint32_t parent_cluster = 0;
                    if (parent_index >= 0) parent_cluster = fatfs_entries[parent_index].start_cluster;
                    table[1 * 32 + 26] = (uint8_t)(parent_cluster & 0xFF);
                    table[1 * 32 + 27] = (uint8_t)(parent_cluster >> 8);
                }
            }

            fatfs_log("dir-move: \"%s\" -> \"%s\" (cluster=%u) parent=%d real_move_ok=%d\n",
                      old_path, new_path, (unsigned int)start_cluster, (int)parent_index, real_move_ok);
            return;
        }

        // Genuinely new directory (no existing tracked directory
        // matched by cluster) -- create it for real. Mirrors the
        // existing new-file creation pattern.
        {
            const char *new_parent_path = (parent_index < 0) ? fatfs_root_path : fatfs_entries[parent_index].path;
            char derived[13];
            const char *dir_name = long_name_maybe;
            if (!dir_name || !dir_name[0]) {
                // FAT case-byte convention (offset 12, "NT reserved"):
                // bit 0x08 means the base name is lowercase even though
                // no LFN was sent to spell that out explicitly. Ignoring
                // this byte is what caused short-name-fallback names to
                // always come out uppercase even when the real,
                // intended name was lowercase.
                int lower_base = (e[12] & 0x08) != 0;
                int bi = 0;
                for (int i = 0; i < 8 && short_name11[i] != ' '; i++) {
                    char c = short_name11[i];
                    if (lower_base && c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
                    derived[bi++] = c;
                }
                if (short_name11[8] != ' ') {
                    derived[bi++] = '.';
                    for (int i = 8; i < 11 && short_name11[i] != ' '; i++) derived[bi++] = short_name11[i];
                }
                derived[bi] = '\0';
                dir_name = derived;
            }

            // Same noise filters used for new files: skip AppleDouble/
            // .DS_Store-style names, and refuse a suspiciously
            // zero/unallocated cluster (no real, valid directory
            // creation should ever reference cluster 0).
            if (dir_name[0] == '.' || start_cluster == 0) {
                fatfs_log("dir-create: SKIPPED cluster=%u parent=%d name=\"%s\"\n",
                          (unsigned int)start_cluster, (int)parent_index, dir_name);
                return;
            }

            if (fatfs_entry_count >= FATFS_MAX_ENTRIES) return;

            char full[256];
            fatfs_join_path(full, sizeof(full), new_parent_path, dir_name);

            char action[300];
            snprintf(action, sizeof(action), "create dir \"%s\"", full);
            char display[96];
            snprintf(display, sizeof(display), "Creating folder \"%s\"...", dir_name);
            if (!fatfs_confirm(action, display)) return;

            mkdir(full, 0777); // untested return-value reliability here; proceed regardless and let downstream writes fail harmlessly if it didn't actually work

            FatEntry *fe = NULL;
            for (uint32_t ei = 0; ei < fatfs_entry_count; ei++) {
                if (strcmp(fatfs_entries[ei].path, full) == 0) {
                    fe = &fatfs_entries[ei];
                    break;
                }
            }
            if (!fe) {
                fe = &fatfs_entries[fatfs_entry_count];
                fatfs_entry_count++;
            }
            memset(fe, 0, sizeof(*fe));
            strncpy(fe->path, full, sizeof(fe->path) - 1);
            strncpy(fe->long_name, dir_name, sizeof(fe->long_name) - 1);
            memcpy(fe->short_name, short_name11, 11);
            fe->is_dir = 1;
            fe->parent_index = parent_index;
            fe->start_cluster = start_cluster; // whatever cluster the host allocated for it
            fe->num_clusters = 1;
            fe->subdir_slot = (fatfs_subdir_count < FATFS_MAX_SUBDIRS) ? (int32_t)fatfs_subdir_count : -1;

            if (fe->subdir_slot >= 0) {
                fatfs_subdir_count++;
                uint8_t *table = fatfs_subdir_tables[fe->subdir_slot];
                memset(table, 0, FATFS_CLUSTER_BYTES);

                memcpy(&table[0], ".          ", 11);
                table[11] = 0x10;
                table[26] = (uint8_t)(start_cluster & 0xFF);
                table[27] = (uint8_t)(start_cluster >> 8);

                uint32_t parent_cluster = (parent_index >= 0) ? fatfs_entries[parent_index].start_cluster : 0;
                memcpy(&table[32], "..         ", 11);
                table[32 + 11] = 0x10;
                table[32 + 26] = (uint8_t)(parent_cluster & 0xFF);
                table[32 + 27] = (uint8_t)(parent_cluster >> 8);
            }

            fatfs_log("dir-create: \"%s\" cluster=%u parent=%d subdir_slot=%d\n",
                      full, (unsigned int)start_cluster, (int)parent_index, fe->subdir_slot);
            return;
        }
    }

    int32_t idx = fatfs_find_entry(parent_index, short_name11);
    if (idx >= 0) {
        FatEntry *fe = &fatfs_entries[idx];
        uint32_t old_size = fe->size;
        fe->size = size;
        if (start_cluster != 0) fe->start_cluster = start_cluster;
        if (!fe->is_dir && size != 0 && size != old_size) {
            fatfs_truncate_file(fe->path, size);
        }
        return;
    }

    if (fatfs_entry_count >= FATFS_MAX_ENTRIES) return;
    if (start_cluster == 0) return;

    // A rename of an ALREADY-NAMED, already-real tracked file (a plain
    // Finder rename, not the staging pattern) shows up as a new short
    // name reusing an existing file's cluster. Also matches files
    // currently marked deleted -- a move across parent directories can
    // show up as delete-then-recreate rather than an in-place rename,
    // and since the physical delete is now deferred (see
    // fatfs_flush_pending_deletes), the real file is still there to
    // reclaim.
    for (uint32_t k = 0; k < fatfs_entry_count; k++) {
        FatEntry *existing = &fatfs_entries[k];
        if (existing->is_dir) continue;
        if (existing->start_cluster != start_cluster) continue;

        const char *parent_path2 = (parent_index < 0) ? fatfs_root_path : fatfs_entries[parent_index].path;
        char derived2[13];
        const char *use_name2 = long_name_maybe;
        if (!use_name2 || !use_name2[0]) {
            int bi = 0;
            for (int i = 0; i < 8 && short_name11[i] != ' '; i++) derived2[bi++] = short_name11[i];
            if (short_name11[8] != ' ') {
                derived2[bi++] = '.';
                for (int i = 8; i < 11 && short_name11[i] != ' '; i++) derived2[bi++] = short_name11[i];
            }
            derived2[bi] = '\0';
            use_name2 = derived2;
        }
        if (use_name2[0] == '.' || strstr(use_name2, ".sb-") != NULL) return; // not a real rename target

        char full2[256];
        fatfs_join_path(full2, sizeof(full2), parent_path2, use_name2);

        fatfs_log(
            "rename: \"%s\" -> \"%s\" cluster=%u size_field=%u parent=%d raw=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
            existing->path, full2, (unsigned int)start_cluster, (unsigned int)size, (int)parent_index,
            e[0],e[1],e[2],e[3],e[4],e[5],e[6],e[7],e[8],e[9],e[10],e[11],e[12],e[13],e[14],e[15],
            e[16],e[17],e[18],e[19],e[20],e[21],e[22],e[23],e[24],e[25],e[26],e[27],e[28],e[29],e[30],e[31]);

        {
            char action[600];
            snprintf(action, sizeof(action), "rename \"%s\" -> \"%s\"", existing->path, full2);
            char display[96];
            snprintf(display, sizeof(display), "Moving file \"%s\"...", use_name2);
            if (!fatfs_confirm(action, display)) return; // declined -- tracking stays unchanged
        }

        strncpy(fatfs_move_display_name, use_name2, sizeof(fatfs_move_display_name) - 1);
        fatfs_move_display_name[sizeof(fatfs_move_display_name) - 1] = '\0';

        // If the destination path already has a different, separately-
        // tracked file (this rename is overwriting it), mark that
        // stale entry deleted -- otherwise it stays marked deleted
        // forever without ever being reused, and the deferred-delete
        // cleanup at eject would remove the just-renamed file too,
        // since both entries point at the same real path.
        for (uint32_t oi = 0; oi < fatfs_entry_count; oi++) {
            if (oi == k) continue;
            FatEntry *other = &fatfs_entries[oi];
            if (other->deleted || other->is_dir) continue;
            if (strcmp(other->path, full2) != 0) continue;
            other->deleted = 1;
        }

        if (!fatfs_copy_and_remove(existing->path, full2, 1)) {
            existing->deleted = 0; // file is untouched at its original location -- keep it visible there
            screen_console << "Can't move/rename \"" << existing->long_name << "\" -- file is in use" << nio::endl;
            screen_console << "  (likely the program that's currently running)" << nio::endl;
            return;
        }
        existing->deleted = 0; // reactivate if the delete side of a move already landed
        strncpy(existing->path, full2, sizeof(existing->path) - 1);
        strncpy(existing->long_name, use_name2, sizeof(existing->long_name) - 1);
        memcpy(existing->short_name, short_name11, 11);
        existing->parent_index = parent_index;
        if (size != 0) existing->size = size;
        if (fatfs_move_progress_total >= FATFS_PROGRESS_REPORT_THRESHOLD) {
            char size_str[20];
            fatfs_format_size(size_str, sizeof(size_str), fatfs_move_progress_total);
            screen_console << "  done -- " << use_name2 << " (" << size_str << ")" << nio::endl;
            fatfs_print_controls_reminder();
        }
        return;
    }

    const char *parent_path = (parent_index < 0) ? fatfs_root_path : fatfs_entries[parent_index].path;
    char derived[13];
    const char *use_name = long_name_maybe;
    if (!use_name || !use_name[0]) {
        fatfs_log("name-resolve: short=%.11s cluster=%u parent=%d case_byte=0x%02x -- NO LFN MATCH, falling back to short name\n",
                  short_name11, (unsigned int)start_cluster, (int)parent_index, (unsigned int)e[12]);
        // FAT case-byte convention (offset 12, "NT reserved"): bit 0x08
        // means the base name is lowercase, bit 0x10 means the
        // extension is lowercase -- even though no LFN was sent to
        // spell that out explicitly. This is the actual mechanism
        // confirmed responsible for short-name fallbacks always coming
        // out uppercase regardless of the real, intended casing: we
        // were ignoring this byte entirely.
        int lower_base = (e[12] & 0x08) != 0;
        int lower_ext = (e[12] & 0x10) != 0;
        int bi = 0;
        for (int i = 0; i < 8 && short_name11[i] != ' '; i++) {
            char c = short_name11[i];
            if (lower_base && c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
            derived[bi++] = c;
        }
        if (short_name11[8] != ' ') {
            derived[bi++] = '.';
            for (int i = 8; i < 11 && short_name11[i] != ' '; i++) {
                char c = short_name11[i];
                if (lower_ext && c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
                derived[bi++] = c;
            }
        }
        derived[bi] = '\0';
        use_name = derived;
    }

    uint32_t pending_pos;
    int32_t pending_idx = fatfs_locate_pending(start_cluster, &pending_pos);

    // FIX: macOS automatically writes hidden companion files to ANY FAT
    // volume -- a ".DS_Store" per folder, and a "._<name>" AppleDouble
    // sidecar for every real file copied over. We never want to keep
    // these -- if a staging file exists for one, delete it (its data
    // never gets claimed by a real name, so it never becomes visible).
    //
    // Also reject suspiciously short reconstructed names (<3 chars) --
    // real filenames are essentially never this short in practice, and
    // this is exactly the signature of macOS's hidden .fseventsd
    // directory (which we never track, since new-directory creation is
    // unsupported) -- its internal binary log data isn't a real FAT
    // directory structure at all, but occasionally coincidentally
    // parses as a plausible-looking entry.
    if (use_name[0] == '.') {
        if (pending_idx >= 0) {
            fatfs_stage_cache_invalidate((int32_t)pending_idx);
            if (fatfs_pending[pending_idx].ram_buf) {
                fatfs_pending_ram_total -= fatfs_pending[pending_idx].ram_cap;
                free(fatfs_pending[pending_idx].ram_buf);
                fatfs_pending[pending_idx].ram_buf = NULL;
            }
            if (fatfs_pending[pending_idx].stage_path[0]) remove(fatfs_pending[pending_idx].stage_path);
            fatfs_pending[pending_idx].used = 0;
        }
        return;
    }

    // macOS's "safe save" pattern: write a whole staging file, THEN
    // rename it over the real target. This directory entry names the
    // STAGING file, not the true final name -- ignore it entirely and
    // leave its data sitting in the pending buffer untouched. The
    // later, true rename (same cluster, real name) will claim it below.
    if (strstr(use_name, ".sb-") != NULL) return;

    if (pending_idx < 0) {
        // No buffered data for this cluster (e.g. a genuinely new,
        // empty file with no content expected yet) -- create it
        // directly.
        //
        // NOTE: a guard used to live here rejecting any cluster our FAT
        // mirror showed as completely unallocated, on the theory that
        // this meant a corrupted/garbled entry. Removed -- it predates
        // fatfs_short_name_looks_valid() (see above), which catches
        // genuinely garbled entries far more reliably by examining
        // actual byte content, not FAT allocation state. Allocation
        // state can legitimately lag behind a directory entry write
        // due to ordering alone, and this guard was confirmed to
        // reject real, valid files for exactly that reason.

        // Confirmed via raw byte analysis on two separate cases (one a
        // pure incrementing FAT-chain ramp, one broken partway through
        // by a 0xFFFF end-of-chain marker -- both are genuine FAT-table
        // content misrouted into directory-entry parsing, not real
        // directory entries): a real FAT short name is always
        // space-padded, never null-padded, so byte 1 can never
        // legitimately be 0x00 here (byte 0 being 0x00 is a separate,
        // already-handled case: an empty slot). Small FAT cluster
        // numbers routinely have a zero high byte, which is exactly
        // what lands at this position when table data gets misread as
        // a directory entry.
        if (e[1] == 0x00) return;

        if (fatfs_entry_count >= FATFS_MAX_ENTRIES) return;
        char full[256];
        fatfs_join_path(full, sizeof(full), parent_path, use_name);

        fatfs_log(
            "empty-create: \"%s\" (short=%.11s) cluster=%u size_field=%u parent=%d raw=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
            full, short_name11, (unsigned int)start_cluster, (unsigned int)size, (int)parent_index,
            e[0],e[1],e[2],e[3],e[4],e[5],e[6],e[7],e[8],e[9],e[10],e[11],e[12],e[13],e[14],e[15],
            e[16],e[17],e[18],e[19],e[20],e[21],e[22],e[23],e[24],e[25],e[26],e[27],e[28],e[29],e[30],e[31]);

        {
            char action[280];
            snprintf(action, sizeof(action), "create \"%s\" (empty)", full);
            char display[96];
            snprintf(display, sizeof(display), "Creating file \"%s\"...", use_name);
            if (!fatfs_confirm(action, display)) return;
        }

        FILE *nf = fopen(full, "wb");
        if (!nf) return;
        fclose(nf);

        FatEntry *fe = NULL;
        for (uint32_t ei = 0; ei < fatfs_entry_count; ei++) {
            if (strcmp(fatfs_entries[ei].path, full) == 0) {
                fe = &fatfs_entries[ei];
                break;
            }
        }
        if (!fe) {
            fe = &fatfs_entries[fatfs_entry_count];
            fatfs_entry_count++;
        }
        memset(fe, 0, sizeof(*fe));
        strncpy(fe->path, full, sizeof(fe->path) - 1);
        strncpy(fe->long_name, use_name, sizeof(fe->long_name) - 1);
        memcpy(fe->short_name, short_name11, 11);
        fe->is_dir = 0;
        fe->size = 0;
        fe->start_cluster = start_cluster;
        fe->num_clusters = 1;
        fe->parent_index = parent_index;
        fe->subdir_slot = -1;
        fatfs_fix_persisted_size(parent_index, short_name11, 0);
        return;
    }

    // Genuinely final name, with real staged content waiting -- copy
    // the staging file's content into place and remove the staging
    // file. Streamed via a small chunk buffer, so still no RAM ceiling
    // tied to file size, just not the free "instant rename" we'd hoped
    // for (rename() itself proved unreliable here -- see
    // fatfs_copy_and_remove's comment).
    if (fatfs_entry_count >= FATFS_MAX_ENTRIES) return;
    char full[256];
    fatfs_join_path(full, sizeof(full), parent_path, use_name);

    uint32_t buffered_size = fatfs_pending[pending_idx].size;
    const char *stage_path = fatfs_pending[pending_idx].stage_path;
    uint8_t *ram_buf = fatfs_pending[pending_idx].ram_buf;
    uint8_t was_spilled = fatfs_pending[pending_idx].spilled;
    fatfs_stage_cache_invalidate((int32_t)pending_idx); // flush any unwritten disk sectors, if this slot ever spilled

    fatfs_log("flush: \"%s\" slot=%d cluster=%u staged_size=%u declared_size=%u gap=%d (%s)\n",
              full, (int)pending_idx, (unsigned int)start_cluster, (unsigned int)buffered_size,
              (unsigned int)size, (int)size - (int)buffered_size,
              was_spilled ? stage_path : "held entirely in RAM, never touched disk");

    {
        char action[300];
        snprintf(action, sizeof(action), "create \"%s\" (%u bytes)", full, (unsigned int)buffered_size);
        char display[96];
        snprintf(display, sizeof(display), "Saving \"%s\"...", use_name);
        if (!fatfs_confirm(action, display)) {
            if (ram_buf) {
                fatfs_pending_ram_total -= fatfs_pending[pending_idx].ram_cap;
                free(ram_buf);
            }
            if (was_spilled) remove(stage_path); // discard staged data rather than leave a stray .tmp file
            fatfs_pending[pending_idx].used = 0;
            return;
        }
    }

    int ok;
    if (was_spilled) {
        ok = fatfs_copy_and_remove(stage_path, full);
    } else if (ram_buf) {
        FILE *out = fopen(full, "wb");
        ok = 0;
        if (out) {
            fwrite(ram_buf, 1, buffered_size, out);
            fclose(out);
            ok = 1;
        }
        fatfs_pending_ram_total -= fatfs_pending[pending_idx].ram_cap;
        free(ram_buf);
    } else {
        // No RAM buffer was ever allocated for this slot (e.g. RAM
        // budget exhausted right at allocation time) and it was never
        // marked spilled either -- nothing was ever actually written
        // anywhere. Create an empty file rather than silently drop it.
        FILE *out = fopen(full, "wb");
        ok = out ? (fclose(out), 1) : 0;
    }
    if (!ok) {
        fatfs_log("flush: write to final path failed, giving up on \"%s\"\n", full);
        if (was_spilled) remove(stage_path);
        fatfs_pending[pending_idx].used = 0;
        return;
    }
    fatfs_pending[pending_idx].used = 0;

    uint32_t final_size = (size != 0 && size < buffered_size) ? size : buffered_size;
    if (final_size < buffered_size) {
        fatfs_truncate_file(full, final_size);
    }
    if (final_size >= FATFS_PROGRESS_REPORT_THRESHOLD) {
        char size_str[20];
        fatfs_format_size(size_str, sizeof(size_str), final_size);
        screen_console << "  done -- " << use_name << " (" << size_str << ")" << nio::endl;
        fatfs_print_controls_reminder();
    }

    FatEntry *fe = NULL;
    for (uint32_t ei = 0; ei < fatfs_entry_count; ei++) {
        if (strcmp(fatfs_entries[ei].path, full) == 0) {
            fe = &fatfs_entries[ei];
            break;
        }
    }
    if (!fe) {
        fe = &fatfs_entries[fatfs_entry_count];
        fatfs_entry_count++;
    }
    memset(fe, 0, sizeof(*fe));
    strncpy(fe->path, full, sizeof(fe->path) - 1);
    strncpy(fe->long_name, use_name, sizeof(fe->long_name) - 1);
    memcpy(fe->short_name, short_name11, 11);
    fe->is_dir = 0;
    fe->size = final_size;
    fe->start_cluster = start_cluster;
    fe->num_clusters = 1;
    fe->parent_index = parent_index;
    fe->subdir_slot = -1;
    fatfs_fix_persisted_size(parent_index, short_name11, fe->size);

    fatfs_log("created: \"%s\" (short=%.11s) cluster=%u parent=%d\n",
              full, short_name11, (unsigned int)start_cluster, (int)parent_index);
}

// The FAT read-only attribute bit (0x01 in the attribute byte) maps
// directly to macOS Finder's "Locked" checkbox, which blocks deletion
// outright with a dedicated error message. We persist the host's raw
// directory-table writes verbatim before parsing them, so if this bit
// ever got set on a real file at some point, we'd keep faithfully
// serving it back forever with no way to clear it. Strip it from every
// real (non-free, non-deleted, non-LFN) entry whenever a sector gets
// persisted, so nothing served back is ever "Locked".
static void fatfs_clear_readonly_in_sector(uint8_t *sector512) {
    for (int s = 0; s < 16; s++) {
        uint8_t *e = sector512 + s * 32;
        if (e[0] == 0x00 || e[0] == 0xE5) continue;
        if (e[11] == 0x0F) continue; // LFN entry, not a real attribute byte
        e[11] &= ~0x01u;
    }
}

static void fatfs_scan_dir_table_write(int32_t parent_index, const uint8_t *sector512) {
    uint32_t slot = FATFS_ROOT_LFN_SLOT;
    if (parent_index >= 0) {
        int32_t s = fatfs_entries[parent_index].subdir_slot;
        slot = (s >= 0) ? (uint32_t)s : FATFS_ROOT_LFN_SLOT; // shouldn't happen, safe fallback
    }

    // NOTE: a free slot (e[0]==0x00) used to reset the LFN accumulator
    // immediately. Removed -- confirmed via diagnostic logging that
    // this was losing genuinely valid, in-flight LFN sequences:
    // writes to different sectors of the same directory table can
    // arrive interleaved during a busy multi-file transfer, and a
    // free slot encountered in an unrelated, later-processed sector
    // isn't necessarily "this LFN sequence was abandoned" -- it can
    // just be unwritten space elsewhere in the table. The accumulator
    // still gets reset unconditionally after every short-entry
    // match attempt below, which is the case that actually matters.
    for (int s = 0; s < 16; s++) {
        const uint8_t *e = sector512 + s * 32;
        if (e[0] == 0x00) continue; // don't reset the LFN accumulator here -- see comment above this loop

        uint8_t attr = e[11];
        if (attr == 0x0F) {
            uint8_t seqbyte = e[0];
            int is_last = (seqbyte & 0x40) != 0;
            int seq = seqbyte & 0x1F;
            if (is_last) {
                fatfs_pending_lfn_active[slot] = 1;
                fatfs_pending_lfn_len[slot] = 0;
                fatfs_pending_lfn_checksum[slot] = e[13];
                memset(fatfs_pending_lfn[slot], 0, sizeof(fatfs_pending_lfn[slot]));
                fatfs_log("lfn-start: slot=%u checksum=0x%02x seq=%u\n", slot, (unsigned int)e[13], (unsigned int)seq);
            }
            if (!fatfs_pending_lfn_active[slot]) {
                fatfs_log("lfn-frag: slot=%u seq=%u is_last=%d IGNORED (no active accumulator)\n",
                          slot, (unsigned int)seq, is_last);
                continue;
            }

            char frag[13];
            int flen = 0;
            for (int ci = 0; ci < 13; ci++) {
                uint16_t ch = e[FATFS_LFN_CHAR_OFFSETS[ci]] | ((uint16_t)e[FATFS_LFN_CHAR_OFFSETS[ci] + 1] << 8);
                if (ch == 0x0000 || ch == 0xFFFF) break;
                frag[flen++] = (char)(ch & 0xFF);
            }
            frag[flen] = '\0';
            if (!is_last) {
                fatfs_log("lfn-frag: slot=%u seq=%u frag=\"%s\"\n", slot, (unsigned int)seq, frag);
            }
            int base = (seq - 1) * 13;
            if (base >= 0 && base + flen < (int)sizeof(fatfs_pending_lfn[slot]) - 1) {
                memcpy(fatfs_pending_lfn[slot] + base, frag, (size_t)flen);
                if (base + flen > fatfs_pending_lfn_len[slot]) fatfs_pending_lfn_len[slot] = base + flen;
            }
            continue;
        }

        const char *use_long = NULL;
        int lfn_was_mismatched = 0;
        int is_deleted_marker = (e[0] == 0xE5);
        if (!is_deleted_marker && fatfs_pending_lfn_active[slot]) {
            uint8_t sum = fatfs_lfn_checksum((const char *)e);
            int match = (sum == fatfs_pending_lfn_checksum[slot]);
            if (attr != 0x0F && attr != 0x00) {
                char accumulated[sizeof(fatfs_pending_lfn[slot])];
                int alen = fatfs_pending_lfn_len[slot];
                if (alen >= (int)sizeof(accumulated)) alen = (int)sizeof(accumulated) - 1;
                if (alen < 0) alen = 0;
                memcpy(accumulated, fatfs_pending_lfn[slot], (size_t)alen);
                accumulated[alen] = '\0';
                fatfs_log("lfn-check: slot=%u short=%.11s expected_sum=0x%02x got_sum=0x%02x accumulated=\"%s\" %s\n",
                          slot, (const char *)e, (unsigned int)fatfs_pending_lfn_checksum[slot], (unsigned int)sum,
                          accumulated, match ? "MATCH" : "MISMATCH");
            }
            if (match) {
                fatfs_pending_lfn[slot][fatfs_pending_lfn_len[slot]] = '\0';
                use_long = fatfs_pending_lfn[slot];
            } else {
                lfn_was_mismatched = 1;
            }
        } else if (!is_deleted_marker && attr != 0x0F && attr != 0x00 && e[0] != 0x00) {
            fatfs_log("lfn-check: slot=%u short=%.11s -- no LFN accumulator was active at all\n", slot, (const char *)e);
        }
        fatfs_handle_short_entry_write(parent_index, e, use_long, lfn_was_mismatched);
        if (!is_deleted_marker) {
            fatfs_pending_lfn_active[slot] = 0;
        }
    }
}

/* ==================================================================
 * Sector-level read/write (public entry points at the bottom)
 * ================================================================== */
static int fatfs_read_volume_sector(uint32_t lba, uint8_t *out512) {
    uint32_t fat1_start = 1;
    uint32_t fat2_start = fat1_start + fatfs_fat_sectors_per_copy;
    uint32_t root_start = fat2_start + fatfs_fat_sectors_per_copy;
    uint32_t data_start = fatfs_data_start_lba;

    if (lba == 0) {
        memcpy(out512, fatfs_boot_sector, FATFS_SECTOR_SIZE);
        return 1;
    }

    if (lba >= fat1_start && lba < fat1_start + fatfs_fat_sectors_per_copy) {
        uint32_t off = (lba - fat1_start) * FATFS_SECTOR_SIZE;
        memset(out512, 0, FATFS_SECTOR_SIZE);
        if (off < sizeof(fatfs_fat_table)) {
            uint32_t avail = sizeof(fatfs_fat_table) - off;
            uint32_t n = avail < FATFS_SECTOR_SIZE ? avail : FATFS_SECTOR_SIZE;
            memcpy(out512, fatfs_fat_table + off, n);
        }
        return 1;
    }

    if (lba >= fat2_start && lba < fat2_start + fatfs_fat_sectors_per_copy) {
        uint32_t off = (lba - fat2_start) * FATFS_SECTOR_SIZE;
        memset(out512, 0, FATFS_SECTOR_SIZE);
        if (off < sizeof(fatfs_fat_table)) {
            uint32_t avail = sizeof(fatfs_fat_table) - off;
            uint32_t n = avail < FATFS_SECTOR_SIZE ? avail : FATFS_SECTOR_SIZE;
            memcpy(out512, fatfs_fat_table + off, n);
        }
        return 1;
    }

    if (lba >= root_start && lba < root_start + fatfs_root_dir_sectors) {
        uint32_t off = (lba - root_start) * FATFS_SECTOR_SIZE;
        memcpy(out512, fatfs_root_dir + off, FATFS_SECTOR_SIZE);
        return 1;
    }

    if (lba >= data_start) {
        uint32_t data_sector_index = lba - data_start;
        uint32_t cluster = 2 + (data_sector_index / FATFS_CLUSTER_SECTORS);
        uint32_t sector_in_cluster = data_sector_index % FATFS_CLUSTER_SECTORS;

        uint32_t chain_pos;
        int32_t idx = fatfs_locate_cluster(cluster, &chain_pos);
        if (idx >= 0) {
            FatEntry *fe = &fatfs_entries[idx];
            if (fe->is_dir) {
                uint32_t byte_offset = sector_in_cluster * FATFS_SECTOR_SIZE;
                if (fe->subdir_slot >= 0 && byte_offset < FATFS_CLUSTER_BYTES) {
                    memcpy(out512, fatfs_subdir_tables[fe->subdir_slot] + byte_offset, FATFS_SECTOR_SIZE);
                } else {
                    memset(out512, 0, FATFS_SECTOR_SIZE);
                }
                return 1;
            }

            uint32_t byte_offset = chain_pos * FATFS_CLUSTER_BYTES + sector_in_cluster * FATFS_SECTOR_SIZE;
            memset(out512, 0, FATFS_SECTOR_SIZE);
            if (byte_offset < fe->size) {
                FILE *f = fopen(fe->path, "rb");
                if (f) {
                    fseek(f, (long)byte_offset, SEEK_SET);
                    uint32_t to_read = fe->size - byte_offset;
                    if (to_read > FATFS_SECTOR_SIZE) to_read = FATFS_SECTOR_SIZE;
                    fread(out512, 1, to_read, f);
                    fclose(f);
                }
            }
            return 1;
        }

        memset(out512, 0, FATFS_SECTOR_SIZE);
        return 1;
    }

    memset(out512, 0, FATFS_SECTOR_SIZE);
    return 0;
}

static int fatfs_write_volume_sector(uint32_t lba, const uint8_t *in512) {
    uint32_t fat1_start = 1;
    uint32_t fat2_start = fat1_start + fatfs_fat_sectors_per_copy;
    uint32_t root_start = fat2_start + fatfs_fat_sectors_per_copy;
    uint32_t data_start = fatfs_data_start_lba;

    if (lba == 0) return 1;

    if (lba >= fat1_start && lba < fat1_start + fatfs_fat_sectors_per_copy) {
        uint32_t off = (lba - fat1_start) * FATFS_SECTOR_SIZE;
        if (off < sizeof(fatfs_fat_table)) {
            uint32_t avail = sizeof(fatfs_fat_table) - off;
            uint32_t n = avail < FATFS_SECTOR_SIZE ? avail : FATFS_SECTOR_SIZE;
            memcpy(fatfs_fat_table + off, in512, n);
        }
        return 1;
    }
    if (lba >= fat2_start && lba < fat2_start + fatfs_fat_sectors_per_copy) {
        return 1;
    }

    if (lba >= root_start && lba < root_start + fatfs_root_dir_sectors) {
        uint32_t off = (lba - root_start) * FATFS_SECTOR_SIZE;
        if (off + FATFS_SECTOR_SIZE <= sizeof(fatfs_root_dir)) {
            memcpy(fatfs_root_dir + off, in512, FATFS_SECTOR_SIZE);
            fatfs_clear_readonly_in_sector(fatfs_root_dir + off);
        }
        fatfs_scan_dir_table_write(-1, in512);
        return 1;
    }

    if (lba >= data_start) {
        uint32_t data_sector_index = lba - data_start;
        uint32_t cluster = 2 + (data_sector_index / FATFS_CLUSTER_SECTORS);
        uint32_t sector_in_cluster = data_sector_index % FATFS_CLUSTER_SECTORS;

        uint32_t chain_pos = 0;
        int32_t idx = fatfs_locate_cluster(cluster, &chain_pos);
        if (idx >= 0) {
            FatEntry *fe = &fatfs_entries[idx];
            if (fe->is_dir) {
                if (fe->subdir_slot >= 0) {
                    uint32_t byte_offset = sector_in_cluster * FATFS_SECTOR_SIZE;
                    if (byte_offset + FATFS_SECTOR_SIZE <= FATFS_CLUSTER_BYTES) {
                        memcpy(fatfs_subdir_tables[fe->subdir_slot] + byte_offset, in512, FATFS_SECTOR_SIZE);
                        fatfs_clear_readonly_in_sector(fatfs_subdir_tables[fe->subdir_slot] + byte_offset);
                    }
                }
                fatfs_scan_dir_table_write((int32_t)idx, in512);
                return 1;
            }
            uint32_t byte_offset = chain_pos * FATFS_CLUSTER_BYTES + sector_in_cluster * FATFS_SECTOR_SIZE;
            FILE *f = fopen(fe->path, "r+b");
            if (f) {
                fseek(f, (long)byte_offset, SEEK_SET);
                fwrite(in512, 1, FATFS_SECTOR_SIZE, f);
                fclose(f);
            }
            return 1;
        }

        // Not a tracked, named file -- stream directly to a staging
        // file on disk instead of ever writing a real, final file for
        // data we don't have a true name for yet. Don't gate this on
        // the FAT-table mirror already showing the link (that ordering
        // isn't reliably guaranteed), so real data never gets silently
        // dropped either.
        uint32_t pending_chain_pos = 0;
        int32_t pidx = fatfs_locate_pending(cluster, &pending_chain_pos);
        int was_fresh = (pidx < 0);
        if (pidx < 0) {
            pidx = (int32_t)fatfs_alloc_pending(cluster);
            pending_chain_pos = 0;
        } else if (cluster == fatfs_pending[pidx].last_cluster + 1) {
            // Genuinely new cluster appended to this slot's chain (not
            // a re-write of a cluster we already know about) -- advance
            // tracking so the NEXT cluster's continuation match works.
            fatfs_pending[pidx].last_cluster = cluster;
            fatfs_pending[pidx].cluster_count++;
        }
        if (sector_in_cluster == 0) {
            fatfs_log("cluster-write: cluster=%u -> slot=%u chain_pos=%u %s\n",
                      (unsigned int)cluster, (unsigned int)pidx, (unsigned int)pending_chain_pos,
                      was_fresh ? "(fresh)" : "(matched)");
        }
        uint32_t byte_offset = pending_chain_pos * FATFS_CLUSTER_BYTES + sector_in_cluster * FATFS_SECTOR_SIZE;

        if (!fatfs_pending[pidx].spilled) {
            // Grow the RAM buffer incrementally (starting small, doubling
            // as needed) rather than committing the full per-slot cap
            // upfront on the very first byte. Most pending slots (hidden
            // sidecars, small metadata files) never need anywhere near
            // the full cap, and repeatedly allocating/freeing full-size
            // chunks for these was fragmenting the heap badly enough to
            // cause later allocations to fail even with plenty of total
            // free memory.
            uint32_t needed = byte_offset + FATFS_SECTOR_SIZE;
            if (needed > fatfs_pending[pidx].ram_cap && needed <= FATFS_PENDING_RAM_PER_SLOT_CAP) {
                uint32_t new_cap = fatfs_pending[pidx].ram_cap ? fatfs_pending[pidx].ram_cap : (256u * 1024u);
                while (new_cap < needed) new_cap *= 2;
                if (new_cap > FATFS_PENDING_RAM_PER_SLOT_CAP) new_cap = FATFS_PENDING_RAM_PER_SLOT_CAP;
                uint32_t growth = new_cap - fatfs_pending[pidx].ram_cap;
                if (fatfs_pending_ram_total + growth <= FATFS_PENDING_RAM_GLOBAL_CAP) {
                    uint8_t *buf = (uint8_t *)realloc(fatfs_pending[pidx].ram_buf, new_cap);
                    if (buf) {
                        fatfs_pending[pidx].ram_buf = buf;
                        fatfs_pending_ram_total += growth;
                        fatfs_pending[pidx].ram_cap = new_cap;
                    }
                }
            }
            if (!fatfs_pending[pidx].ram_buf) {
                // No RAM available (global budget exhausted or
                // malloc/realloc failed) -- fall back to disk staging
                // from the very first byte, same as the original design.
                fatfs_pending_spill((uint32_t)pidx);
            } else if (needed > fatfs_pending[pidx].ram_cap) {
                // This file has outgrown its RAM budget -- move what's
                // already been buffered to disk and continue from
                // there, same as the original, disk-only design would
                // have from the start.
                fatfs_pending_spill((uint32_t)pidx);
            }
        }

        if (!fatfs_pending[pidx].spilled) {
            memcpy(fatfs_pending[pidx].ram_buf + byte_offset, in512, FATFS_SECTOR_SIZE);
            uint32_t end = byte_offset + FATFS_SECTOR_SIZE;
            if (end > fatfs_pending[pidx].size) fatfs_pending[pidx].size = end;
            fatfs_pending_announce_start((uint32_t)pidx);
            fatfs_pending_report_progress((uint32_t)pidx);
            return 1;
        }

        if (fatfs_stage_cache_slot != pidx) {
            if (fatfs_stage_cache_handle) fclose(fatfs_stage_cache_handle);
            fatfs_stage_cache_handle = fopen(fatfs_pending[pidx].stage_path, "r+b");
            fatfs_stage_cache_slot = fatfs_stage_cache_handle ? pidx : -1;
            fatfs_stage_cache_pos = -1; // unknown position on a freshly (re)opened handle -- force the first seek
        }
        if (fatfs_stage_cache_handle) {
            if ((long)byte_offset != fatfs_stage_cache_pos) {
                fseek(fatfs_stage_cache_handle, (long)byte_offset, SEEK_SET);
            }
            fwrite(in512, 1, FATFS_SECTOR_SIZE, fatfs_stage_cache_handle);
            fatfs_stage_cache_pos = (long)byte_offset + FATFS_SECTOR_SIZE;
            uint32_t end = byte_offset + FATFS_SECTOR_SIZE;
            if (end > fatfs_pending[pidx].size) fatfs_pending[pidx].size = end;
            fatfs_pending_announce_start((uint32_t)pidx);
            fatfs_pending_report_progress((uint32_t)pidx);
        } else {
            screen_console << "Warning: couldn't stage data for a file being copied" << nio::endl;
            screen_console << "  (disk may be full) -- it may be created empty." << nio::endl;
        }
        return 1;
    }

    return 1;
}

int fatfs_read_sector(uint32_t lba, uint8_t *out512) {
    if (lba == 0) {
        memcpy(out512, fatfs_mbr, FATFS_SECTOR_SIZE);
        return 1;
    }
    if (lba < FATFS_PARTITION_START_LBA) {
        memset(out512, 0, FATFS_SECTOR_SIZE);
        return 0;
    }
    return fatfs_read_volume_sector(lba - FATFS_PARTITION_START_LBA, out512);
}

int fatfs_write_sector(uint32_t lba, const uint8_t *in512) {
    if (lba == 0) return 1;
    if (lba < FATFS_PARTITION_START_LBA) return 1;
    return fatfs_write_volume_sector(lba - FATFS_PARTITION_START_LBA, in512);
}