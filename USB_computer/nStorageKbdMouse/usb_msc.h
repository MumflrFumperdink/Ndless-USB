#pragma once
// Debug output (screen console + log file) for usb_dma.h's dbg_print()
// -- silenced for normal use. Uncomment to re-enable for debugging.
// #define DEBUG_USB
#include "usb_dma.h"
#include <libndls.h>

/* ------------------------------------------------------------------ *
 * USB MASS STORAGE (Bulk-Only Transport + minimal SCSI)
 *
 * Implements the USB Mass Storage Bulk-Only Transport protocol and
 * enough of the SCSI command set (INQUIRY, READ/WRITE CAPACITY,
 * MODE SENSE, TEST UNIT READY, READ(10)/WRITE(10), REQUEST SENSE) for
 * host OSes to mount this as a normal, writable removable disk. The
 * actual filesystem content is synthesized on the fly by usb_fatfs.h
 * from a real folder on the Nspire.
 *
 * Plugs into usb_dma.h's callback mechanisms
 * (set_usb_rx_callback / set_usb_unhandled_control_callback) rather
 * than running its own separate control-endpoint dispatcher: main()
 * registers the two callbacks, then loops calling
 * service_control_endpoint() + service_bulk_endpoints() every
 * iteration. usb_dma.h's own bulk-OUT free-running arm/receive/re-arm
 * cycle handles CBW reception automatically -- msc_on_cbw_received()
 * just gets called with whatever arrived.
 * ------------------------------------------------------------------ */

/* ---- MSC-specific descriptors still needed ----
 * usb_dma.h's own device_descriptor/config_descriptor already contain
 * MSC's interface (bInterfaceClass=0x08/0x06/0x50), so GET_DESCRIPTOR
 * for device/config types is handled natively -- nothing extra needed
 * here for those. String descriptors are NOT handled here in the
 * merged composite build: the whole device (MSC + HID keyboard + HID
 * mouse) shares one unified string table, defined in
 * usb_hid_mouse_kbd.h, since a composite device has one string table
 * total, not one per subsystem. */

/* ---- Dedicated descriptors MSC still needs of its own ----
 * dtd_bulk_tx (data-phase AND CSW sends -- same descriptor, reused
 * sequentially, same as it's used for every other command) and
 * dtd_bulk_rx/rx_buffer (CBW reception, via the free-running callback
 * mechanism) are reused directly from usb_dma.h.
 *
 * REMOVED: the separate msc_dtd_csw pipelining descriptor. It existed
 * to pre-stage the CSW on a second descriptor while the data phase was
 * still using the first, based on a theory (from an earlier packet
 * capture) that the host's hardware NAK-retry budget was only
 * microseconds wide. A later, more direct logic-analyzer capture
 * disproved that: the actual failure is a persistent lockup lasting
 * hundreds of milliseconds with zero self-recovery, not a brief race
 * -- so pre-staging a second descriptor to "beat the clock" was never
 * actually addressing the real failure mode. This also tests directly
 * whether switching between two different physical descriptor
 * addresses for the same endpoint was itself contributing to the
 * problem.
 *
 * msc_dtd_rx / xfer_buf still needed: WRITE(10) receives up to 512-
 * byte sectors, larger than usb_dma.h's fixed 256-byte rx_buffer
 * (sized for its own simple test payloads) -- so WRITE data reception
 * uses its own separate descriptor and buffer, only ever touched
 * synchronously from inside msc_on_cbw_received(), never conflicting
 * with the free-running CBW reception on dtd_bulk_rx. */
static uint8_t msc_dtd_rx_raw[sizeof(DeviceTransferDescriptor) + DTD_ALIGN];
static DeviceTransferDescriptor *msc_dtd_rx;
static int msc_dtds_initialized = 0;
static int msc_quiet_tx = 0; // set by msc_on_cbw_received() to silence noisy proven-good commands
static uint32_t msc_last_complete_iters = 0; // diagnostic: iterations the last EP1 IN completion wait actually took
static uint64_t msc_total_wait_iters = 0; // cumulative iterations spent waiting across all DMA operations (both send and receive) -- a relative time proxy, since no hardware clock is available
uint64_t main_loop_iters = 0; // incremented once per pass through main.cpp's USB polling loop -- measures the gap between consecutive CBW arrivals, independent of anything happening inside an already-dispatched command's data phase
static char msc_dbg_buf[90];

#define XFER_BUF_ALIGN 4096u
#define XFER_BUF_NEEDED 512u
static uint8_t xfer_buf_raw[XFER_BUF_NEEDED + XFER_BUF_ALIGN];
static uint8_t *xfer_buf_ptr;

static void msc_init_dtds(void) {
    if (msc_dtds_initialized) return;
    msc_dtd_rx = (DeviceTransferDescriptor *)align_up((uint32_t)msc_dtd_rx_raw, DTD_ALIGN);
    xfer_buf_ptr = (uint8_t *)align_up((uint32_t)xfer_buf_raw, XFER_BUF_ALIGN);
    msc_dtds_initialized = 1;
}

/* ---- Backing storage: real /documents folder synthesized as FAT16 ----
 * Stage 1: read-only, flat listing, 8.3 names. See usb_fatfs.h. */
#include "usb_fatfs.h"
#define MSC_SECTOR_SIZE FATFS_SECTOR_SIZE

/* ---- Bulk-Only Transport wire structures ---- */
#define CBW_SIGNATURE 0x43425355u // "USBC"
#define CSW_SIGNATURE 0x53425355u // "USBS"

typedef struct {
    uint32_t dCBWSignature;
    uint32_t dCBWTag;
    uint32_t dCBWDataTransferLength;
    uint8_t  bmCBWFlags;   // bit7: 1 = IN (device->host), 0 = OUT
    uint8_t  bCBWLUN;      // low 4 bits
    uint8_t  bCBWCBLength; // low 5 bits, 1-16
    uint8_t  CBWCB[16];
} __attribute__((packed)) CommandBlockWrapper;

typedef struct {
    uint32_t dCSWSignature;
    uint32_t dCSWTag;
    uint32_t dCSWDataResidue;
    uint8_t  bCSWStatus; // 0 = pass, 1 = fail, 2 = phase error
} __attribute__((packed)) CommandStatusWrapper;

#define CSW_STATUS_PASS  0
#define CSW_STATUS_FAIL  1
#define CSW_STATUS_PHASE 2

/* Minimal SCSI sense state -- just enough to answer REQUEST SENSE
   after we fail a command, so hosts that check sense data after a
   failure don't get confused. */
static uint8_t  msc_sense_key = 0;
static uint8_t  msc_sense_asc = 0;
static uint8_t  msc_sense_ascq = 0;

static void msc_set_sense(uint8_t key, uint8_t asc, uint8_t ascq) {
    msc_sense_key = key;
    msc_sense_asc = asc;
    msc_sense_ascq = ascq;
}

// NOTE: a msc_set_medium_present()/msc_medium_present mechanism used
// to live here, reporting "medium not present" via SCSI sense data
// (the same signal a card reader uses when its card is removed)
// before disconnecting. Removed: confirmed to make no observable
// difference to the host's "improperly ejected" warning, even given
// as long as the user was willing to wait for the host to notice.

/* ---- Low-level chunked bulk transfer helpers ---- */
#define MSC_CHUNK_MAX 4096u

// "Add dTD Tripwire" -- the real ChipIdea Linux driver's synchronization
// primitive for safely priming an endpoint that might still be active
// from a recent transfer. See msc_bulk_send_chunk() below.
#define USBCMD_ATDTW (1u << 14)

static int msc_bulk_send_chunk(const uint8_t *data, uint32_t len) {
    msc_init_dtds();

    // FIX: this step was missing entirely. The working reference
    // implementation (send_usb_message() in usb_dma.h) does this
    // unconditionally, as the very first thing, on every single send --
    // an explicit flush of EP1 IN with its own wait-for-clear, before
    // touching the descriptor at all. We tested a flush once, several
    // rounds ago, but only combined with a toggle reset and only for
    // transfers over 64 bytes; it was dropped while isolating other
    // variables and never reinstated on its own. Matching the working
    // version exactly this time.
    {
        int flush_ok;
        ENDPTFLUSH = (1u << 17);
        int _fi = 2000000;
        while ((ENDPTFLUSH & (1u << 17)) && _fi > 0) { _fi--; }
        flush_ok = !(ENDPTFLUSH & (1u << 17));
        msc_total_wait_iters += (uint64_t)(2000000 - _fi);
        if (!flush_ok) dbg_print("  TX pre-flush TIMEOUT");
    }

    dtd_bulk_tx->next_dtd = 0x00000001;
    dtd_bulk_tx->token = (len << 16) | (1 << 15) | 0x80;
    dtd_bulk_tx->buffer[0] = (uint32_t)data;

    // FIX (confirmed in the isolated bulktest project via byte-level
    // analysis of corrupted output): the dTD's buffer[] array has 5
    // entries because a transfer can span up to 5 physical 4KB pages
    // -- buffer[0] is the real starting address, buffer[1..4] must
    // hold the page-aligned base of each subsequent page. We only ever
    // set buffer[0], leaving buffer[1..4] stale. This matters even
    // though xfer_buf (READ(10)'s 512-byte sector buffer) is already
    // page-aligned -- the CSW and SENSE-data buffers were never given
    // that same protection, so if either lands within the last few
    // bytes of a page on some run, hardware reads a stale/garbage
    // continuation address instead of real memory.
    {
        uint32_t page_base = (uint32_t)data & ~0xFFFu;
        for (int p = 1; p < 5; p++) {
            dtd_bulk_tx->buffer[p] = page_base + ((uint32_t)p * 0x1000u);
        }
    }

    clean_dcache_range(dtd_bulk_tx, sizeof(*dtd_bulk_tx));
    clean_dcache_range(data, len);

    USBSTS = USBSTS;

    // FIX: the real ChipIdea Linux driver (drivers/usb/chipidea/udc.c,
    // _hardware_enqueue()) never just writes the queue head and primes
    // when an endpoint might still be active from a recent transfer --
    // it uses the USBCMD_ATDTW ("Add dTD Tripwire") bit specifically to
    // safely check whether hardware still considers the endpoint active
    // before touching the queue head, avoiding a race with the
    // hardware's own internal state transition. We've never used this
    // mechanism anywhere this session, despite hitting exactly this
    // "re-prime an endpoint right after using it" scenario repeatedly
    // (READ(10)'s data phase immediately followed by its CSW, and every
    // other back-to-back EP1 IN send). ENDPTCOMPLETE going high doesn't
    // guarantee hardware has fully, internally released the endpoint --
    // this is the actual documented primitive for checking that safely.
    {
        int tmp_stat = 1;
        int outer_iters = 100000;
        uint64_t atdtw_iters_spent = 0;
        while (outer_iters-- > 0) {
            int atdtw_set;
            int inner_iters = 2000000;
            int inner_start = inner_iters;
            do {
                USBCMD |= USBCMD_ATDTW;
                tmp_stat = ENDPTSTAT & (1 << 17);
                atdtw_set = (USBCMD & USBCMD_ATDTW) != 0;
            } while (!atdtw_set && tmp_stat && --inner_iters > 0);
            atdtw_iters_spent += (uint64_t)(inner_start - inner_iters);
            USBCMD &= ~USBCMD_ATDTW;
            if (!tmp_stat) break;
        }
        msc_total_wait_iters += atdtw_iters_spent;
        if (tmp_stat) {
            dbg_print("  TX ATDTW: endpoint still active after retries");
        }
    }

    ep_list[3].td.next_dtd = (uint32_t)dtd_bulk_tx;
    ep_list[3].td.token = 0;
    ENDPTCTRL1 &= ~ENDPTCTRL_TXS;

    clean_dcache_range(&ep_list[3].td, sizeof(ep_list[3].td));

    ENDPTCOMPLETE = (1 << 17);
    ENDPTPRIME |= (1 << 17);

    int ok;
    {
        int _pi = 5000000;
        while ((ENDPTPRIME & (1 << 17)) && _pi > 0) { _pi--; }
        ok = !(ENDPTPRIME & (1 << 17));
        msc_total_wait_iters += (uint64_t)(5000000 - _pi);
    }
    if (!ok) {
        dbg_print("  TX PRIME TIMEOUT (ENDPTPRIME never cleared)");
        return 0;
    }

    int ok_complete;
    {
        int _i = 5000000;
        while (!(ENDPTCOMPLETE & (1 << 17)) && _i > 0) { _i--; }
        ok_complete = (ENDPTCOMPLETE & (1 << 17)) != 0;
        msc_last_complete_iters = 5000000 - _i;
        msc_total_wait_iters += (uint64_t)msc_last_complete_iters;
    }
    if (!ok_complete) {
        char dtmo[110];
        sprintf(dtmo, "  TX COMPLETE TIMEOUT: nxt=0x%x tok=0x%x cur=0x%x PRM=0x%x STAT=0x%x",
                (unsigned int)ep_list[3].td.next_dtd, (unsigned int)ep_list[3].td.token,
                (unsigned int)ep_list[3].current_dtd, (unsigned int)ENDPTPRIME, (unsigned int)ENDPTSTAT);
        dbg_print(dtmo);
        return 0;
    }
    ENDPTCOMPLETE = (1 << 17);

    invalidate_dcache_line(dtd_bulk_tx);
    invalidate_dcache_line(((uint8_t*)dtd_bulk_tx) + 32);
    drain_write_buffer();
    {
        uint32_t remaining = (dtd_bulk_tx->token >> 16) & 0x7FFF;
        uint8_t status = dtd_bulk_tx->token & 0xFF;
        if (remaining != 0 || (status & 0xC0)) {
            sprintf(msc_dbg_buf, "  TX VERIFY FAIL: remaining=%u status=0x%02x (wanted 0/0x00)",
                    (unsigned int)remaining, status);
            dbg_print(msc_dbg_buf);
            {
                char d2[100];
                sprintf(d2, "  TX FAIL regs: PRM=0x%x STAT=0x%x CTRL1=0x%x complete_iters=%u",
                        (unsigned int)ENDPTPRIME, (unsigned int)ENDPTSTAT,
                        (unsigned int)ENDPTCTRL1, (unsigned int)msc_last_complete_iters);
                dbg_print(d2);
            }
            return 0;
        }
    }

    return 1;
}

// NOTE: a software ZLP helper used to live here. It's no longer needed
// -- the real fix turned out to be the ZLT bit in the queue head
// (usb_dma.h, setup_endpoint_list()), which disables the *hardware's*
// automatic ZLP generation at the source for every transfer, rather
// than needing software to add or suppress one per-send.

static int msc_bulk_send(const uint8_t *data, uint32_t len) {
    uint32_t off = 0;
    while (off < len) {
        uint32_t chunk = len - off;
        if (chunk > MSC_CHUNK_MAX) chunk = MSC_CHUNK_MAX;
        if (!msc_bulk_send_chunk(data + off, chunk)) return 0;
        off += chunk;
    }
    // REVERTED: sending a ZLP here caused a real "babble error"
    // (0xe00002e8) from macOS's actual Mass Storage driver -- the
    // device sent more data than the host expected. Unlike our
    // bulktest's own request pattern (which was genuinely ambiguous
    // about expected length), MSC's real host driver always knows the
    // exact expected byte count in advance from the CBW itself, so
    // there's no ambiguity for a ZLP to resolve -- it's simply
    // unwanted, protocol-violating extra data from the host's
    // perspective.
    return 1;
}

// WRITE(10) data reception -- uses its own dedicated descriptor
// (msc_dtd_rx), separate from usb_dma.h's dtd_bulk_rx (which stays
// dedicated to CBW reception via the free-running callback mechanism).
// Called synchronously from inside msc_on_cbw_received(), so there's
// no concurrency concern between the two.
static int msc_bulk_recv_chunk(uint8_t *data, uint32_t len) {
    msc_init_dtds();
    msc_dtd_rx->next_dtd = 0x00000001;
    msc_dtd_rx->token = (len << 16) | (1 << 15) | 0x80;
    msc_dtd_rx->buffer[0] = (uint32_t)data;
    {
        uint32_t page_base = (uint32_t)data & ~0xFFFu;
        for (int p = 1; p < 5; p++) {
            msc_dtd_rx->buffer[p] = page_base + ((uint32_t)p * 0x1000u);
        }
    }

    ep_list[2].td.next_dtd = (uint32_t)msc_dtd_rx;
    ep_list[2].td.token &= ~0xC0;

    clean_dcache_range(msc_dtd_rx, sizeof(*msc_dtd_rx));
    clean_dcache_range(&ep_list[2].td, sizeof(ep_list[2].td));
    drain_write_buffer();

    ENDPTPRIME |= (1 << 1);

    const int iters_budget = 100000000;
    int _i1 = iters_budget;
    while ((ENDPTPRIME & (1 << 1)) && _i1 > 0) { _i1--; }
    int ok = !(ENDPTPRIME & (1 << 1));
    msc_total_wait_iters += (uint64_t)(iters_budget - _i1);
    if (!ok) return 0;

    int _i2 = iters_budget;
    while (!(ENDPTCOMPLETE & (1 << 1)) && _i2 > 0) { _i2--; }
    int ok_complete = (ENDPTCOMPLETE & (1 << 1)) != 0;
    msc_total_wait_iters += (uint64_t)(iters_budget - _i2);
    if (!ok_complete) return 0;
    ENDPTCOMPLETE = (1 << 1);

    invalidate_dcache_line(msc_dtd_rx);
    invalidate_dcache_line(((uint8_t *)msc_dtd_rx) + 32);
    drain_write_buffer();

    // FIX: never trusted the ACTUAL transferred length before -- only
    // that ENDPTCOMPLETE fired. If the host ever completes a transfer
    // with fewer bytes than requested (a stray short/zero-length
    // packet, for instance), the destination buffer is left holding
    // whatever was already there from a PREVIOUS operation, and we'd
    // still report success -- silently misinterpreting stale data as
    // this sector's real content.
    uint32_t remaining = (msc_dtd_rx->token >> 16) & 0x7FFF;
    uint32_t received = len - remaining;
    if (received != len) {
        char d[70];
        sprintf(d, "  RX short/partial: expected %u, got %u", (unsigned int)len, (unsigned int)received);
        dbg_print(d);
        return 0;
    }

    invalidate_dcache_range(data, len);
    drain_write_buffer();
    return 1;
}

static int msc_bulk_recv(uint8_t *data, uint32_t len) {
    uint32_t off = 0;
    while (off < len) {
        uint32_t chunk = len - off;
        if (chunk > MSC_CHUNK_MAX) chunk = MSC_CHUNK_MAX;
        if (!msc_bulk_recv_chunk(data + off, chunk)) return 0;
        off += chunk;
    }
    return 1;
}

/* ---- CSW send ---- */
static void msc_send_csw(uint32_t tag, uint32_t residue, uint8_t status) {
    static CommandStatusWrapper csw __attribute__((aligned(32)));
    csw.dCSWSignature = CSW_SIGNATURE;
    csw.dCSWTag = tag;
    csw.dCSWDataResidue = residue;
    csw.bCSWStatus = status;
    msc_bulk_send((const uint8_t *)&csw, sizeof(csw));
}

/* ---- SCSI command handling ----
 * Returns 1 on success (CSW_STATUS_PASS), 0 on failure (caller sends
 * CSW_STATUS_FAIL and the sense data set via msc_set_sense). */
static int msc_scsi_execute(const uint8_t *cdb, uint8_t lun,
                             uint32_t data_len, uint8_t data_in,
                             uint32_t *out_residue, uint32_t tag) {
    (void)lun; // single LUN (0) supported
    (void)tag; // no longer used now that CSW pipelining is gone
    *out_residue = 0;
    uint8_t opcode = cdb[0];

    msc_init_dtds();
    uint8_t *xfer_buf = xfer_buf_ptr;

    // A background move is still copying file data -- report "not
    // ready" specifically for WRITE(10), so a well-behaved host
    // retries automatically instead of a new write racing the
    // ongoing copy. Deliberately NOT applied to TEST_UNIT_READY or
    // READ(10): rejecting reads too (e.g. Finder browsing into a
    // folder) has a much shorter host-side patience budget than a
    // copy does, and was the likely cause of a hang-then-forced-eject
    // rather than the graceful retry this was meant to produce. A
    // specific file that hasn't finished copying to its new location
    // yet may simply fail to open -- a much milder failure than
    // rejecting the read outright.
    if (fatfs_bg_copy_active && opcode == 0x2A) {
        msc_set_sense(0x02, 0x04, 0x01); // NOT READY / LOGICAL UNIT NOT READY, IN PROGRESS OF BECOMING READY
        *out_residue = data_len;
        return 0;
    }

    switch (opcode) {
    case 0x00: { // TEST UNIT READY
        if (!msc_quiet_tx) dbg_print("  TUR (no data)");
        return 1;
    }
    case 0x03: { // REQUEST SENSE
        static uint8_t sense[18] __attribute__((aligned(32)));
        memset(sense, 0, sizeof(sense));
        sense[0] = 0x70;
        sense[2] = msc_sense_key;
        sense[7] = 10;
        sense[12] = msc_sense_asc;
        sense[13] = msc_sense_ascq;
        uint32_t n = data_len < sizeof(sense) ? data_len : sizeof(sense);
        if (!msc_quiet_tx) { char d[50]; sprintf(d, "  SENSE sending n=%u (buf=%u)", (unsigned int)n, (unsigned int)sizeof(sense)); dbg_print(d); }
        if (data_in && n > 0) {
            if (!msc_bulk_send(sense, n)) return 0;
        }
        *out_residue = data_len - n;
        return 1;
    }
    case 0x12: { // INQUIRY
        static uint8_t inq[36] __attribute__((aligned(32)));
        memset(inq, 0, sizeof(inq));
        inq[0] = 0x00;
        inq[1] = 0x80;
        inq[2] = 0x04;
        inq[3] = 0x02;
        inq[4] = 31;
        memcpy(&inq[8],  "NSPIRE  ", 8);
        memcpy(&inq[16], "BareMetal Disk  ", 16);
        memcpy(&inq[32], "1.00", 4);
        uint32_t n = data_len < sizeof(inq) ? data_len : sizeof(inq);
        {
            char d[60];
            sprintf(d, "  INQ n=%u dlen=%u buf=%u", (unsigned int)n, (unsigned int)data_len, (unsigned int)sizeof(inq));
            dbg_print(d);
        }
        if (data_in && n > 0) { if (!msc_bulk_send(inq, n)) return 0; }
        *out_residue = data_len - n;
        return 1;
    }
    case 0x1A: { // MODE SENSE (6) -- WP bit cleared, volume is writable.
        static uint8_t ms[12] __attribute__((aligned(32)));
        memset(ms, 0, sizeof(ms));
        uint32_t total = fatfs_total_sectors_count();
        ms[0] = 11; ms[1] = 0; ms[2] = 0x00; ms[3] = 8; ms[4] = 0;
        ms[5] = (uint8_t)(total >> 16);
        ms[6] = (uint8_t)(total >> 8);
        ms[7] = (uint8_t)(total);
        ms[8] = 0;
        ms[9] = (uint8_t)(MSC_SECTOR_SIZE >> 16);
        ms[10] = (uint8_t)(MSC_SECTOR_SIZE >> 8);
        ms[11] = (uint8_t)(MSC_SECTOR_SIZE);
        uint32_t n = data_len < sizeof(ms) ? data_len : sizeof(ms);
        if (!msc_quiet_tx) { char d[50]; sprintf(d, "  MODESENSE sending n=%u", (unsigned int)n); dbg_print(d); }
        if (data_in && n > 0) {
            if (!msc_bulk_send(ms, n)) return 0;
        }
        *out_residue = data_len - n;
        return 1;
    }
    case 0x25: { // READ CAPACITY (10)
        static uint8_t cap[8] __attribute__((aligned(32)));
        uint32_t total = fatfs_total_sectors_count();
        uint32_t last_lba = (total > 0) ? (total - 1) : 0;
        cap[0] = (uint8_t)(last_lba >> 24);
        cap[1] = (uint8_t)(last_lba >> 16);
        cap[2] = (uint8_t)(last_lba >> 8);
        cap[3] = (uint8_t)(last_lba);
        cap[4] = (uint8_t)(MSC_SECTOR_SIZE >> 24);
        cap[5] = (uint8_t)(MSC_SECTOR_SIZE >> 16);
        cap[6] = (uint8_t)(MSC_SECTOR_SIZE >> 8);
        cap[7] = (uint8_t)(MSC_SECTOR_SIZE);
        uint32_t n = data_len < sizeof(cap) ? data_len : sizeof(cap);
        { char d[60]; sprintf(d, "  CAPACITY total_sectors=%u n=%u", (unsigned int)total, (unsigned int)n); dbg_print(d); }
        if (data_in && n > 0) { if (!msc_bulk_send(cap, n)) return 0; }
        *out_residue = data_len - n;
        return 1;
    }
    case 0x28: { // READ (10)
        uint32_t lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) |
                       ((uint32_t)cdb[4] << 8)  |  (uint32_t)cdb[5];
        uint16_t blocks = ((uint16_t)cdb[7] << 8) | cdb[8];
        { char d[60]; sprintf(d, "  READ10 lba=%u blocks=%u", (unsigned int)lba, blocks); dbg_print(d); }

        if ((uint64_t)lba + blocks > fatfs_total_sectors_count()) {
            msc_set_sense(0x05, 0x21, 0x00);
            return 0;
        }
        for (uint16_t i = 0; i < blocks; i++) {
            if (!fatfs_read_sector(lba + i, xfer_buf)) return 0;
            if (!msc_bulk_send(xfer_buf, MSC_SECTOR_SIZE)) return 0;
        }
        return 1;
    }
    case 0x2A: { // WRITE (10)
        uint32_t lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) |
                       ((uint32_t)cdb[4] << 8)  |  (uint32_t)cdb[5];
        uint16_t blocks = ((uint16_t)cdb[7] << 8) | cdb[8];
        fatfs_log("write10-cmd: lba=%u blocks=%u wait_iters_before=%llu\n",
                  (unsigned int)lba, (unsigned int)blocks, (unsigned long long)msc_total_wait_iters);

        for (uint16_t i = 0; i < blocks; i++) {
            if (!msc_bulk_recv(xfer_buf, MSC_SECTOR_SIZE)) {
                fatfs_log("write10-fail: lba=%u blocks=%u failed at sector %u of %u\n",
                          (unsigned int)lba, (unsigned int)blocks, (unsigned int)i, (unsigned int)blocks);
                return 0;
            }
            fatfs_write_sector(lba + i, xfer_buf);
        }
        fatfs_log("write10-done: lba=%u blocks=%u wait_iters_after=%llu\n",
                  (unsigned int)lba, (unsigned int)blocks, (unsigned long long)msc_total_wait_iters);
        return 1;
    }
    case 0x5A: { // MODE SENSE (10)
        uint8_t dbd = (cdb[1] & 0x08) ? 1 : 0;
        static uint8_t ms[16] __attribute__((aligned(32)));
        memset(ms, 0, sizeof(ms));
        uint32_t total = fatfs_total_sectors_count();
        uint32_t resp_size = dbd ? 8 : 16;
        uint16_t mode_data_len = (uint16_t)(resp_size - 2);
        ms[0] = (uint8_t)(mode_data_len >> 8);
        ms[1] = (uint8_t)(mode_data_len & 0xFF);
        ms[2] = 0;
        ms[3] = 0x00;
        if (dbd) {
            ms[6] = 0; ms[7] = 0;
        } else {
            ms[6] = 0;
            ms[7] = 8;
            ms[8] = 0;
            ms[9] = (uint8_t)(total >> 16);
            ms[10] = (uint8_t)(total >> 8);
            ms[11] = (uint8_t)(total);
            ms[12] = 0;
            ms[13] = (uint8_t)(MSC_SECTOR_SIZE >> 16);
            ms[14] = (uint8_t)(MSC_SECTOR_SIZE >> 8);
            ms[15] = (uint8_t)(MSC_SECTOR_SIZE);
        }
        uint32_t n = data_len < resp_size ? data_len : resp_size;
        if (data_in && n > 0) { if (!msc_bulk_send(ms, n)) return 0; }
        *out_residue = data_len - n;
        return 1;
    }
    case 0x1E: { // PREVENT ALLOW MEDIUM REMOVAL -- no data phase.
        return 1;
    }
    case 0x1B: { // START STOP UNIT -- no data phase.
        return 1;
    }
    case 0x35: { // SYNCHRONIZE CACHE (10) -- no data phase.
        return 1;
    }
    case 0x9E: { // SERVICE ACTION IN (16) -- READ CAPACITY (16)
        uint8_t service_action = cdb[1] & 0x1F;
        if (service_action == 0x10) {
            static uint8_t cap16[32] __attribute__((aligned(32)));
            memset(cap16, 0, sizeof(cap16));
            uint32_t total = fatfs_total_sectors_count();
            uint64_t last_lba = (total > 0) ? (total - 1) : 0;
            for (int i = 0; i < 8; i++) {
                cap16[i] = (uint8_t)(last_lba >> (8 * (7 - i)));
            }
            cap16[8]  = (uint8_t)(MSC_SECTOR_SIZE >> 24);
            cap16[9]  = (uint8_t)(MSC_SECTOR_SIZE >> 16);
            cap16[10] = (uint8_t)(MSC_SECTOR_SIZE >> 8);
            cap16[11] = (uint8_t)(MSC_SECTOR_SIZE);
            uint32_t n = data_len < sizeof(cap16) ? data_len : sizeof(cap16);
            { char d[60]; sprintf(d, "  CAPACITY16 total_sectors=%u n=%u", (unsigned int)total, (unsigned int)n); dbg_print(d); }
            if (data_in && n > 0) { if (!msc_bulk_send(cap16, n)) return 0; }
            *out_residue = data_len - n;
            return 1;
        }
        msc_set_sense(0x05, 0x20, 0x00);
        if (data_len > 0 && data_in) {
            ENDPTCTRL1 |= ENDPTCTRL_TXS;
            ENDPTCTRL1 &= ~ENDPTCTRL_TXS;
        }
        *out_residue = data_len;
        return 0;
    }
    default: {
        msc_set_sense(0x05, 0x20, 0x00);
        if (data_len > 0) {
            if (!data_in) {
                uint32_t remaining = data_len;
                while (remaining > 0) {
                    uint32_t chunk = remaining < XFER_BUF_NEEDED ? remaining : XFER_BUF_NEEDED;
                    msc_bulk_recv(xfer_buf, chunk);
                    remaining -= chunk;
                }
            } else {
                ENDPTCTRL1 |= ENDPTCTRL_TXS;
                ENDPTCTRL1 &= ~ENDPTCTRL_TXS;
            }
            *out_residue = data_len;
        }
        return 0;
    }
    }
}

/* ---- Bulk RX callback: registered via set_usb_rx_callback() ----
 * Called automatically by usb_dma.h's service_bulk_endpoints()
 * whenever EP1 OUT data arrives -- replaces the old manual CBW-arming
 * state machine entirely. usb_dma.h re-arms EP1 OUT for the next CBW
 * on its own right after this returns. */
void msc_on_cbw_received(const uint8_t *data, uint32_t length) {
    if (length < sizeof(CommandBlockWrapper)) return;

    static CommandBlockWrapper cbw __attribute__((aligned(32)));
    memcpy(&cbw, data, sizeof(cbw));

    if (cbw.dCBWSignature != CBW_SIGNATURE || cbw.bCBWCBLength == 0 || cbw.bCBWCBLength > 16) {
        char bad[80];
        sprintf(bad, "CBW bad sig=0x%x cblen=%u", (unsigned int)cbw.dCBWSignature, cbw.bCBWCBLength);
        dbg_print(bad);
        return;
    }

    if (cbw.CBWCB[0] == 0x2A) {
        static uint64_t last_cbw_loop_count = 0;
        fatfs_log("cbw-gap: loop_iters_since_last_write10_cbw=%llu\n",
                  (unsigned long long)(main_loop_iters - last_cbw_loop_count));
        last_cbw_loop_count = main_loop_iters;
    }

    uint8_t op = cbw.CBWCB[0];
    int quiet = 0;
    msc_quiet_tx = quiet;

    if (!quiet) {
        char dbg[100];
        sprintf(dbg, "CBW op=0x%02x dlen=%u dir=%s tag=0x%x",
                cbw.CBWCB[0], (unsigned int)cbw.dCBWDataTransferLength,
                (cbw.bmCBWFlags & 0x80) ? "IN" : "OUT", (unsigned int)cbw.dCBWTag);
        dbg_print(dbg);
    }

    uint8_t data_in = (cbw.bmCBWFlags & 0x80) ? 1 : 0;
    uint32_t residue = 0;

    int ok = msc_scsi_execute(cbw.CBWCB, cbw.bCBWLUN,
                               cbw.dCBWDataTransferLength, data_in, &residue, cbw.dCBWTag);

    // REVERTED: the deferred-CSW mechanism (queue it and send on a
    // later loop iteration) didn't fix anything on its own and just
    // added noise while testing the ATDTW fix in isolation. Back to
    // sending immediately -- ATDTW is the real synchronization
    // primitive for this scenario; if it works, no artificial delay
    // should be needed at all.
    msc_send_csw(cbw.dCBWTag, residue, ok ? CSW_STATUS_PASS : CSW_STATUS_FAIL);
    if (op == 0x2A) {
        fatfs_log("write10-csw-sent: wait_iters_after=%llu\n", (unsigned long long)msc_total_wait_iters);
    }

    if (!quiet || !ok) {
        char dbg[70];
        sprintf(dbg, "  op=0x%02x -> ok=%d residue=%u", op, ok, (unsigned int)residue);
        dbg_print(dbg);
    }
}

/* ---- Unhandled-control callback: registered via
 * set_usb_unhandled_control_callback() ----
 * Called by usb_dma.h's service_control_endpoint() for any control
 * request it doesn't natively recognize: string descriptors (device/
 * config descriptors are already handled natively, since usb_dma.h's
 * own descriptors were changed to MSC-correct values), and MSC's two
 * class-specific requests (GET_MAX_LUN, Bulk-Only Mass Storage
 * Reset). */
void msc_on_unhandled_control(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
    (void)wIndex;
    (void)wValue;

    if (bRequest == 0xFE) { // GET_MAX_LUN (device-to-host, EP0 IN)
        static const uint8_t max_lun = 0; // single LUN
        transmit_ep0_data(&max_lun, 1, wLength);
        return;
    }

    if (bRequest == 0xFF) { // Bulk-Only Mass Storage Reset (host-to-device, no data)
        ack_zero_length_status_phase();
        int ok;
        WAIT_CLEAR_TIMEOUT(ENDPTPRIME, 0x00010000, 5000000, ok);
        (void)ok;
        int ok_complete;
        WAIT_SET_TIMEOUT(ENDPTCOMPLETE, 0x00010000, 5000000, ok_complete);
        if (ok_complete) ENDPTCOMPLETE = 0x00010000;
        return;
    }
}
