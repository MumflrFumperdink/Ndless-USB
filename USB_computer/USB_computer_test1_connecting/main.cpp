#include <os.h>
#include <keys.h>
#include <stdint.h>
#include <string.h>

#include <nspireio/console.hpp>
inline nio::console screen_console;

// Aligned structure required by the DMA engine
typedef struct {
    volatile uint32_t next_dtd;
    volatile uint32_t token;
    volatile uint32_t buffer[5];
} __attribute__((packed)) DeviceTransferDescriptor;

typedef struct {
    volatile uint32_t config;
    volatile uint32_t current_dtd;
    DeviceTransferDescriptor td;
    volatile int32_t reserved;
    volatile uint8_t setup_buffer[8]; // Where SET_ADDRESS / GET_DESCRIPTOR tokens land
    volatile uint8_t _pad[16]; // REQUIRED: queue heads occupy 64-byte slots in
                                // the array, not 48-byte packed structs. Confirmed
                                // directly from firebird/nspire_emu's usb.c (the
                                // actual emulator source that generated the
                                // original trace): queue head addresses are
                                // computed as eplistaddr + ep*0x80 (OUT) and
                                // eplistaddr + ep*0x80 + 0x40 (IN) -- a 64-byte
                                // stride per queue head. Without this padding,
                                // every queue head past index 0 (EP0 OUT, which
                                // happens to sit at offset 0 in both layouts) was
                                // at the WRONG address -- explaining why setup
                                // packet capture always worked but every single
                                // dTD/prime attempt for every endpoint failed
                                // identically no matter what pattern we tried.
} __attribute__((packed)) DeviceQueueHead;

_Static_assert(__builtin_offsetof(DeviceQueueHead, setup_buffer) == 40,
               "DeviceQueueHead.setup must be at hardware offset 40");
_Static_assert(sizeof(DeviceQueueHead) == 64, "DeviceQueueHead must be exactly 64 bytes (64-byte hardware slot, not 48-byte packed struct)");

/* ------------------------------------------------------------------ *
 * MANUAL ALIGNMENT
 *
 * The runtime printout confirmed __attribute__((aligned(2048))) does
 * NOT take effect for these globals on this toolchain/loader -- ep_list
 * still landed at 0x124b8308 (remainder 0x308, not 0). Rather than trust
 * the attribute, we over-allocate raw storage and compute an aligned
 * pointer into it ourselves at runtime. This works regardless of what
 * the linker does with alignment directives.
 * ------------------------------------------------------------------ */
#define EP_LIST_ALIGN   2048u
#define DTD_ALIGN       32u

static uint8_t ep_list_raw[sizeof(DeviceQueueHead) * 4 + EP_LIST_ALIGN];
static uint8_t dtd_tx_raw[sizeof(DeviceTransferDescriptor) + DTD_ALIGN];
static uint8_t dtd_rx_raw[sizeof(DeviceTransferDescriptor) + DTD_ALIGN];
static uint8_t rx_buffer_raw[64 + DTD_ALIGN];

static DeviceQueueHead *ep_list;
static DeviceTransferDescriptor *dtd_tx;
static DeviceTransferDescriptor *dtd_rx;
static volatile uint8_t *rx_buffer;

static inline uint32_t align_up(uint32_t addr, uint32_t align) {
    return (addr + (align - 1)) & ~(align - 1);
}

/* Snapshot of whatever the native OS had configured in the USB
   controller before we took it over. Saved once at the very start of
   main(), before reset_usb_subsystem() ever touches anything, and
   written back on every exit path. This doesn't guarantee the native
   USB stack resumes perfectly (its in-RAM driver state, interrupt
   handlers, and queue-head memory contents are outside what we can
   see or restore from userspace), but putting the actual registers
   back to what they were is the best userspace-level fix available,
   and is much closer to native's own expected state than a generic
   reset to power-on defaults. */
typedef struct {
    uint32_t usbcmd;
    uint32_t usbintr;
    uint32_t deviceaddr;
    uint32_t eplistaddr;
    uint32_t portsc1;
    uint32_t otgsc;
    uint32_t usbmode;
    uint32_t endptctrl0;
    uint32_t endptctrl1;
} UsbRegisterSnapshot;

static UsbRegisterSnapshot native_usb_state;
static int native_usb_state_saved = 0;

/* ---------------- MMIO ---------------- */
#define USBCTRL_BASE 0xB0000000
#define REG32(off) (*(volatile uint32_t*)(USBCTRL_BASE + (off)))

#define HWDEVICE         REG32(0x00C)
#define CAPLENGTH        REG32(0x100)
#define HCSPARAMS        REG32(0x104)

#define USBCMD           REG32(0x140)
#define USBSTS           REG32(0x144)
#define USBINTR          REG32(0x148)
#define DEVICEADDR       REG32(0x154)
#define ENDPOINTLISTADDR REG32(0x158)
#define PORTSC1          REG32(0x184)
#define OTGSC            REG32(0x1A4)
#define USBMODE          REG32(0x1A8)
#define ENDPTSETUPSTAT   REG32(0x1AC)
#define ENDPTPRIME       REG32(0x1B0)
#define ENDPTFLUSH       REG32(0x1B4)
#define ENDPTSTAT        REG32(0x1B8)
#define ENDPTCOMPLETE    REG32(0x1BC)
#define ENDPTCTRL0       REG32(0x1C0)
#define ENDPTCTRL1       REG32(0x1C4)

#define USBCMD_RS        (1u << 0)
#define USBCMD_RST       (1u << 1)
#define USBCMD_SUTW      (1u << 13)

#define ENDPTCTRL_TXE      (1u << 23)
#define ENDPTCTRL_TXR      (1u << 22)
#define ENDPTCTRL_TXT_BULK (2u << 18)
#define ENDPTCTRL_TXS      (1u << 16)

#define ENDPTCTRL_RXE      (1u << 7)
#define ENDPTCTRL_RXR      (1u << 6)
#define ENDPTCTRL_RXT_BULK (2u << 2)
#define ENDPTCTRL_RXS      (1u << 0)

#define OTGSC_BSV         (1u << 11)

static const uint8_t device_descriptor[18] = {
    0x12, 0x01, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x40,
    0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x01
};

// Configuration descriptor: config header (9) + interface (9) +
// EP1 IN endpoint (7) + EP1 OUT endpoint (7) = 32 bytes total.
static const uint8_t config_descriptor[32] = {
    // Configuration Descriptor
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
    // Interface Descriptor (1 interface, 2 endpoints, vendor-specific class)
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x00,
    // Endpoint Descriptor: EP1 IN, bulk, 64 bytes
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
    // Endpoint Descriptor: EP1 OUT, bulk, 64 bytes
    0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00
};

volatile int usb_device_configured = 0;
volatile int usb_should_exit = 0;

static inline void drain_write_buffer(void) {
    unsigned int zero = 0;
    asm volatile("mcr p15, 0, %0, c7, c10, 4" : : "r"(zero) : "memory");
}

static inline void invalidate_dcache_line(const volatile void *addr) {
    unsigned int a = (unsigned int)addr;
    asm volatile("mcr p15, 0, %0, c7, c6, 1" : : "r"(a) : "memory");
}

/* Clean (write-back) a range of the data cache to RAM. This is NOT the
   same thing as drain_write_buffer(): drain only flushes the CPU's
   write buffer, it does not push dirty cache lines out to physical
   memory. Since the DMA engine reads RAM directly and has no visibility
   into the CPU's cache, any queue head overlay field we modify MUST be
   cleaned to RAM before we tell hardware to prime/fetch it -- otherwise the DMA engine can read a
   stale (often all-zero) copy, see an inactive token, and never
   actually perform the transfer. This is almost certainly why
   ENDPTPRIME kept failing to clear: we were priming hardware to read
   descriptors that hadn't actually left the CPU cache yet. */
static inline void clean_dcache_range(const volatile void *addr, uint32_t size) {
    uint32_t start = (uint32_t)addr & ~31u;
    uint32_t end   = ((uint32_t)addr + size + 31u) & ~31u;
    for (uint32_t a = start; a < end; a += 32) {
        asm volatile("mcr p15, 0, %0, c7, c10, 1" : : "r"(a) : "memory");
    }
    drain_write_buffer();
}

static inline void busy_wait(volatile int cycles) {
    while (cycles-- > 0) { }
}

/* REVERTED: the WFI-based idle here almost certainly caused a hang
   somewhere before SET_CONFIGURATION was ever reached -- EVERY wait
   call in the whole program (SET_ADDRESS, GET_DESCRIPTOR, flush,
   everything) went through cpu_wait_for_interrupt() on every
   iteration, and if interrupts are masked in this execution context
   even once, that call never returns at all. That's consistent with
   "nothing prints anymore", including messages that used to print
   reliably earlier in the sequence. Back to plain bounded busy-
   waiting, which is proven to work correctly through this entire
   debugging session. A real "block without spinning" would need an
   actual USB interrupt handler hooked into the native OS's IRQ vector
   table, which needs more certainty about what's safely available
   there than we currently have. */
#define WAIT_CLEAR_TIMEOUT(reg, mask, iters, ok_flag) \
    do { \
        int _i = (iters); \
        while (((reg) & (mask)) && _i > 0) { _i--; } \
        (ok_flag) = !((reg) & (mask)); \
    } while (0)

/* Companion macro: waits for a bit to become SET (the opposite of
   WAIT_CLEAR_TIMEOUT above), used to confirm ENDPTCOMPLETE actually
   fires -- the real "the host received and acknowledged this transfer"
   signal, distinct from ENDPTPRIME merely clearing (which only means
   hardware started processing the transfer, not that it finished). */
#define WAIT_SET_TIMEOUT(reg, mask, iters, ok_flag) \
    do { \
        int _i = (iters); \
        while (!((reg) & (mask)) && _i > 0) { _i--; } \
        (ok_flag) = ((reg) & (mask)) != 0; \
    } while (0)

/* Capture whatever the native OS had configured before we touch
   anything. Must be called as the very first thing in main(), before
   reset_usb_subsystem(). */
void save_native_usb_state(void) {
    native_usb_state.usbcmd      = USBCMD;
    native_usb_state.usbintr     = USBINTR;
    native_usb_state.deviceaddr  = DEVICEADDR;
    native_usb_state.eplistaddr  = ENDPOINTLISTADDR;
    native_usb_state.portsc1     = PORTSC1;
    native_usb_state.otgsc       = OTGSC;
    native_usb_state.usbmode     = USBMODE;
    native_usb_state.endptctrl0  = ENDPTCTRL0;
    native_usb_state.endptctrl1  = ENDPTCTRL1;
    native_usb_state_saved = 1;
}

/* Put the controller back into the state we found it in. Does a full
   reset first to clear out whatever OUR program left behind, then
   writes back native's actual values -- USBCMD (which includes
   Run/Stop) written LAST, since everything else should be in place
   before the controller starts running again. This can't guarantee
   the native driver's in-RAM software state (interrupt handlers,
   queue-head memory contents) is still consistent, but restoring the
   real hardware registers to what native had is the closest a
   userspace program can get. */
void restore_native_usb_state(void) {
    if (!native_usb_state_saved) return;

    int ok;
    USBCMD &= ~USBCMD_RS;
    USBCMD |= USBCMD_RST;
    WAIT_CLEAR_TIMEOUT(USBCMD, USBCMD_RST, 2000000, ok);

    USBMODE          = native_usb_state.usbmode;
    PORTSC1           = native_usb_state.portsc1;
    OTGSC             = native_usb_state.otgsc;
    DEVICEADDR        = native_usb_state.deviceaddr;
    ENDPOINTLISTADDR  = native_usb_state.eplistaddr;
    ENDPTCTRL0        = native_usb_state.endptctrl0;
    ENDPTCTRL1        = native_usb_state.endptctrl1;
    USBINTR           = native_usb_state.usbintr;
    USBCMD            = native_usb_state.usbcmd; // Run/Stop last
}

void print_alignment_check(const char *label, void *addr, uint32_t required) {
    char strbuf[100];
    uint32_t a = (uint32_t)addr;
    uint32_t rem = a & (required - 1);
    sprintf(strbuf, "%s @ 0x%x (align%%%d = 0x%x)%s",
            label, a, required, rem, rem == 0 ? " OK" : " *** MISALIGNED ***");
    screen_console << strbuf << nio::endl;
}

void print_setup_hex(const uint8_t *buf) {
    char strbuf[100];
    sprintf(strbuf, "     setup raw: %02x %02x %02x %02x %02x %02x %02x %02x",
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    screen_console << strbuf << nio::endl;
}

/* Dump the actual hardware/queue-head state for EP0 IN. Invalidates
   the queue head's cache line first so we see what HARDWARE last wrote
   there (it DMAs status/progress back into the same overlay fields),
   not a stale CPU-side copy. Two structurally-justified fixes (cache
   clean, flush bit-mapping) produced byte-identical output, which
   means we need to actually see register values now instead of
   guessing a third time. */
void print_ep0_in_state(const char *label) {
    invalidate_dcache_line(&ep_list[1].td);
    invalidate_dcache_line(((uint8_t*)&ep_list[1].td) + 32);
    invalidate_dcache_line(&ep_list[1].current_dtd);
    drain_write_buffer();

    char buf[80];
    screen_console << buf << nio::endl;
    sprintf(buf, "tok=0x%x", (unsigned int)ep_list[1].td.token);
    screen_console << buf << nio::endl;
    sprintf(buf, "nxt=0x%x cur=0x%x", (unsigned int)ep_list[1].td.next_dtd,
            (unsigned int)ep_list[1].current_dtd);
    screen_console << buf << nio::endl;
    sprintf(buf, "PRM=0x%x STAT=0x%x", (unsigned int)ENDPTPRIME, (unsigned int)ENDPTSTAT);
    screen_console << buf << nio::endl;
    // We've only ever checked these AT BOOT. Verify they're still what
    // we think they are at the actual point of failure -- if either
    // has changed, something (a bus reset we're not handling, a stray
    // write, hardware auto-clearing on error) has knocked our setup
    // over without us noticing.
    sprintf(buf, "LISTADDR=0x%x want=0x%x", (unsigned int)ENDPOINTLISTADDR,
            (unsigned int)ep_list);
    screen_console << buf << nio::endl;
    sprintf(buf, "CTRL0=0x%x", (unsigned int)ENDPTCTRL0);
    screen_console << buf << nio::endl;
    sprintf(buf, "USBCMD=0x%x", (unsigned int)USBCMD);
    screen_console << buf << nio::endl;
}

/* Same idea as print_ep0_in_state, but for EP1 OUT (ep_list[2]),
   bit1 in PRIME/STAT/COMPLETE. Used to check whether arm_bulk_out()
   actually put the endpoint into a listening state. */
void print_ep1_out_state(const char *label) {
    invalidate_dcache_line(&ep_list[2].td);
    invalidate_dcache_line(((uint8_t*)&ep_list[2].td) + 32);
    invalidate_dcache_line(&ep_list[2].current_dtd);
    drain_write_buffer();

    char buf[80];
    screen_console << label << nio::endl;
    sprintf(buf, "tok=0x%x", (unsigned int)ep_list[2].td.token);
    screen_console << buf << nio::endl;
    sprintf(buf, "nxt=0x%x cur=0x%x", (unsigned int)ep_list[2].td.next_dtd,
            (unsigned int)ep_list[2].current_dtd);
    screen_console << buf << nio::endl;
    sprintf(buf, "PRM=0x%x STAT=0x%x", (unsigned int)ENDPTPRIME, (unsigned int)ENDPTSTAT);
    screen_console << buf << nio::endl;
    sprintf(buf, "COMPLETE=0x%x CTRL1=0x%x", (unsigned int)ENDPTCOMPLETE, (unsigned int)ENDPTCTRL1);
    screen_console << buf << nio::endl;
}

/* Point the DeviceQueueHead/dTD pointers at aligned locations inside our
   over-allocated raw buffers. Call this once before touching any of
   ep_list/dtd_tx/dtd_rx/rx_buffer. CONFIRMED against the real Linux
   chipidea driver source (_hardware_enqueue in udc.c): the external
   linked-td pattern (qh.td.next -> separate td, with the ACTIVE bit set
   on the EXTERNAL td, not the overlay) is correct. My earlier "write
   directly into the overlay" change was actually a deviation from how
   this hardware really works -- reverted. */
void init_aligned_pointers(void) {
    ep_list  = (DeviceQueueHead *)align_up((uint32_t)ep_list_raw, EP_LIST_ALIGN);
    dtd_tx   = (DeviceTransferDescriptor *)align_up((uint32_t)dtd_tx_raw, DTD_ALIGN);
    dtd_rx   = (DeviceTransferDescriptor *)align_up((uint32_t)dtd_rx_raw, DTD_ALIGN);
    rx_buffer = (volatile uint8_t *)align_up((uint32_t)rx_buffer_raw, DTD_ALIGN);
}

/* ------------------------------------------------------------------ *
 * BOOT SEQUENCE
 *
 * REVERTED back to the simple single-shot reset. The two-phase
 * native-trace-replay version (with the deliberate pull-up
 * disconnect/reconnect toggle) was a regression: it went from "host
 * sends SET_ADDRESS but we misread the data" to "host never sends
 * anything at all." A real handshake with corrupted data is much
 * closer to working than total silence, and now that alignment is
 * confirmed correct and the setup-packet read uses the tripwire, the
 * simple reset that originally got SET_ADDRESS to arrive is the better
 * foundation to build on.
 * ------------------------------------------------------------------ */
void reset_usb_subsystem(void) {
    // 1. Terminate current operations by clearing the Run/Stop bit
    USBCMD &= ~USBCMD_RS;

    // 2. Issue a Controller Reset (RST bit)
    USBCMD |= USBCMD_RST;
    while (USBCMD & USBCMD_RST) {
        // Wait for hardware to clear the reset bit automatically
    }

    // 3. Force Device Mode in the USBMODE register
    USBMODE = 0x00000002 | 0x00000008;

    // 4. PORTSC1: native writes the literal value 0xED000004 here (not
    // just OR-ing in one bit). This includes PFSC (bit24, force full
    // speed -- matches the Nspire "zevio" platform quirk) PLUS several
    // other bits (26,27,29,30,31) in the same write that we'd never
    // touched at all. Replicating the exact value instead of guessing
    // at individual bit meanings, since guessing individual bits from
    // generic ChipIdea docs hasn't moved the needle so far.
    PORTSC1 = 0xED000004;

    // 5. Enable the OTG tracking infrastructure! Set Bit 5 (IDPU) to
    //    turn back on the internal ID pull-up transceiver routing.
    OTGSC = 0x007F0020;

    // 6. Clear any OS-assigned device address
    DEVICEADDR = 0;
}

/* Flush any stale transfer state on an endpoint before using it.
   IMPORTANT: ENDPTFLUSH/ENDPTPRIME/ENDPTCOMPLETE bits are indexed by
   USB ENDPOINT NUMBER (0 or 1), not by queue-head array index (0-3).
   OUT lives in the low half (bit = ep_num), IN lives in the high half
   (bit = ep_num + 16). A single endpoint number covers both directions
   since EP0's OUT and IN queue heads are still "endpoint 0" in the
   hardware's eyes, same for EP1. The previous version took two
   separate arguments and was called with queue-head indices (0,1) for
   EP0 and (2,3) for EP1 -- which flushed bit 17 (EP1 IN) instead of
   bit 16 (EP0 IN) when handling EP0 setup packets, and flushed
   meaningless bits 2/19 instead of EP1's real bits 1/17. That left
   EP0 IN's actual overlay state never cleared, which is a very
   plausible reason ENDPTPRIME kept failing to settle even after the
   cache-clean fix. */
void flush_endpoint(int ep_num) {
    int ok;
    uint32_t out_bit = (1u << ep_num);
    uint32_t in_bit  = (1u << (ep_num + 16));

    ENDPTFLUSH = out_bit;
    WAIT_CLEAR_TIMEOUT(ENDPTFLUSH, out_bit, 2000000, ok);
    if (!ok) {
        char buf[60];
        sprintf(buf, "flush OUT ep%d TIMEOUT", ep_num);
        screen_console << buf << nio::endl;
    }
    (void)ENDPTSTAT;

    ENDPTFLUSH = in_bit;
    WAIT_CLEAR_TIMEOUT(ENDPTFLUSH, in_bit, 2000000, ok);
    if (!ok) {
        char buf[60];
        sprintf(buf, "flush IN ep%d TIMEOUT", ep_num);
        screen_console << buf << nio::endl;
    }
    (void)ENDPTSTAT;
}

/* Safely copy the 8-byte setup packet out of the queue head using the
   hardware Setup TripWire. */
void read_setup_packet_safe(uint8_t *dst) {
    do {
        USBCMD |= USBCMD_SUTW;

        invalidate_dcache_line(&ep_list[0].setup_buffer[0]);
        drain_write_buffer();

        for (int i = 0; i < 8; i++) {
            dst[i] = ep_list[0].setup_buffer[i];
        }
    } while (!(USBCMD & USBCMD_SUTW));

    USBCMD &= ~USBCMD_SUTW;

    ENDPTSETUPSTAT = 0x00000001;
    int ok_clear;
    WAIT_CLEAR_TIMEOUT(ENDPTSETUPSTAT, 0x00000001, 2000000, ok_clear);
    if (!ok_clear) screen_console << "     ENDPTSETUPSTAT: clear TIMEOUT" << nio::endl;;
}

void ack_zero_length_status_phase() {
    // Pattern confirmed against the real Linux chipidea driver
    // (_hardware_enqueue in udc.c): the ACTIVE bit goes on the EXTERNAL
    // linked td, and the overlay only has HALTED+ACTIVE explicitly
    // CLEARED (real driver: qh->td.token &= ~(TD_STATUS_HALTED|
    // TD_STATUS_ACTIVE) -- note it clears BOTH bits, not just ACTIVE).
    dtd_tx->next_dtd = 0x00000001; // terminate, no further descriptor
    dtd_tx->token = (0 << 16) | (1 << 15) | 0x80; // 0 bytes, IOC, ACTIVE
    dtd_tx->buffer[0] = 0;

    ep_list[1].td.next_dtd = (uint32_t)dtd_tx;
    ep_list[1].td.token &= ~0xC0; // clear HALTED (bit6) + ACTIVE (bit7)

    clean_dcache_range(dtd_tx, sizeof(*dtd_tx));
    clean_dcache_range(&ep_list[1].td, sizeof(ep_list[1].td));

    ENDPTPRIME |= (1 << 16);
    print_ep0_in_state("     post-prime:");
}

/* Arm EP0 OUT to receive the status-phase ZLP that follows any IN-
   direction control data transfer (e.g. GET_DESCRIPTOR). The data
   phase and status phase run in OPPOSITE directions -- after sending
   descriptor bytes via EP0 IN, the host sends a zero-length OUT packet
   that we must be ready to receive and ACK. We were never arming EP0
   OUT for this at all, which matches exactly what the Mac's kernel log
   showed: "8 bytes transferred" (our data got through fine) followed
   by a timeout (the status handshake that comes after never got
   ACKed because nothing was listening for it). Reuses dtd_rx since
   EP1 bulk isn't active yet at this point in enumeration. */
void arm_ep0_out_status(void) {
    dtd_rx->next_dtd = 0x00000001;
    dtd_rx->token = (0 << 16) | (1 << 15) | 0x80; // 0 bytes, IOC, ACTIVE
    dtd_rx->buffer[0] = 0;

    ep_list[0].td.next_dtd = (uint32_t)dtd_rx;
    ep_list[0].td.token &= ~0xC0;

    clean_dcache_range(dtd_rx, sizeof(*dtd_rx));
    clean_dcache_range(&ep_list[0].td, sizeof(ep_list[0].td));

    ENDPTPRIME |= (1 << 0); // EP0 OUT prime bit

    int ok;
    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, (1 << 0), 5000000, ok);
    if (!ok) screen_console << "     arm_ep0_out_status: prime TIMEOUT" << nio::endl;

    int ok_complete;
    WAIT_SET_TIMEOUT(ENDPTCOMPLETE, (1 << 0), 5000000, ok_complete);
    if (!ok_complete) {
        screen_console << "     arm_ep0_out_status: complete TIMEOUT" << nio::endl;
    } else {
        ENDPTCOMPLETE = (1 << 0);
    }
}

/* Generic EP0 IN control-data transmit, used for any GET_DESCRIPTOR-
   style response. Clamps to requested_length (avoiding the "babble"
   bug fixed earlier), waits for real completion, and handles the
   status phase in the opposite direction. */
void transmit_ep0_data(const uint8_t *data, uint32_t actual_length, uint16_t requested_length) {
    uint32_t send_len = actual_length;
    if (requested_length < send_len) send_len = requested_length;

    dtd_tx->next_dtd = 0x00000001;
    dtd_tx->token = (send_len << 16) | (1 << 15) | 0x80;
    dtd_tx->buffer[0] = (uint32_t)data;

    ep_list[1].td.next_dtd = (uint32_t)dtd_tx;
    ep_list[1].td.token &= ~0xC0;

    clean_dcache_range(dtd_tx, sizeof(*dtd_tx));
    clean_dcache_range(&ep_list[1].td, sizeof(ep_list[1].td));
    clean_dcache_range(data, actual_length);

    ENDPTPRIME |= (1 << 16);

    int ok;
    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, (1 << 16), 5000000, ok);
    if (!ok) screen_console << "     transmit_ep0_data: TIMEOUT" << nio::endl;

    int ok_complete;
    WAIT_SET_TIMEOUT(ENDPTCOMPLETE, (1 << 16), 5000000, ok_complete);
    if (!ok_complete) {
        screen_console << "     transmit_ep0_data: complete TIMEOUT" << nio::endl;
    } else {
        ENDPTCOMPLETE = (1 << 16);
    }

    // Data phase done -- now handle the status phase, which runs in
    // the OPPOSITE direction (host sends a ZLP OUT that we must ACK).
    arm_ep0_out_status();
}

void transmit_device_descriptor(uint16_t requested_length) {
    // FIX: we were sending all 18 bytes regardless of what the host
    // asked for. USB hosts commonly send the very first GET_DESCRIPTOR
    // request with wLength=8 (just enough to learn bMaxPacketSize0),
    // before a port reset and a follow-up request for the full 18
    // bytes. Sending more data than requested is a protocol violation
    // ("babble") that gets reported as a transaction error with the
    // whole transfer discarded -- matching exactly the "0 bytes
    // transferred" / "descriptor fragment is invalid" errors seen.
    transmit_ep0_data(device_descriptor, sizeof(device_descriptor), requested_length);
}

void transmit_config_descriptor(uint16_t requested_length) {
    transmit_ep0_data(config_descriptor, sizeof(config_descriptor), requested_length);
}

void setup_endpoint_list() {
    memset(ep_list, 0, sizeof(DeviceQueueHead) * 4);

    ep_list[0].config = (0x40 << 16) | (1 << 15);
    ep_list[1].config = (0x40 << 16);
    ep_list[2].config = (0x40 << 16);
    ep_list[3].config = (0x40 << 16);

    // current_dtd is the one queue-head field we'd never touched.
    // After memset it's raw 0x00000000 -- Terminate bit (bit0) CLEAR,
    // meaning "valid pointer to address 0" rather than "empty/no
    // descriptor". Explicitly terminate it so hardware doesn't think
    // there's already a (bogus) descriptor in flight before we ever
    // prime anything.
    ep_list[0].current_dtd = 0x00000001;
    ep_list[1].current_dtd = 0x00000001;
    ep_list[2].current_dtd = 0x00000001;
    ep_list[3].current_dtd = 0x00000001;

    drain_write_buffer();
    clean_dcache_range(ep_list, sizeof(DeviceQueueHead) * 4);

    // Bind this block pointer to the controller hardware -- restored
    // here (rather than inside the removed boot-phase2) to match the
    // original working order: reset, bind list, THEN wait for host.
    ENDPOINTLISTADDR = (uint32_t)ep_list;

    print_alignment_check("ep_list", ep_list, EP_LIST_ALIGN);
    print_alignment_check("dtd_tx", dtd_tx, DTD_ALIGN);
    print_alignment_check("dtd_rx", dtd_rx, DTD_ALIGN);
}

void send_test_message(const char* msg, uint32_t length) {
    dtd_tx->next_dtd = 0x00000001;
    dtd_tx->token = (length << 16) | (1 << 15) | 0x80;
    dtd_tx->buffer[0] = (uint32_t)msg;

    ep_list[3].td.next_dtd = (uint32_t)dtd_tx;
    ep_list[3].td.token &= ~0xC0;

    clean_dcache_range(dtd_tx, sizeof(*dtd_tx));
    clean_dcache_range(&ep_list[3].td, sizeof(ep_list[3].td));
    clean_dcache_range(msg, length);

    ENDPTPRIME |= (1 << 17);

    int ok;
    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, (1 << 17), 5000000, ok);
    if (!ok) screen_console << "     send_test_message: TIMEOUT" << nio::endl;

    int ok_complete;
    WAIT_SET_TIMEOUT(ENDPTCOMPLETE, (1 << 17), 5000000, ok_complete);
    if (!ok_complete) {
        screen_console << "     send_test_message: complete TIMEOUT" << nio::endl;
    } else {
        ENDPTCOMPLETE = (1 << 17);
    }
}

void arm_bulk_out() {
    dtd_rx->next_dtd = 0x00000001;
    dtd_rx->token = (64 << 16) | (1 << 15) | 0x80;
    dtd_rx->buffer[0] = (uint32_t)rx_buffer;

    ep_list[2].td.next_dtd = (uint32_t)dtd_rx;
    ep_list[2].td.token &= ~0xC0;

    clean_dcache_range(dtd_rx, sizeof(*dtd_rx));
    clean_dcache_range(&ep_list[2].td, sizeof(ep_list[2].td));
    drain_write_buffer();

    ENDPTPRIME |= (1 << 1);

    // Unlike every other transfer function, this never checked whether
    // its own PRIME actually succeeded. Only checking PRIME here (NOT
    // ENDPTCOMPLETE) -- PRIME clearing means hardware accepted the
    // queued request and is now actively listening, which should
    // happen quickly regardless of whether the host has sent anything
    // yet. Waiting for ENDPTCOMPLETE here would block until real data
    // arrives, which defeats the fire-and-poll design (arm once,
    // service_bulk_endpoints() checks for completion later).
    int ok;
    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, (1 << 1), 5000000, ok);
    if (!ok) {
        screen_console << "     arm_bulk_out: prime TIMEOUT" << nio::endl;
    }
}

void usb_device_shutdown() {
    int ok;
    USBCMD &= ~USBCMD_RS;

    ENDPTFLUSH = 0xFFFFFFFF;
    WAIT_CLEAR_TIMEOUT(ENDPTFLUSH, 0xFFFFFFFF, 2000000, ok);

    ENDPTCTRL1 = 0;

    ENDPTCOMPLETE = 0xFFFFFFFF;
    USBSTS = USBSTS;

    OTGSC &= ~(1u << 5);
    busy_wait(200000);

    USBCMD |= USBCMD_RST;
    WAIT_CLEAR_TIMEOUT(USBCMD, USBCMD_RST, 2000000, ok);

    USBINTR = 0;
    DEVICEADDR = 0;

    usb_device_configured = 0;
    usb_should_exit = 0;
}

void setup_minimal_usb_stack() {
    uint32_t heartbeat = 0;

    while (!usb_device_configured) {
        if (isKeyPressed(KEY_NSPIRE_ESC)) {
            usb_should_exit = 1;
            return;
        }

        // Heartbeat so it's visible whether this loop is alive and
        // simply waiting for the host to send something, vs. actually
        // hung. Also dumps live register state so we can see if the
        // port even thinks it's connected/reset.
        heartbeat++;
        if (heartbeat % 4000000 == 0) {
            char hb[100];
            sprintf(hb, "     alive: PORTSC1=0x%x OTGSC=0x%x USBSTS=0x%x",
                    PORTSC1, OTGSC, USBSTS);
            screen_console << hb << nio::endl;
        }

        if (ENDPTSETUPSTAT & 0x00000001) {

            // Mirror the native trace's port-reset-style handling:
            // ack status, flush EP0 both directions BEFORE touching
            // the setup buffer, matching the observed write order.
            USBSTS = 0x00000005;
            (void)ENDPTSETUPSTAT;
            (void)ENDPTCOMPLETE;
            flush_endpoint(0); // EP0, both directions

            uint8_t setup_local[8];
            read_setup_packet_safe(setup_local);
            print_setup_hex(setup_local);

            uint8_t  bmRequestType = setup_local[0];
            uint8_t  bRequest      = setup_local[1];
            uint16_t wValue        = setup_local[2] | (setup_local[3] << 8);
            uint16_t wIndex        = setup_local[4] | (setup_local[5] << 8);
            uint16_t wLength       = setup_local[6] | (setup_local[7] << 8);
            (void)bmRequestType;
            (void)wIndex;

            char strbuf[100];
            sprintf(strbuf, "     s.bRequest=0x%x", bRequest);
            screen_console << strbuf << nio::endl;
            sprintf(strbuf, "     s.wValue=0x%x", wValue);
            screen_console << strbuf << nio::endl;
            sprintf(strbuf, "     s.wLength=0x%x", wLength);
            screen_console << strbuf << nio::endl;

            if (bRequest == 0x05) { // SET_ADDRESS
                uint32_t new_addr = (uint32_t)(wValue & 0x7F);

                // FIX: use the USBADRA "advance" bit (bit 24), matching
                // what the native trace actually wrote (0x03000000 =
                // (1<<25)|(1<<24), address 1 WITH this bit set -- not
                // "address 1 shifted by 24" as first guessed, and not
                // "just the plain shift" as later assumed once the
                // generic Linux driver's deliberate avoidance of this
                // bit was found). USBADRA tells hardware to defer
                // actually applying the new address until the in-flight
                // status-phase transaction genuinely completes on the
                // wire, instead of applying it the instant this register
                // is written. Without it, our software has to guess
                // when it's safe to switch addresses -- and if hardware
                // adopts the new address even slightly before the host
                // has truly finished receiving the ZLP, the very next
                // packet (GET_DESCRIPTOR) would be silently dropped,
                // which matches exactly what we're seeing: SET_ADDRESS
                // completes cleanly on our end every time, but
                // GET_DESCRIPTOR never arrives at all, and the host
                // resets and retries the whole enumeration from
                // scratch. With USBADRA, hardware handles this
                // synchronization atomically, so writing DEVICEADDR
                // before priming (matching the trace's literal order)
                // is safe rather than racy.
                DEVICEADDR = (new_addr << 25) | (1u << 24);

                ack_zero_length_status_phase();

                int ok_addr;
                WAIT_CLEAR_TIMEOUT(ENDPTPRIME, 0x00010000, 5000000, ok_addr);
                if (!ok_addr) {
                    screen_console << "     SET_ADDRESS: prime TIMEOUT" << nio::endl;
                    print_ep0_in_state("     post-timeout:");
                }

                int ok_complete;
                WAIT_SET_TIMEOUT(ENDPTCOMPLETE, 0x00010000, 5000000, ok_complete);
                if (!ok_complete) {
                    screen_console << "     SET_ADDRESS: complete TIMEOUT" << nio::endl;
                } else {
                    ENDPTCOMPLETE = 0x00010000; // write-1-to-clear
                }
            }
            else if (bRequest == 0x06) { // GET_DESCRIPTOR
                uint8_t desc_type = (wValue >> 8) & 0xFF;
                if (desc_type == 0x01) {
                    transmit_device_descriptor(wLength);
                } else if (desc_type == 0x02) {
                    transmit_config_descriptor(wLength);
                }
            }
            else if (bRequest == 0x09) { // SET_CONFIGURATION
                uint8_t config_val = wValue & 0xFF;

                if (config_val > 0) {
                    ack_zero_length_status_phase();
                    int ok_cfg;
                    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, 0x00010000, 5000000, ok_cfg);
                    if (!ok_cfg) screen_console << "     SET_CONFIGURATION: prime TIMEOUT" << nio::endl;

                    int ok_cfg_complete;
                    WAIT_SET_TIMEOUT(ENDPTCOMPLETE, 0x00010000, 5000000, ok_cfg_complete);
                    if (!ok_cfg_complete) {
                        screen_console << "     SET_CONFIGURATION: complete TIMEOUT" << nio::endl;
                    } else {
                        ENDPTCOMPLETE = 0x00010000;
                    }

                    flush_endpoint(1); // EP1, both directions

                    ENDPTCTRL1 = ENDPTCTRL_TXE | ENDPTCTRL_TXR | ENDPTCTRL_TXT_BULK;
                    ENDPTCTRL1 |= ENDPTCTRL_RXE | ENDPTCTRL_RXR | ENDPTCTRL_RXT_BULK;

                    arm_bulk_out();
                    print_ep1_out_state("     initial arm_bulk_out:");

                    usb_device_configured = 1;
                }
            }
        }
    }
}

void service_bulk_endpoints() {
    uint32_t complete = ENDPTCOMPLETE;

    if (complete & (1 << 1)) {           // EP1 OUT complete
        ENDPTCOMPLETE = (1 << 1);

        // Figure out how many bytes actually arrived. Token bits
        // 16-30 hold "bytes remaining to transfer" -- hardware
        // decrements this as data comes in, so what we actually
        // received = what we asked for (64) minus what's left over.
        invalidate_dcache_line(dtd_rx);
        invalidate_dcache_line(((uint8_t*)dtd_rx) + 32);
        drain_write_buffer();

        uint32_t remaining = (dtd_rx->token >> 16) & 0x7FFF;
        uint32_t received = 64 - remaining;

        if (received > 0 && received <= 64) {
            invalidate_dcache_line(rx_buffer);
            invalidate_dcache_line(rx_buffer + 32);
            drain_write_buffer();

            char header[40];
            sprintf(header, "RX %u bytes:", (unsigned int)received);
            screen_console << header << nio::endl;

            // Show as text where printable, '.' elsewhere -- good
            // enough for a quick look without needing a full hex dump.
            char textbuf[68];
            uint32_t n = received;
            if (n > 63) n = 63;
            for (uint32_t i = 0; i < n; i++) {
                uint8_t c = rx_buffer[i];
                textbuf[i] = (c >= 32 && c < 127) ? (char)c : '.';
            }
            textbuf[n] = '\0';
            screen_console << textbuf << nio::endl;
        }

        arm_bulk_out(); // re-arm for the next packet
    }

    if (complete & (1 << 17)) {
        ENDPTCOMPLETE = (1 << 17);
    }
}

int main(void) {
    init_aligned_pointers();

    // Capture whatever the native OS had configured BEFORE we change
    // anything, so it can be restored on exit instead of leaving
    // native USB unusable until a hard reset.
    save_native_usb_state();

    screen_console << "Initializing USB Controller Hardware..." << nio::endl;
    reset_usb_subsystem();
    setup_endpoint_list();

    // We have NEVER written ENDPTCTRL0 anywhere in this program. On
    // some ChipIdea-family controllers EP0 is hard-wired always-enabled
    // by hardware (since control transfers must always work), but on
    // others it may need explicit enabling just like EP1 does. This
    // tells us which case we're in.
    {
        char buf[80];
        sprintf(buf, "baseline ENDPTCTRL0=0x%x", ENDPTCTRL0);
        screen_console << buf << nio::endl;
        sprintf(buf, "PORTSC1=0x%x (PFSC=%d)", (unsigned int)PORTSC1,
                (PORTSC1 & (1u << 24)) ? 1 : 0);
        screen_console << buf << nio::endl;
    }

    screen_console << "Waiting for USB host link..." << nio::endl;
    while (!(OTGSC & OTGSC_BSV)) {
        if (isKeyPressed(KEY_NSPIRE_ESC)) {
            usb_device_shutdown();
            restore_native_usb_state();
            return 0;
        }
    }
    screen_console << "Connected!" << nio::endl;

    screen_console << "Setting up usb stack..." << nio::endl;
    USBCMD |= USBCMD_RS; // fire up the engine -- Run/Stop bit
    setup_minimal_usb_stack();

    if (usb_should_exit) {
        usb_device_shutdown();
        restore_native_usb_state();
        return 0;
    }

    const char* test_payload = "HELLO FROM THE BARE METAL CALCULATOR!";
    uint32_t payload_len = (uint32_t)strlen(test_payload);

    // Resend periodically instead of once. A single one-shot send
    // right after SET_CONFIGURATION completes has no guarantee a host
    // program is listening yet -- if nothing reads EP1 IN in time it
    // just times out and is gone for good. Resending every ~2 seconds
    // means whenever a host-side test program opens the endpoint, it
    // will catch the next attempt regardless of timing.
    uint32_t bulk_heartbeat = 0;
    bool sent = false;
    while (!isKeyPressed(KEY_NSPIRE_ESC)) {
        service_bulk_endpoints();

        if (!sent) {
            send_test_message(test_payload, payload_len);
            sent = true;
        }
    }

    usb_device_shutdown();
    restore_native_usb_state();
    return 0;
}