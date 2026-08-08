#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstddef>
#include <nspireio/console.hpp>

static nio::console screen_console;
static FILE *g_log_file = NULL;

static inline void dbg_print(const char *msg) {
#ifdef DEBUG_USB
    screen_console << msg << nio::endl;
    if (g_log_file) {
        fprintf(g_log_file, "%s\n", msg);
        fflush(g_log_file);
    }
#else
    (void)msg;
#endif
}

static inline void open_log(void) {
#ifdef DEBUG_USB
    g_log_file = fopen("/documents/usb_debug.txt.tns", "w");
    if (g_log_file) {
        fprintf(g_log_file, "=== USB debug log ===\n");
        fflush(g_log_file);
    } else {
        dbg_print("WARNING: could not open /documents/usb_debug.txt for logging");
    }
#endif
}

static inline void close_log(void) {
    if (g_log_file) {
        fclose(g_log_file);
        g_log_file = NULL;
    }
}

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
    volatile uint8_t setup_buffer[8];
    volatile uint8_t _pad[16];
} __attribute__((packed)) DeviceQueueHead;

_Static_assert(__builtin_offsetof(DeviceQueueHead, setup_buffer) == 40,
               "DeviceQueueHead.setup must be at hardware offset 40");
_Static_assert(sizeof(DeviceQueueHead) == 64, "DeviceQueueHead must be exactly 64 bytes (64-byte hardware slot, not 48-byte packed struct)");

#define EP_LIST_ALIGN   2048u
#define DTD_ALIGN       32u

static uint8_t ep_list_raw[sizeof(DeviceQueueHead) * 6 + EP_LIST_ALIGN];
static uint8_t dtd_tx_raw[sizeof(DeviceTransferDescriptor) + DTD_ALIGN];
static uint8_t dtd_rx_raw[sizeof(DeviceTransferDescriptor) + DTD_ALIGN];
static uint8_t dtd_kbd_tx_raw[sizeof(DeviceTransferDescriptor) + DTD_ALIGN];
static uint8_t dtd_mouse_tx_raw[sizeof(DeviceTransferDescriptor) + DTD_ALIGN];
#define HID_EP_LIST_ENTRIES 6

static DeviceQueueHead *ep_list;
static DeviceTransferDescriptor *dtd_tx;
static DeviceTransferDescriptor *dtd_rx;
static DeviceTransferDescriptor *dtd_kbd_tx;
static DeviceTransferDescriptor *dtd_mouse_tx;

static inline uint32_t align_up(uint32_t addr, uint32_t align) {
    return (addr + (align - 1)) & ~(align - 1);
}

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

#define USBCTRL_BASE 0xB0000000
#define REG32(off) (*(volatile uint32_t*)(USBCTRL_BASE + (off)))

#define HWDEVICE         REG32(0x00C)
#define CAPLENGTH        REG32(0x100)
#define HCSPARAMS        REG32(0x104)

#define USBCMD           REG32(0x140)
#define USBSTS           REG32(0x144)
#define USBINTR          REG32(0x148)
#define FRINDEX          REG32(0x14C)
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
#define ENDPTCTRL2       REG32(0x1C8)

#define USBCMD_RS        (1u << 0)
#define USBCMD_RST       (1u << 1)
#define USBCMD_SUTW      (1u << 13)

#define ENDPTCTRL_TXE      (1u << 23)
#define ENDPTCTRL_TXR      (1u << 22)
#define ENDPTCTRL_TXT_BULK (2u << 18)
#define ENDPTCTRL_TXT_INTERRUPT (3u << 18)
#define ENDPTCTRL_TXS      (1u << 16)

#define ENDPTCTRL_RXE      (1u << 7)
#define ENDPTCTRL_RXR      (1u << 6)
#define ENDPTCTRL_RXT_BULK (2u << 2)
#define ENDPTCTRL_RXS      (1u << 0)

#define OTGSC_BSV         (1u << 11)

/* Power management block -- a DIFFERENT peripheral entirely from the
   USB OTG controller (900B0000 vs B0000000). Per hackspire.org:
     "900B0018 (R/W): Disable bus access to peripherals. Reads will
      just return the last word read from anywhere in the address
      range, and writes will be ignored." -- Bit 5: USB OTG controller.
   Without clearing this, is_usb_port_connected() (and every other USB
   register read) returns stale garbage completely disconnected from
   real hardware state. */
#define PWR_MGMT_BASE 0x900B0000
#define PWR_REG32(off) (*(volatile uint32_t*)(PWR_MGMT_BASE + (off)))
#define PWR_BUS_DISABLE          PWR_REG32(0x0018)
#define PWR_BUS_DISABLE_USB_OTG  (1u << 5)

static inline void ensure_usb_bus_access_enabled(void) {
    PWR_BUS_DISABLE &= ~PWR_BUS_DISABLE_USB_OTG;

    // CONFIRMED WORKING (2026-07): a second, previously-undocumented
    // gate. Found empirically, not from documentation, by dumping
    // 900B0020 -- hackspire.org's unconfirmed "possibly another
    // peripheral bus access disable register" -- and comparing a
    // cable-connected-at-boot run against a cable-disconnected-at-boot
    // run. Bits 2 and 10 were set ONLY in the disconnected case
    // (0xc1c vs 0x818, everything else identical), matching the same
    // "1 = disabled" convention as the confirmed 900B0018. Clearing
    // both bits here is what actually fixes booting with the USB
    // cable disconnected -- without this, reset_usb_subsystem()'s
    // first register write hangs the CPU completely (real hardware
    // bus stall, requires a physical reset to recover).
    PWR_REG32(0x0020) &= ~((1u << 2) | (1u << 10));
}

static const uint8_t device_descriptor[18] = {
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0x09, 0x12, 0x02, 0x00, 0x00, 0x01, 0x01, 0x02,
    0x03, 0x01
};

// Composite HID device: interface 0 = keyboard (boot subclass), interface
// 1 = mouse (boot subclass). See usb_hid.h for the report descriptors
// this HID descriptor's wDescriptorLength fields refer to.
static const uint8_t config_descriptor[59] = {
    // Configuration descriptor
    0x09, 0x02, 0x3B, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,
    // Interface 0: Keyboard
    0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00,
    // HID descriptor for interface 0 (report descriptor is 63 bytes)
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x3F, 0x00,
    // Endpoint descriptor: EP1 IN, interrupt, 8 bytes, 10ms poll
    0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0A,
    // Interface 1: Mouse
    0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00,
    // HID descriptor for interface 1 (report descriptor is 62 bytes)
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x3E, 0x00,
    // Endpoint descriptor: EP2 IN, interrupt, 8 bytes, 10ms poll
    0x07, 0x05, 0x82, 0x03, 0x08, 0x00, 0x0A,
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

static inline void invalidate_dcache_range(const volatile void *addr, uint32_t size) {
    uint32_t start = (uint32_t)addr & ~31u;
    uint32_t end   = ((uint32_t)addr + size + 31u) & ~31u;
    for (uint32_t a = start; a < end; a += 32) {
        asm volatile("mcr p15, 0, %0, c7, c6, 1" : : "r"(a) : "memory");
    }
    drain_write_buffer();
}

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

#define WAIT_CLEAR_TIMEOUT(reg, mask, iters, ok_flag) \
    do { \
        int _i = (iters); \
        while (((reg) & (mask)) && _i > 0) { _i--; } \
        (ok_flag) = !((reg) & (mask)); \
    } while (0)

#define WAIT_SET_TIMEOUT(reg, mask, iters, ok_flag) \
    do { \
        int _i = (iters); \
        while (!((reg) & (mask)) && _i > 0) { _i--; } \
        (ok_flag) = ((reg) & (mask)) != 0; \
    } while (0)

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

void restore_native_usb_state(void) {
    if (!native_usb_state_saved) return;

    int ok;
    USBCMD &= ~USBCMD_RS;
    USBCMD |= USBCMD_RST;
    WAIT_CLEAR_TIMEOUT(USBCMD, USBCMD_RST, 2000000, ok);
    if (!ok) {
        dbg_print("     restore_native_usb_state: reset TIMEOUT");
    }

    USBMODE          = native_usb_state.usbmode;
    PORTSC1           = native_usb_state.portsc1;
    OTGSC             = native_usb_state.otgsc;
    DEVICEADDR        = native_usb_state.deviceaddr;
    ENDPOINTLISTADDR  = native_usb_state.eplistaddr;
    ENDPTCTRL0        = native_usb_state.endptctrl0;
    ENDPTCTRL1        = native_usb_state.endptctrl1;
    USBINTR           = native_usb_state.usbintr;
    USBCMD            = native_usb_state.usbcmd;
}

void print_alignment_check(const char *label, void *addr, uint32_t required) {
    char strbuf[100];
    uint32_t a = (uint32_t)addr;
    uint32_t rem = a & (required - 1);
    sprintf(strbuf, "%s @ 0x%x (align%%%d = 0x%x)%s",
            label, (unsigned int)a, (int)required, (unsigned int)rem,
            rem == 0 ? " OK" : " *** MISALIGNED ***");
    dbg_print(strbuf);
}

void print_setup_hex(const uint8_t *buf) {
    char strbuf[100];
    sprintf(strbuf, "     setup raw: %02x %02x %02x %02x %02x %02x %02x %02x",
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    dbg_print(strbuf);
}

void print_ep0_in_state(const char *label) {
    invalidate_dcache_line(&ep_list[1].td);
    invalidate_dcache_line(((uint8_t*)&ep_list[1].td) + 32);
    invalidate_dcache_line(&ep_list[1].current_dtd);
    drain_write_buffer();

    char buf[80];
    dbg_print(label);
    sprintf(buf, "tok=0x%x", (unsigned int)ep_list[1].td.token);
    dbg_print(buf);
    sprintf(buf, "nxt=0x%x cur=0x%x", (unsigned int)ep_list[1].td.next_dtd,
            (unsigned int)ep_list[1].current_dtd);
    dbg_print(buf);
    sprintf(buf, "PRM=0x%x STAT=0x%x", (unsigned int)ENDPTPRIME, (unsigned int)ENDPTSTAT);
    dbg_print(buf);
    sprintf(buf, "LISTADDR=0x%x want=0x%x", (unsigned int)ENDPOINTLISTADDR,
            (unsigned int)ep_list);
    dbg_print(buf);
    sprintf(buf, "CTRL0=0x%x", (unsigned int)ENDPTCTRL0);
    dbg_print(buf);
    sprintf(buf, "USBCMD=0x%x", (unsigned int)USBCMD);
    dbg_print(buf);
}

void print_ep1_out_state(const char *label) {
    invalidate_dcache_line(&ep_list[2].td);
    invalidate_dcache_line(((uint8_t*)&ep_list[2].td) + 32);
    invalidate_dcache_line(&ep_list[2].current_dtd);
    drain_write_buffer();

    char buf[80];
    dbg_print(label);
    sprintf(buf, "tok=0x%x", (unsigned int)ep_list[2].td.token);
    dbg_print(buf);
    sprintf(buf, "nxt=0x%x cur=0x%x", (unsigned int)ep_list[2].td.next_dtd,
            (unsigned int)ep_list[2].current_dtd);
    dbg_print(buf);
    sprintf(buf, "PRM=0x%x STAT=0x%x", (unsigned int)ENDPTPRIME, (unsigned int)ENDPTSTAT);
    dbg_print(buf);
    sprintf(buf, "COMPLETE=0x%x CTRL1=0x%x", (unsigned int)ENDPTCOMPLETE, (unsigned int)ENDPTCTRL1);
    dbg_print(buf);
}

void init_aligned_pointers(void) {
    ep_list      = (DeviceQueueHead *)align_up((uint32_t)ep_list_raw, EP_LIST_ALIGN);
    dtd_tx       = (DeviceTransferDescriptor *)align_up((uint32_t)dtd_tx_raw, DTD_ALIGN);
    dtd_rx       = (DeviceTransferDescriptor *)align_up((uint32_t)dtd_rx_raw, DTD_ALIGN);
    dtd_kbd_tx   = (DeviceTransferDescriptor *)align_up((uint32_t)dtd_kbd_tx_raw, DTD_ALIGN);
    dtd_mouse_tx = (DeviceTransferDescriptor *)align_up((uint32_t)dtd_mouse_tx_raw, DTD_ALIGN);
}

void reset_usb_subsystem(void) {

    USBCMD &= ~USBCMD_RS;

    USBCMD |= USBCMD_RST;
    while (USBCMD & USBCMD_RST) { }

    USBMODE = 0x00000002 | 0x00000008;

    PORTSC1 = 0xED000004;

    OTGSC = 0x007F0020;

    DEVICEADDR = 0;
}

void flush_endpoint(int ep_num) {
    int ok;
    uint32_t out_bit = (1u << ep_num);
    uint32_t in_bit  = (1u << (ep_num + 16));

    ENDPTFLUSH = out_bit;
    WAIT_CLEAR_TIMEOUT(ENDPTFLUSH, out_bit, 2000000, ok);
    if (!ok) {
        char buf[60];
        sprintf(buf, "flush OUT ep%d TIMEOUT", ep_num);
        dbg_print(buf);
    }
    (void)ENDPTSTAT;

    ENDPTFLUSH = in_bit;
    WAIT_CLEAR_TIMEOUT(ENDPTFLUSH, in_bit, 2000000, ok);
    if (!ok) {
        char buf[60];
        sprintf(buf, "flush IN ep%d TIMEOUT", ep_num);
        dbg_print(buf);
    }
    (void)ENDPTSTAT;
}

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
    if (!ok_clear) dbg_print("     ENDPTSETUPSTAT: clear TIMEOUT");;
}

void ack_zero_length_status_phase() {
    dtd_tx->next_dtd = 0x00000001;
    dtd_tx->token = (0 << 16) | (1 << 15) | 0x80;
    dtd_tx->buffer[0] = 0;

    ep_list[1].td.next_dtd = (uint32_t)dtd_tx;
    ep_list[1].td.token &= ~0xC0;

    clean_dcache_range(dtd_tx, sizeof(*dtd_tx));
    clean_dcache_range(&ep_list[1].td, sizeof(ep_list[1].td));

    ENDPTPRIME |= (1 << 16);
    print_ep0_in_state("     post-prime:");
}

void arm_ep0_out_status(void) {
    dtd_rx->next_dtd = 0x00000001;
    dtd_rx->token = (0 << 16) | (1 << 15) | 0x80;
    dtd_rx->buffer[0] = 0;

    ep_list[0].td.next_dtd = (uint32_t)dtd_rx;
    ep_list[0].td.token &= ~0xC0;

    clean_dcache_range(dtd_rx, sizeof(*dtd_rx));
    clean_dcache_range(&ep_list[0].td, sizeof(ep_list[0].td));

    ENDPTPRIME |= (1 << 0);

    int ok;
    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, (1 << 0), 5000000, ok);
    if (!ok) dbg_print("     arm_ep0_out_status: prime TIMEOUT");

    int ok_complete;
    WAIT_SET_TIMEOUT(ENDPTCOMPLETE, (1 << 0), 5000000, ok_complete);
    if (!ok_complete) {
        dbg_print("     arm_ep0_out_status: complete TIMEOUT");
    } else {
        ENDPTCOMPLETE = (1 << 0);
    }
}

void transmit_ep0_data(const uint8_t *data, uint32_t actual_length, uint16_t requested_length) {
    uint32_t send_len = actual_length;
    if (requested_length < send_len) send_len = requested_length;

    dtd_tx->next_dtd = 0x00000001;
    dtd_tx->token = (send_len << 16) | (1 << 15) | 0x80;
    dtd_tx->buffer[0] = (uint32_t)data;
    {
        uint32_t page_base = (uint32_t)data & ~0xFFFu;
        for (int p = 1; p < 5; p++) {
            dtd_tx->buffer[p] = page_base + ((uint32_t)p * 0x1000u);
        }
    }

    ep_list[1].td.next_dtd = (uint32_t)dtd_tx;
    ep_list[1].td.token &= ~0xC0;

    clean_dcache_range(dtd_tx, sizeof(*dtd_tx));
    clean_dcache_range(&ep_list[1].td, sizeof(ep_list[1].td));
    clean_dcache_range(data, actual_length);

    ENDPTPRIME |= (1 << 16);

    int ok;
    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, (1 << 16), 5000000, ok);
    if (!ok) dbg_print("     transmit_ep0_data: TIMEOUT");

    int ok_complete;
    WAIT_SET_TIMEOUT(ENDPTCOMPLETE, (1 << 16), 5000000, ok_complete);
    if (!ok_complete) {
        dbg_print("     transmit_ep0_data: complete TIMEOUT");
    } else {
        ENDPTCOMPLETE = (1 << 16);
    }

    arm_ep0_out_status();
}

void transmit_device_descriptor(uint16_t requested_length) {
    transmit_ep0_data(device_descriptor, sizeof(device_descriptor), requested_length);
}

void transmit_config_descriptor(uint16_t requested_length) {
    transmit_ep0_data(config_descriptor, sizeof(config_descriptor), requested_length);
}

void setup_endpoint_list() {
    memset(ep_list, 0, sizeof(DeviceQueueHead) * HID_EP_LIST_ENTRIES);

    // FIX: bit 29 of the queue head's capabilities word is ZLT (Zero
    // Length Termination Select). Default is 0, which means hardware
    // AUTOMATICALLY appends a zero-length packet after any transfer
    // whose length is an exact multiple of max packet size. Confirmed
    // via tinyusb's ChipIdea driver source and a real Linux kernel
    // stable-tree patch for this exact hardware family. Setting ZLT=1
    // disables this -- HID reports are fixed-size and never want an
    // extra trailing packet.
    #define QH_ZLT (1u << 29)
    ep_list[0].config = (0x40 << 16) | (1 << 15) | QH_ZLT; // EP0 OUT
    ep_list[1].config = (0x40 << 16) | QH_ZLT;              // EP0 IN
    ep_list[2].config = (0x08 << 16) | QH_ZLT;              // EP1 OUT (unused, but must be valid)
    ep_list[3].config = (0x08 << 16) | QH_ZLT;              // EP1 IN (keyboard reports)
    ep_list[4].config = (0x08 << 16) | QH_ZLT;              // EP2 OUT (unused, but must be valid)
    ep_list[5].config = (0x08 << 16) | QH_ZLT;              // EP2 IN (mouse reports)

    for (int i = 0; i < HID_EP_LIST_ENTRIES; i++) {
        ep_list[i].current_dtd = 0x00000001;
    }

    drain_write_buffer();
    clean_dcache_range(ep_list, sizeof(DeviceQueueHead) * HID_EP_LIST_ENTRIES);

    ENDPOINTLISTADDR = (uint32_t)ep_list;

    // Same undocumented-quirk experiment carried over from the MSC
    // project: enabling these interrupt status-latch bits even though
    // we're fully polled, not actually interrupt-driven.
    #define USBINTR_UI  (1u << 0)
    #define USBINTR_UEI (1u << 1)
    #define USBINTR_PCI (1u << 2)
    #define USBINTR_URI (1u << 6)
    USBINTR = USBINTR_UI | USBINTR_UEI | USBINTR_PCI | USBINTR_URI;

    print_alignment_check("ep_list", ep_list, EP_LIST_ALIGN);
    print_alignment_check("dtd_tx", dtd_tx, DTD_ALIGN);
    print_alignment_check("dtd_rx", dtd_rx, DTD_ALIGN);
}

void print_ep1_in_state(const char *label) {
    invalidate_dcache_line(&ep_list[3].td);
    invalidate_dcache_line(((uint8_t*)&ep_list[3].td) + 32);
    invalidate_dcache_line(&ep_list[3].current_dtd);
    drain_write_buffer();

    char buf[80];
    dbg_print(label);
    sprintf(buf, "tok=0x%x", (unsigned int)ep_list[3].td.token);
    dbg_print(buf);
    sprintf(buf, "nxt=0x%x cur=0x%x", (unsigned int)ep_list[3].td.next_dtd,
            (unsigned int)ep_list[3].current_dtd);
    dbg_print(buf);
    sprintf(buf, "PRM=0x%x STAT=0x%x", (unsigned int)ENDPTPRIME, (unsigned int)ENDPTSTAT);
    dbg_print(buf);
    sprintf(buf, "COMPLETE=0x%x CTRL1=0x%x", (unsigned int)ENDPTCOMPLETE, (unsigned int)ENDPTCTRL1);
    dbg_print(buf);
    sprintf(buf, "USBSTS=0x%x", (unsigned int)USBSTS);
    dbg_print(buf);
}

typedef void (*usb_setup_callback_t)();
static usb_setup_callback_t g_usb_setup_callback = NULL;

// Called for any control request service_control_endpoint() doesn't
// natively recognize (e.g. GET_DESCRIPTOR for a type it doesn't send
// itself, or a class-specific bRequest) -- lets device-specific control
// handling live in a registered callback instead of a parallel
// dispatcher.
typedef void (*usb_unhandled_control_callback_t)(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength);
static usb_unhandled_control_callback_t g_usb_unhandled_control_callback = NULL;

// Sends one keyboard report (8 bytes: modifier, reserved, 6 keycodes)
// via EP1 IN (index 3 in ep_list, prime/complete bit 17). Blocks until
// the hardware confirms completion, same pattern as every other send
// in this codebase.
void send_keyboard_report(const uint8_t *report, uint32_t length) {
    dtd_kbd_tx->next_dtd = 0x00000001;
    dtd_kbd_tx->token = (length << 16) | (1 << 15) | 0x80;
    dtd_kbd_tx->buffer[0] = (uint32_t)report;
    {
        uint32_t page_base = (uint32_t)report & ~0xFFFu;
        for (int p = 1; p < 5; p++) {
            dtd_kbd_tx->buffer[p] = page_base + ((uint32_t)p * 0x1000u);
        }
    }

    ep_list[3].td.next_dtd = (uint32_t)dtd_kbd_tx;
    ep_list[3].td.token &= ~0xC0;

    clean_dcache_range(dtd_kbd_tx, sizeof(*dtd_kbd_tx));
    clean_dcache_range(&ep_list[3].td, sizeof(ep_list[3].td));
    clean_dcache_range(report, length);
    drain_write_buffer();

    ENDPTPRIME |= (1 << 17);

    int ok;
    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, (1 << 17), 5000000, ok);
    if (!ok) dbg_print("     send_keyboard_report: prime TIMEOUT");

    int ok_complete;
    WAIT_SET_TIMEOUT(ENDPTCOMPLETE, (1 << 17), 5000000, ok_complete);
    if (!ok_complete) {
        dbg_print("     send_keyboard_report: complete TIMEOUT");
    } else {
        ENDPTCOMPLETE = (1 << 17);
    }
}

// Sends one mouse report (4 bytes: buttons, dX, dY, wheel) via EP2 IN
// (index 5 in ep_list, prime/complete bit 18).
void send_mouse_report(const uint8_t *report, uint32_t length) {
    dtd_mouse_tx->next_dtd = 0x00000001;
    dtd_mouse_tx->token = (length << 16) | (1 << 15) | 0x80;
    dtd_mouse_tx->buffer[0] = (uint32_t)report;
    {
        uint32_t page_base = (uint32_t)report & ~0xFFFu;
        for (int p = 1; p < 5; p++) {
            dtd_mouse_tx->buffer[p] = page_base + ((uint32_t)p * 0x1000u);
        }
    }

    ep_list[5].td.next_dtd = (uint32_t)dtd_mouse_tx;
    ep_list[5].td.token &= ~0xC0;

    clean_dcache_range(dtd_mouse_tx, sizeof(*dtd_mouse_tx));
    clean_dcache_range(&ep_list[5].td, sizeof(ep_list[5].td));
    clean_dcache_range(report, length);
    drain_write_buffer();

    ENDPTPRIME |= (1 << 18);

    int ok;
    WAIT_CLEAR_TIMEOUT(ENDPTPRIME, (1 << 18), 5000000, ok);
    if (!ok) dbg_print("     send_mouse_report: prime TIMEOUT");

    int ok_complete;
    WAIT_SET_TIMEOUT(ENDPTCOMPLETE, (1 << 18), 5000000, ok_complete);
    if (!ok_complete) {
        dbg_print("     send_mouse_report: complete TIMEOUT");
    } else {
        ENDPTCOMPLETE = (1 << 18);
    }
}

void usb_device_shutdown() {
    int ok;
    USBCMD &= ~USBCMD_RS;

    ENDPTFLUSH = 0xFFFFFFFF;
    WAIT_CLEAR_TIMEOUT(ENDPTFLUSH, 0xFFFFFFFF, 2000000, ok);
    if (!ok) {
        dbg_print("     usb_device_shutdown: flush TIMEOUT");
    }

    ENDPTCTRL1 = 0;

    ENDPTCOMPLETE = 0xFFFFFFFF;
    USBSTS = USBSTS;

    OTGSC &= ~(1u << 5);
    busy_wait(200000);

    USBCMD |= USBCMD_RST;
    WAIT_CLEAR_TIMEOUT(USBCMD, USBCMD_RST, 2000000, ok);
    if (!ok) {
        dbg_print("     usb_device_shutdown: reset TIMEOUT");
    }

    USBINTR = 0;
    DEVICEADDR = 0;

    usb_device_configured = 0;
    usb_should_exit = 0;
}

// NOTE: a usb_soft_disconnect() function used to live here, re-gating
// the same PWR-management bits ensure_usb_bus_access_enabled() clears
// at startup, attempting to simulate a clean electrical unplug.
// Removed: it never proved effective at avoiding the host's
// "improperly ejected" warning, and it left the calculator's own
// native USB stack unable to recover after the program exited (the
// save/restore_native_usb_state() pair doesn't cover these separate
// power-gate bits). Confirmed actively harmful with no offsetting
// benefit.

void service_control_endpoint(void) {
if (ENDPTSETUPSTAT & 0x00000001) {

    USBSTS = 0x00000005;
    (void)ENDPTSETUPSTAT;
    (void)ENDPTCOMPLETE;
    flush_endpoint(0);

    uint8_t setup_local[8];
    read_setup_packet_safe(setup_local);
    print_setup_hex(setup_local);

    uint8_t  bmRequestType = setup_local[0];
    uint8_t  bRequest      = setup_local[1];
    uint16_t wValue        = setup_local[2] | (setup_local[3] << 8);
    uint16_t wIndex        = setup_local[4] | (setup_local[5] << 8);
    uint16_t wLength       = setup_local[6] | (setup_local[7] << 8);
    (void)bmRequestType;

    char strbuf[100];
    sprintf(strbuf, "     s.bRequest=0x%x", bRequest);
    dbg_print(strbuf);
    sprintf(strbuf, "     s.wValue=0x%x", wValue);
    dbg_print(strbuf);
    sprintf(strbuf, "     s.wLength=0x%x", wLength);
    dbg_print(strbuf);

    if (bRequest == 0x01) { // CLEAR_FEATURE
        if (wValue == 0x00) {
            uint8_t ep_addr = wIndex & 0xFF;
            uint8_t ep_num  = ep_addr & 0x0F;
            int is_in       = (ep_addr & 0x80) != 0;

            if (is_in && (ep_num == 1 || ep_num == 2)) {
                if (ep_num == 1) {
                    ENDPTCTRL1 &= ~ENDPTCTRL_TXS;
                    ENDPTCTRL1 |= ENDPTCTRL_TXR;
                    ep_list[3].td.token &= ~0xC0;
                    clean_dcache_range(&ep_list[3].td, sizeof(ep_list[3].td));
                } else {
                    ENDPTCTRL2 &= ~ENDPTCTRL_TXS;
                    ENDPTCTRL2 |= ENDPTCTRL_TXR;
                    ep_list[5].td.token &= ~0xC0;
                    clean_dcache_range(&ep_list[5].td, sizeof(ep_list[5].td));
                }
                dbg_print("     CLEAR_FEATURE(ENDPOINT_HALT) applied");
            }
        }
        ack_zero_length_status_phase();
    }
    else if (bRequest == 0x05) { // SET_ADDRESS
        uint32_t new_addr = (uint32_t)(wValue & 0x7F);
        DEVICEADDR = (new_addr << 25) | (1u << 24);

        ack_zero_length_status_phase();

        int ok_addr;
        WAIT_CLEAR_TIMEOUT(ENDPTPRIME, 0x00010000, 5000000, ok_addr);
        if (!ok_addr) {
            dbg_print("     SET_ADDRESS: prime TIMEOUT");
            print_ep0_in_state("     post-timeout:");
        }

        int ok_complete;
        WAIT_SET_TIMEOUT(ENDPTCOMPLETE, 0x00010000, 5000000, ok_complete);
        if (!ok_complete) {
            dbg_print("     SET_ADDRESS: complete TIMEOUT");
        } else {
            ENDPTCOMPLETE = 0x00010000;
        }
    }
    else if (bRequest == 0x06) { // GET_DESCRIPTOR
        uint8_t desc_type = (wValue >> 8) & 0xFF;
        if (desc_type == 0x01) {
            transmit_device_descriptor(wLength);
        } else if (desc_type == 0x02) {
            transmit_config_descriptor(wLength);
        } else if (g_usb_unhandled_control_callback) {
            g_usb_unhandled_control_callback(bRequest, wValue, wIndex, wLength);
        }
    }
    else if (bRequest == 0x09) { // SET_CONFIGURATION
        uint8_t config_val = wValue & 0xFF;

        if (config_val > 0) {
            ack_zero_length_status_phase();

            flush_endpoint(1);
            flush_endpoint(2);

            // TX (IN) only for both -- HID reports only flow
            // device-to-host, there's no OUT direction to arm.
            ENDPTCTRL1 = ENDPTCTRL_TXE | ENDPTCTRL_TXR | ENDPTCTRL_TXT_INTERRUPT;
            ENDPTCTRL2 = ENDPTCTRL_TXE | ENDPTCTRL_TXR | ENDPTCTRL_TXT_INTERRUPT;

            int ok_cfg;
            WAIT_CLEAR_TIMEOUT(ENDPTPRIME, 0x00010000, 5000000, ok_cfg);
            if (!ok_cfg) dbg_print("     SET_CONFIGURATION: prime TIMEOUT");

            int ok_cfg_complete;
            WAIT_SET_TIMEOUT(ENDPTCOMPLETE, 0x00010000, 5000000, ok_cfg_complete);
            if (!ok_cfg_complete) {
                dbg_print("     SET_CONFIGURATION: complete TIMEOUT");
            } else {
                ENDPTCOMPLETE = 0x00010000;
            }
        }
    }
    else if (g_usb_unhandled_control_callback) {
        // Covers class-specific requests this generic dispatcher
        // doesn't know about (e.g. Mass Storage's GET_MAX_LUN=0xFE and
        // Bulk-Only Mass Storage Reset=0xFF).
        g_usb_unhandled_control_callback(bRequest, wValue, wIndex, wLength);
    }
}
}

void setup_minimal_usb_stack() {
    USBCMD |= USBCMD_RS; // Run/Stop bit
}

typedef void (*usb_rx_callback_t)(const uint8_t *data, uint32_t length);
static usb_rx_callback_t g_usb_rx_callback = NULL;

static inline void set_usb_rx_callback(usb_rx_callback_t cb) {
    g_usb_rx_callback = cb;
}

static inline void set_usb_setup_callback(usb_setup_callback_t cb) {
    g_usb_setup_callback = cb;
}

static inline void set_usb_unhandled_control_callback(usb_unhandled_control_callback_t cb) {
    g_usb_unhandled_control_callback = cb;
}

bool is_usb_port_connected() {
    return (OTGSC & OTGSC_BSV);
}
