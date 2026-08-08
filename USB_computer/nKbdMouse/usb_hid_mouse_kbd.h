#include "usb_dma.h"

/* ==================================================================
 * String descriptors (UTF-16LE, same format/pattern as the MSC
 * project's own string descriptors)
 * ================================================================== */
static const uint8_t hid_string_desc_langid[4] = { 0x04, 0x03, 0x09, 0x04 }; // English (US)
static const uint8_t hid_string_desc_manufacturer[14] = {
    14, 0x03,
    'N',0,'S',0,'P',0,'I',0,'R',0,'E',0
};
static const uint8_t hid_string_desc_product[44] = {
    44, 0x03,
    'N',0,'s',0,'p',0,'i',0,'r',0,'e',0,' ',0,'K',0,'e',0,'y',0,'b',0,'o',0,'a',0,'r',0,'d',0,'+',0,'M',0,'o',0,'u',0,'s',0,'e',0
};
static const uint8_t hid_string_desc_serial[18] = {
    18, 0x03,
    '0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'5',0
};

static void hid_transmit_string_descriptor(uint8_t index, uint16_t requested_length) {
    const uint8_t *data;
    uint32_t len;
    switch (index) {
        case 0: data = hid_string_desc_langid;       len = sizeof(hid_string_desc_langid);       break;
        case 1: data = hid_string_desc_manufacturer; len = sizeof(hid_string_desc_manufacturer); break;
        case 2: data = hid_string_desc_product;      len = sizeof(hid_string_desc_product);      break;
        case 3: data = hid_string_desc_serial;       len = sizeof(hid_string_desc_serial);       break;
        default: return; // unknown index -- shouldn't be requested given our tables
    }
    transmit_ep0_data(data, len, requested_length);
}

/* ==================================================================
 * HID Report Descriptors -- one per interface. These define the exact
 * byte layout of the reports send_keyboard_report()/send_mouse_report()
 * transmit. Both follow the standard USB HID "boot protocol" layout
 * for maximum host compatibility (BIOS/bootloader keyboard support,
 * older or minimal HID stacks, etc.), even though we're always
 * running under a full OS that understands the report protocol too.
 * ================================================================== */

// Keyboard report: byte0 = modifier bitmap (ctrl/shift/alt/gui, left+right),
// byte1 = reserved (always 0), bytes2-7 = up to 6 simultaneously-held
// keycodes (0 = no key in that slot).
static const uint8_t hid_report_desc_keyboard[63] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (224)
    0x29, 0xE7,        //   Usage Maximum (231)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs) -- modifier byte
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Const) -- reserved byte
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x05,        //   Usage Maximum (5)
    0x91, 0x02,        //   Output (Data,Var,Abs) -- LED report (unused, but required by boot protocol)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x01,        //   Output (Const) -- LED padding
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x81, 0x00,        //   Input (Data,Array) -- 6-byte keycode array
    0xC0               // End Collection
};

// Mouse report: byte0 = button bitmap (bits 0-2 = left/right/middle),
// byte1 = dX (signed, relative), byte2 = dY (signed, relative),
// byte3 = wheel (signed, relative -- always 0, no wheel hardware, but
// included for broad host compatibility).
static const uint8_t hid_report_desc_mouse[62] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Buttons)
    0x19, 0x01,        //     Usage Minimum (1)
    0x29, 0x03,        //     Usage Maximum (3)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs) -- 3 button bits
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x01,        //     Input (Const) -- 5 bit padding
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data,Var,Rel) -- X, Y relative movement
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel) -- wheel movement (always 0)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

// Last report sent on each interface -- kept so GET_REPORT (some hosts
// query this once at connect time, before the first interrupt report
// has gone out, to establish initial state) always has something
// valid to return instead of garbage.
static uint8_t hid_last_keyboard_report[8] __attribute__((aligned(32))) = {0,0,0,0,0,0,0,0};
static uint8_t hid_last_mouse_report[4] __attribute__((aligned(32)))    = {0,0,0,0};

/* ---- Unhandled-control callback: registered via
 * set_usb_unhandled_control_callback() ----
 * Called by usb_dma.h's service_control_endpoint() for any control
 * request it doesn't natively recognize: string descriptors, the two
 * HID-class descriptor types (HID descriptor itself and the Report
 * descriptor, both requested via GET_DESCRIPTOR with a class-specific
 * high byte), and HID's standard class-specific requests
 * (GET_REPORT, SET_IDLE, SET_PROTOCOL, GET_PROTOCOL). */
void hid_on_unhandled_control(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
    if (bRequest == 0x06) { // GET_DESCRIPTOR
        uint8_t desc_type = (wValue >> 8) & 0xFF;
        uint8_t desc_index = wValue & 0xFF;
        if (desc_type == 0x03) {
            hid_transmit_string_descriptor(desc_index, wLength);
            return;
        }
        if (desc_type == 0x22) { // HID Report Descriptor
            if (wIndex == 0) {
                transmit_ep0_data(hid_report_desc_keyboard, sizeof(hid_report_desc_keyboard), wLength);
            } else if (wIndex == 1) {
                transmit_ep0_data(hid_report_desc_mouse, sizeof(hid_report_desc_mouse), wLength);
            }
            return;
        }
        if (desc_type == 0x21) { // HID Descriptor itself -- pull straight from the config descriptor, same bytes
            if (wIndex == 0) {
                transmit_ep0_data(&config_descriptor[18], 9, wLength);
            } else if (wIndex == 1) {
                transmit_ep0_data(&config_descriptor[43], 9, wLength);
            }
            return;
        }
        return;
    }

    // HID class-specific requests (bmRequestType class+interface,
    // stripped by service_control_endpoint() before we see it -- we
    // only get bRequest/wValue/wIndex/wLength here).
    if (bRequest == 0x0A) { // SET_IDLE -- acknowledge, no data phase. We don't rate-limit reports (nothing to idle).
        ack_zero_length_status_phase();
        return;
    }
    if (bRequest == 0x0B) { // SET_PROTOCOL -- acknowledge. Report descriptors are boot-compatible either way, so both protocol values behave identically for us.
        ack_zero_length_status_phase();
        return;
    }
    if (bRequest == 0x03) { // GET_PROTOCOL -- report protocol (1), always.
        static const uint8_t protocol = 1;
        transmit_ep0_data(&protocol, 1, wLength);
        return;
    }
    if (bRequest == 0x01) { // GET_REPORT -- return the last report sent, so a host that asks before we've sent anything still gets valid (all-zero/no-input) data.
        if (wIndex == 0) {
            transmit_ep0_data(hid_last_keyboard_report, sizeof(hid_last_keyboard_report), wLength);
        } else if (wIndex == 1) {
            transmit_ep0_data(hid_last_mouse_report, sizeof(hid_last_mouse_report), wLength);
        }
        return;
    }
}

/* ==================================================================
 * Report senders -- build on usb_dma.h's send_keyboard_report()/
 * send_mouse_report(), tracking the last report sent (for GET_REPORT)
 * and only actually transmitting when something changed, since
 * sending an identical report on every single main-loop iteration
 * would be wasteful and could itself introduce timing issues on the
 * host side we haven't tested for.
 * ================================================================== */
static uint8_t hid_kbd_report_buf[8] __attribute__((aligned(32)));
static uint8_t hid_mouse_report_buf[4] __attribute__((aligned(32)));

static void hid_send_keyboard_report_if_changed(const uint8_t *report8) {
    if (memcmp(report8, hid_last_keyboard_report, 8) == 0) return;
    memcpy(hid_kbd_report_buf, report8, 8);
    send_keyboard_report(hid_kbd_report_buf, 8);
    memcpy(hid_last_keyboard_report, report8, 8);
}

static void hid_send_mouse_report_if_changed(const uint8_t *report4) {
    // Mouse reports are relative (dX/dY), so unlike the keyboard, an
    // "unchanged" report isn't meaningful the same way -- a nonzero
    // dX/dY should always be sent even if identical to the last one
    // (continuous movement in the same direction), but an all-zero
    // report (no buttons, no movement) is safe to suppress once
    // already sent, avoiding a flood of no-op reports while idle.
    static int last_was_idle = 1;
    int is_idle = (report4[0] == 0 && report4[1] == 0 && report4[2] == 0 && report4[3] == 0);
    if (is_idle && last_was_idle) return;
    memcpy(hid_mouse_report_buf, report4, 4);
    send_mouse_report(hid_mouse_report_buf, 4);
    memcpy(hid_last_mouse_report, report4, 4);
    last_was_idle = is_idle;
}
