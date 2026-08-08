#include "hid_keymap.h"

// Held-key state from the previous scan, for the string-sequence keys
// only -- these need edge detection (fire once per press, not
// repeated every scan while held), unlike the simple/dual maps which
// just continuously reflect current physical state.
static uint8_t hid_string_key_was_down[HID_STRING_MAP_COUNT];

// Sends a short ASCII string as a sequence of individual keyboard
// reports (press, then release, one character at a time). Used for
// math/CAS keys that represent more than a single character. Blocks
// briefly per character -- acceptable here since this only fires once
// per physical key press, not continuously.
static void hid_type_string(const char *s) {
    static uint8_t report[8] __attribute__((aligned(32)));
    static uint8_t release[8] __attribute__((aligned(32))) = {0};
    while (*s) {
        uint8_t keycode, shift;
        if (ascii_to_hid(*s, &keycode, &shift)) {
            memset(report, 0, 8);
            report[0] = shift ? 0x02 : 0x00; // left shift modifier bit
            report[2] = keycode;
            send_keyboard_report(report, 8);
            memcpy(hid_last_keyboard_report, report, 8);

            send_keyboard_report(release, 8);
            memcpy(hid_last_keyboard_report, release, 8);
        }
        s++;
    }
}

// Scans every mapped key, builds the current keyboard report (up to 6
// simultaneously-held non-modifier keys, packed into a single 8-byte
// boot-protocol report), fires any string-sequence keys on their
// rising edge, and sends the report only if it actually changed.
static void hid_scan_keyboard(void) {
    uint8_t report[8] = {0};
    int slot = 2; // report[2..7] hold up to 6 simultaneous keycodes

    if (isKeyPressed(KEY_NSPIRE_SHIFT)) report[0] |= 0x02; // left shift
    if (isKeyPressed(KEY_NSPIRE_CTRL))  report[0] |= 0x01; // left ctrl

    for (unsigned i = 0; i < HID_SIMPLE_MAP_COUNT && slot < 8; i++) {
        if (!isKeyPressed(*hid_simple_map[i].key)) continue;
        report[slot++] = hid_simple_map[i].hid_code;
        if (hid_simple_map[i].needs_shift) report[0] |= 0x02;
    }

    for (unsigned i = 0; i < HID_DUAL_MAP_COUNT && slot < 8; i++) {
        if (!isKeyPressed(*hid_dual_map[i].key)) continue;
        if (slot < 8) report[slot++] = hid_dual_map[i].hid_code1;
        if (slot < 8) report[slot++] = hid_dual_map[i].hid_code2;
    }

    hid_send_keyboard_report_if_changed(report);

    // String-sequence keys: edge-triggered, independent of the
    // regular report above (they never occupy one of the 6 keycode
    // slots -- typing "sin(" doesn't hold a single key down).
    for (unsigned i = 0; i < HID_STRING_MAP_COUNT; i++) {
        int down = isKeyPressed(*hid_string_map[i].key) ? 1 : 0;
        if (down && !hid_string_key_was_down[i]) {
            hid_type_string(hid_string_map[i].text);
        }
        hid_string_key_was_down[i] = (uint8_t)down;
    }
}

// Converts the touchpad's absolute position into relative movement
// since the last scan, and sends a mouse report when anything
// changed. contact must be true on both this scan and the last for a
// delta to make sense -- lifting and re-placing a finger at a
// different spot shouldn't register as a sudden, large jump.
// Default true: HID's relative-Y convention is positive=down, and the
// unflipped touchpad reading produced the opposite of that (reported
// as "up/down is flipped"). Toggleable at runtime (hold Ctrl+Shift and
// tap the touchpad click) in case this diagnosis doesn't hold on some
// other host -- no rebuild needed to recover either way.
static int hid_mouse_flip_y = 1;

static void hid_scan_touchpad(void) {
    static int have_prev = 0;
    static uint16_t prev_x = 0, prev_y = 0;
    static int toggle_combo_was_down = 0;

    touchpad_report_t rep;
    if (touchpad_scan(&rep) != 0) return;

    uint8_t buttons = 0;
    if (rep.pressed) buttons |= 0x01;               // touchpad press-down = left click
    if (isKeyPressed(KEY_NSPIRE_CLICK)) buttons |= 0x01; // dedicated click button, same effect

    // Runtime toggle gesture: Ctrl+Shift+click. Edge-triggered so
    // holding the combo doesn't flip the setting repeatedly.
    int toggle_combo_down = isKeyPressed(KEY_NSPIRE_CTRL) && isKeyPressed(KEY_NSPIRE_SHIFT) && (buttons & 0x01);
    if (toggle_combo_down && !toggle_combo_was_down) {
        hid_mouse_flip_y = !hid_mouse_flip_y;
        screen_console << "Mouse Y-axis: " << (hid_mouse_flip_y ? "flipped" : "normal") << nio::endl;
    }
    toggle_combo_was_down = toggle_combo_down;

    int dx = 0, dy = 0;
    if (rep.contact && have_prev) {
        dx = (int)rep.x - (int)prev_x;
        dy = (int)rep.y - (int)prev_y;
        if (hid_mouse_flip_y) dy = -dy;
        // Scale down -- the touchpad's native coordinate resolution
        // is much finer than comfortable mouse movement needs. This
        // divisor is a reasonable starting point, easy to retune.
        dx /= 2;
        dy /= 2;
        if (dx > 127) dx = 127; else if (dx < -127) dx = -127;
        if (dy > 127) dy = 127; else if (dy < -127) dy = -127;
    }

    if (rep.contact) {
        prev_x = rep.x;
        prev_y = rep.y;
        have_prev = 1;
    } else {
        have_prev = 0;
    }

    uint8_t report[4];
    report[0] = buttons;
    report[1] = (uint8_t)(int8_t)dx;
    report[2] = (uint8_t)(int8_t)dy;
    report[3] = 0; // no wheel hardware

    hid_send_mouse_report_if_changed(report);
}

int main(void) {
    open_log();

    ensure_usb_bus_access_enabled();

    screen_console << "Nspire Keyboard+Mouse" << nio::endl;
    screen_console << "  Waiting for USB host link..." << nio::endl;
    screen_console << "  (connect the USB cable now, or press Esc to quit)" << nio::endl;

    while (!is_usb_port_connected()) {
        if (isKeyPressed(KEY_NSPIRE_ESC)) {
            close_log();
            return 0;
        }
    }

    init_aligned_pointers();
    save_native_usb_state();

    reset_usb_subsystem();
    setup_endpoint_list();

    setup_minimal_usb_stack();

    if (usb_should_exit) {
        usb_device_shutdown();
        restore_native_usb_state();
        close_log();
        return 0;
    }

    set_usb_unhandled_control_callback(hid_on_unhandled_control);

    screen_console << "Connected! Acting as keyboard + mouse." << nio::endl;
    screen_console << "  Esc  - eject and exit" << nio::endl;
    screen_console << "  Ctrl+Shift+click - toggle mouse Y-axis direction" << nio::endl;
    dbg_print("Connected!");

    while (!isKeyPressed(KEY_NSPIRE_ESC)) {
        service_control_endpoint();
        hid_scan_keyboard();
        hid_scan_touchpad();
    }

    screen_console << "Ejecting..." << nio::endl;
    usb_device_shutdown();
    restore_native_usb_state();
    close_log();
    return 0;
}