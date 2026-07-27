#include "usb_msc.h"

int main(int argc, char *argv[]) {
    open_log();
    fatfs_log_reset();

    // Mark our own running file read-only in what we serve, so
    // macOS's Finder shows its native "locked" indicator and refuses
    // to move/rename it, rather than the user attempting a move that
    // can't actually succeed while the program is executing.
    if (argc >= 1 && argv[0]) fatfs_set_self_path(argv[0]);

    bool first_run = true;

    while (1) {
        ensure_usb_bus_access_enabled();

        if (first_run) {
            fatfs_init();
            first_run = false;
        }
        init_aligned_pointers();
        save_native_usb_state();

        reset_usb_subsystem();
        setup_endpoint_list();

        screen_console << "Waiting for USB host link..." << nio::endl;
        screen_console << "  (connect the USB cable now, or press Esc to quit)" << nio::endl;
        dbg_print("Waiting for USB host link...");
        while (!is_usb_port_connected()) {
            if (isKeyPressed(KEY_NSPIRE_ESC)) {
                fatfs_flush_pending_deletes();
                usb_device_shutdown();
                restore_native_usb_state();
                close_log();
                return 0;
            }
        }

        screen_console << "Connected! Serving: " << fatfs_root_path << nio::endl;
        screen_console << "  Esc  - eject and exit" << nio::endl;
        screen_console << "  Tab  - switch root between /documents and / (reconnects)" << nio::endl;
        dbg_print("Connected!");

        setup_minimal_usb_stack();

        if (usb_should_exit) {
            fatfs_flush_pending_deletes();
            usb_device_shutdown();
            restore_native_usb_state();
            close_log();
            return 0;
        }

        set_usb_unhandled_control_callback(msc_on_unhandled_control);
        set_usb_rx_callback(msc_on_cbw_received);

        bool want_root_switch = false;
        while (!isKeyPressed(KEY_NSPIRE_ESC)) {
            service_control_endpoint();
            service_bulk_endpoints();
            fatfs_bg_copy_step();
            main_loop_iters++;
            if (isKeyPressed(KEY_NSPIRE_TAB)) {
                want_root_switch = true;
                break;
            }
        }

        // Finish any move still copying in the background before
        // disconnecting -- doing so mid-copy would leave the
        // filesystem inconsistent (old directory not yet removed,
        // destination only partially written).
        while (fatfs_bg_copy_active) fatfs_bg_copy_step();

        if (want_root_switch) {
            // Immediate disconnect + reconnect, no signal/wait step of
            // any kind -- just the proven-safe shutdown sequence,
            // followed by looping back to re-enable and re-enumerate.
            // usb_soft_disconnect() (a separate PWR-gate power-down)
            // stays removed -- confirmed to break the calculator's own
            // native USB recovery afterward.
            screen_console << "Switching root, reconnecting..." << nio::endl;
            fatfs_flush_pending_deletes();
            usb_device_shutdown();
            restore_native_usb_state();
            busy_wait(200000); // brief settle time so the host clearly sees the disconnect
            fatfs_toggle_root();
            continue; // loop back around: re-enable, re-enumerate with the new root
        }

        // Esc pressed -- eject and exit
        screen_console << "Ejecting..." << nio::endl;
        fatfs_flush_pending_deletes();
        usb_device_shutdown();
        restore_native_usb_state();
        close_log();
        return 0;
    }
}