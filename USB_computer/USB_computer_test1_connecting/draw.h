#include <os.h>
#include <ngc.h>
extern "C" {
    void ascii2utf16(void *buf, const char *str, int max_size);
}

static Gc gc;
static int line_y = 10; // current draw cursor, in pixels

void gc_init() {
    gc = gui_gc_global_GC();
    gui_gc_setRegion(gc, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    gui_gc_begin(gc);

    gui_gc_setColor(gc, 0xFFFFFF);
    gui_gc_fillRect(gc, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT); // white background
    gui_gc_setColor(gc, 0x000000);                          // black text
    gui_gc_blit_to_screen(gc);
}

// Draws one line of text and immediately blits just that line's region
void gc_print_line(const char* text) {
    char utf16buf[256];
    ascii2utf16(utf16buf, text, sizeof(utf16buf));

    gui_gc_drawString(gc, utf16buf, 4, line_y, GC_SM_TOP);
    gui_gc_blit_to_screen_region(gc, 0, line_y, SCREEN_WIDTH, 14);

    line_y += 14;
    if (line_y > SCREEN_HEIGHT - 14) {
        // simple reset instead of real scrolling
        line_y = 10;
        gui_gc_setColor(gc, 0xFFFFFF);
        gui_gc_fillRect(gc, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
        gui_gc_setColor(gc, 0x000000);
        gui_gc_blit_to_screen(gc);
    }
}

void gc_cleanup() {
    gui_gc_finish(gc);
}