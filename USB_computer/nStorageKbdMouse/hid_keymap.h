#pragma once
#include <keys.h>
#include <libndls.h>
#include "usb_hid_mouse_kbd.h"

/* ==================================================================
 * ASCII -> HID keycode conversion, used both for the small set of
 * ASCII-representable keys below and for typing out multi-character
 * sequences (math/CAS keys that don't have a single-key PC
 * equivalent, e.g. "sin(" for the Sin key).
 * ================================================================== */
static int ascii_to_hid(char c, uint8_t *keycode, uint8_t *shift) {
    *shift = 0;
    if (c >= 'a' && c <= 'z') { *keycode = (uint8_t)(0x04 + (c - 'a')); return 1; }
    if (c >= 'A' && c <= 'Z') { *keycode = (uint8_t)(0x04 + (c - 'A')); *shift = 1; return 1; }
    if (c >= '1' && c <= '9') { *keycode = (uint8_t)(0x1E + (c - '1')); return 1; }
    if (c == '0') { *keycode = 0x27; return 1; }
    switch (c) {
        case ' ': *keycode = 0x2C; return 1;
        case '(': *keycode = 0x26; *shift = 1; return 1; // shift+9
        case ')': *keycode = 0x27; *shift = 1; return 1; // shift+0
        case '^': *keycode = 0x23; *shift = 1; return 1; // shift+6
        case '-': *keycode = 0x2D; return 1;
        case '=': *keycode = 0x2E; return 1;
        case '/': *keycode = 0x38; return 1;
        case '.': *keycode = 0x37; return 1;
        case ',': *keycode = 0x36; return 1;
        default: return 0; // unsupported character -- silently skipped
    }
}

/* ==================================================================
 * Simple, single-key, continuously-held mappings: one Nspire key ->
 * one HID keycode (optionally with the Shift modifier), held for
 * exactly as long as the physical key is held. Covers letters,
 * digits, and every key with a direct, unambiguous PC equivalent.
 * ================================================================== */
typedef struct {
    const t_key *key;
    uint8_t hid_code;
    uint8_t needs_shift;
} HidSimpleMapEntry;

#define K(name) (&KEY_NSPIRE_##name)

static const HidSimpleMapEntry hid_simple_map[] = {
    // Letters
    {K(A),0x04,0},{K(B),0x05,0},{K(C),0x06,0},{K(D),0x07,0},{K(E),0x08,0},
    {K(F),0x09,0},{K(G),0x0A,0},{K(H),0x0B,0},{K(I),0x0C,0},{K(J),0x0D,0},
    {K(K),0x0E,0},{K(L),0x0F,0},{K(M),0x10,0},{K(N),0x11,0},{K(O),0x12,0},
    {K(P),0x13,0},{K(Q),0x14,0},{K(R),0x15,0},{K(S),0x16,0},{K(T),0x17,0},
    {K(U),0x18,0},{K(V),0x19,0},{K(W),0x1A,0},{K(X),0x1B,0},{K(Y),0x1C,0},
    {K(Z),0x1D,0},
    // Digits
    {K(1),0x1E,0},{K(2),0x1F,0},{K(3),0x20,0},{K(4),0x21,0},{K(5),0x22,0},
    {K(6),0x23,0},{K(7),0x24,0},{K(8),0x25,0},{K(9),0x26,0},{K(0),0x27,0},
    // Basic editing/navigation
    {K(RET),0x28,0},{K(ENTER),0x28,0},{K(ESC),0x29,0},{K(DEL),0x2A,0},
    {K(TAB),0x2B,0},{K(SPACE),0x2C,0},{K(HOME),0x4A,0},{K(MENU),0x65,0},
    {K(UP),0x52,0},{K(DOWN),0x51,0},{K(LEFT),0x50,0},{K(RIGHT),0x4F,0},
    // Punctuation with a direct, unshifted equivalent
    {K(MINUS),0x2D,0},{K(NEGATIVE),0x2D,0},{K(EQU),0x2E,0},
    {K(APOSTROPHE),0x34,0},{K(COMMA),0x36,0},{K(PERIOD),0x37,0},
    // Punctuation requiring Shift on a standard US layout
    {K(QUOTE),0x34,1},        // Shift+' = "
    {K(COLON),0x33,1},        // Shift+; = :
    {K(LTHAN),0x36,1},        // Shift+, = <
    {K(GTHAN),0x37,1},        // Shift+. = >
    {K(LP),0x26,1},           // Shift+9 = (
    {K(RP),0x27,1},           // Shift+0 = )
    {K(BAR),0x31,1},          // Shift+\ = |
    {K(QUES),0x38,1},         // Shift+/ = ?
    {K(QUESEXCL),0x1E,1},     // Shift+1 = !
    // Operators -- keypad equivalents, so they don't need Shift at all
    {K(PLUS),0x57,0}, {K(MULTIPLY),0x55,0}, {K(DIVIDE),0x54,0},
    // No direct PC equivalent -- arbitrary but memorable F-key slots
    {K(DOC),0x3A,0}, {K(VAR),0x3B,0}, {K(TRIG),0x3C,0},
    {K(CAT),0x3D,0}, {K(SCRATCHPAD),0x3E,0}, {K(FLAG),0x45,0},
};
#define HID_SIMPLE_MAP_COUNT (sizeof(hid_simple_map) / sizeof(hid_simple_map[0]))

/* ==================================================================
 * Dual-key mappings: the touchpad's diagonal "arrow" keys don't exist
 * on a PC keyboard, but sending both adjacent arrows simultaneously
 * is the natural, intuitive equivalent of what the diagonal
 * represents.
 * ================================================================== */
typedef struct {
    const t_key *key;
    uint8_t hid_code1;
    uint8_t hid_code2;
} HidDualMapEntry;

static const HidDualMapEntry hid_dual_map[] = {
    {K(UPRIGHT),   0x52, 0x4F}, // Up + Right
    {K(RIGHTDOWN), 0x4F, 0x51}, // Right + Down
    {K(DOWNLEFT),  0x51, 0x50}, // Down + Left
    {K(LEFTUP),    0x50, 0x52}, // Left + Up
};
#define HID_DUAL_MAP_COUNT (sizeof(hid_dual_map) / sizeof(hid_dual_map[0]))

/* ==================================================================
 * String-sequence mappings: math/CAS keys that represent more than a
 * single character don't have a one-key PC equivalent, so instead
 * they type out their most natural text representation, once per
 * press (not repeated every scan while held -- see the edge-detection
 * logic in the main loop).
 * ================================================================== */
typedef struct {
    const t_key *key;
    const char *text;
} HidStringMapEntry;

static const HidStringMapEntry hid_string_map[] = {
    {K(SIN),   "sin("},
    {K(COS),   "cos("},
    {K(TAN),   "tan("},
    {K(EXP),   "^"},
    {K(eEXP),  "e^("},
    {K(TENX),  "10^("},
    {K(SQU),   "^2"},
    {K(PI),    "pi"},
    {K(II),    "i"},
    {K(FRAC),  "/"},
    {K(THETA), "theta"},
    {K(EE),    "E"},
};
#define HID_STRING_MAP_COUNT (sizeof(hid_string_map) / sizeof(hid_string_map[0]))

#undef K
