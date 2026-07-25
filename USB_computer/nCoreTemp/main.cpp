#include <os.h>
#include <keys.h>
#include <stdint.h>
#include <stdio.h>

#include <vector>
#include <string>

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>


// #define DEBUG_USB
#include "usb_dma.h"
#include "images.h"

typedef enum {
    NONE,
    ERROR,
    WARNING,
    CONNECTING,
    CONNECTED,
    DISCONNECTING
} ProgramState;
ProgramState program_current_state = NONE;
std::string update_message = "";

struct CPUStatsHeader {
    int32_t raw_cores_physics;
    int32_t raw_cores_logical;
    float raw_usage;
    float raw_temp;
} g_header;
std::vector<float> per_core_usage_pct;

void on_usb_data_received(const uint8_t *data, uint32_t length) {
    if (length < sizeof(CPUStatsHeader)) {
        dbg_print("Buffer too small for header!");
        return;
    }

    memcpy(&g_header, data, sizeof(CPUStatsHeader));

    size_t bytes_remaining = length - sizeof(CPUStatsHeader);
    size_t num_cores = bytes_remaining / sizeof(float);

    per_core_usage_pct = std::vector<float>(num_cores);
    if (num_cores > 0) {
        memcpy(per_core_usage_pct.data(), 
                    data + sizeof(CPUStatsHeader), 
                    bytes_remaining);
    }
}

void on_usb_setup_done() {
    const char* test_payload = "HELLO FROM THE BARE METAL CALCULATOR!";
    uint32_t payload_len = (uint32_t)strlen(test_payload);
    send_usb_message(test_payload, payload_len);

    program_current_state = CONNECTED;
    update_message = "Connected!";
}

SDL_Surface *m_screen_image;
nSDL_Font *black_font, *black_cover_font, *red_font, *yellow_font, *green_font, *blue_font, *orange_font;
SDL_Surface *logo;

bool keep_app_running = true;
void initSDL() {
    if (SDL_Init(SDL_INIT_VIDEO) == -1) {
        printf("Couldn't initialize SDL: %s\n", SDL_GetError());
        exit(EXIT_FAILURE);
    }
    m_screen_image = SDL_SetVideoMode(320, 240, has_colors ? 16 : 8, SDL_SWSURFACE);

    if (m_screen_image == NULL) {
        printf("Couldn't initialize display: %s\n", SDL_GetError());
        SDL_Quit();
        exit(EXIT_FAILURE);
    }
    SDL_ShowCursor(SDL_DISABLE);

    black_font = nSDL_LoadFont(NSDL_FONT_THIN, 0, 0, 0);
    red_font = nSDL_LoadFont(NSDL_FONT_VGA, 197, 49, 37);
    yellow_font = nSDL_LoadFont(NSDL_FONT_VGA, 238, 210, 2);
    green_font = nSDL_LoadFont(NSDL_FONT_VGA, 51, 204, 51);
    blue_font = nSDL_LoadFont(NSDL_FONT_VGA, 5, 186, 221);
    orange_font = nSDL_LoadFont(NSDL_FONT_VGA, 255, 128, 7);

    logo = nSDL_LoadImage(nCoreTempLogo);
    SDL_SetColorKey(logo, SDL_SRCCOLORKEY | SDL_RLEACCEL, SDL_MapRGB(logo->format, 255, 255, 255));

    printf("Initialized SDL and display.\n");
}

void quit_SDL() {
    SDL_FreeSurface(m_screen_image);

    nSDL_FreeFont(black_font);
    nSDL_FreeFont(red_font);
    nSDL_FreeFont(yellow_font);
    nSDL_FreeFont(green_font);
    nSDL_FreeFont(blue_font);
    nSDL_FreeFont(orange_font);

    SDL_Quit();
}

bool isCapsLockOn = false;
int blinkCounter = 0;

void draw_scene() {
    SDL_Rect border = {0, 0, 320, 240};
    SDL_FillRect(m_screen_image, &border, 0xffffff);

    SDL_Rect img_rect = {84, 2, (Uint16)logo->w, (Uint16)logo->h};
    SDL_BlitSurface(logo, NULL, m_screen_image, &img_rect);

    constexpr int update_y = 30;
    switch (program_current_state) {
        case ProgramState::ERROR:
            nSDL_DrawString(m_screen_image, red_font, 10, update_y, "Error: ");
            nSDL_DrawString(m_screen_image, black_font, 10 + nSDL_GetStringWidth(red_font, "Error: "), update_y, update_message.c_str());
            break;
        case ProgramState::WARNING:
            nSDL_DrawString(m_screen_image, yellow_font, 10, update_y, "Warning: ");
            nSDL_DrawString(m_screen_image, black_font, 10 + nSDL_GetStringWidth(yellow_font, "Warning: "), update_y, update_message.c_str());
            break;
        case ProgramState::CONNECTING:
            nSDL_DrawString(m_screen_image, green_font, 10, update_y, update_message.c_str());
            break;
        case ProgramState::CONNECTED:
            nSDL_DrawString(m_screen_image, green_font, 11, update_y, update_message.c_str());
            break;
        case ProgramState::DISCONNECTING:
            nSDL_DrawString(m_screen_image, red_font, 10, update_y, update_message.c_str());
            break;
        case NONE:
        default:
            break;
    }

    if (program_current_state != ProgramState::CONNECTING) {
        nSDL_DrawString(m_screen_image, black_font, 10, 40, "Physical Cores: %ld", g_header.raw_cores_physics);
        nSDL_DrawString(m_screen_image, black_font, 10, 50, "Logical Cores: %ld", g_header.raw_cores_logical);
        nSDL_DrawString(m_screen_image, black_font, 10, 60, "Total Usage: %.1f%%", g_header.raw_usage);
        
        rectangleRGBA(m_screen_image, 140, 62, 260, 68, 0, 0, 0, 255);
        boxRGBA(m_screen_image, 142, 64, 142 + 116 * (g_header.raw_usage / 100.0f), 66, 0, 0, 0, 255);

        nSDL_DrawString(m_screen_image, black_font, 10, 70, "Temperature: %.1f%cC", g_header.raw_temp, 248);

        float minTemp = 30.0f;
        float maxTemp = 80.0f;
        float normalized = (g_header.raw_temp - minTemp) / (maxTemp - minTemp);
        if (normalized < 0.0f) normalized = 0.0f;
        if (normalized > 1.0f) normalized = 1.0f;

        Uint8 r = static_cast<Uint8>(normalized * 255);
        Uint8 g = 0;
        Uint8 b = static_cast<Uint8>((1.0f - normalized) * 255);
        Uint8 a = 255;

        const constexpr int temp_circle_r = 4;
        const int temp_circle_x = 10 + nSDL_GetStringWidth(black_font, "Temperature: 100.0 dC");
        const constexpr int temp_circle_y = 74;
        filledCircleRGBA(m_screen_image, temp_circle_x, temp_circle_y, temp_circle_r, r, g, b, a);

        for (size_t core = 0; core < per_core_usage_pct.size(); core++) {
            nSDL_DrawString(m_screen_image, black_font, 30, 80 + core * 10, "Core %d Usage:", (int)(core+1));

            rectangleRGBA(m_screen_image, 140, 82 + core * 10, 260, 88 + core * 10, 0, 0, 0, 255);
            boxRGBA(m_screen_image, 142, 84 + core * 10, 142 + 116 * (per_core_usage_pct[core] / 100.0f), 86 + core * 10, 0, 0, 0, 255);

            nSDL_DrawString(m_screen_image, black_font, 270, 80 + core * 10, "%.1f%%", per_core_usage_pct[core]);
        }
    }

    SDL_Flip(m_screen_image);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch(event.type) {
            case SDL_KEYDOWN: {
                SDL_KeyboardEvent key = event.key;

                switch (key.keysym.sym) {
                    case SDLK_ESCAPE:
                        keep_app_running = false;
                        break;
                    default:
                        break;
                }
                break;
            }
            default:
                break;
        }
    }
}

int main(void) {
    initSDL();

    open_log();

    program_current_state = CONNECTING;

    ensure_usb_bus_access_enabled();

    init_aligned_pointers();
    save_native_usb_state();

    update_message = "Initializing USB Controller Hardware..."; draw_scene();
    reset_usb_subsystem();
    setup_endpoint_list();

    update_message = "Waiting for USB Host Cable Connection..."; draw_scene();
    while (!is_usb_port_connected()) {
        if (isKeyPressed(KEY_NSPIRE_ESC)) {
            usb_device_shutdown();
            restore_native_usb_state();
            close_log();
            quit_SDL();
            return 0;
        }
    }

    update_message = "Creating USB stack, waiting for server program..."; draw_scene();
    setup_minimal_usb_stack();

    if (usb_should_exit) {
        usb_device_shutdown();
        restore_native_usb_state();
        close_log();
        quit_SDL();
        return 0;
    }

    set_usb_rx_callback(on_usb_data_received);
    set_usb_setup_callback(on_usb_setup_done);

    while (keep_app_running) {
        draw_scene();
        service_control_endpoint();
        service_bulk_endpoints();

        if (!is_usb_port_connected()) {
            program_current_state = DISCONNECTING;
            update_message = "Detected Disconnection";
        }
    }

    usb_device_shutdown();
    restore_native_usb_state();
    close_log();
    quit_SDL();
    return 0;
}