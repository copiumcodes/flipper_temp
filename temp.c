#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <storage/storage.h>
#include <math.h>

/* --- 1. STATE MANAGEMENT --- */
// We store our temperature in a struct protected by a mutex
// so the UI doesn't try to read it while the sensor is updating it.
typedef struct {
    float temperature;
    FuriMutex* mutex;
    bool logging_enabled;
    bool last_log_ok;
    uint32_t samples_logged;
    uint8_t page;
} TempMonitorApp;

typedef enum {
    AppPageMain = 0,
    AppPagePinout = 1,
} AppPage;

#define TEMP_LOG_PATH "/ext/temp_log.csv"


/* --- 2. HARDWARE LOGIC --- */
// DS18B20 1-Wire sensor reading using bit-banging on GPIO
// Connected to pin A7 (gpio_ext_pa7)

#define DS18B20_PIN &gpio_ext_pa7

// 1-Wire timing constants (in microseconds)
#define OW_RESET_LOW_TIME 480
#define OW_RESET_SAMPLE_TIME 70
#define OW_RESET_RECOVERY_TIME 410

#define OW_WRITE_1_LOW_TIME 6
#define OW_WRITE_1_RECOVERY_TIME 64
#define OW_WRITE_0_LOW_TIME 60
#define OW_WRITE_0_RECOVERY_TIME 10

#define OW_READ_INIT_LOW_TIME 3
#define OW_READ_SAMPLE_TIME 10
#define OW_READ_RECOVERY_TIME 53

static void ow_write_bit(const GpioPin* pin, bool bit) {
    if(bit) {
        // Write '1': pull low briefly, then release for rest of slot
        furi_hal_gpio_write(pin, false);
        furi_delay_us(OW_WRITE_1_LOW_TIME);
        furi_hal_gpio_write(pin, true);
        furi_delay_us(OW_WRITE_1_RECOVERY_TIME);
    } else {
        // Write '0': hold low for most of the slot
        furi_hal_gpio_write(pin, false);
        furi_delay_us(OW_WRITE_0_LOW_TIME);
        furi_hal_gpio_write(pin, true);
        furi_delay_us(OW_WRITE_0_RECOVERY_TIME);
    }
}

static bool ow_read_bit(const GpioPin* pin) {
    bool bit;
    furi_hal_gpio_write(pin, false);
    furi_delay_us(OW_READ_INIT_LOW_TIME);
    furi_hal_gpio_write(pin, true);
    furi_delay_us(OW_READ_SAMPLE_TIME);
    bit = furi_hal_gpio_read(pin);
    furi_delay_us(OW_READ_RECOVERY_TIME);
    return bit;
}

static bool ow_reset(const GpioPin* pin) {
    // Pull low for reset pulse, release, sample for presence pulse.
    furi_hal_gpio_write(pin, false);
    furi_delay_us(OW_RESET_LOW_TIME);
    furi_hal_gpio_write(pin, true);
    furi_delay_us(OW_RESET_SAMPLE_TIME);
    
    // Check for presence pulse (slave should pull line low)
    bool presence = !furi_hal_gpio_read(pin);
    furi_delay_us(OW_RESET_RECOVERY_TIME);

    // After the presence window, line must return high.
    if(!furi_hal_gpio_read(pin)) {
        return false;
    }

    return presence;
}

static void ow_write_byte(const GpioPin* pin, uint8_t byte) {
    for(int i = 0; i < 8; i++) {
        ow_write_bit(pin, (byte >> i) & 1);
    }
}

static uint8_t ow_read_byte(const GpioPin* pin) {
    uint8_t byte = 0;
    for(int i = 0; i < 8; i++) {
        if(ow_read_bit(pin)) {
            byte |= (1 << i);
        }
    }
    return byte;
}

static uint8_t ds18b20_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for(size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for(uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if(mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

static bool append_temperature_log(float temp) {
    bool ok = false;
    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);

    if(storage_file_open(
           file,
           TEMP_LOG_PATH,
           FSAM_WRITE,
           (FS_OpenMode)(FSOM_OPEN_APPEND | FSOM_OPEN_ALWAYS))) {
        uint64_t file_pos = storage_file_tell(file);
        if(file_pos == 0) {
            const char* header = "uptime_s,temperature_c\n";
            size_t header_len = strlen(header);
            ok = (storage_file_write(file, header, header_len) == header_len);
        } else {
            ok = true;
        }

        if(ok) {
            char line[64];
            uint32_t uptime_s = furi_get_tick() / 1000;
            int line_len = snprintf(line, sizeof(line), "%lu,%.2f\n", (unsigned long)uptime_s, (double)temp);
            if(line_len > 0) {
                ok = (storage_file_write(file, line, (size_t)line_len) == (size_t)line_len);
            } else {
                ok = false;
            }
        }
    }

    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
    return ok;
}

static float read_ds18b20() {
    const GpioPin* pin = DS18B20_PIN;

    // If the bus is not high while idle, wiring is likely wrong (stuck-low line).
    if(!furi_hal_gpio_read(pin)) {
        return -999.0f;
    }
    
    // Reset the bus
    if(!ow_reset(pin)) {
        return -999.0f; // No sensor detected
    }
    
    // Skip ROM (0xCC) - address all devices on bus
    ow_write_byte(pin, 0xCC);
    
    // Convert temperature (0x44)
    ow_write_byte(pin, 0x44);
    
    // Wait for conversion (750ms for 12-bit)
    furi_delay_ms(800);
    
    // Reset again for reading
    if(!ow_reset(pin)) {
        return -999.0f;
    }
    
    // Skip ROM (0xCC)
    ow_write_byte(pin, 0xCC);
    
    // Read Scratchpad (0xBE)
    ow_write_byte(pin, 0xBE);
    
    // Read full scratchpad (9 bytes) and validate CRC.
    uint8_t scratchpad[9];
    for(size_t i = 0; i < 9; i++) {
        scratchpad[i] = ow_read_byte(pin);
    }

    if(ds18b20_crc8(scratchpad, 8) != scratchpad[8]) {
        return -999.0f;
    }

    uint8_t lsb = scratchpad[0];
    uint8_t msb = scratchpad[1];
    
    // Combine into 16-bit value
    int16_t raw = (msb << 8) | lsb;
    
    // Convert to Celsius (divide by 16 as per DS18B20 datasheet)
    float temp = (float)raw / 16.0f;

    // Physical limits of DS18B20, reject impossible values.
    if(temp < -55.0f || temp > 125.0f) {
        return -999.0f;
    }

    return temp;
}

static void draw_big_digit(Canvas* canvas, int32_t x, int32_t y, char ch) {
    const int32_t w = 22;
    const int32_t h = 40;
    const int32_t t = 4;

    if(ch == '-') {
        canvas_draw_box(canvas, x + t, y + (h / 2) - (t / 2), w - (2 * t), t);
        return;
    }

    if(ch == '.') {
        canvas_draw_box(canvas, x + 2, y + h - 6, 4, 4);
        return;
    }

    uint8_t seg = 0;
    switch(ch) {
    case '0':
        seg = 0x3F;
        break;
    case '1':
        seg = 0x06;
        break;
    case '2':
        seg = 0x5B;
        break;
    case '3':
        seg = 0x4F;
        break;
    case '4':
        seg = 0x66;
        break;
    case '5':
        seg = 0x6D;
        break;
    case '6':
        seg = 0x7D;
        break;
    case '7':
        seg = 0x07;
        break;
    case '8':
        seg = 0x7F;
        break;
    case '9':
        seg = 0x6F;
        break;
    default:
        return;
    }

    if(seg & 0x01) canvas_draw_box(canvas, x + t, y, w - (2 * t), t); // A
    if(seg & 0x02) canvas_draw_box(canvas, x + w - t, y + t, t, (h / 2) - t); // B
    if(seg & 0x04) canvas_draw_box(canvas, x + w - t, y + (h / 2), t, (h / 2) - t); // C
    if(seg & 0x08) canvas_draw_box(canvas, x + t, y + h - t, w - (2 * t), t); // D
    if(seg & 0x10) canvas_draw_box(canvas, x, y + (h / 2), t, (h / 2) - t); // E
    if(seg & 0x20) canvas_draw_box(canvas, x, y + t, t, (h / 2) - t); // F
    if(seg & 0x40) canvas_draw_box(canvas, x + t, y + (h / 2) - (t / 2), w - (2 * t), t); // G
}

static int32_t big_char_width(char ch) {
    if(ch == '.') return 8;
    return 22;
}

static void draw_main_temp_big(Canvas* canvas, float temp) {
    if(temp <= -100.0f) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 34, AlignCenter, AlignCenter, "No Sensor");
        return;
    }

    char temp_buf[16];
    snprintf(temp_buf, sizeof(temp_buf), "%.1f", (double)temp);

    const int32_t gap = 4;
    size_t len = strlen(temp_buf);
    int32_t total_w = 0;
    for(size_t i = 0; i < len; i++) {
        total_w += big_char_width(temp_buf[i]);
        if(i + 1 < len) total_w += gap;
    }
    int32_t start_x = (128 - total_w) / 2;
    int32_t y = 12;

    int32_t x = start_x;
    for(size_t i = 0; i < len; i++) {
        draw_big_digit(canvas, x, y, temp_buf[i]);
        x += big_char_width(temp_buf[i]) + gap;
    }
}

static void draw_main_page(Canvas* canvas, float temp) {
    draw_main_temp_big(canvas, temp);
}

static void draw_pinout_page(Canvas* canvas, bool logging_enabled, bool last_log_ok, uint32_t samples_logged) {
    canvas_draw_frame(canvas, 0, 0, 128, 64);
    canvas_draw_line(canvas, 0, 12, 127, 12);

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str_aligned(canvas, 64, 10, AlignCenter, AlignBottom, "Pinout + Logging");

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 22, "DS18B20 pins:");
    canvas_draw_str(canvas, 2, 31, "1:GND  2:DQ  3:3V3");
    canvas_draw_str(canvas, 2, 40, "DQ->A7, 4k7 to 3V3");

    char status[32];
    snprintf(
        status,
        sizeof(status),
        "LOG:%s %s %lu",
        logging_enabled ? "ON" : "OFF",
        last_log_ok ? "OK" : "ERR",
        (unsigned long)samples_logged);
    canvas_draw_str(canvas, 2, 49, status);
    canvas_draw_str(canvas, 2, 62, "OK log  R save  L back");
}


/* --- 3. GUI DRAW CALLBACK --- */
// This function is called every time viewport_update() is triggered
static void draw_callback(Canvas* canvas, void* ctx) {
    TempMonitorApp* app = ctx;
    
    // Safely grab the temperature from our state
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    float temp = app->temperature;
    bool logging_enabled = app->logging_enabled;
    bool last_log_ok = app->last_log_ok;
    uint32_t samples_logged = app->samples_logged;
    uint8_t page = app->page;
    furi_mutex_release(app->mutex);

    // Draw to the screen
    canvas_clear(canvas);

    if(page == AppPageMain) {
        draw_main_page(canvas, temp);
    } else {
        draw_pinout_page(canvas, logging_enabled, last_log_ok, samples_logged);
    }
}


/* --- 4. INPUT CALLBACK --- */
// Pushes button presses into a message queue to be handled by the main loop
static void input_callback(InputEvent* input_event, void* ctx) {
    FuriMessageQueue* event_queue = ctx;
    furi_message_queue_put(event_queue, input_event, FuriWaitForever);
}


/* --- 5. MAIN APP ENTRY --- */
int32_t my_temp_app_main(void* p) {
    UNUSED(p);

    // Configure 1-Wire data pin as open-drain output with pull-up.
    furi_hal_gpio_init(DS18B20_PIN, GpioModeOutputOpenDrain, GpioPullUp, GpioSpeedLow);
    furi_hal_gpio_write(DS18B20_PIN, true);
    
    // Allocate memory for our state
    TempMonitorApp* app = malloc(sizeof(TempMonitorApp));
    app->temperature = -999.0f;
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->logging_enabled = false;
    app->last_log_ok = true;
    app->samples_logged = 0;
    app->page = AppPageMain;
    
    // Setup queue and ViewPort
    FuriMessageQueue* event_queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    ViewPort* viewport = view_port_alloc();
    view_port_draw_callback_set(viewport, draw_callback, app);
    view_port_input_callback_set(viewport, input_callback, event_queue);
    
    // Register the ViewPort to the GUI system
    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, viewport, GuiLayerFullscreen);
    
    InputEvent event;
    bool running = true;
    float last_valid_temp = -999.0f;
    
    // The Event Loop
    while(running) {
        // Wait for a button press for up to 1 second (1000 ticks)
        // If the queue times out, it means no buttons were pressed, so we read the sensor.
        if(furi_message_queue_get(event_queue, &event, 1000) == FuriStatusOk) {
            if(event.type == InputTypeShort && event.key == InputKeyBack) {
                running = false; // Exit the app on the 'Back' button
            } else if(event.type == InputTypeShort) {
                furi_mutex_acquire(app->mutex, FuriWaitForever);
                uint8_t page = app->page;
                float current_temp = app->temperature;
                furi_mutex_release(app->mutex);

                if(page == AppPageMain) {
                    if(event.key == InputKeyRight) {
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        app->page = AppPagePinout;
                        furi_mutex_release(app->mutex);
                        view_port_update(viewport);
                    }
                } else {
                    if(event.key == InputKeyLeft) {
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        app->page = AppPageMain;
                        furi_mutex_release(app->mutex);
                        view_port_update(viewport);
                    } else if(event.key == InputKeyOk) {
                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        app->logging_enabled = !app->logging_enabled;
                        furi_mutex_release(app->mutex);
                        view_port_update(viewport);
                    } else if(event.key == InputKeyRight) {
                        bool ok = append_temperature_log(current_temp);

                        furi_mutex_acquire(app->mutex, FuriWaitForever);
                        app->last_log_ok = ok;
                        if(ok) app->samples_logged++;
                        furi_mutex_release(app->mutex);
                        view_port_update(viewport);
                    }
                }
            }
        } else {
            // Read the sensor
            float new_temp = read_ds18b20();

            // Ignore one-off corrupt samples if we already have a valid reading.
            if(new_temp <= -100.0f && last_valid_temp > -100.0f) {
                new_temp = last_valid_temp;
            } else if(new_temp > -100.0f) {
                // Additional spike filter for brief protocol glitches.
                if(last_valid_temp > -100.0f && fabsf(new_temp - last_valid_temp) > 8.0f) {
                    new_temp = last_valid_temp;
                } else {
                    last_valid_temp = new_temp;
                }
            }
            
            // Update state safely
            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->temperature = new_temp;
            bool do_log = app->logging_enabled;
            furi_mutex_release(app->mutex);

            if(do_log) {
                bool ok = append_temperature_log(new_temp);
                furi_mutex_acquire(app->mutex, FuriWaitForever);
                app->last_log_ok = ok;
                if(ok) app->samples_logged++;
                furi_mutex_release(app->mutex);
            }
            
            // Tell the GUI it needs to redraw with the new numbers
            view_port_update(viewport);
        }
    }
    
    // Cleanup memory before exiting
    gui_remove_view_port(gui, viewport);
    view_port_free(viewport);
    furi_message_queue_free(event_queue);
    furi_mutex_free(app->mutex);
    free(app);
    furi_record_close(RECORD_GUI);
    
    return 0;
}