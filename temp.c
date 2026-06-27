#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>

/* --- 1. STATE MANAGEMENT --- */
// We store our temperature in a struct protected by a mutex
// so the UI doesn't try to read it while the sensor is updating it.
typedef struct {
    float temperature;
    FuriMutex* mutex;
} TempMonitorApp;


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
    
    // Read first two bytes (LSB, MSB)
    uint8_t lsb = ow_read_byte(pin);
    uint8_t msb = ow_read_byte(pin);
    
    // Combine into 16-bit value
    int16_t raw = (msb << 8) | lsb;
    
    // Convert to Celsius (divide by 16 as per DS18B20 datasheet)
    return (float)raw / 16.0f;
}


/* --- 3. GUI DRAW CALLBACK --- */
// This function is called every time viewport_update() is triggered
static void draw_callback(Canvas* canvas, void* ctx) {
    TempMonitorApp* app = ctx;
    
    // Safely grab the temperature from our state
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    float temp = app->temperature;
    furi_mutex_release(app->mutex);

    // Draw to the screen
    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 10, 20, "DS18B20 Temp:");
    
    char temp_str[32];
    if (temp <= -100.0f) {
        snprintf(temp_str, sizeof(temp_str), "No Sensor");
    } else {
        snprintf(temp_str, sizeof(temp_str), "%.1f C", (double)temp);
    }
    
    canvas_set_font(canvas, FontBigNumbers);
    canvas_draw_str(canvas, 10, 45, temp_str);
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
    
    // The Event Loop
    while(running) {
        // Wait for a button press for up to 1 second (1000 ticks)
        // If the queue times out, it means no buttons were pressed, so we read the sensor.
        if(furi_message_queue_get(event_queue, &event, 1000) == FuriStatusOk) {
            if(event.type == InputTypeShort && event.key == InputKeyBack) {
                running = false; // Exit the app on the 'Back' button
            }
        } else {
            // Read the sensor
            float new_temp = read_ds18b20();
            
            // Update state safely
            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->temperature = new_temp;
            furi_mutex_release(app->mutex);
            
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