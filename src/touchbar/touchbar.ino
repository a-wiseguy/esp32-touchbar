#include "lvgl.h"      //display library
#include "AXS15231B.h" // touch controller
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h> // mqtt client
#include "ui_setup.h"     // menu layouts
#include "mqtt_config.h"  // mqtt config

// two buffers to implement double buffering for better render performance
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

static int brightnesses[5] = {40, 80, 120, 150, 240};
static int selected_bright = 0;

uint8_t ALS_ADDRESS = 0x3B; // I2C address for the touch controller; used for communication with the touch controller.

#define TOUCH_IICSCL 10 // I2C clock pin definition, necessary for setting up communication.
#define TOUCH_IICSDA 15 // I2C data pin definition, used alongside the clock pin to transfer data.
#define TOUCH_RES 16    // This pin controls the reset signal to the touch controller, allowing the software to reset it when needed.

// The following macros and constants are designed to extract specific touch data from the buffer.
// These macros help simplify accessing the touch data, such as the X and Y coordinates of a touch, making the code more readable and maintainable.

#define AXS_TOUCH_ONE_POINT_LEN 6
#define AXS_TOUCH_BUF_HEAD_LEN 2
#define AXS_TOUCH_GESTURE_POS 0
#define AXS_TOUCH_POINT_NUM 1
#define AXS_TOUCH_EVENT_POS 2
#define AXS_TOUCH_X_H_POS 2
#define AXS_TOUCH_X_L_POS 3
#define AXS_TOUCH_ID_POS 4
#define AXS_TOUCH_Y_H_POS 4
#define AXS_TOUCH_Y_L_POS 5
#define AXS_TOUCH_WEIGHT_POS 6
#define AXS_TOUCH_AREA_POS 7

#define AXS_GET_POINT_NUM(buf) buf[AXS_TOUCH_POINT_NUM]
#define AXS_GET_GESTURE_TYPE(buf) buf[AXS_TOUCH_GESTURE_POS]
#define AXS_GET_POINT_X(buf, point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_X_H_POS] & 0x0F) << 8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_X_L_POS])
#define AXS_GET_POINT_Y(buf, point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_Y_H_POS] & 0x0F) << 8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_Y_L_POS])
#define AXS_GET_POINT_EVENT(buf, point_index) (buf[AXS_TOUCH_ONE_POINT_LEN * point_index + AXS_TOUCH_EVENT_POS] >> 6)

// Global references to the screens
extern lv_obj_t *main_menu;
extern lv_obj_t *volume_screen;
extern lv_obj_t *slider_label;

// Forward declarations of event handler functions
void slider_event_cb(lv_event_t *e);
void nav_click_event_cb(lv_event_t *e);
void nav_mute_event_cb(lv_event_t *e);
void back_event_cb(lv_event_t *e);
void btn_event_cb(lv_event_t *e);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// This function is responsible for transferring the rendered graphics buffer to the physical display.
// It abstracts the process of how the display gets updated, allowing LVGL to focus on drawing while this function handles the specifics of the display hardware.
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

#ifdef LCD_SPI_DMA
    // Ensure that any previous SPI DMA transaction is complete before starting a new one.
    char i = 0;
    while (get_lcd_spi_dma_write())
    {
        i = i >> 1; // Simple delay, essentially a no-op.
        lcd_PushColors(0, 0, 0, 0, NULL);
    }
#endif
    lcd_PushColors(area->x1, area->y1, w, h, (uint16_t *)&color_p->full); // Sends the pixel data to the display.

#ifdef LCD_SPI_DMA
#else
    lv_disp_flush_ready(disp); // Let LVGL know that flushing is done, so it can continue.
#endif
}

// Command sequence sent to the touch controller to initiate a read.
uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0, 0x0, 0x8};

// Global variables to keep track of the last touch position and time
static uint16_t lastX = 0;
static uint16_t lastY = 0;
static uint32_t lastTouchTime = 0;

// This function reads the touch data from the controller and updates the LVGL input device state.
// It handles how touch inputs are interpreted and fed into the LVGL system, translating raw touch data into usable coordinates.
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    uint8_t buff[20] = {0}; // Buffer for storing data read from the touch controller.

    // Begin communication with the touch controller and request data.
    Wire.beginTransmission(0x3B);
    Wire.write(read_touchpad_cmd, 8); // Send the read command.
    Wire.endTransmission();
    Wire.requestFrom(0x3B, 8); // Request 8 bytes of data.
    while (!Wire.available())
        ;                    // Wait until data is available.
    Wire.readBytes(buff, 8); // Read the data into the buffer.

    uint16_t pointX;
    uint16_t pointY;
    uint16_t type = 0;

    // Extract gesture type, X, and Y coordinates from the buffer.
    type = AXS_GET_GESTURE_TYPE(buff);
    pointX = AXS_GET_POINT_X(buff, 0);
    pointY = AXS_GET_POINT_Y(buff, 0);

    // Debouncing: Ignore very frequent updates within a short time interval
    uint32_t currentTime = millis();
    if (currentTime - lastTouchTime < 30)
    { // 30 ms debounce interval
        data->state = LV_INDEV_STATE_REL;
        return;
    }
    lastTouchTime = currentTime;

    // If a valid touch is detected, update the LVGL input data structure.
    if (!type && (pointX || pointY))
    {
        pointX = (640 - pointX); // Adjust X coordinate to the correct orientation.
        if (pointX > 640)
            pointX = 640; // Ensure X is within bounds.
        if (pointY > 180)
            pointY = 180; // Ensure Y is within bounds.

        // Ignore small movements (threshold)
        if (abs(pointX - lastX) < 5 && abs(pointY - lastY) < 5)
        {
            data->state = LV_INDEV_STATE_REL;
            return;
        }

        lastX = pointX;
        lastY = pointY;

        data->state = LV_INDEV_STATE_PR; // Mark input state as pressed.
        data->point.x = pointY;          // Update LVGL with the touch Y coordinate.
        data->point.y = pointX;          // Update LVGL with the touch X coordinate.

        // Optionally print the touch coordinates for debugging
        // char buf[20] = {0};
        // sprintf(buf, "(%d, %d)", data->point.x, data->point.y);
        // Serial.println(buf);
    }
    else
    {
        data->state = LV_INDEV_STATE_REL; // If no valid touch, set state to released.
    }
}

void slider_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    Serial.println(buf);
    lv_label_set_text(slider_label, buf);
    if (mqttClient.connected())
    {
        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d", (int)lv_slider_get_value(slider));
        mqttClient.publish(TOPIC_VOLUME, buf);
    }
}

// Switch to the vol slider screen
void nav_volume_event_cb(lv_event_t *e)
{
    lv_scr_load_anim(volume_screen, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, false);
}

void nav_mute_event_cb(lv_event_t *e)
{
    lv_obj_t * btn = lv_event_get_target(e);
    if (lv_obj_has_state(btn, LV_STATE_CHECKED))
    {
        Serial.print("mute is checked, publishing MUTE");
        if (mqttClient.connected())
        {
            mqttClient.publish(TOPIC_VOLUME, "MUTE");
        }
    }
    else
    {
        Serial.print("mute is unchecked, publishing UNMUTE");
        if (mqttClient.connected())
        {
            mqttClient.publish(TOPIC_VOLUME, "UNMUTE");
        }
    }
}

void nav_light_event_cb(lv_event_t *e)
{
    lv_obj_t *button = (lv_obj_t *)lv_event_get_user_data(e);
    // get toggle state
    bool is_toggled = lv_obj_has_state(button, LV_STATE_CHECKED);
    Serial.print("is_toggled: ");
    Serial.println(is_toggled);
    // if mqtt client is connected and checked, send mqtt message
    if (mqttClient.connected() && is_toggled)
    {
        mqttClient.publish(TOPIC_COUCH_LIGHT, "OFF");
    }
    else if (mqttClient.connected())
    {
        mqttClient.publish(TOPIC_COUCH_LIGHT, "ON");
    }
    else
    {
        Serial.println("mqtt not connected");
    }
}

// wifi events
void nav_wifi_event_cb(lv_event_t *e)
{
    lv_scr_load_anim(wifi_screen, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, false);
}

void connect_mqtt()
{
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    while (!mqttClient.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqttClient.connect("ESP32TOUCHBAR", MQTT_USER, MQTT_PASSWORD))
        {
            Serial.println("mqtt connected");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

// Function to connect to Wi-Fi
void setup_wifi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void connect_wifi_event_cb(lv_event_t *e)
{
    lv_obj_t *wifi_btn = (lv_obj_t *)lv_event_get_user_data(e);
    // clear flag clickable
    lv_obj_set_style_bg_color(wifi_btn, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_state(wifi_btn, LV_STATE_DISABLED);
    // connect
    setup_wifi();
    connect_mqtt();
    // after connect
    lv_obj_clear_state(wifi_btn, LV_STATE_DISABLED);
    lv_obj_set_style_bg_color(wifi_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
}

// Switch back to the main menu screen
void back_event_cb(lv_event_t *e)
{
    // lv_scr_load(main_menu);
    lv_scr_load_anim(main_menu, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, false);
}

// initializes hardware and software components, including the display, touch controller, and LVGL.
void setup()
{
    Serial.begin(115200); // Start serial communication for debugging.
    Serial.println("started setup");

    // pinMode(PIN_BUTTON_2, INPUT_PULLUP); // brightness button
    pinMode(TOUCH_RES, OUTPUT); // Set the touch controller reset pin as an output.

    // Perform a reset sequence on the touch controller. This ensures the controller starts in a known state.
    digitalWrite(TOUCH_RES, HIGH);
    delay(2);
    digitalWrite(TOUCH_RES, LOW);
    delay(10);
    digitalWrite(TOUCH_RES, HIGH);
    delay(2);

    // Initialize I2C communication with the touch controller using the specified pins.
    Wire.begin(TOUCH_IICSDA, TOUCH_IICSCL);

    pinMode(TFT_BL, OUTPUT);    // Set the backlight pin as an output.
    digitalWrite(TFT_BL, HIGH); // Turn on the display backlight.

    // setup brightness selector
    ledcSetup(0, 10000, 8);
    ledcAttachPin(TFT_BL, 0);
    ledcWrite(0, brightnesses[selected_bright]);

    axs15231_init(); // Initialize the AXS15231B touch controller. This is a custom function likely defined elsewhere.

    lv_init(); // Initialize the LVGL library, which sets up the internal structures needed to manage the GUI.

    // Allocate memory for the display buffer. LVGL needs this buffer to hold the current screen data.
    size_t buffer_size = sizeof(lv_color_t) * LVGL_LCD_BUF_SIZE;
    buf = (lv_color_t *)ps_malloc(buffer_size); // Allocate primary buffer.
    if (buf == NULL)
    { // Check if allocation failed.
        while (1)
        {
            Serial.println("buf NULL");
            delay(500);
        }
    }

    buf1 = (lv_color_t *)ps_malloc(buffer_size); // Allocate secondary buffer.
    if (buf1 == NULL)
    { // Check if allocation failed.
        while (1)
        {
            Serial.println("buf NULL");
            delay(500);
        }
    }

    // Initialize the LVGL draw buffer, linking it to the two allocated buffers.
    lv_disp_draw_buf_init(&draw_buf, buf, buf1, buffer_size);

    // Initialize and configure the display driver.
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES; // Set display resolution.
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = my_disp_flush; // Set the flush callback, which handles rendering.
    disp_drv.draw_buf = &draw_buf;     // Attach the draw buffer.
    disp_drv.full_refresh = 1;         // Enable full-refresh mode for the display, necessary for certain rotation settings.
    // rotate screen
    disp_drv.sw_rotate = 1;
    disp_drv.rotated = LV_DISP_ROT_90;
    lv_disp_drv_register(&disp_drv); // Register the display driver with LVGL.

    // Initialize the input device (touch) driver and configure it.
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER; // Indicate that this is a pointer-type device (e.g., touch screen).
    indev_drv.read_cb = my_touchpad_read;   // Set the read callback to handle touch input.
    lv_indev_drv_register(&indev_drv);      // Register the input device driver with LVGL.

    setup_ui(); // build UI screens and components

    Serial.println("setup finished");
}

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;
static int debounce1 = 0;

void loop()
{
    delay(1); // Short delay to prevent overwhelming the CPU.

    // manage the timing of when the screen gets updated.
    if (transfer_num <= 0 && lcd_PushColors_len <= 0)
        lv_timer_handler(); // updates LVGL's internal timers, refreshing the display if needed.

    if (transfer_num <= 1 && lcd_PushColors_len > 0)
    {
        lcd_PushColors(0, 0, 0, 0, NULL); // Ensure any pending data is pushed to the display.
    }

    // Check if the brightness button is pressed with debounce
    // if (digitalRead(PIN_BUTTON_2) == 0 && debounce1 == 0)
    // {
    //     Serial.println("PRESSED BTN2");
    //     debounce1 = 1;
    //     selected_bright++;
    //     if (selected_bright > 4) selected_bright = 0;
    // }
    // else if (digitalRead(PIN_BUTTON_2) == 1 && debounce1 == 1)
    // {
    //     debounce1 = 0;
    // }
}
