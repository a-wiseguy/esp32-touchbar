#ifndef UI_SETUP_H
#define UI_SETUP_H

#include "lvgl.h"

// Global references to the screens
extern lv_obj_t *main_menu;
extern lv_obj_t *volume_screen;
extern lv_obj_t *wifi_screen;
extern lv_obj_t *slider_label;
extern lv_obj_t *vol_slider;
extern lv_obj_t *nav_btns[4];

// Forward declarations for event callbacks
void nav_volume_event_cb(lv_event_t *e);
void nav_mute_event_cb(lv_event_t *e);
void nav_light_event_cb(lv_event_t *e);
void nav_wifi_event_cb(lv_event_t *e);
void connect_wifi_event_cb(lv_event_t *e);
void slider_event_cb(lv_event_t *e);
void back_event_cb(lv_event_t *e);

// Function to set up the UI
void setup_ui();

#endif // UI_SETUP_H