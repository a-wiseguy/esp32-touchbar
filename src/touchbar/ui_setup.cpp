#include "ui_setup.h"
#include <cstdio>

lv_obj_t *main_menu;
lv_obj_t *volume_screen;
lv_obj_t *wifi_screen;
lv_obj_t *slider_label;
lv_obj_t *nav_btns[4];

static lv_style_t style_large_text; 
static lv_style_t style_nav_btn;


lv_obj_t* create_nav_button(lv_obj_t *parent, const char *label_text, bool is_checkable)
{
    // Create a button on the parent object
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_add_style(btn, &style_nav_btn, 0);

    // Create label for the button
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, label_text);
    lv_obj_add_style(btn_label, &style_large_text, 0);

    if (is_checkable) {
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
    }

    return btn;
}

lv_obj_t* setup_main_menu()
{
    // Create the Main Menu screen
    main_menu = lv_obj_create(NULL); // dont load it yet

    // flex container for the nav buttons
    lv_obj_t * cont_row = lv_obj_create(main_menu);
    lv_obj_set_size(cont_row, LV_PCT(100), LV_PCT(100));
    lv_obj_align(cont_row, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_column(cont_row, 50, 0);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);

    // configure scrolling
    lv_obj_set_scroll_dir(cont_row, LV_DIR_HOR);
    lv_obj_set_scrollbar_mode(cont_row, LV_SCROLLBAR_MODE_ACTIVE); // show scrollbar only while dragging

    // nav buttons creation
    nav_btns[0] = create_nav_button(cont_row, LV_SYMBOL_VOLUME_MAX, true);  //mute btn
    nav_btns[1] = create_nav_button(cont_row, LV_SYMBOL_VOLUME_MID, false); // vol btn
    nav_btns[2] = create_nav_button(cont_row, LV_SYMBOL_WIFI, false);
    nav_btns[3] = create_nav_button(cont_row, LV_SYMBOL_POWER, true);

    // Assign btn events for nav/actions
    lv_obj_add_event_cb(nav_btns[0], nav_mute_event_cb, LV_EVENT_VALUE_CHANGED, lv_obj_get_child(nav_btns[0], 0));
    lv_obj_add_event_cb(nav_btns[1], nav_volume_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(nav_btns[2], nav_wifi_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(nav_btns[3], nav_light_event_cb, LV_EVENT_VALUE_CHANGED, nav_btns[3]);

    return main_menu;
}

lv_obj_t* setup_volume_screen()
{
    volume_screen = lv_obj_create(NULL);

    // Create and configure the slider
    lv_obj_t *slider = lv_slider_create(volume_screen);
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 50, LV_ANIM_ON);
    lv_obj_set_size(slider, 400, 25);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // slider volume percentage label
    slider_label = lv_label_create(volume_screen);
    lv_label_set_text(slider_label, "50%");
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -10);

    // Create a back button
    lv_obj_t *back_btn = lv_btn_create(volume_screen);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 15, 0);
    lv_obj_set_height(back_btn, LV_SIZE_CONTENT);
    lv_obj_set_style_opa(back_btn, LV_OPA_80, 0);
    lv_obj_add_event_cb(back_btn, back_event_cb, LV_EVENT_CLICKED, NULL);

    // Create a label for the back button
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_CLOSE);

    return volume_screen;
}

lv_obj_t* setup_wifi_screen()
{
    wifi_screen = lv_obj_create(NULL);

    // create connect wifi button
    lv_obj_t * wifi_btn = lv_btn_create(wifi_screen);
    lv_obj_align(wifi_btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(wifi_btn, 100, 100);

    // label
    lv_obj_t * wifi_btn_label = lv_label_create(wifi_btn);
    lv_label_set_text(wifi_btn_label, "connect");
    lv_obj_align(wifi_btn_label, LV_ALIGN_CENTER, 0, 0);

    // Set the event callback for the button
    lv_obj_add_event_cb(wifi_btn, connect_wifi_event_cb, LV_EVENT_CLICKED, wifi_btn);

    // Create a back button
    lv_obj_t *back_btn = lv_btn_create(wifi_screen);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 15, 0);
    lv_obj_set_height(back_btn, LV_SIZE_CONTENT);
    lv_obj_set_style_opa(back_btn, LV_OPA_80, 0);
    lv_obj_add_event_cb(back_btn, back_event_cb, LV_EVENT_CLICKED, NULL);

    // Create a label for the back button
    lv_obj_t *back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, LV_SYMBOL_CLOSE);

    return wifi_screen;
}

void setup_ui()
{
    // Setup Styles
    lv_style_init(&style_large_text);  // Font
    lv_style_set_text_font(&style_large_text, &lv_font_montserrat_38);
    lv_style_set_align(&style_large_text, LV_ALIGN_CENTER);

    lv_style_init(&style_nav_btn); // Nav buttons
    lv_coord_t btn_size = 125;
    lv_style_set_size(&style_nav_btn, btn_size);

    // Create the pages
    main_menu = setup_main_menu();
    volume_screen = setup_volume_screen();
    wifi_screen = setup_wifi_screen();

    lv_scr_load(main_menu); // Load the main menu screen
}
