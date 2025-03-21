// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.0.5
// LVGL VERSION: 8.2
// PROJECT: Medical

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t * ui_Medical_Screen;
lv_obj_t * ui_Panel_History;
lv_obj_t * ui_Label_History;
lv_obj_t * ui_Label_YBP;
lv_obj_t * ui_Panel_History_Content;
lv_obj_t * ui_Panel_Today;
lv_obj_t * ui_Label_Today;
lv_obj_t * ui_Panel_Today_Content_1;
lv_obj_t * ui_Label_List_1;
lv_obj_t * ui_Label_Date_1;
lv_obj_t * ui_Label_List_1_copy;
lv_obj_t * ui_Image1;
lv_obj_t * ui_Panel_Today_Content_2;
lv_obj_t * ui_Label_List_2;
lv_obj_t * ui_Label_Date_2;
lv_obj_t * ui_Label_List_Pul_2;
lv_obj_t * ui_IMG_Heart_Small_1;
lv_obj_t * ui_Panel_Today_Content_3;
lv_obj_t * ui_Label_List_3;
lv_obj_t * ui_Label_Date_3;
lv_obj_t * ui_Label_List_Pul_3;
lv_obj_t * ui_IMG_Heart_Small_2;
lv_obj_t * ui_Panel_Yesterday;
lv_obj_t * ui_Label_Yesterday;
lv_obj_t * ui_Panel_Today_Content_5;
lv_obj_t * ui_Label_List_5;
lv_obj_t * ui_Label_Date_5;
lv_obj_t * ui_Label_List_Pul_5;
lv_obj_t * ui_IMG_Heart_Small_5;
lv_obj_t * ui_Panel_Today_Content_6;
lv_obj_t * ui_Label_List_6;
lv_obj_t * ui_Label_Date_6;
lv_obj_t * ui_Label_List_Pul_6;
lv_obj_t * ui_IMG_Heart_Small_6;
lv_obj_t * ui_Panel_Today_Content_7;
lv_obj_t * ui_Label_List_7;
lv_obj_t * ui_Label_Date_7;
lv_obj_t * ui_Label_List_Pul_7;
lv_obj_t * ui_IMG_Heart_Small_7;
lv_obj_t * ui_Panel_S1;
lv_obj_t * ui_BTN_History;
lv_obj_t * ui_Label_Date;
lv_obj_t * ui_Label_Hour;
lv_obj_t * ui_IMG_Battery;
lv_obj_t * ui_IMG_Bluetooth;
lv_obj_t * ui_Label_Battey;
lv_obj_t * ui_Panel_Content;
lv_obj_t * ui_Slider1;
lv_obj_t * ui_Line_1;
lv_obj_t * ui_Line_2;
lv_obj_t * ui_Line_3;
lv_obj_t * ui_Line_4;
lv_obj_t * ui_IMG_Line_1;
lv_obj_t * ui_IMG_Line_2;
lv_obj_t * ui_IMG_Line_3;
lv_obj_t * ui_Number_SYS;
lv_obj_t * ui_Number_DIA;
lv_obj_t * ui_Number_PUL;
lv_obj_t * ui_Label_SYS;
lv_obj_t * ui_Label_mmHg_1;
lv_obj_t * ui_Label_DIA;
lv_obj_t * ui_Label_mmHg_2;
lv_obj_t * ui_Label_PUL;
lv_obj_t * ui_Label_mmHg_3;
lv_obj_t * ui_IMG_Heart;
lv_obj_t * ui_Panel_Bottom;
lv_obj_t * ui_IMG_BTN_Bg;
lv_obj_t * ui_BTN_Power;
lv_obj_t * ui_Switch_Sound;
lv_obj_t * ui_Label_Sound;
lv_obj_t * ui_Switch_Bluetooth;
lv_obj_t * ui_Label_Bluetooth;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "#error LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

//
// FUNCTION HEADER
void HistoryAnim_Animation(lv_obj_t * TargetObject, int delay);

// FUNCTION
void HistoryAnim_Animation(lv_obj_t * TargetObject, int delay)
{

    //
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 300);
    lv_anim_set_user_data(&PropertyAnimation_0, TargetObject);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_x);
    lv_anim_set_values(&PropertyAnimation_0, 0, 200);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_overshoot);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_set_get_value_cb(&PropertyAnimation_0, &_ui_anim_callback_get_x);
    lv_anim_start(&PropertyAnimation_0);

}

//
// FUNCTION HEADER
void HistoryAnimOff_Animation(lv_obj_t * TargetObject, int delay);

// FUNCTION
void HistoryAnimOff_Animation(lv_obj_t * TargetObject, int delay)
{

    //
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 300);
    lv_anim_set_user_data(&PropertyAnimation_0, TargetObject);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_x);
    lv_anim_set_values(&PropertyAnimation_0, 200, 0);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_ease_out);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_start(&PropertyAnimation_0);

}

//
// FUNCTION HEADER
void HeartAnim_Animation(lv_obj_t * TargetObject, int delay);

// FUNCTION
void HeartAnim_Animation(lv_obj_t * TargetObject, int delay)
{

    //
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 200);
    lv_anim_set_user_data(&PropertyAnimation_0, TargetObject);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_image_zoom);
    lv_anim_set_values(&PropertyAnimation_0, 400, 256);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_start(&PropertyAnimation_0);

    //
    lv_anim_t PropertyAnimation_1;
    lv_anim_init(&PropertyAnimation_1);
    lv_anim_set_time(&PropertyAnimation_1, 200);
    lv_anim_set_user_data(&PropertyAnimation_1, TargetObject);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_1, _ui_anim_callback_set_image_zoom);
    lv_anim_set_values(&PropertyAnimation_1, 400, 256);
    lv_anim_set_path_cb(&PropertyAnimation_1, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_1, delay + 500);
    lv_anim_set_early_apply(&PropertyAnimation_1, false);
    lv_anim_start(&PropertyAnimation_1);

    //
    lv_anim_t PropertyAnimation_2;
    lv_anim_init(&PropertyAnimation_2);
    lv_anim_set_time(&PropertyAnimation_2, 200);
    lv_anim_set_user_data(&PropertyAnimation_2, TargetObject);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_2, _ui_anim_callback_set_image_zoom);
    lv_anim_set_values(&PropertyAnimation_2, 400, 256);
    lv_anim_set_path_cb(&PropertyAnimation_2, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_2, delay + 1000);
    lv_anim_set_early_apply(&PropertyAnimation_2, false);
    lv_anim_start(&PropertyAnimation_2);

    //
    lv_anim_t PropertyAnimation_3;
    lv_anim_init(&PropertyAnimation_3);
    lv_anim_set_time(&PropertyAnimation_3, 200);
    lv_anim_set_user_data(&PropertyAnimation_3, TargetObject);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_3, _ui_anim_callback_set_image_zoom);
    lv_anim_set_values(&PropertyAnimation_3, 400, 256);
    lv_anim_set_path_cb(&PropertyAnimation_3, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_3, delay + 1500);
    lv_anim_set_early_apply(&PropertyAnimation_3, false);
    lv_anim_start(&PropertyAnimation_3);

    //
    lv_anim_t PropertyAnimation_4;
    lv_anim_init(&PropertyAnimation_4);
    lv_anim_set_time(&PropertyAnimation_4, 200);
    lv_anim_set_user_data(&PropertyAnimation_4, TargetObject);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_4, _ui_anim_callback_set_image_zoom);
    lv_anim_set_values(&PropertyAnimation_4, 400, 256);
    lv_anim_set_path_cb(&PropertyAnimation_4, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_4, delay + 2000);
    lv_anim_set_early_apply(&PropertyAnimation_4, false);
    lv_anim_start(&PropertyAnimation_4);

}

///////////////////// FUNCTIONS ////////////////////
static void ui_event_Panel_S1(lv_event_t * e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    if(event == LV_EVENT_CLICKED) {
        HistoryAnimOff_Animation(ui_Panel_S1, 0);
        _ui_flag_modify(ui_Panel_S1, LV_OBJ_FLAG_CLICKABLE, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_BTN_History, LV_OBJ_FLAG_CLICKABLE, _UI_MODIFY_FLAG_ADD);
    }
}
static void ui_event_BTN_History(lv_event_t * e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    if(event == LV_EVENT_CLICKED) {
        HistoryAnim_Animation(ui_Panel_S1, 0);
        _ui_flag_modify(ui_BTN_History, LV_OBJ_FLAG_CLICKABLE, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_Panel_S1, LV_OBJ_FLAG_CLICKABLE, _UI_MODIFY_FLAG_ADD);
    }
}
static void ui_event_BTN_Power(lv_event_t * e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    if(event == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(ta, LV_STATE_CHECKED)) {
        HeartAnim_Animation(ui_IMG_Heart, 0);
    }
}
static void ui_event_Switch_Bluetooth(lv_event_t * e)
{
    lv_event_code_t event = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    if(event == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_IMG_Bluetooth, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
    }
}

///////////////////// SCREENS ////////////////////
void ui_Medical_Screen_screen_init(void)
{

    // ui_Medical_Screen

    ui_Medical_Screen = lv_obj_create(NULL);

    lv_obj_clear_flag(ui_Medical_Screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Medical_Screen, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Medical_Screen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Panel_History

    ui_Panel_History = lv_obj_create(ui_Medical_Screen);

    lv_obj_set_width(ui_Panel_History, lv_pct(100));
    lv_obj_set_height(ui_Panel_History, lv_pct(100));

    lv_obj_set_x(ui_Panel_History, 0);
    lv_obj_set_y(ui_Panel_History, 0);

    lv_obj_set_align(ui_Panel_History, LV_ALIGN_CENTER);

    lv_obj_clear_flag(ui_Panel_History, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_History, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_History, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_History, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_History, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_History, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_History

    ui_Label_History = lv_label_create(ui_Panel_History);

    lv_obj_set_width(ui_Label_History, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_History, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_History, 0);
    lv_obj_set_y(ui_Label_History, 22);

    lv_label_set_text(ui_Label_History, "History");

    lv_obj_set_style_text_color(ui_Label_History, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_History, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_History, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_YBP

    ui_Label_YBP = lv_label_create(ui_Panel_History);

    lv_obj_set_width(ui_Label_YBP, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_YBP, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_YBP, 0);
    lv_obj_set_y(ui_Label_YBP, 0);

    lv_label_set_text(ui_Label_YBP, "Your blood pressure");

    lv_obj_set_style_text_color(ui_Label_YBP, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_YBP, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_YBP, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Panel_History_Content

    ui_Panel_History_Content = lv_obj_create(ui_Panel_History);

    lv_obj_set_width(ui_Panel_History_Content, 272);
    lv_obj_set_height(ui_Panel_History_Content, 406);

    lv_obj_set_x(ui_Panel_History_Content, -13);
    lv_obj_set_y(ui_Panel_History_Content, 60);

    lv_obj_set_style_bg_main_stop(ui_Panel_History_Content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_History_Content, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_History_Content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Panel_Today

    ui_Panel_Today = lv_obj_create(ui_Panel_History);

    lv_obj_set_width(ui_Panel_Today, 266);
    lv_obj_set_height(ui_Panel_Today, 177);

    lv_obj_set_x(ui_Panel_Today, -10);
    lv_obj_set_y(ui_Panel_Today, 60);

    lv_obj_set_style_bg_color(ui_Panel_Today, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Today, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Today, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Today, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Today, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Today, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Today, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Today, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Today, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Today

    ui_Label_Today = lv_label_create(ui_Panel_Today);

    lv_obj_set_width(ui_Label_Today, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Today, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Today, 15);
    lv_obj_set_y(ui_Label_Today, 10);

    lv_label_set_text(ui_Label_Today, "TODAY");

    lv_obj_set_style_text_color(ui_Label_Today, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Today, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Today, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Panel_Today_Content_1

    ui_Panel_Today_Content_1 = lv_obj_create(ui_Panel_Today);

    lv_obj_set_height(ui_Panel_Today_Content_1, 45);
    lv_obj_set_width(ui_Panel_Today_Content_1, lv_pct(100));

    lv_obj_set_x(ui_Panel_Today_Content_1, 0);
    lv_obj_set_y(ui_Panel_Today_Content_1, 30);

    lv_obj_clear_flag(ui_Panel_Today_Content_1, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_Today_Content_1, lv_color_hex(0xE5EEF5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Today_Content_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Today_Content_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Today_Content_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Today_Content_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Today_Content_1, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Today_Content_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Today_Content_1, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Today_Content_1, 5, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_1

    ui_Label_List_1 = lv_label_create(ui_Panel_Today_Content_1);

    lv_obj_set_width(ui_Label_List_1, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_1, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_1, 0);
    lv_obj_set_y(ui_Label_List_1, 0);

    lv_label_set_text(ui_Label_List_1, "185 / 65");

    lv_obj_set_style_text_color(ui_Label_List_1, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_1, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Date_1

    ui_Label_Date_1 = lv_label_create(ui_Panel_Today_Content_1);

    lv_obj_set_width(ui_Label_Date_1, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Date_1, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Date_1, 0);
    lv_obj_set_y(ui_Label_Date_1, 0);

    lv_obj_set_align(ui_Label_Date_1, LV_ALIGN_BOTTOM_LEFT);

    lv_label_set_text(ui_Label_Date_1, "18:02  05 10 2022");

    lv_obj_set_style_text_color(ui_Label_Date_1, lv_color_hex(0x68727F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Date_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Date_1, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_1_copy

    ui_Label_List_1_copy = lv_label_create(ui_Panel_Today_Content_1);

    lv_obj_set_width(ui_Label_List_1_copy, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_1_copy, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_1_copy, 0);
    lv_obj_set_y(ui_Label_List_1_copy, 0);

    lv_obj_set_align(ui_Label_List_1_copy, LV_ALIGN_CENTER);

    lv_label_set_text(ui_Label_List_1_copy, "75");

    lv_obj_set_style_text_color(ui_Label_List_1_copy, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_1_copy, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_1_copy, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Image1

    ui_Image1 = lv_img_create(ui_Panel_Today_Content_1);
    lv_img_set_src(ui_Image1, &ui_img_img_heart_small_png);

    lv_obj_set_width(ui_Image1, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Image1, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Image1, 20);
    lv_obj_set_y(ui_Image1, 0);

    lv_obj_set_align(ui_Image1, LV_ALIGN_CENTER);

    lv_obj_add_flag(ui_Image1, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_Image1, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Panel_Today_Content_2

    ui_Panel_Today_Content_2 = lv_obj_create(ui_Panel_Today);

    lv_obj_set_height(ui_Panel_Today_Content_2, 45);
    lv_obj_set_width(ui_Panel_Today_Content_2, lv_pct(100));

    lv_obj_set_x(ui_Panel_Today_Content_2, 0);
    lv_obj_set_y(ui_Panel_Today_Content_2, 80);

    lv_obj_clear_flag(ui_Panel_Today_Content_2, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_Today_Content_2, lv_color_hex(0xE5EEF5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Today_Content_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Today_Content_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Today_Content_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Today_Content_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Today_Content_2, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Today_Content_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Today_Content_2, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Today_Content_2, 5, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_2

    ui_Label_List_2 = lv_label_create(ui_Panel_Today_Content_2);

    lv_obj_set_width(ui_Label_List_2, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_2, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_2, 0);
    lv_obj_set_y(ui_Label_List_2, 0);

    lv_label_set_text(ui_Label_List_2, "162 / 70");

    lv_obj_set_style_text_color(ui_Label_List_2, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_2, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Date_2

    ui_Label_Date_2 = lv_label_create(ui_Panel_Today_Content_2);

    lv_obj_set_width(ui_Label_Date_2, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Date_2, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Date_2, 0);
    lv_obj_set_y(ui_Label_Date_2, 0);

    lv_obj_set_align(ui_Label_Date_2, LV_ALIGN_BOTTOM_LEFT);

    lv_label_set_text(ui_Label_Date_2, "16:58  05 10 2022");

    lv_obj_set_style_text_color(ui_Label_Date_2, lv_color_hex(0x68727F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Date_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Date_2, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_Pul_2

    ui_Label_List_Pul_2 = lv_label_create(ui_Panel_Today_Content_2);

    lv_obj_set_width(ui_Label_List_Pul_2, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_Pul_2, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_Pul_2, 0);
    lv_obj_set_y(ui_Label_List_Pul_2, 0);

    lv_obj_set_align(ui_Label_List_Pul_2, LV_ALIGN_CENTER);

    lv_label_set_text(ui_Label_List_Pul_2, "68");

    lv_obj_set_style_text_color(ui_Label_List_Pul_2, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_Pul_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_Pul_2, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_Heart_Small_1

    ui_IMG_Heart_Small_1 = lv_img_create(ui_Panel_Today_Content_2);
    lv_img_set_src(ui_IMG_Heart_Small_1, &ui_img_img_heart_small_png);

    lv_obj_set_width(ui_IMG_Heart_Small_1, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_Heart_Small_1, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_Heart_Small_1, 20);
    lv_obj_set_y(ui_IMG_Heart_Small_1, 0);

    lv_obj_set_align(ui_IMG_Heart_Small_1, LV_ALIGN_CENTER);

    lv_obj_add_flag(ui_IMG_Heart_Small_1, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Heart_Small_1, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Panel_Today_Content_3

    ui_Panel_Today_Content_3 = lv_obj_create(ui_Panel_Today);

    lv_obj_set_height(ui_Panel_Today_Content_3, 45);
    lv_obj_set_width(ui_Panel_Today_Content_3, lv_pct(100));

    lv_obj_set_x(ui_Panel_Today_Content_3, 0);
    lv_obj_set_y(ui_Panel_Today_Content_3, 130);

    lv_obj_clear_flag(ui_Panel_Today_Content_3, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_Today_Content_3, lv_color_hex(0xE5EEF5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Today_Content_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Today_Content_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Today_Content_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Today_Content_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Today_Content_3, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Today_Content_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Today_Content_3, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Today_Content_3, 5, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_3

    ui_Label_List_3 = lv_label_create(ui_Panel_Today_Content_3);

    lv_obj_set_width(ui_Label_List_3, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_3, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_3, 0);
    lv_obj_set_y(ui_Label_List_3, 0);

    lv_label_set_text(ui_Label_List_3, "140 / 68");

    lv_obj_set_style_text_color(ui_Label_List_3, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_3, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Date_3

    ui_Label_Date_3 = lv_label_create(ui_Panel_Today_Content_3);

    lv_obj_set_width(ui_Label_Date_3, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Date_3, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Date_3, 0);
    lv_obj_set_y(ui_Label_Date_3, 0);

    lv_obj_set_align(ui_Label_Date_3, LV_ALIGN_BOTTOM_LEFT);

    lv_label_set_text(ui_Label_Date_3, "10:35  05 10 2022");

    lv_obj_set_style_text_color(ui_Label_Date_3, lv_color_hex(0x68727F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Date_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Date_3, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_Pul_3

    ui_Label_List_Pul_3 = lv_label_create(ui_Panel_Today_Content_3);

    lv_obj_set_width(ui_Label_List_Pul_3, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_Pul_3, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_Pul_3, 0);
    lv_obj_set_y(ui_Label_List_Pul_3, 0);

    lv_obj_set_align(ui_Label_List_Pul_3, LV_ALIGN_CENTER);

    lv_label_set_text(ui_Label_List_Pul_3, "59");

    lv_obj_set_style_text_color(ui_Label_List_Pul_3, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_Pul_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_Pul_3, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_Heart_Small_2

    ui_IMG_Heart_Small_2 = lv_img_create(ui_Panel_Today_Content_3);
    lv_img_set_src(ui_IMG_Heart_Small_2, &ui_img_img_heart_small_png);

    lv_obj_set_width(ui_IMG_Heart_Small_2, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_Heart_Small_2, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_Heart_Small_2, 20);
    lv_obj_set_y(ui_IMG_Heart_Small_2, 0);

    lv_obj_set_align(ui_IMG_Heart_Small_2, LV_ALIGN_CENTER);

    lv_obj_add_flag(ui_IMG_Heart_Small_2, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Heart_Small_2, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Panel_Yesterday

    ui_Panel_Yesterday = lv_obj_create(ui_Panel_History);

    lv_obj_set_width(ui_Panel_Yesterday, 266);
    lv_obj_set_height(ui_Panel_Yesterday, 177);

    lv_obj_set_x(ui_Panel_Yesterday, -10);
    lv_obj_set_y(ui_Panel_Yesterday, 240);

    lv_obj_set_style_bg_color(ui_Panel_Yesterday, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Yesterday, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Yesterday, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Yesterday, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Yesterday, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Yesterday, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Yesterday, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Yesterday, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Yesterday, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Yesterday

    ui_Label_Yesterday = lv_label_create(ui_Panel_Yesterday);

    lv_obj_set_width(ui_Label_Yesterday, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Yesterday, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Yesterday, 15);
    lv_obj_set_y(ui_Label_Yesterday, 10);

    lv_label_set_text(ui_Label_Yesterday, "Yesterday");

    lv_obj_set_style_text_color(ui_Label_Yesterday, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Yesterday, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Yesterday, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Panel_Today_Content_5

    ui_Panel_Today_Content_5 = lv_obj_create(ui_Panel_Yesterday);

    lv_obj_set_height(ui_Panel_Today_Content_5, 45);
    lv_obj_set_width(ui_Panel_Today_Content_5, lv_pct(100));

    lv_obj_set_x(ui_Panel_Today_Content_5, 0);
    lv_obj_set_y(ui_Panel_Today_Content_5, 30);

    lv_obj_clear_flag(ui_Panel_Today_Content_5, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_Today_Content_5, lv_color_hex(0xE5EEF5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Today_Content_5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Today_Content_5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Today_Content_5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Today_Content_5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Today_Content_5, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Today_Content_5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Today_Content_5, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Today_Content_5, 5, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_5

    ui_Label_List_5 = lv_label_create(ui_Panel_Today_Content_5);

    lv_obj_set_width(ui_Label_List_5, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_5, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_5, 0);
    lv_obj_set_y(ui_Label_List_5, 0);

    lv_label_set_text(ui_Label_List_5, "145 / 58");

    lv_obj_set_style_text_color(ui_Label_List_5, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_5, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Date_5

    ui_Label_Date_5 = lv_label_create(ui_Panel_Today_Content_5);

    lv_obj_set_width(ui_Label_Date_5, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Date_5, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Date_5, 0);
    lv_obj_set_y(ui_Label_Date_5, 0);

    lv_obj_set_align(ui_Label_Date_5, LV_ALIGN_BOTTOM_LEFT);

    lv_label_set_text(ui_Label_Date_5, "18:02  04 10 2022");

    lv_obj_set_style_text_color(ui_Label_Date_5, lv_color_hex(0x68727F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Date_5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Date_5, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_Pul_5

    ui_Label_List_Pul_5 = lv_label_create(ui_Panel_Today_Content_5);

    lv_obj_set_width(ui_Label_List_Pul_5, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_Pul_5, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_Pul_5, 0);
    lv_obj_set_y(ui_Label_List_Pul_5, 0);

    lv_obj_set_align(ui_Label_List_Pul_5, LV_ALIGN_CENTER);

    lv_label_set_text(ui_Label_List_Pul_5, "75");

    lv_obj_set_style_text_color(ui_Label_List_Pul_5, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_Pul_5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_Pul_5, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_Heart_Small_5

    ui_IMG_Heart_Small_5 = lv_img_create(ui_Panel_Today_Content_5);
    lv_img_set_src(ui_IMG_Heart_Small_5, &ui_img_img_heart_small_png);

    lv_obj_set_width(ui_IMG_Heart_Small_5, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_Heart_Small_5, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_Heart_Small_5, 20);
    lv_obj_set_y(ui_IMG_Heart_Small_5, 0);

    lv_obj_set_align(ui_IMG_Heart_Small_5, LV_ALIGN_CENTER);

    lv_obj_add_flag(ui_IMG_Heart_Small_5, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Heart_Small_5, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Panel_Today_Content_6

    ui_Panel_Today_Content_6 = lv_obj_create(ui_Panel_Yesterday);

    lv_obj_set_height(ui_Panel_Today_Content_6, 45);
    lv_obj_set_width(ui_Panel_Today_Content_6, lv_pct(100));

    lv_obj_set_x(ui_Panel_Today_Content_6, 0);
    lv_obj_set_y(ui_Panel_Today_Content_6, 80);

    lv_obj_clear_flag(ui_Panel_Today_Content_6, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_Today_Content_6, lv_color_hex(0xE5EEF5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Today_Content_6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Today_Content_6, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Today_Content_6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Today_Content_6, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Today_Content_6, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Today_Content_6, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Today_Content_6, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Today_Content_6, 5, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_6

    ui_Label_List_6 = lv_label_create(ui_Panel_Today_Content_6);

    lv_obj_set_width(ui_Label_List_6, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_6, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_6, 0);
    lv_obj_set_y(ui_Label_List_6, 0);

    lv_label_set_text(ui_Label_List_6, "162 / 72");

    lv_obj_set_style_text_color(ui_Label_List_6, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_6, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Date_6

    ui_Label_Date_6 = lv_label_create(ui_Panel_Today_Content_6);

    lv_obj_set_width(ui_Label_Date_6, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Date_6, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Date_6, 0);
    lv_obj_set_y(ui_Label_Date_6, 0);

    lv_obj_set_align(ui_Label_Date_6, LV_ALIGN_BOTTOM_LEFT);

    lv_label_set_text(ui_Label_Date_6, "12:41  04 10 2022");

    lv_obj_set_style_text_color(ui_Label_Date_6, lv_color_hex(0x68727F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Date_6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Date_6, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_Pul_6

    ui_Label_List_Pul_6 = lv_label_create(ui_Panel_Today_Content_6);

    lv_obj_set_width(ui_Label_List_Pul_6, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_Pul_6, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_Pul_6, 0);
    lv_obj_set_y(ui_Label_List_Pul_6, 0);

    lv_obj_set_align(ui_Label_List_Pul_6, LV_ALIGN_CENTER);

    lv_label_set_text(ui_Label_List_Pul_6, "72");

    lv_obj_set_style_text_color(ui_Label_List_Pul_6, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_Pul_6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_Pul_6, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_Heart_Small_6

    ui_IMG_Heart_Small_6 = lv_img_create(ui_Panel_Today_Content_6);
    lv_img_set_src(ui_IMG_Heart_Small_6, &ui_img_img_heart_small_png);

    lv_obj_set_width(ui_IMG_Heart_Small_6, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_Heart_Small_6, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_Heart_Small_6, 20);
    lv_obj_set_y(ui_IMG_Heart_Small_6, 0);

    lv_obj_set_align(ui_IMG_Heart_Small_6, LV_ALIGN_CENTER);

    lv_obj_add_flag(ui_IMG_Heart_Small_6, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Heart_Small_6, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Panel_Today_Content_7

    ui_Panel_Today_Content_7 = lv_obj_create(ui_Panel_Yesterday);

    lv_obj_set_height(ui_Panel_Today_Content_7, 45);
    lv_obj_set_width(ui_Panel_Today_Content_7, lv_pct(100));

    lv_obj_set_x(ui_Panel_Today_Content_7, 0);
    lv_obj_set_y(ui_Panel_Today_Content_7, 130);

    lv_obj_clear_flag(ui_Panel_Today_Content_7, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_Today_Content_7, lv_color_hex(0xE5EEF5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Today_Content_7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Today_Content_7, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Today_Content_7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Today_Content_7, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Today_Content_7, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Today_Content_7, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Today_Content_7, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Today_Content_7, 5, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_7

    ui_Label_List_7 = lv_label_create(ui_Panel_Today_Content_7);

    lv_obj_set_width(ui_Label_List_7, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_7, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_7, 0);
    lv_obj_set_y(ui_Label_List_7, 0);

    lv_label_set_text(ui_Label_List_7, "132 / 69");

    lv_obj_set_style_text_color(ui_Label_List_7, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_7, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Date_7

    ui_Label_Date_7 = lv_label_create(ui_Panel_Today_Content_7);

    lv_obj_set_width(ui_Label_Date_7, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Date_7, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Date_7, 0);
    lv_obj_set_y(ui_Label_Date_7, 0);

    lv_obj_set_align(ui_Label_Date_7, LV_ALIGN_BOTTOM_LEFT);

    lv_label_set_text(ui_Label_Date_7, "10:30  04 10 2022");

    lv_obj_set_style_text_color(ui_Label_Date_7, lv_color_hex(0x68727F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Date_7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Date_7, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_List_Pul_7

    ui_Label_List_Pul_7 = lv_label_create(ui_Panel_Today_Content_7);

    lv_obj_set_width(ui_Label_List_Pul_7, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_List_Pul_7, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_List_Pul_7, 0);
    lv_obj_set_y(ui_Label_List_Pul_7, 0);

    lv_obj_set_align(ui_Label_List_Pul_7, LV_ALIGN_CENTER);

    lv_label_set_text(ui_Label_List_Pul_7, "64");

    lv_obj_set_style_text_color(ui_Label_List_Pul_7, lv_color_hex(0x284364), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_List_Pul_7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_List_Pul_7, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_Heart_Small_7

    ui_IMG_Heart_Small_7 = lv_img_create(ui_Panel_Today_Content_7);
    lv_img_set_src(ui_IMG_Heart_Small_7, &ui_img_img_heart_small_png);

    lv_obj_set_width(ui_IMG_Heart_Small_7, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_Heart_Small_7, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_Heart_Small_7, 20);
    lv_obj_set_y(ui_IMG_Heart_Small_7, 0);

    lv_obj_set_align(ui_IMG_Heart_Small_7, LV_ALIGN_CENTER);

    lv_obj_add_flag(ui_IMG_Heart_Small_7, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Heart_Small_7, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Panel_S1

    ui_Panel_S1 = lv_obj_create(ui_Medical_Screen);

    lv_obj_set_width(ui_Panel_S1, lv_pct(100));
    lv_obj_set_height(ui_Panel_S1, lv_pct(100));

    lv_obj_set_x(ui_Panel_S1, 0);
    lv_obj_set_y(ui_Panel_S1, 0);

    lv_obj_set_align(ui_Panel_S1, LV_ALIGN_CENTER);

    lv_obj_clear_flag(ui_Panel_S1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_event_cb(ui_Panel_S1, ui_event_Panel_S1, LV_EVENT_ALL, NULL);
    lv_obj_set_style_bg_color(ui_Panel_S1, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_S1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_S1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_S1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_S1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Panel_S1, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_S1, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_S1, 60, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_S1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_S1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_S1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_S1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_S1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_BTN_History

    ui_BTN_History = lv_img_create(ui_Panel_S1);
    lv_img_set_src(ui_BTN_History, &ui_img_icn_history_png);

    lv_obj_set_width(ui_BTN_History, 20);
    lv_obj_set_height(ui_BTN_History, 20);

    lv_obj_set_x(ui_BTN_History, 15);
    lv_obj_set_y(ui_BTN_History, 15);

    lv_obj_add_flag(ui_BTN_History, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_BTN_History, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_event_cb(ui_BTN_History, ui_event_BTN_History, LV_EVENT_ALL, NULL);

    // ui_Label_Date

    ui_Label_Date = lv_label_create(ui_Panel_S1);

    lv_obj_set_width(ui_Label_Date, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Date, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Date, 0);
    lv_obj_set_y(ui_Label_Date, 8);

    lv_obj_set_align(ui_Label_Date, LV_ALIGN_TOP_MID);

    lv_label_set_text(ui_Label_Date, "01 05 2022");

    lv_obj_set_style_text_color(ui_Label_Date, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Date, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Date, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_Hour

    ui_Label_Hour = lv_label_create(ui_Panel_S1);

    lv_obj_set_width(ui_Label_Hour, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Hour, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Hour, 0);
    lv_obj_set_y(ui_Label_Hour, 25);

    lv_obj_set_align(ui_Label_Hour, LV_ALIGN_TOP_MID);

    lv_label_set_text(ui_Label_Hour, "20:15");

    lv_obj_set_style_text_color(ui_Label_Hour, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Hour, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Hour, &ui_font_Bold_Font, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_Battery

    ui_IMG_Battery = lv_img_create(ui_Panel_S1);
    lv_img_set_src(ui_IMG_Battery, &ui_img_icn_battery_png);

    lv_obj_set_width(ui_IMG_Battery, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_Battery, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_Battery, -10);
    lv_obj_set_y(ui_IMG_Battery, 10);

    lv_obj_set_align(ui_IMG_Battery, LV_ALIGN_TOP_RIGHT);

    lv_obj_add_flag(ui_IMG_Battery, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Battery, LV_OBJ_FLAG_SCROLLABLE);

    // ui_IMG_Bluetooth

    ui_IMG_Bluetooth = lv_img_create(ui_Panel_S1);
    lv_img_set_src(ui_IMG_Bluetooth, &ui_img_icn_bluetooth_png);

    lv_obj_set_width(ui_IMG_Bluetooth, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_Bluetooth, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_Bluetooth, -40);
    lv_obj_set_y(ui_IMG_Bluetooth, 10);

    lv_obj_set_align(ui_IMG_Bluetooth, LV_ALIGN_TOP_RIGHT);

    lv_obj_add_flag(ui_IMG_Bluetooth, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Bluetooth, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Label_Battey

    ui_Label_Battey = lv_label_create(ui_Panel_S1);

    lv_obj_set_width(ui_Label_Battey, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Battey, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Battey, -10);
    lv_obj_set_y(ui_Label_Battey, 25);

    lv_obj_set_align(ui_Label_Battey, LV_ALIGN_TOP_RIGHT);

    lv_label_set_text(ui_Label_Battey, "18%");

    lv_obj_set_style_text_color(ui_Label_Battey, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Battey, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Battey, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Panel_Content

    ui_Panel_Content = lv_obj_create(ui_Panel_S1);

    lv_obj_set_height(ui_Panel_Content, 450);
    lv_obj_set_width(ui_Panel_Content, lv_pct(100));

    lv_obj_set_x(ui_Panel_Content, 0);
    lv_obj_set_y(ui_Panel_Content, 54);

    lv_obj_set_align(ui_Panel_Content, LV_ALIGN_TOP_MID);

    lv_obj_clear_flag(ui_Panel_Content, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_Content, lv_color_hex(0xF8F8F9), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Content, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Panel_Content, lv_color_hex(0xDCE5ED), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Content, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui_Panel_Content, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Content, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Slider1

    ui_Slider1 = lv_slider_create(ui_Panel_Content);
    lv_slider_set_range(ui_Slider1, 0, 100);
    lv_slider_set_value(ui_Slider1, 80, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_Slider1) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_Slider1, 0, LV_ANIM_OFF);

    lv_obj_set_width(ui_Slider1, 19);
    lv_obj_set_height(ui_Slider1, 235);

    lv_obj_set_x(ui_Slider1, -100);
    lv_obj_set_y(ui_Slider1, -78);

    lv_obj_set_align(ui_Slider1, LV_ALIGN_CENTER);

    lv_obj_set_style_radius(ui_Slider1, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Slider1, lv_color_hex(0xD0D7DF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Slider1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Slider1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Slider1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_Slider1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_Slider1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_Slider1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_Slider1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Slider1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Slider1, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Slider1, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Slider1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Slider1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Slider1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Slider1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Slider1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(ui_Slider1, 4, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Slider1, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Slider1, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Slider1, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Slider1, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_Slider1, &ui_img_img_slider_png, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_opa(ui_Slider1, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    // ui_Line_1

    ui_Line_1 = lv_obj_create(ui_Panel_Content);

    lv_obj_set_width(ui_Line_1, 10);
    lv_obj_set_height(ui_Line_1, 2);

    lv_obj_set_x(ui_Line_1, 0);
    lv_obj_set_y(ui_Line_1, 85);

    lv_obj_set_style_bg_color(ui_Line_1, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Line_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Line_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Line_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Line_1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Line_2

    ui_Line_2 = lv_obj_create(ui_Panel_Content);

    lv_obj_set_width(ui_Line_2, 10);
    lv_obj_set_height(ui_Line_2, 2);

    lv_obj_set_x(ui_Line_2, 0);
    lv_obj_set_y(ui_Line_2, 170);

    lv_obj_set_style_bg_color(ui_Line_2, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Line_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Line_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Line_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Line_2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Line_3

    ui_Line_3 = lv_obj_create(ui_Panel_Content);

    lv_obj_set_width(ui_Line_3, 10);
    lv_obj_set_height(ui_Line_3, 2);

    lv_obj_set_x(ui_Line_3, 37);
    lv_obj_set_y(ui_Line_3, 170);

    lv_obj_set_style_bg_color(ui_Line_3, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Line_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Line_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Line_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Line_3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Line_4

    ui_Line_4 = lv_obj_create(ui_Panel_Content);

    lv_obj_set_width(ui_Line_4, 10);
    lv_obj_set_height(ui_Line_4, 2);

    lv_obj_set_x(ui_Line_4, 37);
    lv_obj_set_y(ui_Line_4, 85);

    lv_obj_set_style_bg_color(ui_Line_4, lv_color_hex(0x132234), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Line_4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Line_4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Line_4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Line_4, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_Line_1

    ui_IMG_Line_1 = lv_img_create(ui_Panel_Content);
    lv_img_set_src(ui_IMG_Line_1, &ui_img_img_line_png);

    lv_obj_set_width(ui_IMG_Line_1, 224);
    lv_obj_set_height(ui_IMG_Line_1, 2);

    lv_obj_set_x(ui_IMG_Line_1, 15);
    lv_obj_set_y(ui_IMG_Line_1, 85);

    lv_obj_set_align(ui_IMG_Line_1, LV_ALIGN_TOP_RIGHT);

    lv_obj_add_flag(ui_IMG_Line_1, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Line_1, LV_OBJ_FLAG_SCROLLABLE);

    // ui_IMG_Line_2

    ui_IMG_Line_2 = lv_img_create(ui_Panel_Content);
    lv_img_set_src(ui_IMG_Line_2, &ui_img_img_line_png);

    lv_obj_set_width(ui_IMG_Line_2, 224);
    lv_obj_set_height(ui_IMG_Line_2, 2);

    lv_obj_set_x(ui_IMG_Line_2, 10);
    lv_obj_set_y(ui_IMG_Line_2, 170);

    lv_obj_set_align(ui_IMG_Line_2, LV_ALIGN_TOP_RIGHT);

    lv_obj_add_flag(ui_IMG_Line_2, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Line_2, LV_OBJ_FLAG_SCROLLABLE);

    // ui_IMG_Line_3

    ui_IMG_Line_3 = lv_img_create(ui_Panel_Content);
    lv_img_set_src(ui_IMG_Line_3, &ui_img_img_line_png);

    lv_obj_set_width(ui_IMG_Line_3, 224);
    lv_obj_set_height(ui_IMG_Line_3, 2);

    lv_obj_set_x(ui_IMG_Line_3, 10);
    lv_obj_set_y(ui_IMG_Line_3, 255);

    lv_obj_set_align(ui_IMG_Line_3, LV_ALIGN_TOP_RIGHT);

    lv_obj_add_flag(ui_IMG_Line_3, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Line_3, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Number_SYS

    ui_Number_SYS = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Number_SYS, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Number_SYS, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Number_SYS, 0);
    lv_obj_set_y(ui_Number_SYS, 10);

    lv_obj_set_align(ui_Number_SYS, LV_ALIGN_TOP_MID);

    lv_label_set_text(ui_Number_SYS, "132");

    lv_obj_set_style_text_color(ui_Number_SYS, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Number_SYS, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Number_SYS, &ui_font_Big_Number, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Number_DIA

    ui_Number_DIA = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Number_DIA, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Number_DIA, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Number_DIA, 0);
    lv_obj_set_y(ui_Number_DIA, 95);

    lv_obj_set_align(ui_Number_DIA, LV_ALIGN_TOP_MID);

    lv_label_set_text(ui_Number_DIA, "65");

    lv_obj_set_style_text_color(ui_Number_DIA, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Number_DIA, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Number_DIA, &ui_font_Big_Number, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Number_PUL

    ui_Number_PUL = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Number_PUL, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Number_PUL, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Number_PUL, 0);
    lv_obj_set_y(ui_Number_PUL, 180);

    lv_obj_set_align(ui_Number_PUL, LV_ALIGN_TOP_MID);

    lv_label_set_text(ui_Number_PUL, "72");

    lv_obj_set_style_text_color(ui_Number_PUL, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Number_PUL, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Number_PUL, &ui_font_Big_Number, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_SYS

    ui_Label_SYS = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Label_SYS, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_SYS, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_SYS, -15);
    lv_obj_set_y(ui_Label_SYS, 30);

    lv_obj_set_align(ui_Label_SYS, LV_ALIGN_TOP_RIGHT);

    lv_label_set_text(ui_Label_SYS, "SYS");

    lv_obj_set_style_text_color(ui_Label_SYS, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_SYS, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_SYS, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_mmHg_1

    ui_Label_mmHg_1 = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Label_mmHg_1, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_mmHg_1, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_mmHg_1, -15);
    lv_obj_set_y(ui_Label_mmHg_1, 49);

    lv_obj_set_align(ui_Label_mmHg_1, LV_ALIGN_TOP_RIGHT);

    lv_label_set_text(ui_Label_mmHg_1, "mmHg");

    lv_obj_set_style_text_color(ui_Label_mmHg_1, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_mmHg_1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_mmHg_1, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_DIA

    ui_Label_DIA = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Label_DIA, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_DIA, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_DIA, -15);
    lv_obj_set_y(ui_Label_DIA, 113);

    lv_obj_set_align(ui_Label_DIA, LV_ALIGN_TOP_RIGHT);

    lv_label_set_text(ui_Label_DIA, "DIA");

    lv_obj_set_style_text_color(ui_Label_DIA, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_DIA, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_DIA, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_mmHg_2

    ui_Label_mmHg_2 = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Label_mmHg_2, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_mmHg_2, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_mmHg_2, -15);
    lv_obj_set_y(ui_Label_mmHg_2, 133);

    lv_obj_set_align(ui_Label_mmHg_2, LV_ALIGN_TOP_RIGHT);

    lv_label_set_text(ui_Label_mmHg_2, "mmHg");

    lv_obj_set_style_text_color(ui_Label_mmHg_2, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_mmHg_2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_mmHg_2, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_PUL

    ui_Label_PUL = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Label_PUL, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_PUL, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_PUL, -15);
    lv_obj_set_y(ui_Label_PUL, 180);

    lv_obj_set_align(ui_Label_PUL, LV_ALIGN_TOP_RIGHT);

    lv_label_set_text(ui_Label_PUL, "PUL");

    lv_obj_set_style_text_color(ui_Label_PUL, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_PUL, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_PUL, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Label_mmHg_3

    ui_Label_mmHg_3 = lv_label_create(ui_Panel_Content);

    lv_obj_set_width(ui_Label_mmHg_3, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_mmHg_3, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_mmHg_3, -15);
    lv_obj_set_y(ui_Label_mmHg_3, 198);

    lv_obj_set_align(ui_Label_mmHg_3, LV_ALIGN_TOP_RIGHT);

    lv_label_set_text(ui_Label_mmHg_3, "mmHg");

    lv_obj_set_style_text_color(ui_Label_mmHg_3, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_mmHg_3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_mmHg_3, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_Heart

    ui_IMG_Heart = lv_img_create(ui_Panel_Content);
    lv_img_set_src(ui_IMG_Heart, &ui_img_img_heart_big_png);

    lv_obj_set_width(ui_IMG_Heart, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_Heart, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_Heart, -24);
    lv_obj_set_y(ui_IMG_Heart, 19);

    lv_obj_set_align(ui_IMG_Heart, LV_ALIGN_RIGHT_MID);

    lv_obj_add_flag(ui_IMG_Heart, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_Heart, LV_OBJ_FLAG_SCROLLABLE);

    // ui_Panel_Bottom

    ui_Panel_Bottom = lv_obj_create(ui_Panel_Content);

    lv_obj_set_height(ui_Panel_Bottom, 150);
    lv_obj_set_width(ui_Panel_Bottom, lv_pct(100));

    lv_obj_set_x(ui_Panel_Bottom, 0);
    lv_obj_set_y(ui_Panel_Bottom, -15);

    lv_obj_set_align(ui_Panel_Bottom, LV_ALIGN_BOTTOM_MID);

    lv_obj_clear_flag(ui_Panel_Bottom, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(ui_Panel_Bottom, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel_Bottom, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Panel_Bottom, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Panel_Bottom, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel_Bottom, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Bottom, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Bottom, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Bottom, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Bottom, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_IMG_BTN_Bg

    ui_IMG_BTN_Bg = lv_img_create(ui_Panel_Bottom);
    lv_img_set_src(ui_IMG_BTN_Bg, &ui_img_img_btn_bg_png);

    lv_obj_set_width(ui_IMG_BTN_Bg, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_IMG_BTN_Bg, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_IMG_BTN_Bg, 0);
    lv_obj_set_y(ui_IMG_BTN_Bg, 0);

    lv_obj_set_align(ui_IMG_BTN_Bg, LV_ALIGN_RIGHT_MID);

    lv_obj_add_flag(ui_IMG_BTN_Bg, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(ui_IMG_BTN_Bg, LV_OBJ_FLAG_SCROLLABLE);

    // ui_BTN_Power

    ui_BTN_Power = lv_obj_create(ui_IMG_BTN_Bg);

    lv_obj_set_width(ui_BTN_Power, 102);
    lv_obj_set_height(ui_BTN_Power, 102);

    lv_obj_set_x(ui_BTN_Power, 0);
    lv_obj_set_y(ui_BTN_Power, 0);

    lv_obj_set_align(ui_BTN_Power, LV_ALIGN_CENTER);

    lv_obj_add_flag(ui_BTN_Power, LV_OBJ_FLAG_CHECKABLE);

    lv_obj_add_event_cb(ui_BTN_Power, ui_event_BTN_Power, LV_EVENT_ALL, NULL);
    lv_obj_set_style_radius(ui_BTN_Power, 102, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_BTN_Power, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_BTN_Power, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BTN_Power, &ui_img_img_btn_off_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BTN_Power, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BTN_Power, &ui_img_img_btn_on_png, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_shadow_color(ui_BTN_Power, lv_color_hex(0x00A1FF), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_shadow_opa(ui_BTN_Power, 255, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_shadow_width(ui_BTN_Power, 30, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_shadow_spread(ui_BTN_Power, 0, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_img_src(ui_BTN_Power, &ui_img_img_btn_off_png, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_img_recolor(ui_BTN_Power, lv_color_hex(0x1690FA), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_bg_img_recolor_opa(ui_BTN_Power, 100, LV_PART_MAIN | LV_STATE_PRESSED);

    // ui_Switch_Sound

    ui_Switch_Sound = lv_switch_create(ui_Panel_Bottom);

    lv_obj_set_width(ui_Switch_Sound, 76);
    lv_obj_set_height(ui_Switch_Sound, 36);

    lv_obj_set_x(ui_Switch_Sound, 15);
    lv_obj_set_y(ui_Switch_Sound, 30);

    lv_obj_set_style_bg_img_src(ui_Switch_Sound, &ui_img_img_switch_bg_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Switch_Sound, 3, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_pad_right(ui_Switch_Sound, 3, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_pad_top(ui_Switch_Sound, 3, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_pad_bottom(ui_Switch_Sound, 3, LV_PART_MAIN | LV_STATE_CHECKED);

    lv_obj_set_style_bg_color(ui_Switch_Sound, lv_color_hex(0x00A1FF), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_Switch_Sound, 255, LV_PART_INDICATOR | LV_STATE_CHECKED);

    lv_obj_set_style_bg_color(ui_Switch_Sound, lv_color_hex(0x274264), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Switch_Sound, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Switch_Sound, -6, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Switch_Sound, -6, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Switch_Sound, -6, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Switch_Sound, -6, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Switch_Sound, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_Switch_Sound, 255, LV_PART_KNOB | LV_STATE_CHECKED);

    // ui_Label_Sound

    ui_Label_Sound = lv_label_create(ui_Panel_Bottom);

    lv_obj_set_width(ui_Label_Sound, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Sound, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Sound, 32);
    lv_obj_set_y(ui_Label_Sound, 12);

    lv_label_set_text(ui_Label_Sound, "Sound");

    lv_obj_set_style_text_color(ui_Label_Sound, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Sound, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label_Sound, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Sound, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ui_Switch_Bluetooth

    ui_Switch_Bluetooth = lv_switch_create(ui_Panel_Bottom);

    lv_obj_set_width(ui_Switch_Bluetooth, 76);
    lv_obj_set_height(ui_Switch_Bluetooth, 36);

    lv_obj_set_x(ui_Switch_Bluetooth, 15);
    lv_obj_set_y(ui_Switch_Bluetooth, 100);

    lv_obj_add_state(ui_Switch_Bluetooth, LV_STATE_CHECKED);

    lv_obj_add_event_cb(ui_Switch_Bluetooth, ui_event_Switch_Bluetooth, LV_EVENT_ALL, NULL);
    lv_obj_set_style_bg_img_src(ui_Switch_Bluetooth, &ui_img_img_switch_bg_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Switch_Bluetooth, 3, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_pad_right(ui_Switch_Bluetooth, 3, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_pad_top(ui_Switch_Bluetooth, 3, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_pad_bottom(ui_Switch_Bluetooth, 3, LV_PART_MAIN | LV_STATE_CHECKED);

    lv_obj_set_style_bg_color(ui_Switch_Bluetooth, lv_color_hex(0x00A1FF), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_Switch_Bluetooth, 255, LV_PART_INDICATOR | LV_STATE_CHECKED);

    lv_obj_set_style_bg_color(ui_Switch_Bluetooth, lv_color_hex(0x274264), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Switch_Bluetooth, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Switch_Bluetooth, -6, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Switch_Bluetooth, -6, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Switch_Bluetooth, -6, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Switch_Bluetooth, -6, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Switch_Bluetooth, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_Switch_Bluetooth, 255, LV_PART_KNOB | LV_STATE_CHECKED);

    // ui_Label_Bluetooth

    ui_Label_Bluetooth = lv_label_create(ui_Panel_Bottom);

    lv_obj_set_width(ui_Label_Bluetooth, LV_SIZE_CONTENT);
    lv_obj_set_height(ui_Label_Bluetooth, LV_SIZE_CONTENT);

    lv_obj_set_x(ui_Label_Bluetooth, 22);
    lv_obj_set_y(ui_Label_Bluetooth, 81);

    lv_label_set_text(ui_Label_Bluetooth, "Bluetooth");

    lv_obj_set_style_text_color(ui_Label_Bluetooth, lv_color_hex(0x1D3450), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Bluetooth, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label_Bluetooth, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Bluetooth, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

}

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Medical_Screen_screen_init();
    lv_disp_load_scr(ui_Medical_Screen);
}

