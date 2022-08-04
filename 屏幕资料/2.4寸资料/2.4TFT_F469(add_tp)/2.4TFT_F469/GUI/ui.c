//// SquareLine LVGL GENERATED FILE
//// EDITOR VERSION: SquareLine Studio 1.0.5
//// LVGL VERSION: 8.2
//// PROJECT: 2.4tft

//#include "ui.h"
//#include "ui_helpers.h"

/////////////////////// VARIABLES ////////////////////
//lv_obj_t * ui_Screen1;
//lv_obj_t * ui_Image2;

///////////////////////// TEST LVGL SETTINGS ////////////////////
////#if LV_COLOR_DEPTH != 16
////    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
////#endif
////#if LV_COLOR_16_SWAP !=0
////    #error "#error LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
////#endif

/////////////////////// ANIMATIONS ////////////////////

/////////////////////// FUNCTIONS ////////////////////

/////////////////////// SCREENS ////////////////////
//void ui_Screen1_screen_init(void)
//{

//    // ui_Screen1

//    ui_Screen1 = lv_obj_create(NULL);

//    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);

//    // ui_Image2

//    ui_Image2 = lv_img_create(ui_Screen1);
//    lv_img_set_src(ui_Image2, &ui_img_520_png);

//    lv_obj_set_width(ui_Image2, LV_SIZE_CONTENT);
//    lv_obj_set_height(ui_Image2, LV_SIZE_CONTENT);

//    lv_obj_set_x(ui_Image2, 0);
//    lv_obj_set_y(ui_Image2, 0);

//    lv_obj_set_align(ui_Image2, LV_ALIGN_CENTER);

//    lv_obj_add_flag(ui_Image2, LV_OBJ_FLAG_ADV_HITTEST);
//    lv_obj_clear_flag(ui_Image2, LV_OBJ_FLAG_SCROLLABLE);

//}

//void ui_init(void)
//{
//    lv_disp_t * dispp = lv_disp_get_default();
//    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
//                                               false, LV_FONT_DEFAULT);
//    lv_disp_set_theme(dispp, theme);
//    ui_Screen1_screen_init();
//    lv_disp_load_scr(ui_Screen1);
//}

