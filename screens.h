#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *move_steps_page;
    lv_obj_t *manual_jog_page;
    lv_obj_t *continuous_rotation_page;
    lv_obj_t *header;
    lv_obj_t *menu;
    lv_obj_t *move_steps;
    lv_obj_t *manual_jog;
    lv_obj_t *continuous;
    lv_obj_t *header_1;
    lv_obj_t *back;
    lv_obj_t *start;
    lv_obj_t *step_num;
    lv_obj_t *clockwise;
    lv_obj_t *speed;
    lv_obj_t *header_2;
    lv_obj_t *back_1;
    lv_obj_t *start_1;
    lv_obj_t *speed_manual_jog;
    lv_obj_t *header_3;
    lv_obj_t *back_2;
    lv_obj_t *continuous_rotation_start_button;
    lv_obj_t *continuous_rotation_speed_button;
    lv_obj_t *continuous_rotation_direction_button;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN = 1,
    SCREEN_ID_MOVE_STEPS_PAGE = 2,
    SCREEN_ID_MANUAL_JOG_PAGE = 3,
    SCREEN_ID_CONTINUOUS_ROTATION_PAGE = 4,
};

void create_screen_main();
void tick_screen_main();

void create_screen_move_steps_page();
void tick_screen_move_steps_page();

void create_screen_manual_jog_page();
void tick_screen_manual_jog_page();

void create_screen_continuous_rotation_page();
void tick_screen_continuous_rotation_page();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/