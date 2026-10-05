#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_MAIN = 1,
    SCREEN_ID_MENU_PRINCIPAL = 2,
    SCREEN_ID_MENU_REMOTO = 3,
    SCREEN_ID_MENU_LOCAL_JERINGA = 4,
    SCREEN_ID_MENU_LOCAL_MODO = 5,
    SCREEN_ID_MENU_LOCAL_PARAMETROS = 6,
    SCREEN_ID_MENU_LOCAL_PARAMETROS_TEST = 7,
    SCREEN_ID_MENU_LOCAL_PARAMETROS_CONFIRM = 8,
    SCREEN_ID_MENU_LOCAL_TIEMPO = 9,
    _SCREEN_ID_LAST = 9
};

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *menu_principal;
    lv_obj_t *menu_remoto;
    lv_obj_t *menu_local_jeringa;
    lv_obj_t *menu_local_modo;
    lv_obj_t *menu_local_parametros;
    lv_obj_t *menu_local_parametros_test;
    lv_obj_t *menu_local_parametros_confirm;
    lv_obj_t *menu_local_tiempo;
    lv_obj_t *obj0;
    lv_obj_t *obj0__obj0;
    lv_obj_t *obj1;
    lv_obj_t *obj1__obj0;
    lv_obj_t *btn_inicio;
    lv_obj_t *obj2;
    lv_obj_t *obj3;
    lv_obj_t *obj4;
    lv_obj_t *obj5;
    lv_obj_t *obj6;
    lv_obj_t *obj7;
    lv_obj_t *btn_inicio_1;
    lv_obj_t *btn_inicio_2;
    lv_obj_t *obj8;
    lv_obj_t *obj9;
    lv_obj_t *obj10;
    lv_obj_t *obj11;
    lv_obj_t *obj12;
    lv_obj_t *obj13;
    lv_obj_t *obj14;
    lv_obj_t *obj15;
    lv_obj_t *obj16;
    lv_obj_t *obj17;
    lv_obj_t *menu_modo_1;
    lv_obj_t *obj18;
    lv_obj_t *obj19;
    lv_obj_t *menu_modo;
    lv_obj_t *obj20;
    lv_obj_t *obj21;
    lv_obj_t *obj22;
    lv_obj_t *obj23;
    lv_obj_t *obj24;
    lv_obj_t *obj25;
    lv_obj_t *obj26;
    lv_obj_t *obj27;
    lv_obj_t *obj28;
    lv_obj_t *obj29;
    lv_obj_t *container_caudal;
    lv_obj_t *obj30;
    lv_obj_t *obj31;
    lv_obj_t *container_tiempo;
    lv_obj_t *obj32;
    lv_obj_t *obj33;
    lv_obj_t *container_volumen;
    lv_obj_t *obj34;
    lv_obj_t *obj35;
    lv_obj_t *teclado_num;
    lv_obj_t *obj36;
    lv_obj_t *obj37;
    lv_obj_t *obj38;
    lv_obj_t *obj39;
    lv_obj_t *obj40;
    lv_obj_t *obj41;
    lv_obj_t *obj42;
    lv_obj_t *obj43;
    lv_obj_t *obj44;
    lv_obj_t *obj45;
    lv_obj_t *obj46;
    lv_obj_t *obj47;
    lv_obj_t *obj48;
    lv_obj_t *obj49;
    lv_obj_t *obj50;
    lv_obj_t *obj51;
    lv_obj_t *obj52;
    lv_obj_t *obj53;
    lv_obj_t *obj54;
    lv_obj_t *obj55;
    lv_obj_t *obj56;
    lv_obj_t *container_caudal_1;
    lv_obj_t *obj57;
    lv_obj_t *obj58;
    lv_obj_t *teclado_num_1;
    lv_obj_t *obj59;
    lv_obj_t *obj60;
    lv_obj_t *obj61;
    lv_obj_t *obj62;
    lv_obj_t *obj63;
    lv_obj_t *obj64;
    lv_obj_t *obj65;
    lv_obj_t *obj66;
    lv_obj_t *obj67;
    lv_obj_t *obj68;
    lv_obj_t *obj69;
    lv_obj_t *obj70;
    lv_obj_t *obj71;
    lv_obj_t *obj72;
    lv_obj_t *obj73;
    lv_obj_t *bar_infusion;
    lv_obj_t *obj74;
    lv_obj_t *obj75;
    lv_obj_t *obj76;
    lv_obj_t *obj77;
    lv_obj_t *obj78;
    lv_obj_t *obj79;
    lv_obj_t *obj80;
    lv_obj_t *obj81;
} objects_t;

extern objects_t objects;

void create_screen_main();
void tick_screen_main();

void create_screen_menu_principal();
void tick_screen_menu_principal();

void create_screen_menu_remoto();
void tick_screen_menu_remoto();

void create_screen_menu_local_jeringa();
void tick_screen_menu_local_jeringa();

void create_screen_menu_local_modo();
void tick_screen_menu_local_modo();

void create_screen_menu_local_parametros();
void tick_screen_menu_local_parametros();

void create_screen_menu_local_parametros_test();
void tick_screen_menu_local_parametros_test();

void create_screen_menu_local_parametros_confirm();
void tick_screen_menu_local_parametros_confirm();

void create_screen_menu_local_tiempo();
void tick_screen_menu_local_tiempo();

void create_user_widget_battery(lv_obj_t *parent_obj, void *flowState, int startWidgetIndex);
void tick_user_widget_battery(void *flowState, int startWidgetIndex);

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/