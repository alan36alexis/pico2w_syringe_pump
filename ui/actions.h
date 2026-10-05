#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void action_update_label_modo(lv_event_t * e);
extern void action_update_modo_dropdown(lv_event_t * e);
extern void action_update_tipo_jeringa(lv_event_t * e);
extern void action_select_metodo(lv_event_t * e);
extern void action_btn_set_caudal(lv_event_t * e);
extern void action_btn_set_volumen(lv_event_t * e);
extern void action_btn_set_tiempo(lv_event_t * e);
extern void action_teclado_num_cancel(lv_event_t * e);
extern void action_teclado_num_ok(lv_event_t * e);
extern void action_update_teclado_num(lv_event_t * e);
extern void action_stop_infusion(lv_event_t * e);
extern void action_reanudar_infusion(lv_event_t * e);
extern void action_btn_set_micrometros(lv_event_t * e);
extern void action_btn_home(lv_event_t * e);
extern void action_btn_search(lv_event_t * e);

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_EVENTS_H*/