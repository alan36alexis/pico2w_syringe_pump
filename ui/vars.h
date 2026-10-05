#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations

typedef enum {
    modo_infusion_Continuo = 0,
    modo_infusion_Bolo = 1,
    modo_infusion_KVO = 2,
    modo_infusion_Purga = 3,
    modo_infusion_Intermitente = 4,
    modo_infusion_Tiempo = 5
} modo_infusion;

typedef enum {
    input_target_t_INPUT_NONE = 0,
    input_target_t_INPUT_CAUDAL = 1,
    input_target_t_INPUT_VOLUMEN = 2,
    input_target_t_INPUT_TIEMPO = 3,
    input_target_t_INPUT_MICROMETROS = 4
} input_target_t;

// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_NONE
};

// Native global variables

extern int32_t get_var_tipo_jeringa();
extern void set_var_tipo_jeringa(int32_t value);
extern const char *get_var_str_modo_seleccionado();
extern void set_var_str_modo_seleccionado(const char *value);
extern const char *get_var_str_tipo_jeringa();
extern void set_var_str_tipo_jeringa(const char *value);
extern float get_var_set_caudal();
extern void set_var_set_caudal(float value);
extern int32_t get_var_metodo_seleccionado();
extern void set_var_metodo_seleccionado(int32_t value);
extern int32_t get_var_modo_seleccionado();
extern void set_var_modo_seleccionado(int32_t value);
extern float get_var_set_volumen();
extern void set_var_set_volumen(float value);
extern float get_var_set_tiempo();
extern void set_var_set_tiempo(float value);
extern const char *get_var_set_teclado_num();
extern void set_var_set_teclado_num(const char *value);
extern const char *get_var_str_unidad();
extern void set_var_str_unidad(const char *value);
extern float get_var_value_infusion_test();
extern void set_var_value_infusion_test(float value);
extern const char *get_var_str_metodo_seleccionado();
extern void set_var_str_metodo_seleccionado(const char *value);
extern int32_t get_var_set_micrometros();
extern void set_var_set_micrometros(int32_t value);
extern int32_t get_var_value_battery_charge();
extern void set_var_value_battery_charge(int32_t value);

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/