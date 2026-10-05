#include "lvgl.h"
#include "ui.h"   // importante para acceder a variables de EEZ
#include "vars.h"
#include "ui/screens.h"
#include "actions.h"
#include "cmd_gate.h"
#include "eez-flow.h"
#include "cmd_dispatcher.h"
#include "system_config.h"
#include "system_queues.h"
#include "syringe_pump_api.h"
#include <stdio.h>
#include <stdlib.h>
#define HOME_RESET_TIMEOUT_MS 65000U
#define HOME_STATE_POLL_MS 10U

static lv_timer_t *home_sequence_timer;
static uint16_t home_sequence_wait_ms;
static void home_sequence_wait_for_unhomed(lv_timer_t *timer);
#include <string.h>

typedef enum {
    MODO_NULL = -1,
    MODO_CONTINUO = 0,
    MODO_BOLO = 1,
    MODO_KVO = 2,
    MODO_PURGA = 3,
    MODO_INTERMITENTE = 4,
    MODO_POR_TIEMPO = 5
} modo_t;




/***************VARIABLES*******************/

input_target_t input_activo = input_target_t_INPUT_NONE;
modo_t selected_mode = MODO_NULL;
char modo_string[100] = { 0 };
int32_t modo_seleccionado;
int32_t metodo_seleccionado;
int32_t tipo_jeringa;
int32_t set_micrometros;
static bool manual_micrometers_target_selected;
char str_modo_seleccionado[100] = { 0 };
char str_tipo_jeringa[100] = { 0 };
char str_metodo_seleccionado[100] = { 0 };
float value_infusion_test;
static int32_t value_battery_charge = 100;
static lv_timer_t *battery_simulation_timer;

float set_caudal, set_tiempo, set_volumen;
char set_teclado_num[32] = { 0 };
char str_unidad[100] = { 0 };

#define INFUSION_TEST_TARGET_UM 1000.0f
#define INFUSION_TEST_SPEED_UMS 2500.0f
#define UI_HOME_VELOCITY_UMS 4000.0f
#define INFUSION_UI_SIMULATION 0     // Set to 1 to enable infusion test simulation, 0 to disable
#define BATTERY_SIMULATION_INTERVAL_MS 200U
#define MIN_INFUSION_RATE_ML_H 0.01f
#define MAX_INFUSION_RATE_ML_H 2000.0f

typedef struct {
    float capacity_ml;
    float area_mm2;
} SyringeUiProfile_t;

static const SyringeUiProfile_t syringe_ui_profiles[] = {
    { 1.0f, 17.35f },
    { 2.0f, 58.77f },
    { 3.0f, 57.68f },
    { 5.0f, 112.72f },
    { 10.0f, 163.54f },
    { 20.0f, 282.63f },
    { 30.0f, 362.38f },
    { 50.0f, 549.88f },
    { 60.0f, 555.72f },
};

static void ui_report_dispatch_debug(const char *message) {
    DtoDebugStr_t dto = {0};
    snprintf(dto.buf, sizeof(dto.buf), "[UI] %s", message);
    CORE0_EMIT(EV_DBG_STRING, dbg_str, dto);
}

static bool ui_calculate_dispense_motion(float *target_um, float *velocity_ums) {
    if (!target_um || !velocity_ums || modo_seleccionado < MODO_CONTINUO ||
        modo_seleccionado > MODO_POR_TIEMPO) {
        return false;
    }

    if (manual_micrometers_target_selected) {
        if (set_micrometros <= 0 || set_micrometros > 105000) return false;
        *target_um = (float)set_micrometros;
        *velocity_ums = INFUSION_TEST_SPEED_UMS;
        return true;
    }

    const size_t profile_count =
        sizeof(syringe_ui_profiles) / sizeof(syringe_ui_profiles[0]);
    if (tipo_jeringa < 0 || (size_t)tipo_jeringa >= profile_count) {
        return false;
    }

    const SyringeUiProfile_t *profile = &syringe_ui_profiles[tipo_jeringa];
    float volume_ml = set_volumen;
    float rate_ml_h = 0.0f;

    switch (modo_seleccionado) {
    case MODO_CONTINUO:
    case MODO_BOLO:
    case MODO_INTERMITENTE:
        if (metodo_seleccionado == 0) {
            rate_ml_h = set_caudal;
        } else if (metodo_seleccionado == 1) {
            if (set_tiempo <= 0.0f) return false;
            rate_ml_h = volume_ml * 3600.0f / set_tiempo;
        } else {
            return false;
        }
        break;

    case MODO_POR_TIEMPO:
        if (metodo_seleccionado != 1 || set_tiempo <= 0.0f) return false;
        rate_ml_h = volume_ml * 3600.0f / set_tiempo;
        break;

    case MODO_KVO:
        rate_ml_h = KVO_FLOW_RATE_MLH;
        if (volume_ml <= 0.0f) volume_ml = profile->capacity_ml;
        break;

    case MODO_PURGA:
        volume_ml = PURGE_VOLUME_ML;
        rate_ml_h = PURGE_FLOW_RATE_MLH;
        break;

    default:
        return false;
    }

    if (volume_ml <= 0.0f || volume_ml > profile->capacity_ml ||
        rate_ml_h < MIN_INFUSION_RATE_ML_H || rate_ml_h > MAX_INFUSION_RATE_ML_H) {
        return false;
    }

    *target_um = Pump_MlToUm(volume_ml, profile->area_mm2);
    if (metodo_seleccionado == 1 && modo_seleccionado != MODO_PURGA &&
        modo_seleccionado != MODO_KVO) {
        *velocity_ums = Pump_MlPerSecondToUmPerSecond(
            volume_ml / set_tiempo, profile->area_mm2);
    } else {
        *velocity_ums = Pump_MlPerHourToUmPerSecond(
            rate_ml_h, profile->area_mm2);
    }

    return *target_um > 0.0f && *velocity_ums > 0.0f;
}

#if INFUSION_UI_SIMULATION
static lv_timer_t *infusion_test_timer;

static void infusion_test_timer_cb(lv_timer_t *timer) {
    (void)timer;
    float progress = get_var_value_infusion_test() + 1.0f;
    if (progress >= 100.0f) {
        progress = 100.0f;
        lv_timer_pause(infusion_test_timer);
    }
    set_var_value_infusion_test(progress);
}
#endif

void actualizar_seteo_por_metodo(int metodo);
void teclado_add_char(const char *c);
void teclado_del();

static void battery_simulation_timer_cb(lv_timer_t *timer) {
    (void)timer;
    int32_t charge = get_var_value_battery_charge();
    set_var_value_battery_charge(charge <= 0 ? 100 : charge - 1);
}

static void battery_simulation_start_if_needed(void) {
    if (!battery_simulation_timer) {
        battery_simulation_timer = lv_timer_create(
            battery_simulation_timer_cb, BATTERY_SIMULATION_INTERVAL_MS, NULL);
    }
}

/* ------ Funciones para obtener y establecer variables globales    ------ */
int32_t get_var_modo_seleccionado() {
    return modo_seleccionado;
}

void set_var_modo_seleccionado(int32_t value) {
    modo_seleccionado = value;
}

int32_t get_var_tipo_jeringa() {
    return tipo_jeringa;
}

void set_var_tipo_jeringa(int32_t value) {
    tipo_jeringa = value;
}

int32_t get_var_metodo_seleccionado() {
    return metodo_seleccionado;
}

void set_var_metodo_seleccionado(int32_t value) {
    metodo_seleccionado = value;
    switch (value) {
        case 0:
            set_var_str_metodo_seleccionado("Caudal");
            break;
        case 1:
            set_var_str_metodo_seleccionado("Volumen y tiempo");
            break;
        default:
            set_var_str_metodo_seleccionado("");
            break;
    }
}

float get_var_set_caudal() {
    return set_caudal;
}

void set_var_set_caudal(float value) {
    set_caudal = value;
}

float get_var_set_tiempo() {
    return set_tiempo;
}

void set_var_set_tiempo(float value) {
    set_tiempo = value;
}

float get_var_set_volumen() {
    return set_volumen;
}

void set_var_set_volumen(float value) {
    set_volumen = value;
}

const char *get_var_set_teclado_num() {
    return set_teclado_num;
}

int32_t get_var_set_micrometros() {
    return set_micrometros;
}

void set_var_set_micrometros(int32_t value) {
    set_micrometros = value;
}

void set_var_set_teclado_num(const char *value) {
    if (value) {
        strncpy(set_teclado_num, value, sizeof(set_teclado_num) - 1);
        set_teclado_num[sizeof(set_teclado_num) - 1] = '\0';
    } else {
        set_teclado_num[0] = '\0';
    }
}

const char *get_var_str_modo_seleccionado() {
    return str_modo_seleccionado;
}

void set_var_str_modo_seleccionado(const char *value) {
    if (value) {
        strncpy(str_modo_seleccionado, value, sizeof(str_modo_seleccionado) - 1);
        str_modo_seleccionado[sizeof(str_modo_seleccionado) - 1] = '\0';
    }
}

const char *get_var_str_tipo_jeringa() {
    return str_tipo_jeringa;
}

void set_var_str_tipo_jeringa(const char *value) {
    if (value) {
        strncpy(str_tipo_jeringa, value, sizeof(str_tipo_jeringa) - 1);
        str_tipo_jeringa[sizeof(str_tipo_jeringa) - 1] = '\0';
    }
}

const char *get_var_str_unidad() {
    return str_unidad;
}

void set_var_str_unidad(const char *value) {
    if (value) {
        strncpy(str_unidad, value, sizeof(str_unidad) - 1);
        str_unidad[sizeof(str_unidad) - 1] = '\0';
    }
}

float get_var_value_infusion_test() {
    return value_infusion_test;
}

void set_var_value_infusion_test(float value) {
    value_infusion_test = value;
}

int32_t get_var_value_battery_charge() {
    battery_simulation_start_if_needed();
    return value_battery_charge;
}

void set_var_value_battery_charge(int32_t value) {
    if (value < 0) value = 0;
    if (value > 100) value = 100;
    value_battery_charge = value;
}

const char *get_var_str_metodo_seleccionado() {
    return str_metodo_seleccionado;
}

void set_var_str_metodo_seleccionado(const char *value) {
    if (!value) {
        str_metodo_seleccionado[0] = '\0';
        return;
    }

    strncpy(str_metodo_seleccionado, value, sizeof(str_metodo_seleccionado) - 1);
    str_metodo_seleccionado[sizeof(str_metodo_seleccionado) / sizeof(char) - 1] = 0;
}



/* void tipo_modo_handler(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);

    selected_mode = lv_btnmatrix_get_selected_btn(obj);
}
 */

 /*************** FUNCIONES *******************/

const char *get_var_modo_string() {
    switch(modo_seleccionado)
    {
        case MODO_CONTINUO: return "Continuo";
        case MODO_BOLO: return "Bolo";
        case MODO_KVO: return "KVO";
        case MODO_PURGA: return "Purga";
        case MODO_INTERMITENTE: return "Intermitente";
        case MODO_POR_TIEMPO: return "Por Tiempo";
        default: return "-";
    }
}


const char *jeringa_index_to_string(int32_t index)
{
    switch(index)
    {
        case 0: return "1 mL";
        case 1: return "2 mL";
        case 2: return "3 mL";
        case 3: return "5mL";
        case 4: return "10 mL";
        case 5: return "20 mL";
        case 6: return "30 mL";
        case 7: return "50 mL";
        case 8: return "60 mL";
        default: return "-";
    }
}


/**************** EVENT HANDLERS *****************/

void action_update_label_modo(lv_event_t * e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    lv_obj_t * obj = lv_event_get_target(e);
    void *flowState = lv_event_get_user_data(e);
    int selected_btn = lv_btnmatrix_get_selected_btn(obj);

    if (selected_btn < 0) {
        return;
    }

    selected_mode = (modo_t) selected_btn;
    set_var_modo_seleccionado((int32_t)selected_mode);

    const char *btn_text = lv_btnmatrix_get_btn_text(obj, selected_btn);
    /* if (btn_text == NULL) {
        btn_text = modo_to_string(selected_mode);
    } */

    // Actualizar la variable de cadena con el texto del botón
    set_var_str_modo_seleccionado(btn_text);
}


void action_update_modo_dropdown(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    lv_obj_t * obj = lv_event_get_target(e);
    void *flowState = lv_event_get_user_data(e);
    uint16_t selected_idx = lv_dropdown_get_selected(obj);

    selected_mode = (modo_t) selected_idx;
    set_var_modo_seleccionado((int32_t)selected_mode);

    //const char *selected_text = modo_to_string(selected_mode);
}

void action_select_metodo(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    lv_obj_t *obj = lv_event_get_target(e);
    if (obj == NULL) {
        return;
    }

    int selected_idx = lv_dropdown_get_selected(obj);
    if (selected_idx < 0) {
        return;
    }

    set_var_metodo_seleccionado((int32_t)selected_idx);

    actualizar_seteo_por_metodo(selected_idx);
}

void action_update_tipo_jeringa(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    lv_obj_t * obj = lv_event_get_target(e);
    void *flowState = lv_event_get_user_data(e);
    int selected_btn = lv_btnmatrix_get_selected_btn(obj);

    if (selected_btn < 0) {
        return;
    }

    set_var_tipo_jeringa((int32_t)selected_btn);

    const char *btn_text = lv_btnmatrix_get_btn_text(obj, selected_btn);
    if (btn_text == NULL) {
        btn_text = jeringa_index_to_string((int32_t)selected_btn);
    }

    // Actualizar la variable de cadena con el texto del botón
    set_var_str_tipo_jeringa(btn_text);
}

// FUNCIONES SET

void action_btn_set_caudal(lv_event_t *e)
{
    manual_micrometers_target_selected = false;
    input_activo = input_target_t_INPUT_CAUDAL;
    lv_obj_clear_flag(objects.teclado_num, LV_OBJ_FLAG_HIDDEN);
    //abrir_teclado();
}

void action_btn_set_tiempo(lv_event_t *e)
{
    manual_micrometers_target_selected = false;
    input_activo = input_target_t_INPUT_TIEMPO;
    lv_obj_clear_flag(objects.teclado_num, LV_OBJ_FLAG_HIDDEN);
    //abrir_teclado();
}

void action_btn_set_volumen(lv_event_t *e)
{
    manual_micrometers_target_selected = false;
    input_activo = input_target_t_INPUT_VOLUMEN;
    lv_obj_clear_flag(objects.teclado_num, LV_OBJ_FLAG_HIDDEN);
    //abrir_teclado();
}

void action_btn_set_micrometros(lv_event_t *e) {
    // TODO: Implement action btn_set_micrometros here
    input_activo = input_target_t_INPUT_MICROMETROS;
    lv_obj_clear_flag(objects.teclado_num_1, LV_OBJ_FLAG_HIDDEN);
}

void action_teclado_num_cancel(lv_event_t *e)
{
    // limpiar teclado
    set_var_set_teclado_num("");
    lv_obj_add_flag(objects.teclado_num, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(objects.teclado_num_1, LV_OBJ_FLAG_HIDDEN);
}

void action_teclado_num_ok(lv_event_t *e)
{
    //float valor = atof(buffer_teclado);
    const char *txt = get_var_set_teclado_num();
    char tmp[32];

    if (txt == NULL) {
        txt = "";
    }

    strncpy(tmp, txt, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    for (char *p = tmp; *p; ++p) {
        if (*p == ',') {
            *p = '.';
        }
    }

    float valor = atof(tmp);

    switch(input_activo)
    {
        case input_target_t_INPUT_CAUDAL:
            set_var_set_caudal(valor);
            break;

        case input_target_t_INPUT_VOLUMEN:
            set_var_set_volumen(valor);
            break;

        case input_target_t_INPUT_TIEMPO:
            set_var_set_tiempo(valor);
            break;

        case input_target_t_INPUT_MICROMETROS:
            if (valor > 0.0f && valor <= 105000.0f) {
                set_var_set_micrometros((int32_t)valor);
                manual_micrometers_target_selected = true;
            } else {
                manual_micrometers_target_selected = false;
            }
            break;

        default:
            break;
    }
    // limpiar teclado
    set_var_set_teclado_num("");

    //limpiar_buffer();
    lv_obj_add_flag(objects.teclado_num, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(objects.teclado_num_1, LV_OBJ_FLAG_HIDDEN);
    // actualizar UI (por si no es automático)
    //actualizar_valores_ui();

}

void action_update_teclado_num(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }

    lv_obj_t *obj = lv_event_get_target(e);
    if (obj == NULL) {
        return;
    }

    int selected_btn = lv_btnmatrix_get_selected_btn(obj);
    if (selected_btn < 0) {
        return;
    }

    const char *btn_txt = lv_btnmatrix_get_btn_text(obj, selected_btn);
    if (btn_txt == NULL) {
        return;
    }

    if (strcmp(btn_txt, "DEL") == 0) {
        teclado_del();
        return;
    }

    if (strcmp(btn_txt, ",") == 0 || strcmp(btn_txt, ".") == 0 || (btn_txt[0] >= '0' && btn_txt[0] <= '9' && btn_txt[1] == '\0')) {
        teclado_add_char(btn_txt);
    }
}



void action_reanudar_infusion(lv_event_t *e) {
    (void)e;
#if INFUSION_UI_SIMULATION
    if (!infusion_test_timer) {
        infusion_test_timer = lv_timer_create(infusion_test_timer_cb, 100, NULL);
    } else {
        lv_timer_resume(infusion_test_timer);
    }
/* #else
    char command[48];
    snprintf(command, sizeof(command), "fsm_dispense,%.1f,%.1f",
             INFUSION_TEST_TARGET_UM, INFUSION_TEST_SPEED_UMS);
    CmdDispatchResult_t result =
        cmd_dispatch_string(command, CMD_SRC_HMI, -1);
    if (result.accepted) {
        set_var_value_infusion_test(0.0f);
    }
#endif */
#else
    float target_um = 0.0f;
    float velocity_ums = 0.0f;
    if (!ui_calculate_dispense_motion(&target_um, &velocity_ums)) {
        char diagnostic[56];
        float requested_rate_ml_h = set_tiempo > 0.0f
            ? set_volumen * 3600.0f / set_tiempo
            : 0.0f;
        snprintf(diagnostic, sizeof(diagnostic),
                 "bad config mode=%ld method=%ld syringe=%ld V=%.1f t=%.1f q=%.0f",
                 (long)modo_seleccionado, (long)metodo_seleccionado,
                 (long)tipo_jeringa, set_volumen, set_tiempo,
                 requested_rate_ml_h);
        ui_report_dispatch_debug(diagnostic);
        return;
    }

    char command[64];
    snprintf(command, sizeof(command), "fsm_dispense,%.1f,%.1f",
             target_um, velocity_ums);
    char diagnostic[56];
    snprintf(diagnostic, sizeof(diagnostic), "fsm_dispense target=%.1f um vel=%.1f um/s",
             target_um, velocity_ums);
    ui_report_dispatch_debug(diagnostic);
    CmdDispatchResult_t result =
        cmd_dispatch_string(command, CMD_SRC_HMI, -1);
    if (result.accepted) {
        set_var_value_infusion_test(0.0f);
        ui_report_dispatch_debug("dispense command accepted");
    } else {
        char diagnostic[56];
        snprintf(diagnostic, sizeof(diagnostic), "dispense rejected: %.40s",
                 result.reason ? result.reason : "unknown");
        ui_report_dispatch_debug(diagnostic);
    }
#endif
}

void action_stop_infusion(lv_event_t *e) {
    (void)e;
#if INFUSION_UI_SIMULATION
    if (infusion_test_timer) {
        lv_timer_pause(infusion_test_timer);
    }
#else
    (void)cmd_dispatch_string("stop", CMD_SRC_HMI, -1);
#endif
}

/**************** FUNCIONES INDEPENDIENTES *****************/

void actualizar_seteo_por_metodo(int metodo)
{
    // Metodo por CAUDAL
    if(metodo == 0)
    {
        lv_obj_clear_flag(objects.container_caudal, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(objects.container_volumen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(objects.container_tiempo, LV_OBJ_FLAG_HIDDEN);
    }
    // Metodo por VOLUMEN y TIEMPO
    else
    {
        lv_obj_add_flag(objects.container_caudal, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(objects.container_volumen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(objects.container_tiempo, LV_OBJ_FLAG_HIDDEN);
    }
}

void teclado_add_char(const char *c)
{
    const char *actual = get_var_set_teclado_num();

    char nuevo[32];
    snprintf(nuevo, sizeof(nuevo), "%s%s", actual, c);

    set_var_set_teclado_num(nuevo);
}

void teclado_del()
{
    char buffer[32];
    strcpy(buffer, get_var_set_teclado_num());

    int len = strlen(buffer);
    if(len > 0)
    {
        buffer[len-1] = '\0';
        set_var_set_teclado_num(buffer);
    }
}

/* Espera el estado publicado por el broker tras stop_imm antes de pedir HOME. */
static void home_sequence_wait_for_unhomed(lv_timer_t *timer) {
    Core1State_t state = cmd_gate_get_fsm_state();
    if (state == ST_UNHOMED || state == ST_READY_AT_HOME) {
        lv_timer_del(timer);
        home_sequence_timer = NULL;

        char command[32];
        snprintf(command, sizeof(command), "fsm_home,%.1f", UI_HOME_VELOCITY_UMS);
        CmdDispatchResult_t result = cmd_dispatch_string(command, CMD_SRC_HMI, -1);
        if (result.accepted) {
            set_var_value_infusion_test(0.0f);
        }
        return;
    }

    // Incrementa el tiempo de espera y verifica si se ha superado el tiempo máximo permitido
    home_sequence_wait_ms += HOME_STATE_POLL_MS;
    if (home_sequence_wait_ms >= HOME_RESET_TIMEOUT_MS) {
        lv_timer_del(timer);
        home_sequence_timer = NULL;
    }
}

/* Resetea/parada primero y programa el homing cuando Core 1 confirme reposo. */
void action_btn_home(lv_event_t *e) {
    (void)e;
    if (home_sequence_timer) {
        lv_timer_del(home_sequence_timer);
        home_sequence_timer = NULL;
    }

    CmdDispatchResult_t result =
        cmd_dispatch_string("stop_imm", CMD_SRC_HMI, -1);
    if (!result.accepted) return;

    home_sequence_wait_ms = 0;
    home_sequence_timer = lv_timer_create (home_sequence_wait_for_unhomed, HOME_STATE_POLL_MS, NULL);
}

void action_btn_search(lv_event_t *e) {
    (void)e;
    char command[32];
    // fsm_search a velocidad FSM_SEARCH_VELOCITY_UMS
    snprintf(command, sizeof(command), "fsm_search,%.1f", FSM_SEARCH_VELOCITY_UMS);
    CmdDispatchResult_t result = cmd_dispatch_string(command, CMD_SRC_HMI, -1);
    if (result.accepted) {
        set_var_value_infusion_test(0.0f);
    }
}