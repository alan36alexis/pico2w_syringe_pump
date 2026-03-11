#ifndef TMC2209_H
#define TMC2209_H

#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef TMC2209_DMA_MAX_STEPS
#define TMC2209_DMA_MAX_STEPS 256u
#endif

// Tamaño del buffer Ping-Pong DMA
#ifndef TMC2209_PING_PONG_BUFFER_STEPS
#define TMC2209_PING_PONG_BUFFER_STEPS 64u
#endif
#define TMC2209_PING_PONG_BUFFER_WORDS (TMC2209_PING_PONG_BUFFER_STEPS * 2u)

typedef enum {
  TMC2209_MODE_STANDBY_FREE = 0, // Motor en reposo, ENA deshabilitado
  TMC2209_MODE_STANDBY_HOLD = 1, // Motor en reposo, ENA habilitado
  // TODO: Cambiar RUN_CW y RUN_CCW por nombres más descriptivos, como AVANCE y
  // RETROCESO
  TMC2209_MODE_RUN_CW = 2,  // Motor en movimiento sentido horario
  TMC2209_MODE_RUN_CCW = 3, // Motor en movimiento sentido antihorario
  TMC2209_MODE_NSTEPS =
      4, // Motor en movimiento por un número específico de pasos
  TMC2209_MODE_ALARM = 5 // Modo de alarma, motor detenido
} TMC2209_Mode_t;

typedef enum {
  TMC2209_PHASE_NONE = 0,
  TMC2209_PHASE_ACCEL = 1,
  TMC2209_PHASE_STEADY = 2,
  TMC2209_PHASE_DECEL = 3
} TMC2209_ProfilePhase_t;

// Estructura para contener la configuración del motor y sus pines
typedef struct {
  // Pines GPIO
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t ms1_pin;
  uint8_t ms2_pin;
  uint8_t tx_pin;
  uint8_t rx_pin;
  uint8_t limit_switch_start_pin;
  uint8_t limit_switch_end_pin;
  bool limit_switches_enabled;

  // Parámetros del motor
  uint16_t steps_per_rev;
  uint16_t microsteps;

  // Estado interno
  TMC2209_Mode_t mode; // Modo actual del driver
  bool direction;      // true: adelante, false: atrás

  // PIO State
  PIO pio;
  uint sm;
  uint offset;

  // Comunicación UART
  uart_inst_t *uart; // Instancia UART (ej. uart0 o uart1)
  uint8_t addr;      // Dirección del esclavo (0-3)

  // DMA / S-Curve State
  int dma_ramp_ch;
  int dma_steady_ch;
  int dma_stop_ch;

  float duty_cycle;
  float freq_start_hz;
  float freq_target_hz;
  uint ramp_steps;

  // Double Buffering (Ping-Pong)
  uint32_t bufA[TMC2209_PING_PONG_BUFFER_WORDS] __attribute__((aligned(8)));
  uint32_t bufB[TMC2209_PING_PONG_BUFFER_WORDS] __attribute__((aligned(8)));
  bool active_buffer_is_A; // false -> B is active, true -> A is active

  // Variables de estado interno para la rampa en curso
  uint32_t current_step_idx;
  uint32_t total_ramp_steps;
  float current_slope1;
  float current_slope2;
  uint32_t transition_step_idx;
  float current_freq_mid;
  bool is_braking;

  // Fases del perfil de movimiento completo
  TMC2209_ProfilePhase_t current_phase;
  uint32_t steady_total_steps;

  // Parámetros cacheados para la fase de frenado (DECEL)
  uint32_t decel_total_steps;
  uint32_t decel_transition_step_idx;
  float decel_freq_mid;
  float decel_freq_target_hz;
  float decel_slope1;
  float decel_slope2;

  uint32_t steady_buf[2] __attribute__((aligned(8)));

  // Legacy S-Curve Buffers
  uint32_t ramp_buf[2u * TMC2209_DMA_MAX_STEPS] __attribute__((aligned(8)));
  uint32_t stop_buf[2u * TMC2209_DMA_MAX_STEPS] __attribute__((aligned(8)));
} TMC2209_t;

// Valores de microstepping
typedef enum {
  TMC2209_MICROSTEPS_256 = 0,
  TMC2209_MICROSTEPS_128 = 1,
  TMC2209_MICROSTEPS_64 = 2,
  TMC2209_MICROSTEPS_32 = 3,
  TMC2209_MICROSTEPS_16 = 4,
  TMC2209_MICROSTEPS_8 = 5,
  TMC2209_MICROSTEPS_4 = 6,
  TMC2209_MICROSTEPS_2 = 7,
  TMC2209_MICROSTEPS_1 = 8
} TMC2209_Microsteps_t;

// Modos de Chopper (Silencioso vs Potencia)
typedef enum {
  TMC2209_CHOPPER_STEALTHCHOP = 0, // Modo silencioso (por defecto)
  TMC2209_CHOPPER_SPREADCYCLE = 1, // Modo de alto torque/velocidad
  TMC2209_CHOPPER_DYNAMIC = 2      // Ajuste automático StealthChop/SpreadCycle
} TMC2209_ChopperMode_t;

/**
 * @brief Inicializa el driver TMC2209 y los pines GPIO correspondientes.
 *
 * @param motor Puntero a la estructura de configuración del motor.
 * @param step_pin Pin GPIO para STEP.
 * @param dir_pin Pin GPIO para DIR.
 * @param enable_pin Pin GPIO para ENA.
 * @param steps_per_rev Pasos completos por revolución del motor (ej: 200).
 * @param microsteps Micropasos configurados en el driver (ej: 16).
 */
void tmc2209_init(TMC2209_t *motor, uint8_t step_pin, uint8_t dir_pin,
                  uint8_t enable_pin, uint16_t steps_per_rev,
                  uint8_t microsteps, uint8_t ms1_pin, uint8_t ms2_pin);

/**
 * @brief Habilita o deshabilita el driver del motor.
 *
 * @param motor Puntero a la estructura del motor.
 * @param enable true para habilitar (energizar), false para deshabilitar.
 */
void tmc2209_enable(TMC2209_t *motor, bool enable);

/**
 * @brief Establece la dirección de rotación.
 *
 * @param motor Puntero a la estructura del motor.
 * @param direction true para una dirección, false para la opuesta.
 */
void tmc2209_set_direction(TMC2209_t *motor, bool direction);

/**
 * @brief Establece la velocidad de rotación del motor en RPM.
 * Si RPM es 0, el motor se detiene.
 *
 * @param motor Puntero a la estructura del motor.
 * @param rpm Velocidad deseada en Revoluciones Por Minuto.
 */
void tmc2209_set_rpm(TMC2209_t *motor, float rpm);

/**
 * @brief Gira el motor un número específico de vueltas a una velocidad dada en
 * RPM.
 *
 * @param motor Puntero a la estructura del motor.
 * @param rpm Velocidad deseada en Revoluciones Por Minuto.
 * @param turns Número de vueltas a girar.
 */
void tmc2209_set_turns_at_rpm(TMC2209_t *motor, float rpm, float turns);

/**
 * @brief Detiene el motor, cancela el timer y pone en LOW el pin de step.
 *
 * @param motor Puntero a la estructura del motor.
 */
void tmc2209_stop(TMC2209_t *motor);

/**
 * @brief Detiene el motor, cancela el timer, pone en LOW el pin de step y
 * deshabilita el driver.
 *
 * @param motor Puntero a la estructura del motor.
 */
void tmc2209_stop_and_disable(TMC2209_t *motor);

/**
 * @brief Establece el número de pasos a enviar al motor a una frecuencia dada.
 * Esta función es útil para pruebas o movimientos específicos.
 *
 * @param motor Puntero a la estructura del motor.
 * @param nsteps Número de pasos a enviar.
 * @param freq Frecuencia en Hz a la que se enviarán los pasos.
 */
void tmc2209_send_nsteps_at_freq(TMC2209_t *motor, int nsteps, float freq);

void tmc2209_set_microstepping_by_pins(TMC2209_t *motor,
                                       TMC2209_Microsteps_t microsteps);

/**
 * @brief Configura los pines para los finales de carrera (Start y End).
 * Se configuran como entradas con Pull-Up.
 *
 * @param motor Puntero a la estructura del motor.
 * @param start_pin Pin GPIO para el final de carrera inicial.
 * @param end_pin Pin GPIO para el final de carrera final.
 */
void tmc2209_setup_limit_switches(TMC2209_t *motor, uint8_t start_pin,
                                  uint8_t end_pin);

/**
 * @brief Lee e imprime por consola el estado actual de los finales de carrera.
 *
 * @param motor Puntero a la estructura del motor.
 */
void tmc2209_print_limit_switches_status(TMC2209_t *motor);

/**
 * @brief Configura los pines MS1 y MS2 como pines de dirección UART.
 *
 * @param motor Puntero a la estructura del motor.
 * @param addr Dirección UART deseada (0-3).
 */
void tmc2209_set_uart_address_pins(TMC2209_t *motor, uint8_t addr);

/**
 * @brief Configura el microstepping mediante UART modificando el registro
 * CHOPCONF.
 *
 * @param motor Puntero a la estructura del motor.
 * @param microsteps Valor de microstepping (enum TMC2209_Microsteps_t).
 */
void tmc2209_set_microstepping_uart(TMC2209_t *motor,
                                    TMC2209_Microsteps_t microsteps);

/**
 * @brief Lee la configuración actual de microstepping del driver.
 * Consulta GCONF para determinar si usa pines o UART, y lee el registro
 * correspondiente (IOIN o CHOPCONF).
 *
 * @param motor Puntero a la estructura del motor.
 * @return uint16_t Número de micropasos (ej. 8, 16, 32, ...).
 */
uint16_t tmc2209_get_microsteps(TMC2209_t *motor);

bool tmc2209_is_moving(TMC2209_t *motor);

TMC2209_Mode_t tmc2209_get_mode(TMC2209_t *motor);

/**
 * @brief Configura la instancia UART para el control del driver.
 *
 * @param motor Puntero a la estructura del motor.
 * @param uart Instancia UART a utilizar (uart0 o uart1).
 * @param baudrate Velocidad en baudios (típicamente 115200).
 * @param addr Dirección del esclavo (determinado por pines MS1/MS2, por defecto
 * 0).
 * @param tx_pin Pin GPIO para transmisión UART (TX).
 * @param rx_pin Pin GPIO para recepción UART (RX).
 */
void tmc2209_setup_uart(TMC2209_t *motor, uart_inst_t *uart, uint32_t baudrate,
                        uint8_t addr, uint8_t tx_pin, uint8_t rx_pin);

/**
 * @brief Escribe un valor en un registro interno del TMC2209 vía UART.
 *
 * @param motor Puntero a la estructura del motor.
 * @param reg Dirección del registro (ej. 0x10 para IHOLD_IRUN).
 * @param value Valor de 32 bits a escribir.
 */
void tmc2209_write_register(TMC2209_t *motor, uint8_t reg, uint32_t value);

/**
 * @brief Lee un valor de un registro interno del TMC2209 vía UART.
 * Nota: Esta función es bloqueante y maneja el eco del bus de un solo cable.
 *
 * @param motor Puntero a la estructura del motor.
 * @param reg Dirección del registro a leer.
 * @return uint32_t Valor leído del registro. Devuelve 0 si hay error o timeout.
 */
uint32_t tmc2209_read_register(TMC2209_t *motor, uint8_t reg);

/**
 * @brief Configura la corriente de funcionamiento y de espera (Registro
 * IHOLD_IRUN).
 *
 * @param motor Puntero a la estructura del motor.
 * @param run_current Escala de corriente de funcionamiento (0-31).
 * @param hold_current Escala de corriente de espera (0-31).
 * @param tpowerdown Tiempo de espera antes de reducir la corriente a hold
 * (0-255). 20 es un valor típico (~0.4s).
 */
void tmc2209_set_current(TMC2209_t *motor, uint8_t run_current,
                         uint8_t hold_current, uint8_t tpowerdown);

/**
 * @brief Configura el umbral de sensibilidad para StallGuard4 (Registro
 * SGTHRS). StallGuard4 permite la detección de carga y parada sin sensores.
 *
 * @param motor Puntero a la estructura del motor.
 * @param threshold Valor del umbral (0-255).
 *        0: Deshabilitado / Alta sensibilidad (detección con poca carga).
 *        255: Baja sensibilidad (requiere mucha carga para detectar).
 */
void tmc2209_set_stallguard_threshold(TMC2209_t *motor, uint8_t threshold);

/**
 * @brief Lee el valor de carga mecánica actual (Registro SG_RESULT).
 *
 * @param motor Puntero a la estructura del motor.
 * @return uint16_t Valor de carga (0 a 510). 0 indica carga máxima (posible
 * bloqueo).
 */
uint16_t tmc2209_get_stallguard_result(TMC2209_t *motor);

/**
 * @brief Lee directamente el valor de SG_RESULT sin verificar GCONF.
 * Más rápido para bucles de control en tiempo real.
 */
uint16_t tmc2209_read_sg_result(TMC2209_t *motor);

/**
 * @brief Lee el registro DRV_STATUS para monitoreo detallado.
 * Permite detectar fallas como cortocircuitos (S2G, S2VS), temperatura (OTW,
 * OT), ola abierta (OLA, OLB) y flag de StallGuard.
 *
 * @param motor Puntero a la estructura del motor.
 * @return uint32_t Valor del registro DRV_STATUS.
 */
uint32_t tmc2209_read_drv_status(TMC2209_t *motor);

/**
 * @brief Lee el registro GSTAT (Global Status) para monitoreo de fallas
 * generales. Permite detectar reset, UV_CP (carga insuficiente de la bomba), o
 * sobretemperatura.
 *
 * @param motor Puntero a la estructura del motor.
 * @return uint32_t Valor del registro GSTAT.
 */
uint32_t tmc2209_read_gstat(TMC2209_t *motor);

/**
 * @brief Limpia los flags de error específicos en GSTAT escribiendo un 1 en el
 * bit.
 *
 * @param motor Puntero a la estructura del motor.
 * @param clear_mask Los bits a limpiar (ej. 1 para reset, 2 para drv_err, 4
 * para uv_cp).
 */
void tmc2209_clear_gstat(TMC2209_t *motor, uint8_t clear_mask);

/**
 * @brief Configura el modo de chopper (StealthChop o SpreadCycle o Dinámico)
 * modificando el registro GCONF.
 *
 * @param motor Puntero a la estructura del motor.
 * @param mode Modo deseado (TMC2209_CHOPPER_STEALTHCHOP,
 * TMC2209_CHOPPER_SPREADCYCLE, o TMC2209_CHOPPER_DYNAMIC).
 * @param tpwmthrs Umbral de velocidad para cambio automático a SpreadCycle
 * (solo en modo dinámico). 0 lo deshabilita.
 */
void tmc2209_set_chopper_mode(TMC2209_t *motor, TMC2209_ChopperMode_t mode,
                              uint32_t tpwmthrs);

/**
 * @brief Configura el umbral de velocidad inferior para CoolStep (Registro
 * TCOOLTHRS). CoolStep se desactiva si la velocidad cae por debajo de este
 * umbral.
 *
 * @param motor Puntero a la estructura del motor.
 * @param threshold Valor del umbral (0-1048575).
 */
void tmc2209_set_coolstep_threshold(TMC2209_t *motor, uint32_t threshold);

/**
 * @brief Calcula el valor para el registro TCOOLTHRS a partir de una frecuencia
 * de pasos. Utiliza la frecuencia de reloj interna típica de 12 MHz.
 *
 * @param steps_per_sec Frecuencia de pasos en Hz (micropasos por segundo).
 * @return uint32_t Valor calculado para TCOOLTHRS (limitado a 20 bits).
 */
uint32_t tmc2209_compute_coolstep_threshold(float steps_per_sec);

void tmc2209_set_pdn_disable(TMC2209_t *motor, bool disable);

/**
 * @brief Configura TOFF e Interpolación en el registro CHOPCONF.
 *
 * @param motor Puntero a la estructura del motor.
 * @param toff Tiempo de apagado (0=Deshabilitado, 1-15=Activo). Típico: 3-5.
 * @param intpol true para activar interpolación a 256 pasos, false para
 * desactivar.
 */
void tmc2209_configure_chopconf(TMC2209_t *motor, uint8_t toff, bool intpol);

/**
 * @brief Configura los parámetros detallados del chopper: HSTRT, HEND y TBL en
 * el registro CHOPCONF. Útiles para afinar el modo SpreadCycle.
 *
 * @param motor Puntero a la estructura del motor.
 * @param hstrt Hysteresis start (0-7).
 * @param hend Hysteresis end (0-15).
 * @param tbl Blank time (0-3). Mapea a 16, 24, 36, o 54 ciclos de reloj.
 */
void tmc2209_set_chopper_parameters(TMC2209_t *motor, uint8_t hstrt,
                                    uint8_t hend, uint8_t tbl);

/**
 * @brief Establece la velocidad del motor usando el generador de pulsos interno
 * (VACTUAL). Al usar VACTUAL != 0, el driver ignora los pines STEP/DIR y genera
 * los pasos internamente.
 *
 * @param motor Puntero a la estructura del motor.
 * @param vactual Valor de velocidad (con signo). 0 detiene el generador
 * interno. Fórmula: VACTUAL = (Pasos/s * 2^24) / f_CLK
 */
void tmc2209_set_vactual(TMC2209_t *motor, int32_t vactual);

/**
 * @brief Calcula el valor de VACTUAL necesario para una velocidad dada en RPM.
 *
 * @param motor Puntero a la estructura del motor.
 * @param rpm Velocidad deseada en RPM.
 * @return int32_t Valor calculado para el registro VACTUAL.
 */
int32_t tmc2209_compute_vactual(TMC2209_t *motor, float rpm);

// --- Funciones DMA / Curva S (Portadas de stepgen) ---

void tmc2209_start_s_curve_dma(TMC2209_t *motor, float freq_start_hz,
                               float freq_target_hz, float duty_cycle,
                               uint ramp_steps, int aggressiveness);
void tmc2209_stop_s_curve_dma(TMC2209_t *motor, float freq_end_hz,
                              uint ramp_steps);
void tmc2209_change_frequency_dma(TMC2209_t *motor, float freq_new_hz,
                                  uint ramp_steps);

/**
 * @brief Inicia un movimiento con una curva de aceleración de 2 partes (2
 * rectas).
 *
 * @param motor Puntero a la estructura del motor.
 * @param freq_start Frecuencia inicial en Hz.
 * @param pulses_seg1 Cantidad de pulsos del segmento 1.
 * @param freq_mid Frecuencia intermedia en Hz.
 * @param pulses_seg2 Cantidad de pulsos del segmento 2.
 * @param freq_target Frecuencia objetivo final en Hz.
 */
void tmc2209_start_2part_curve_dma(TMC2209_t *motor, float freq_start,
                                   uint32_t pulses_seg1, float freq_mid,
                                   uint32_t pulses_seg2, float freq_target);

/**
 * @brief Inicia un frenado con una curva de desaceleración de 2 partes (2
 * rectas).
 *
 * @param motor Puntero a la estructura del motor.
 * @param freq_start Frecuencia inicial actual (o deseada) de frenado en Hz.
 * @param pulses_seg1 Cantidad de pulsos del segmento 1 de frenado.
 * @param freq_mid Frecuencia intermedia en Hz.
 * @param pulses_seg2 Cantidad de pulsos del segmento 2 de frenado.
 * @param freq_target Frecuencia final en Hz (generalmente 0 o muy baja para
 * detenerse).
 */
void tmc2209_stop_2part_curve_dma(TMC2209_t *motor, float freq_start,
                                  uint32_t pulses_seg1, float freq_mid,
                                  uint32_t pulses_seg2, float freq_target);

/**
 * @brief Ejecuta un perfil de movimiento completo: Aceleración -> Velocidad
 * Constante -> Frenado. Todo generado por DMA en hardware de forma continua.
 *
 * @param motor Puntero a la estructura del motor.
 * @param freq_start Frecuencia inicial (despegue).
 * @param accel_pulses_1 Pulsos segmento 1 de aceleración.
 * @param accel_freq_mid Frecuencia intermedia aceleración.
 * @param accel_pulses_2 Pulsos segmento 2 de aceleración.
 * @param freq_target Frecuencia de crucero (constante).
 * @param steady_pulses Pulsos a velocidad constante.
 * @param decel_pulses_1 Pulsos segmento 1 de frenado.
 * @param decel_freq_mid Frecuencia intermedia frenado.
 * @param decel_pulses_2 Pulsos segmento 2 de frenado.
 * @param freq_end Frecuencia final tras frenado (casi 0).
 */
void tmc2209_move_2part_profile_dma(
    TMC2209_t *motor, float freq_start, uint32_t accel_pulses_1,
    float accel_freq_mid, uint32_t accel_pulses_2, float freq_target,
    uint32_t steady_pulses, uint32_t decel_pulses_1, float decel_freq_mid,
    uint32_t decel_pulses_2, float freq_end);

/**
 * @brief Inicia un movimiento de avance (CW) con perfil de velocidad en Curva
 * S.
 *
 * @param motor Puntero a la estructura del motor.
 * @param freq_start Frecuencia inicial en Hz.
 * @param freq_end Frecuencia final (objetivo) en Hz.
 * @param ramp_steps Cantidad de pasos para completar la rampa de aceleración.
 */
void tmc2209_move_forward_s_curve(TMC2209_t *motor, float freq_start,
                                  float freq_end, uint ramp_steps);

/**
 * @brief Inicia un movimiento de retroceso (CCW) con perfil de velocidad en
 * Curva S.
 *
 * @param motor Puntero a la estructura del motor.
 * @param freq_start Frecuencia inicial en Hz.
 * @param freq_end Frecuencia final (objetivo) en Hz.
 * @param ramp_steps Cantidad de pasos para completar la rampa de aceleración.
 */
void tmc2209_move_backward_s_curve(TMC2209_t *motor, float freq_start,
                                   float freq_end, uint ramp_steps);

#endif // TMC2209_H