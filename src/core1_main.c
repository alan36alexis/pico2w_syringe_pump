#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

// Componentes del proyecto
#include "closed_loop.h"
#include "core1_main.h"
#include "crosscore_cmd.h"
#include "crosscore_logger.h"
#include "hardware/pio.h"
#include "honeywell_spi.h"
#include "pulse_counter.pio.h"
#include "quadrature_encoder.pio.h"
#include "system_config.h"
#include "tmc2209.h"

// --- DEBUG MODE ---
// 1 = Activado (printf habilitado), 0 = Desactivado (printf mudo)
#define DEBUG_MODE 0

#if DEBUG_MODE
#define LOG_DEBUG(...) printf(__VA_ARGS__)
#else
#define LOG_DEBUG(...)
#endif

// --- Configuracion ---
// Definicion de pines
#define MOTOR_DIR_PIN 2
#define MOTOR_STEP_PIN 3

// Pines para UART (UART1 por defecto en Pico: TX=GP4, RX=GP5)
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// Pines para microstepping y enable
#define MOTOR_MS2_PIN 6
#define MOTOR_MS1_PIN 7
#define MOTOR_ENA_PIN 8

// Alias para modo UART (Mismos pines físicos)
#define MOTOR_ADDR_PIN_1 MOTOR_MS2_PIN // Bit 1 de dirección
#define MOTOR_ADDR_PIN_0 MOTOR_MS1_PIN // Bit 0 de dirección

// Selección de Modo: true = Modo UART (Pines fijan dirección), false = Modo
// Pines (Pines fijan pasos)
#define USE_UART_MODE true

// Pines de Finales de Carrera
#define LIMIT_SW_START_PIN 27
#define LIMIT_SW_END_PIN 26

// Pines de Encoder
#define ENCODER_PIN_A 20
#define ENCODER_PIN_B 21

// --- Pines Honeywell SPI0 ---
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

honeywell_hsc_t pressure_sensor;
TMC2209_t *global_motor = NULL;

typedef enum {
  EMERGENCY_NORMAL = 0,
  EMERGENCY_STOPPING,
  EMERGENCY_RETRACTING,
  EMERGENCY_STOPPED
} EmergencyState;

static EmergencyState emergency_state = EMERGENCY_NORMAL;

bool honeywell_timer_callback(repeating_timer_t *rt) {
  honeywell_hsc_data_t data;
  if (honeywell_hsc_read(&pressure_sensor, &data)) {
    float pressure_mmhg = data.pressure_psi * 51.7149f;
    logger_send_pressure_update(data.pressure_psi);

    switch (emergency_state) {
    case EMERGENCY_NORMAL:
      if (data.status == 0 && data.pressure_psi > 20.0f) {
        if (global_motor != NULL && tmc2209_is_moving(global_motor)) {
          logger_send_pressure_alert(data.pressure_psi);
          tmc2209_stop_s_curve_dma(global_motor, 0.0f, 20);
          emergency_state = EMERGENCY_STOPPING;
        }
      }
      break;

    case EMERGENCY_STOPPING:
      if (global_motor != NULL && !tmc2209_is_moving(global_motor)) {
        logger_send_motor_stopped();
        bool inv_dir = !global_motor->direction;
        tmc2209_set_direction(global_motor, inv_dir);
        // Retroceso continuo (e.g. 3000 Hz)
        logger_send_motor_retracting();
        tmc2209_start_s_curve_dma(global_motor, 500.0f, 3000.0f, 0.5f, 100, 2);
        emergency_state = EMERGENCY_RETRACTING;
      }
      break;

    case EMERGENCY_RETRACTING:
      if (data.status == 0 && data.pressure_psi < 15.0f) {
        logger_send_pressure_safe(data.pressure_psi);
        tmc2209_stop_s_curve_dma(global_motor, 0.0f, 20);
        emergency_state = EMERGENCY_STOPPED;
      }
      break;

    case EMERGENCY_STOPPED:
      // Permanece detenido
      break;
    }
  } else {
    LOG_DEBUG("SPI read error\n");
  }
  return true; // Keep repeating
}

// --- Helper para conversión de corriente (Amperes -> CS) ---
uint8_t tmc2209_amps_to_cs(float amps) {
  float cs = (amps * 18.11f) - 1.0f;

  // Clamping de seguridad
  if (cs < 0.0f)
    return 0;
  if (cs > 31.0f)
    return 31;

  // Redondeo al entero más cercano
  return (uint8_t)(cs + 0.5f);
}

void tmc2209_set_current_amps(TMC2209_t *motor, float run_amps,
                              float hold_amps) {
  tmc2209_set_current(motor, tmc2209_amps_to_cs(run_amps),
                      tmc2209_amps_to_cs(hold_amps), 20);
}

void tmc2209_move_linear_um_dma(TMC2209_t *motor, float target_um,
                                float target_velocity_ums) {
  LOG_DEBUG("--- Abstracción de Movimiento Lineal ---\n");
  LOG_DEBUG("Target: %.1f um a %.1f um/s\n", target_um, target_velocity_ums);

  if (target_velocity_ums <= 0.0f || target_um == 0.0f) {
    LOG_DEBUG("Error: Velocidad cero o distancia cero.\n");
    return;
  }

  // 1. Determinar dirección
  bool direction = (target_um > 0.0f);
  target_um = fabsf(target_um); // Trabajar con magnitud absoluta

  // 2. Lookup Table para Configuración del Motor según Velocidad (um/s)
  TMC2209_Microsteps_t msteps;
  TMC2209_ChopperMode_t chop_mode;
  float run_amps;

  if (target_velocity_ums == 0.0f) {
    msteps = TMC2209_MICROSTEPS_16;
    chop_mode = TMC2209_CHOPPER_STEALTHCHOP;
    run_amps = 0.5f;
  } else if (target_velocity_ums < 350.0f) {
    msteps = TMC2209_MICROSTEPS_16;
    chop_mode = TMC2209_CHOPPER_STEALTHCHOP;
    run_amps = 0.5f;
  } else if (target_velocity_ums <= 550.0f) {
    msteps = TMC2209_MICROSTEPS_16;
    chop_mode = TMC2209_CHOPPER_SPREADCYCLE;
    run_amps = 1.0f;
  } else if (target_velocity_ums <= 800.0f) {
    msteps = TMC2209_MICROSTEPS_8;
    chop_mode = TMC2209_CHOPPER_SPREADCYCLE;
    run_amps = 1.5f;
  } else {
    msteps = TMC2209_MICROSTEPS_2; // > 800 um/s
    chop_mode = TMC2209_CHOPPER_SPREADCYCLE;
    run_amps = 1.5f;
  }

  // Aplicar Conf. al Driver
  tmc2209_set_direction(motor, direction);

  // Detener temporalmente (justin) y configurar driver
  tmc2209_stop(motor);
  sleep_ms(5); // Pequeña pausa
  tmc2209_set_current_amps(motor, run_amps, 0.4f);
  tmc2209_set_chopper_mode(motor, chop_mode, 100);
  tmc2209_set_microstepping_uart(motor, msteps);

  uint16_t current_msteps_val = tmc2209_get_microsteps(motor);

  LOG_DEBUG(">> Configuración OK: Microsteps=1/%d, Chopper=%s, IRUN=%.1fA\n",
            current_msteps_val,
            (chop_mode == TMC2209_CHOPPER_STEALTHCHOP) ? "StealthChop"
                                                       : "SpreadCycle",
            run_amps);

  // 3. Conversiones Cinemáticas
  // Cuántos micrómetros se avanza por cada paso completo del motor (sin
  // microstepping) factor = (Avance del Husillo) / (Pasos por rev del motor *
  // Reducción)
  float um_per_full_step =
      LEAD_SCREW_PITCH_UM / (MOTOR_STEPS_PER_REV * REAL_GEARBOX_RATIO);

  // Cuántos micrómetros se avanza por micropaso
  float um_per_microstep = um_per_full_step / (float)current_msteps_val;

  // Total de micropasos necesarios para alcanzar target_um
  uint32_t total_microsteps = (uint32_t)(target_um / um_per_microstep);

  // Consideraciones para perfiles de aceleración cortos
  if (total_microsteps < 100) {
    LOG_DEBUG("Advertencia: Movimiento solicitado muy corto (%u micropasos). "
              "Se enviará en burst.\n",
              total_microsteps);
    float target_freq_hz = target_velocity_ums / um_per_microstep;
    tmc2209_send_nsteps_at_freq(motor, total_microsteps, target_freq_hz);
    return;
  }

  // Convertir Velocidad Lineal a Frecuencia de Micropasos (Hz)
  float target_freq_hz = target_velocity_ums / um_per_microstep;

  // Frecuencias inicial y final
  float f_start = target_freq_hz * 0.1f; // 10% de target
  if (f_start < 50.0f)
    f_start = 50.0f; // Limitador bajo

  float f_mid_accel = f_start + (target_freq_hz - f_start) * 0.5f;
  float f_mid_decel = target_freq_hz - (target_freq_hz - f_start) * 0.5f;
  float f_end = f_start;

  // 4. Perfil de Velocidad Trapecial 2-Partes (Curva S)
  // Vamos a usar la heurística clásica: 10% de distancia para acelerar, 80%
  // constante, 10% frenar O en su defecto, que la curva sea suficientemente
  // suave, limitando la aceleración.

  // Porcentajes para pasos (debe sumar total_microsteps)
  uint32_t pasos_aceleracion = (uint32_t)(total_microsteps * 0.05f);
  uint32_t pasos_frenado = pasos_aceleracion;

  // Si nos sobran para hacer 2-part profile
  if (pasos_aceleracion < 20) {
    pasos_aceleracion = 20;
    pasos_frenado = 20;
  }

  uint32_t pasos_constantes =
      total_microsteps - (pasos_aceleracion + pasos_frenado);

  // Si el movimiento es demasiado corto para 15% de aceleración
  if (pasos_aceleracion * 2 >= total_microsteps) {
    // Perfil triangular
    pasos_aceleracion = total_microsteps / 2;
    pasos_frenado = total_microsteps - pasos_aceleracion;
    pasos_constantes = 0;
  }

  // Dividimos la aceleracion en 2 fases
  uint32_t a_p1 = pasos_aceleracion / 2;
  uint32_t a_p2 = pasos_aceleracion - a_p1;
  uint32_t d_p1 = pasos_frenado / 2;
  uint32_t d_p2 = pasos_frenado - d_p1;

  LOG_DEBUG("Cinemática => Pasos totales: %u, Freq: %.1f Hz\n",
            total_microsteps, target_freq_hz);
  LOG_DEBUG("Perfil => Accel: %u (P1:%u P2:%u) | Crucero: %u | Decel: %u "
            "(P1:%u P2:%u)\n",
            pasos_aceleracion, a_p1, a_p2, pasos_constantes, pasos_frenado,
            d_p1, d_p2);

  // 5. Iniciar Movimiento DMA
  tmc2209_move_2part_profile_dma(motor, f_start, a_p1, f_mid_accel, a_p2,
                                 target_freq_hz, pasos_constantes, d_p1,
                                 f_mid_decel, d_p2, f_end);
}

// --- Función para medir la velocidad de giro basada en el encoder ---
// Devuelve la velocidad en Pulsos Por Segundo (PPS)
float measure_encoder_speed(int32_t current_count, int32_t *last_count_ptr,
                            uint32_t *last_time_us_ptr) {
  uint32_t current_time = time_us_32();
  uint32_t delta_time = current_time - *last_time_us_ptr;

  // Evitar division por 0
  if (delta_time == 0)
    return 0.0f;

  int32_t delta_count = current_count - *last_count_ptr;
  float pps = ((float)delta_count * 1000000.0f) / (float)delta_time;

  *last_count_ptr = current_count;
  *last_time_us_ptr = current_time;

  return pps;
}

/**
 * @brief Funciones y ejecución principal para Core 1 (Baremetal)
 */
void core1_main(void) {
  // Allow Core 0 to pause us during Flash writes
  multicore_lockout_victim_init();

  uint32_t counter = 0;
  int32_t last_encoder_count = 0;
  int32_t last_encoder_a = 0;
  int32_t last_encoder_b = 0;

  // Variables para la medición de velocidad
  int32_t last_speed_encoder_count = 0;
  uint32_t last_speed_calc_time = time_us_32();

  // Instancia del controlador de lazo cerrado
  ClosedLoopState_t scl;
  closed_loop_init(&scl);

  // Configurar pines SPI1 para Honeywell
  spi_init(SPI_PORT, 1000 * 1000);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SIO); // CS is handled manually
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);

  honeywell_hsc_init(&pressure_sensor, SPI_PORT, PIN_CS, -100.0f, 100.0f);

  // Iniciar timer repetitivo cada 500ms
  repeating_timer_t honeywell_timer;
  // add_repeating_timer_ms(500, honeywell_timer_callback, NULL,
  // &honeywell_timer);

  TMC2209_t motor1;
  global_motor = &motor1;
  bool last_motor_direction = true;

  tmc2209_init(&motor1, MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_ENA_PIN,
               MOTOR_STEPS_PER_REV, MOTOR_MICROSTEPS, MOTOR_MS1_PIN,
               MOTOR_MS2_PIN);
  sleep_ms(200);

  // Configurar finales de carrera
  tmc2209_setup_limit_switches(&motor1, LIMIT_SW_START_PIN, LIMIT_SW_END_PIN);

  if (USE_UART_MODE) {
    // MODO UART: Configuramos los pines como dirección fija (Addr 0: Ambos LOW)
    LOG_DEBUG(
        "Iniciando en MODO UART (Pines MS usados para direccionamiento 0)\n");
    tmc2209_set_uart_address_pins(&motor1, 0);
    LOG_DEBUG("Direccion configurada: %d\n", motor1.addr);
    // Inicializar UART
    tmc2209_setup_uart(&motor1, uart1, 57600, 0, UART_TX_PIN, UART_RX_PIN);
    // --- DIAGNÓSTICO UART ---
    // Leemos el registro IOIN (0x06) para verificar si el driver responde.
    // Si devuelve 0, hay un problema físico (cableado, resistencia 1k faltante,
    // o falta de VM).
    uint32_t check_uart = tmc2209_read_register(&motor1, 0x06);
    if (check_uart == 0) {
      logger_send_uart_init_fail();
    } else {
      logger_send_uart_init_ok(check_uart);
    }

    // Configurar pdn_disable = 1 en GCONF (Bit 6)
    // Esto deshabilita la función de apagado en el pin UART, dejándolo solo
    // para comunicación.
    tmc2209_set_pdn_disable(&motor1, true);

    // Configurar StealthChop explícitamente (necesario para lectura válida de
    // SG_RESULT)
    tmc2209_set_chopper_mode(&motor1, TMC2209_CHOPPER_DYNAMIC, 100);

    // Configurar TOFF e Interpolación (CHOPCONF)
    // TOFF=4 (Activa el driver), intpol=true (Suaviza movimiento interpolando a
    // 256 pasos)
    tmc2209_configure_chopconf(&motor1, 2, true);

    // Ejemplo: HSTRT = 4, HEND = 1, TBL = 2 (36 ciclos de reloj)
    tmc2209_set_chopper_parameters(&motor1, 5, 0, 1);

    tmc2209_set_microstepping_uart(&motor1, MOTOR_MICROSTEPS);

    // Configuración de corriente en Amperes
    // IRUN: 1.0A (~CS 14/15), IHOLD: 0.5A (~CS 7)
    tmc2209_set_current_amps(&motor1, 0.6f, 0.3f);

    // Habilitar el driver AL FINAL de la configuración para evitar movimientos
    // bruscos
    tmc2209_enable(&motor1, true);

    uint16_t msteps_read = tmc2209_get_microsteps(&motor1);
    logger_send_uart_init_microsteps_read(msteps_read);
  } else {
    // MODO PINES: Configuramos los pines para microstepping
    logger_send_pins_init_mode();
    tmc2209_set_microstepping_by_pins(&motor1, MOTOR_MICROSTEPS);
  }

  // --- INICIALIZACION ENCODER (PIO) ---
  uint sm_enc_q = 0, sm_enc_a = 0, sm_enc_b = 0;
  if (ENABLE_ENCODER) {
    if (USE_QUADRATURE_ENCODER) {
      uint offset = pio_add_program(pio1, &quadrature_encoder_program);
      sm_enc_q = pio_claim_unused_sm(pio1, true);
      quadrature_encoder_program_init(pio1, sm_enc_q, ENCODER_PIN_A, 0);
    } else {
      uint offset = pio_add_program(pio1, &pulse_counter_program);
      sm_enc_a = pio_claim_unused_sm(pio1, true);
      sm_enc_b = pio_claim_unused_sm(pio1, true);
      pulse_counter_program_init(pio1, sm_enc_a, offset, ENCODER_PIN_A);
      pulse_counter_program_init(pio1, sm_enc_b, offset, ENCODER_PIN_B);
    }
  }

  // --- EJECUCION DE MOVIMIENTO LINEAL ---
  // tmc2209_move_linear_um_dma(&motor1, -15000.0f, 450.0f);
  // tmc2209_set_direction(&motor1, false);
  // tmc2209_send_nsteps_at_freq(&motor1, 3200, 500.0f);

  while (true) {
#define RESET_ENCODER_COUNTS()                                                 \
  do {                                                                         \
    if (ENABLE_ENCODER) {                                                      \
      if (USE_QUADRATURE_ENCODER) {                                            \
        pio_sm_exec(pio1, sm_enc_q, 0xe040); /* set y, 0 */                    \
      } else {                                                                 \
        pio_sm_exec(pio1, sm_enc_a, 0xa02b); /* mov x, ~null */                \
        pio_sm_exec(pio1, sm_enc_b, 0xa02b); /* mov x, ~null */                \
      }                                                                        \
      last_encoder_count = 0;                                                  \
      last_encoder_a = 0;                                                      \
      last_encoder_b = 0;                                                      \
      last_speed_encoder_count = 0;                                            \
      scl.start_encoder_count_cl = 0;                                          \
    }                                                                          \
  } while (0)

    Core1CmdMessage_t cmd;

    // Check for commands from Core 0
    if (queue_try_remove(&crosscore_cmd_queue, &cmd)) {
      switch (cmd.id) {
      case CMD_MOVE_LINEAR_UM:
        LOG_DEBUG("CMD received: move_linear_um (%.1f, %.1f)\n",
                  cmd.payload.move_linear.target_um,
                  cmd.payload.move_linear.target_velocity_ums);

        RESET_ENCODER_COUNTS();

        if (ENABLE_ENCODER) {
          int32_t current_count =
              USE_QUADRATURE_ENCODER
                  ? quadrature_encoder_get_count(pio1, sm_enc_q)
                  : pulse_counter_get_count(pio1, sm_enc_a);
          closed_loop_init_move(&scl, cmd.payload.move_linear.target_um,
                                cmd.payload.move_linear.target_velocity_ums,
                                current_count);
        }

        logger_send_motor_moving();
        tmc2209_move_linear_um_dma(global_motor,
                                   cmd.payload.move_linear.target_um,
                                   cmd.payload.move_linear.target_velocity_ums);
        break;

      case CMD_STOP_MOTOR:
        LOG_DEBUG("CMD received: stop_motor\n");
        scl.waiting_for_correction = false;
        if (tmc2209_is_moving(global_motor)) {
          float current_freq = tmc2209_get_current_freq_hz(global_motor);
          tmc2209_stop_from_current_freq_dma(global_motor, 200,
                                             current_freq * 0.5f, 200, 50.0f);
        } else {
          tmc2209_stop(global_motor);
        }
        break;

      case CMD_MOVE_2PART_PROFILE:
        LOG_DEBUG("CMD received: move_2part_profile\n");
        RESET_ENCODER_COUNTS();
        logger_send_motor_moving();
        tmc2209_move_2part_profile_dma(
            global_motor, cmd.payload.move_2part.start_freq,
            cmd.payload.move_2part.a_p1, cmd.payload.move_2part.f_mid_accel,
            cmd.payload.move_2part.a_p2, cmd.payload.move_2part.f_target,
            cmd.payload.move_2part.c_steps, cmd.payload.move_2part.d_p1,
            cmd.payload.move_2part.f_mid_decel, cmd.payload.move_2part.d_p2,
            cmd.payload.move_2part.f_end);
        break;

      case CMD_HOME_START: {
        float speed = 1200.0f; // Velocidad fija solicitada de 1200 um/s
        LOG_DEBUG("CMD received: home_start (%.1f um/s)\n", speed);
        RESET_ENCODER_COUNTS();
        scl.waiting_for_correction = false;
        logger_send_motor_moving();
        // Distancia negativa larga para asegurar que llegue al sensor (Longitud
        // del eje: 100mm = 100000um)
        tmc2209_move_linear_um_dma(global_motor, -105000.0f, speed);
        break;
      }

      case CMD_HOME_END: {
        float speed = 1200.0f; // Velocidad fija solicitada de 1200 um/s
        LOG_DEBUG("CMD received: home_end (%.1f um/s)\n", speed);
        RESET_ENCODER_COUNTS();
        scl.waiting_for_correction = false;
        logger_send_motor_moving();
        // Distancia positiva larga para asegurar que llegue al sensor
        tmc2209_move_linear_um_dma(global_motor, 105000.0f, speed);
        break;
      }

      case CMD_MOVE_NSTEPS: {
        int32_t steps = cmd.payload.move_nsteps.nsteps;
        LOG_DEBUG("CMD received: move_nsteps (%d steps, %.1f Hz)\n", steps,
                  cmd.payload.move_nsteps.freq_hz);
        RESET_ENCODER_COUNTS();
        scl.waiting_for_correction = false;
        logger_send_motor_moving();

        if (steps < 0) {
          tmc2209_set_direction(global_motor, false);
          steps = -steps;
        } else {
          tmc2209_set_direction(global_motor, true);
        }

        tmc2209_send_nsteps_at_freq(global_motor, steps,
                                    cmd.payload.move_nsteps.freq_hz);
        break;
      }

      case CMD_STOP_IMMEDIATE:
        LOG_DEBUG("CMD received: stop_immediate\n");
        scl.waiting_for_correction = false;
        tmc2209_stop(global_motor);
        break;

      default:
        break;
      }
    }

    bool was_moving = tmc2209_is_moving(&motor1);

    bool is_braking = false;
    uint8_t start_sw_debounce = 0;
    uint8_t end_sw_debounce = 0;
    const uint8_t DEBOUNCE_THRESHOLD = 3; // 3 iteraciones de ~10ms = 30ms

    while (tmc2209_is_moving(&motor1)) {
      // Check for incoming commands while moving
      if (queue_try_remove(&crosscore_cmd_queue, &cmd)) {
        if (cmd.id == CMD_STOP_IMMEDIATE) {
          LOG_DEBUG("CMD received while moving, aborting immediately.\n");
          scl.waiting_for_correction = false;
          tmc2209_stop(global_motor);
          is_braking = true;
        } else if (cmd.id == CMD_STOP_MOTOR || cmd.id == CMD_MOVE_LINEAR_UM ||
                   cmd.id == CMD_MOVE_2PART_PROFILE ||
                   cmd.id == CMD_HOME_START || cmd.id == CMD_HOME_END ||
                   cmd.id == CMD_MOVE_NSTEPS) {
          LOG_DEBUG("CMD received while moving, aborting current move.\n");
          scl.waiting_for_correction = false;

          if (!is_braking) {
            float current_freq = tmc2209_get_current_freq_hz(global_motor);
            tmc2209_stop_from_current_freq_dma(global_motor, 200,
                                               current_freq * 0.5f, 200, 50.0f);
            is_braking = true;
          }
        }
      }

      // Check Limit Switches for emergency braking
      if (global_motor->limit_switches_enabled && !is_braking) {
        bool start_sw_active = !gpio_get(global_motor->limit_switch_start_pin);
        bool end_sw_active = !gpio_get(global_motor->limit_switch_end_pin);

        // Simple debounce counter (se asume un sleep_ms(10) al final del while)
        if (start_sw_active)
          start_sw_debounce++;
        else
          start_sw_debounce = 0;
        if (end_sw_active)
          end_sw_debounce++;
        else
          end_sw_debounce = 0;

        // Si estamos yendo hacia el limit switch activo (false =
        // Start/Backward, true = End/Forward)
        if ((!global_motor->direction &&
             start_sw_debounce >= DEBOUNCE_THRESHOLD) ||
            (global_motor->direction &&
             end_sw_debounce >= DEBOUNCE_THRESHOLD)) {

          LOG_DEBUG("Limit switch alcanzado y debounced. Frenado agresivo!\n");
          scl.waiting_for_correction = false;
          is_braking = true; // Activar flag para evitar reentradas continuas al
                             // flete de frenado

          float current_freq = tmc2209_get_current_freq_hz(global_motor);
          tmc2209_stop_from_current_freq_dma(global_motor, 500,
                                             current_freq * 0.5f, 500, 50.0f);
        }
      }

      uint32_t drv_status = tmc2209_read_drv_status(&motor1);
      uint32_t gstat = tmc2209_read_gstat(&motor1);
      uint16_t stall = tmc2209_read_sg_result(&motor1);

      // Detectar y reportar errores
      if (gstat & 0x01)
        tmc2209_clear_gstat(&motor1, 1);
      if (gstat & 0x02)
        tmc2209_clear_gstat(&motor1, 2);
      if (gstat & 0x04)
        tmc2209_clear_gstat(&motor1, 4);

      if (gstat || drv_status || stall) {
        logger_send_drv_status_error(stall, drv_status, gstat);
      }

      // Enviar progreso calculado desde el TMC2209 (aproximadamente cada 100ms)
      if (counter % 10 == 0) {
        float pct = tmc2209_get_move_progress_pct(&motor1);
        logger_send_motor_progress(pct);
      }

      // Enviar progreso del encoder cada 100ms aprox (10 * 10ms)
      if (ENABLE_ENCODER && counter % 10 == 0) {
        float pps_actual = 0.0f;
        if (USE_QUADRATURE_ENCODER) {
          int32_t current_count = quadrature_encoder_get_count(pio1, sm_enc_q);

          float pps = measure_encoder_speed(
              current_count, &last_speed_encoder_count, &last_speed_calc_time);
          logger_send_encoder_speed(pps);
          pps_actual = pps;

          if (current_count != last_encoder_count) {
            logger_send_encoder_count(current_count);
            last_encoder_count = current_count;
          }
        } else {
          int32_t a = pulse_counter_get_count(pio1, sm_enc_a);
          int32_t b = pulse_counter_get_count(pio1, sm_enc_b);

          float pps_a = measure_encoder_speed(a, &last_speed_encoder_count,
                                              &last_speed_calc_time);
          logger_send_encoder_speed(pps_a);
          pps_actual = pps_a;

          if (a != last_encoder_a || b != last_encoder_b) {
            logger_send_encoder_indep_counts(a, b);
            last_encoder_a = a;
            last_encoder_b = b;
          }
        }

        // --- Speed Deviation Warning ---
        if (scl.waiting_for_correction) {
          closed_loop_check_speed(&scl, pps_actual, USE_QUADRATURE_ENCODER);
        }
      }
      counter++;

      sleep_ms(10);
    }

    if (was_moving && emergency_state == EMERGENCY_NORMAL) {
      float missing_um = 0.0f;
      int32_t current_count =
          USE_QUADRATURE_ENCODER
              ? quadrature_encoder_get_count(pio1, sm_enc_q)
              : pulse_counter_get_count(pio1, sm_enc_a);

      // if (closed_loop_calculate_correction(&scl, current_count,
      //                                      USE_QUADRATURE_ENCODER,
      //                                      &missing_um)) {
      //   logger_send_correction_applied(missing_um);
      //   sleep_ms(150); // Mínima pausa antes de corregir
      //   tmc2209_move_linear_um_dma(global_motor, missing_um,
      //                              scl.expected_target_velocity_ums * 0.5f);
      //   continue;
      // } else {
      //   if (!scl.waiting_for_correction) {
      //     logger_send_motor_stopped();
      //   }
      // }
      logger_send_motor_stopped();


    }

    // Reportar encoder en idle si cambió
    if (ENABLE_ENCODER) {
      if (USE_QUADRATURE_ENCODER) {
        int32_t current_count = quadrature_encoder_get_count(pio1, sm_enc_q);

        // Medir velocidad en idle también (probablemente sea cercana a 0, útil
        // para test)
        if (counter % 10 == 0) {
          float pps = measure_encoder_speed(
              current_count, &last_speed_encoder_count, &last_speed_calc_time);
          if (was_moving)
            logger_send_encoder_speed(pps);
        }

        if (current_count != last_encoder_count) {
          logger_send_encoder_count(current_count);
          last_encoder_count = current_count;
        }
      } else {
        int32_t a = pulse_counter_get_count(pio1, sm_enc_a);
        int32_t b = pulse_counter_get_count(pio1, sm_enc_b);

        if (counter % 10 == 0) {
          float pps_a = measure_encoder_speed(a, &last_speed_encoder_count,
                                              &last_speed_calc_time);
          if (was_moving)
            logger_send_encoder_speed(pps_a);
        }

        if (a != last_encoder_a || b != last_encoder_b) {
          logger_send_encoder_indep_counts(a, b);
          last_encoder_a = a;
          last_encoder_b = b;
        }
      }
    }
    counter++;

    sleep_ms(
        10); // Changed from 100 on idle to 10 so we poll properly for timing
  }
}
