#include <math.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stdio.h>

#include "hardware/adc.h"
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
#define DEBUG_MODE 1

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
#define MOTOR_ADDR_PIN_1 MOTOR_MS2_PIN // Bit 1 de direccion
#define MOTOR_ADDR_PIN_0 MOTOR_MS1_PIN // Bit 0 de direccion

// Seleccion de Modo: true = Modo UART (Pines fijan direccion), false = Modo
// Pines (Pines fijan pasos)
#define USE_UART_MODE true

// Pines de Finales de Carrera
#define LIMIT_SW_START_PIN 17 // 27
#define LIMIT_SW_END_PIN 16   // 26

// Pines de ADC
#define ADC_PIN 28

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

const char* get_state_name(Core1State_t state) {
    switch(state) {
        case ST_UNHOMED: return "ST_UNHOMED";
        case ST_HOMING: return "ST_HOMING";
        case ST_TOUCHING_LSW_START: return "ST_TOUCHING_LSW_START";
        case ST_READY_AT_HOME: return "ST_READY_AT_HOME";
        case ST_SEARCHING_SYRINGE: return "ST_SEARCHING_SYRINGE";
        case ST_SYRINGE_ENGAGED: return "ST_SYRINGE_ENGAGED";
        case ST_DISPENSING: return "ST_DISPENSING";
        case ST_DISPENSE_COMPLETED: return "ST_DISPENSE_COMPLETED";
        case ST_SET_NEW_DISPENSE: return "ST_SET_NEW_DISPENSE";
        case ST_SEARCHING_EOT: return "ST_SEARCHING_EOT";
        case ST_TOUCHING_LSW_END: return "ST_TOUCHING_LSW_END";
        case ST_END_OF_TRAVEL: return "ST_END_OF_TRAVEL";
        case ST_FAULT: return "ST_FAULT";
        case ST_OCCLUSION_STOPPING: return "ST_OCCLUSION_STOPPING";
        case ST_OCCLUSION_RELEASE: return "ST_OCCLUSION_RELEASE";
        case ST_OCCLUSION_PAUSED: return "ST_OCCLUSION_PAUSED";
        case ST_MANUAL_OVERRIDE: return "ST_MANUAL_OVERRIDE";
        case ST_CALIB_SEEK_START: return "ST_CALIB_SEEK_START";
        case ST_CALIB_SEEK_END: return "ST_CALIB_SEEK_END";
        case ST_BRAKING_LSW_START: return "ST_BRAKING_LSW_START";
        case ST_BRAKING_LSW_END: return "ST_BRAKING_LSW_END";
        default: return "UNKNOWN_STATE";
    }
}

typedef enum {
  EMERGENCY_NORMAL = 0,
  EMERGENCY_STOPPING,
  EMERGENCY_RETRACTING,
  EMERGENCY_STOPPED
} EmergencyState;

static EmergencyState emergency_state = EMERGENCY_NORMAL;

volatile int32_t calibration_max_encoder_count = 0;

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

// --- Helper para conversion de corriente (Amperes -> CS) ---
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
  LOG_DEBUG("--- Abstraccion de Movimiento Lineal ---\n");
  LOG_DEBUG("Target: %.1f um a %.1f um/s\n", target_um, target_velocity_ums);

  if (target_velocity_ums <= 0.0f || target_um == 0.0f) {
    LOG_DEBUG("Error: Velocidad cero o distancia cero.\n");
    return;
  }

  // 1. Determinar direccion
  bool direction = (target_um > 0.0f);
  target_um = fabsf(target_um); // Trabajar con magnitud absoluta

  // 2. Lookup Table para Configuracion del Motor según Velocidad (um/s)
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

  LOG_DEBUG(">> Configuracion OK: Microsteps=1/%d, Chopper=%s, IRUN=%.1fA\n",
            current_msteps_val,
            (chop_mode == TMC2209_CHOPPER_STEALTHCHOP) ? "StealthChop"
                                                       : "SpreadCycle",
            run_amps);

  // 3. Conversiones Cinemáticas
  // Cuántos micrometros se avanza por cada paso completo del motor (sin
  // microstepping) factor = (Avance del Husillo) / (Pasos por rev del motor *
  // Reduccion)
  float um_per_full_step =
      LEAD_SCREW_PITCH_UM / (MOTOR_STEPS_PER_REV * REAL_GEARBOX_RATIO);

  // Cuántos micrometros se avanza por micropaso
  float um_per_microstep = um_per_full_step / (float)current_msteps_val;

  // Total de micropasos necesarios para alcanzar target_um
  uint32_t total_microsteps = (uint32_t)(target_um / um_per_microstep);

  // Consideraciones para perfiles de aceleracion cortos
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
  // suave, limitando la aceleracion.

  // Porcentajes para pasos (debe sumar total_microsteps)
  uint32_t pasos_aceleracion = (uint32_t)(total_microsteps * 0.02f);
  uint32_t pasos_frenado = pasos_aceleracion;

  // Si nos sobran para hacer 2-part profile
  if (pasos_aceleracion < 20) {
    pasos_aceleracion = 20;
    pasos_frenado = 20;
  }

  uint32_t pasos_constantes =
      total_microsteps - (pasos_aceleracion + pasos_frenado);

  // Si el movimiento es demasiado corto para 15% de aceleracion
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

// --- Funcion para medir la velocidad de giro basada en el encoder ---
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
 * @brief Funciones y ejecucion principal para Core 1 (Baremetal)
 */
void core1_main(void) {
  // Allow Core 0 to pause us during Flash writes
  multicore_lockout_victim_init();

  uint32_t counter = 0;
  int32_t last_encoder_count = 0;
  int32_t last_encoder_a = 0;
  int32_t last_encoder_b = 0;

  // Variables para la medicion de velocidad
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
    // MODO UART: Configuramos los pines como direccion fija (Addr 0: Ambos LOW)
    LOG_DEBUG(
        "Iniciando en MODO UART (Pines MS usados para direccionamiento 0)\n");
    tmc2209_set_uart_address_pins(&motor1, 0);
    LOG_DEBUG("Direccion configurada: %d\n", motor1.addr);
    // Inicializar UART
    tmc2209_setup_uart(&motor1, uart1, 57600, 0, UART_TX_PIN, UART_RX_PIN);
    // --- DIAGNoSTICO UART ---
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
    // Esto deshabilita la funcion de apagado en el pin UART, dejándolo solo
    // para comunicacion.
    tmc2209_set_pdn_disable(&motor1, true);

    // Configurar StealthChop explícitamente (necesario para lectura válida de
    // SG_RESULT)
    tmc2209_set_chopper_mode(&motor1, TMC2209_CHOPPER_DYNAMIC, 100);

    // Configurar TOFF e Interpolacion (CHOPCONF)
    // TOFF=4 (Activa el driver), intpol=true (Suaviza movimiento interpolando a
    // 256 pasos)
    tmc2209_configure_chopconf(&motor1, 2, true);

    // Ejemplo: HSTRT = 4, HEND = 1, TBL = 2 (36 ciclos de reloj)
    tmc2209_set_chopper_parameters(&motor1, 5, 0, 1);

    tmc2209_set_microstepping_uart(&motor1, MOTOR_MICROSTEPS);

    // Configuracion de corriente en Amperes
    // IRUN: 1.0A (~CS 14/15), IHOLD: 0.5A (~CS 7)
    tmc2209_set_current_amps(&motor1, 0.6f, 0.3f);

    // Habilitar el driver AL FINAL de la configuracion para evitar movimientos
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

  // --- INICIALIZACION ADC (Placeholder Presion / Contacto) ---
  adc_init();
  adc_gpio_init(ADC_PIN); // Usaremos GPIO 28 (ADC 2) para el Trimpot
  // --- ADC TEST BLOCKING LOOP ---
  adc_select_input(2);
  // while (true) {
  //   uint16_t raw_val = adc_read();
  //   float voltage = raw_val * 3.3f / 4096.0f;
  //   printf("ADC Test [GPIO 28] - Raw: %u, Volts: %.2f V\n", raw_val,
  //   voltage); sleep_ms(250);
  // }

  // --- VARIABLES DE LA MAQUINA DE ESTADOS (FSM) ---
  Core1State_t current_state = ST_UNHOMED;
  Core1State_t post_lsw_state = ST_UNHOMED;
  Core1Event_t active_event = EV_NONE;

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
    active_event = EV_NONE;

    // 1. Gather Events from IPC
    if (queue_try_remove(&crosscore_cmd_queue, &cmd)) {
      switch (cmd.id) {
      case CMD_HOME:
        active_event = EV_CMD_HOME;
        break;
      case CMD_SEARCH_SYRINGE:
        active_event = EV_CMD_SEARCH_SYRINGE;
        break;
      case CMD_START_DISPENSE:
        active_event = EV_CMD_START_DISPENSE;
        break;
      case CMD_SEARCH_EOT:
        active_event = EV_CMD_SEARCH_EOT;
        break;
      case CMD_RESET:
        active_event = EV_CMD_RESET;
        break;
      case CMD_CONTINUE_DISPENSE:
        active_event = EV_CMD_CONTINUE_DISPENSE;
        break;
      case CMD_OCC_RELEASE:
        active_event = EV_CMD_OCC_RELEASE;
        break;
      case CMD_RESUME_DISPENSE:
        active_event = EV_CMD_RESUME_DISPENSE;
        break;
      case CMD_CALIBRATE:
        active_event = EV_CMD_CALIBRATE;
        break;

      // Handle raw config/debug commands manually, override FSM
      case CMD_MOVE_LINEAR_UM:
        current_state = ST_MANUAL_OVERRIDE;
        RESET_ENCODER_COUNTS();
        logger_send_motor_moving();
        tmc2209_move_linear_um_dma(global_motor,
                                   cmd.payload.move_linear.target_um,
                                   cmd.payload.move_linear.target_velocity_ums);
        closed_loop_init_move(&scl, cmd.payload.move_linear.target_um,
                              cmd.payload.move_linear.target_velocity_ums, 0);
        break;
      case CMD_STOP_MOTOR:
        current_state = ST_UNHOMED;
        if (tmc2209_is_moving(global_motor)) {
          float current_freq = tmc2209_get_current_freq_hz(global_motor);
          tmc2209_stop_from_current_freq_dma(global_motor, 200,
                                             current_freq * 0.5f, 200, 50.0f);
        } else {
          tmc2209_stop(global_motor);
        }
        break;
      case CMD_MOVE_2PART_PROFILE:
        current_state = ST_MANUAL_OVERRIDE;
        RESET_ENCODER_COUNTS();
        logger_send_motor_moving();
        tmc2209_move_2part_profile_dma(
            global_motor, cmd.payload.move_2part.start_freq,
            cmd.payload.move_2part.a_p1, cmd.payload.move_2part.f_mid_accel,
            cmd.payload.move_2part.a_p2, cmd.payload.move_2part.f_target,
            cmd.payload.move_2part.c_steps, cmd.payload.move_2part.d_p1,
            cmd.payload.move_2part.f_mid_decel, cmd.payload.move_2part.d_p2,
            cmd.payload.move_2part.f_end);
        {
          float um_per_microstep =
              LEAD_SCREW_PITCH_UM /
              (MOTOR_STEPS_PER_REV * REAL_GEARBOX_RATIO * MOTOR_MICROSTEPS_VAL);
          float vel_ums = cmd.payload.move_2part.f_target * um_per_microstep;
          closed_loop_init_move(&scl, 0.0f, vel_ums, 0);
        }
        break;
      case CMD_HOME_START:
        current_state = ST_MANUAL_OVERRIDE;
        RESET_ENCODER_COUNTS();
        logger_send_motor_moving();
        tmc2209_move_linear_um_dma(global_motor, -105000.0f, 1200.0f);
        break;
      case CMD_HOME_END:
        current_state = ST_MANUAL_OVERRIDE;
        RESET_ENCODER_COUNTS();
        logger_send_motor_moving();
        tmc2209_move_linear_um_dma(global_motor, 105000.0f, 1200.0f);
        break;
      case CMD_MOVE_NSTEPS:
        current_state = ST_MANUAL_OVERRIDE;
        RESET_ENCODER_COUNTS();
        logger_send_motor_moving();
        if (cmd.payload.move_nsteps.nsteps < 0) {
          tmc2209_set_direction(global_motor, false);
          tmc2209_send_nsteps_at_freq(global_motor,
                                      -cmd.payload.move_nsteps.nsteps,
                                      cmd.payload.move_nsteps.freq_hz);
        } else {
          tmc2209_set_direction(global_motor, true);
          tmc2209_send_nsteps_at_freq(global_motor,
                                      cmd.payload.move_nsteps.nsteps,
                                      cmd.payload.move_nsteps.freq_hz);
        }
        break;
      case CMD_STOP_IMMEDIATE:
        current_state = ST_UNHOMED;
        tmc2209_stop(global_motor);
        break;
      default:
        break;
      }
    }

    // 2. Gather Events from Hardware / Motor
    static uint8_t start_sw_debounce = 0;
    static uint8_t end_sw_debounce = 0;
    const uint8_t DEBOUNCE_THRESHOLD = 3;

    if (global_motor->limit_switches_enabled) {
      bool start_sw_active = gpio_get(global_motor->limit_switch_start_pin);
      bool end_sw_active = gpio_get(global_motor->limit_switch_end_pin);

      if (start_sw_active)
        start_sw_debounce++;
      else
        start_sw_debounce = 0;
      if (end_sw_active)
        end_sw_debounce++;
      else
        end_sw_debounce = 0;

      if (start_sw_debounce >= DEBOUNCE_THRESHOLD && !global_motor->direction &&
          current_state != ST_BRAKING_LSW_START && current_state != ST_TOUCHING_LSW_START)
        active_event = iEV_LSW_START_HIT;
      else if (end_sw_debounce >= DEBOUNCE_THRESHOLD && global_motor->direction &&
               current_state != ST_BRAKING_LSW_END && current_state != ST_TOUCHING_LSW_END)
        active_event = iEV_LSW_END_HIT;
      else if (start_sw_debounce == 0 && current_state == ST_TOUCHING_LSW_START)
        active_event = iEV_LSW_START_RELEASED;
      else if (end_sw_debounce == 0 && current_state == ST_TOUCHING_LSW_END)
        active_event = iEV_LSW_END_RELEASED;
    }

    if (current_state == ST_SEARCHING_SYRINGE ||
        current_state == ST_DISPENSING ||
        current_state == ST_OCCLUSION_RELEASE) {
      uint16_t adc_val = adc_read();
      float voltage = adc_val * 3.3f / (1 << 12);

      // Imprimir el voltaje cada ~500ms basado en el `counter` global.
      // Dado que el loop tiene un sleep_ms(10), 50 iteraciones son aprox 500ms.
      if (counter % 50 == 0) {
        LOG_DEBUG("ADC Voltage (State %d): %.2f V\n", current_state, voltage);
      }

      // Sensibilidad Simulada
      if (voltage > 2.0f && current_state == ST_SEARCHING_SYRINGE) {
        active_event = iEV_CONTACT_DETECTED;
      } else if (voltage > 3.0f && current_state == ST_DISPENSING) {
        active_event = iEV_OCCLUSION_DETECTED;
      } else if (voltage < 2.0f && current_state == ST_OCCLUSION_RELEASE) {
        active_event = iEV_OCC_RELEASED;
      }
    }

    static bool was_moving = false;
    bool is_moving = tmc2209_is_moving(global_motor);
    if (was_moving && !is_moving) {
      if (active_event != iEV_LSW_START_HIT &&
          active_event != iEV_LSW_END_HIT) {
        active_event = iEV_TARGET_REACHED;
      }
      logger_send_motor_stopped();
    }
    was_moving = is_moving;

    // 3. Evaluate FSM (State transitions based on events)
    static Core1State_t previous_state = ST_UNHOMED;
    if (current_state != previous_state) {
      LOG_DEBUG("[FSM] State changed: %s -> %s\n", get_state_name(previous_state), get_state_name(current_state));
      previous_state = current_state;
    }

    switch (current_state) {
    case ST_UNHOMED:
      if (active_event == EV_CMD_HOME) {
        RESET_ENCODER_COUNTS();
        tmc2209_move_linear_um_dma(global_motor, -105000.0f,
                                   600.0f); // TODO: Aumentar velocidad a >1000
        current_state = ST_HOMING;
      } else if (active_event == EV_CMD_CALIBRATE) {
        tmc2209_move_linear_um_dma(global_motor, -105000.0f, 1200.0f);
        current_state = ST_CALIB_SEEK_START;
      }
      break;

    case ST_CALIB_SEEK_START:
      if (active_event == iEV_LSW_START_HIT) {
        RESET_ENCODER_COUNTS();
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_CALIB_SEEK_END;
        current_state = ST_BRAKING_LSW_START;
      }
      break;

    case ST_CALIB_SEEK_END:
      if (active_event == iEV_LSW_END_HIT) {
        int32_t current_enc = USE_QUADRATURE_ENCODER
                                  ? quadrature_encoder_get_count(pio1, sm_enc_q)
                                  : pulse_counter_get_count(pio1, sm_enc_a);
        calibration_max_encoder_count = current_enc;
        LOG_DEBUG("Calibration Complete! Max Encoder Count: %d\n", calibration_max_encoder_count);
        
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_END_OF_TRAVEL;
        current_state = ST_BRAKING_LSW_END;
      }
      break;

    case ST_HOMING:
      if (active_event == iEV_LSW_START_HIT) {
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_READY_AT_HOME;
        current_state = ST_BRAKING_LSW_START;
      }
      break;

    case ST_BRAKING_LSW_START:
      if (active_event == iEV_TARGET_REACHED) {
        tmc2209_move_linear_um_dma(global_motor, 105000.0f, 50.0f);
        current_state = ST_TOUCHING_LSW_START;
      }
      break;

    case ST_BRAKING_LSW_END:
      if (active_event == iEV_TARGET_REACHED) {
        tmc2209_move_linear_um_dma(global_motor, -105000.0f, 50.0f);
        current_state = ST_TOUCHING_LSW_END;
      }
      break;

    case ST_TOUCHING_LSW_START:
      if (active_event == iEV_LSW_START_RELEASED) {
        tmc2209_stop(global_motor);
        if (post_lsw_state == ST_CALIB_SEEK_END) {
            tmc2209_move_linear_um_dma(global_motor, 105000.0f, 1200.0f);
        }
        current_state = post_lsw_state;
      }
      break;

    case ST_READY_AT_HOME:
      if (active_event == EV_CMD_SEARCH_SYRINGE) {
        // Avanzar buscando contacto
        tmc2209_move_linear_um_dma(global_motor, 105000.0f, 1200.0f);
        current_state = ST_SEARCHING_SYRINGE;
      }
      break;

    case ST_SEARCHING_SYRINGE:
      if (active_event == iEV_CONTACT_DETECTED) {
        tmc2209_stop(global_motor);
        current_state = ST_SYRINGE_ENGAGED;
      } else if (active_event == iEV_LSW_END_HIT) {
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_END_OF_TRAVEL;
        current_state = ST_BRAKING_LSW_END;
      }
      break;

    case ST_SYRINGE_ENGAGED:
      if (active_event == EV_CMD_START_DISPENSE) {
        RESET_ENCODER_COUNTS();
        tmc2209_move_linear_um_dma(
            global_motor, cmd.payload.start_dispense.target_um,
            cmd.payload.start_dispense.target_velocity_ums);
        closed_loop_init_move(&scl, cmd.payload.start_dispense.target_um,
                              cmd.payload.start_dispense.target_velocity_ums,
                              0);
        current_state = ST_DISPENSING;
      }
      break;

    case ST_DISPENSING:
      if (active_event == iEV_TARGET_REACHED) {
        float missing_um = 0.0f;
        int32_t current_enc = USE_QUADRATURE_ENCODER
                                  ? quadrature_encoder_get_count(pio1, sm_enc_q)
                                  : pulse_counter_get_count(pio1, sm_enc_a);
        if (closed_loop_calculate_correction(
                &scl, current_enc, USE_QUADRATURE_ENCODER, &missing_um)) {
          LOG_DEBUG("Closed loop: Faltan %.1f um. Aplicando correccion...\n",
                    missing_um);
          tmc2209_move_linear_um_dma(global_motor, missing_um,
                                     scl.expected_target_velocity_ums);
        } else {
          current_state = ST_DISPENSE_COMPLETED;
        }
      } else if (active_event == iEV_LSW_END_HIT) {
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_END_OF_TRAVEL;
        current_state = ST_BRAKING_LSW_END;
      } else if (active_event == iEV_OCCLUSION_DETECTED) {
        tmc2209_stop(global_motor);
        current_state = ST_OCCLUSION_STOPPING;
      }
      break;

    case ST_DISPENSE_COMPLETED:
      if (active_event == EV_CMD_RESET) {
        current_state = ST_UNHOMED;
      } else if (active_event == EV_CMD_CONTINUE_DISPENSE) {
        current_state = ST_SET_NEW_DISPENSE;
      } else if (active_event == EV_CMD_SEARCH_EOT) {
        tmc2209_move_linear_um_dma(global_motor, 105000.0f, 1200.0f);
        current_state = ST_SEARCHING_EOT;
      }
      break;

    case ST_SET_NEW_DISPENSE:
      if (active_event == EV_CMD_START_DISPENSE) {
        RESET_ENCODER_COUNTS();
        tmc2209_move_linear_um_dma(
            global_motor, cmd.payload.start_dispense.target_um,
            cmd.payload.start_dispense.target_velocity_ums);
        closed_loop_init_move(&scl, cmd.payload.start_dispense.target_um,
                              cmd.payload.start_dispense.target_velocity_ums,
                              0);
        current_state = ST_DISPENSING;
      }
      break;

    case ST_SEARCHING_EOT:
      if (active_event == iEV_LSW_END_HIT) {
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_END_OF_TRAVEL;
        current_state = ST_BRAKING_LSW_END;
      }
      break;

    case ST_TOUCHING_LSW_END:
      if (active_event == iEV_LSW_END_RELEASED) {
        tmc2209_stop(global_motor);
        current_state = ST_END_OF_TRAVEL;
      }
      break;

    case ST_END_OF_TRAVEL:
      if (active_event == EV_CMD_RESET) {
        current_state = ST_UNHOMED;
      }
      break;

    case ST_OCCLUSION_STOPPING:
      if (active_event == EV_CMD_OCC_RELEASE) {
        // Mover hacia atrás continuamente para liberar presion (hasta caer
        // debajo de 2.0V)
        tmc2209_move_linear_um_dma(global_motor, -105000.0f, 200.0f);
        current_state = ST_OCCLUSION_RELEASE;
      }
      break;

    case ST_OCCLUSION_RELEASE:
      if (active_event == iEV_OCC_RELEASED) {
        tmc2209_stop(global_motor);
        current_state = ST_OCCLUSION_PAUSED;
      } else if (active_event == iEV_LSW_START_HIT) {
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_UNHOMED;
        current_state = ST_BRAKING_LSW_START;
      }
      break;

    case ST_OCCLUSION_PAUSED:
      if (active_event == EV_CMD_RESUME_DISPENSE) {
        current_state = ST_DISPENSING;
      }
      break;

    case ST_MANUAL_OVERRIDE:
      if (active_event == EV_CMD_RESET) {
        tmc2209_stop(global_motor);
        current_state = ST_UNHOMED;
      } else if (active_event == iEV_TARGET_REACHED) {
        float missing_um = 0.0f;
        int32_t current_enc = USE_QUADRATURE_ENCODER
                                  ? quadrature_encoder_get_count(pio1, sm_enc_q)
                                  : pulse_counter_get_count(pio1, sm_enc_a);
        if (closed_loop_calculate_correction(
                &scl, current_enc, USE_QUADRATURE_ENCODER, &missing_um)) {
          LOG_DEBUG(
              "Closed loop manual: Faltan %.1f um. Aplicando correccion...\n",
              missing_um);
          tmc2209_move_linear_um_dma(global_motor, missing_um,
                                     scl.expected_target_velocity_ums);
        }
      } else if (active_event == iEV_LSW_START_HIT) {
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_UNHOMED;
        current_state = ST_BRAKING_LSW_START;
      } else if (active_event == iEV_LSW_END_HIT) {
        float current_freq = tmc2209_get_current_freq_hz(global_motor);
        uint32_t pulses = (uint32_t)(current_freq * 0.5f);
        if (pulses < 100) pulses = 100;
        tmc2209_stop_from_current_freq_dma(global_motor, pulses, current_freq * 0.5f, pulses, 50.0f);
        post_lsw_state = ST_UNHOMED;
        current_state = ST_BRAKING_LSW_END;
      }
      break;

    default:
      break;
    }

    // Fallbacks globales removidos en favor de la FSM y logica nativa de
    // ST_MANUAL_OVERRIDE

    uint32_t drv_status = tmc2209_read_drv_status(global_motor);
    uint32_t gstat = tmc2209_read_gstat(global_motor);
    uint16_t stall = tmc2209_read_sg_result(global_motor);

    if (gstat & 0x01)
      tmc2209_clear_gstat(global_motor, 1);
    if (gstat & 0x02)
      tmc2209_clear_gstat(global_motor, 2);
    if (gstat & 0x04)
      tmc2209_clear_gstat(global_motor, 4);

    if (gstat || drv_status || stall) {
      logger_send_drv_status_error(stall, drv_status, gstat);
    }

    if (counter % 10 == 0) {
      float pct = tmc2209_get_move_progress_pct(global_motor);
      logger_send_motor_progress(pct);
    }

    if (ENABLE_ENCODER) {
      if (USE_QUADRATURE_ENCODER) {
        int32_t current_count = quadrature_encoder_get_count(pio1, sm_enc_q);

        if (counter % 10 == 0) {
          float pps = measure_encoder_speed(
              current_count, &last_speed_encoder_count, &last_speed_calc_time);
          if (is_moving) {
            if (counter % 50 == 0) {
              float pulses_per_rev = (ENCODER_LINES_PER_REV * 4.0f);
              float um_per_pulse = LEAD_SCREW_PITCH_UM / pulses_per_rev;
              logger_send_encoder_speed(fabsf(pps) * um_per_pulse);
            }
            closed_loop_check_speed(&scl, pps, USE_QUADRATURE_ENCODER);
          }
        }

        if (current_count != last_encoder_count) {
          if (counter % 50 == 0) {
            if (is_moving)
              logger_send_encoder_count(current_count);
          }
          last_encoder_count = current_count;
        }
      } else {
        int32_t a = pulse_counter_get_count(pio1, sm_enc_a);
        int32_t b = pulse_counter_get_count(pio1, sm_enc_b);

        if (counter % 10 == 0) {
          float pps_a = measure_encoder_speed(a, &last_speed_encoder_count,
                                              &last_speed_calc_time);
          if (is_moving) {
            if (counter % 50 == 0) {
              float pulses_per_rev = (float)ENCODER_LINES_PER_REV;
              float um_per_pulse = LEAD_SCREW_PITCH_UM / pulses_per_rev;
              logger_send_encoder_speed(fabsf(pps_a) * um_per_pulse);
            }
            closed_loop_check_speed(&scl, pps_a, USE_QUADRATURE_ENCODER);
          }
        }

        if (a != last_encoder_a || b != last_encoder_b) {
          if (counter % 50 == 0) {
            if (is_moving)
              logger_send_encoder_indep_counts(a, b);
          }
          last_encoder_a = a;
          last_encoder_b = b;
        }
      }
    }
    counter++;

    sleep_ms(10);
  }
}
