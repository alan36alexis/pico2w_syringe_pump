#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "hardware/spi.h"
#include "pico/stdlib.h"

// Componentes del proyecto
#include "core1_main.h"
#include "crosscore_logger.h"
#include "honeywell_spi.h"
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

// Parámetros de resolucion
#define MOTOR_STEPS_PER_REV 200
#define MOTOR_MICROSTEPS_VAL 16
#define MOTOR_MICROSTEPS                                                       \
  TMC2209_MICROSTEPS_2 // 1-> 1600, 2-> 6400, 4-> 12800, 16-> 3200
#define STEPS_PER_REV MOTOR_MICROSTEPS_VAL *MOTOR_STEPS_PER_REV
#define NSTEPS STEPS_PER_REV * 1000
#define STEP_FREQ 6400 * 2   // Frecuencia de pasos en Hz
#define GEARBOX_RATIO 27.00f // Relación de reducción (1.0 = Directo)
#define GEARBOX_STEPS_PER_REV STEPS_PER_REV *GEARBOX_RATIO
#define GEARBOX_NSTEPS GEARBOX_STEPS_PER_REV * 1000
#define GEARBOX_STEP_FREQ 7000
// Selección de Modo: true = Modo UART (Pines fijan dirección), false = Modo
// Pines (Pines fijan pasos)
#define USE_UART_MODE true

// Pines de Finales de Carrera
#define LIMIT_SW_START_PIN 20
#define LIMIT_SW_END_PIN 21

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

// --- Cinematica y Reducción ---
#define REAL_GEARBOX_RATIO 26.85f
#define LEAD_SCREW_PITCH_UM 2000.0f // TR8x2 (2mm por revolución)

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

  if (target_velocity_ums < 50.0f) {
    msteps = TMC2209_MICROSTEPS_16;
    chop_mode = TMC2209_CHOPPER_STEALTHCHOP;
    run_amps = 0.5f;
  } else if (target_velocity_ums <= 500.0f) {
    msteps = TMC2209_MICROSTEPS_8;
    chop_mode = TMC2209_CHOPPER_STEALTHCHOP;
    run_amps = 1.0f;
  } else if (target_velocity_ums <= 800.0f) {
    msteps = TMC2209_MICROSTEPS_4;
    chop_mode = TMC2209_CHOPPER_SPREADCYCLE;
    run_amps = 1.5f;
  } else {
    msteps = TMC2209_MICROSTEPS_2; // > 800 um/s
    chop_mode = TMC2209_CHOPPER_SPREADCYCLE;
    run_amps = 1.5f;
  }

  // Aplicar Conf. al Driver
  tmc2209_set_direction(motor, direction);

  // Detener temporalmente (por si acaso) y configurar driver
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
  uint32_t pasos_aceleracion = (uint32_t)(total_microsteps * 0.15f);
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

/**
 * @brief Funciones y ejecución principal para Core 1 (Baremetal)
 */
void core1_main(void) {
  uint32_t counter = 0;

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
  add_repeating_timer_ms(500, honeywell_timer_callback, NULL, &honeywell_timer);

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
    tmc2209_set_current_amps(&motor1, 1.5f, 0.4f);

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

  // --- EJECUCION DE MOVIMIENTO LINEAL ---
  tmc2209_move_linear_um_dma(&motor1, 53000.0f, 50.0f);

  while (true) {
    while (tmc2209_is_moving(&motor1)) {
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
      sleep_ms(10);
    }
    sleep_ms(100);
  }
}
