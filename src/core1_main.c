#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"

// Componentes del proyecto
#include "core1_main.h"
#include "test_modes.h"
#include "tmc2209.h"
#include "honeywell_spi.h"

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
#define MOTOR_ADDR_PIN_1 MOTOR_MS2_PIN // Bit 1 de dirección
#define MOTOR_ADDR_PIN_0 MOTOR_MS1_PIN // Bit 0 de dirección

// Parámetros de resolucion
#define MOTOR_STEPS_PER_REV 200
#define MOTOR_MICROSTEPS_VAL 16
#define MOTOR_MICROSTEPS TMC2209_MICROSTEPS_2 // 1-> 1600, 2-> 6400, 4-> 12800, 16-> 3200
#define STEPS_PER_REV MOTOR_MICROSTEPS_VAL * MOTOR_STEPS_PER_REV
#define NSTEPS STEPS_PER_REV * 1000
#define STEP_FREQ 6400 * 2   // Frecuencia de pasos en Hz
#define GEARBOX_RATIO 27.00f // Relación de reducción (1.0 = Directo)
#define GEARBOX_STEPS_PER_REV STEPS_PER_REV * GEARBOX_RATIO
#define GEARBOX_NSTEPS GEARBOX_STEPS_PER_REV * 1000
#define GEARBOX_STEP_FREQ 7000
// Selección de Modo: true = Modo UART (Pines fijan dirección), false = Modo Pines (Pines fijan pasos)
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
    LOG_DEBUG("Status: %d, Pressure: %.2f psi (%.2f mmHg)\n", data.status,
              data.pressure_psi, pressure_mmhg);

    switch (emergency_state) {
    case EMERGENCY_NORMAL:
      if (data.status == 0 && data.pressure_psi > 20.0f) {
        if (global_motor != NULL && tmc2209_is_moving(global_motor)) {
          LOG_DEBUG(
              "ALERTA: Sobrepresion (%.2f PSI). Frenando para retroceder!\n",
              data.pressure_psi);
          tmc2209_stop_s_curve_dma(global_motor, 0.0f, 20);
          emergency_state = EMERGENCY_STOPPING;
        }
      }
      break;

    case EMERGENCY_STOPPING:
      if (global_motor != NULL && !tmc2209_is_moving(global_motor)) {
        LOG_DEBUG("Motor detenido. Iniciando retroceso...\n");
        bool inv_dir = !global_motor->direction;
        tmc2209_set_direction(global_motor, inv_dir);
        // Retroceso continuo (e.g. 3000 Hz)
        tmc2209_start_s_curve_dma(global_motor, 500.0f, 3000.0f, 0.5f, 100, 2);
        emergency_state = EMERGENCY_RETRACTING;
      }
      break;

    case EMERGENCY_RETRACTING:
      if (data.status == 0 && data.pressure_psi < 15.0f) {
        LOG_DEBUG("Presion segura (%.2f PSI). Deteniendo definitivamente.\n",
                  data.pressure_psi);
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

void move_back_and_forth_limits(TMC2209_t *motor, float freq_cw,
                                float freq_ccw) {
  // Deshabilitar el manejo automático de finales de carrera del driver
  // para manejarlo manualmente aquí con lógica de dirección
  motor->limit_switches_enabled = false;

  // Estado inicial: Hacia el inicio (False)
  bool direction = false;

  // Parámetros de la curva S
  // Escalamiento correpondiente a 256/16 = 16
  const uint32_t RAMP_STEPS_ACCEL = 1000 / 16; // Pasos para acelerar
  const uint32_t RAMP_STEPS_DECEL = 1000 / 16; // Pasos para frenar
  const float START_FREQ_HZ =
      1000.0f / 16.0f; // Frecuencia inicial de arranque suave
  const int S_CURVE_AGGRESSIVENESS =
      3; // Agresividad de la curva (1=Lineal, 5=Muy Agresiva)

  LOG_DEBUG("Iniciando bucle de va y ven con LIMIT SW (Curva S)...\n");

  while (true) {
    // Configurar dirección
    tmc2209_set_direction(motor, direction);

    // Seleccionar frecuencia objetivo según dirección
    float target_freq = direction ? freq_ccw : freq_cw;

    // Iniciar movimiento con aceleración en Curva S
    // Iniciar movimiento
    LOG_DEBUG("Iniciando movimiento %s -> Target: %.1f Hz\n",
              direction ? "CCW" : "CW", target_freq);

    // Registrar tiempo de inicio para cálculo de pasos
    absolute_time_t start_time = get_absolute_time();

    tmc2209_start_s_curve_dma(motor, START_FREQ_HZ, target_freq, 0.5f,
                              RAMP_STEPS_ACCEL, S_CURVE_AGGRESSIVENESS);

    // Bucle de polling activo
    bool stop_triggered = false;
    while (tmc2209_is_moving(motor)) {
      // Leer finales de carrera (Lógica negativa: 0 = Activado)
      bool start_hit = !gpio_get(motor->limit_switch_start_pin);
      bool end_hit = !gpio_get(motor->limit_switch_end_pin);

      // Check if any limit is hit to calculate steps BEFORE stopping logic (for
      // precision)
      if ((!direction && start_hit) || (direction && end_hit)) {
        absolute_time_t end_time = get_absolute_time();
        int64_t total_us = absolute_time_diff_us(start_time, end_time);

        // Cálculo de pasos estimados
        // 1. Calcular tiempo de rampa de aceleración
        float avg_v = (START_FREQ_HZ + target_freq) / 2.0f;
        float accel_time_s = (float)RAMP_STEPS_ACCEL / avg_v;
        int64_t accel_time_us = (int64_t)(accel_time_s * 1000000.0f);

        long estimated_steps = 0;
        if (total_us <= accel_time_us) {
          // Si el movimiento duró menos que la rampa (muy corto)
          // Aproximación lineal simple (aunque es curva S)
          float frac = (float)total_us / (float)accel_time_us;
          estimated_steps = (long)(RAMP_STEPS_ACCEL * frac);
        } else {
          // Rampa completa + Estado estacionario
          int64_t steady_us = total_us - accel_time_us;
          long steady_steps = (long)((steady_us / 1000000.0f) * target_freq);
          estimated_steps = RAMP_STEPS_ACCEL + steady_steps;
        }
        LOG_DEBUG(">>> Pasos estimados (%s): %ld (Tiempo: %lld ms)\n",
                  direction ? "IDA" : "VUELTA", estimated_steps,
                  total_us / 1000);
      }

      if (!direction) { // Moviendo hacia Inicio (False)
        if (start_hit) {
          LOG_DEBUG("Tope INICIO alcanzado. Iniciando frenado suave...\n");
          stop_triggered = true;
          // Frenado suave en Curva S hasta 0 Hz
          tmc2209_stop_s_curve_dma(motor, 0.0f, RAMP_STEPS_DECEL);

          // Esperar a que el motor se detenga completamente antes de cambiar
          // dirección
          while (tmc2209_is_moving(motor)) {
            sleep_ms(1);
          }

          sleep_ms(200);    // Pausa extra de seguridad
          direction = true; // Cambiar a True (Hacia Fin)
          break;            // Salir del polling para reiniciar movimiento
        }
      } else { // Moviendo hacia Fin (True)
        if (end_hit) {
          LOG_DEBUG("Tope FIN alcanzado. Iniciando frenado suave...\n");
          stop_triggered = true;
          // Frenado suave en Curva S hasta 0 Hz
          tmc2209_stop_s_curve_dma(motor, 0.0f, RAMP_STEPS_DECEL);

          // Esperar a que el motor se detenga completamente antes de cambiar
          // dirección
          while (tmc2209_is_moving(motor)) {
            sleep_ms(1);
          }

          sleep_ms(200);     // Pausa extra de seguridad
          direction = false; // Cambiar a False (Hacia Inicio)
          break;             // Salir del polling para reiniciar movimiento
        }
      }
      uint16_t stall = tmc2209_read_sg_result(motor);
      LOG_DEBUG("$%u;", stall);

      sleep_ms(10); // Evitar saturación
    }
  }
}

void move_back_and_forth_steps(TMC2209_t *motor, float freq_cw, int steps_cw,
                               float freq_ccw, int steps_ccw) {
  LOG_DEBUG("Iniciando bucle de va y ven por PASOS (Sin Limit SW)...\n");

  // Estado inicial: Hacia el inicio (False)
  bool direction = false;

  while (true) {
    // Configurar dirección
    tmc2209_set_direction(motor, direction);

    float target_freq = direction ? freq_ccw : freq_cw;
    int target_steps = direction ? steps_ccw : steps_cw;

    LOG_DEBUG("Moviendo %d pasos %s a %.1f Hz\n", target_steps,
              direction ? "CCW" : "CW", target_freq);

    // Usar modo pasos fijos (Burst mode - velocidad constante por ahora)
    // Nota: tmc2209_send_nsteps_at_freq usa PIO en modo burst
    tmc2209_send_nsteps_at_freq(motor, target_steps, target_freq);

    // Esperar a que termine el movimiento
    while (tmc2209_is_moving(motor)) {
      sleep_ms(10);
    }

    sleep_ms(500); // Pausa entre movimientos

    // Cambiar dirección
    direction = !direction;
  }
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

void move_back_and_forth_profile_dma(TMC2209_t *motor) {
  LOG_DEBUG(
      "Iniciando prueba de perfil completo IDA y VUELTA (Sin Limit SW)...\n");

  bool direction = false;

  // Frecuencias y pasos de ejemplo
  float f_start = 50.0f;
  float f_mid_accel = 500.0f;
  float f_target = 1000.0f;
  float f_mid_decel = 500.0f;
  float f_end = 50.0f;

  uint32_t a_p1 = 100;  // Pasos de Aceleracion 1
  uint32_t a_p2 = 100;  // Pasos de Aceleracion 2
  uint32_t s_p = 20000; // Pasos de Crucero Constante
  uint32_t d_p1 = 100;  // Pasos de Deceleracion 1
  uint32_t d_p2 = 100;  // Pasos de Deceleracion 2

  uint16_t stall = 0;

  while (true) {
    tmc2209_set_direction(motor, direction);

    LOG_DEBUG("Lanzando perfil 2-partes %s. Total Pasos: %u\n",
              direction ? "CCW" : "CW", a_p1 + a_p2 + s_p + d_p1 + d_p2);

    tmc2209_move_2part_profile_dma(motor, f_start, a_p1, f_mid_accel, a_p2,
                                   f_target, s_p, d_p1, f_mid_decel, d_p2,
                                   f_end);

    // Bloquear mientras el DMA hace ping pong con el perfil
    while (tmc2209_is_moving(motor)) {
      sleep_ms(10);
      stall = tmc2209_read_sg_result(motor);
      LOG_DEBUG("$%u;", stall);
    }

    LOG_DEBUG("Fin del perfil. Pausa de 1 seg...\n");
    sleep_ms(1000); // Pausa entre idas y vueltas

    direction = !direction;
  }
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
      LOG_DEBUG("⚠️ ERROR CRÍTICO: No hay comunicación UART (Lectura = "
                "0).\nRevisar conexión TX/RX y alimentación VM.\n");
    } else {
      LOG_DEBUG("✅ Conexión UART Exitosa. IOIN: 0x%08X\n", check_uart);
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
    LOG_DEBUG("Microsteps Leido: %d\n", msteps_read);
  } else {
    // MODO PINES: Configuramos los pines para microstepping
    LOG_DEBUG("Iniciando en MODO PINES (Pines MS usados para microstepping)\n");
    tmc2209_set_microstepping_by_pins(&motor1, MOTOR_MICROSTEPS);
  }

  // --- SELECCION DE MODO DE EJECUCION ---
  // 0: Modo Normal (Limit Switches)
  // 1: Modo Pasos Fijos
  // 2: Test RPM Demo
  // 3: Test StallGuard Analysis
  // 4: Test Microstepping Cycle
  // 5: Test Single Function 2-Part Profile (Ida y Vuelta Autónomo)
  int run_mode = 7;

  switch (run_mode) {
  case 0:
    move_back_and_forth_limits(&motor1, 6250.0f, 6250.0f);
    break;
  case 1:
    move_back_and_forth_steps(&motor1, 100.0f, 20000, 100.0f, 10000);
    break;
  case 2:
    while (1)
      test_rpm_movement_demo(&motor1);
    break;
  case 3:
    test_stallguard_analysis(&motor1);
    break;
  case 4:
    while (1)
      test_microstepping_cycle(&motor1, USE_UART_MODE);
    break;
  case 5:
    move_back_and_forth_profile_dma(&motor1);
    break;
  case 6:
    tmc2209_set_direction(&motor1, true);
    // tmc2209_move_2part_profile_dma(&motor1, 500.0f, 5000, 3000.0f, 15000,
    //                                5000.0f, 500000, 5000, 3000.0f, 5000,
    //                                500.0f); //1/4
    tmc2209_move_2part_profile_dma(&motor1, 500.0f, 3000, 2000.0f, 5000,
                                   5300.0f, 500000, 3000, 1200.0f, 5000,
                                   500.0f); // 1/2
    // tmc2209_move_2part_profile_dma(&motor1, 100.0f, 1000, 200.0f, 3000,
    // 500.0f,
    //                                50000, 3000, 200.0f, 5000,
    //                                100.0f); // 1/2
    while (tmc2209_is_moving(&motor1)) {

      uint32_t drv_status = tmc2209_read_drv_status(&motor1);
      uint32_t gstat = tmc2209_read_gstat(&motor1);
      uint16_t stall = tmc2209_read_sg_result(&motor1);

      // Detectar y reportar errores
      if (gstat & 0x01) {
        LOG_DEBUG("⚠️ GSTAT: Reset detectado.\n");
        tmc2209_clear_gstat(&motor1, 1);
      }
      if (gstat & 0x02) {
        LOG_DEBUG("⚠️ GSTAT: DRV_ERR (Falla en driver).\n");
        tmc2209_clear_gstat(&motor1, 2);
      }
      if (gstat & 0x04) {
        LOG_DEBUG("⚠️ GSTAT: UV_CP (Bomba de carga baja).\n");
        tmc2209_clear_gstat(&motor1, 4);
      }

      if (drv_status & 0x00000001) {
        LOG_DEBUG("⚠️ DRV_STATUS: OLA (Ola abierta A).\n");
      }
      if (drv_status & 0x00000002) {
        LOG_DEBUG("⚠️ DRV_STATUS: OLB (Ola abierta B).\n");
      }
      if (drv_status & 0x00000004) {
        LOG_DEBUG("⚠️ DRV_STATUS: OTW (Sobre temp).\n");
      }
      if (drv_status & 0x00000008) {
        LOG_DEBUG("⚠️ DRV_STATUS: OT (Sobre temp critica).\n");
      }
      if (drv_status & 0x00000010) {
        LOG_DEBUG("⚠️ DRV_STATUS: S2G (Corto a tierra A).\n");
      }
      if (drv_status & 0x00000020) {
        LOG_DEBUG("⚠️ DRV_STATUS: S2G (Corto a tierra B).\n");
      }
      if (drv_status & 0x00000040) {
        LOG_DEBUG("⚠️ DRV_STATUS: S2V (Corto a VM A).\n");
      }
      if (drv_status & 0x00000080) {
        LOG_DEBUG("⚠️ DRV_STATUS: S2V (Corto a VM B).\n");
      }
      if (drv_status & 0x00000100) {
        LOG_DEBUG("⚠️ DRV_STATUS: StallGuard detected.\n");
      }

      LOG_DEBUG("Stall: %u | DRV: 0x%X | GSTAT: 0x%X\n", stall, drv_status,
                gstat);

      sleep_ms(10);
    }
    while (true)
      ;
    break;
  case 7:
    tmc2209_move_linear_um_dma(&motor1, -53000.0f, 1000.0f);
    while (tmc2209_is_moving(&motor1)) {
      sleep_ms(10);
      
      // Enviar contador de prueba hacia el core 0 por FIFO a modo de Heartbeat
      counter++;
      if (multicore_fifo_wready()) {
          multicore_fifo_push_blocking(counter);
      }
    }
    while (true)
      ;
    break;
  default:
    move_back_and_forth_limits(&motor1, 6250.0f, 6250.0f);
    break;
  }
}
