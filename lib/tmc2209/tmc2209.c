#include "tmc2209.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/regs/uart.h"
#include "hardware/structs/uart.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "tmc2209.pio.h" // Header generado automáticamente por CMake

#include <math.h>
#include <stdio.h>

// Direcciones de registros TMC2209
#define TMC2209_REG_IHOLD_IRUN 0x10
#define TMC2209_REG_TPOWERDOWN 0x11 // TPOWERDOWN
#define TMC2209_REG_SGTHRS 0x40     // StallGuard4 Threshold
#define TMC2209_REG_SG_RESULT 0x41  // StallGuard4 Readout
#define TMC2209_REG_IOIN 0x06       // Inputs
#define TMC2209_REG_GCONF 0x00      // Global Configuration
#define TMC2209_REG_GSTAT 0x01      // Global Status
#define TMC2209_REG_TPWMTHRS 0x13   // StealthChop PWM Threshold
#define TMC2209_REG_TCOOLTHRS 0x14  // CoolStep Lower Velocity Threshold
#define TMC2209_REG_CHOPCONF 0x6C   // Chopper Configuration
#define TMC2209_REG_DRV_STATUS 0x6F // Driver Status
#define TMC2209_REG_VACTUAL 0x22    // VACTUAL (Motion Controller)

#define TMC2209_UART_DELAY_US 100

// Mapa para enrutar interrupciones DMA a la instancia correcta
static TMC2209_t *g_dma_ctx_map[NUM_DMA_CHANNELS] = {NULL};

// Helpers para cálculo de ciclos PIO (tomados de stepgen.c)
static inline uint32_t clamp_u32(uint64_t v, uint32_t lo, uint32_t hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return (uint32_t)v;
}

static inline void split_total_cycles(uint32_t total_cycles, float duty_cycle,
                                      uint32_t *high_cycles_out,
                                      uint32_t *low_cycles_out) {
  if (total_cycles < 3u)
    total_cycles = 3u;

  uint32_t high_cycles = (uint32_t)((float)total_cycles * duty_cycle);
  if (high_cycles < 1u)
    high_cycles = 1u;
  if (high_cycles >= total_cycles)
    high_cycles = total_cycles - 1u;

  *high_cycles_out = high_cycles;
  *low_cycles_out = total_cycles - high_cycles;
}

static inline float get_curve_factor(float t, int aggressiveness) {
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;

  // Si agresividad es 1, usamos lineal (Trapezoidal)
  if (aggressiveness <= 1)
    return t;

  // Fórmula sigmoide ajustable: x^k / (x^k + (1-x)^k)
  // k = agresividad
  float p = (float)aggressiveness;
  // Evitamos pow si es 2 o 3 para eficiencia, O usamos pow directamente
  // Para simplificar y dar soporte hasta 5, usamos powf
  float t_p = powf(t, p);
  float t_inv_p = powf(1.0f - t, p);

  return t_p / (t_p + t_inv_p);
}

static void build_s_curve_cycles(uint32_t *out_words, uint steps, float f0_hz,
                                 float f1_hz, float duty_cycle,
                                 int aggressiveness) {
  const uint32_t sys_hz = clock_get_hz(clk_sys);
  if (steps < 2)
    steps = 2;

  const uint32_t min_cycles = 1u;
  const uint32_t max_cycles = 0x7fffffffu;

  for (uint i = 0; i < steps; ++i) {
    const float t = (float)i / (float)(steps - 1u);
    const float s = get_curve_factor(t, aggressiveness);
    const float f_hz = f0_hz + (f1_hz - f0_hz) * s;

    float safe_f_hz = f_hz;
    if (safe_f_hz < 0.1f)
      safe_f_hz = 0.1f;

    const uint64_t total_cycles_64 =
        (uint64_t)((double)sys_hz / (double)safe_f_hz);
    const uint32_t total_cycles =
        clamp_u32(total_cycles_64, min_cycles + 2u, max_cycles);

    uint32_t high_cycles = 0;
    uint32_t low_cycles = 0;
    split_total_cycles(total_cycles, duty_cycle, &high_cycles, &low_cycles);

    out_words[2u * i + 0u] = high_cycles;
    out_words[2u * i + 1u] = low_cycles;
  }
}

// Tiempo fijo del pulso en Alto por recomendación del TMC2209 en nanosegundos
// (típicamente >100ns)
#define TMC2209_FIXED_HIGH_TIME_US 2.0f

static void build_constant_cycles_pair(uint32_t out_pair[2], float freq_hz) {
  const uint32_t sys_hz = clock_get_hz(clk_sys);

  float safe_f_hz = freq_hz;
  if (safe_f_hz < 0.1f)
    safe_f_hz = 0.1f;

  const uint64_t total_cycles_64 =
      (uint64_t)((double)sys_hz / (double)safe_f_hz);
  const uint32_t total_cycles = clamp_u32(total_cycles_64, 3u, 0x7fffffffu);

  // Calcular ciclos altos fijos (2us)
  uint32_t high_cycles =
      (uint32_t)(TMC2209_FIXED_HIGH_TIME_US * 1e-6f * sys_hz);
  if (high_cycles < 1u)
    high_cycles = 1u;
  if (high_cycles >= total_cycles)
    high_cycles = total_cycles - 1u;

  uint32_t low_cycles = total_cycles - high_cycles;

  out_pair[0] = high_cycles;
  out_pair[1] = low_cycles;
}

static void fill_ping_pong_buffer(TMC2209_t *ctx, uint32_t *target_buf) {
  const uint32_t sys_hz = clock_get_hz(clk_sys);
  uint32_t high_cycles =
      (uint32_t)(TMC2209_FIXED_HIGH_TIME_US * 1e-6f * sys_hz);
  if (high_cycles < 1u)
    high_cycles = 1u;

  // Llenaremos hasta TMC2209_PING_PONG_BUFFER_STEPS pasos
  uint32_t steps_to_fill = TMC2209_PING_PONG_BUFFER_STEPS;

  // Validar finalización
  if (ctx->current_step_idx >= ctx->total_ramp_steps) {
    if (ctx->is_braking) {
      // Si es frenado, al terminar nos detenemos (no mandamos más pulsos)
      // Llenar con ciclos en bajo masivos o detener directamente.
      // Para ser seguros, simulamos frecuencia muy baja (0 Hz es infinito,
      // ponemos max uint32)
      for (uint32_t i = 0; i < steps_to_fill; i++) {
        target_buf[2u * i + 0u] = high_cycles;
        target_buf[2u * i + 1u] = 0x7fffffffu; // Espera casi infinita
      }
    } else {
      // Rampa terminó en aceleración normal. Llenamos con la frecuencia target
      // constante.
      for (uint32_t i = 0; i < steps_to_fill; i++) {
        uint64_t total_cycles =
            (uint64_t)((double)sys_hz / (double)ctx->freq_target_hz);
        total_cycles = clamp_u32(total_cycles, high_cycles + 1u, 0x7fffffffu);
        target_buf[2u * i + 0u] = high_cycles;
        target_buf[2u * i + 1u] = total_cycles - high_cycles;
      }
    }
    return;
  }

  // Calcular cuánto le queda a la rampa
  uint32_t remaining = ctx->total_ramp_steps - ctx->current_step_idx;
  if (remaining < steps_to_fill) {
    steps_to_fill = remaining;
  }

  for (uint32_t i = 0; i < steps_to_fill; i++) {
    float f_hz;
    if (ctx->current_step_idx < ctx->transition_step_idx) {
      // Segmento 1
      f_hz = ctx->freq_start_hz +
             ctx->current_slope1 * (float)ctx->current_step_idx;
    } else {
      // Segmento 2
      uint32_t steps_in_seg2 = ctx->current_step_idx - ctx->transition_step_idx;
      f_hz = ctx->current_freq_mid + ctx->current_slope2 * (float)steps_in_seg2;
    }

    if (f_hz < 0.1f)
      f_hz = 0.1f;

    const uint64_t total_cycles_64 = (uint64_t)((double)sys_hz / (double)f_hz);
    uint32_t total_cycles =
        clamp_u32(total_cycles_64, high_cycles + 1u, 0x7fffffffu);

    target_buf[2u * i + 0u] = high_cycles;
    target_buf[2u * i + 1u] = total_cycles - high_cycles;

    ctx->current_step_idx++;
  }

  // Rellenar lo sobrante del buffer con la última frecuencia para
  // evitar latencia si era el tramo final
  if (steps_to_fill < TMC2209_PING_PONG_BUFFER_STEPS) {
    uint32_t last_high = target_buf[2u * (steps_to_fill - 1) + 0u];
    uint32_t last_low = target_buf[2u * (steps_to_fill - 1) + 1u];
    for (uint32_t i = steps_to_fill; i < TMC2209_PING_PONG_BUFFER_STEPS; i++) {
      target_buf[2u * i + 0u] = last_high;
      target_buf[2u * i + 1u] = last_low;
    }
  }
}

static void tmc2209_dma_irq_handler(void) {
  uint32_t ints = dma_hw->ints0;

  // Revisamos todos los canales activos para ver cuál disparó la IRQ
  for (int ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
    if (ints & (1u << ch)) {
      TMC2209_t *ctx = g_dma_ctx_map[ch];
      if (!ctx)
        continue;

      // Limpiamos bandera de IRQ
      dma_hw->ints0 = 1u << ch;

      // Ping-Pong Rampa (Canales A o B dependientes)
      // Aquí usamos dma_ramp_ch como el canal dinámico en general.
      // Necesitamos verificar si acabamos de terminar el canal de rampa...
      if (ctx->dma_ramp_ch == ch || ctx->dma_steady_ch == ch) {
        // El canal DMA completó uno de los buffers.
        // Como tenemos chained DMA bufA <-> bufB iterativamente, actualizamos
        // el buffer recién vaciado.

        bool stop_engine = false;
        while (ctx->current_step_idx >= ctx->total_ramp_steps) {
          if (ctx->current_phase == TMC2209_PHASE_ACCEL) {
            ctx->current_phase = TMC2209_PHASE_STEADY;
            ctx->current_step_idx = 0;
            ctx->total_ramp_steps = ctx->steady_total_steps;
            ctx->freq_start_hz = ctx->freq_target_hz;
            ctx->current_slope1 = 0.0f;
            ctx->current_slope2 = 0.0f;
            ctx->transition_step_idx = ctx->steady_total_steps;
            ctx->is_braking = false;
          } else if (ctx->current_phase == TMC2209_PHASE_STEADY) {
            ctx->current_phase = TMC2209_PHASE_DECEL;
            ctx->current_step_idx = 0;
            ctx->total_ramp_steps = ctx->decel_total_steps;
            ctx->freq_start_hz =
                ctx->freq_target_hz; // Empieza donde terminó steady
            ctx->current_freq_mid = ctx->decel_freq_mid;
            ctx->freq_target_hz = ctx->decel_freq_target_hz;
            ctx->current_slope1 = ctx->decel_slope1;
            ctx->current_slope2 = ctx->decel_slope2;
            ctx->transition_step_idx = ctx->decel_transition_step_idx;
            ctx->is_braking = true;
          } else {
            // TMC2209_PHASE_DECEL o TMC2209_PHASE_NONE
            stop_engine = true;
            break;
          }
        }

        if (stop_engine) {
          if (ctx->is_braking) {
            // Deshabilitar el PIO y poner motor en reposo
            pio_sm_set_enabled(ctx->pio, ctx->sm, false);
            pio_sm_set_pins(ctx->pio, ctx->sm, 0);
            pio_sm_clear_fifos(ctx->pio, ctx->sm);
            ctx->mode = TMC2209_MODE_STANDBY_HOLD;

            dma_channel_abort(ctx->dma_ramp_ch);
            dma_channel_abort(ctx->dma_steady_ch);
          } else {
            // Dejar en velocidad constante mantenida por el ping-pong residual
            dma_channel_set_irq0_enabled(ctx->dma_ramp_ch, false);
            dma_channel_set_irq0_enabled(ctx->dma_steady_ch, false);
          }
          continue;
        }

        // Generar próximos pasos en el buffer que ahora está "libre"
        if (ctx->dma_ramp_ch == ch) {
          // dma_ramp_ch (bufA) terminó, B está corriendo. Rellenamos A.
          fill_ping_pong_buffer(ctx, ctx->bufA);
          dma_channel_set_read_addr(ctx->dma_ramp_ch, ctx->bufA, false);
        } else {
          // dma_steady_ch (bufB) terminó, A está corriendo. Rellenamos B.
          fill_ping_pong_buffer(ctx, ctx->bufB);
          dma_channel_set_read_addr(ctx->dma_steady_ch, ctx->bufB, false);
        }
      }
      // Si este canal es su canal de parada definitiva (stop_ch)
      else if (ctx->dma_stop_ch == ch) {
        pio_sm_set_enabled(ctx->pio, ctx->sm, false);
        pio_sm_set_pins(ctx->pio, ctx->sm, 0);
        pio_sm_clear_fifos(ctx->pio, ctx->sm);
        ctx->mode = TMC2209_MODE_STANDBY_HOLD;
      }
    }
  }
}

// Variable estática para saber si el programa PIO ya fue cargado (para soportar
// múltiples instancias)
static bool _pio_program_loaded = false;
static uint _pio_program_offset = 0;

static void tmc2209_pio_init_sm(PIO pio, uint sm, uint offset, uint pin) {
  pio_sm_config c = tmc2209_stepgen_program_get_default_config(offset);
  sm_config_set_set_pins(&c, pin, 1);
  pio_gpio_init(pio, pin);
  pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);
}

void tmc2209_init(TMC2209_t *motor, uint8_t step_pin, uint8_t dir_pin,
                  uint8_t enable_pin, uint16_t steps_per_rev,
                  uint8_t microsteps, uint8_t ms1_pin, uint8_t ms2_pin) {
  // Guardar la configuración en la estructura
  motor->step_pin = step_pin;
  motor->dir_pin = dir_pin;
  motor->enable_pin = enable_pin;
  motor->ms1_pin = ms1_pin;
  motor->ms2_pin = ms2_pin;
  motor->steps_per_rev = steps_per_rev;
  motor->microsteps = microsteps;
  motor->limit_switches_enabled = false;

  // Inicializar pines GPIO
  gpio_init(motor->dir_pin);
  gpio_set_dir(motor->dir_pin, GPIO_OUT);
  gpio_init(motor->enable_pin);
  gpio_set_dir(motor->enable_pin, GPIO_OUT);

  // Por defecto, el driver está deshabilitado (ENA a nivel alto)
  tmc2209_enable(motor, false);
  motor->mode = TMC2209_MODE_STANDBY_HOLD; // Modo inicial

  // --- Inicialización PIO ---
  motor->pio = pio0; // Usamos pio0 por defecto, podría ser configurable

  // Cargar programa PIO si no está cargado
  if (!_pio_program_loaded) {
    _pio_program_offset = pio_add_program(motor->pio, &tmc2209_stepgen_program);
    _pio_program_loaded = true;
  }
  motor->offset = _pio_program_offset;

  // Reclamar una State Machine libre
  motor->sm = pio_claim_unused_sm(motor->pio, true);

  // Configurar la SM para el pin STEP
  tmc2209_pio_init_sm(motor->pio, motor->sm, motor->offset, motor->step_pin);

  // --- Inicialización DMA ---
  motor->dma_ramp_ch = dma_claim_unused_channel(true);
  motor->dma_steady_ch = dma_claim_unused_channel(true);
  motor->dma_stop_ch = dma_claim_unused_channel(true);

  // Registrar esta instancia en el mapa global para la IRQ
  // Ahora interceptamos el canal de rampa para el double buffering ping-pong
  g_dma_ctx_map[motor->dma_ramp_ch] = motor;
  g_dma_ctx_map[motor->dma_steady_ch] =
      motor; // Agregado como doble safety (el ping-pong usará el steady u otro
             // loop)
  g_dma_ctx_map[motor->dma_stop_ch] = motor;

  irq_set_exclusive_handler(DMA_IRQ_0, tmc2209_dma_irq_handler);
  irq_set_enabled(DMA_IRQ_0, true);
}

void tmc2209_enable(TMC2209_t *motor, bool enable) {
  // El pin ENA es activo a nivel bajo
  gpio_put(motor->enable_pin, !enable);
  sleep_ms(2);
}

void tmc2209_set_microstepping_by_pins(TMC2209_t *motor,
                                       TMC2209_Microsteps_t microsteps) {
  if (!motor)
    return;

  // Inicializar pines como salida
  gpio_init(motor->ms1_pin);
  gpio_set_dir(motor->ms1_pin, GPIO_OUT);
  gpio_init(motor->ms2_pin);
  gpio_set_dir(motor->ms2_pin, GPIO_OUT);

  bool ms1 = 0, ms2 = 0;

  switch (microsteps) {
  case TMC2209_MICROSTEPS_1: // 8 * 200 = 1600 pasos/rev
    ms1 = 0;
    ms2 = 0;
    motor->microsteps = 1;
    break;
  case TMC2209_MICROSTEPS_2: // 256/2 = 128 -> 128 * 200 = 25600 pasos/rev
    ms1 = 1;
    ms2 = 0;
    motor->microsteps = 2;
    break;
  case TMC2209_MICROSTEPS_4: // 256/4 = 64 -> 64 * 200 = 12800 pasos/rev
    ms1 = 0;
    ms2 = 1;
    motor->microsteps = 4;
    break;
  case TMC2209_MICROSTEPS_16: // 256/16 = 16 -> 16 * 200 = 3200 pasos/rev
    ms1 = 1;
    ms2 = 1;
    motor->microsteps = 16;
    break;
  default:
    printf("⚠️ Microstepping %d no soportado por pines. Usar UART.\n",
           microsteps);
    return;
  }

  gpio_put(motor->ms1_pin, ms1);
  gpio_put(motor->ms2_pin, ms2);

  // Actualizar la dirección UART interna, ya que cambiar los pines MS cambia la
  // dirección física del chip
  motor->addr = (ms2 << 1) | ms1;

  printf("Microstepping configurado por pines: %d (MS1=%d, MS2=%d)\n",
         motor->microsteps, ms1, ms2);
}

void tmc2209_setup_limit_switches(TMC2209_t *motor, uint8_t start_pin,
                                  uint8_t end_pin) {
  motor->limit_switch_start_pin = start_pin;
  motor->limit_switch_end_pin = end_pin;

  gpio_init(start_pin);
  gpio_set_dir(start_pin, GPIO_IN);
  gpio_pull_up(start_pin);

  gpio_init(end_pin);
  gpio_set_dir(end_pin, GPIO_IN);
  gpio_pull_up(end_pin);

  motor->limit_switches_enabled = true;
}

void tmc2209_print_limit_switches_status(TMC2209_t *motor) {
  if (!motor || !motor->limit_switches_enabled) {
    printf("Finales de carrera no habilitados o motor invalido.\n");
    return;
  }

  bool start_val = gpio_get(motor->limit_switch_start_pin);
  bool end_val = gpio_get(motor->limit_switch_end_pin);

  printf("Finales de Carrera -> Inicio: %s (%d) | Fin: %s (%d)\n",
         start_val ? "ABIERTO" : "ACTIVADO", start_val,
         end_val ? "ABIERTO" : "ACTIVADO", end_val);
}

void tmc2209_set_uart_address_pins(TMC2209_t *motor, uint8_t addr) {
  gpio_init(motor->ms1_pin);
  gpio_set_dir(motor->ms1_pin, GPIO_OUT);
  gpio_put(motor->ms1_pin, addr & 0x01); // Bit 0

  gpio_init(motor->ms2_pin);
  gpio_set_dir(motor->ms2_pin, GPIO_OUT);
  gpio_put(motor->ms2_pin, (addr >> 1) & 0x01); // Bit 1

  motor->addr = addr; // Actualizar la dirección almacenada en la estructura
}

void tmc2209_set_microstepping_uart(TMC2209_t *motor,
                                    TMC2209_Microsteps_t microsteps) {
  // Asegurar que mstep_reg_select (bit 7) en GCONF esté activo para usar MRES
  uint32_t gconf = tmc2209_read_register(motor, TMC2209_REG_GCONF);
  if ((gconf & (1 << 7)) == 0) {
    gconf |= (1 << 7);
    tmc2209_write_register(motor, TMC2209_REG_GCONF, gconf);
  }

  uint32_t chopconf = tmc2209_read_register(motor, TMC2209_REG_CHOPCONF);

  // Protección: Si la lectura falla (retorna 0), no escribir para evitar poner
  // TOFF=0 (Driver Disable)
  if (chopconf == 0) {
    return;
  }
  // sleep_us(100);
  chopconf &= ~(0x0F << 24);                // Limpiar bits MRES (24-27)
  chopconf |= ((uint32_t)microsteps << 24); // Establecer nuevos bits MRES
  tmc2209_write_register(motor, TMC2209_REG_CHOPCONF, chopconf);

  // Actualizar la estructura interna para cálculos de velocidad
  motor->microsteps = 256 >> microsteps;
}

uint16_t tmc2209_get_microsteps(TMC2209_t *motor) {
  uint32_t gconf = tmc2209_read_register(motor, TMC2209_REG_GCONF);

  // Verificar bit 7 (mstep_reg_select) de GCONF
  // 1 = Configuración por registro MRES (CHOPCONF)
  // 0 = Configuración por pines MS1/MS2
  if (gconf & (1 << 7)) {
    // Leer el registro CHOPCONF para obtener MRES
    uint32_t chopconf = tmc2209_read_register(motor, TMC2209_REG_CHOPCONF);
    uint8_t mres = (chopconf >> 24) & 0x0F;
    return 256 >> mres;
  } else {
    // Leer los pines MS1 y MS2
    uint32_t ioin = tmc2209_read_register(motor, TMC2209_REG_IOIN);
    bool ms1 = (ioin >> 6) & 1;
    bool ms2 = (ioin >> 7) & 1;

    // Mapeo según datasheet TMC2209 para pines
    if (!ms2 && !ms1)
      return 8;
    if (!ms2 && ms1)
      return 32;
    if (ms2 && !ms1)
      return 64;
    return 16; // ms2 && ms1
  }
}

void tmc2209_set_direction(TMC2209_t *motor, bool direction) {
  motor->direction = direction;
  gpio_put(motor->dir_pin, motor->direction);
}

void tmc2209_set_rpm(TMC2209_t *motor, float rpm) {
  // Si RPM es 0 o negativo, el motor se detiene
  if (rpm <= 0) {
    tmc2209_stop(motor);
    return;
  }

  // Calcular la frecuencia de pulso en Hz
  float pulse_freq = (rpm / 60.0f) * motor->steps_per_rev * motor->microsteps;

  // Calcular ciclos de reloj para PIO
  const uint32_t sys_hz = clock_get_hz(clk_sys);
  const uint64_t total_cycles_64 =
      (uint64_t)((double)sys_hz / (double)pulse_freq);
  const uint32_t total_cycles = clamp_u32(total_cycles_64, 3u, 0x7fffffffu);

  uint32_t high_cycles = 0, low_cycles = 0;
  split_total_cycles(total_cycles, 0.5f, &high_cycles, &low_cycles);

  // Configurar PIO en modo INFINITE
  pio_sm_set_enabled(motor->pio, motor->sm, false);
  pio_sm_clear_fifos(motor->pio, motor->sm);
  pio_sm_exec(motor->pio, motor->sm,
              pio_encode_jmp(motor->offset + tmc2209_stepgen_offset_infinite));
  pio_sm_set_enabled(motor->pio, motor->sm, true);

  // Enviar tiempos (High, Low)
  pio_sm_put_blocking(motor->pio, motor->sm, high_cycles);
  pio_sm_put_blocking(motor->pio, motor->sm, low_cycles);

  if (motor->direction == true) {
    motor->mode = TMC2209_MODE_RUN_CW; // Modo avance
  } else {
    motor->mode = TMC2209_MODE_RUN_CCW; // Modo retroceso
  }
}

void tmc2209_stop(TMC2209_t *motor) {
  // Detener la SM y limpiar FIFOs
  pio_sm_set_enabled(motor->pio, motor->sm, false);
  pio_sm_clear_fifos(motor->pio, motor->sm);

  // Cambiar el modo a STANDBY_HOLD
  motor->mode = TMC2209_MODE_STANDBY_HOLD;
}

void tmc2209_stop_and_disable(TMC2209_t *motor) {
  // Detener el motor
  tmc2209_stop(motor);
  // Deshabilitar el driver
  tmc2209_enable(motor, false);
  // Reiniciar el modo a STANDBY_HOLD
  motor->mode = TMC2209_MODE_STANDBY_HOLD;
}

void tmc2209_set_turns_at_rpm(TMC2209_t *motor, float rpm, float turns) {
  // Calcular el número total de micropasos para las vueltas deseadas
  uint64_t total_microsteps =
      (uint64_t)(turns * motor->steps_per_rev * motor->microsteps);

  // Llamar a la función para enviar un número específico de pasos a una
  // frecuencia calculada La frecuencia se calcula a partir de las RPM deseadas
  float freq = (rpm / 60.0f) * motor->steps_per_rev * motor->microsteps;
  tmc2209_send_nsteps_at_freq(motor, total_microsteps, freq);
}

void tmc2209_send_nsteps_at_freq(TMC2209_t *motor, int nsteps, float freq) {
  // Si nsteps es 0 o negativo, el motor se detiene
  if (nsteps <= 0 || freq <= 0) {
    tmc2209_stop(motor);
    return;
  }

  // Calcular ciclos de reloj para PIO
  const uint32_t sys_hz = clock_get_hz(clk_sys);
  const uint64_t total_cycles_64 = (uint64_t)((double)sys_hz / (double)freq);
  const uint32_t total_cycles = clamp_u32(total_cycles_64, 3u, 0x7fffffffu);

  uint32_t high_cycles = 0, low_cycles = 0;
  split_total_cycles(total_cycles, 0.5f, &high_cycles, &low_cycles);

  // Configurar PIO en modo BURST
  pio_sm_set_enabled(motor->pio, motor->sm, false);
  pio_sm_clear_fifos(motor->pio, motor->sm);
  pio_sm_exec(motor->pio, motor->sm,
              pio_encode_jmp(motor->offset + tmc2209_stepgen_offset_burst));
  pio_sm_set_enabled(motor->pio, motor->sm, true);

  // Enviar datos: N-1, High, Low
  pio_sm_put_blocking(motor->pio, motor->sm, nsteps - 1);
  pio_sm_put_blocking(motor->pio, motor->sm, high_cycles);
  pio_sm_put_blocking(motor->pio, motor->sm, low_cycles);

  motor->mode = TMC2209_MODE_NSTEPS;
}

bool tmc2209_is_moving(TMC2209_t *motor) {
  // Si estamos en modo NSTEPS, verificamos si el PIO ha terminado
  if (motor->mode == TMC2209_MODE_NSTEPS) {
    // Verificamos si la SM está bloqueada en PULL (TX Stall)
    // Esto indica que el bucle burst terminó y volvió al inicio esperando datos
    if (motor->pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + motor->sm))) {
      // Limpiar el flag de stall
      motor->pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + motor->sm));
      tmc2209_stop(motor); // Esto pone el modo en STANDBY
      return false;
    }
  }

  if (motor->mode == TMC2209_MODE_RUN_CW ||
      motor->mode == TMC2209_MODE_RUN_CCW ||
      motor->mode == TMC2209_MODE_NSTEPS) {
    return true;
  }
  return false;
}

TMC2209_Mode_t tmc2209_get_mode(TMC2209_t *motor) { return motor->mode; }

// --- Funciones UART ---

// Cálculo de CRC-8 para protocolo TMC (Polinomio: x^8 + x^2 + x^1 + 1 = 0x07)
static uint8_t tmc2209_calc_crc(uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t currentByte = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (currentByte & 0x01)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc = (crc << 1);
      }
      currentByte = currentByte >> 1;
    }
  }
  return crc;
}

void tmc2209_setup_uart(TMC2209_t *motor, uart_inst_t *uart, uint32_t baudrate,
                        uint8_t addr, uint8_t tx_pin, uint8_t rx_pin) {
  motor->uart = uart;
  motor->addr = addr;
  motor->tx_pin = tx_pin;
  motor->rx_pin = rx_pin;

  if (motor->uart) {
    uart_init(motor->uart, baudrate);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_format(motor->uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(motor->uart, true);
  }
}

void tmc2209_write_register(TMC2209_t *motor, uint8_t reg, uint32_t value) {
  if (!motor->uart)
    return;

  uint8_t datagram[8];
  datagram[0] = 0x05;        // Sync Byte
  datagram[1] = motor->addr; // Slave Address
  datagram[2] = reg | 0x80;  // Register Address + Write Bit (bit 7)
  datagram[3] = (value >> 24) & 0xFF;
  datagram[4] = (value >> 16) & 0xFF;
  datagram[5] = (value >> 8) & 0xFF;
  datagram[6] = value & 0xFF;
  datagram[7] = tmc2209_calc_crc(datagram, 7);

  uart_write_blocking(motor->uart, datagram, 8);
  uart_tx_wait_blocking(motor->uart);
  sleep_us(TMC2209_UART_DELAY_US);
}

uint32_t tmc2209_read_register(TMC2209_t *motor, uint8_t reg) {
  if (!motor->uart)
    return 0;

  // Limpiar el buffer RX antes de enviar para evitar leer datos viejos
  while (uart_is_readable(motor->uart)) {
    uart_getc(motor->uart);
  }

  // Enviar solicitud de lectura
  uint8_t datagram[4];
  datagram[0] = 0x05; // Sync Byte
  datagram[1] = motor->addr;
  datagram[2] = reg; // Register Address (Write Bit 0)
  datagram[3] = tmc2209_calc_crc(datagram, 3);

  uart_write_blocking(motor->uart, datagram, 4);
  uart_tx_wait_blocking(motor->uart);

  // Poner TX en Hi-Z (Input) para liberar el bus de un solo hilo
  sleep_us(20);
  gpio_set_function(motor->tx_pin, GPIO_FUNC_SIO);
  gpio_set_dir(motor->tx_pin, GPIO_IN);

  // Leer respuesta (con timeout simple)
  // La respuesta esperada son 8 bytes: 0x05, 0xFF, REG, DATA(4), CRC

  uint8_t rx_buf[16];
  int idx = 0;
  absolute_time_t timeout = make_timeout_time_ms(200);
  int64_t time_diff = absolute_time_diff_us(get_absolute_time(), timeout);
  // printf("timeout: %lld\n", time_diff);
  while (!time_reached(timeout) && idx < 16) {
    if (uart_is_readable(motor->uart)) {
      rx_buf[idx++] = uart_getc(motor->uart);

      // Verificar si tenemos un paquete válido al final del buffer
      if (idx >= 8) {
        // Buscar la firma de respuesta: Sync 0x05 y Master Addr 0xFF
        int start = idx - 8;
        if (rx_buf[start] == 0x05 && rx_buf[start + 1] == 0xFF &&
            rx_buf[start + 2] == reg) {
          if (tmc2209_calc_crc(&rx_buf[start], 7) == rx_buf[start + 7]) {
            // Restaurar TX a modo UART antes de salir
            sleep_us(20);
            gpio_set_function(motor->tx_pin, GPIO_FUNC_UART);
            sleep_us(TMC2209_UART_DELAY_US);
            return (rx_buf[start + 3] << 24) | (rx_buf[start + 4] << 16) |
                   (rx_buf[start + 5] << 8) | rx_buf[start + 6];
          }
        }
      }
    }
  }
  // Restaurar TX a modo UART en caso de timeout
  gpio_set_function(motor->tx_pin, GPIO_FUNC_UART);
  sleep_us(TMC2209_UART_DELAY_US);
  return 0; // Error o timeout
}

void tmc2209_set_current(TMC2209_t *motor, uint8_t run_current,
                         uint8_t hold_current, uint8_t tpowerdown) {
  // Limitar valores a 5 bits (0-31)
  if (run_current > 31)
    run_current = 31;
  if (hold_current > 31)
    hold_current = 31;

  uint32_t ihold_irun_val = 0;
  ihold_irun_val |= (hold_current & 0x1F);       // IHOLD en bits 0-4
  ihold_irun_val |= ((run_current & 0x1F) << 8); // IRUN en bits 8-12
  ihold_irun_val |= (1 << 16); // IHOLDDELAY (default 1) en bits 16-19

  tmc2209_write_register(motor, TMC2209_REG_IHOLD_IRUN, ihold_irun_val);
  tmc2209_write_register(motor, TMC2209_REG_TPOWERDOWN, tpowerdown);
}

void tmc2209_set_stallguard_threshold(TMC2209_t *motor, uint8_t threshold) {
  tmc2209_write_register(motor, TMC2209_REG_SGTHRS, threshold);
}

uint16_t tmc2209_get_stallguard_result(TMC2209_t *motor) {
  // Verificar el modo de operación en GCONF (Bit 2: en_spreadCycle)
  // StallGuard4 solo entrega valores válidos en modo StealthChop (bit 2 = 0).
  uint32_t gconf = tmc2209_read_register(motor, TMC2209_REG_GCONF);
  if (gconf & (1 << 2)) {
    return 0; // SpreadCycle activo, el valor de SG_RESULT no es válido.
  }
  return (uint16_t)(tmc2209_read_register(motor, TMC2209_REG_SG_RESULT) &
                    0x3FF);
}

uint16_t tmc2209_read_sg_result(TMC2209_t *motor) {
  return (uint16_t)(tmc2209_read_register(motor, TMC2209_REG_SG_RESULT) &
                    0x3FF);
}

uint32_t tmc2209_read_drv_status(TMC2209_t *motor) {
  return tmc2209_read_register(motor, TMC2209_REG_DRV_STATUS);
}

uint32_t tmc2209_read_gstat(TMC2209_t *motor) {
  return tmc2209_read_register(motor, TMC2209_REG_GSTAT);
}

void tmc2209_clear_gstat(TMC2209_t *motor, uint8_t clear_mask) {
  // Escribir 1 en los bits correspondientes de GSTAT limpia la falla
  // correspondiente. Bits: 0 -> reset, 1 -> drv_err, 2 -> uv_cp
  tmc2209_write_register(motor, TMC2209_REG_GSTAT, (uint32_t)clear_mask);
}

void tmc2209_set_chopper_mode(TMC2209_t *motor, TMC2209_ChopperMode_t mode,
                              uint32_t tpwmthrs) {
  uint32_t gconf = tmc2209_read_register(motor, TMC2209_REG_GCONF);
  // Bit 2: en_spreadCycle (1 = SpreadCycle, 0 = StealthChop)
  if (mode == TMC2209_CHOPPER_SPREADCYCLE) {
    gconf |= (1 << 2);
    tmc2209_write_register(motor, TMC2209_REG_TPWMTHRS,
                           0); // Deshabilitar auto-switch
  } else if (mode == TMC2209_CHOPPER_STEALTHCHOP) {
    gconf &= ~(1 << 2);
    tmc2209_write_register(motor, TMC2209_REG_TPWMTHRS,
                           0); // Deshabilitar auto-switch
  } else if (mode == TMC2209_CHOPPER_DYNAMIC) {
    gconf &= ~(1 << 2); // Debe iniciar en StealthChop para que TPWMTHRS aplique
    tmc2209_write_register(motor, TMC2209_REG_TPWMTHRS, tpwmthrs);
  }
  tmc2209_write_register(motor, TMC2209_REG_GCONF, gconf);
}

void tmc2209_set_pdn_disable(TMC2209_t *motor, bool disable) {
  uint32_t gconf = tmc2209_read_register(motor, TMC2209_REG_GCONF);
  if (disable) {
    gconf |= (1 << 6); // Set bit 6 (pdn_disable)
  } else {
    gconf &= ~(1 << 6); // Clear bit 6
  }
  tmc2209_write_register(motor, TMC2209_REG_GCONF, gconf);
}

void tmc2209_set_coolstep_threshold(TMC2209_t *motor, uint32_t threshold) {
  if (threshold > 0xFFFFF)
    threshold = 0xFFFFF; // Limitar a 20 bits
  tmc2209_write_register(motor, TMC2209_REG_TCOOLTHRS, threshold);
}

uint32_t tmc2209_compute_coolstep_threshold(float steps_per_sec) {
  // TSTEP = f_CLK / f_STEP. f_CLK típico es 12 MHz.
  // Si la velocidad es muy baja (< ~12 pasos/s), el valor excede los 20 bits.
  if (steps_per_sec < 12.0f)
    return 0xFFFFF;
  uint32_t result = (uint32_t)(12000000.0f / steps_per_sec);
  if (result > 0xFFFFF)
    return 0xFFFFF;
  return result;
}

void tmc2209_configure_chopconf(TMC2209_t *motor, uint8_t toff, bool intpol) {
  uint32_t chopconf = tmc2209_read_register(motor, TMC2209_REG_CHOPCONF);

  // Si la lectura falla (retorna 0), evitamos escribir para no deshabilitar el
  // driver accidentalmente (TOFF=0)
  if (chopconf == 0)
    return;

  // Configurar TOFF (Bits 0-3)
  chopconf &= ~0x0F;
  chopconf |= (toff & 0x0F);

  // Configurar intpol (Bit 28)
  if (intpol)
    chopconf |= (1 << 28);
  else
    chopconf &= ~(1 << 28);

  tmc2209_write_register(motor, TMC2209_REG_CHOPCONF, chopconf);
}

void tmc2209_set_chopper_parameters(TMC2209_t *motor, uint8_t hstrt,
                                    uint8_t hend, uint8_t tbl) {
  uint32_t chopconf = tmc2209_read_register(motor, TMC2209_REG_CHOPCONF);

  if (chopconf == 0)
    return;

  // Limpiar HSTRT (bits 4-6), HEND (bits 7-10), TBL (bits 15-16)
  chopconf &= ~((0x07 << 4) | (0x0F << 7) | (0x03 << 15));

  // Aplicar nuevos valores
  chopconf |= ((hstrt & 0x07) << 4);
  chopconf |= ((hend & 0x0F) << 7);
  chopconf |= ((tbl & 0x03) << 15);

  tmc2209_write_register(motor, TMC2209_REG_CHOPCONF, chopconf);
}

void tmc2209_set_vactual(TMC2209_t *motor, int32_t vactual) {
  // Escribir en el registro VACTUAL (0x22).
  // Si vactual != 0, el generador interno toma el control y STEP/DIR se
  // ignoran. Si vactual == 0, el driver se detiene y devuelve el control a
  // STEP/DIR.
  tmc2209_write_register(motor, TMC2209_REG_VACTUAL, (uint32_t)vactual);

  // Actualizar el estado lógico para reflejar el movimiento
  if (vactual == 0) {
    motor->mode = TMC2209_MODE_STANDBY_HOLD;
  } else {
    motor->mode = (vactual > 0) ? TMC2209_MODE_RUN_CW : TMC2209_MODE_RUN_CCW;
  }
}

int32_t tmc2209_compute_vactual(TMC2209_t *motor, float rpm) {
  // 1. Calcular pasos por segundo (Microsteps/s)
  float steps_per_sec =
      (rpm / 60.0f) * motor->steps_per_rev * motor->microsteps;

  // 2. Convertir a VACTUAL usando la fórmula del datasheet:
  // VACTUAL = (steps_per_sec * 2^24) / f_CLK
  // f_CLK típico = 12 MHz, 2^24 = 16777216
  float vactual = (steps_per_sec * 16777216.0f) / 12000000.0f;

  return (int32_t)vactual;
}

// --- Implementación de funciones DMA / Curva S ---

void tmc2209_start_s_curve_dma(TMC2209_t *motor, float freq_start_hz,
                               float freq_target_hz, float duty_cycle,
                               uint ramp_steps, int aggressiveness) {
  if (ramp_steps > TMC2209_DMA_MAX_STEPS)
    ramp_steps = TMC2209_DMA_MAX_STEPS;
  if (ramp_steps < 2u)
    ramp_steps = 2u;

  motor->freq_start_hz = freq_start_hz;
  motor->freq_target_hz = freq_target_hz;
  motor->duty_cycle = duty_cycle;
  motor->ramp_steps = ramp_steps;

  build_s_curve_cycles(motor->ramp_buf, ramp_steps, freq_start_hz,
                       freq_target_hz, duty_cycle, aggressiveness);
  build_constant_cycles_pair(motor->steady_buf, freq_target_hz);

  const uint dreq = pio_get_dreq(motor->pio, motor->sm, true);
  volatile void *pio_txf = &motor->pio->txf[motor->sm];

  dma_channel_abort(motor->dma_ramp_ch);
  dma_channel_abort(motor->dma_steady_ch);
  dma_channel_abort(motor->dma_stop_ch);

  pio_sm_set_enabled(motor->pio, motor->sm, false);
  pio_sm_clear_fifos(motor->pio, motor->sm);
  pio_sm_exec(
      motor->pio, motor->sm,
      pio_encode_jmp(motor->offset + tmc2209_stepgen_offset_dma_stream));
  pio_sm_set_enabled(motor->pio, motor->sm, true);

  // DMA steady: lectura circular de 2 words (alto/bajo) para pulsos indefinidos
  dma_channel_config c_steady =
      dma_channel_get_default_config(motor->dma_steady_ch);
  channel_config_set_transfer_data_size(&c_steady, DMA_SIZE_32);
  channel_config_set_read_increment(&c_steady, true);
  channel_config_set_write_increment(&c_steady, false);
  channel_config_set_dreq(&c_steady, dreq);
  // Ring en READ sobre 8 bytes = 2 words
  channel_config_set_ring(&c_steady, false /* write */, 3 /* 2^3 bytes */);
  dma_channel_configure(motor->dma_steady_ch, &c_steady, pio_txf,
                        motor->steady_buf, 0xffffffffu, false);

  // DMA de rampa: one-shot, y encadena a steady al terminar
  dma_channel_config c_ramp =
      dma_channel_get_default_config(motor->dma_ramp_ch);
  channel_config_set_transfer_data_size(&c_ramp, DMA_SIZE_32);
  channel_config_set_read_increment(&c_ramp, true);
  channel_config_set_write_increment(&c_ramp, false);
  channel_config_set_dreq(&c_ramp, dreq);
  channel_config_set_chain_to(&c_ramp, motor->dma_steady_ch);
  dma_channel_configure(motor->dma_ramp_ch, &c_ramp, pio_txf, motor->ramp_buf,
                        2u * ramp_steps, true);

  motor->mode =
      TMC2209_MODE_RUN_CW; // Asumimos movimiento (la dirección se setea aparte)
}

void tmc2209_start_2part_curve_dma(TMC2209_t *motor, float freq_start,
                                   uint32_t pulses_seg1, float freq_mid,
                                   uint32_t pulses_seg2, float freq_target) {
  uint32_t total_steps = pulses_seg1 + pulses_seg2;
  if (total_steps < 2u)
    total_steps = 2u;

  // Inicializar estado interno de curvoas
  motor->is_braking = false;
  motor->freq_start_hz = freq_start;
  motor->freq_target_hz = freq_target;
  motor->current_freq_mid = freq_mid;

  motor->total_ramp_steps = total_steps;
  motor->transition_step_idx = pulses_seg1;
  motor->current_step_idx = 0;

  motor->current_slope1 = 0.0f;
  if (pulses_seg1 > 0)
    motor->current_slope1 = (freq_mid - freq_start) / (float)pulses_seg1;

  motor->current_slope2 = 0.0f;
  if (pulses_seg2 > 0)
    motor->current_slope2 = (freq_target - freq_mid) / (float)pulses_seg2;

  motor->current_phase = TMC2209_PHASE_NONE; // Modo individual

  // Llenar BufA y BufB iniciales
  fill_ping_pong_buffer(motor, motor->bufA); // Consume pasos 0 a 15
  fill_ping_pong_buffer(motor, motor->bufB); // Consume pasos 16 a 31
  motor->active_buffer_is_A = true;

  // Y el steady state para el final total por si a caso (aunque podemos dejar
  // al ping pong clavado resolviéndolo)
  build_constant_cycles_pair(motor->steady_buf, freq_target);

  const uint dreq = pio_get_dreq(motor->pio, motor->sm, true);
  volatile void *pio_txf = &motor->pio->txf[motor->sm];

  dma_channel_abort(motor->dma_ramp_ch);
  dma_channel_abort(motor->dma_steady_ch);
  dma_channel_abort(motor->dma_stop_ch);

  pio_sm_set_enabled(motor->pio, motor->sm, false);
  pio_sm_clear_fifos(motor->pio, motor->sm);
  pio_sm_exec(
      motor->pio, motor->sm,
      pio_encode_jmp(motor->offset + tmc2209_stepgen_offset_dma_stream));
  pio_sm_set_enabled(motor->pio, motor->sm, true);

  // DMA steady_ch funcionará como "segundo canal" del ping pong usando bufB
  // temporalmente Opcionalmente podemos usar ramp_ch + steady_ch de forma
  // cruzada. Vamos a cruzarlos: ramp_ch envía memoria en read_addr. Encadena a
  // steady_ch. steady_ch envía memoria en read_addr. Encadena a ramp_ch. Con
  // interrupciones encendidas solo en ramp_ch, el procesador atiende.

  dma_channel_config c_a = dma_channel_get_default_config(motor->dma_ramp_ch);
  channel_config_set_transfer_data_size(&c_a, DMA_SIZE_32);
  channel_config_set_read_increment(&c_a, true);
  channel_config_set_write_increment(&c_a, false);
  channel_config_set_dreq(&c_a, dreq);
  channel_config_set_chain_to(&c_a, motor->dma_steady_ch); // Chain to B

  dma_channel_config c_b = dma_channel_get_default_config(motor->dma_steady_ch);
  channel_config_set_transfer_data_size(&c_b, DMA_SIZE_32);
  channel_config_set_read_increment(&c_b, true);
  channel_config_set_write_increment(&c_b, false);
  channel_config_set_dreq(&c_b, dreq);
  channel_config_set_chain_to(&c_b, motor->dma_ramp_ch); // Chain back to A

  // Configuramos los canales, sin iniciarlos
  dma_channel_configure(motor->dma_steady_ch, &c_b, pio_txf, motor->bufB,
                        TMC2209_PING_PONG_BUFFER_WORDS, false);

  dma_channel_configure(motor->dma_ramp_ch, &c_a, pio_txf, motor->bufA,
                        TMC2209_PING_PONG_BUFFER_WORDS, false);

  // Activar interrupción cuando TERMINA ramp_ch (bufA) y steady_ch (bufB)
  dma_channel_set_irq0_enabled(motor->dma_ramp_ch, true);
  dma_channel_set_irq0_enabled(motor->dma_steady_ch, true);

  if (motor->mode != TMC2209_MODE_RUN_CW &&
      motor->mode != TMC2209_MODE_RUN_CCW) {
    motor->mode = TMC2209_MODE_RUN_CW; // Default si estaba detenido
  }

  // Iniciar el baile con el canal A
  dma_channel_start(motor->dma_ramp_ch);
}

void tmc2209_stop_2part_curve_dma(TMC2209_t *motor, float freq_start,
                                  uint32_t pulses_seg1, float freq_mid,
                                  uint32_t pulses_seg2, float freq_target) {
  uint32_t total_steps = pulses_seg1 + pulses_seg2;
  if (total_steps < 2u)
    total_steps = 2u;

  motor->is_braking = true;

  motor->freq_start_hz = freq_start;
  motor->freq_target_hz = freq_target;
  motor->current_freq_mid = freq_mid;

  motor->total_ramp_steps = total_steps;
  motor->transition_step_idx = pulses_seg1;
  motor->current_step_idx = 0;

  motor->current_slope1 = 0.0f;
  if (pulses_seg1 > 0)
    motor->current_slope1 = (freq_mid - freq_start) / (float)pulses_seg1;

  motor->current_slope2 = 0.0f;
  if (pulses_seg2 > 0)
    motor->current_slope2 = (freq_target - freq_mid) / (float)pulses_seg2;

  motor->current_phase = TMC2209_PHASE_NONE; // Modo individual

  fill_ping_pong_buffer(motor, motor->bufA);
  fill_ping_pong_buffer(motor, motor->bufB);
  motor->active_buffer_is_A = true;

  build_constant_cycles_pair(
      motor->steady_buf,
      freq_target); // Opcional, pero al ser stop igual no se usará.

  const uint dreq = pio_get_dreq(motor->pio, motor->sm, true);
  volatile void *pio_txf = &motor->pio->txf[motor->sm];

  dma_channel_abort(motor->dma_ramp_ch);
  dma_channel_abort(motor->dma_steady_ch);
  dma_channel_abort(motor->dma_stop_ch);

  pio_sm_set_enabled(motor->pio, motor->sm, false);
  pio_sm_clear_fifos(motor->pio, motor->sm);
  pio_sm_exec(
      motor->pio, motor->sm,
      pio_encode_jmp(motor->offset + tmc2209_stepgen_offset_dma_stream));
  pio_sm_set_enabled(motor->pio, motor->sm, true);

  dma_channel_config c_a = dma_channel_get_default_config(motor->dma_ramp_ch);
  channel_config_set_transfer_data_size(&c_a, DMA_SIZE_32);
  channel_config_set_read_increment(&c_a, true);
  channel_config_set_write_increment(&c_a, false);
  channel_config_set_dreq(&c_a, dreq);
  channel_config_set_chain_to(&c_a, motor->dma_steady_ch);

  dma_channel_config c_b = dma_channel_get_default_config(motor->dma_steady_ch);
  channel_config_set_transfer_data_size(&c_b, DMA_SIZE_32);
  channel_config_set_read_increment(&c_b, true);
  channel_config_set_write_increment(&c_b, false);
  channel_config_set_dreq(&c_b, dreq);
  channel_config_set_chain_to(&c_b, motor->dma_ramp_ch);

  dma_channel_configure(motor->dma_steady_ch, &c_b, pio_txf, motor->bufB,
                        TMC2209_PING_PONG_BUFFER_WORDS, false);
  dma_channel_configure(motor->dma_ramp_ch, &c_a, pio_txf, motor->bufA,
                        TMC2209_PING_PONG_BUFFER_WORDS, false);

  dma_channel_set_irq0_enabled(motor->dma_ramp_ch, true);
  dma_channel_set_irq0_enabled(motor->dma_steady_ch, true);

  dma_channel_start(motor->dma_ramp_ch);
}

void tmc2209_move_2part_profile_dma(
    TMC2209_t *motor, float freq_start, uint32_t accel_pulses_1,
    float accel_freq_mid, uint32_t accel_pulses_2, float freq_target,
    uint32_t steady_pulses, uint32_t decel_pulses_1, float decel_freq_mid,
    uint32_t decel_pulses_2, float freq_end) {
  // Usamos start como base para acelerar primero
  tmc2209_start_2part_curve_dma(motor, freq_start, accel_pulses_1,
                                accel_freq_mid, accel_pulses_2, freq_target);

  // Y pisamos las fases internas para cachear el resto del perfil dinámico
  motor->current_phase = TMC2209_PHASE_ACCEL;
  motor->steady_total_steps = steady_pulses;

  uint32_t d_total = decel_pulses_1 + decel_pulses_2;
  if (d_total < 2)
    d_total = 2; // Prevenir división por cero si envían rampa vacía final
  motor->decel_total_steps = d_total;
  motor->decel_transition_step_idx = decel_pulses_1;
  motor->decel_freq_mid = decel_freq_mid;
  motor->decel_freq_target_hz = freq_end;

  motor->decel_slope1 = 0.0f;
  if (decel_pulses_1 > 0) {
    motor->decel_slope1 =
        (decel_freq_mid - freq_target) / (float)decel_pulses_1;
  }

  motor->decel_slope2 = 0.0f;
  if (decel_pulses_2 > 0) {
    motor->decel_slope2 = (freq_end - decel_freq_mid) / (float)decel_pulses_2;
  }
}

void tmc2209_stop_s_curve_dma(TMC2209_t *motor, float freq_end_hz,
                              uint ramp_steps) {
  if (ramp_steps > TMC2209_DMA_MAX_STEPS)
    ramp_steps = TMC2209_DMA_MAX_STEPS;
  if (ramp_steps < 2u)
    ramp_steps = 2u;

  float f_end = freq_end_hz;
  if (f_end < 0.1f)
    f_end = 0.1f;
  // Usamos agresividad 2 por defecto para parada suave
  build_s_curve_cycles(motor->stop_buf, ramp_steps, motor->freq_target_hz,
                       f_end, motor->duty_cycle, 2);

  const uint dreq = pio_get_dreq(motor->pio, motor->sm, true);
  volatile void *pio_txf = &motor->pio->txf[motor->sm];

  dma_channel_abort(motor->dma_ramp_ch);
  dma_channel_abort(motor->dma_steady_ch);
  dma_channel_abort(motor->dma_stop_ch);

  // Reiniciar PIO
  pio_sm_set_enabled(motor->pio, motor->sm, false);
  pio_sm_clear_fifos(motor->pio, motor->sm);
  pio_sm_exec(
      motor->pio, motor->sm,
      pio_encode_jmp(motor->offset + tmc2209_stepgen_offset_dma_stream));
  pio_sm_set_enabled(motor->pio, motor->sm, true);

  dma_channel_set_irq0_enabled(motor->dma_stop_ch, true);

  dma_channel_config c_stop =
      dma_channel_get_default_config(motor->dma_stop_ch);
  channel_config_set_transfer_data_size(&c_stop, DMA_SIZE_32);
  channel_config_set_read_increment(&c_stop, true);
  channel_config_set_write_increment(&c_stop, false);
  channel_config_set_dreq(&c_stop, dreq);
  dma_channel_configure(motor->dma_stop_ch, &c_stop, pio_txf, motor->stop_buf,
                        2u * ramp_steps, true);
}

void tmc2209_change_frequency_dma(TMC2209_t *motor, float freq_new_hz,
                                  uint ramp_steps) {
  // Reutilizamos la lógica de start_s_curve_dma, pero usando la frecuencia
  // actual como inicio Nota: Esto detiene momentáneamente el DMA para
  // reconfigurar. Para una transición perfecta sin glitches se requeriría doble
  // buffer o ping-pong DMA, pero esta implementación es suficiente para cambios
  // rápidos.

  // Si el motor no estaba corriendo, usar freq_new_hz como inicio también
  // (arranque suave)
  float current_freq = motor->freq_target_hz;
  if (motor->mode == TMC2209_MODE_STANDBY_HOLD ||
      motor->mode == TMC2209_MODE_STANDBY_FREE) {
    current_freq = 10.0f; // Frecuencia mínima de arranque
  }

  // Usar agresividad 2 por defecto en cambio de frecuencia
  tmc2209_start_s_curve_dma(motor, current_freq, freq_new_hz, motor->duty_cycle,
                            ramp_steps, 2);
}

void tmc2209_move_forward_s_curve(TMC2209_t *motor, float freq_start,
                                  float freq_end, uint ramp_steps) {
  tmc2209_set_direction(motor, true); // Configurar dirección de avance
  tmc2209_start_s_curve_dma(motor, freq_start, freq_end, 0.5f, ramp_steps, 2);
}

void tmc2209_move_backward_s_curve(TMC2209_t *motor, float freq_start,
                                   float freq_end, uint ramp_steps) {
  tmc2209_set_direction(motor, false); // Configurar dirección de avance
  tmc2209_start_s_curve_dma(motor, freq_start, freq_end, 0.5f, ramp_steps, 2);
}
