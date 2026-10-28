#include <stdio.h>
#include "pico/stdlib.h"
#include "test_modes.h"
#include "serial_consumer.h"

// Include main definition simply for constants if needed, 
// strictly we should share constants in a common header but for now we'll redefine or use args.
// For simplicity, we assume the motor is already initialized.

void test_rpm_movement_demo(TMC2209_t *motor) {
    serial_log("TST", "demo=RPM_START");
    
    // Calcular valor para 60 RPM
    // Nota: Asegúrate de que steps_per_rev y microsteps estén configurados en 'motor'
    int32_t speed_val = tmc2209_compute_vactual(motor, 60.0f);

    serial_log("TST", "vactual=%d rpm=60", speed_val);
    
    // Mover en sentido horario (UART)
    tmc2209_set_vactual(motor, speed_val);

    sleep_ms(2000);

    serial_log("TST", "demo=STOP");
    // Detener (Devuelve control a pines STEP/DIR)
    tmc2209_set_vactual(motor, 0);
}

void test_stallguard_analysis(TMC2209_t *motor) {
    serial_log("TST", "demo=STALLGUARD_START");
    
    // Nota: Esto requiere que las funciones auxiliares de corriente (tmc2209_set_current_amps) 
    // sean accesibles o reimplementadas aquí. Asumiremos acceso o usaremos tmc2209_set_current raw.
    
    float run_current = 0.4f;
    // uint32_t plotter_marker = 0;
    // float freq = 0.0f;
    // int nsteps = 0;
    
    // Configuracion inicial de ejemplo
    // tmc2209_set_current(motor, ...); // Usar valores raw si no tenemos acceso al helper de amps
    
    const float alpha = 0.2f; 
    uint32_t sg_result_filtered = 0;

    // Bucle infinito de prueba (el usuario deberá reiniciar o adaptar para salir)
    while (true) {
        if (tmc2209_get_mode(motor) == TMC2209_MODE_STANDBY_HOLD) {
            run_current += 0.1f;
            
            // Simulación de lógica de variación de corriente
            if (run_current > 1.5f) {
                run_current = 0.2f;
                // Desenergizar bobinas para bajar corriente
                tmc2209_enable(motor, false);
                sleep_ms(100);
                tmc2209_enable(motor, true);
            }
            
            // Aquí deberías llamar a tu función de setear corriente en Amperes si la mueves a un common.h
            // Por ahora solo imrimimos
            serial_log("TST", "run_current=%.2fA", run_current);
            
            sleep_ms(10);
            // tmc2209_set_current_amps(motor, run_current, 0.05f); // Necesita ser pública
            sleep_ms(10);

            // tmc2209_send_nsteps_at_freq(motor, ...);
            sleep_ms(100);
        }

        // Lectura de StallGuard
        uint32_t sg_result_raw = tmc2209_read_register(motor, 0x41); // SG_RESULT
        sg_result_filtered = (sg_result_raw * alpha) + (sg_result_filtered * (1 - alpha));
        
        serial_log("TST", "sg_raw=%u sg_filt=%u", sg_result_raw, sg_result_filtered);
        sleep_ms(100);
    }
}

void test_microstepping_cycle(TMC2209_t *motor, bool use_uart_mode) {
    serial_log("TST", "demo=MICROSTEP_START");
    
    static int ms_idx = 0;
    const TMC2209_Microsteps_t ms_options[] = {
        TMC2209_MICROSTEPS_1, TMC2209_MICROSTEPS_2, 
        TMC2209_MICROSTEPS_4, TMC2209_MICROSTEPS_16
    };

    // Ciclo
    if (use_uart_mode) {
        tmc2209_set_microstepping_uart(motor, ms_options[ms_idx]);
    } else {
        tmc2209_set_microstepping_by_pins(motor, ms_options[ms_idx]);
    }

    uint16_t msteps_read = tmc2209_get_microsteps(motor);
    serial_log("TST", "ms_idx=%d msteps_read=%d", ms_idx, msteps_read);

    ms_idx++;
    if (ms_idx >= 4) ms_idx = 0;
}

void test_ioin_polling(TMC2209_t *motor) {
    serial_log("TST", "demo=IOIN_START");
    
    tmc2209_set_direction(motor, true);
    
    uint32_t ioin_val = tmc2209_read_register(motor, 0x06);
    serial_log("TST", "ioin=0x%08X dir=%d", ioin_val, (ioin_val >> 9) & 1);
    
    sleep_ms(500);
    
    tmc2209_set_direction(motor, false);
    
    ioin_val = tmc2209_read_register(motor, 0x06);
    serial_log("TST", "ioin=0x%08X dir=%d", ioin_val, (ioin_val >> 9) & 1);
    
    sleep_ms(500); 
}
