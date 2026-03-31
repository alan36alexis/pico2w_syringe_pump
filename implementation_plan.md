# Integración de Encoder de Cuadratura con PIO

Esta es la propuesta técnica para integrar el encoder rotativo mediante PIO, confirmando que **Core 1 es la opción ideal y más robusta** para esta tarea.

## Por qué Core 1 es la mejor opción
1. **Baja Latencia y Lazo Cerrado:** Core 1 es el responsable del control del motor paso a paso (TMC2209). Leer la posición real del encoder directamente en el mismo hilo que supervisa el estado del motor (`tmc2209_is_moving`) te permite tener reacción inmediata (ej. frenar si el encoder no detecta movimiento y asume un atasco, o parar al alcanzar un recuento específico) sin pasar por IPC (Inter-Processor Communication).
2. **Descarga al Core 0:** Core 0 actualmente maneja FreeRTOS, LwIP (WiFi) y MQTT. Mantener el bucle rápido de adquisición sensórica y control en Core 1 evita introducir "jitter" a la pila de red y previene la interrupción de procesos de networking.
3. **Hardware Independiente (PIO):** El seguimiento real de los flancos A/B del encoder lo hace el hardware de PIO a altas frecuencias, de forma totalmente paralela a los Cores. La CPU solo necesita vaciar la FIFO del PIO y leer el contador cuando lo necesita (durante el bucle del motor).

---

## Cambios Propuestos

### 1. Archivos base y CMake
- **[NEW] `src/quadrature_encoder.pio`**: Copiaremos el código nativo y el "C-SDK assembler wrapper" oficial del `pico-sdk/examples`.
- **[MODIFY] `CMakeLists.txt`**: Agregaremos la directiva `pico_generate_pio_header(pico2w_syringe_pump src/quadrature_encoder.pio)` e incluiremos la vinculación con la librería `hardware_pio`.

### 2. Módulo `core1_main.c`
Se agregará la inicialización de PIO antes del bucle principal de Core 1:
```c
// Se seleccionarán 2 pines consecutivos para el encoder (Falta definir cuáles)
uint offset = pio_add_program(pio1, &quadrature_encoder_program);
uint sm = pio_claim_unused_sm(pio1, true);
quadrature_encoder_program_init(pio1, sm, PINES_ENCODER_BASE, 0); 
```

Dentro del bucle de movimiento (`while (tmc2209_is_moving(&motor1))`), y también en estado *idle*, leeremos el número atómico provisto por el PIO:
```c
int32_t current_count = quadrature_encoder_get_count(pio1, sm);
// (Opcional) Implementar lógica de validación de avance
```

### 3. Log/Telemetry (`crosscore_logger` / MQTT)
- **[MODIFY] `crosscore_logger` & `core0_main`**: Expondremos una forma de enviar de forma periódica el valor del encoder al Core 0 como datos de telemetría (para que viaje por MQTT, ej: `syringe_pump/telemetry`).

---

> [!IMPORTANT]
> ## ❓ Preguntas Abiertas (User Review Required)
> 
> Para proceder con la ejecución técnica, por favor confírmame lo siguiente:
> 1. **Pines del Encoder:** La máquina de estados PIO requiere que las señales A y B estén en **pines GPIO consecutivos** (por ejemplo, GPIO 10 y GPIO 11). ¿Cuáles son los números de GPIO físicos exactos que reservaste para esto?
> 2. **Instancia de PIO:** ¿Sabes si la librería TMC2209 (o DMA utils) ya está usando `pio0` al completo? Mi plan es asignar esto en `pio1` para evitar colisiones.
> 3. **Lógica local:** Por ahora, ¿el objetivo es solo exponer la lectura actual mandándola como un log por UART/MQTT, o quieres implementar inmediatamente una lógica matemática en Core 1 (como "detener el motor si el delta del encoder no coincide con el delta de los pasos")?

## Plan de Verificación
1. **Compilación:** Asegurar que el CMake genera los headers `.pio.h` y linkea todo correctamente.
2. **Prueba Manual / Consola:** Leer la posición del encoder rotándolo a mano (sin mover el motor) y verificar que Core 1 obtiene cuentas coherentes incrementales / decrementales y los envía hasta la salida serial o MQTT.
