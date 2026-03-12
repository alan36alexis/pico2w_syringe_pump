# Pico W Syringe Pump

Este proyecto implementa el control de una bomba de jeringa de alta precisión utilizando la placa **Raspberry Pi Pico W/2W (RP2040/RP2350)**. 

Se destaca por el uso concurrente de un driver para motores paso a paso **TMC2209**, un sensor de presión SPI **Honeywell HSC**, conectividad Wi-Fi, y la utilización extensiva del hardware de la placa (DMA, Hardware Spinlocks, PIO).

## Arquitectura Dual-Core Híbrida

Para garantizar estabilidad, respuesta en tiempo real (hard real-time) y funciones conectadas en un mismo microcontrolador, el proyecto divide lógicamente las responsabilidades de procesamiento entre ambos núcleos disponibles bajo una arquitectura híbrida:

### **Core 1: Baremetal (Tiempo Real)**
El núcleo 1 se ejecuta **sin sistema operativo (Baremetal)** impulsado completamente por interrupciones de hardware, temporizadores dedicados, DMA (Direct Memory Access) y algoritmos bloqueantes controlados.
- **Responsabilidades:**
  - Comunicación SPI1 ultrarrápida (1 MHz) con el sensor de presión Honeywell para monitorear sobrepresiones con lecturas deterministas cada 500 ms.
  - Generación de pulsos para el motor usando la abstracción en C hacia el componente PIO y perfiles avanzados en DMA S-Curve y Trapezoidales para aceleración/desaceleración suave del TMC2209.
  - Configuración UART bidireccional asíncrona dedicada (57600 baudios) para programar microstepping, corriente del motor dinámicamente, y sensar pasivamente *StallGuard* (choques sin final de carrera físico) leyendo registros del TMC2209.
  - Lectura en alta frecuencia en modo "polling" (mientras las transferencias DMA operan en paralelo) de los finales de carrera pasivos mediante GPIO (`START_PIN` / `END_PIN`).

### **Core 0: FreeRTOS (Procesamiento Concurrente Asíncrono)**
El núcleo 0 ejecuta un kernel de **FreeRTOS** y centraliza todas las entradas, salidas globales del usuario (puerto serie / WiFi CYW43), temporizadores relajados y telemetría general.
- **Responsabilidades:**
  - Inicialización del subsistema Wi-Fi e interacción asíncrona por redes.
  - Parpadeo dinámico del LED nativo de la placa por intermedio de tareas RTOS (latidos de estado).
  - Tarea central `task_logger` para despachar, formatear e imprimir (`printf`) los eventos diagnosticados previamente en el Core 1.

---

## Comunicación Inter-Núcleo (Cross-Core Logging)

El núcleo 1 está forzado a operar en plazos estrictos de microsegundos para preservar la pureza de los perfiles de velocidad del motor y la seguridad ante sobrepresión de la jeringa. Sin embargo, en el SDK de C/C++ de Pico, llamar a constantes funciones como `printf()` para diagnosticar o reportar la presión implica un serio cuello de botella y riesgo de *Kernel Panic*.
**¿Por qué?** Porque `printf()` sobre el puerto serie/USB protege sus flujos a través de *mutexes/spinlocks* globales de hardware. Si el Core 0 se encuentra imprimiendo o demorado en una rutina del CYW43, el intento de hacer `printf()` en el Core 1 bloqueará al Core 1 completamente por una cantidad impredecible de tiempo.

### La Solución: Hardware Spinlock Queues (Cola de hardware del SDK de Pico)
El proyecto mitiga por completo este problema valiéndose de la librería estándar `pico/util/queue.h`, creando así el componente `crosscore_logger`:

1. **`LogMessage_t` (Payload optimizado):** Una estructura `union` optimizada permite empacar en la memoria RAM el identificador numérico de qué evento ocurrió (ej: `LOG_EVENT_PRESSURE_ALERT`) adjunto de una porción pura de solo 4 bytes del valor en el momento del evento (ej: `float pressure_psi`).
2. **Transferencia No-Bloqueante (`queue_try_add()`):** Cuando ocurre una falla crítica o una lectura correcta en el *Baremetal* (Core 1), invoca rápidamente funciones como `logger_send_pressure_update(float psi)`. Internamente sólo intentan insertarse asíncronamente en la cola RAM inter-núcleo en nanosegundos y regresan inmediatamente a mover el motor, incluso si la cola se saturó de mensajes y los datos se pierden.
3. **Impresión Asíncrona (FreeRTOS `task_logger`):** En el Core 0, el FreeRTOS ejecuta un ciclo cada `10ms` que explora la cola. Extrae (*pop*) todos los eventos acumulados y se hace cargo del retardo bloqueante de utilizar `printf()`, convirtiendo los crudos `floats` y `uint_32` transmitidos por el Core 1 a extensos renglones entendibles para el operador de diagnóstico en el monitor serie, sin obstaculizar la maquinaria.
