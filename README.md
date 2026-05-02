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

---

## Interfaz de Comandos y Telemetría (MQTT / CLI)

El sistema soporta el envío de comandos de movimiento y la configuración dinámica de credenciales mediante *dos interfaces unificadas*:
1. **MQTT**: Mediante la subscripción al tópico de comandos definido y publicando payloads de texto.
2. **CLI (Puerto Serial)**: Abriendo la consola UART/USB de la Pico y tecleando los comandos directamente.

### Comandos de Operación Disponibles (Vía CLI o MQTT payload)

| Comando Payload | Descripción | Notas |
|---|---|---|
| `stop_imm` o `STOP_IMM` | Parada inmediata (Hard Stop) | Frena el motor deteniendo su generador abruptamente. |
| `stop` o `STOP` | Parada suave (Soft Stop) | Desacelera respetando la rampa configurada hasta llegar a 0. |
| `fsm_home` | FSM: Inicio (Homing) | Inicia la secuencia de búsqueda del tope de inicio. |
| `fsm_search` | FSM: Buscar jeringa | Inicia la búsqueda del émbolo de la jeringa. |
| `fsm_dispense,<TARGET>,<VEL>` | FSM: Dosificar | Inicia la dosificación a una posición dada (um) y velocidad (um/s). Ej: `fsm_dispense,10000.0,450.0` |
| `fsm_search_eot` | FSM: Buscar Fin de Carrera | Busca el tope de fin de carrera (End Of Travel). |
| `fsm_reset` | FSM: Reset | Resetea la máquina de estados. |
| `fsm_cont` | FSM: Continuar | Continúa la dosificación previamente pausada. |
| `fsm_occ_rel` | FSM: Liberar Oclusión | Retrocede el motor para liberar presión tras una oclusión. |
| `fsm_resume` | FSM: Reanudar | Reanuda la operación después de resolver un evento. |
| `fsm_calibrate` | FSM: Calibrar Encoder | Secuencia de ida y vuelta a los topes para capturar en RAM el recorrido máximo en encoder. |
| `home_start,<velocidad>` | Busca el inicio / Homing (Atrás) | Motor se mueve negativo a velocidad constante hasta hallar el tope. Ej: `home_start,1200` |
| `home_end,<velocidad>` | Busca el fin / Homing (Adelante)| Motor se mueve positivo a velocidad constante hasta hallar el tope. Ej: `home_end,1200` |
| `nsteps,<pasos>,<freq_hz>`| Movimiento por Pasos puros | Inyecta N pasos fijos a cierta frecuencia. Ej: `nsteps,3200,500.0` |
| `<velocidad>,<posicion>` | Movimiento Lineal | Mueve a una posición dada (um) a cierta velocidad (um/s). Falla si el payload no es reconocido como ningún otro comando previo. Ej: `500.0,15000.0` |

### Comandos de Configuración Exclusivos de CLI

Actualmente, estos comandos son accesibles mediante la consola serial y se utilizan para guardar datos persistentes en la memoria **Flash** del microcontrolador (Thread-Safe mediante *multicore lockout*).

| Comando CLI | Descripción |
|---|---|
| `config_wifi,<SSID>,<PASS>`| Guarda temporalmente en RAM y usa las nuevas credenciales de Wi-Fi. |
| `config_mqtt,<IP>,<PORT>` | Guarda temporalmente en RAM y usa la nueva IP/Puerto del Broker. |
| `set_ssid,<SSID>` | Cambia únicamente el SSID del Wi-Fi en RAM. |
| `set_wpass,<PASS>` | Cambia únicamente la contraseña del Wi-Fi en RAM. |
| `set_mqtt_ip,<IP>` | Cambia únicamente la IP del Broker MQTT en RAM. |
| `set_mqtt_port,<PORT>`| Cambia únicamente el Puerto del Broker MQTT en RAM. |
| `config_save` | Escribe los valores de RAM en la Memoria Flash profunda de forma definitiva. (Solo funcionará si el motor no se está moviendo). |
| `config_info` | Muestra un resumen de variables actuales cargadas en el gestor. |
| `reconnect` | Efectúa un reseteo *suave* (Soft Reset) del hardware Wi-Fi desasociándolo de su actual red (LwIP leave) obligándolo a re-engancharse y conectar MQTT con la nueva configuración sin reiniciar el procesador. |
| `net_disable` | Apaga el chip Wi-Fi (desactiva RF y tareas de red) para máximo ahorro de batería. |
| `net_enable` | Enciende el chip Wi-Fi y restaura la conectividad de red a sus valores guardados. |

### Tópicos MQTT 

* **Comandos entrantes hacia la Pico**:
  * `syringe_pump/cmd` - (Espera los Payloads descritos más arriba).

* **Telemetría y Logging (Publicados por la Pico)**:
  * `syringe_pump/telemetry` - JSON global con estado general enviado cada X segundos.
  * `syringe_pump/telemetry/system_health` - JSON con datos de memoria/temperatura del procesador.
  * `syringe_pump/log/system` - Advertencias, correcciones y eventos genéricos del logger centralizado.
  * `syringe_pump/log/motor_state` - Eventos de parada, retroceso, movimiento y límites de carrera.
  * `syringe_pump/log/motor_regs` - Diagnóstico UART con sus registros StallGuard, DRV_STATUS, inicialización cruzada, etc.
  * `syringe_pump/log/pressure` - Eventos sobre alertas de sobre-presión o retornos de protección (desde el sensor de presión SPI).
  * `syringe_pump/log/encoder` y `.../encoder_indep` - Valores crudos o en cuadratura del Encoder si está habilitado.
  * `syringe_pump/log/encoder_speed` - PPS (Pulsos Por Segundo).
  * `syringe_pump/log/progress` - Telemetría pura del porcentaje completado (`progress_pct`) durante el movimiento.

### TODO
- [ ] Implementar libreria de control de TFT+Touch y lógica de menues. **WIP**
- [ ] Implementar control lazo cerrado (driver+motor PAP , encoder). **WIP**
- [ ] Implementar CLI para control del sistema. **WIP**
- [ ] Implementar el uso del watchdog multi-thread(event group o challenge-response).
- [ ] Implementar libreria para manejo de memoria no volatil. Actualmente se usa funciones de flash nativas del SDK, analizar uso de littlefs.
- [ ] Implementar mini database para guardar datos de uso, estado del sistema y logs..
- [ ] Implementar sincronización con hora actual + RTC.
- [ ] Implementar lectura de sensor de fuerza y lógica de seguridad asociada.
- [ ] Implementar control de modo bajo consumo(modo sleep).
- [ ] Implementar sistema de alarma (buzzer, led, logs) en base a salud y estado de sistema. **WIP**
- [ ] Implementar monitoreo y actuación sobre estado de energía (modo batería, nivel de carga, etc)