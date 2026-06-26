# Prompt para Claude Code: Refactorización a Command Broker

**Rol y Objetivo:**
Actúa como un desarrollador experto en C para sistemas embebidos y FreeRTOS. Tu misión es refactorizar la arquitectura de ejecución de comandos del proyecto `pico2w_syringe_pump`. 
Actualmente, el proyecto utiliza un excelente patrón EDA (Event Broker) para la telemetría, pero la gestión de comandos es monolítica, está acoplada al módulo HMI y se basa en el paso ineficiente de strings (String Typing). Queremos migrar a un patrón **Command Broker / Command Bus**.

**Problemas de la Arquitectura Actual:**
1. Los generadores de comandos (Serial CLI en `core0_main.c` y MQTT en `mqtt_client.c`) delegan el parseo de strings a `pump_hmi_parse_and_execute` en `pump_hmi.c`.
2. Si la HMI no reconoce el comando, hace un fallback a `cmd_parse_and_execute` en `crosscore_cmd.c`, el cual vuelve a hacer `strncmp` y `sscanf`.
3. Esto genera un alto acoplamiento, duplicación de esfuerzo (parsing de strings repetitivo) y no permite saber el origen del comando para responder con un ACK.

**Instrucciones de Implementación:**

Por favor, implementa la nueva arquitectura siguiendo estrictamente estos pasos:

### 1. Definir los Tipos de Comandos (Nuevo archivo: `system_commands.h`)
Crea un archivo `src/system_commands.h` que defina:
- Un enum `CommandSource_t` (ej. `CMD_SRC_SERIAL`, `CMD_SRC_MQTT`, `CMD_SRC_HMI`).
- Un enum `SysCmdID_t` con todos los comandos posibles del sistema (movimientos del motor, configuraciones wifi/mqtt, etc). Toma de referencia `Core1CmdID_t` en `crosscore_cmd.h` y agrega los comandos lógicos.
- Un struct `SystemCommand_t` que contenga `CommandSource_t source`, `SysCmdID_t id`, y un `union` con las cargas útiles (payloads) específicas de cada comando.

### 2. Crear el Command Broker (Nuevos archivos: `command_broker.h` y `command_broker.c`)
- Implementa una cola de FreeRTOS (`g_command_q`) para recibir `SystemCommand_t`.
- Implementa la función de inicio y la tarea `task_command_broker`.
- Esta tarea debe desencolar comandos, realizar validación de alto nivel (por ejemplo, verificando el estado de la máquina de estados) y enrutar la ejecución:
  - Enviar comandos de movimiento a la cola del Core 1 usando las funciones de `crosscore_cmd.c`.
  - Ejecutar configuraciones del sistema (llamando a `config_manager`).
- Opcionalmente (o como TODO) prepara el terreno para que el broker emita un evento de ACK o NACK (vía `event_broker`) indicando si el comando fue aceptado o rechazado.

### 3. Refactorizar los Generadores (Productores)
- **Serial CLI (`core0_main.c`):** En `task_cli`, modifica el comportamiento para que al recibir una línea completa, se parsee directamente allí (puedes aislar el código de parseo de strings a una pequeña función estática) convirtiéndolo en un `SystemCommand_t` y enviándolo a la cola del `command_broker`.
- **MQTT (`mqtt_client.c`):** Modifica `mqtt_rx_task` o el callback correspondiente para que parsee el texto/JSON entrante y ensamble un `SystemCommand_t`, encolándolo en el `command_broker`.
- **HMI (`pump_hmi.c`):** Limpia este archivo eliminando `pump_hmi_parse_and_execute(const char *str)`. El HMI debe limitarse a generar eventos de UI y emitir `SystemCommand_t` al `command_broker` sin necesidad de parsear strings externos.

### 4. Limpiar el Enrutamiento de Bajo Nivel
- **`crosscore_cmd.c`:** Elimina por completo la función `cmd_parse_and_execute(const char *payload_str)` ya que el parseo de texto (strings) ahora es responsabilidad exclusiva de los generadores (borde del sistema) antes de entrar al Command Broker.

**Reglas Críticas:**
- Modifica los archivos uno por uno y verifica la coherencia de las dependencias (`#include`).
- Asegúrate de actualizar el archivo de CMake (`CMakeLists.txt` si existe) para incluir los nuevos archivos `command_broker.c`.
- Mantén el código limpio, comentado y respeta la estructura actual de logs (`printf` / `LOG_DEBUG`).
- Presenta los cambios de forma ordenada y pide revisión si encuentras alguna ambigüedad antes de refactorizar módulos grandes.
