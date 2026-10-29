# Guía de Inicio: Stack Docker (Mosquitto + Node-RED + Simulador EDA) en Linux

Este documento describe la arquitectura, puesta en marcha y batería de pruebas de integración para el entorno de desarrollo y validación del dashboard de la **Bomba de Infusión a Jeringa**, ejecutado en contenedores Docker bajo entornos **Linux** (Ubuntu, Debian, Fedora, Arch o WSL2).

---

## 1. Arquitectura del Stack

El stack definido en [`docker/docker-compose.yml`](docker-compose.yml) orquesta tres servicios interconectados en una red puente (`bridge`):

```
                     ┌────────────────────────────────────────────────────────┐
                     │                 Host Linux (Docker)                    │
                     │                                                        │
                     │   ┌────────────────────────────────────────────────┐   │
                     │   │        Servicio: mosquitto (Broker MQTT)       │   │
                     │   │   - 1883: TCP MQTT (Firmware / Simulador)      │   │
                     │   │   - 9001: WebSockets MQTT (Dashboard Node-RED) │   │
                     │   └───────────────▲────────────────▲───────────────┘   │
                     │                   │                │                   │
                     │        bj/+/...   │                │   bj/+/...        │
                     │                   │                │                   │
  ┌──────────────┐   │   ┌───────────────▼────────┐  ┌────▼───────────────┐   │
  │ Navegador    │◄──┼───┤ Servicio: nodered      │  │ Servicio: simulator│   │
  │ Web (UI)     │   │   │ Dashboard v2           │  │ pump_simulator.py  │   │
  │ :1880        │   │   │ Flujos de control      │  │ Gemelo digital EDA │   │
  └──────────────┘   │   └────────────────────────┘  └────────────────────┘   │
                     └────────────────────────────────────────────────────────┘
```

### Contrato de Comunicación (Arquitectura EDA Multi-rate):
* `bj/{id}/telemetry/sensors` (500 ms, QoS 0): Presión hidráulica (`pressure_psi`, `pressure_mmhg`).
* `bj/{id}/telemetry/realtime` (500 ms, QoS 0): Sensores de bajo nivel (`lsw`, `encoder`, `adc`, `pcal9555`, `motor`).
* `bj/{id}/telemetry/motion` (1000 ms, QoS 0): Cinemática del motor (`encoder_count`, `position_mm`, `speed_ums`, `progress_pct`).
* `bj/{id}/telemetry/session` (2000 ms, QoS 0): Sesión clínica (`infused_ml`, `target_ml`, `rate_ml_h`, `elapsed_s`, `fsm_state`).
* `bj/{id}/event` (Disparo, QoS 1): Transiciones FSM de Core 1 (0 a 20) y alarmas (`occ`, `bub`, `eot`, `drv`).
* `bj/{id}/cmd` / `bj/{id}/cmd/ack` (Disparo, QoS 1): Comandos con envelope `{"cid": N, "cmd": "..."}` y acuses de recibo.
* `bj/{id}/status` (LWT retain, QoS 1): Presencia del equipo (`online` / `offline`).
* `bj/{id}/sim_fault` (Disparo, QoS 0): Inyección controlada de fallas físicas al simulador.

---

## 2. Requisitos Previos en Linux

### A. Docker y Docker Compose
Asegúrate de tener instalados Docker Engine y el plugin Compose v2:
```bash
docker --version
docker compose version
```

### B. Permisos de Usuario (Evitar `sudo`)
Para ejecutar comandos Docker sin privilegios de root:
```bash
sudo usermod -aG docker $USER
newgrp docker
```

### C. Herramientas CLI de MQTT (Recomendado)
Para pruebas en consola y monitoreo de paquetes:
```bash
# Debian / Ubuntu / Mint / WSL2
sudo apt update && sudo apt install -y mosquitto-clients jq

# Fedora / RHEL
sudo dnf install -y mosquitto jq

# Arch Linux
sudo pacman -S mosquitto jq
```

### D. Verificación de Puertos Libres
Verifica que ningún servicio local esté ocupando los puertos `1883`, `9001` o `1880`:
```bash
sudo ss -tulpn | grep -E ':(1883|9001|1880)'
```
*(Si tienes un servicio local de Mosquitto o Node-RED corriendo en el host, detenlo con `sudo systemctl stop mosquitto`).*

---

## 3. Puesta en Marcha (Startup)

### Paso 1: Ingresar al directorio `docker`
```bash
cd /ruta/hacia/pico2w_syringe_pump/docker
```

### Paso 2: Construir e Iniciar los Contenedores
```bash
docker compose up -d --build
```
> El flag `-d` corre en segundo plano (*detached*), y `--build` compila la imagen del simulador Python con sus dependencias (`paho-mqtt`).

### Paso 3: Verificar que todos los servicios estén `healthy` / `Up`
```bash
docker compose ps
```
Salida esperada:
```
NAME             IMAGE                     COMMAND                  SERVICE     CREATED         STATUS         PORTS
mosquitto        eclipse-mosquitto:2       "/docker-entrypoint.…"   mosquitto   5 seconds ago   Up 5 seconds   0.0.0.0:1883->1883/tcp, 0.0.0.0:9001->9001/tcp
nodered          nodered/node-red:latest   "./entrypoint.sh"        nodered     5 seconds ago   Up 5 seconds   0.0.0.0:1880->1880/tcp
pump_simulator   docker-simulator          "python pump_simulat…"   simulator   5 seconds ago   Up 5 seconds   
```

### Paso 4: Monitorear Logs en Vivo
Para inspeccionar la salida en consola del simulador:
```bash
docker compose logs -f simulator
```
Para ver todos los logs juntos:
```bash
docker compose logs -f
```

---

## 4. Configuración Inicial del Dashboard Node-RED

1. Abre tu navegador web e ingresa al editor de Node-RED:
   👉 **[http://localhost:1880](http://localhost:1880)**
2. **Dependencia Dashboard 2.0 (si es una instalación limpia):**
   * Este flujo utiliza nodos de `@flowfuse/node-red-dashboard` (Dashboard 2.0).
   * Si tras iniciar un volumen limpio aparecen advertencias de nodos faltantes (`ui-base`, `ui-page`, etc.):
     1. Haz clic en el menú superior derecho `☰` ➔ **Manage palette** (Administrar paleta).
     2. Ve a la pestaña **Install**, busca `@flowfuse/node-red-dashboard` y haz clic en **Install**.
     *(En despliegues subsiguientes el paquete queda persistido en el volumen Docker `nodered-data`).*
3. **Importar Flujos:**
   * Haz clic en el menú superior derecho (tres líneas horizontales `☰`) ➔ **Import**.
   * Haz clic en **select a file to import** y selecciona el archivo [`flows_bomba.json`](flows_bomba.json) ubicado en esta carpeta (o pega su contenido en la caja de texto).
   * Selecciona **Import into: Current flow** (o nuevo flujo).
   * Haz clic en el botón rojo **Deploy** (arriba a la derecha).
4. **Acceder al Dashboard Clínico:**
   * Abre en una pestaña nueva:
     👉 **[http://localhost:1880/dashboard](http://localhost:1880/dashboard)**
   * En la barra de navegación lateral izquierda encontrarás 5 secciones:
     * **Monitor:** Badge de conexión, caudal instantáneo, curva de presión hidráulica a 2 Hz, estadísticas de infusión y pastillas de alarmas.
     * **Control:** Mando de la máquina de estados FSM (`fsm_home`, `fsm_search`, `fsm_dispense`), parada de emergencia y disparo de fallas.
     * **Terminal:** Consola interactiva Pub-Sub MQTT (ícono `>_`). Permite suscribirse con patrones (`bj/#`, `bj/+/event`, etc.), filtrar payloads, ver el stream en tiempo real con marcas de tiempo y publicar comandos o fallas.
     * **Real-time:** Panel de telemetría de hardware de bajo nivel (ícono chip). Monitorea en vivo los finales de carrera (`LSW_START`, `LSW_END`), el contador del encoder, los 3 canales ADC (`FRC_SENS`, `BUB_SENS`, `VIN_SENS`), las entradas del expansor I2C `PCAL9555` y los pines del driver TMC2209 (`ENA`, `DIR`, `StallGuard`).
     * **Gráficos:** Panel de tendencias históricas de los 3 canales ADC (`FRC_SENS`, `BUB_SENS`, `VIN_SENS`) con ventana deslizante de 1 hora (60 minutos) y botón de retorno rápido a Real-time. Accesible también desde el botón **Gráficos** de la tarjeta ADC.

---

## 5. Batería de Pruebas del Sistema (System Integration Testing)

Abre una o varias terminales en tu máquina Linux para ejecutar estas pruebas usando `mosquitto_sub` y `mosquitto_pub`.

### Prueba 1: Inspeccionar la Telemetría Multi-rate EDA en Tiempo Real
Abre un terminal y suscríbete a todo el árbol de telemetría:
```bash
mosquitto_sub -h localhost -t "bj/+/telemetry/#" -v
```
**Resultado esperado:**
* Cada 500 ms verás mensajes en `telemetry/sensors` con `pressure_psi` y `pressure_mmhg`.
* Cada 1000 ms verás mensajes en `telemetry/motion` con `position_mm`, `speed_ums` y `progress_pct`.
* Cada 2000 ms verás mensajes en `telemetry/session` con `infused_ml`, `target_ml`, `rate_ml_h` y `fsm_state`.

---

### Prueba 2: Monitorear Eventos FSM y Acuses de Recibo (ACKs)
En una terminal separada, deja escuchando los canales de eventos y respuestas:
```bash
mosquitto_sub -h localhost -t "bj/+/event" -t "bj/+/cmd/ack" -v
```

---

### Prueba 3: Rutina de Referenciamiento (Homing)
Envía la orden de ir a Home desde otra terminal:
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/cmd" -m '{"cid":10,"cmd":"fsm_home,1500"}'
```
**Resultado esperado:**
* El canal ACK responderá inmediatamente: `{"cid":10,"result":"accepted"}`.
* En `bj/bj-deadbeef/event` se registrará la secuencia de estados mecánicos:
  * `{"type":"state","from":3,"to":1}` (HOMING)
  * `{"type":"state","from":1,"to":2}` (REL_LSW_INI)
  * `{"type":"state","from":2,"to":3}` (LISTO EN HOME)
* El dashboard actualizará en vivo la etiqueta del estado a **`LISTO EN HOME`**.

---

### Prueba 4: Búsqueda y Acople de Jeringa
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/cmd" -m '{"cid":11,"cmd":"fsm_search,200"}'
```
**Resultado esperado:**
* El motor pasa a estado `ST_SEARCHING_SYRINGE` (4) y luego a `ST_SYRINGE_ENGAGED` (5) al detectar el contacto con el émbolo.

---

### Prueba 5: Iniciar Infusión Continua (Dispensado)
Inicia un dispensado de 30.000 µm (30 mm de recorrido) a 450 µm/s:
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/cmd" -m '{"cid":12,"cmd":"fsm_dispense,30000,450"}'
```
**Resultado esperado:**
* La FSM pasa a estado `6` (`ST_DISPENSING`).
* En el Dashboard:
  * La aguja del **Caudal** subirá a la tasa equivalente calculada.
  * La **Barra de Progreso** comenzará a llenarse.
  * Los contadores de volumen infundido y tiempo transcurrido avanzarán continuamente.
  * La posición lineal del émbolo (`position_mm`) irá en aumento.

---

### Prueba 6: Inyección de Alarma de Oclusión Hidráulica
Durante una infusión activa, simula un taponamiento en la aguja o línea IV:
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "fault_occ"
```
**Resultado esperado:**
1. El modelo de presión comenzará una rampa ascendente a ~100 mmHg/s (visible en la curva de Node-RED).
2. Al superar el umbral clínico configurado:
   * Se emitirá: `{"type":"alarm","code":"occ","level":2}`.
   * La pastilla **OCC** en el dashboard cambiará a rojo `⚠`.
   * Aparecerá una notificación flotante de alarma en la pantalla.
   * La FSM frenará de emergencia: pasa a `ST_OCCLUSION_STOPPING` (13) ➔ retroceso anti-bolus `ST_OCCLUSION_RELEASE` (14) ➔ `ST_OCCLUSION_PAUSED` (15).

---

### Prueba 7: Liberación de Oclusión y Reanudación
Limpia la falla simulada:
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "fault_clear"
```
Envía comando para reanudar la terapia:
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/cmd" -m '{"cid":13,"cmd":"fsm_resume"}'
```
**Resultado esperado:**
* La bomba retoma la infusión (`ST_DISPENSING`), la alarma se apaga y el indicador vuelve a verde.

---

### Prueba 8: Parada Inmediata de Emergencia (Safety Invariant)
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/cmd" -m '{"cid":14,"cmd":"stop_imm"}'
```
**Resultado esperado:**
* Detención incondicional instantánea. El caudal cae a 0.0 mL/h y la FSM vuelve al estado seguro de reposo (`ST_READY_AT_HOME`).

---

### Prueba 9: Inyección y Detección de Burbuja de Aire en Línea
Simula la detección por el sensor óptico de burbuja (BUB_SENS):
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "fault_bubble"
```
**Resultado esperado:**
* Se emite evento: `{"type":"alarm","code":"bub","level":2}`.
* La FSM pasa a estado `ST_FAULT` (12) y detiene el flujo (`rate_ml_h = 0.0`).
* La pastilla **BUB** en el dashboard se ilumina en rojo `⚠`. En la página **Real-time**, el canal ADC1 (`BUB_SENS`) cae por debajo de 1.5V y avisa `⚠ BURBUJA DE AIRE DETECTADA`.
* Para restablecer la falla:
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "fault_clear"
```

---

### Prueba 10: Reinicio Completo de Sesión (FSM Reset)
Para reiniciar todos los contadores acumulados (volumen infundido, tiempo transcurrido, progreso):
```bash
mosquitto_pub -h localhost -t "bj/bj-deadbeef/cmd" -m '{"cid":15,"cmd":"fsm_reset"}'
```
**Resultado esperado:**
* El canal ACK responderá: `{"cid":15,"result":"accepted"}`.
* Los contadores en el dashboard volverán a `0.00 mL`, `0 s` y `0.0 %`.
* La FSM regresa a `ST_READY_AT_HOME` (3).

---

### Prueba 11: Inyección y Verificación de Telemetría Real-time
Verifica en la página **Real-time** (`http://localhost:1880/dashboard/realtime`) los niveles y sensores físicos:
```bash
# 1. Simular batería baja en VIN_SENS (<12V inicia apagado):
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "sim_vin,11.5"

# 2. Simular batería 100% cargada (16.8V en 4S):
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "sim_vin,16.8"

# 3. Simular apagado por Switch ON/OFF principal (P0_2: SW_DETECT_H = 1):
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "sim_pcal,sw_detect_h,1"

# 4. Simular encendido por Switch ON/OFF principal (P0_2: SW_DETECT_H = 0):
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "sim_pcal,sw_detect_h,0"

# 5. Limpiar y volver a condiciones nominales (15.6V nominal):
mosquitto_pub -h localhost -t "bj/bj-deadbeef/sim_fault" -m "fault_clear"
```

---

## 6. Comandos Útiles de Mantenimiento en Linux

| Acción | Comando |
|--------|---------|
| Ver contenedores en ejecución | `docker compose ps` |
| Ver consumo de CPU y RAM de cada contenedor | `docker stats` |
| Reiniciar únicamente el simulador Python | `docker compose restart simulator` |
| Recompilar la imagen del simulador tras editar `pump_simulator.py` | `docker compose up -d --build simulator` |
| Ver los últimos 100 logs del broker MQTT | `docker compose logs --tail=100 -f mosquitto` |
| Detener el stack preservando datos de Node-RED | `docker compose down` |
| Detener y borrar todos los volúmenes (reseteo total) | `docker compose down -v` |

---

## 7. Solución de Problemas Frecuentes (Troubleshooting)

* **Error `bind: address already in use`:**
  Hay un servicio ocupando el puerto 1883 o 1880 en tu host Linux.
  * Solución: Detén los servicios conflictivos con `sudo systemctl stop mosquitto` o busca el proceso con `sudo lsof -i :1883` y mátalo con `kill -9 <PID>`.
* **Node-RED muestra el badge del dispositivo en rojo (`offline`):**
  * Verifica en los logs que el contenedor del simulador esté corriendo: `docker compose logs simulator`.
  * Asegúrate de que el nodo MQTT Broker en Node-RED tenga como dirección `mosquitto` en el puerto `1883` (nombre del servicio dentro de la red Docker, no `localhost`).
* **El simulador no toma cambios tras modificar `pump_simulator.py`:**
  * En `docker-compose.yml`, el archivo local está montado en caliente como volumen de solo lectura (`../nodered/pump_simulator.py:/app/pump_simulator.py:ro`). Solo basta reiniciar el contenedor:
    ```bash
    docker compose restart simulator
    ```
