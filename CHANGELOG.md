# Changelog

Todos los cambios notables de este proyecto se documentan en este archivo.

El formato sigue las convenciones de [Keep a Changelog](https://keepachangelog.com/es-ES/1.0.0/),
y este proyecto usa [Versionamiento Semántico](https://semver.org/lang/es/) (MAJOR.MINOR.PATCH).

---

## [3.0] — 2026-07

### Cambiado
- **Se eliminó toda detección automática de eventos** (caída, forcejeo, signos vitales fuera de rango). El único disparador de la alerta de pánico es ahora el botón físico.
- Se removieron los umbrales `UMBRAL_G_FORCEJEO`, `UMBRAL_G_CAIDA`, `UMBRAL_BPM_ALTO`, `UMBRAL_SPO2_BAJO` y la clasificación automática de tipo de evento.
- Se removió la pantalla de alerta intermedia (`mostrarAlerta()`), ya que dependía de la detección automática.

### Se mantuvo
- MPU6050 y MAX30102 se siguen leyendo y enviando en cada evidencia (BPM, SpO2, aceleración, giro) como datos de contexto para quien reciba la alerta — ya no deciden nada por sí solos.

### Motivo
Pruebas de campo mostraron falsos positivos por movimiento normal (caminar, subir escaleras). Se priorizó un disparo 100% intencional sobre la inferencia automática para este caso de uso.

---

## [2.6] — 2026-07

### Agregado
- `delay(3000)` + `modem.init()` en el camino de despertar de deep sleep, antes del primer intento de conexión a red, en respuesta al diagnóstico de v2.5 (el módem no respondía a comandos AT inmediatamente después de despertar).

### Resultado de la prueba
No resolvió el problema de reconexión irregular — llevó al hallazgo de un `POWERON_RESET` (no un despertar normal de deep sleep) durante transmisión activa, lo que apunta a una posible causa de alimentación (brownout) en vez de timing de software. Ver [Estado del proyecto](README.md#estado-del-proyecto-y-limitaciones-conocidas).

---

## [2.5] — 2026-07

### Agregado
- Diagnóstico de red (`imprimirDiagnosticoRed`): reporta antes de cada intento de conexión el estado de `celularOK_rtc`, calidad de señal (`AT+CSQ`) y estado de registro (`AT+CREG?`), más tiempos medidos de cada etapa de conexión.

### Motivo
Se observó un patrón irregular de tiempos de reconexión tras deep sleep (algunos ciclos casi instantáneos, otros de varios minutos o con fallo). Este diagnóstico se agregó para poder correlacionar el patrón con una causa real en vez de especular.

---

## [2.4] — 2026-07

### Agregado
- `celularOK_rtc` (memoria RTC, persiste entre ciclos de deep sleep): recuerda si la última vez que la pulsera durmió, había conexión a red/MQTT activa.
- Camino de reconexión "rápido" (`conectarCelularYMqtt(true)`, timeout corto) usado al despertar cuando `celularOK_rtc` sugiere que probablemente seguimos registrados; si falla, cae automáticamente al proceso completo (timeout largo).
- Resincronización de hora (`AT+CCLK?`) en cada despertar, en vez de intentar persistir un offset (se determinó que no es viable: `millis()` se reinicia en cada deep sleep, así que un offset guardado no puede corregirse por el tiempo real dormido).

### Motivo
Tras el deep sleep, `celularOK`, `horaSincronizada`, `lat` y `lon` — variables normales, no persistidas en RTC — se reseteaban en cada despertar, obligando a un proceso de reconexión completo (hasta 60s) y perdiendo la hora sincronizada, cada vez.

---

## [2.3] — 2026-07

### Cambiado
- MAX30102 movido a un bus I2C separado (`SDA=18, SCL=19`, segunda instancia de hardware I2C del ESP32), independiente del bus compartido por OLED y MPU6050 (`SDA=21, SCL=22`).
- Se eliminó la función `clasificarEvento()` por redundante; la lógica de clasificación se integró directamente en el punto donde se dispara la alerta.

### Base
Migración de arquitectura Heltec ESP32 LoRa V3 → LilyGO T-SIM7000G, con deep sleep integrado (módem y GPS permanecen siempre encendidos por diseño, para respuesta rápida al pánico; el ahorro de batería viene de apagar MPU6050, MAX30102 y pantalla OLED en inactividad).

---

## [1.0] — 2026-03 a 2026-05 — Arquitectura LoRa (histórica)

Versión completa preservada en el tag [`v1.0-arquitectura-lora`](../../releases/tag/v1.0-arquitectura-lora).

### Características de esta versión
- Microcontrolador: Heltec ESP32 LoRa v3 (pulsera emisora) + nodo receptor + gateway LilyGO T-SIM7000 (GPRS) vía ESP-NOW.
- Transmisión LoRa 915 MHz de la pulsera al nodo receptor.
- Detección automática de caídas (>3.5g) y forcejeo (>2.0g).
- GPS NEO-6M y RTC DS1307 como módulos discretos (en vez de vía módem).
- Backend: Node-RED + InfluxDB + Grafana, con notificaciones automáticas por WhatsApp.

### Por qué se reemplazó
El alcance real de LoRa en entorno urbano (obstáculos, no línea de vista) resultó insuficiente para garantizar respuesta desde cualquier punto donde la usuaria pudiera encontrarse. Se migró a conexión celular directa (ver v2.3 en adelante) para priorizar cobertura sobre costo operativo.
