# Women's Safety IoT — Pulsera Shero

Sistema IoT de seguridad para mujeres, basado en una pulsera con botón de pánico que transmite ubicación y datos de contexto (signos vitales, movimiento) vía red celular directa a un broker MQTT.

**Versión actual:** v3.0 — arquitectura GPRS directa (ver [Evolución del proyecto](#evolución-del-proyecto) para la arquitectura anterior basada en LoRa)

**Institución:** Instituto Tecnológico Superior del Occidente del Estado de Hidalgo (ITSOEH)
**Colaboración:** Instancia de la Mujer y Seguridad Pública, municipio de Tlahuelilpan, Hidalgo
**Equipo:** Shero

---

## Tabla de contenidos

- [Descripción general](#descripción-general)
- [Evolución del proyecto](#evolución-del-proyecto)
- [Arquitectura actual](#arquitectura-actual)
- [Hardware](#hardware)
- [Conexiones](#conexiones)
- [Software y librerías](#software-y-librerías)
- [Cómo funciona](#cómo-funciona)
- [Estado del proyecto y limitaciones conocidas](#estado-del-proyecto-y-limitaciones-conocidas)
- [Instalación](#instalación)
- [Changelog](#changelog)
- [Licencia](#licencia)

---

## Descripción general

La pulsera Shero permite a una usuaria activar una alerta de emergencia con un solo botón físico. Al presionarlo, el dispositivo:

1. Obtiene su ubicación GPS.
2. Lee signos vitales (ritmo cardíaco, oxigenación) y movimiento como datos de contexto.
3. Envía esta información, cada 2 segundos durante una ventana de tiempo, a un servidor vía red celular y MQTT, para que el Instituto de la Mujer y/o Seguridad Pública puedan responder.

El disparo de la alerta es **exclusivamente manual** (botón físico) — el sistema no intenta detectar automáticamente caídas, forcejeo o signos vitales anómalos. Esta fue una decisión de diseño explícita (ver [Evolución del proyecto](#evolución-del-proyecto)).

## Evolución del proyecto

Este proyecto pasó por dos arquitecturas distintas durante su desarrollo. Documentamos ambas porque el cambio refleja decisiones de ingeniería basadas en pruebas de campo, no solo preferencia.

### v1 — Arquitectura LoRa + Node-RED (histórica)

La primera versión usaba un microcontrolador Heltec ESP32 LoRa v3 como pulsera emisora, transmitiendo por radio LoRa (915 MHz) a un nodo receptor, que reenviaba por ESP-NOW a un gateway LilyGO T-SIM7000 con conexión GPRS, publicando finalmente a un broker MQTT. Incluía además detección automática de caídas (aceleración > 3.5g), forcejeo (> 2.0g), y signos vitales fuera de rango.

Esta versión completa está preservada en el tag [`v1.0-arquitectura-lora`](../../releases/tag/v1.0-arquitectura-lora) de este mismo repositorio.

**Por qué se descartó:** el análisis de cobertura mostró que el alcance de LoRa punto a punto (aunque teóricamente de varios km en línea de vista) no es confiable para uso a nivel de calle en el municipio, donde obstáculos urbanos reducen el alcance real de forma significativa. Una pulsera de emergencia necesita responder desde prácticamente cualquier punto donde la usuaria se encuentre, no solo dentro del alcance de un gateway fijo.

### v2/v3 — Arquitectura GPRS directa (actual)

Se migró a un único microcontrolador (LilyGO T-SIM7000G) que se conecta directamente a la red celular y publica a MQTT sin nodos intermedios. Esto elimina la dependencia de un gateway local a cambio de un costo recurrente de datos móviles por dispositivo — un trade-off aceptado porque prioriza la cobertura real sobre el costo operativo.

También se eliminó la detección automática de eventos (ver [CHANGELOG](CHANGELOG.md) v3.0): las pruebas de campo mostraron que los falsos positivos por movimiento normal (caminar, subir escaleras) generaban ruido en el sistema, y se decidió que un disparo 100% intencional por parte de la usuaria es más confiable para este caso de uso que la inferencia automática.

## Arquitectura actual

```
Pulsera (LilyGO T-SIM7000G)
   │
   │  Red celular (GPRS/2G/3G, Telcel)
   ▼
Broker MQTT (HiveMQ)
   │
   │  Tópico: instituto/mujer/alertas
   ▼
Backend de procesamiento y almacenamiento
(en desarrollo: AWS Lambda + RDS PostgreSQL)
```

El módem y el GPS del SIM7000G permanecen siempre encendidos (incluso en deep sleep del ESP32) para poder responder al botón de pánico sin necesidad de re-registrarse en la red desde cero. El ahorro de batería en reposo viene de apagar los sensores (MPU6050, MAX30102) y la pantalla OLED durante la inactividad.

## Hardware

| Componente | Modelo | Función |
|---|---|---|
| Microcontrolador + módem celular + GPS | LilyGO T-SIM7000G (ESP32 WROVER-B) | Procesamiento, conectividad celular, geolocalización |
| Sensor biométrico | MAX30102 | Ritmo cardíaco (BPM) y oxigenación (SpO2) |
| Acelerómetro/giroscopio | MPU6050 | Movimiento (dato de contexto, no dispara alerta) |
| Pantalla | OLED SSD1306 128x64 | Estado del dispositivo y de la alerta |
| Botón de pánico | Pulsador físico | Único disparador de la alerta |

## Conexiones

### Bus I2C #1 (Wire) — OLED + MPU6050

| Señal | GPIO |
|---|---|
| SDA | 21 |
| SCL | 22 |

- OLED, dirección I2C `0x3C`
- MPU6050, dirección `0x69` (requiere AD0 conectado a 3.3V — si tu módulo tiene AD0 a GND, la dirección real sería `0x68`; verificar físicamente)
- MPU6050 VCC → 3.3V, GND → GND. Pines INT, XDA, XCL no se usan.

### Bus I2C #2 (separado) — MAX30102

| Señal | GPIO |
|---|---|
| SDA | 18 |
| SCL | 19 |

### Pines del módem/placa (confirmados)

| Señal | GPIO |
|---|---|
| PWRKEY | 4 |
| DTR | 25 |
| TX (ESP32 → módem) | 27 |
| RX (ESP32 ← módem) | 26 |
| LED indicador | 12 |

### Botón de pánico

| Señal | GPIO |
|---|---|
| Botón | 33 (INPUT_PULLUP, activa en LOW; también usado como wakeup de deep sleep) |

## Software y librerías

```
TinyGsmClient.h    — comunicación con el módem SIM7000
PubSubClient.h     — cliente MQTT
SSD1306Wire.h      — pantalla OLED (librería ThingPulse/Squix)
MPU6050.h          — acelerómetro/giroscopio
MAX30105.h         — sensor biométrico (compatible MAX30102)
heartRate.h        — algoritmo de detección de pulso
driver/rtc_io.h    — control de GPIO en dominio RTC (deep sleep)
```

Arduino IDE 2.0+, placa ESP32 (paquete estándar de Espressif, no el de Heltec — este proyecto ya no usa hardware Heltec).

## Cómo funciona

1. **Reposo:** la pulsera muestra hora y signos vitales en pantalla. Tras 30 segundos sin actividad, entra en deep sleep (apaga sensores y pantalla; módem y GPS quedan encendidos).
2. **Botón presionado:** despierta (o, si ya estaba despierta, activa directamente) el estado de pánico. Intenta conectar a red celular y MQTT, resincroniza la hora, y comienza a transmitir evidencia cada 2 segundos durante 1 hora.
3. **Evidencia enviada:** cada paquete incluye ubicación GPS, BPM, SpO2, aceleración y giro en los tres ejes, y timestamp — publicado por MQTT en el tópico `instituto/mujer/alertas`.

## Estado del proyecto y limitaciones conocidas

Documentado con transparencia porque forma parte de la metodología de pruebas del proyecto (ver también [CHANGELOG](CHANGELOG.md)):

- **Reconexión tras deep sleep:** se implementó un mecanismo de reconexión rápida (memoria RTC) más un diagnóstico de red (calidad de señal, estado de registro) para investigar tiempos de reconexión irregulares observados en pruebas de campo.
- **Sospecha de brownout:** se detectaron reinicios tipo `POWERON_RESET` (no deep sleep) durante transmisión activa, lo que sugiere que el regulador de la placa podría no sostener bien los picos de corriente del módem durante la transmisión celular. Pendiente: prueba con capacitor de refuerzo cerca de los pines de alimentación del módem.
- **PCB en miniaturización:** en evaluación el paso de breadboard a PCB SMD de 40×35mm.

## Instalación

1. Instalar Arduino IDE 2.0+ y el paquete de placas ESP32 (Espressif).
2. Instalar las librerías listadas en [Software y librerías](#software-y-librerías) desde el Gestor de Librerías.
3. Abrir `firmware/pulsera-emisor/pulsera_emisor.ino`.
4. Configurar credenciales de red celular y broker MQTT (ver notas de seguridad en el propio archivo).
5. Seleccionar la placa correspondiente al ESP32 WROVER-B / T-SIM7000G y cargar.

## Changelog

El historial detallado de cambios está en [CHANGELOG.md](CHANGELOG.md).

## Licencia

MIT — ver [LICENSE](LICENSE).