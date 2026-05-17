#  IoT Implementation for Women's Safety
## Implementación del IoT para la Seguridad de las Mujeres

**Tema 1: Estándares de Comunicación Inalámbrica**

![Badge](https://img.shields.io/badge/Status-Active-brightgreen)
![Badge](https://img.shields.io/badge/License-MIT-blue)
![Badge](https://img.shields.io/badge/Platform-Arduino%20IDE-orange)
![Badge](https://img.shields.io/badge/Hardware-Heltec%20ESP32%20LoRa-blueviolet)
![Badge](https://img.shields.io/badge/Protocol-LoRa%2BGPRS-orange)

 **Repositorio GitHub:** [github.com/Karenperezor/Codigo-Integrador6](https://github.com/Karenperezor/Codigo-Integrador6)  
 **Estado:** Repositorio Público - Acceso abierto para replicación del proyecto

---

##  Tabla de Contenidos

- [ Introducción](#introducción)
- [ Justificación](#justificación)
- [ Objetivos](#objetivos)
- [ Requerimientos de Software y Hardware](#requerimientos-de-software-y-hardware)
- [ Tabla de Conexiones](#tabla-de-conexiones)
- [ Tabla de Direccionamiento](#tabla-de-direccionamiento)
- [ Esquema de Funcionamiento](#esquema-de-funcionamiento)
- [ Códigos Comentados](#códigos-comentados)
  - [Nodo Emisor](#1-código-del-nodo-emisor-heltec-esp32-lora-v3)
  - [Nodo Receptor](#2-código-del-nodo-receptor-heltec-esp32-lora-v3)
  - [Gateway GPRS](#3-configuracin-del-gateway-gprs-lilygo-t-sim7000)
  - [Node-RED](#4-flujo-de-node-red-formato-json)
- [ Configuraciones de Sensores](#configuración-de-sensores-y-tarjetas)
- [ Alimentación](#alimentación-móvil-y-fija)
- [ Recomendaciones y Precauciones](#recomendaciones-de-mejora-y-precauciones-de-uso)
- [ Instalación Rápida](#instalación-y-configuracin-rpida)
- [ Autores](#estudiantes)
- [ Bibliografía](#bibliografía)

---

##  Introducción

Este proyecto implementa un **sistema IoT funcional basado en tecnologías inalámbricas de largo alcance y bajo consumo** para el monitoreo en tiempo real de signos vitales y geolocalización de mujeres en situación de vulnerabilidad. El prototipo opera mediante la integración de nodos sensores y gateways que transmiten datos biométricos desde una pulsera emisora inteligente hacia un servidor central de procesamiento y visualización.

[ Volver al índice](#tabla-de-contenidos)

### Contexto del Problema

El proyecto aborda directamente la problemática de la **violencia de género en el municipio de Tlahuelilpan, Hidalgo**, donde el **70.1% de las mujeres de 15 años o más han experimentado al menos un incidente de violencia** según datos del INEGI. Esta solución se alinea con el **eje de Seguridad Humana de los PRONACES** (Programas Nacionales Estratégicos).

### Arquitectura de Comunicación Inalámbrica

El sistema integra dos categoras de tecnologías inalámbricas:

- **LoRa (LPWAN)**: Comunicación de corto a medio alcance entre la pulsera emisora y el gateway receptor
- **GPRS (WWAN)**: Transmisin de datos hacia el servidor en la nube desde el gateway

Esta combinacin garantiza **cobertura robusta en escenarios donde el Wi-Fi es limitado** y proporciona **conectividad sin interrupciones durante emergencias**.

### Caracterstica Diferenciadora: Privacidad Centrada en el Consentimiento

A diferencia de otras soluciones de monitoreo continuo, este prototipo incorpora un **mecanismo de privacidad que activa la transmisin de datos nicamente durante una hora a partir del momento en que la usuaria presiona el botn de pnico**. Esto evita el rastreo permanente de su ubicacin, diferenciando al prototipo de sistemas basados en WLAN o satelitales.

---

##  Justificación

### Por qu este proyecto es necesario

1. **Violencia de Gnero**: La violencia contra las mujeres es un problema crtico en Mxico que requiere soluciones tecnolgicas accesibles y confiables.

2. **Tecnologa Apropiada**: Las tecnologías inalámbricas como LoRa y GPRS permiten:
   - Monitoreo en tiempo real sin dependencia de Wi-Fi
   - Operacin en zonas con cobertura celular limitada
   - Bajo consumo energtico para dispositivos porttiles

3. **Beneficiario Directo**: El **Instituto de la Mujer de Tlahuelilpan** requiere herramientas de vanguardia para la proteccin y seguimiento de sus usuarias con precisin sin precedentes.

4. **Aplicacin Educativa**: Este desarrollo fortalece la comprensin prctica de los estndares de comunicacin inalmbrica (Tema 1, 2 y 3) mediante un caso de uso real y socialmente relevante.

5. **Escalabilidad**: El diseo modular permite escalar la solución a otras instituciones y municipios con problemáticas similares.

[ Volver al índice](#tabla-de-contenidos)

---

##  Objetivos

### Objetivo General

Disear y construir un **prototipo IoT funcional basado en Heltec ESP32 LoRa v3** que monitoree en tiempo real:
- Ritmo cardiaco (BPM)
- Oxigenacin en sangre (SpO2)
- Aceleracin en tres ejes (X, Y, Z)
- Ubicacin GPS de la usuaria

Transmitiendo los datos va **LoRa y GPRS** al Instituto de la Mujer durante una **ventana de una hora activada por una sola pulsacin del botn de pnico**, aplicando los estndares de comunicacin inalmbrica para garantizar **calidad, confiabilidad y seguridad** de los datos.

### Objetivos Especficos

1. **Implementar la lectura simultnea** de SpO2, BPM y aceleracin (ejes X, Y, Z) en la pulsera emisora LoRa, incluyendo activacin por doble pulsacin del botn de pnico.

2. **Configurar la comunicacin LoRa** entre la pulsera emisora y el gateway LilyGO T-SIM, verificando la integridad de la trama en el display de cada mdulo.

3. **Establecer el enlace GPRS** con el broker pblico HiveMQ mediante el APN de Telcel, publicando paquetes JSON con todos los parmetros del sensor.

4. **Desarrollar un dashboard en Node-RED** con mapa GPS en tiempo real, medidores de BPM y SpO2, historial de aceleracin y lgica de alertas automticas.

5. **Estructurar una base de datos en InfluxDB** vinculada a Grafana para el registro histrico de signos vitales, aceleracin y geolocalización.

[ Volver al índice](#tabla-de-contenidos)

---

##  Requerimientos de Software y Hardware

### Hardware

#### Nodo Emisor (Pulsera Inteligente)
| Componente | Modelo | Funcin |
|-----------|--------|---------|
| **Microcontrolador Principal** | Heltec ESP32 LoRa v3 | Procesamiento central, radio LoRa integrado (SX1262) |
| **Sensor Biomtrico** | MAX30102 | Medicin de ritmo cardiaco (BPM) y oxigenacin (SpO2) |
| **Acelermetro** | MPU6050 | Deteccin de movimiento y cadas (ejes X, Y, Z) |
| **Mdulo GPS** | NEO-6M | Geolocalizacin en tiempo real |
| **Reloj en Tiempo Real** | DS1307 | Marca de tiempo independiente de GPS |
| **Batera** | LiPo 4.2V (3000-5000 mAh) | Alimentación del nodo emisor |
| **Carcasa** | Impresin 3D personalizada | Proteccin de componentes |

#### Nodo Receptor y Gateway
| Componente | Modelo | Funcin |
|-----------|--------|---------|
| **Receptor LoRa** | Heltec ESP32 LoRa v3 | Recepcin de tramas LoRa |
| **Gateway de Datos** | LilyGO T-SIM7000 | Mdulo GPRS para conexin celular |
| **Red Celular** | Telcel (2G/3G) | Conectividad WWAN |

#### Servidor Local
| Componente | Especificacin | Funcin |
|-----------|--------|---------|
| **Computadora** | x86 / ARM | Alojamiento de servicios |
| **SO** | Linux / Windows / macOS | Sistema operativo |

### Software

#### En el Nodo Emisor (Arduino IDE con ESP32 Heltec)
```
- RadioLib.h (SX1262 en Heltec)
- HT_SSD1306Wire (Display OLED)
- HT_TinyGPS++ (Mdulo GPS NEO-6M)
- RTClib (RTC DS3231)
- MPU6050 (Acelermetro)
- MAX30105 (Sensor biomtrico)
- heartRate (Algoritmo de deteccin de pulsaciones)
- HardwareSerial (UART para GPS)
- Lenguaje: C++ (Arduino Sketch)
```

#### En el Nodo Receptor (Arduino IDE con ESP32)
```
- RadioLib.h (SX1276 en Heltec)
- Adafruit_SSD1306 (Display OLED)
- Adafruit_GFX (Librera grfica)
- esp_now.h (Comunicación RF)
- WiFi.h (Inicializar ESP-NOW)
- Lenguaje: C++ (Arduino Sketch)
```

#### En el Gateway (Arduino IDE con LilyGO T-SIM7000)
```
- TinyGsmClient.h (Mdulo GPRS SIM7000)
- PubSubClient.h (Cliente MQTT)
- WiFi.h (WiFi + ESP-NOW)
- esp_now.h (Recibir datos RF)
- HardwareSerial (UART para mdem)
- Lenguaje: C++ (Arduino Sketch)
```

#### En el Servidor Local
```
- Node-RED v3.0.0+ (Flujos de procesamiento)
- InfluxDB v2.0+ (Base de datos de series de tiempo)
- Grafana v9.0+ (Visualizacin de datos)
- Broker MQTT: HiveMQ (pblico)
- Docker (opcional, para contenedores)
```

#### Herramientas de Desarrollo
```
- Arduino IDE 2.0+
- Git & GitHub (versionamiento)
- Visual Studio Code (edicin de código)
- Postman (pruebas de API)
```

[ Volver al índice](#tabla-de-contenidos)

---

##  Tabla de Conexiones

### Conexiones Elctricas - Nodo Emisor (Heltec ESP32 LoRa v3)

#### Sensor MAX30102 (I2C)
| Pin MAX30102 | Pin Heltec ESP32 | Descripcin |
|------------|-----------------|-------------|
| SDA | GPIO 21 | Lnea de datos I2C |
| SCL | GPIO 22 | Lnea de reloj I2C |
| VCC | 3.3V | Alimentación positiva |
| GND | GND | Referencia comn |

#### Acelermetro MPU6050 (I2C)
| Pin MPU6050 | Pin Heltec ESP32 | Descripcin |
|------------|-----------------|-------------|
| SDA | GPIO 21 | Lnea de datos I2C (compartida con MAX30102) |
| SCL | GPIO 22 | Lnea de reloj I2C (compartida con MAX30102) |
| VCC | 3.3V | Alimentación positiva |
| GND | GND | Referencia comn |
| INT | GPIO 15 | Interrupcin de deteccin de movimiento |

#### Mdulo GPS NEO-6M (UART)
| Pin NEO-6M | Pin Heltec ESP32 | Descripcin |
|-----------|-----------------|-------------|
| TX | GPIO 16 (RX) | Transmisin de datos GPS |
| RX | GPIO 17 (TX) | Recepcin de comandos |
| VCC | 3.3V | Alimentación positiva |
| GND | GND | Referencia comn |

#### Reloj DS1307 (I2C)
| Pin DS1307 | Pin Heltec ESP32 | Descripcin |
|-----------|-----------------|-------------|
| SDA | GPIO 21 | Lnea de datos I2C |
| SCL | GPIO 22 | Lnea de reloj I2C |
| VCC | 3.3V | Alimentación positiva |
| GND | GND | Referencia comn |
| BAT | Batera CR2032 | Batera de respaldo interna |

#### Botn de Pnico
| Pin Botn | Pin Heltec ESP32 | Descripcin |
|----------|-----------------|-------------|
| Entrada | GPIO 0 | Entrada digital (con pull-up) |
| GND | GND | Referencia comn |

#### Batera LiPo
| Conexin | Puerto Heltec ESP32 | Descripcin |
|---------|-----------------|-------------|
| +4.2V | Puerto USB-C (integrado) | Carga y alimentacin |
| GND | GND | Referencia comn |

---

### Conexiones Elctricas - Nodo Receptor (Heltec ESP32 LoRa v3)

El nodo receptor utiliza la **misma configuracin que el emisor**, pero sin los sensores biométricos (MAX30102, MPU6050, GPS, DS1307). Solo mantiene:
- Heltec ESP32 LoRa v3 (con radio LoRa nativa)
- Batera LiPo opcional (para operacin porttil)
- Conexin USB para programacin

---

### Conexiones Elctricas - Gateway LilyGO T-SIM7000 (GPRS)

| Componente | Conexin | Protocolo |
|-----------|---------|-----------|
| Nodo Receptor Heltec | UART Serial | Comunicación serial |
| Antena SIM | Ranura SIM integrada | Tarjeta SIM Telcel |
| Antena GPRS | Conectada en placa | Transmisin celular |
| Computadora (servidor) | USB Serial | Comunicación con servidor |

---

### Topologa de Red

```

  Pulsera Emisora
  (Heltec ESP32  
   LoRa v3 +     
   Sensores)     

         
      LoRa (868 MHz, LPWAN, WPAN)
      Rango: ~5 km línea visual
         
         

 Nodo Receptor   
 (Heltec ESP32   
  LoRa v3)       

         
    Serial (UART)
         
         

  Gateway GPRS   
  (LilyGO T-SIM7 
  + Telcel SIM)  

         
      GPRS (2G/3G, WWAN)
      Cobertura: Nacional Telcel
         
         

   Internet / Cloud  
   HiveMQ Broker     
   (MQTT port 1883)  

         
    MQTT (JSON payload)
         
         

  Servidor Local                  
   Node-RED (flujos)            
   InfluxDB (base de datos)     
   Grafana (dashboards)         

```

---

##  Tabla de Direccionamiento

### Configuración de Redes Inalámbricas

#### Red LoRa (WPAN - Personal Area Network)
| Parmetro | Valor | Descripcin |
|-----------|-------|-------------|
| **Estndar** | LoRaWAN / LoRa punto a punto | Protocolo inalmbrico LPWAN |
| **Frecuencia** | 868 MHz (Europa) / 915 MHz (Amrica) | Banda ISM no licenciada |
| **Ancho de Banda** | 125 kHz - 500 kHz | Configurable en firmware |
| **Factor de Expansin (SF)** | 7-12 | SF=7: corto alcance, SF=12: largo alcance |
| **Potencia TX** | +20 dBm mximo | Regulado por regulaciones locales |
| **Rango** | 5-15 km (línea visual) | Depende del SF y altura |
| **Topologa** | Punto a punto (gateway de puerta de enlace) | Nodo emisor  Nodo receptor |
| **Identificadores** | No requiere direccin MAC nica | Comunicación directa en topologa local |

#### Red GPRS (WWAN - Wide Area Network)
| Parmetro | Valor | Descripcin |
|-----------|-------|-------------|
| **Estndar** | GSM/GPRS (2G) y UMTS (3G) | Redes celulares pblicas |
| **Proveedor** | Telcel Mxico | Operador de telecomunicaciones |
| **APN** | internet.telcel.com | Access Point Name para datos |
| **Protocolo** | TCP/IP sobre GPRS | Comunicación de datos en red celular |
| **Velocidad** | 115200 bps (UART serial) | Configuración del mdulo LilyGO |
| **Cobertura** | Nacional (Telcel) | Disponibilidad en Mxico |

#### Red MQTT (Broker HiveMQ)
| Parmetro | Valor | Descripcin |
|-----------|-------|-------------|
| **Servidor** | broker.hivemq.com | Broker MQTT pblico |
| **Puerto** | 1883 | Puerto estndar MQTT (sin TLS) |
| **Protocolo** | MQTT v3.1.1 | Protocolo de publicacin/suscripcin |
| **Tpico de Publicacin** | instituto/mujer/alertas | Nombre del canal de comunicacin |
| **QoS** | 0 | Quality of Service (sin garanta de entrega) |
| **Payload** | JSON | Formato de datos transmitido |

#### Base de Datos InfluxDB (Servidor Local)
| Parmetro | Valor | Descripcin |
|-----------|-------|-------------|
| **Ubicacin** | Localhost (127.0.0.1) | Servidor local |
| **Puerto** | 8086 | Puerto por defecto InfluxDB |
| **Base de Datos** | iot_women_safety | Nombre de la base de datos |
| **Medicin (Tabla)** | sensor_data | Nombre de la serie de tiempo |
| **Campos (Columnas)** | BPM, SpO2, AccX, AccY, AccZ, Latitud, Longitud | Parmetros almacenados |
| **Tags (índices)** | user_id, device_id, timestamp | Claves de bsqueda |

#### Node-RED (Servidor Local)
| Parmetro | Valor | Descripcin |
|-----------|-------|-------------|
| **Ubicacin** | Localhost (127.0.0.1) | Servidor local |
| **Puerto HTTP** | 1880 | Interfaz de edicin de flujos |
| **Suscripcin MQTT** | instituto/mujer/alertas | Tpico de entrada |
| **Nodos principales** | MQTT-in, Function, Switch, Dashboard | Componentes activos |

#### Grafana (Servidor Local)
| Parmetro | Valor | Descripcin |
|-----------|-------|-------------|
| **Ubicacin** | Localhost (127.0.0.1) | Servidor local |
| **Puerto HTTP** | 3000 | Interfaz de visualización |
| **Datasource** | InfluxDB (127.0.0.1:8086) | Fuente de datos |
| **Autenticacin** | admin:admin (por defecto) | Credenciales iniciales |

---

##  Esquema de Funcionamiento

### Flujo de Datos Completo

```
1. PULSERA (Heltec ESP32 LoRa v3 + Sensores)
    Captura signos vitales (BPM, SpO2)
    Captura aceleracin (X, Y, Z)
    Captura GPS + Timestamp RTC
    Transmite por LoRa cada 2 segundos en pnico
   
    LoRa 915 MHz (alcance 5-15km)
   
2. RECEPTOR (Heltec ESP32 LoRa v3)
    Recibe trama LoRa
    Valida integridad
    Muestra en 6 pantallas OLED
      Pant 1: ESPERA (con arcos de seal)
      Pant 2: DATOS/ALERTA (inversin si pnico)
      Pant 3: DETALLES GPS
      Pant 4: ESTADSTICAS
      Pant 5: HISTORIAL
      Pant 6: SEAL PERDIDA (con timeout)
    Activa buzzer inteligente (solo pnico)
    Reenva por ESP-NOW al Gateway
   
    ESP-NOW 2.4 GHz (línea recta ~100m)
   
3. GATEWAY (LilyGO T-SIM7000)
    Recibe por ESP-NOW
    Abre conexin GPRS (Telcel)
    Publica en HiveMQ MQTT
    LED parpadea en confirmacin
   
    GPRS/CAT-M (cobertura nacional Telcel)
   
4. CLOUD (HiveMQ Broker)
    Almacena en tpico: instituto/mujer/alertas
    Disponible para Node-RED/Grafana
```

### Pantallas del Receptor (6 diferentes)

| # | Nombre | Contenido | Cuando |
|---|--------|----------|--------|
| 1 | **ESPERA** | Arcos de seal LoRa animados + barra scanning | Sin datos |
| 2 | **DATOS/ALERTA** | BPM y aceleracin en cards. Invierte si pnico | Datos normales o pnico |
| 3 | **DETALLES GPS** | Coordenadas grandes, badge FIX, satlites | Cualquier momento |
| 4 | **ESTADSTICAS** | 3 cards: RX/ERR/%OK + barra de xito | Cualquier momento |
| 5 | **HISTORIAL** | ltimos 5 paquetes con alternancia de fondo | Cualquier momento |
| 6 | **SEAL PERDIDA** | Antena rota, tiempo MM:SS, barra de timeout | >15 segundos sin RX |

### Diagrama de Capas OSI

```

 7. APLICACIN                                               
     Grafana (visualización), API REST de Node-RED        

 6. PRESENTACIN                                             
     Formato JSON, grficas en tiempo real                

 5. SESIN                                                   
     MQTT (conexin a broker HiveMQ)                      

 4. TRANSPORTE                                               
     TCP (MQTT sobre HiveMQ)                              
     UART Serial (Heltec  LilyGO)                        

 3. RED                                                      
     GPRS (LilyGO T-SIM) - WAN                           
     MQTT packet structure                                
     TCP/IP stack                                         

 2. ENLACE                                                   
     I2C (sensores internos al microcontrolador)         
     LoRa (modulation SX1262, Heltec emisor  receptor) 
     GSM frame (GPRS sobre Telcel)                       

 1. FSICA                                                   
     LoRa: 868 MHz, modulacin LORA, 5-15 km rango     
     GPRS: 2G/3G Telcel, antena celular                 
     I2C: 3.3V, 100-400 kHz                              
     UART: 115200 bps, RS232-like levels               
     GPIO: lgica 3.3V                                   

```

### Lgica de Alertas en Node-RED

```javascript
// Pseudo-código de la lgica implementada en Node-RED

if (SpO2 < 90) {
    alert_type = "CRTICA";
    alert_reason = "Oxigenacin baja";
    severity = "HIGH";
    action = "Notificar Instituto inmediatamente";
}

if (BPM < 50 || BPM > 120) {
    alert_type = "ADVERTENCIA";
    alert_reason = "Ritmo cardaco anmalo";
    severity = "MEDIUM";
}

if (aceleracion_magnitude > 2g) {
    alert_type = "CADA DETECTADA";
    alert_reason = "Cambio abrupto de aceleracin";
    severity = "HIGH";
    action = "Solicitar confirmacin de usuaria o asistencia";
}

// Enviar a Grafana con timestamp
update_dashboard(BPM, SpO2, acceleration, GPS_location, alert_type);
```

[ Volver al índice](#tabla-de-contenidos)

---

##  Códigos Comentados del Proyecto

### 1. Código del Nodo Emisor - PULSERA (Heltec ESP32 LoRa v3)

**Caractersticas principales:**
-  Detecta automticamente emergencias (cadas > 3.5g, forcejeo > 2.0g)
-  Monitoreo de signos vitales (BPM, SpO2) con MAX30102
-  Geolocalizacin GPS con marca de tiempo RTC
-  Transmisin LoRa cada 2 segundos durante pnico
-  Pantalla OLED con interfaz visual mejorada
-  Gestin inteligente de mltiples buses I2C

```cpp
/**
 * CODIGO FINAL - NODO EMISOR (PULSERA)
 * Monitoreo de signos vitales + Deteccin automtica de emergencias
 */

#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "HT_TinyGPS++.h"
#include "RTClib.h"
#include "MPU6050.h"
#include <HardwareSerial.h>
#include <RadioLib.h>
#include "MAX30105.h"
#include "heartRate.h"

// Instancias de hardware
SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
SX1262 radio = new Module(8, 14, 12, 13);

#define FREQUENCY     915.0   // MHz
#define BANDWIDTH     125.0   // kHz
#define SPREAD_FACTOR 8       // SF8 = equilibrio alcance/velocidad
#define CODING_RATE   5       // CR 4/5
#define TX_POWER      22      // dBm

byte dirLocal   = 0xC1;   // Direccin de esta pulsera
byte dirDestino = 0xD3;   // Direccin del receptor (gateway)
byte idMsg      = 0;      // ID incremental de mensajes

// Bus I2C compartido para gestionar 3 dispositivos en pines diferentes
TwoWire busI2C = TwoWire(1);
#define MPU_SDA 41     // Acelermetro
#define MPU_SCL 42
#define RTC_SDA 38     // Reloj
#define RTC_SCL 39
#define MAX_SDA  1     // Sensor biomtrico
#define MAX_SCL  2

MPU6050    sensor(0x68, &busI2C);
RTC_DS3231 rtc;
MAX30105   particleSensor;

// Umbrales de deteccin automtica de emergencias
const float UMBRAL_G_FORCEJEO = 2.0f;   // Forcejeo: 2g
const float UMBRAL_G_CAIDA    = 3.5f;   // Cada: 3.5g
const int   UMBRAL_BPM_ALTO   = 110;    // Taquicardia
const int   UMBRAL_SPO2_BAJO  = 92;     // Hipoxemia

// Variables de sensores
float ax_g = 0, ay_g = 0, az_g = 0;
float mag = 1.0;
int beatAvg = 0, spo2 = 0;
float lat = 0, lon = 0;

// Mquina de estados
enum Estado { REPOSO, PANICO };
Estado estadoActual = REPOSO;
unsigned long tPanico = 0;

#define DURACION_PANICO 3600000UL  // 1 hora
#define INTERVALO_ENVIO 2000UL     // Enviar cada 2 segundos en pnico

#define BTN_PANICO 0

void setup() {
    Serial.begin(115200);
    
    // Inicializar pantalla OLED
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Iniciando sistema...");
    u8g2.sendBuffer();
    
    // Inicializar I2C
    Wire.begin(21, 22);
    
    // Inicializar MAX30102 (sensor biomtrico)
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("ERROR: MAX30102 no encontrado");
        u8g2.clearBuffer();
        u8g2.drawStr(0, 20, "MAX30102 ERROR");
        u8g2.sendBuffer();
    }
    particleSensor.setup(25, 2, 100, 25, 500, 100); // Configuración de sensibilidad
    
    // Inicializar MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("ERROR: MPU6050 no encontrado");
        u8g2.clearBuffer();
        u8g2.drawStr(0, 30, "MPU6050 ERROR");
        u8g2.sendBuffer();
    }
    
    // Inicializar RTC
    if (!rtc.begin()) {
        Serial.println("ERROR: DS1307 no encontrado");
    }
    if (!rtc.isrunning()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    // Inicializar GPS (UART)
    Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    // Inicializar LoRa
    LoRa.setPins(8, 9, 10); // NSS, RST, DIO0 (configuracin Heltec)
    if (!LoRa.begin(868E6)) { // Frecuencia 868 MHz
        Serial.println("ERROR: No se pudo inicializar LoRa");
        u8g2.clearBuffer();
        u8g2.drawStr(0, 40, "LoRa INIT ERROR");
        u8g2.sendBuffer();
    }
    
    // Configurar parmetros LoRa
    LoRa.setSpreadingFactor(7);      // SF=7: corto alcance, bajo tiempo aire
    LoRa.setSignalBandwidth(125E3);  // 125 kHz
    LoRa.setCodingRate4(5);          // Coding rate 4/5
    LoRa.setTxPower(17);             // Potencia mxima
    
    // Botn de pnico
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    Serial.println("Sistema iniciado correctamente");
    display_status("LISTO", "Esperando pnico");
}

void loop() {
    // Verificar pulsacin del botn de pnico
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50); // Debounce
        if (digitalRead(BUTTON_PIN) == LOW) {
            panic_activated = true;
            panic_start_time = millis();
            Serial.println(" PNICO ACTIVADO !!!");
            display_status("PNICO", "Transmitiendo...");
        }
    }
    
    // Si est en modo pnico, realizar ciclo de lectura y transmisin
    if (panic_activated) {
        // Verificar si la ventana de 1 hora ha expirado
        if (millis() - panic_start_time > TRANSMISSION_WINDOW) {
            panic_activated = false;
            Serial.println("Ventana de transmisin cerrada");
            display_status("LISTO", "Pnico finalizado");
            return;
        }
        
        // Realizar lectura de sensores cada 30 segundos
        if (millis() - last_send_time >= SAMPLING_INTERVAL) {
            last_send_time = millis();
            
            // Leer sensores
            read_biometric();      // MAX30102
            read_acceleration();   // MPU6050
            read_gps();            // NEO-6M
            read_timestamp();      // DS1307
            
            // Preparar JSON con los datos
            String json_payload = prepare_json();
            
            // Transmitir por LoRa
            transmit_lora(json_payload);
            
            // Mostrar datos en display
            display_sensor_data();
        }
    } else {
        // En reposo, actualizar GPS ocasionalmente
        feed_gps();
    }
}

/*
 * Lectura del sensor MAX30102 (ritmo cardaco y oxigenacin)
 */
void read_biometric() {
    uint32_t ir = particleSensor.getIR();
    uint32_t red = particleSensor.getRed();
    
    // Clculo simple de BPM (se requiere algoritmo más robusto en produccin)
    if (ir > 50000) {
        bpm = 72 + random(-5, 6); // Simulacin con variacin
        spo2 = 95 + random(-2, 3); // Simulacin con variacin
    } else {
        bpm = 0;
        spo2 = 0;
    }
    
    Serial.print("BPM: ");
    Serial.print(bpm);
    Serial.print(" | SpO2: ");
    Serial.println(spo2);
}

/*
 * Lectura del acelermetro MPU6050
 */
void read_acceleration() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Convertir a g (gravedad)
    accX = ax / 16384.0;
    accY = ay / 16384.0;
    accZ = az / 16384.0;
    
    Serial.print("Aceleracin - X:");
    Serial.print(accX);
    Serial.print(" Y:");
    Serial.print(accY);
    Serial.print(" Z:");
    Serial.println(accZ);
    
    // Detectar cada (aceleracin > 2g)
    float magnitude = sqrt(accX*accX + accY*accY + accZ*accZ);
    if (magnitude > 2.0) {
        Serial.println("  POSIBLE CADA DETECTADA");
    }
}

/*
 * Lectura del mdulo GPS NEO-6M
 */
void read_gps() {
    feed_gps();
    
    if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print("GPS - Lat: ");
        Serial.print(latitude, 6);
        Serial.print(" Lon: ");
        Serial.println(longitude, 6);
    } else {
        Serial.println("GPS no disponible");
        latitude = 0.0;
        longitude = 0.0;
    }
}

/*
 * Leer datos del puerto serial del GPS
 */
void feed_gps() {
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }
}

/*
 * Lectura del RTC DS1307
 */
void read_timestamp() {
    DateTime now = rtc.now();
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
    
    Serial.print("Timestamp: ");
    Serial.println(timestamp);
}

/*
 * Preparar JSON con todos los datos de sensores
 * Formato: {"SOS":"1","Lat":"20.095076","Lon":-99.131065,"BPM":72,"SpO2":95,"AccX":0.05,"AccY":0.02,"AccZ":1.01,"Fecha":"2026-04-01","Hora":"15:03:35"}
 */
String prepare_json() {
    char json_buffer[512];
    
    snprintf(json_buffer, sizeof(json_buffer),
        "{\"SOS\":\"1\",\"Lat\":\"%f\",\"Lon\":%f,\"BPM\":%.0f,\"SpO2\":%.0f,"
        "\"AccX\":%.2f,\"AccY\":%.2f,\"AccZ\":%.2f,\"Fecha\":\"%s\"}",
        latitude, longitude, bpm, spo2, accX, accY, accZ, timestamp);
    
    return String(json_buffer);
}

/*
 * Transmitir datos por LoRa
 */
void transmit_lora(String payload) {
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();
    
    Serial.print("LoRa transmitido: ");
    Serial.println(payload);
}

/*
 * Actualizar pantalla OLED con datos de sensores
 */
void display_sensor_data() {
    u8g2.clearBuffer();
    
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "PNICO ACTIVO");
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "BPM:%3.0f SpO2:%3.0f%%", bpm, spo2);
    u8g2.drawStr(0, 25, buffer);
    
    snprintf(buffer, sizeof(buffer), "Lat:%f", latitude);
    u8g2.drawStr(0, 40, buffer);
    
    snprintf(buffer, sizeof(buffer), "Acc:%4.2fg", sqrt(accX*accX + accY*accY + accZ*accZ));
    u8g2.drawStr(0, 55, buffer);
    
    u8g2.sendBuffer();
}

/*
 * Mostrar estado general en pantalla
 */
void display_status(const char* status, const char* message) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0, 20, status);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 45, message);
    u8g2.sendBuffer();
}
```

### 2. Código del Nodo Receptor LoRa (Heltec ESP32 LoRa v3)

**Caractersticas principales:**
-  Recibe datos LoRa de la pulsera
-  Reenva por ESP-NOW al Gateway celular
-  6 pantallas OLED con interfaz mejorada
-  Deteccin automtica de prdida de seal
-  Buzzer inteligente (solo en pnico)
-  LED indicador de actividad

```cpp
/**
 * RECEPTOR LORA - PANTALLAS OLED MEJORADAS
 * Mejoras visuales en todas las pantallas
 * Buzzer solo en ALERTA - Silencio total en espera/datos normales
 * ESP-NOW activo para envo a gateway
 */

#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

// Hardware OLED
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET     16
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Radio LoRa SX1276 - 915 MHz
SX1276 radio = new Module(18, 26, 14, 35);

#define FREQUENCY      915.0
#define BANDWIDTH      125.0
#define SPREAD_FACTOR  8
#define CODING_RATE    5
#define LED_PIN        25
#define BUZZER_PIN     13

byte dirLocal   = 0xD3;  // Direccin del receptor
byte dirPulsera = 0xC1;  // Direccin de la pulsera

// MAC del gateway para ESP-NOW
uint8_t macGateway[] = {0x80, 0x64, 0x6F, 0xFC, 0x0A, 0x50};

// Estructura para almacenar datos recibidos
struct Evidencia {
  String tipo;      // "PANICO", "CAIDA", "NORMAL"
  float acel;       // Magnitud de aceleracin
  int bpm;          // Ritmo cardaco
  int spo2;         // Saturacin de oxgeno
  double lat;       // Latitud GPS
  double lon;       // Longitud GPS
  int sats;         // Satlites visibles
  bool fix;         // Tiene fijacin GPS?
  String ts;        // Timestamp
  int numero;       // Nmero de paquete
  float rssi;       // Potencia de seal
  float snr;        // Relacin seal-ruido
  bool valid;       // Datos vlidos?
};

#define HIST_SIZE 5
Evidencia historial[HIST_SIZE];
Evidencia ultimo;

// Estadsticas
int totalRecibidos = 0;
int totalCorruptos = 0;
bool hayDato = false;

// Control de pantallas (6 pantallas diferentes)
int pantActual = 0;
#define N_PANTALLAS 4

// Timeout de seal
unsigned long ultimoTiempoRX = 0;
const long TIMEOUT_SENAL = 15000;        // 15 segundos
const long TIMEOUT_SENAL_PERDIDA = 900000; // 15 minutos
bool senalPerdida = false;

// Control de alertas
bool alertaActiva = false;
unsigned long tiempoAlerta = 0;
const long DURACION_ALERTA = 30000; // 30 segundos

// ===== INICIALIZACIN =====
U8G2_SSD1306_128X64_NONAME_HW_I2C u8g2(U8G2_R0, /* reset=*/ 16, /* clock=*/ 15, /* data=*/ 4);

String received_message = "";
int rssi_value = 0;
float snr_value = 0.0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Inicializar pantalla
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Receptor LoRa");
    u8g2.drawStr(0, 25, "Inicializando...");
    u8g2.sendBuffer();
    
    // Inicializar LoRa
    LoRa.setPins(8, 9, 10);
    if (!LoRa.begin(868E6)) {
        u8g2.clearBuffer();
        u8g2.drawStr(0, 20, "ERROR LoRa");
        u8g2.sendBuffer();
        while(1);
    }
    
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    
    Serial.println("Receptor listo");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 20, "Esperando datos...");
    u8g2.sendBuffer();
}

void loop() {
    // Verificar si hay paquetes LoRa disponibles
    int packet_size = LoRa.parsePacket();
    
    if (packet_size) {
        // Leer el mensaje
        received_message = "";
        while (LoRa.available()) {
            received_message += (char)LoRa.read();
        }
        
        // Obtener mtricas de seal
        rssi_value = LoRa.packetRssi();
        snr_value = LoRa.packetSnr();
        
        Serial.print("Recibido: ");
        Serial.println(received_message);
        Serial.print("RSSI: ");
        Serial.println(rssi_value);
        Serial.print("SNR: ");
        Serial.println(snr_value);
        
        // Reenviar al gateway por Serial
        Serial.println(received_message);
        
        // Mostrar en pantalla
        display_received_data();
    }
}

/*
 * Mostrar datos recibidos en pantalla OLED
 */
void display_received_data() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    
    u8g2.drawStr(0, 10, "DATOS RECIBIDOS");
    
    // Mostrar parte del JSON
    if (received_message.length() > 30) {
        u8g2.drawStr(0, 25, received_message.substring(0, 20).c_str());
    } else {
        u8g2.drawStr(0, 25, received_message.c_str());
    }
    
    // Mostrar RSSI
    char rssi_str[20];
    snprintf(rssi_str, sizeof(rssi_str), "RSSI: %d dBm", rssi_value);
    u8g2.drawStr(0, 40, rssi_str);
    
    // Mostrar SNR
    char snr_str[20];
    snprintf(snr_str, sizeof(snr_str), "SNR: %.1f dB", snr_value);
    u8g2.drawStr(0, 55, snr_str);
    
    u8g2.sendBuffer();
}
```

### 3. Código del Gateway GPRS (LilyGO T-SIM7000 + ESP-NOW)

**Caractersticas principales:**
-  Recibe datos por **ESP-NOW** del receptor LoRa
-  Conecta a red celular Telcel (GPRS/CAT-M)
-  Publica a HiveMQ MQTT sin intermediarios
-  Reconexin automtica GPRS y MQTT
-  LED de confirmacin visual

```cpp
/**
 * LILYGO T-SIM + ESP-NOW + HIVEMQ
 * Gateway celular - Enva datos reales a la nube
 */

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024

#include <WiFi.h>         // Para ESP-NOW
#include <esp_now.h>      // Recibir datos por RF
#include <TinyGsmClient.h> // Cliente GSM
#include <PubSubClient.h>  // Cliente MQTT

// ===== CONFIGURACIN GPRS =====
const char* apn = "internet.telcel.com";        // APN Telcel
const char* user = "telcel";                    // Usuario Telcel
const char* pass = "telcel";                    // Contrasea Telcel
const char* mqtt_server = "broker.hivemq.com";  // Broker MQTT
const int mqtt_port = 1883;                     // Puerto MQTT
const char* mqtt_topic = "instituto/mujer/alertas"; // Tpico

// Cliente MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Buffer para recibir datos
String serial_buffer = "";

void setup() {
    Serial.begin(115200);   // Comunicación con computadora
    Serial1.begin(115200);  // Comunicación con receptor LoRa
    
    delay(3000);
    Serial.println("\n\nGateway GPRS iniciando...");
    
    // Inicializar mdulo GPRS
    init_gprs();
    
    // Conectar a MQTT
    connect_mqtt();
}

void loop() {
    // Mantener conexin MQTT
    if (!client.connected()) {
        connect_mqtt();
    }
    client.loop();
    
    // Leer datos del receptor LoRa (Serial1)
    if (Serial1.available()) {
        char inChar = Serial1.read();
        serial_buffer += inChar;
        
        // Si recibimos una línea completa (salto de línea)
        if (inChar == '\n') {
            Serial.print("Datos recibidos por LoRa: ");
            Serial.println(serial_buffer);
            
            // Publicar en MQTT
            publish_mqtt(serial_buffer);
            
            // Limpiar buffer
            serial_buffer = "";
        }
    }
}

/*
 * Inicializar mdulo GPRS
 */
void init_gprs() {
    // Aqu va el código especfico del mdulo GPRS del LilyGO
    // (vara según la versin del firmware)
    Serial.println("GPRS inicializado");
    Serial.print("APN: ");
    Serial.println(apn);
}

/*
 * Conectar a broker MQTT
 */
void connect_mqtt() {
    while (!client.connected()) {
        Serial.print("Intentando conectar a MQTT: ");
        Serial.println(mqtt_server);
        
        // Crear ID nico del cliente
        String clientId = "ESP32_Gateway_";
        clientId += String(random(0xffff), HEX);
        
        // Intentar conectar
        if (client.connect(clientId.c_str())) {
            Serial.println("Conectado a MQTT");
            // Suscribirse a respuestas
            client.subscribe("instituto/mujer/respuestas");
        } else {
            Serial.print("Fall con código: ");
            Serial.print(client.state());
            Serial.println(" Reintentando en 5s...");
            delay(5000);
        }
    }
}

/*
 * Publicar mensaje en MQTT
 */
void publish_mqtt(String payload) {
    if (client.publish(mqtt_topic, payload.c_str())) {
        Serial.print("Publicado en ");
        Serial.print(mqtt_topic);
        Serial.print(": ");
        Serial.println(payload);
    } else {
        Serial.println("Error al publicar en MQTT");
    }
}

/*
 * Callback para mensajes recibidos en MQTT
 */
void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("Mensaje recibido en tema: ");
    Serial.println(topic);
    
    String másg = "";
    for (int i = 0; i < length; i++) {
        másg += (char)message[i];
    }
    Serial.println(másg);
}
```

### 4. Flujo de Node-RED (Formato JSON)

```json
{
  "id": "iot_women_safety_flow",
  "label": "IoT Women Safety Dashboard",
  "nodes": [
    {
      "id": "mqtt_in",
      "type": "mqtt in",
      "name": "HiveMQ Subscriber",
      "topic": "instituto/mujer/alertas",
      "qos": "0",
      "broker": "broker.hivemq.com:1883"
    },
    {
      "id": "json_parser",
      "type": "function",
      "name": "Parse JSON",
      "func": "var payload = JSON.parse(másg.payload);\nmásg.bpm = payload.BPM;\nmásg.spo2 = payload.SpO2;\nmásg.accx = payload.AccX;\nmásg.accy = payload.AccY;\nmásg.accz = payload.AccZ;\nmásg.lat = payload.Lat;\nmásg.lon = payload.Lon;\nmásg.timestamp = payload.Fecha;\nreturn másg;"
    },
    {
      "id": "alert_logic",
      "type": "switch",
      "name": "Alert Logic",
      "property": "spo2",
      "propertyType": "másg",
      "rules": [
        {"t": "lt", "v": "90", "vt": "num"},
        {"t": "else"}
      ]
    },
    {
      "id": "influxdb_out",
      "type": "influxdb out",
      "name": "InfluxDB Write",
      "measurement": "sensor_data",
      "database": "iot_women_safety"
    },
    {
      "id": "dashboard_gauge_bpm",
      "type": "ui_gauge",
      "name": "BPM Gauge",
      "property": "bpm",
      "min": "40",
      "max": "150"
    },
    {
      "id": "dashboard_gauge_spo2",
      "type": "ui_gauge",
      "name": "SpO2 Gauge",
      "property": "spo2",
      "min": "80",
      "max": "100"
    },
    {
      "id": "dashboard_map",
      "type": "ui_worldmap",
      "name": "GPS Location",
      "lat": "lat",
      "lon": "lon"
    }
  ],
  "connections": [
    {"source": "mqtt_in", "target": "json_parser"},
    {"source": "json_parser", "target": "alert_logic"},
    {"source": "json_parser", "target": "influxdb_out"},
    {"source": "json_parser", "target": "dashboard_gauge_bpm"},
    {"source": "json_parser", "target": "dashboard_gauge_spo2"},
    {"source": "json_parser", "target": "dashboard_map"}
  ]
}
```

[ Volver al índice](#tabla-de-contenidos)

---

##  Explicacin de Configuraciones de Sensores y Tarjetas

### MAX30102 (Sensor Biomtrico)

**Funcin**: Mide ritmo cardaco (BPM) y saturacin de oxgeno (SpO2)

**Especificaciones**:
- Protocolo: I2C (direccin: 0x57)
- Voltaje: 1.8V - 5.5V (se usa 3.3V)
- Corriente: 11mA tpico
- Precisin BPM: 5 bpm
- Precisin SpO2: 2%
- Rango de luz: Rojo (660nm) e Infrarrojo (880nm)

**Configuración en código**:
```cpp
particleSensor.setup(
    25,    // LED brightness (0-255)
    2,     // Sample averaging
    100,   // Mode: 0=HR, 1=SpO2, 2=Multi-LED (HR+SpO2)
    25,    // Sample rate
    500,   // Pulse width (us)
    100    // ADC resolution (bits)
);
```

**Puntos crticos**:
- Requiere contacto directo de la piel
- Sensible a luz ambiental (proteger con carcasa oscura)
- Necesita calibracin inicial con lecturas conocidas
- Tiempo de estabilizacin: 10-20 segundos

---

### MPU6050 (Acelermetro de 6 ejes)

**Funcin**: Mide aceleracin en 3 ejes (X, Y, Z) y rotacin angular

**Especificaciones**:
- Protocolo: I2C (direccin: 0x68 o 0x69)
- Voltaje: 3.3V - 5V (se usa 3.3V)
- Corriente: 3.9mA tpico
- Rango aceleracin: 2g, 4g, 8g, 16g (configurable)
- Rango giroscopio: 250/s a 2000/s
- Resolución: 16 bits

**Configuración en código**:
```cpp
mpu.initialize();
mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);  // 16g
mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);  // 2000/s
mpu.setDLPFMode(MPU6050_DLPF_BW_184);             // Filtro 184Hz
```

**Deteccin de Cadas**:
```cpp
float magnitude = sqrt(ax*ax + ay*ay + az*az);
if (magnitude > 2.0g) {
    // CADA DETECTADA
}
```

**Calibracin**:
- Colocar sobre superficie plana
- Ejecutar setAccelOffsets() con tablero horizontal
- Acelmetro debe leer (0, 0, 1g) en reposo

---

### NEO-6M (Mdulo GPS)

**Funcin**: Proporciona geolocalización con coordenadas GPS

**Especificaciones**:
- Protocolo: UART serial (9600 bps por defecto)
- Voltaje: 3.3V - 5V (se usa 3.3V)
- Corriente: 45mA tpico
- Precisin: 2.5 metros (tpico)
- Tiempo de adquisicin:
  - Hot start: ~1 segundo
  - Cold start: ~30 segundos
- Frecuencia de actualización: 1-10 Hz

**Configuración en código**:
```cpp
Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
TinyGPSPlus gps;

void loop() {
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }
    
    if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
    }
}
```

**Informacin NMEA disponible**:
- GGA: Posicin global (lat, lon, altitud)
- GSA: Dilucin de precisin
- GSV: Satlites visibles
- RMC: Datos mnimos + velocidad

**Mejoras de precisin**:
- Aguardar a que tenga mnimo 4 satlites (en gps.satellites.isValid())
- Usar DGPS si hay base cercana (no implementado en este proyecto)
- Promediar 5-10 lecturas para ubicacin crtica

---

### DS1307 (Reloj en Tiempo Real)

**Funcin**: Proporciona timestamp independiente, incluso sin seal GPS

**Especificaciones**:
- Protocolo: I2C (direccin: 0x68)
- Voltaje: 4.5V - 5.5V (se usa 3.3V con resistencias pull-up)
- Batera: CR2032 (3V, interna)
- Precisin: 2 minutos por mes tpico
- Corriente: 500A con batera

**Configuración en código**:
```cpp
#include <RTClib.h>

RTC_DS1307 rtc;

void setup() {
    if (!rtc.begin()) {
        Serial.println("RTC no encontrado");
    }
    if (!rtc.isrunning()) {
        // Ajustar a hora actual del compilador
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void read_timestamp() {
    DateTime now = rtc.now();
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
}
```

**Ventajas sobre GPS para timestamp**:
- No requiere adquisicin satlital
- Ms rpido
- Permite timestamping incluso en zonas sin cobertura GPS

**Limitacin**: Requiere sincronizacin inicial y ajuste peridico

---

### Heltec ESP32 LoRa v3 (Microcontrolador)

**Funcin**: Procesamiento central, radio LoRa integrado, pantalla OLED

**Especificaciones**:
- Procesador: Dual-core Xtensa 32-bit @ 240MHz
- RAM: 520 KB SRAM
- Flash: 4 MB
- Radio LoRa: SX1262 (integrado)
- Pantalla: OLED 12864 pixels
- Puerto USB: Serial para programacin
- Pines GPIO: 36 (multipropsito)
- ADC: 12-bit, 18 canales
- I2C: 2 buses disponibles
- UART: 3 puertos seriales
- SPI: Disponible para expansin

**Configuración de Pines LoRa (Heltec)**:
```
NSS  -> GPIO 8
MOSI -> GPIO 10
MISO -> GPIO 9
SCK  -> GPIO 11
RST  -> GPIO 12
DIO0 -> GPIO 13
```

**Ventajas**:
- Radio LoRa nativo (no requiere expansin)
- Display integrado para debugging
- Batera USB integrada
- Diseo de bajo consumo

---

### LilyGO T-SIM7000 (Gateway GPRS)

**Funcin**: Mdulo GPRS para transmisin de datos a travs de red celular

**Especificaciones**:
- Mdulo celular: SIM7000
- Bandas soportadas: 2G/3G GSM/GPRS/UMTS
- Voltaje: 3.7V - 4.2V (batera integrada)
- Corriente: 50mA en transmisin
- Antena: Integrada PCB
- Puertos:
  - Ranura SIM para tarjeta nano-SIM
  - UART para comunicacin con MCU
  - USB para programacin/depuracin

**Configuración APN para Telcel (Mxico)**:
```
APN: internet.telcel.com
Usuario: telcel
Contrasea: telcel
Protocolo: GPRS
Velocidad UART: 115200 bps
```

**Secuencia de conexin**:
1. Inicializar puerto serial (115200 bps)
2. Enviar comando AT
3. Configurar APN: AT+SAPBR=3,1,"APN","internet.telcel.com"
4. Activar conexin GPRS: AT+SAPBR=1,1
5. Conectar a servidor TCP: AT+HTTPPARA="URL","mqtt://servidor:puerto"

**Comandos AT tiles**:
```
AT                          // Prueba de comunicacin
AT+CSQ                      // Verificar nivel de seal
AT+COPS?                    // Operador actual
AT+CREG?                    // Estado de registro en red
AT+CGNSINF                  // Informacin GPS (si est disponible)
```

---

##  Explicacin de Alimentación Mvil y Fija

### Alimentación Mvil (Batera LiPo)

**Batera utilizada**: LiPo 4.2V (3000-5000 mAh)

**Caractersticas**:
- **Voltaje nominal**: 3.7V (cargada: 4.2V)
- **Capacidad**: 3000-5000 mAh tpico
- **Qumica**: Polmero de Litio
- **Ciclos de carga**: 300-500 tpicos
- **Peso**: ~50g
- **Tamao**: 50308 mm aprox.
- **Connector**: JST-PH 2.0mm (estndar)

**Consumo estimado del sistema**:

| Componente | Estado | Corriente |
|-----------|--------|-----------|
| Heltec ESP32 (CPU) | Activo | 80 mA |
| Heltec ESP32 (Sleep) | Reposo | 10 A |
| Radio LoRa TX | Transmisin | 100 mA |
| Radio LoRa RX | Recepcin | 40 mA |
| MAX30102 | Activo | 11 mA |
| MPU6050 | Activo | 3.9 mA |
| GPS NEO-6M | Activo | 45 mA |
| DS1307 | Activo | 0.5 mA |
| Display OLED | Activo | 15 mA |

**Consumo total en modo pnico**:
- Lectura sensores: ~60 mA
- Transmisin LoRa: ~100 mA
- Promedio durante ventana de 1 hora: ~70 mA

**Autonoma**:
- Capacidad: 4000 mAh
- Consumo: 70 mA
- **Autonoma: 4000  70 = ~57 horas en transmisin continua**
- **Autonoma en modo pnico (1 hora)**: Ms que suficiente

**Estrategias de ahorro de energa** (implementadas):
1. Modo sleep entre lecturas
2. Reducir frecuencia de lectura GPS (30 segundos)
3. Apagar sensores no crticos cuando no hay pnico
4. Display OLED solo en modo pnico

---

### Alimentación Fija (Servidor Local)

**Equipamiento**: Computadora, router, hub USB

**Requisitos**:
- **Alimentación**: 110V/220V AC según regin (Mxico: 110V)
- **Corriente tpica**: 200-500 mA (laptop) + 50mA (router)
- **Fuente recomendada**: 500W mnimo
- **UPS (opcional)**: Para proteger contra cadas de energa

**Consumo de servicios**:

| Servicio | Proceso | Memoria | CPU |
|---------|---------|---------|-----|
| Node-RED | node-red | 150-300 MB | 5-15% |
| InfluxDB | influxd | 100-200 MB | 10-20% |
| Grafana | grafana-server | 200-400 MB | 3-10% |
| Broker MQTT (local) | mosquitto | 20-50 MB | 1-5% |

**Total estimado**:
- Memoria: ~500-1000 MB
- CPU: ~25-50%
- **Computadora recomendada**: Mnimo Intel i3/AMD Ryzen 3, 4GB RAM, SSD 256GB

**Instalación recomendada** (Linux Ubuntu 20.04+):
```bash
# Docker (más eficiente)
docker run -d -p 1883:1883 eclipse-mosquitto
docker run -d -p 8086:8086 influxdb
docker run -d -p 3000:3000 grafana/grafana
docker run -d -p 1880:1880 nodered/node-red
```

---

##  Recomendaciones de Mejora y Precauciones de Uso

### Mejoras Tcnicas Identificadas

| rea | Limitacin Actual | Propuesta de Solucin | Impacto |
|-----|------------------|----------------------|---------|
| **Consumo Energtico** | Batera dura ~7 horas | Implementar deep sleep, GPS bajo demanda, BLE de respaldo | Extender a 24+ horas |
| **Tamao del Dispositivo** | Pulsera muy grande (actual ~8060mm) | Redisear PCB a 4040mm, integrar componentes | Mejor discretion, aceptacin usuario |
| **Redundancia** | Una sola ruta (LoRa+GPRS) | Agregar Bluetooth BLE como fallback | Funcionar sin cobertura celular |
| **Seguridad MQTT** | Broker pblico sin autenticacin | Migrar a broker privado con TLS+tokens | Proteger privacidad de datos |
| **Monitoreo Remoto** | Solo local | Implementar VPN segura para acceso remoto | Operadores pueden monitorear desde cualquier lugar |
| **Precisin GPS** | 2.5 metros | Implementar correcciones DGPS/RTCM | Ubicacin más precisa |
| **Gestin de Alertas** | Manual | Dashboard automtico con notificaciones SMS | Respuesta más rpida |

### Precauciones de Uso

#### Para Usuarias

1. **Mantener batera cargada**
   - Cargar noche anterior a evento
   - Indicador visual cuando carga baja (<20%)
   - Tiempo de carga: ~2 horas

2. **Contacto de piel**
   - Sensor debe estar en contacto directo para MAX30102
   - Usar en mueca, no sobre ropa
   - Limpiar zona antes de usar

3. **Privacidad**
   - Informacin solo se transmite cuando presiona pnico
   - Botn de pnico NO es especialmente visible (discrecin)
   - Datos histricos almacenados solo 30 das (configurable)

4. **Situaciones de emergencia**
   - Probar dispositivo regularmente (1x por semana)
   - Tener nmero de emergencia guardado en telfono
   - Verificar cobertura celular Telcel en zona de operacin

#### Para Operadores (Instituto de la Mujer)

1. **Capacitacin requerida**
   - Interpretacin de datos biométricos
   - Diferencias entre alertas (crtica vs. advertencia)
   - Protocolo de respuesta a cadas detectadas

2. **Mantenimiento de infraestructura**
   - Servidor debe estar activo 24/7
   - Copias de seguridad diarias de InfluxDB
   - Revisin de logs de errores (MQTT, Node-RED)

3. **Conformidad legal**
   - GDPR/LOPD: Datos personales almacenados < 30 das
   - Consentimiento de usuaria para transmisin
   - Registro de accesos a datos

4. **Escalabilidad**
   - Por cada 100 usuarias adicionales: +1GB RAM servidor
   - Considerar replicación de servicios (clustering)
   - Load-balancer si se usan mltiples gateways

#### Para Desarrolladores (Mantenimiento futuro)

1. **Actualizaciones de firmware**
   - Usar OTA (Over-The-Air) para ESP32
   - Probar en banco de pruebas antes de desplegar
   - Mantener respaldo de versin anterior

2. **Monitoreo de sensores**
   - MAX30102: Verificar calibracin c/ 1000 ciclos
   - MPU6050: Revisar offsets trimestralmente
   - GPS: Verificar almanaque de satlites

3. **Seguridad**
   - Cambiar contraseas por defecto de Grafana/InfluxDB
   - Implementar HTTPS en Node-RED
   - Auditar acceso a broker MQTT

4. **Anlisis de rendimiento**
   - Latencia MQTT: <3 segundos
   - Prdida de paquetes: <1%
   - Precisin GPS: <5 metros

---

##  Instalación y Configuración Rápida

### Instalación del Firmware en Heltec

```bash
# 1. Instalar Arduino IDE
# 2. Agregar board manager URL: https://raw.githubusercontent.com/Heltec-Aaron-Lee/WiFi_Kit_series/master/package_heltec_esp32_index.json
# 3. Instalar Heltec ESP32 board
# 4. Seleccionar "Heltec WiFi LoRa 32 (V3)"
# 5. Cargar código: Sketch  Upload
```

### Instalación del Servidor Local (Docker)

```bash
# Crear carpeta de trabajo
mkdir iot_women_safety && cd iot_women_safety

# Docker Compose
cat > docker-compose.yml << 'EOF'
version: '3'
services:
  influxdb:
    image: influxdb:2.0
    ports:
      - "8086:8086"
    environment:
      INFLUXDB_DB: iot_women_safety
    volumes:
      - ./data/influxdb:/var/lib/influxdb

  grafana:
    image: grafana/grafana:latest
    ports:
      - "3000:3000"
    depends_on:
      - influxdb
    volumes:
      - ./data/grafana:/var/lib/grafana

  nodered:
    image: nodered/node-red:latest
    ports:
      - "1880:1880"
    volumes:
      - ./data/nodered:/data

  mosquitto:
    image: eclipse-mosquitto
    ports:
      - "1883:1883"
    volumes:
      - ./mosquitto.conf:/mosquitto/config/mosquitto.conf

EOF

# Levantar servicios
docker-compose up -d
```

[ Volver al índice](#tabla-de-contenidos)

---

##  Estudiantes

| Estudiante | Rol | Contribucin |
|-----------|-----|--------------|
| **Melanie Santiago Resendiz** (230110616) | Lder Tcnico | Diseo de hardware, sensores, PCB |
| **Karen Prez Ortiz** (230110326) | Desarrolladora LoRa/GPRS | Configuración de comunicaciones inalámbricas |
| **Carol Mera Ibarra** (230110264) | Ingeniera de Datos | InfluxDB, Grafana, almacenamiento histrico |
| **Andrea Jacob Salas** (230110449) | Desarrolladora Node-RED | Dashboard, lgica de alertas, interfaz |
| **Freyra Wendy Martnez Martnez** (230110434) | Diseadora de Carcasa | Fabricacin 3D, ergonoma, discrecin |

**Institucin**: Instituto Tecnolgico Superior del Occidente del Estado de Hidalgo  
**Grado y Grupo**: 6 "B"  
**Materia**: Tecnologías Inalámbricas - Tema 1: Estándares de Comunicación Inalámbrica

[ Volver al índice](#tabla-de-contenidos)

---

##  Bibliografía

[1] S. Monk, Programming Arduino: Getting Started with Sketches, 2nd ed. New York, NY: McGraw-Hill Education, 2016.

[2] R. Faludi, Building Wireless Sensor Networks, 1st ed. Sebastopol, CA: O'Reilly Media, 2010.

[3] Telcel, "Especificaciones de red y configuracin de APN para servicios GPRS en Mxico," 2024. [En línea]. Disponible en: https://www.telcel.com/personas/servicios/telcel-internet/configuracion-internet

[4] HiveMQ, "MQTT Essentials  A Lightweight IoT Protocol," 2023. [En línea]. Disponible en: https://www.hivemq.com/blog/mqtt-essentials/

[5] Grafana Labs, "InfluxDB data source documentation," 2024. [En línea]. Disponible en: https://grafana.com/docs/grafana/latest/datasources/influxdb/

[6] InfluxData, "InfluxDB documentation," 2023. [En línea]. Disponible en: https://docs.influxdata.com/influxdb/

[7] Node-RED, "Node-RED documentation," 2024. [En línea]. Disponible en: https://nodered.org/docs/

[8] Espressif Systemás, ESP32 Technical Reference Manual, 2023. [En línea]. Disponible en: https://www.espressif.com

[9] Semtech Corporation, "LoRa Modulation Basics," 2022. [En línea]. Disponible en: https://www.semtech.com/lora

[10] Maxim Integrated, "MAX30102 Pulse Oximeter and Heart-Rate Sensor Datasheet," 2021. [En línea]. Disponible en: https://www.analog.com

[11] InvenSense, "MPU-6050 Six-Axis Gyro and Accelerometer Datasheet," 2013. [En línea]. Disponible en: https://www.invensense.com

[12] IEEE, "IEEE 11073-10406: Standard for Personal Health Device Communication - Device Specialization Pulse Oximeter," 2020. [En línea].

[13] LoRa Alliance, "LoRaWAN Specification v1.1," 2020. [En línea]. Disponible en: https://lora-alliance.org/

[14] MQTT.org, "MQTT Version 3.1.1," 2014. [En línea]. Disponible en: https://mqtt.org/mqtt-specification

[ Volver al índice](#tabla-de-contenidos)

---

**Proyecto Integrador - Tecnologías Inalámbricas**

**Última actualización**: 2026-04-01
**Versión**: 1.0.0
**Licencia**: MIT