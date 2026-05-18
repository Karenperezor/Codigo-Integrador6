/**
 * NODO EMISOR - PULSERA INTELIGENTE PARA SEGURIDAD DE MUJERES
 * 
 * Descripción:
 *   Dispositivo portátil que captura datos biométricos (BPM, SpO2),
 *   aceleración, geolocalización y timestamp. Detecta automáticamente
 *   emergencias (caídas, forcejeo) y transmite por LoRa en 915 MHz.
 * 
 * Hardware:
 *   - Heltec ESP32 LoRa v3 (SX1262)
 *   - MAX30102 (sensor biométrico)
 *   - MPU6050 (acelerómetro 6-ejes)
 *   - NEO-6M (GPS)
 *   - DS3231 (RTC)
 *   - Display OLED 128x64
 * 
 * Modo de operación:
 *   - REPOSO: Monitoreo pasivo, alertas visuales en emergencias detectadas
 *   - PÁNICO: Transmisión activa cada 2 segundos durante 1 hora
 * 
 * Autor: Equipo Shero
 * Fecha: Mayo 2026
 * Versión: 1.1
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

/* ================================================================
 *  CONFIGURACION DE HARDWARE E INSTANCIAS
 * ================================================================ */

// Display OLED SSD1306 (128x64 píxeles, I2C a 0x3C)
SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// Radio LoRa SX1262 en Heltec ESP32 v3
SX1262 radio = new Module(8, 14, 12, 13);

// Parámetros de transmisión LoRa
#define FREQUENCY     915.0      // Frecuencia: 915 MHz
#define BANDWIDTH     125.0      // Ancho de banda: 125 kHz
#define SPREAD_FACTOR 8          // Factor de propagación: SF8 (equilibrio alcance/velocidad)
#define CODING_RATE   5          // Velocidad de código: 4/5
#define TX_POWER      22         // Potencia de transmisión: 22 dBm

// Direcciones de red LoRa personalizadas
byte dirLocal   = 0xC1;          // Dirección de esta pulsera
byte dirDestino = 0xD3;          // Dirección del receptor (gateway)
byte idMsg      = 0;             // Contador incremental de mensajes
byte bufTx[256];                 // Buffer de transmisión LoRa

// Bus I2C compartido (Controlador 1 del ESP32)
TwoWire busI2C = TwoWire(1);

// Definición de pines para cada bus I2C independiente
#define MPU_SDA 41               // SDA del MPU6050
#define MPU_SCL 42               // SCL del MPU6050
#define RTC_SDA 38               // SDA del DS3231
#define RTC_SCL 39               // SCL del DS3231
#define MAX_SDA  1               // SDA del MAX30102
#define MAX_SCL  2               // SCL del MAX30102

// Instancias de sensores
MPU6050    sensor(0x68, &busI2C);  // Acelerómetro en dirección 0x68
RTC_DS3231 rtc;                    // Reloj de tiempo real
MAX30105   particleSensor;         // Sensor biométrico

// Estados de inicialización de sensores
bool mpuOK = false;                // ¿MPU6050 detectado correctamente?
bool rtcOK = false;                // ¿RTC detectado correctamente?
bool maxOK = false;                // ¿MAX30102 detectado correctamente?
#define FORZAR_AJUSTE false        // Forzar ajuste de hora (false = mantener hora guardada)

/* ================================================================
 *  VARIABLES DEL ACELERÓMETRO Y GIROSCOPIO (MPU6050)
 * ================================================================ */

float ax_g = 0, ay_g = 0, az_g = 0;  // Aceleración en ejes X, Y, Z (en g)
float gx_dps = 0, gy_dps = 0, gz_dps = 0;  // Velocidad angular en °/s
float mag = 1.0;                      // Magnitud de aceleración total
DateTime ultimaHora;                  // Última hora leída del RTC
const float LSB_POR_G    = 16384.0f;  // Conversión LSB a gravedad (escala ±2g)
const float LSB_POR_GYRO = 131.0f;    // Conversión LSB a °/s (escala ±250°/s)

/* ================================================================
 *  UMBRALES DE DETECCIÓN DE EMERGENCIAS
 * ================================================================ */

const float UMBRAL_G_FORCEJEO = 2.0f;   // Aceleración para forcejeo: 2.0g
const float UMBRAL_G_CAIDA    = 3.5f;   // Aceleración para caída: 3.5g
const int   UMBRAL_BPM_ALTO   = 110;    // BPM alto: 110 (taquicardia)
const int   UMBRAL_SPO2_BAJO  = 92;     // SpO2 bajo: 92% (hipoxemia)

/* ================================================================
 *  VARIABLES DEL SENSOR BIOMETRICO (MAX30102)
 * ================================================================ */

const byte RATE_SIZE = 4;        // Número de muestras para promediar BPM
byte  rates[RATE_SIZE];          // Buffer circular de BPM
byte  rateSpot = 0;              // Índice actual en buffer
long  lastBeat = 0;              // Timestamp del último latido detectado
float beatsPerMinute = 0;        // BPM actual (sin filtrar)
int   beatAvg = 0;               // BPM promediado (filtrado)
int   spo2    = 0;               // Saturación de oxígeno en sangre (%)

/* ================================================================
 *  VARIABLES DE GPS Y GEOLOCALIZACIÓN
 * ================================================================ */

HardwareSerial neogps(1);        // Serial 1 para módulo GPS NEO-6M
TinyGPSPlus    gps;              // Objeto para parsear datos NMEA
float lat = 0, lon = 0;          // Latitud y longitud actuales
bool  gpsValido   = false;       // ¿Hay ubicación válida?
bool  sincronizado = false;      // ¿RTC sincronizado con GPS?

/* ================================================================
 *  MÁQUINA DE ESTADOS Y CONTROL DE PÁNICO
 * ================================================================ */

enum Estado { REPOSO, PANICO };  // Estados del dispositivo
Estado estadoActual = REPOSO;    // Estado inicial: reposo
unsigned long tPanico      = 0;  // Timestamp de activación de pánico
unsigned long tUltimoEnvio = 0;  // Timestamp del último envío
int  numEvidencia  = 0;          // Contador de paquetes enviados
String tipoMovimiento = "REPOSO"; // Clasificación del evento

#define DURACION_PANICO 3600000UL  // Duración de pánico: 1 hora (ms)
#define INTERVALO_ENVIO 2000UL     // Intervalo entre envíos: 2 segundos (ms)

/* ================================================================
 *  VARIABLES DEL BOTÓN DE PÁNICO
 * ================================================================ */

#define BTN_PANICO 0             // GPIO 0 = botón de pánico
bool          botonPresionado = false;  // Estado anterior del botón
unsigned long tBoton          = 0;      // Timestamp de última pulsación

/* ================================================================
 *  VARIABLES DE ALERTAS VISUALES Y ANIMACIÓN
 * ================================================================ */

bool          enAlerta    = false;      // ¿Mostrando alerta visual?
unsigned long tiempoAlerta = 0;         // Timestamp de inicio de alerta
String        tipoAlerta  = "";         // Tipo de alerta detectada
float         magAlerta   = 0;          // Magnitud de aceleración en alerta
const unsigned long DUR_ALERTA = 3000;  // Duración de pantalla de alerta: 3 seg

// Variables para animación de parpadeo en pantalla
bool blinkState = false;                // Estado actual del parpadeo
unsigned long lastBlink = 0;            // Timestamp del último cambio de parpadeo

/* ================================================================
 *  ICONOS XBM PARA PANTALLA OLED
 * ================================================================ */

// Ícono de corazón (12x12 píxeles) - representa BPM
static const uint8_t HEART_BITS[] = {
  0x36, 0x00, 0x7F, 0x00, 0xFF, 0x00, 0xFF, 0x00,
  0x7E, 0x00, 0x3C, 0x00, 0x18, 0x00, 0x08, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// Ícono de gota (8x10 píxeles) - representa SpO2
static const uint8_t DROP_BITS[] = {
  0x0C, 0x1E, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1E,
  0x0C, 0x00, 0x00, 0x00,
};

// Ícono de reloj (10x10 píxeles) - representa hora
static const uint8_t CLOCK_BITS[] = {
  0x7C, 0x00, 0x82, 0x00, 0x9A, 0x00, 0xBA, 0x00,
  0x9A, 0x00, 0x82, 0x00, 0x82, 0x00, 0x7C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// Ícono de pin/mapa (8x12 píxeles) - representa ubicación GPS
static const uint8_t PIN_BITS[] = {
  0x3E, 0x7F, 0x7F, 0x7F, 0x7F, 0x3E, 0x1C, 0x08,
  0x08, 0x00, 0x00, 0x00,
};

// Ícono de onda (10x8 píxeles) - representa movimiento/acelerómetro
static const uint8_t WAVE_BITS[] = {
  0x00, 0x00, 0x22, 0x00, 0x55, 0x00, 0x88, 0x00,
  0x55, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* ================================================================
 *  GESTIÓN INTELIGENTE DE BUS I2C
 * ================================================================ */

// Variables de estado del bus I2C
int active_sda = -1;  // GPIO del SDA actualmente seleccionado
int active_scl = -1;  // GPIO del SCL actualmente seleccionado

/**
 * Cambia los pines del bus I2C a los especificados
 * Evita reinicializaciones innecesarias si ya está en el mismo bus
 */
void cambiarBusI2C(int nuevo_sda, int nuevo_scl) {
  // Optimización: no cambiar si ya estamos en el mismo bus
  if (active_sda == nuevo_sda && active_scl == nuevo_scl) return;
  
  busI2C.end();         // Finalizar bus I2C anterior
  delay(1);             // Esperar estabilización
  
  // Liberar pines anteriores
  if (active_sda != -1) {
    pinMode(active_sda, INPUT);
    pinMode(active_scl, INPUT);
  }
  
  delay(5);             // Esperar antes de reinicializar
  
  // Inicializar nuevo bus I2C
  busI2C.begin(nuevo_sda, nuevo_scl);
  busI2C.setClock(50000);  // Velocidad: 50 kHz (bajo para compatibilidad)
  busI2C.setTimeOut(50);   // Timeout: 50 ms
  
  active_sda = nuevo_sda;
  active_scl = nuevo_scl;
}

// Funciones de conveniencia para seleccionar cada sensor
void seleccionarMPU() { cambiarBusI2C(MPU_SDA, MPU_SCL); }
void seleccionarRTC() { cambiarBusI2C(RTC_SDA, RTC_SCL); }
void seleccionarMAX() { cambiarBusI2C(MAX_SDA, MAX_SCL); }

/* ================================================================
 *  FUNCIONES AUXILIARES
 * ================================================================ */

/**
 * Genera hash FNV-1a de una cadena
 * Usado para checksums simples de mensajes
 */
uint32_t hashFNV(const String &s) {
  uint32_t h = 0x811c9dc5UL;  // FNV offset basis
  for (size_t i = 0; i < s.length(); i++) { 
    h ^= (uint8_t)s[i]; 
    h *= 0x01000193UL;  // FNV prime
  }
  return h;
}

/**
 * Convierte hash FNV a cadena hexadecimal (8 caracteres)
 */
String hashHex(const String &s) {
  char buf[9];
  snprintf(buf, 9, "%08X", hashFNV(s));
  return String(buf);
}

/**
 * Calcula el tiempo restante de pánico en formato MM:SS
 * Usado para mostrar en pantalla tiempo de transmisión activa
 */
String tiempoRestante() {
  long ms = (long)(DURACION_PANICO - (millis() - tPanico));
  if (ms < 0) ms = 0;
  char buf[8];
  snprintf(buf, 8, "%02d:%02d", (int)(ms / 60000), (int)((ms % 60000) / 1000));
  return String(buf);
}

/**
 * Obtiene timestamp en formato ISO 8601 (YYYY-MM-DDTHH:MM:SS)
 * Prioridad: RTC > GPS > millis
 */
String obtenerTimestamp() {
  // Opción 1: Usar RTC (más confiable)
  if (rtcOK) {
    DateTime local = ultimaHora;
    char buf[20];
    snprintf(buf, 20, "%04d-%02d-%02dT%02d:%02d:%02d",
             local.year(), local.month(), local.day(),
             local.hour(), local.minute(), local.second());
    return String(buf);
  }
  
  // Opción 2: Usar GPS como respaldo
  if (gps.time.isValid() && gps.date.isValid()) {
    int hLocal = (gps.time.hour() - 6 + 24) % 24;  // Convertir UTC-6 (México)
    char buf[20];
    snprintf(buf, 20, "%04d-%02d-%02dT%02d:%02d:%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             hLocal, gps.time.minute(), gps.time.second());
    return String(buf);
  }
  
  // Opción 3: Usar millis como último recurso
  return String(millis());
}

/**
 * Clasifica el tipo de evento basado en sensores
 * Retorna etiqueta descriptiva para diagnóstico
 */
String clasificarEvento(float m, int bpm, int ox) {
  if (m > UMBRAL_G_CAIDA)                        return "CAIDA DETECTADA";
  if (m > UMBRAL_G_FORCEJEO)                     return "FORCEJEO";
  if (bpm > UMBRAL_BPM_ALTO && m < 1.5f)         return "ESTRES/MIEDO";
  if (ox < UMBRAL_SPO2_BAJO && ox > 0)           return "ALERTA SALUD";
  return "MOV BRUSCO";
}

/* ================================================================
 *  FUNCIONES DE RENDERIZADO OLED
 * ================================================================ */

/**
 * Dibuja ícono animado de campana (usado en pantalla de pánico)
 * El badajo se mueve si ring=true (simulando sonido)
 */
void drawBell(int cx, int cy, bool ring) {
  // Asa superior de la campana
  oled.drawLine(cx-3, cy-13, cx+3, cy-13);
  oled.drawLine(cx,   cy-16, cx,   cy-13);
  
  // Arco superior del cuerpo
  oled.drawCircle(cx, cy-4, 8);
  
  // Paredes laterales
  oled.drawLine(cx-8, cy-4, cx-8, cy+4);
  oled.drawLine(cx+8, cy-4, cx+8, cy+4);
  
  // Alero base (borde inferior ancho)
  oled.drawLine(cx-10, cy+5, cx+10, cy+5);
  oled.drawLine(cx-10, cy+4, cx-8,  cy+4);
  oled.drawLine(cx+10, cy+4, cx+8,  cy+4);
  
  // Badajo (martillo interior)
  int bx = ring ? cx+3 : cx;  // Se desplaza si está "sonando"
  oled.drawLine(cx, cy+5, bx, cy+9);
  oled.fillCircle(bx, cy+12, 3);
  
  // Ondas de sonido (solo si está sonando)
  if (ring) {
    oled.drawLine(cx-12, cy-9,  cx-15, cy-5);
    oled.drawLine(cx-15, cy-5,  cx-14, cy-1);
    oled.drawLine(cx-15, cy-12, cx-19, cy-7);
    oled.drawLine(cx-19, cy-7,  cx-18, cy-2);
    oled.drawLine(cx+12, cy-9,  cx+15, cy-5);
    oled.drawLine(cx+15, cy-5,  cx+14, cy-1);
    oled.drawLine(cx+15, cy-12, cx+19, cy-7);
    oled.drawLine(cx+19, cy-7,  cx+18, cy-2);
  }
}

/**
 * Pantalla principal en modo REPOSO
 * Muestra reloj digital grande, fecha, y datos de sensores en la barra inferior
 */
void pantallaUnificada() {
  oled.clear();

  // === FECHA (arriba, centrada) ===
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  if (rtcOK) {
    char buf[11];
    snprintf(buf, 11, "%04d/%02d/%02d", ultimaHora.year(), ultimaHora.month(), ultimaHora.day());
    oled.drawString(64, 0, String(buf));
  }

  oled.drawLine(0, 12, 128, 12);  // Separador

  // === HORA GRANDE (centro superior) ===
  oled.setFont(ArialMT_Plain_24);
  oled.setTextAlignment(TEXT_ALIGN_RIGHT);
  
  char hhStr[3], mmStr[3], ssStr[3];
  if (rtcOK) {
    snprintf(hhStr, 3, "%02d", ultimaHora.hour());
    snprintf(mmStr, 3, "%02d", ultimaHora.minute());
    snprintf(ssStr, 3, "%02d", ultimaHora.second());
  } else {
    snprintf(hhStr, 3, "--");
    snprintf(mmStr, 3, "--");
    snprintf(ssStr, 3, "--");
  }

  oled.drawString(52, 16, String(hhStr));  // Horas

  // Dos puntos con parpadeo (simulan latido)
  if (blinkState) {
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    oled.drawString(64, 14, ":");
  }

  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(68, 16, String(mmStr));  // Minutos

  // Segundos (pequeños)
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(104, 28, String(ssStr));

  oled.drawLine(0, 50, 128, 50);  // Separador

  // === BARRA INFERIOR: BPM | SpO2 | MOV ===
  
  // Sección 1: BPM
  oled.drawXbm(2, 53, 12, 8, HEART_BITS);
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(16, 53, String(beatAvg));

  oled.drawLine(42, 51, 42, 63);  // Separador vertical

  // Sección 2: SpO2
  oled.drawXbm(44, 54, 6, 8, DROP_BITS);
  oled.drawString(52, 53, String(spo2) + "%");

  oled.drawLine(84, 51, 84, 63);  // Separador vertical

  // Sección 3: Estado de movimiento
  oled.drawXbm(86, 53, 10, 8, WAVE_BITS);
  oled.drawString(98, 53, mag > 1.1 ? "MOV" : "OK");

  oled.display();  // Actualizar pantalla
}

/**
 * Pantalla de PÁNICO - Diseño de emergencia
 * Muestra campana animada a la izquierda, datos vitales y ubicación a la derecha
 */
void pantallaPanico() {
  oled.clear();

  // Marco doble (siempre visible, indica emergencia)
  oled.drawRect(0, 0, 128, 64);
  oled.drawRect(2, 2, 124, 60);

  // Campana animada (parpadea con blinkState)
  drawBell(26, 28, blinkState);

  // Texto PÁNICO (debajo de campana)
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  oled.drawString(26, 51, "PANICO");

  // Divisor vertical (separa campana de datos)
  oled.drawLine(52, 4, 52, 60);

  // === COLUMNA DERECHA: Datos vitales ===
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);

  // Encabezado "EMERGENCIA!!"
  oled.drawString(55, 2, "!EMERGENCIA!!");
  oled.drawLine(53, 13, 125, 13);

  // BPM
  oled.drawXbm(55, 15, 12, 12, HEART_BITS);
  oled.drawString(70, 15, String(beatAvg) + " BPM");

  // SpO2
  oled.drawXbm(56, 28, 8, 12, DROP_BITS);
  oled.drawString(70, 28, "SpO2:" + String(spo2) + "%");

  // Hora
  if (rtcOK) {
    char buf[6];
    snprintf(buf, 6, "%02d:%02d", ultimaHora.hour(), ultimaHora.minute());
    oled.drawXbm(55, 41, 10, 12, CLOCK_BITS);
    oled.drawString(70, 41, String(buf));
  }

  // Localización GPS
  oled.drawXbm(56, 51, 8, 12, PIN_BITS);
  oled.drawString(70, 52, gpsValido ? "Loc.enviada" : "Sin GPS");

  oled.display();  // Actualizar pantalla
}

/**
 * Función auxiliar de dibujo (no se usa actualmente)
 * Se mantiene por compatibilidad o futuras mejoras
 */
void dibujarEstado() {
  // Esta función está disponible para depuración pero no es llamada
  // en la máquina de estados actual (se usa pantallaUnificada directamente)
}

/**
 * Pantalla de alerta física breve (3 segundos)
 * Muestra el tipo de emergencia detectada y magnitud de aceleración
 */
void mostrarAlerta(const String &tipo, float m) {
  oled.clear();
  oled.drawRect(0, 0, 128, 64);
  oled.drawRect(2, 2, 124, 60);
  
  // Signo de exclamación grande
  oled.setFont(ArialMT_Plain_16);
  oled.drawString(50, 4, "!!");
  
  // Tipo de alerta (centrado)
  oled.setFont(ArialMT_Plain_10);
  int w = oled.getStringWidth(tipo);
  oled.drawString((128 - w) / 2, 26, tipo);
  
  // Magnitud de aceleración
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f g", m);
  oled.drawString((128 - oled.getStringWidth(buf)) / 2, 40, buf);
  
  oled.display();  // Actualizar pantalla
}

/* ================================================================
 *  MÁQUINA DE ESTADOS Y LÓGICA PRINCIPAL
 * ================================================================ */

/**
 * Lee el botón de pánico (GPIO 0)
 * Activa modo PÁNICO al presionar
 */
void leerBoton() {
  bool btn = (digitalRead(BTN_PANICO) == LOW);  // Botón activo en bajo
  
  // Detectar flanco descendente (presión)
  if (btn && !botonPresionado && millis() - tBoton > 400) {
    botonPresionado = true;
    tBoton = millis();
    
    // Evitar activar pánico si ya está activo
    if (estadoActual == PANICO) return;
    
    // Cambiar a estado PÁNICO
    estadoActual  = PANICO;
    tPanico       = millis();      // Marcar inicio
    tUltimoEnvio  = 0;             // Forzar envío inmediato
    numEvidencia  = 0;             // Reset contador
    tipoMovimiento = "PANICO MANUAL";
    Serial.println(F("PANICO ACTIVADO"));
  }
  
  if (!btn) botonPresionado = false;  // Resetear cuando se suelta
}

/**
 * Crea y transmite paquete JSON con datos de emergencia
 * Ejecutado cada INTERVALO_ENVIO durante estado PÁNICO
 */
void enviarEvidencia() {
  numEvidencia++;  // Incrementar contador de paquetes

  // Clasificar el tipo de evento (si no es pánico manual)
  if (tipoMovimiento != "PANICO MANUAL") {
    if      (mag > UMBRAL_G_CAIDA)       tipoMovimiento = "CAIDA";
    else if (mag > UMBRAL_G_FORCEJEO)    tipoMovimiento = "FORCEJEO";
    else if (beatAvg > UMBRAL_BPM_ALTO)  tipoMovimiento = "ESTRES";
    else                                 tipoMovimiento = "ALERTA";
  }

  String ts = obtenerTimestamp();  // Obtener timestamp actual

  // Construir paquete JSON con todos los datos
  String json = "{";
  json += "\"SOS\":1,";
  json += "\"Lat\":"   + String(lat, 6)    + ",";
  json += "\"Lon\":"   + String(lon, 6)    + ",";
  json += "\"BPM\":"   + String(beatAvg)   + ",";
  json += "\"SpO2\":"  + String(spo2)      + ",";
  json += "\"AccX\":"  + String(ax_g, 2)   + ",";
  json += "\"AccY\":"  + String(ay_g, 2)   + ",";
  json += "\"AccZ\":"  + String(az_g, 2)   + ",";
  json += "\"GyroX\":" + String(gx_dps, 1) + ",";
  json += "\"GyroY\":" + String(gy_dps, 1) + ",";
  json += "\"GyroZ\":" + String(gz_dps, 1) + ",";
  json += "\"Mag\":"   + String(mag, 2)    + ",";
  json += "\"Fecha\":\"" + ts.substring(0, 10) + "\",";
  json += "\"Hora\":\""  + ts.substring(11)    + "\"";
  json += "}";

  // Empaquetar para transmisión LoRa
  int i = 0;
  bufTx[i++] = dirDestino;           // Destino
  bufTx[i++] = dirLocal;             // Origen
  bufTx[i++] = idMsg++;              // ID del mensaje (autoincremento)
  bufTx[i++] = (byte)json.length();  // Longitud de payload
  for (size_t j = 0; j < json.length(); j++) bufTx[i++] = (byte)json[j];

  // Transmitir por LoRa
  int r = radio.transmit(bufTx, i);
  Serial.println(r == RADIOLIB_ERR_NONE
                 ? "TX OK: " + json
                 : "TX err: " + String(r));
}

/* ================================================================
 *  CONFIGURACION INICIAL (SETUP)
 * ================================================================ */

void setup() {
  // Inicializar comunicación serial (debug)
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 2000);  // Esperar puerto serial (máx 2s)

  // Configurar entrada del botón de pánico
  pinMode(BTN_PANICO, INPUT_PULLUP);

  // Encender pantalla OLED (controlar voltage externo)
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // Encender (activo en bajo)
  delay(300);

  // Inicializar display OLED
  oled.init();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Iniciando...");
  oled.display();
  delay(500);

  // --- Inicializar RTC DS3231 ---
  oled.drawString(0, 14, "DS3231...");
  oled.display();
  seleccionarRTC();
  rtcOK = rtc.begin(&busI2C);
  if (rtcOK) {
    if (rtc.lostPower() || FORZAR_AJUSTE)
      rtc.adjust(DateTime(2026, 3, 1, 23, 26, 0));
    ultimaHora = rtc.now();
  }

  // --- Inicializar MPU6050 ---
  oled.drawString(0, 28, "MPU6050...");
  oled.display();
  seleccionarMPU();
  sensor.initialize();
  mpuOK = sensor.testConnection();
  Serial.println(mpuOK ? "MPU6050: OK" : "MPU6050: ERROR");

  // --- Inicializar MAX30102 ---
  oled.drawString(0, 42, "MAX30102...");
  oled.display();
  seleccionarMAX();
  delay(300);
  if (particleSensor.begin(busI2C, I2C_SPEED_STANDARD)) {
    particleSensor.setup();                 // Configuración automática
    particleSensor.setPulseAmplitudeRed(0x1F);   // Potencia LED rojo
    particleSensor.setPulseAmplitudeGreen(0);    // LED verde desactivado
    maxOK = true;
  }

  // --- Inicializar GPS NEO-6M ---
  neogps.begin(9600, SERIAL_8N1, 45, 46);  // Serial 1 en pines 45/46

  // --- Inicializar LoRa SX1262 ---
  SPI.begin(9, 11, 10, 8);  // SPI en pines especificados
  delay(200);
  int st = radio.begin(FREQUENCY, BANDWIDTH, SPREAD_FACTOR, CODING_RATE);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setOutputPower(TX_POWER);  // Configurar potencia
    radio.setCRC(true);              // Habilitar CRC
  }

  // Mostrar resumen de inicialización en pantalla
  oled.clear();
  oled.drawString(0,  0, rtcOK ? "DS3231:   OK" : "DS3231:   ERROR");
  oled.drawString(0, 14, mpuOK ? "MPU6050:  OK" : "MPU6050:  ERROR");
  oled.drawString(0, 28, maxOK ? "MAX30102: OK" : "MAX30102: ERROR");
  oled.display();
  delay(3000);

  // Establecer bus I2C por defecto
  seleccionarMPU();
}

/* ================================================================
 *  CICLO PRINCIPAL (LOOP)
 * ================================================================ */

void loop() {
  // === ANIMACIÓN: Parpadeo global (para reloj y campana) ===
  if (millis() - lastBlink >= 400) {  // Parpadeo cada 400ms
    lastBlink = millis();
    blinkState = !blinkState;
  }

  // === LECTURA DE ENTRADAS ===
  leerBoton();  // Verificar presión de botón de pánico

  // === GPS: Lectura y sincronización ===
  while (neogps.available()) gps.encode(neogps.read());  // Parsear NMEA
  if (gps.location.isValid()) {
    gpsValido = true;
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  // Sincronizar RTC con GPS (una única vez al arranque)
  if (!sincronizado && rtcOK &&
      gps.location.isValid() && gps.time.isValid() && gps.date.isValid()) {
    seleccionarRTC();
    DateTime utcGPS(gps.date.year(), gps.date.month(), gps.date.day(),
                    gps.time.hour(), gps.time.minute(), gps.time.second());
    DateTime localGPS(utcGPS.unixtime() - 6UL * 3600UL);  // UTC-6 (México)
    rtc.adjust(localGPS);
    sincronizado = true;
    Serial.println("RTC sincronizado GPS");
  }

  // === MAX30102: Lectura de BPM y SpO2 ===
  if (maxOK) {
    seleccionarMAX();
    long ir  = particleSensor.getIR();    // Infrarrojo (referencia)
    long red = particleSensor.getRed();   // Rojo (oxígeno)

    if (ir < 50000) {  // Sensor no detecta dedo
      beatAvg = 0;
      spo2    = 0;
    } else {
      // Calcular BPM usando algoritmo de detección de picos
      if (checkForBeat(ir)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        beatsPerMinute = 60.0 / (delta / 1000.0);
        if (beatsPerMinute > 20 && beatsPerMinute < 255) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
          beatAvg /= RATE_SIZE;  // Promediado de últimas 4 lecturas
        }
      }
      
      // Calcular SpO2 usando ratio rojo/infrarrojo
      if (ir > 0 && red > 0) {
        float ratio = (float)red / (float)ir;
        spo2 = constrain((int)(110 - 25 * ratio), 70, 100);
      }
    }
  }

  // === MPU6050: Lectura de aceleración y giroscopio ===
  static unsigned long tAlterna = 0;
  if (millis() - tAlterna > 200) {  // Leer cada 200ms
    tAlterna = millis();

    seleccionarMPU();
    if (!sensor.testConnection()) {  // Verificar conexión
      sensor.initialize();
      mpuOK = sensor.testConnection();
    }

    if (mpuOK) {
      int16_t axr, ayr, azr, gxr, gyr, gzr;
      sensor.getMotion6(&axr, &ayr, &azr, &gxr, &gyr, &gzr);

      // Convertir LSB a unidades físicas
      ax_g = axr / LSB_POR_G;
      ay_g = ayr / LSB_POR_G;
      az_g = azr / LSB_POR_G;
      mag  = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);  // Magnitud

      gx_dps = gxr / LSB_POR_GYRO;
      gy_dps = gyr / LSB_POR_GYRO;
      gz_dps = gzr / LSB_POR_GYRO;

      // Detección automática de emergencias (solo en REPOSO)
      if (estadoActual == REPOSO) {
        if (mag > UMBRAL_G_FORCEJEO) {  // Forcejeo o caída detectados
          enAlerta    = true;
          tiempoAlerta = millis();
          tipoAlerta  = clasificarEvento(mag, beatAvg, spo2);
          magAlerta   = mag;
          Serial.printf("ALERTA FISICA: %s | %.2f g\n",
                        tipoAlerta.c_str(), mag);
        } else if (beatAvg > UMBRAL_BPM_ALTO ||(spo2 < UMBRAL_SPO2_BAJO && spo2 > 0 && beatAvg > 20)) {
          // Alerta biométrica (taquicardia o hipoxemia)
          enAlerta    = true;
          tiempoAlerta = millis();
          tipoAlerta  = clasificarEvento(mag, beatAvg, spo2);
          magAlerta   = mag;
          Serial.printf("ALERTA BIO: %s | BPM:%d SpO2:%d\n",
                        tipoAlerta.c_str(), beatAvg, spo2);
        }
      }
    }

    // Actualizar hora del RTC (alternadamente cada 400ms)
    static bool turnoRTC = false;
    if (turnoRTC && rtcOK) {
      seleccionarRTC();
      ultimaHora = rtc.now();
    }
    turnoRTC = !turnoRTC;
  }

  // === MÁQUINA DE ESTADOS ===
  switch (estadoActual) {

    case REPOSO:
      // Mostrar alerta visual si se detectó emergencia
      if (enAlerta) {
        mostrarAlerta(tipoAlerta, magAlerta);
        if (millis() - tiempoAlerta >= DUR_ALERTA) enAlerta = false;
      } else {
        pantallaUnificada();  // Mostrar reloj y datos normales
      }
      break;

    case PANICO:
      // Transmitir paquete cada INTERVALO_ENVIO
      if (millis() - tUltimoEnvio >= INTERVALO_ENVIO) {
        enviarEvidencia();
        tUltimoEnvio = millis();
      }
      pantallaPanico();  // Mostrar pantalla de emergencia
      
      // Salir de pánico si expiró el tiempo (1 hora)
      if (millis() - tPanico >= DURACION_PANICO) {
        estadoActual = REPOSO;
        numEvidencia = 0;
      }
      break;
  }

  delay(10);  // Pequeña pausa para estabilizar (50ms de ciclo total)
}
