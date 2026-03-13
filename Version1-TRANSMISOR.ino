/**.
 * Integra los siguientes módulos de hardware:
 *   - OLED SSD1306: Pantalla de 128x64 píxeles para mostrar información al usuario.
 *   - SX1262 (RadioLib): Módulo LoRa para transmisión inalámbrica de datos de emergencia.
 *   - MPU6050: Acelerómetro/giroscopio para detectar movimientos bruscos, caídas o forcejeos.
 *   - DS3231 (RTC): Reloj de tiempo real para registro preciso de timestamps.
 *   - MAX30102: Sensor óptico de frecuencia cardíaca (BPM).
 *   - NEO GPS (UART): Módulo GPS para obtener posición geográfica del portador.
 *   - Botón de pánico: Activa manualmente el modo de emergencia.
 *
 * Flujo de operación:
 *   1. En modo REPOSO: muestra datos rotativos en pantalla (movimiento, GPS, hora).
 *      Si se detecta una aceleración superior al umbral, lanza una alerta visual temporal.
 *   2. En modo PÁNICO (botón o automático): transmite paquetes LoRa con JSON de evidencia
 *      cada 2 segundos durante hasta 1 hora, incluyendo: tipo de evento, aceleración,
 *      BPM, coordenadas GPS, timestamp y hash de integridad.
 */

#include <Wire.h>
#include "HT_SSD1306Wire.h"      // Driver OLED específico para Heltec
#include "HT_TinyGPS++.h"        // Parser NMEA para módulo GPS
#include "RTClib.h"               // Librería para el RTC DS3231
#include "MPU6050.h"              // Driver para el acelerómetro MPU6050
#include <HardwareSerial.h>       // UART por hardware para el GPS
#include <RadioLib.h>             // Abstracción del módulo LoRa SX1262
#include "MAX30105.h"             // Driver para el sensor MAX30102
#include "heartRate.h"            // Algoritmo de detección de pulso cardíaco

/* ================================================================
 *  INSTANCIAS Y PINES
 * ================================================================ */

// Pantalla OLED: dirección I2C 0x3C, velocidad 500kHz, pines internos del Heltec
SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// Módulo LoRa SX1262: CS=8, IRQ=14, RST=12, BUSY=13
SX1262 radio = new Module(8, 14, 12, 13);

// Parámetros de configuración de la radio LoRa
#define FREQUENCY    915.0   // Frecuencia en MHz (banda ISM para América)
#define BANDWIDTH    125.0   // Ancho de banda en kHz
#define SPREAD_FACTOR  8     // Factor de dispersión (mayor = más alcance, menor velocidad)
#define CODING_RATE    5     // Tasa de codificación 4/5
#define TX_POWER      22     // Potencia de transmisión en dBm (máximo legal)

// Direccionamiento del protocolo propio sobre LoRa
byte dirLocal   = 0xC1;  // Dirección de este nodo (el wearable)
byte dirDestino = 0xD3;  // Dirección del receptor (base/servidor)
byte idMsg      = 0;     // Contador de mensajes para identificación de paquetes
byte bufTx[256];         // Buffer de transmisión LoRa

// Bus I2C compartido entre MPU6050, DS3231 y MAX30102.
// Se usa TwoWire(1) para el bus secundario del ESP32 y se cambian los
// pines físicos dinámicamente según el sensor que se vaya a leer.
TwoWire busI2C = TwoWire(1);

// Pines SDA/SCL para cada sensor (conexión física independiente por dispositivo)
#define MPU_SDA 41
#define MPU_SCL 42
#define RTC_SDA 38
#define RTC_SCL 39
#define MAX_SDA  1
#define MAX_SCL  2

// Instancias de los sensores apuntando al bus I2C compartido
MPU6050  sensor(0x68, &busI2C); // MPU6050 en dirección 0x68
RTC_DS3231 rtc;                  // RTC DS3231
MAX30105 particleSensor;         // Sensor de pulso MAX30102

// Flags de estado de inicialización de cada sensor
bool mpuOK = false;
bool rtcOK = false;
bool maxOK = false;

// Si se pone en true, fuerza la escritura de hora al RTC en cada arranque
#define FORZAR_AJUSTE false

// Variables de aceleración lineal leídas del MPU6050 (en g)
float ax_g = 0, ay_g = 0, az_g = 0;
// Módulo del vector de aceleración (magnitud resultante)
float mag = 1.0;

// Último valor leído del RTC para mostrar en pantalla
DateTime ultimaHora;

// Constante de conversión: el MPU6050 en rango ±2g produce 16384 LSB por g
const float LSB_POR_G = 16384.0f;

// Umbral en g que activa la alerta de movimiento brusco en modo REPOSO
const float UMBRAL_G = 2.5f;

// Variables para el algoritmo de promedio de frecuencia cardíaca (BPM)
const byte RATE_SIZE = 4;    // Tamaño del buffer circular de muestras BPM
byte rates[RATE_SIZE];       // Buffer de últimas 4 mediciones BPM
byte rateSpot = 0;           // Índice actual del buffer circular
long lastBeat = 0;           // Timestamp del último latido detectado
float beatsPerMinute = 0;    // BPM calculado del último intervalo
int  beatAvg = 0;            // Promedio móvil de BPM (sobre RATE_SIZE muestras)

// GPS: usa UART1 del ESP32 (RX=45, TX=46) a 9600 baudios
HardwareSerial neogps(1);
TinyGPSPlus gps;       // Objeto parser de tramas NMEA
float lat = 0, lon = 0;   // Última posición válida
bool gpsValido = false;   // True si se ha obtenido un fix GPS
bool sincronizado = false; // True si el RTC ya fue sincronizado con el GPS

// Máquina de estados principal
enum Estado { REPOSO, PANICO };
Estado estadoActual = REPOSO;

unsigned long tPanico     = 0;    // Timestamp de inicio del modo pánico
unsigned long tUltimoEnvio = 0;   // Timestamp del último paquete LoRa enviado
int numEvidencia = 0;             // Contador incremental de paquetes de evidencia
String tipoMovimiento = "PANICO"; // Clasificación del evento para el JSON

// Duración máxima del modo pánico: 1 hora = 3 600 000 ms
#define DURACION_PANICO  3600000UL
// Intervalo entre transmisiones LoRa en modo pánico: 2 segundos
#define INTERVALO_ENVIO  2000UL

// Botón de pánico en GPIO0 (activo en LOW, con pull-up interno)
#define BTN_PANICO 0
bool botonPresionado = false;  // Estado anterior del botón (para debounce)
unsigned long tBoton = 0;      // Timestamp del último evento del botón

// Variables del sistema de alerta visual en modo REPOSO
bool enAlerta = false;
unsigned long tiempoAlerta = 0; // Timestamp de inicio de la alerta actual
String tipoAlerta = "";         // Texto descriptivo de la alerta
float magAlerta = 0;            // Magnitud g que disparó la alerta
const unsigned long DUR_ALERTA = 3000; // Duración de la pantalla de alerta (3 s)

// Control de la rotación de páginas en modo REPOSO
uint8_t pagina = 0;                        // Página actualmente mostrada (0, 1 o 2)
unsigned long tCambioPagina = 0;           // Timestamp del último cambio de página
const unsigned long INTERVALO_PAG = 4000;  // Tiempo entre cambios de página (4 s)

/* ================================================================
 *  FUNCIÓN DE CAMBIO DE BUS I2C CORREGIDA
 *  Usa pinMode estándar para liberar pines (evita error de compilación)
 *
 *  Problema: el ESP32 IDF tiene funciones para desconectar pines del
 *  periférico I2C, pero no son accesibles desde el framework Arduino
 *  de Heltec sin errores de compilación. Solución: configurar los pines
 *  como INPUT (alta impedancia) antes de reasignarlos a otro periférico.
 * ================================================================ */

// Pines activos actuales del bus I2C (para detectar si ya están configurados)
int active_sda = -1;
int active_scl = -1;

/**
 * cambiarBusI2C()
 * Reasigna el bus I2C compartido (busI2C) a un par de pines SDA/SCL diferente.
 * Pasos:
 *   1. Si los pines nuevos coinciden con los actuales, no hace nada.
 *   2. Detiene el bus I2C para liberar el hardware.
 *   3. Pone los pines anteriores en modo INPUT para desconectarlos eléctricamente.
 *   4. Espera 5 ms para que el bus se estabilice.
 *   5. Reinicia el bus en los nuevos pines a 50 kHz (modo estable).
 *   6. Actualiza el registro de pines activos.
 */
void cambiarBusI2C(int nuevo_sda, int nuevo_scl) {
  // 1. Si ya estamos en estos pines, no hacer nada
  if (active_sda == nuevo_sda && active_scl == nuevo_scl) return;

  // 2. Detener el bus
  busI2C.end();
  delay(1);

  // 3. LIBERAR PINES ANTIGUOS (Método compatible Arduino)
  // Al ponerlos en INPUT, se desconectan eléctricamente del periférico I2C
  if (active_sda != -1) {
    pinMode(active_sda, INPUT);
    pinMode(active_scl, INPUT);
  }

  // 4. Pausa para estabilización
  delay(5);

  // 5. Iniciar bus en nuevos pines
  busI2C.begin(nuevo_sda, nuevo_scl);
  busI2C.setClock(50000); // 50kHz para máxima estabilidad
  busI2C.setTimeOut(50);

  // 6. Guardar estado actual
  active_sda = nuevo_sda;
  active_scl = nuevo_scl;
}

// Funciones de conveniencia para seleccionar cada sensor en el bus I2C
void seleccionarMPU() { cambiarBusI2C(MPU_SDA, MPU_SCL); }
void seleccionarRTC() { cambiarBusI2C(RTC_SDA, RTC_SCL); }
void seleccionarMAX() { cambiarBusI2C(MAX_SDA, MAX_SCL); }

/* ================================================================
 *  UTILIDADES Y PANTALLAS
 * ================================================================ */

/**
 * hashFNV()
 * Implementación del algoritmo FNV-1a (32 bits) sobre un String de Arduino.
 * Se usa para generar un hash de integridad del JSON de evidencia antes de
 * enviarlo por LoRa, permitiendo al receptor detectar corrupción de datos.
 */
uint32_t hashFNV(const String &s) {
  uint32_t h = 0x811c9dc5UL; // Valor inicial FNV offset basis
  for (size_t i = 0; i < s.length(); i++) {
    h ^= (uint8_t)s[i]; // XOR con el byte actual
    h *= 0x01000193UL;  // Multiplicación FNV prime
  }
  return h;
}

/**
 * hashHex()
 * Convierte el hash FNV-1a a una cadena hexadecimal de 8 caracteres
 * para incluirla en el campo "hash" del JSON de evidencia.
 */
String hashHex(const String &s) {
  char buf[9];
  snprintf(buf, 9, "%08X", hashFNV(s));
  return String(buf);
}

/**
 * tiempoRestante()
 * Calcula y formatea el tiempo restante del modo pánico en MM:SS.
 * Se muestra en la pantalla de emergencia para que el usuario sepa
 * cuánto tiempo de transmisión automática le queda.
 */
String tiempoRestante() {
  long ms = (long)(DURACION_PANICO - (millis() - tPanico));
  if (ms < 0) ms = 0;
  char buf[8];
  snprintf(buf, 8, "%02d:%02d", (int)(ms / 60000), (int)((ms % 60000) / 1000));
  return String(buf);
}

/**
 * obtenerTimestamp()
 * Devuelve una cadena ISO 8601 con la hora actual, con la siguiente
 * prioridad de fuente:
 *   1. RTC DS3231 (más preciso si está inicializado)
 *   2. GPS NMEA con ajuste UTC-6 (hora de México)
 *   3. millis() como último recurso (sin referencia absoluta)
 */
String obtenerTimestamp() {
  if (rtcOK) {
    DateTime local = ultimaHora;
    char buf[20];
    snprintf(buf, 20, "%04d-%02d-%02dT%02d:%02d:%02d",
             local.year(), local.month(), local.day(),
             local.hour(), local.minute(), local.second());
    return String(buf);
  }
  if (gps.time.isValid() && gps.date.isValid()) {
    // El GPS entrega hora UTC; se aplica offset de -6 horas (CST México)
    int hLocal = (gps.time.hour() - 6 + 24) % 24;
    char buf[20];
    snprintf(buf, 20, "%04d-%02d-%02dT%02d:%02d:%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             hLocal, gps.time.minute(), gps.time.second());
    return String(buf);
  }
  // Último recurso: millis() desde el arranque (no es una fecha real)
  return String(millis());
}

/**
 * clasificarEvento()
 * Clasifica el tipo de evento de movimiento brusco según la magnitud
 * del vector de aceleración:
 *   > 4.5g → CAIDA BRUSCA
 *   > 3.5g → IMPACTO FUERTE
 *   <= 3.5g → FORCEJEO
 */
String clasificarEvento(float m) {
  if (m > 4.5f) return "CAIDA BRUSCA";
  if (m > 3.5f) return "IMPACTO FUERTE";
  return "FORCEJEO";
}

/* ---- Funciones de renderizado de pantalla ---- */

/**
 * paginaMovimiento()
 * Muestra en el OLED los datos del acelerómetro MPU6050:
 *   - Aceleración en X, Y, Z en g
 *   - Módulo total (magnitud) en g (fuente grande)
 *   - BPM promedio del MAX30102 (si está activo)
 *   - Barra de progreso proporcional a la magnitud (0-5g → 0-100%)
 */
void paginaMovimiento() {
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, mpuOK ? "-- MOVIMIENTO --" : "-- MPU: ERROR --");
  char buf[28];
  snprintf(buf, sizeof(buf), "X:%.2f  Y:%.2f", ax_g, ay_g);
  oled.drawString(0, 13, buf);
  snprintf(buf, sizeof(buf), "Z:%.2f", az_g);
  oled.drawString(0, 25, buf);
  oled.setFont(ArialMT_Plain_16);
  snprintf(buf, sizeof(buf), "%.2f g", mag);
  oled.drawString(0, 38, buf);
  if (maxOK) {
    oled.setFont(ArialMT_Plain_10);
    snprintf(buf, sizeof(buf), "BPM: %d", beatAvg);
    oled.drawString(75, 38, buf);
  }
  // Barra de progreso: mapea [0, 5g] a [0, 100%]
  oled.drawProgressBar(14, 2, 100, 8, constrain((int)((mag / 5.0f) * 100), 0, 100));
  oled.display();
}

/**
 * paginaGPS()
 * Muestra en el OLED el estado del GPS:
 *   - Número de satélites y estado del fix
 *   - Latitud y longitud (si hay fix válido)
 *   - Mensaje de espera si no hay fix
 */
void paginaGPS() {
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  char buf[34];
  snprintf(buf, sizeof(buf), "Sats: %d  %s",
           (int)gps.satellites.value(), gpsValido ? "FIX OK!" : "sin fix");
  oled.drawString(0, 0, buf);
  if (gpsValido) {
    snprintf(buf, sizeof(buf), "Lat: %.5f", lat);
    oled.drawString(0, 14, buf);
    snprintf(buf, sizeof(buf), "Lon: %.5f", lon);
    oled.drawString(0, 28, buf);
  } else {
    oled.drawString(0, 14, "Esperando fix...");
  }
  oled.display();
}

/**
 * paginaRTC()
 * Muestra en el OLED la fecha y hora actuales del RTC DS3231.
 * Si el RTC no está disponible, muestra mensajes de error y líneas vacías.
 */
void paginaRTC() {
  char hora[10]  = "--:--:--";
  char fecha[14] = "----/--/--";
  if (rtcOK) {
    DateTime local = ultimaHora;
    snprintf(hora,  sizeof(hora),  "%02d:%02d:%02d", local.hour(), local.minute(), local.second());
    snprintf(fecha, sizeof(fecha), "%04d/%02d/%02d", local.year(), local.month(), local.day());
  }
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, rtcOK ? "-- FECHA / HORA --" : "-- RTC: ERROR --");
  oled.setFont(ArialMT_Plain_16);
  oled.drawString(10, 16, hora);
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(15, 38, fecha);
  oled.display();
}

/**
 * mostrarAlerta()
 * Muestra una pantalla de alerta con bordes dobles, texto del tipo
 * de evento y la magnitud de aceleración que lo provocó.
 * Se llama cuando mag supera UMBRAL_G en modo REPOSO.
 */
void mostrarAlerta(const String& tipo, float m) {
  oled.clear();
  oled.drawRect(0, 0, 128, 64);   // Borde exterior
  oled.drawRect(2, 2, 124, 60);   // Borde interior
  oled.setFont(ArialMT_Plain_16);
  oled.drawString(50, 4, "!!");   // Símbolo de alerta
  oled.setFont(ArialMT_Plain_10);
  int w = oled.getStringWidth(tipo);
  oled.drawString((128 - w) / 2, 26, tipo); // Centrado horizontal
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f g", m);
  oled.drawString((128 - oled.getStringWidth(buf)) / 2, 40, buf);
  oled.display();
}

/**
 * pantallaPanico()
 * Pantalla de emergencia con efecto de parpadeo invertido (cada 500 ms).
 * Muestra: tipo de movimiento, magnitud, estado GPS, hora RTC,
 * número de evidencia, BPM y tiempo restante del modo pánico.
 * El encabezado alterna entre blanco sobre negro y negro sobre blanco.
 */
void pantallaPanico() {
  static unsigned long tBlink = 0;
  static bool inv = false;
  // Parpadeo cada 500 ms
  if (millis() - tBlink < 500) return;
  tBlink = millis();
  inv = !inv;
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  if (inv) {
    // Fondo blanco para el encabezado (efecto invertido)
    oled.setColor(WHITE);
    oled.fillRect(0, 0, 128, 14);
    oled.setColor(BLACK);
  }
  oled.drawString(15, 2, "!! EMERGENCIA !!");
  oled.setColor(WHITE);
  oled.drawLine(0, 14, 127, 14); // Separador
  char buf[30];
  snprintf(buf, sizeof(buf), "Mov: %.2fg %s", mag, tipoMovimiento.substring(0, 8).c_str());
  oled.drawString(0, 17, buf);
  oled.drawString(0, 29, gpsValido ? "GPS OK" : "GPS NO DISP");
  if (rtcOK) {
    char bufT[10];
    DateTime localP = ultimaHora;
    snprintf(bufT, 10, "%02d:%02d:%02d", localP.hour(), localP.minute(), localP.second());
    oled.drawString(0, 41, "Hora: " + String(bufT));
  }
  // Línea inferior: número de evidencia, BPM y tiempo restante
  snprintf(buf, sizeof(buf), "#%d BPM:%d %s", numEvidencia, beatAvg, tiempoRestante().c_str());
  oled.drawString(0, 53, buf);
  oled.display();
}

/* ================================================================
 *  LÓGICA PRINCIPAL
 * ================================================================ */

/**
 * leerBoton()
 * Lee el estado del botón de pánico (GPIO0, activo en LOW).
 * Incluye debounce de 400 ms. Si se detecta una pulsación y el sistema
 * está en REPOSO, transiciona al estado PÁNICO e inicializa sus variables.
 * Si ya está en PÁNICO, ignora pulsaciones adicionales (no lo desactiva).
 */
void leerBoton() {
  bool btn = (digitalRead(BTN_PANICO) == LOW);
  if (btn && !botonPresionado && millis() - tBoton > 400) {
    botonPresionado = true;
    tBoton = millis();
    if (estadoActual == PANICO) return; // Ya en pánico, ignorar
    estadoActual = PANICO;
    tPanico = millis();
    tUltimoEnvio = 0;
    numEvidencia = 0;
    tipoMovimiento = "PANICO";
    Serial.println(F("PANICO ACTIVADO"));
  }
  if (!btn) botonPresionado = false; // Liberar flag cuando se suelta el botón
}

/**
 * enviarEvidencia()
 * Construye y transmite un paquete LoRa con los datos de emergencia.
 * El payload es un objeto JSON con:
 *   - tipo: clasificación del evento (PANICO, PANICO+CAIDA, PANICO+FORCEJEO)
 *   - acel: magnitud de aceleración en g
 *   - bpm: frecuencia cardíaca promedio
 *   - lat/lon: coordenadas GPS (0.0 si sin fix)
 *   - sats: número de satélites GPS
 *   - fix: booleano de validez del GPS
 *   - ts: timestamp ISO 8601
 *   - num: número secuencial de evidencia
 *   - hash: FNV-1a del JSON para verificación de integridad
 *
 * El paquete inicia con un encabezado de 4 bytes:
 *   [dirDestino][dirLocal][idMsg][longitud del JSON]
 * seguido de los bytes del JSON.
 */
void enviarEvidencia() {
  numEvidencia++;
  // Clasificar el tipo de evento según la aceleración actual
  if (mag > 4.5f) tipoMovimiento = "PANICO+CAIDA";
  else if (mag > 2.5f) tipoMovimiento = "PANICO+FORCEJEO";
  else tipoMovimiento = "PANICO";

  String ts   = obtenerTimestamp();
  String json = "{";
  json += "\"tipo\":\""  + tipoMovimiento      + "\",";
  json += "\"acel\":"    + String(mag, 2)       + ",";
  json += "\"bpm\":"     + String(beatAvg)      + ",";
  json += "\"lat\":"     + String(lat, 6)       + ",";
  json += "\"lon\":"     + String(lon, 6)       + ",";
  json += "\"sats\":"    + String((int)gps.satellites.value()) + ",";
  json += "\"fix\":"     + String(gpsValido ? "true" : "false") + ",";
  json += "\"ts\":\""    + ts                   + "\",";
  json += "\"num\":"     + String(numEvidencia) + ",";
  json += "\"hash\":\""  + hashHex(json)        + "\""; // Hash calculado sobre el JSON parcial
  json += "}";

  // Construir el paquete LoRa con encabezado de enrutamiento
  int i = 0;
  bufTx[i++] = dirDestino;         // Dirección del receptor
  bufTx[i++] = dirLocal;           // Dirección del emisor
  bufTx[i++] = idMsg++;            // ID de mensaje (incrementa post-uso)
  bufTx[i++] = (byte)json.length(); // Longitud del payload JSON
  for (size_t j = 0; j < json.length(); j++) bufTx[i++] = (byte)json[j];

  int r = radio.transmit(bufTx, i);
  Serial.println(r == RADIOLIB_ERR_NONE ? "TX OK: " + json : "TX err: " + String(r));
}

/* ================================================================
 *  SETUP: Inicialización de todos los periféricos
 * ================================================================ */
void setup() {
  Serial.begin(115200);
  delay(1000); // Tiempo para que el monitor serie se conecte

  pinMode(BTN_PANICO, INPUT_PULLUP); // Botón con pull-up interno; activo en LOW

  // Encender el pin Vext para alimentar periféricos externos del módulo Heltec
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); // LOW = alimentado en el Heltec
  delay(300);

  // Inicializar pantalla OLED y mostrar mensaje de arranque
  oled.init();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Iniciando...");
  oled.display();
  delay(500);

  // --- Inicializar RTC DS3231 ---
  oled.drawString(0, 14, "DS3231...");
  oled.display();
  seleccionarRTC(); // Cambiar el bus I2C a los pines del RTC
  rtcOK = rtc.begin(&busI2C);
  if (rtcOK) {
    // Si el RTC perdió energía (batería agotada) o se fuerza el ajuste, escribir hora de compilación
    if (rtc.lostPower() || FORZAR_AJUSTE)
      rtc.adjust(DateTime(2026, 3, 1, 23, 26, 0)); // Hora inicial hardcodeada
    ultimaHora = rtc.now(); // Leer hora actual del RTC
  }

  // --- Inicializar MPU6050 ---
  oled.drawString(0, 28, "MPU6050...");
  oled.display();
  seleccionarMPU(); // Cambiar el bus I2C a los pines del MPU
  sensor.initialize();
  mpuOK = sensor.testConnection();
  Serial.println(mpuOK ? "MPU6050: OK" : "MPU6050: ERROR");

  // --- Inicializar MAX30102 ---
  oled.drawString(0, 42, "MAX30102...");
  oled.display();
  seleccionarMAX();
  delay(300);
  // begin() devuelve true si el sensor respondió correctamente
  if (particleSensor.begin(busI2C, I2C_SPEED_STANDARD)) {
    particleSensor.setup();                      // Configuración por defecto
    particleSensor.setPulseAmplitudeRed(0x0A);   // LED rojo a baja potencia (medición de SpO2/HR)
    particleSensor.setPulseAmplitudeGreen(0);    // LED verde apagado (no necesario para HR)
    maxOK = true;
  }

  // --- Inicializar GPS (UART1: RX=45, TX=46, 9600 baud) ---
  neogps.begin(9600, SERIAL_8N1, 45, 46);

  // --- Inicializar radio LoRa SX1262 ---
  SPI.begin(9, 11, 10, 8); // SCK, MISO, MOSI, CS
  delay(200);
  int st = radio.begin(FREQUENCY, BANDWIDTH, SPREAD_FACTOR, CODING_RATE);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setOutputPower(TX_POWER); // Potencia máxima
    radio.setCRC(true);             // Habilitar CRC para detección de errores en el aire
  }

  // --- Mostrar resumen de diagnóstico en pantalla ---
  oled.clear();
  oled.drawString(0, 0,  rtcOK ? "DS3231:  OK"     : "DS3231:  ERROR");
  oled.drawString(0, 14, mpuOK ? "MPU6050: OK"     : "MPU6050: ERROR");
  oled.drawString(0, 28, maxOK ? "MAX30102: OK"    : "MAX30102: ERROR");
  oled.display();
  delay(3000); // Mantener pantalla de diagnóstico 3 segundos

  seleccionarMPU(); // Empezar el loop con el bus apuntando al MPU
}

/* ================================================================
 *  LOOP: Ciclo principal de ejecución
 * ================================================================ */
void loop() {
  // --- Lectura del botón de pánico ---
  leerBoton();

  // --- Lectura y decodificación de tramas NMEA del GPS (sin bloqueo) ---
  while (neogps.available()) gps.encode(neogps.read());
  // Actualizar posición si el GPS tiene fix válido
  if (gps.location.isValid()) {
    gpsValido = true;
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  // --- Sincronización RTC con GPS (se ejecuta solo una vez) ---
  // Cuando el GPS adquiere fix por primera vez, se escribe la hora UTC-6 al RTC
  if (!sincronizado && rtcOK && gps.location.isValid() &&
      gps.time.isValid() && gps.date.isValid()) {
    seleccionarRTC();
    DateTime utcGPS(gps.date.year(), gps.date.month(), gps.date.day(),
                    gps.time.hour(), gps.time.minute(), gps.time.second());
    DateTime localGPS(utcGPS.unixtime() - 6UL * 3600UL); // Convertir UTC a UTC-6
    rtc.adjust(localGPS);
    sincronizado = true;
    Serial.println("RTC sincronizado GPS");
  }

  // --- Lectura del sensor de pulso MAX30102 ---
  // Solo se intenta leer si el sensor se inicializó correctamente
  if (maxOK) {
    seleccionarMAX();
    long ir = particleSensor.getIR(); // Valor de intensidad infrarroja
    if (ir < 50000) {
      // Si el valor IR es muy bajo, no hay dedo sobre el sensor
      beatAvg = 0;
    } else if (checkForBeat(ir)) {
      // checkForBeat() devuelve true cuando detecta un pico (latido)
      long delta = millis() - lastBeat; // Intervalo entre latidos en ms
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0); // Convertir intervalo a BPM
      if (beatsPerMinute > 20 && beatsPerMinute < 255) {
        // Solo guardar lecturas plausibles (20-255 BPM)
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE; // Buffer circular
        // Calcular promedio de las últimas RATE_SIZE lecturas
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  }

  // --- Alternancia MPU6050 / RTC en el bus I2C (cada 500 ms) ---
  // Para no bloquear el loop, se usa un timer no bloqueante que
  // alterna entre leer el acelerómetro y actualizar el reloj.
  static unsigned long tAlterna = 0;
  if (millis() - tAlterna > 500) {
    tAlterna = millis();
    static bool turnoMPU = true; // Alterna entre MPU y RTC en cada iteración

    if (turnoMPU) {
      // --- Turno MPU6050: leer aceleración ---
      seleccionarMPU();
      // Verificar que el sensor sigue respondiendo (reconexión automática)
      if (!sensor.testConnection()) {
        Serial.println("MPU perdido, reinit...");
        sensor.initialize();
        mpuOK = sensor.testConnection();
      }
      if (mpuOK) {
        int16_t axr, ayr, azr;
        sensor.getAcceleration(&axr, &ayr, &azr); // Lectura raw en LSB
        // Convertir de LSB a g usando la constante de escala
        ax_g = axr / LSB_POR_G;
        ay_g = ayr / LSB_POR_G;
        az_g = azr / LSB_POR_G;
        // Módulo del vector (magnitud total de la aceleración)
        mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
      }
      // Disparar alerta si la magnitud supera el umbral (solo en REPOSO)
      if (estadoActual == REPOSO && mag > UMBRAL_G) {
        enAlerta = true;
        tiempoAlerta = millis();
        tipoAlerta = clasificarEvento(mag);
        magAlerta  = mag;
        Serial.printf("ALERTA: %s | %.2f g\n", tipoAlerta.c_str(), mag);
      }
    } else {
      // --- Turno RTC: actualizar la hora en memoria ---
      if (rtcOK) {
        seleccionarRTC();
        ultimaHora = rtc.now(); // Leer hora actual y guardarla globalmente
      }
    }
    turnoMPU = !turnoMPU; // Cambiar turno para la próxima iteración
  }

  // --- Máquina de estados principal ---
  switch (estadoActual) {

    case REPOSO:
      // Si hay una alerta activa, mostrarla hasta que expire (DUR_ALERTA ms)
      if (enAlerta) {
        mostrarAlerta(tipoAlerta, magAlerta);
        if (millis() - tiempoAlerta >= DUR_ALERTA) enAlerta = false;
      } else {
        // Rotar entre las 3 páginas de información cada INTERVALO_PAG ms
        if (millis() - tCambioPagina > INTERVALO_PAG) {
          pagina = (pagina + 1) % 3;
          tCambioPagina = millis();
        }
        switch (pagina) {
          case 0: paginaMovimiento(); break; // Datos MPU6050 y BPM
          case 1: paginaGPS();        break; // Coordenadas y fix GPS
          case 2: paginaRTC();        break; // Fecha y hora del RTC
        }
      }
      break;

    case PANICO:
      // Enviar paquete LoRa cada INTERVALO_ENVIO ms
      if (millis() - tUltimoEnvio >= INTERVALO_ENVIO) {
        enviarEvidencia();
        tUltimoEnvio = millis();
      }
      // Actualizar pantalla de emergencia (con parpadeo interno)
      pantallaPanico();
      // Volver a REPOSO automáticamente al agotar la duración máxima
      if (millis() - tPanico >= DURACION_PANICO) {
        estadoActual = REPOSO;
        numEvidencia = 0;
      }
      break;
  }

  delay(10); // Pequeña pausa para ceder tiempo al scheduler y al watchdog
}
