/**
 * NODO RECEPTOR - ESTACION BASE CON PANTALLAS OLED MEJORADAS
 * 
 * Descripción:
 *   Dispositivo receptor que captura tramas LoRa de la pulsera,
 *   valida integridad de datos, muestra información en 6 pantallas OLED,
 *   y reenvía datos por ESP-NOW al gateway celular.
 * 
 * Características principales:
 *   - 6 pantallas OLED diferentes para visualización de datos
 *   - Detección automática de pérdida de señal
 *   - Buzzer inteligente (solo en alertas de pánico/caída)
 *   - Historial de últimos 5 paquetes recibidos
 *   - Estadísticas de recepción (RX, errores, tasa de éxito)
 *   - Comunicación bidireccional por ESP-NOW con gateway
 * 
 * Pantallas disponibles:
 *   1. ESPERA: Arcos de señal LoRa animados + barra de escaneo
 *   2. DATOS/ALERTA: Mini-cards de BPM y aceleración
 *   3. DETALLES GPS: Coordenadas, satélites, SNR
 *   4. ESTADÍSTICAS: RX/ERR/%OK + barra de éxito
 *   5. HISTORIAL: Últimos 5 eventos recibidos
 *   6. SEÑAL PERDIDA: Antena rota, tiempo MM:SS, timeout
 * 
 * Hardware:
 *   - Heltec ESP32 LoRa v3 (SX1276)
 *   - Display OLED Adafruit SSD1306 (128x64)
 *   - Buzzer activo (GPIO 13)
 *   - LED indicador (GPIO 25)
 * 
 * Autor: Equipo Shero
 * Fecha: Mayo 2026
 * Versión: 2.0
 */

#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

/* ================================================================
 *  CONFIGURACION DE HARDWARE Y DISPLAY
 * ================================================================ */

// Parámetros del display OLED SSD1306 (Adafruit)
#define SCREEN_WIDTH  128         // Ancho en píxeles
#define SCREEN_HEIGHT 64          // Alto en píxeles
#define OLED_RESET     16         // Pin de reset
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Radio LoRa SX1276 en Heltec ESP32 v3
SX1276 radio = new Module(18, 26, 14, 35);

// Parámetros de recepción LoRa
#define FREQUENCY      915.0      // Frecuencia: 915 MHz
#define BANDWIDTH      125.0      // Ancho de banda: 125 kHz
#define SPREAD_FACTOR  8          // Factor de propagación: SF8
#define CODING_RATE    5          // Velocidad de código: 4/5
#define LED_PIN        25         // GPIO 25 = LED indicador
#define BUZZER_PIN     13         // GPIO 13 = Buzzer activo

// Direcciones LoRa personalizadas
byte dirLocal   = 0xD3;           // Dirección de este receptor
byte dirPulsera = 0xC1;           // Dirección esperada de la pulsera

// Dirección MAC del gateway para ESP-NOW
uint8_t macGateway[] = {0x80, 0x64, 0x6F, 0xFC, 0x0A, 0x50};

/* ================================================================
 *  ESTRUCTURA DE DATOS PARA ALMACENAR EVIDENCIAS
 * ================================================================ */

/**
 * Estructura que contiene todos los datos de una alerta recibida
 * Facilita el almacenamiento en historial y visualización en pantallas
 */
struct Evidencia {
  String tipo;      // Tipo de evento: PANICO, CAIDA, NORMAL, etc
  float  acel;      // Magnitud de aceleración (g)
  int    bpm;       // Frecuencia cardíaca (latidos por minuto)
  int    spo2;      // Saturación de oxígeno (%)
  double lat;       // Latitud (coordenada GPS)
  double lon;       // Longitud (coordenada GPS)
  int    sats;      // Número de satélites visibles
  bool   fix;       // ¿Tiene fijación GPS?
  String ts;        // Timestamp (formato ISO 8601)
  int    numero;    // Número de secuencia del paquete
  float  rssi;      // Potencia de señal LoRa (dBm)
  float  snr;       // Relación señal-ruido (dB)
  bool   valid;     // ¿Datos válidos/parseados correctamente?
};

// Buffer de historial (últimos 5 paquetes)
#define HIST_SIZE 5
Evidencia historial[HIST_SIZE];  // Array circular de evidencias
Evidencia ultimo;                // Último paquete recibido

// Estadísticas globales
int  totalRecibidos = 0;         // Total de paquetes válidos recibidos
int  totalCorruptos = 0;         // Total de paquetes con errores de parseo
bool hayDato        = false;     // ¿Hay datos nuevos para mostrar?

// Control de pantallas
int  pantActual    = 0;          // Pantalla actualmente mostrada (0-3)
#define N_PANTALLAS 4            // Número total de pantallas
bool btnPresionado = false;      // Estado anterior del botón
unsigned long tBtn   = 0;        // Timestamp de última pulsación
unsigned long tBlink = 0;        // Timestamp del último parpadeo

// Control de señal LoRa
unsigned long ultimoTiempoRX = 0;           // Timestamp del último RX válido
const long    TIMEOUT_SENAL  = 15000;       // Timeout de señal perdida: 15s
const long TIMEOUT_SENAL_PERDIDA = 900000;  // Timeout total: 15 minutos (900s)

// Estado de pérdida de señal
bool senalPerdida   = false;     // ¿Detectada pérdida de señal LoRa?

// Control de alertas y buzzer
bool          alertaActiva   = false;     // ¿Buzzer activo?
unsigned long tiempoAlerta   = 0;        // Timestamp de inicio de alerta
const long    DURACION_ALERTA = 30000;   // Duración de alerta: 30 segundos

/* ================================================================
 *  ICONOS XBM PARA PANTALLA OLED
 * ================================================================ */

// Ícono de corazón (12x12 píxeles) - usado para mostrar BPM
const uint8_t HEART_BITS[]  = { 
  0x36,0x00,0x7F,0x00,0xFF,0x00,0xFF,0x00,0x7E,0x00,0x3C,0x00,0x18,0x00,0x08,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 
};

// Ícono de gota (8x10 píxeles) - usado para mostrar SpO2
const uint8_t DROP_BITS[]   = { 
  0x0C,0x1E,0x3F,0x3F,0x3F,0x3F,0x3F,0x1E,0x0C,0x00,0x00,0x00 
};

// Ícono de pin/mapa (8x12 píxeles) - usado para mostrar ubicación GPS
const uint8_t PIN_BITS[]    = { 
  0x3E,0x7F,0x7F,0x7F,0x7F,0x3E,0x1C,0x08,0x08,0x00,0x00,0x00 
};

// Ícono de onda (10x8 píxeles) - usado para mostrar movimiento
const uint8_t WAVE_BITS[]   = { 
  0x00,0x00,0x22,0x00,0x55,0x00,0x88,0x00,0x55,0x00,0x22,0x00,0x00,0x00,0x00,0x00 
};

// Ícono de señal (10x10 píxeles) - usado para mostrar potencia LoRa
const uint8_t SIGNAL_BITS[] = { 
  0x00,0x00,0x10,0x00,0x38,0x00,0x7C,0x00,0x38,0x00,0x10,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 
};

/* ================================================================
 *  FUNCIONES DE PARSEO DE DATOS JSON
 * ================================================================ */

/**
 * Parsea un JSON recibido de la pulsera y extrae datos en estructura Evidencia
 * Utiliza lambdas para extraer números y cadenas de manera flexible
 */
bool parsearJSON(const String &raw, Evidencia &ev) {
  // Lambda para extraer valores numéricos
  auto extraerNum = [&](const String &key) -> String {
    String k = "\"" + key + "\":";
    int i = raw.indexOf(k);
    if (i < 0) return "0";  // Valor por defecto si no existe
    i += k.length();
    int j = i;
    // Buscar fin del número (coma, cierre o espacio)
    while (j < (int)raw.length() && raw[j] != ',' && raw[j] != '}' && raw[j] != ' ') j++;
    return raw.substring(i, j);
  };
  
  // Lambda para extraer cadenas entre comillas
  auto extraerStr = [&](const String &key) -> String {
    String k = "\"" + key + "\":\"";
    int i = raw.indexOf(k);
    if (i < 0) return "";
    i += k.length();
    int j = raw.indexOf("\"", i);
    return j < 0 ? "" : raw.substring(i, j);
  };

  // Extraer campos numéricos
  ev.lat  = extraerNum("Lat").toDouble();
  ev.lon  = extraerNum("Lon").toDouble();
  ev.bpm  = extraerNum("BPM").toInt();
  ev.spo2 = extraerNum("SpO2").toInt();
  ev.acel = extraerNum("Mag").toFloat();
  
  // Extraer campos de cadena
  String f = extraerStr("Fecha");
  String h = extraerStr("Hora");
  ev.ts   = (f.length() > 0 && h.length() > 0) ? f + "T" + h : "N/A";
  
  // Determinar tipo de evento
  int sos = extraerNum("SOS").toInt();
  ev.tipo = (sos == 1) ? "PANICO" : "NORMAL";
  
  // Verificar fijación GPS
  ev.fix  = (ev.lat != 0.0 || ev.lon != 0.0);
  ev.sats = ev.fix ? 8 : 0;  // Simulación: 8 satélites si hay fix
  ev.valid = true;
  
  return true;
}

/* ================================================================
 *  FUNCIONES AUXILIARES DE PANTALLA
 * ================================================================ */

/**
 * Dibuja encabezado con fondo blanco e texto negro
 * Estilo invertido para resaltar
 */
void drawHdr(const char *titulo) {
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);  // Fondo blanco
  display.setTextColor(SSD1306_BLACK);              // Texto negro
  display.setTextSize(1);
  display.setCursor((128 - (strlen(titulo) * 6)) / 2, 2);
  display.print(titulo);
  display.setTextColor(SSD1306_WHITE);  // Restaurar color blanco
}

/**
 * Dibuja puntos de paginación en la parte inferior
 * Indica qué pantalla está activa (llena) vs inactivas (huecas)
 */
void drawDots() {
  int dotW = 8, gap = 3;
  int startX = (128 - (N_PANTALLAS * dotW + (N_PANTALLAS - 1) * gap)) / 2;
  for (int i = 0; i < N_PANTALLAS; i++) {
    int x = startX + i * (dotW + gap);
    if (i == pantActual) display.fillRect(x, 61, dotW, 3, SSD1306_WHITE);  // Activa (llena)
    else                 display.drawRect(x, 61, dotW, 3, SSD1306_WHITE);   // Inactiva (hueca)
  }
}

/**
 * Dibuja barra de progreso con bordes redondeados
 * Relleno proporcional al porcentaje
 */
void drawBarra(int x, int y, int w, int h, int pct) {
  display.drawRoundRect(x, y, w, h, 1, SSD1306_WHITE);  // Marco
  int fill = (pct * (w - 2)) / 100;  // Calcular ancho de relleno
  if (fill > 0) display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);  // Rellenar
}

/**
 * Dibuja mini-card redondeada con etiqueta y valor
 * Parámetros: posición (x,y), tamaño (w,h), etiqueta, valor
 */
void drawCard(int x, int y, int w, int h, const char* label, const char* valor) {
  display.drawRoundRect(x, y, w, h, 2, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x + 3, y + 2);
  display.print(label);
  display.setTextSize(2);
  display.setCursor(x + 3, y + 11);
  display.print(valor);
  display.setTextSize(1);
}

/* ================================================================
 *  ÍCONO DE ANTENA ROTA (para pantalla de señal perdida)
 *  Técnica: Dibujar círculos completos + rellenar rectángulos negros
 *           para "borrar" la mitad inferior, simulando arcos superiores
 * ================================================================ */
void dibujarAntenRota(int cx, int cy) {
  // Palo vertical de la antena
  display.drawLine(cx, cy - 14, cx, cy - 6, SSD1306_WHITE);
  
  // Base horizontal de la antena
  display.drawLine(cx - 5, cy - 6, cx + 5, cy - 6, SSD1306_WHITE);

  // Arco pequeño (radio 6 píxeles)
  display.drawCircle(cx, cy - 4, 6, SSD1306_WHITE);
  display.fillRect(cx - 7, cy - 4, 15, 8, SSD1306_BLACK);  // Borra mitad inferior

  // Arco mediano (radio 11 píxeles)
  display.drawCircle(cx, cy - 4, 11, SSD1306_WHITE);
  display.fillRect(cx - 12, cy - 4, 25, 13, SSD1306_BLACK);  // Borra mitad inferior

  // X de "sin señal" en la esquina superior derecha del ícono
  display.drawLine(cx + 7, cy - 16, cx + 12, cy - 11, SSD1306_WHITE);
  display.drawLine(cx + 12, cy - 16, cx + 7, cy - 11, SSD1306_WHITE);
}

/* ================================================================
 *  FUNCIONES DE CONTROL DEL BUZZER
 * ================================================================ */

/**
 * Enciende el buzzer (GPIO 13 = HIGH)
 */
void buzzerOn()  { digitalWrite(BUZZER_PIN, HIGH); }

/**
 * Apaga el buzzer (GPIO 13 = LOW)
 */
void buzzerOff() { digitalWrite(BUZZER_PIN, LOW);  }

/* ================================================================
 *  PANTALLA 1: ESPERA (modo escucha)
 *  Muestra: Arcos de señal LoRa animados + barra de escaneo animada
 * ================================================================ */
void pantEspera() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Título superior centrado
  display.setTextSize(1);
  display.setCursor((128 - (15 * 6)) / 2, 1);
  display.print("INSTITUTO MUJER");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // Arcos de señal LoRa tipo WiFi (centrados)
  int cx = 64, cy = 38;
  
  // Punto central con parpadeo
  if (millis() % 1200 < 600) display.fillCircle(cx, cy, 2, SSD1306_WHITE);  // Lleno
  else                        display.drawCircle(cx, cy, 2, SSD1306_WHITE);  // Hueco

  // Primer arco (radio 7)
  display.drawCircle(cx, cy, 7, SSD1306_WHITE);
  display.fillRect(cx - 8, cy, 17, 9, SSD1306_BLACK);   // Borra mitad inferior

  // Segundo arco (radio 13)
  display.drawCircle(cx, cy, 13, SSD1306_WHITE);
  display.fillRect(cx - 14, cy, 29, 15, SSD1306_BLACK);  // Borra mitad inferior

  // Tercer arco (radio 19)
  display.drawCircle(cx, cy, 19, SSD1306_WHITE);
  display.fillRect(cx - 20, cy, 41, 21, SSD1306_BLACK);  // Borra mitad inferior

  // Barra animada de "escaneando" (va y viene)
  unsigned long t = millis() % 2000;
  int barPos = (t < 1000) ? (t * 110 / 1000) : ((2000 - t) * 110 / 1000);
  display.drawRoundRect(8, 54, 112, 4, 1, SSD1306_WHITE);
  display.fillRect(9 + barPos, 55, 8, 2, SSD1306_WHITE);

  // Información de configuración LoRa
  display.setTextSize(1);
  display.setCursor(28, 46);
  display.print("915MHz | SF8 | BW125");

  display.display();  // Actualizar pantalla
}

/* ================================================================
 *  PANTALLA 2: SEÑAL PERDIDA
 *  Muestra: Antena rota, tiempo MM:SS, barra de timeout
 * ================================================================ */
void pantSignalLost() {
  display.clearDisplay();

  // Header parpadeante para indicar urgencia
  bool blinking = (millis() % 800 < 400);
  if (blinking) {
    display.fillRect(0, 0, 128, 12, SSD1306_WHITE);  // Fondo blanco
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
    display.drawRect(0, 0, 128, 12, SSD1306_WHITE);  // Solo marco
  }
  display.setTextSize(1);
  display.setCursor(14, 2);
  display.print("!! SENAL PERDIDA !!");
  display.setTextColor(SSD1306_WHITE);

  // Ícono de antena rota (lado izquierdo)
  dibujarAntenRota(24, 38);

  // Tiempo transcurrido sin señal (formato MM:SS, tamaño grande)
  unsigned long segs = (millis() - ultimoTiempoRX) / 1000;
  char buf[8];
  sprintf(buf, "%02lu:%02lu", segs / 60, segs % 60);
  display.setTextSize(2);
  int txtW = strlen(buf) * 12;
  display.setCursor(64 + (64 - txtW) / 2, 18);  // Centrado en mitad derecha
  display.print(buf);
  display.setTextSize(1);

  // Subtítulo "sin datos"
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(50, 35);
  display.print("sin datos");

  // Línea separadora
  display.drawLine(4, 45, 124, 45, SSD1306_WHITE);

  // Último paquete recibido antes de perder señal
  display.setCursor(2, 48);
  display.print("Ult:#" + String(ultimo.numero) + " | " + ultimo.tipo);

  // Barra de progreso del timeout (llena de izquierda a derecha)
  int pctTimeout = min(100, (int)((millis() - ultimoTiempoRX) * 100L / TIMEOUT_SENAL));
  display.drawRoundRect(2, 57, 124, 4, 1, SSD1306_WHITE);
  int fill = (pctTimeout * 122) / 100;
  if (fill > 0) display.fillRect(3, 58, fill, 2, SSD1306_WHITE);

  display.display();  // Actualizar pantalla
}

/* ================================================================
 *  PANTALLA 3: DATOS / ALERTA
 *  Muestra: Mini-cards de BPM y aceleración, ubicación GPS
 *  - Si es PANICO/CAIDA: pantalla parpadea, datos resaltados
 *  - Si es NORMAL: visualización limpia de datos recibidos
 * ================================================================ */
void pant_alerta() {
  bool esAlerta = (ultimo.tipo == "PANICO" || ultimo.tipo == "CAIDA");
  display.clearDisplay();

  if (esAlerta) {
    // === MODO ALERTA: Pantalla de emergencia ===
    
    // Header que parpadea (solo el header, no toda la pantalla)
    static bool inv = false;
    static unsigned long tI = 0;
    if (millis() - tI > 500) { inv = !inv; tI = millis(); }

    if (inv) {
      display.fillRect(0, 0, 128, 13, SSD1306_WHITE);  // Fondo blanco
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.drawRect(0, 0, 128, 13, SSD1306_WHITE);  // Solo marco
      display.setTextColor(SSD1306_WHITE);
    }
    display.setTextSize(1);
    display.setCursor((128 - (strlen(ultimo.tipo.c_str()) * 6 + 6)) / 2, 3);
    display.print("! " + ultimo.tipo + " !");

    // Resto de datos siempre legible (blanco sobre negro)
    display.setTextColor(SSD1306_WHITE);

    // Mini-card BPM (lado izquierdo)
    display.drawRoundRect(1, 15, 60, 26, 2, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(5, 17);
    display.print("BPM");
    display.setTextSize(2);
    display.setCursor(5, 25);
    display.print(String(ultimo.bpm));
    display.setTextSize(1);

    // Mini-card Aceleración (lado derecho)
    display.drawRoundRect(66, 15, 61, 26, 2, SSD1306_WHITE);
    display.setCursor(70, 17);
    display.print("Impacto");
    display.setTextSize(2);
    display.setCursor(70, 25);
    display.print(String(ultimo.acel, 1) + "g");
    display.setTextSize(1);

    // Línea separadora
    display.drawLine(0, 43, 128, 43, SSD1306_WHITE);

    // Ubicación GPS
    display.setCursor(2, 46);
    if (ultimo.fix) {
      display.print(String(ultimo.lat, 5));
      display.setCursor(2, 55);
      display.print(String(ultimo.lon, 5));
    } else {
      display.setCursor(20, 50);
      display.print("Sin ubicacion GPS");
    }

    drawDots();  // Mostrar indicador de pantalla actual

  } else {
    // === MODO NORMAL: Visualización de datos recibidos ===
    
    display.setTextColor(SSD1306_WHITE);
    drawHdr("DATOS RECIBIDOS");

    // Mini-card BPM (lado izquierdo)
    display.drawRoundRect(1, 13, 60, 22, 2, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(5, 15);
    display.print("BPM");
    display.setTextSize(2);
    display.setCursor(5, 22);
    display.print(String(ultimo.bpm));
    display.setTextSize(1);

    // Mini-card Movimiento (lado derecho)
    display.drawRoundRect(66, 13, 61, 22, 2, SSD1306_WHITE);
    display.setCursor(70, 15);
    display.print("Mov");
    display.setTextSize(2);
    display.setCursor(70, 22);
    display.print(String(ultimo.acel, 1) + "g");
    display.setTextSize(1);

    // Línea separadora
    display.drawLine(0, 37, 128, 37, SSD1306_WHITE);

    // Ubicación GPS con ícono
    display.drawXBitmap(0, 39, PIN_BITS, 8, 12, SSD1306_WHITE);
    display.setCursor(11, 40);
    if (ultimo.fix) display.print(String(ultimo.lat, 4) + " / " + String(ultimo.lon, 4));
    else            display.print("Sin ubicacion GPS");

    drawDots();  // Mostrar indicador de pantalla actual
  }

  display.display();  // Actualizar pantalla
}

/* ================================================================
 *  PANTALLA 4: DETALLES GPS (pantalla 1)
 *  Muestra: Coordenadas grandes, badge FIX/SIN FIX, satélites, SNR
 * ================================================================ */
void pant_gps() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawHdr("DETALLES GPS");

  if (ultimo.fix) {
    // Badge "GPS FIX" (fondo blanco, texto negro)
    display.fillRoundRect(90, 13, 37, 10, 2, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(93, 15); 
    display.print("GPS FIX");
    display.setTextColor(SSD1306_WHITE);
  } else {
    // Badge "SIN FIX" (solo marco, sin fondo)
    display.drawRoundRect(90, 13, 37, 10, 2, SSD1306_WHITE);
    display.setCursor(93, 15); 
    display.print("SIN FIX");
  }

  // === LATITUD (grande) ===
  display.setTextSize(1);
  display.setCursor(1, 14);
  display.print("LAT");
  display.setTextSize(1);
  display.setCursor(1, 23);
  if (ultimo.fix) display.print(String(ultimo.lat, 6));
  else            display.print("---.------");

  // === LONGITUD (grande) ===
  display.setCursor(1, 33);
  display.print("LON");
  display.setCursor(1, 42);
  if (ultimo.fix) display.print(String(ultimo.lon, 6));
  else            display.print("---.------");

  // Línea separadora
  display.drawLine(0, 51, 128, 51, SSD1306_WHITE);

  // === SATÉLITES (con barra) ===
  display.setCursor(1, 53);
  display.print("Sats:" + String(ultimo.sats));
  int pctSats = (ultimo.sats > 12) ? 100 : (ultimo.sats * 100) / 12;
  drawBarra(40, 52, 78, 5, pctSats);

  // === SNR (Relación Señal-Ruido) ===
  display.setCursor(1, 57);
  display.print("SNR:" + String(ultimo.snr, 1) + "dB");

  drawDots();  // Mostrar indicador de pantalla actual
  display.display();  // Actualizar pantalla
}

/* ================================================================
 *  PANTALLA 5: ESTADÍSTICAS (pantalla 2)
 *  Muestra: 3 mini-cards (RX / ERR / %OK) + barra de éxito
 * ================================================================ */
void pant_stats() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawHdr("ESTADISTICAS");

  // Calcular estadísticas
  int ok  = totalRecibidos - totalCorruptos;
  int pct = totalRecibidos > 0 ? (ok * 100) / totalRecibidos : 0;

  // Array de 3 mini-cards: RX, ERR, %OK
  struct { const char* lbl; int val; } cards[3] = {
    {"RX",  totalRecibidos},
    {"ERR", totalCorruptos},
    {"%OK", pct}
  };

  // Dibujar las 3 mini-cards horizontales
  for (int i = 0; i < 3; i++) {
    int x = i * 43;  // Espaciado: 43 píxeles entre inicio de cards
    
    // Marco redondeado
    display.drawRoundRect(x, 13, 41, 22, 2, SSD1306_WHITE);
    
    // Etiqueta centrada
    int lw = strlen(cards[i].lbl) * 6;
    display.setCursor(x + (41 - lw) / 2, 15);
    display.print(cards[i].lbl);
    
    // Valor centrado (tamaño 2)
    display.setTextSize(2);
    String vStr = String(cards[i].val);
    if (i == 2) vStr += "%";  // Agregar % al último card
    int vw = vStr.length() * 12;
    display.setCursor(x + (41 - vw) / 2, 22);
    display.print(vStr);
    display.setTextSize(1);
  }

  // Línea separadora
  display.drawLine(0, 37, 128, 37, SSD1306_WHITE);

  // Etiqueta de barra de éxito
  display.setCursor(1, 39);
  display.print("Tasa de exito:");
  drawBarra(0, 46, 128, 8, pct);

  // Resumen: paquetes OK / total
  display.setCursor(1, 56);
  display.print(String(ok) + "/" + String(totalRecibidos) + " paquetes OK");

  drawDots();  // Mostrar indicador de pantalla actual
  display.display();  // Actualizar pantalla
}

/* ================================================================
 *  PANTALLA 6: HISTORIAL (pantalla 3)
 *  Muestra: Últimos 5 paquetes con patrón zebra (filas alternas)
 * ================================================================ */
void pant_historial() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawHdr("HISTORIAL");

  int count = min(totalRecibidos, HIST_SIZE);

  if (count == 0) {
    // Si no hay datos, mostrar "Vacío"
    display.setCursor(35, 30);
    display.print("Vacio");
  } else {
    // Mostrar últimos count paquetes (hasta 5)
    for (int i = 0; i < count; i++) {
      // Calcular índice circular (más recientes primero)
      int idx = (totalRecibidos - 1 - i + HIST_SIZE) % HIST_SIZE;
      int y = 13 + i * 10;

      // Patrón zebra: filas pares con fondo blanco, texto negro
      if (i % 2 == 1) {
        display.fillRect(0, y, 128, 10, SSD1306_WHITE);  // Fondo blanco
        display.setTextColor(SSD1306_BLACK);
      } else {
        display.setTextColor(SSD1306_WHITE);
      }

      // Contenido de la fila: #N [TIPO] BPM [bpm] ACC [g]
      String lin = "#" + String(historial[idx].numero)
                   + " " + historial[idx].tipo
                   + " " + String(historial[idx].bpm) + "bpm"
                   + " " + String(historial[idx].acel, 1) + "g";
      display.setCursor(2, y + 1);
      display.print(lin);
    }
    display.setTextColor(SSD1306_WHITE);
  }

  drawDots();  // Mostrar indicador de pantalla actual
  display.display();  // Actualizar pantalla
}

/* ================================================================
 *  SELECTOR PRINCIPAL DE PANTALLA
 * ================================================================ */

/**
 * Elige qué pantalla mostrar basándose en el estado del sistema
 * Prioridad: 
 *   1. ESPERA (si no hay datos recibidos)
 *   2. SEÑAL PERDIDA (si se perdió conexión)
 *   3. Pantallas de datos (0-3) según pantActual
 */
void dibujarPantalla() {
  if (totalRecibidos == 0) { 
    pantEspera();      // Mostrar pantalla de espera si no hay datos
    return; 
  }
  if (senalPerdida) {
    pantSignalLost();  // Mostrar pantalla de señal perdida
    return; 
  }
  // Si hay datos válidos, mostrar pantalla seleccionada
  switch (pantActual) {
    case 0: pant_alerta();    break;  // Datos/Alerta
    case 1: pant_gps();       break;  // Detalles GPS
    case 2: pant_stats();     break;  // Estadísticas
    case 3: pant_historial(); break;  // Historial
  }
}

/* ================================================================
 *  LÓGICA DEL BUZZER (máquina de estados)
 * ================================================================ */

/**
 * Controla el buzzer con patrón de sonido inteligente
 * Máquina de estados con 4 fases:
 *   0. BEEP 1 ON (80ms)
 *   1. SILENCIO (80ms)
 *   2. BEEP 2 ON (80ms)
 *   3. PAUSA LARGA (2760ms)
 * Ciclo total: 3000ms (3 segundos)
 */
void checkBuzzerAlert() {
  // Condición 1: No hay datos o señal perdida → silencio absoluto
  if (totalRecibidos == 0 || senalPerdida) {
    buzzerOff();
    alertaActiva = false;
    return;
  }

  // Condición 2: Alerta expiró (30 segundos sin renovación)
  if (alertaActiva && millis() - tiempoAlerta > DURACION_ALERTA) {
    alertaActiva = false;
    buzzerOff();
    Serial.println(F("BUZZER: Timeout alerta, cancelando."));
  }

  // Si no hay alerta activa, apagar y retornar
  if (!alertaActiva) {
    buzzerOff();
    return;
  }

  // === MÁQUINA DE ESTADOS DEL BUZZER ===
  static uint8_t  estado     = 0;        // Estado actual (0-3)
  static unsigned long tEstado = 0;      // Timestamp del estado

  // Duraciones de cada estado en milisegundos
  const unsigned long duraciones[] = {
    80,    // estado 0: primer beep ON
    80,    // estado 1: silencio entre beeps
    80,    // estado 2: segundo beep ON
    2760   // estado 3: pausa larga (total ciclo = 3000ms)
  };

  // Salida (ON/OFF) para cada estado
  const bool sonido[] = {
    true,  // estado 0: ON
    false, // estado 1: OFF
    true,  // estado 2: ON
    false  // estado 3: OFF
  };

  // Transición a siguiente estado si pasó el tiempo
  if (millis() - tEstado >= duraciones[estado]) {
    tEstado = millis();
    estado = (estado + 1) % 4;  // Circular: 0→1→2→3→0
  }

  // Aplicar salida actual
  if (sonido[estado]) buzzerOn();
  else                buzzerOff();
}

/* ================================================================
 *  FUNCIONES PRINCIPALES DE ENTRADA
 * ================================================================ */

/**
 * Lee botón en GPIO 0 (botón de usuario)
 * Función: Cambiar pantalla actual (circular)
 * Debounce: 300ms
 */
void leerBoton() {
  bool btn = (digitalRead(0) == LOW);  // Botón activo en bajo
  
  if (btn && !btnPresionado && millis() - tBtn > 300) {
    btnPresionado = true;
    tBtn = millis();
    // Cambiar a siguiente pantalla (circular 0→1→2→3→0)
    pantActual = (pantActual + 1) % N_PANTALLAS;
    hayDato = true;  // Marcar para redibujar
  }
  if (!btn) btnPresionado = false;
}

/**
 * Recibe y procesa tramas LoRa de la pulsera
 * Validaciones:
 *   1. Longitud mínima (4 bytes encabezado)
 *   2. Dirección destino (debe ser dirLocal)
 *   3. Dirección origen (debe ser dirPulsera)
 *   4. Longitud del payload
 * Luego: Parsea JSON, actualiza estadísticas, reenvía por ESP-NOW
 */
void recibirLoRa() {
  byte buf[256];
  
  // Intentar recibir paquete LoRa
  if (radio.receive(buf, 0) != RADIOLIB_ERR_NONE) return;

  int len = radio.getPacketLength();
  
  // Validación 1: Longitud mínima
  if (len < 4) return;
  
  // Validación 2: Dirección destino correcta
  if (buf[0] != dirLocal)   return;
  
  // Validación 3: Dirección origen correcta
  if (buf[1] != dirPulsera) return;

  // Validación 4: Longitud del payload
  int  idPaquete = buf[2];
  byte payLen    = buf[3];
  if (len < (int)(4 + payLen)) return;

  // Extraer payload (JSON)
  String payload = "";
  for (int i = 4; i < 4 + payLen; i++) payload += (char)buf[i];

  Serial.println(F("──────────────────────────────"));
  Serial.print(F("RX: ")); 
  Serial.println(payload);

  // Crear estructura de evidencia
  Evidencia ev;
  ev.rssi   = radio.getRSSI();    // Potencia de señal recibida
  ev.snr    = radio.getSNR();     // Relación señal-ruido
  ev.numero = idPaquete;          // ID del paquete

  // Parsear JSON
  if (!parsearJSON(payload, ev)) {
    totalCorruptos++;  // Contar error de parseo
    Serial.println(F("Error Parseo"));
    return;
  }

  // Actualizar estado
  ultimo = ev;
  totalRecibidos++;
  hayDato = true;
  ultimoTiempoRX = millis();
  senalPerdida   = false;
  
  // Agregar al historial circular
  historial[(totalRecibidos - 1) % HIST_SIZE] = ev;

  // Lógica de alertas: activar buzzer si es pánico o caída
  if (ev.tipo == "PANICO" || ev.tipo == "CAIDA") {
    if (!alertaActiva) Serial.println(F("BUZZER: Alerta activada."));
    alertaActiva = true;
    tiempoAlerta = millis();
  } else {
    if (alertaActiva) Serial.println(F("BUZZER: Paquete NORMAL, alerta cancelada."));
    alertaActiva = false;
  }

  // Reenviar payload por ESP-NOW al gateway
  esp_now_send(macGateway, (uint8_t*)payload.c_str(), payload.length());

  // Log detallado en serial
  Serial.printf("OK #%d [%s] G:%.2f BPM:%d SpO2:%d\n",
    ev.numero, ev.tipo.c_str(), ev.acel, ev.bpm, ev.spo2);

  // Feedback visual: LED parpadea
  // Pánico: 5 parpadeos largos (120ms ON)
  // Normal: 2 parpadeos cortos (50ms ON)
  bool alerta = (ev.tipo == "PANICO" || ev.tipo == "CAIDA");
  int blinks = alerta ? 5 : 2;
  for (int k = 0; k < blinks; k++) {
    digitalWrite(LED_PIN, HIGH); 
    delay(alerta ? 120 : 50);
    digitalWrite(LED_PIN, LOW);  
    delay(50);
  }
}

/* ================================================================
 *  INICIALIZACIÓN (SETUP)
 * ================================================================ */

void setup() {
  // Inicializar puerto serial (debug)
  Serial.begin(115200);
  delay(500);

  // Configurar entradas/salidas digitales
  pinMode(0, INPUT_PULLUP);           // Botón de usuario
  pinMode(LED_PIN, OUTPUT);           // LED indicador
  digitalWrite(LED_PIN, LOW);         // Apagar LED
  pinMode(BUZZER_PIN, OUTPUT);        // Buzzer
  digitalWrite(BUZZER_PIN, LOW);      // Apagar buzzer
  buzzerOff();

  // Inicializar I2C (display OLED)
  Wire.begin(4, 15);                  // SDA=GPIO4, SCL=GPIO15
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);  delay(50);
  digitalWrite(OLED_RESET, HIGH); delay(50);

  // Inicializar display OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED ERROR"));
    while (true) delay(1000);  // Bloquear si falla
  }

  // Pantalla de arranque
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor((128 - 10 * 6) / 2, 10); 
  display.print("INICIANDO...");
  display.setCursor((128 - 15 * 6) / 2, 24); 
  display.print("INSTITUTO MUJER");
  display.drawLine(10, 35, 118, 35, SSD1306_WHITE);
  display.setCursor((128 - 14 * 6) / 2, 40); 
  display.print("915MHz | SF8");
  display.display();
  delay(1500);

  // Inicializar LoRa SX1276
  SPI.begin(5, 19, 27, 18);  // SCLK, MOSI, MISO, CS
  Serial.print(F("LoRa SX1276 915MHz... "));
  int st = radio.begin(FREQUENCY, BANDWIDTH, SPREAD_FACTOR, CODING_RATE);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setCRC(true);  // Habilitar CRC
    Serial.println(F("OK"));
  } else {
    Serial.printf("ERROR %d\n", st);
    display.clearDisplay();
    display.setCursor(0, 20);
    display.print("ERROR LORA: " + String(st));
    display.display();
    while (true) delay(1000);  // Bloquear si falla
  }

  // Inicializar WiFi para ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, macGateway, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.println(F("ESP-NOW listo"));
  } else {
    Serial.println(F("ESP-NOW ERROR"));
  }

  Serial.println(F("Escuchando..."));
  display.clearDisplay();
}

/* ================================================================
 *  CICLO PRINCIPAL (LOOP)
 * ================================================================ */

void loop() {
  // Leer entrada de usuario (cambiar pantalla)
  leerBoton();
  
  // Intentar recibir datos LoRa
  recibirLoRa();

  // === LÓGICA DE DETECCIÓN DE PÉRDIDA DE SEÑAL ===
  if (totalRecibidos > 0) {
    unsigned long sinSenal = millis() - ultimoTiempoRX;

    // Transición: No hay señal → activar pantalla de alerta (15s)
    if (!senalPerdida && sinSenal > TIMEOUT_SENAL) {
      senalPerdida = true;
      hayDato      = true;
      Serial.println(F("ALERTA: Senal perdida."));
    }

    // Transición: Demasiado tiempo sin señal → volver a modo ESPERA (15min)
    if (senalPerdida && sinSenal > TIMEOUT_SENAL + TIMEOUT_SENAL_PERDIDA) {
      senalPerdida   = false;
      totalRecibidos = 0;      // Reset para volver a pantalla ESPERA
      hayDato        = true;
      Serial.println(F("INFO: Volviendo a modo escucha."));
    }
  }

  // === ACTUALIZAR BUZZER ===
  checkBuzzerAlert();

  // === ACTUALIZAR PANTALLA ===
  // Redibujar si hay datos nuevos o cada 500ms (para animaciones)
  static unsigned long tRef = 0;
  if (hayDato || millis() - tRef > 500) {
    tRef    = millis();
    hayDato = false;
    dibujarPantalla();
  }

  // === FEEDBACK LED EN MODO ESPERA ===
  // Mientras está esperando, parpadear LED cada 2 segundos
  if (totalRecibidos == 0) {
    if (millis() - tBlink > 2000) {
      tBlink = millis();
      digitalWrite(LED_PIN, HIGH); delay(10);
      digitalWrite(LED_PIN, LOW);
    }
  }

  delay(10);  // Pequeña pausa para dar tiempo a otros procesos
}
