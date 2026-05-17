/**
 * Receptor LoRa - PANTALLAS OLED MEJORADAS
 * ─────────────────────────────────────────
 * Mejoras visuales en todas las pantallas:
 *  - Pantalla ESPERA      : arcos de señal + barra animada
 *  - Pantalla DATOS       : mini-cards BPM/SpO2, separadores
 *  - Pantalla PANICO      : inversión total de pantalla
 *  - Pantalla GPS         : coordenadas grandes, badge FIX, barra sats
 *  - Pantalla STATS       : 3 mini-cards + barra de éxito
 *  - Pantalla HISTORIAL   : filas alternas con número de paquete
 *  - Pantalla SENAL PERDIDA: ícono antena rota, tiempo MM:SS grande,
 *                            barra de progreso de timeout
 * Buzzer: solo en ALERTA. Silencio total en espera/datos normales.
 * ESP-NOW activo.
 */

#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

/* ================================================================
 *  CONFIGURACIÓN HARDWARE
 * ================================================================ */
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET     16
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

SX1276 radio = new Module(18, 26, 14, 35);

#define FREQUENCY      915.0
#define BANDWIDTH      125.0
#define SPREAD_FACTOR  8
#define CODING_RATE    5
#define LED_PIN        25
#define BUZZER_PIN     13

byte dirLocal   = 0xD3;
byte dirPulsera = 0xC1;

uint8_t macGateway[] = {0x80, 0x64, 0x6F, 0xFC, 0x0A, 0x50};

/* ================================================================
 *  ESTRUCTURA DE DATOS
 * ================================================================ */
struct Evidencia {
  String tipo;
  float  acel;
  int    bpm;
  int    spo2;
  double lat;
  double lon;
  int    sats;
  bool   fix;
  String ts;
  int    numero;
  float  rssi;
  float  snr;
  bool   valid;
};

#define HIST_SIZE 5
Evidencia historial[HIST_SIZE];
Evidencia ultimo;

int  totalRecibidos = 0;
int  totalCorruptos = 0;
bool hayDato        = false;

int  pantActual    = 0;
#define N_PANTALLAS 4
bool btnPresionado = false;
unsigned long tBtn   = 0;
unsigned long tBlink = 0;

unsigned long ultimoTiempoRX = 0;
const long    TIMEOUT_SENAL  = 15000;
//const long TIMEOUT_SENAL = 900000; // 15 minutos en milisegundos
const long TIMEOUT_SENAL_PERDIDA = 900000;

bool          senalPerdida   = false;

bool          alertaActiva   = false;
unsigned long tiempoAlerta   = 0;
const long    DURACION_ALERTA = 30000;

/* ================================================================
 *  ICONOS XBM (originales, usados como respaldo)
 * ================================================================ */
const uint8_t HEART_BITS[]  = { 0x36,0x00,0x7F,0x00,0xFF,0x00,0xFF,0x00,0x7E,0x00,0x3C,0x00,0x18,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
const uint8_t DROP_BITS[]   = { 0x0C,0x1E,0x3F,0x3F,0x3F,0x3F,0x3F,0x1E,0x0C,0x00,0x00,0x00 };
const uint8_t PIN_BITS[]    = { 0x3E,0x7F,0x7F,0x7F,0x7F,0x3E,0x1C,0x08,0x08,0x00,0x00,0x00 };
const uint8_t WAVE_BITS[]   = { 0x00,0x00,0x22,0x00,0x55,0x00,0x88,0x00,0x55,0x00,0x22,0x00,0x00,0x00,0x00,0x00 };
const uint8_t SIGNAL_BITS[] = { 0x00,0x00,0x10,0x00,0x38,0x00,0x7C,0x00,0x38,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };

/* ================================================================
 *  PARSING JSON
 * ================================================================ */
bool parsearJSON(const String &raw, Evidencia &ev) {
  auto extraerNum = [&](const String &key) -> String {
    String k = "\"" + key + "\":";
    int i = raw.indexOf(k);
    if (i < 0) return "0";
    i += k.length();
    int j = i;
    while (j < (int)raw.length() && raw[j] != ',' && raw[j] != '}' && raw[j] != ' ') j++;
    return raw.substring(i, j);
  };
  auto extraerStr = [&](const String &key) -> String {
    String k = "\"" + key + "\":\"";
    int i = raw.indexOf(k);
    if (i < 0) return "";
    i += k.length();
    int j = raw.indexOf("\"", i);
    return j < 0 ? "" : raw.substring(i, j);
  };

  ev.lat  = extraerNum("Lat").toDouble();
  ev.lon  = extraerNum("Lon").toDouble();
  ev.bpm  = extraerNum("BPM").toInt();
  ev.spo2 = extraerNum("SpO2").toInt();
  ev.acel = extraerNum("Mag").toFloat();
  String f = extraerStr("Fecha");
  String h = extraerStr("Hora");
  ev.ts   = (f.length() > 0 && h.length() > 0) ? f + "T" + h : "N/A";
  int sos = extraerNum("SOS").toInt();
  ev.tipo = (sos == 1) ? "PANICO" : "NORMAL";
  ev.fix  = (ev.lat != 0.0 || ev.lon != 0.0);
  ev.sats = ev.fix ? 8 : 0;
  ev.valid = true;
  return true;
}

/* ================================================================
 *  HELPERS DE PANTALLA
 * ================================================================ */

// Header blanco invertido (texto negro sobre fondo blanco)
void drawHdr(const char *titulo) {
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor((128 - (strlen(titulo) * 6)) / 2, 2);
  display.print(titulo);
  display.setTextColor(SSD1306_WHITE);
}

// Puntos de paginación en la parte inferior
void drawDots() {
  int dotW = 8, gap = 3;
  int startX = (128 - (N_PANTALLAS * dotW + (N_PANTALLAS - 1) * gap)) / 2;
  for (int i = 0; i < N_PANTALLAS; i++) {
    int x = startX + i * (dotW + gap);
    if (i == pantActual) display.fillRect(x, 61, dotW, 3, SSD1306_WHITE);
    else                 display.drawRect(x, 61, dotW, 3, SSD1306_WHITE);
  }
}

// Barra de progreso con borde redondeado
void drawBarra(int x, int y, int w, int h, int pct) {
  display.drawRoundRect(x, y, w, h, 1, SSD1306_WHITE);
  int fill = (pct * (w - 2)) / 100;
  if (fill > 0) display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
}

// Mini-card con etiqueta arriba y valor grande abajo
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
 *  ÍCONO ANTENA ROTA (para pantalla señal perdida)
 *  Técnica: drawCircle completo + fillRect negro para "borrar"
 *  la mitad inferior, simulando arcos superiores.
 * ================================================================ */
void dibujarAntenRota(int cx, int cy) {
  // Palo de antena
  display.drawLine(cx, cy - 14, cx, cy - 6, SSD1306_WHITE);
  // Base
  display.drawLine(cx - 5, cy - 6, cx + 5, cy - 6, SSD1306_WHITE);

  // Arco pequeño
  display.drawCircle(cx, cy - 4, 6, SSD1306_WHITE);
  display.fillRect(cx - 7, cy - 4, 15, 8, SSD1306_BLACK);

  // Arco mediano
  display.drawCircle(cx, cy - 4, 11, SSD1306_WHITE);
  display.fillRect(cx - 12, cy - 4, 25, 13, SSD1306_BLACK);

  // X de "sin señal" — esquina superior derecha del ícono
  display.drawLine(cx + 7, cy - 16, cx + 12, cy - 11, SSD1306_WHITE);
  display.drawLine(cx + 12, cy - 16, cx + 7, cy - 11, SSD1306_WHITE);
}

/* ================================================================
 *  FUNCIONES BUZZER
 * ================================================================ */
void buzzerOn()  { digitalWrite(BUZZER_PIN, HIGH); }
void buzzerOff() { digitalWrite(BUZZER_PIN, LOW);  }

/* ================================================================
 *  PANTALLA 1 — ESPERA
 *  Mejoras:
 *   - Arcos de señal LoRa centrados (con animación de punto)
 *   - Barra de "scanning" animada
 *   - Título + frecuencia más legibles
 * ================================================================ */
void pantEspera() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Título superior
  display.setTextSize(1);
  display.setCursor((128 - (15 * 6)) / 2, 1);
  display.print("INSTITUTO MUJER");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // Arcos de señal tipo WiFi, centrados
  int cx = 64, cy = 38;
  // Punto central (parpadeante)
  if (millis() % 1200 < 600) display.fillCircle(cx, cy, 2, SSD1306_WHITE);
  else                        display.drawCircle(cx, cy, 2, SSD1306_WHITE);

  // Arcos superiores — se dibujan incompletos con truco fillRect
  display.drawCircle(cx, cy, 7, SSD1306_WHITE);
  display.fillRect(cx - 8, cy, 17, 9, SSD1306_BLACK);   // borra mitad inferior arco 1

  display.drawCircle(cx, cy, 13, SSD1306_WHITE);
  display.fillRect(cx - 14, cy, 29, 15, SSD1306_BLACK);  // borra mitad inferior arco 2

  display.drawCircle(cx, cy, 19, SSD1306_WHITE);
  display.fillRect(cx - 20, cy, 41, 21, SSD1306_BLACK);  // borra mitad inferior arco 3

  // Barra animada de "escaneando"
  unsigned long t = millis() % 2000;
  int barPos = (t < 1000) ? (t * 110 / 1000) : ((2000 - t) * 110 / 1000);
  display.drawRoundRect(8, 54, 112, 4, 1, SSD1306_WHITE);
  display.fillRect(9 + barPos, 55, 8, 2, SSD1306_WHITE);

  // Info frecuencia
  display.setTextSize(1);
  display.setCursor(28, 46);
  display.print("915MHz | SF8 | BW125");

  display.display();
}

/* ================================================================
 *  PANTALLA 2 — SEÑAL PERDIDA
 *  Mejoras:
 *   - Ícono antena rota centrado
 *   - Tiempo transcurrido en formato MM:SS grande
 *   - Barra de progreso del timeout (15s)
 *   - Header parpadeante (ya existía, mejorado)
 *   - Línea separadora + último paquete en una sola línea
 * ================================================================ */
void pantSignalLost() {
  display.clearDisplay();

  // Header parpadeante
  bool blinking = (millis() % 800 < 400);
  if (blinking) {
    display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
    display.drawRect(0, 0, 128, 12, SSD1306_WHITE);
  }
  display.setTextSize(1);
  display.setCursor(14, 2);
  display.print("!! SENAL PERDIDA !!");
  display.setTextColor(SSD1306_WHITE);

  // Ícono antena rota (centrado horizontalmente, zona media-izquierda)
  dibujarAntenRota(24, 38);

  // Tiempo transcurrido grande — formato MM:SS
  unsigned long segs = (millis() - ultimoTiempoRX) / 1000;
  char buf[8];
  sprintf(buf, "%02lu:%02lu", segs / 60, segs % 60);
  display.setTextSize(2);
  int txtW = strlen(buf) * 12;
  display.setCursor(64 + (64 - txtW) / 2, 18);  // centrado en mitad derecha
  display.print(buf);
  display.setTextSize(1);

  // Subtítulo "sin datos"
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(50, 35);
  display.print("sin datos");

  // Separador
  display.drawLine(4, 45, 124, 45, SSD1306_WHITE);

  // Último paquete recibido
  display.setCursor(2, 48);
  display.print("Ult:#" + String(ultimo.numero) + " | " + ultimo.tipo);

  // Barra de progreso del timeout
  // Llena de izquierda a derecha hasta que se alcanza TIMEOUT_SENAL
  int pctTimeout = min(100, (int)((millis() - ultimoTiempoRX) * 100L / TIMEOUT_SENAL));
  display.drawRoundRect(2, 57, 124, 4, 1, SSD1306_WHITE);
  int fill = (pctTimeout * 122) / 100;
  if (fill > 0) display.fillRect(3, 58, fill, 2, SSD1306_WHITE);

  display.display();
}

/* ================================================================
 *  PANTALLA 3 — DATOS / ALERTA (pantalla 0)
 *  Mejoras:
 *   - NORMAL: mini-cards BPM y SpO2, separadores, layout 2 cols
 *   - PANICO: inversión TOTAL de pantalla (blanco completo)
 *   - Número de paquete en esquina inferior derecha
 * ================================================================ */
/* ================================================================
 *  PANTALLA DATOS / ALERTA  — reemplaza pant_alerta() completa
 * ================================================================ */
void pant_alerta() {
  bool esAlerta = (ultimo.tipo == "PANICO" || ultimo.tipo == "CAIDA");
  display.clearDisplay();

  if (esAlerta) {

    // Header — solo él parpadea
    static bool inv = false;
    static unsigned long tI = 0;
    if (millis() - tI > 500) { inv = !inv; tI = millis(); }

    if (inv) {
      display.fillRect(0, 0, 128, 13, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.drawRect(0, 0, 128, 13, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }
    display.setTextSize(1);
    display.setCursor((128 - (strlen(ultimo.tipo.c_str()) * 6 + 6)) / 2, 3);
    display.print("! " + ultimo.tipo + " !");

    // Resto siempre legible — blanco sobre negro
    display.setTextColor(SSD1306_WHITE);

    // Card BPM
    display.drawRoundRect(1, 15, 60, 26, 2, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(5, 17);
    display.print("BPM");
    display.setTextSize(2);
    display.setCursor(5, 25);
    display.print(String(ultimo.bpm));
    display.setTextSize(1);

    // Card Acelerometro
    display.drawRoundRect(66, 15, 61, 26, 2, SSD1306_WHITE);
    display.setCursor(70, 17);
    display.print("Impacto");
    display.setTextSize(2);
    display.setCursor(70, 25);
    display.print(String(ultimo.acel, 1) + "g");
    display.setTextSize(1);

    // Separador
    display.drawLine(0, 43, 128, 43, SSD1306_WHITE);

    // GPS
    display.setCursor(2, 46);
    if (ultimo.fix) {
      display.print(String(ultimo.lat, 5));
      display.setCursor(2, 55);
      display.print(String(ultimo.lon, 5));
    } else {
      display.setCursor(20, 50);
      display.print("Sin ubicacion GPS");
    }

    drawDots();

  } else {

    // Datos normales
    display.setTextColor(SSD1306_WHITE);
    drawHdr("DATOS RECIBIDOS");

    // Card BPM
    display.drawRoundRect(1, 13, 60, 22, 2, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(5, 15);
    display.print("BPM");
    display.setTextSize(2);
    display.setCursor(5, 22);
    display.print(String(ultimo.bpm));
    display.setTextSize(1);

    // Card Acelerometro
    display.drawRoundRect(66, 13, 61, 22, 2, SSD1306_WHITE);
    display.setCursor(70, 15);
    display.print("Mov");
    display.setTextSize(2);
    display.setCursor(70, 22);
    display.print(String(ultimo.acel, 1) + "g");
    display.setTextSize(1);

    // Separador
    display.drawLine(0, 37, 128, 37, SSD1306_WHITE);

    // GPS
    display.drawXBitmap(0, 39, PIN_BITS, 8, 12, SSD1306_WHITE);
    display.setCursor(11, 40);
    if (ultimo.fix) display.print(String(ultimo.lat, 4) + " / " + String(ultimo.lon, 4));
    else            display.print("Sin ubicacion GPS");

    drawDots();
  }

  display.display();
}



/* ================================================================
 *  PANTALLA 4 — GPS (pantalla 1)
 *  Mejoras:
 *   - Coordenadas en texto grande (tamaño 2)
 *   - Badge "GPS FIX" / "SIN FIX" invertido
 *   - Barra de satélites con porcentaje
 *   - SNR añadido
 * ================================================================ */
void pant_gps() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawHdr("DETALLES GPS");

  if (ultimo.fix) {
    // Badge FIX (caja blanca, texto negro)
    display.fillRoundRect(90, 13, 37, 10, 2, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(93, 15); display.print("GPS FIX");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.drawRoundRect(90, 13, 37, 10, 2, SSD1306_WHITE);
    display.setCursor(93, 15); display.print("SIN FIX");
  }

  // Coordenadas grandes
  display.setTextSize(1);
  display.setCursor(1, 14);
  display.print("LAT");
  display.setTextSize(1);
  display.setCursor(1, 23);
  if (ultimo.fix) display.print(String(ultimo.lat, 6));
  else            display.print("---.------");

  display.setCursor(1, 33);
  display.print("LON");
  display.setCursor(1, 42);
  if (ultimo.fix) display.print(String(ultimo.lon, 6));
  else            display.print("---.------");

  // Separador
  display.drawLine(0, 51, 128, 51, SSD1306_WHITE);

  // Barra de satélites
  display.setCursor(1, 53);
  display.print("Sats:" + String(ultimo.sats));
  int pctSats = (ultimo.sats > 12) ? 100 : (ultimo.sats * 100) / 12;
  drawBarra(40, 52, 78, 5, pctSats);

  // SNR
  display.setCursor(1, 57);
  display.print("SNR:" + String(ultimo.snr, 1) + "dB");

  drawDots();
  display.display();
}

/* ================================================================
 *  PANTALLA 5 — ESTADÍSTICAS (pantalla 2)
 *  Mejoras:
 *   - 3 mini-cards horizontales: RX / ERR / %OK
 *   - Barra de éxito con etiqueta
 *   - Línea separadora
 * ================================================================ */
void pant_stats() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawHdr("ESTADISTICAS");

  int ok  = totalRecibidos - totalCorruptos;
  int pct = totalRecibidos > 0 ? (ok * 100) / totalRecibidos : 0;

  // 3 mini-cards: ancho ~41px cada una
  struct { const char* lbl; int val; } cards[3] = {
    {"RX",  totalRecibidos},
    {"ERR", totalCorruptos},
    {"%OK", pct}
  };

  for (int i = 0; i < 3; i++) {
    int x = i * 43;
    display.drawRoundRect(x, 13, 41, 22, 2, SSD1306_WHITE);
    // Etiqueta centrada
    int lw = strlen(cards[i].lbl) * 6;
    display.setCursor(x + (41 - lw) / 2, 15);
    display.print(cards[i].lbl);
    // Valor centrado, tamaño 2
    display.setTextSize(2);
    String vStr = String(cards[i].val);
    if (i == 2) vStr += "%";
    int vw = vStr.length() * 12;
    display.setCursor(x + (41 - vw) / 2, 22);
    display.print(vStr);
    display.setTextSize(1);
  }

  // Separador
  display.drawLine(0, 37, 128, 37, SSD1306_WHITE);

  // Etiqueta + barra de éxito
  display.setCursor(1, 39);
  display.print("Tasa de exito:");
  drawBarra(0, 46, 128, 8, pct);

  // Paquetes OK / total debajo de la barra
  display.setCursor(1, 56);
  display.print(String(ok) + "/" + String(totalRecibidos) + " paquetes OK");

  drawDots();
  display.display();
}

/* ================================================================
 *  PANTALLA 6 — HISTORIAL (pantalla 3)
 *  Mejoras:
 *   - Filas alternas con fondo (zebra) para mayor legibilidad
 *   - Tipo de evento resaltado
 *   - Número de paquete visible
 * ================================================================ */
void pant_historial() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawHdr("HISTORIAL");

  int count = min(totalRecibidos, HIST_SIZE);

  if (count == 0) {
    display.setCursor(35, 30);
    display.print("Vacio");
  } else {
    for (int i = 0; i < count; i++) {
      int idx = (totalRecibidos - 1 - i + HIST_SIZE) % HIST_SIZE;
      int y = 13 + i * 10;

      // Fila alterna (zebra): filas impares con fondo
      if (i % 2 == 1) {
        display.fillRect(0, y, 128, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
      } else {
        display.setTextColor(SSD1306_WHITE);
      }

      // Contenido de la fila
      String lin = "#" + String(historial[idx].numero)
                   + " " + historial[idx].tipo
                   + " " + String(historial[idx].bpm) + "bpm"
                   + " " + String(historial[idx].acel, 1) + "g";
      display.setCursor(2, y + 1);
      display.print(lin);
    }
    display.setTextColor(SSD1306_WHITE);
  }

  drawDots();
  display.display();
}

/* ================================================================
 *  SELECTOR DE PANTALLA
 * ================================================================ */
void dibujarPantalla() {
  if (totalRecibidos == 0) { pantEspera();      return; }
  if (senalPerdida)        { pantSignalLost();  return; }
  switch (pantActual) {
    case 0: pant_alerta();    break;
    case 1: pant_gps();       break;
    case 2: pant_stats();     break;
    case 3: pant_historial(); break;
  }
}

/* ================================================================
 *  LÓGICA BUZZER
 * ================================================================ */
void checkBuzzerAlert() {

  // Silencio absoluto si no hay datos o señal perdida
  if (totalRecibidos == 0 || senalPerdida) {
    buzzerOff();
    alertaActiva = false;
    return;
  }

  // Auto-cancelar si pasaron 30s sin recibir otro paquete de alerta
  if (alertaActiva && millis() - tiempoAlerta > DURACION_ALERTA) {
    alertaActiva = false;
    buzzerOff();
    Serial.println(F("BUZZER: Timeout alerta, cancelando."));
  }

  if (!alertaActiva) {
    buzzerOff();
    return;
  }

  // Máquina de estados del buzzer
  // Estados: 0=BEEP1 ON, 1=BEEP1 OFF, 2=BEEP2 ON, 3=BEEP2 OFF (pausa larga)
  static uint8_t  estado     = 0;
  static unsigned long tEstado = 0;

  // Duraciones de cada estado en ms
  const unsigned long duraciones[] = {
    80,    // estado 0: primer beep ON
    80,    // estado 1: silencio entre beeps
    80,    // estado 2: segundo beep ON
    2760   // estado 3: pausa larga (total ciclo = 3000ms)
  };

  const bool sonido[] = {
    true,  // estado 0: ON
    false, // estado 1: OFF
    true,  // estado 2: ON
    false  // estado 3: OFF
  };

  // Avanzar estado si ya pasó el tiempo
  if (millis() - tEstado >= duraciones[estado]) {
    tEstado = millis();
    estado = (estado + 1) % 4;
  }

  // Aplicar salida
  if (sonido[estado]) buzzerOn();
  else                buzzerOff();
}

/* ================================================================
 *  LÓGICA PRINCIPAL
 * ================================================================ */
void leerBoton() {
  bool btn = (digitalRead(0) == LOW);
  if (btn && !btnPresionado && millis() - tBtn > 300) {
    btnPresionado = true;
    tBtn = millis();
    pantActual = (pantActual + 1) % N_PANTALLAS;
    hayDato = true;
  }
  if (!btn) btnPresionado = false;
}

void recibirLoRa() {
  byte buf[256];
  if (radio.receive(buf, 0) != RADIOLIB_ERR_NONE) return;

  int len = radio.getPacketLength();
  if (len < 4) return;
  if (buf[0] != dirLocal)   return;
  if (buf[1] != dirPulsera) return;

  int  idPaquete = buf[2];
  byte payLen    = buf[3];
  if (len < (int)(4 + payLen)) return;

  String payload = "";
  for (int i = 4; i < 4 + payLen; i++) payload += (char)buf[i];

  Serial.println(F("──────────────────────────────"));
  Serial.print(F("RX: ")); Serial.println(payload);

  Evidencia ev;
  ev.rssi   = radio.getRSSI();
  ev.snr    = radio.getSNR();
  ev.numero = idPaquete;

  if (!parsearJSON(payload, ev)) {
    totalCorruptos++;
    Serial.println(F("Error Parseo"));
    return;
  }

  ultimo = ev;
  totalRecibidos++;
  hayDato = true;
  ultimoTiempoRX = millis();
  senalPerdida   = false;
  historial[(totalRecibidos - 1) % HIST_SIZE] = ev;

  if (ev.tipo == "PANICO" || ev.tipo == "CAIDA") {
    if (!alertaActiva) Serial.println(F("BUZZER: Alerta activada."));
    alertaActiva = true;
    tiempoAlerta = millis();
  } else {
    if (alertaActiva) Serial.println(F("BUZZER: Paquete NORMAL, alerta cancelada."));
    alertaActiva = false;
  }

  esp_now_send(macGateway, (uint8_t*)payload.c_str(), payload.length());

  Serial.printf("OK #%d [%s] G:%.2f BPM:%d SpO2:%d\n",
    ev.numero, ev.tipo.c_str(), ev.acel, ev.bpm, ev.spo2);

  bool alerta = (ev.tipo == "PANICO" || ev.tipo == "CAIDA");
  int blinks = alerta ? 5 : 2;
  for (int k = 0; k < blinks; k++) {
    digitalWrite(LED_PIN, HIGH); delay(alerta ? 120 : 50);
    digitalWrite(LED_PIN, LOW);  delay(50);
  }
}

/* ================================================================
 *  SETUP
 * ================================================================ */
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(0, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  buzzerOff();

  Wire.begin(4, 15);
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);  delay(50);
  digitalWrite(OLED_RESET, HIGH); delay(50);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED ERROR"));
    while (true) delay(1000);
  }

  // Pantalla de arranque
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor((128 - 10 * 6) / 2, 10); display.print("INICIANDO...");
  display.setCursor((128 - 15 * 6) / 2, 24); display.print("INSTITUTO MUJER");
  display.drawLine(10, 35, 118, 35, SSD1306_WHITE);
  display.setCursor((128 - 14 * 6) / 2, 40); display.print("915MHz | SF8");
  display.display();
  delay(1500);

  SPI.begin(5, 19, 27, 18);
  Serial.print(F("LoRa SX1276 915MHz... "));
  int st = radio.begin(FREQUENCY, BANDWIDTH, SPREAD_FACTOR, CODING_RATE);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setCRC(true);
    Serial.println(F("OK"));
  } else {
    Serial.printf("ERROR %d\n", st);
    display.clearDisplay();
    display.setCursor(0, 20);
    display.print("ERROR LORA: " + String(st));
    display.display();
    while (true) delay(1000);
  }

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
 *  LOOP
 * ================================================================ */
void loop() {
  leerBoton();
  recibirLoRa();

  if (totalRecibidos > 0) {
    unsigned long sinSenal = millis() - ultimoTiempoRX;

    if (!senalPerdida && sinSenal > TIMEOUT_SENAL) {
      // Acaba de perder señal — activa la pantalla
      senalPerdida = true;
      hayDato      = true;
      Serial.println(F("ALERTA: Senal perdida."));
    }

    if (senalPerdida && sinSenal > TIMEOUT_SENAL + TIMEOUT_SENAL_PERDIDA) {
      // Ya estuvo 15 min en pantalla de señal perdida — regresa a escuchar
      senalPerdida   = false;
      totalRecibidos = 0;   // regresa a pantalla ESPERA como si acabara de arrancar
      hayDato        = true;
      Serial.println(F("INFO: Volviendo a modo escucha."));
    }
  }


  checkBuzzerAlert();

  static unsigned long tRef = 0;
  if (hayDato || millis() - tRef > 500) {
    tRef    = millis();
    hayDato = false;
    dibujarPantalla();
  }

  if (totalRecibidos == 0) {
    if (millis() - tBlink > 2000) {
      tBlink = millis();
      digitalWrite(LED_PIN, HIGH); delay(10);
      digitalWrite(LED_PIN, LOW);
    }
  }

  delay(10);
}
