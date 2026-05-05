/**
 * Receptor LoRa - FINAL ESTABLE
 * - Buzzer: Suena SOLO en ALERTA (PANICO/CAIDA).
 * - Silencio TOTAL en ESPERA y datos normales.
 * - ESP-NOW activo.
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

int pantActual = 0;
#define N_PANTALLAS 4
bool btnPresionado = false;
unsigned long tBtn   = 0;
unsigned long tBlink = 0;

unsigned long ultimoTiempoRX = 0;
const long    TIMEOUT_SENAL  = 15000;
bool          senalPerdida   = false;

// ── BUZZER: variables nuevas ──────────────────────────────────────
bool          alertaActiva  = false;
unsigned long tiempoAlerta  = 0;
const long    DURACION_ALERTA = 30000; // 30s sin nuevo PANICO → cancela
// ─────────────────────────────────────────────────────────────────

/* ================================================================
 *  ICONOS XBM
 * ================================================================ */
const uint8_t HEART_BITS[]  = { 0x36, 0x00, 0x7F, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x7E, 0x00, 0x3C, 0x00, 0x18, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t DROP_BITS[]   = { 0x0C, 0x1E, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1E, 0x0C, 0x00, 0x00, 0x00 };
const uint8_t PIN_BITS[]    = { 0x3E, 0x7F, 0x7F, 0x7F, 0x7F, 0x3E, 0x1C, 0x08, 0x08, 0x00, 0x00, 0x00 };
const uint8_t WAVE_BITS[]   = { 0x00, 0x00, 0x22, 0x00, 0x55, 0x00, 0x88, 0x00, 0x55, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t SIGNAL_BITS[] = { 0x00, 0x00, 0x10, 0x00, 0x38, 0x00, 0x7C, 0x00, 0x38, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

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
  if (f.length() > 0 && h.length() > 0) ev.ts = f + "T" + h;
  else ev.ts = "N/A";
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
void drawHdr(const char *titulo) {
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor((128 - (strlen(titulo) * 6)) / 2, 2);
  display.print(titulo);
  display.setTextColor(SSD1306_WHITE);
  display.drawLine(0, 12, 127, 12, SSD1306_WHITE);
}

void drawDots() {
  int dotW = 6, gap = 4;
  int startX = (128 - (N_PANTALLAS * dotW + (N_PANTALLAS - 1) * gap)) / 2;
  for (int i = 0; i < N_PANTALLAS; i++) {
    int x = startX + i * (dotW + gap);
    if (i == pantActual) display.fillRect(x, 61, dotW, 3, SSD1306_WHITE);
    else                 display.drawRect(x, 61, dotW, 3, SSD1306_WHITE);
  }
}

void drawBarra(int x, int y, int w, int h, int pct) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int fill = (pct * (w - 2)) / 100;
  if (fill > 0) display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
}

/* ================================================================
 *  FUNCIONES BUZZER
 * ================================================================ */
void buzzerOn()  { digitalWrite(BUZZER_PIN, HIGH); } // HIGH = suena
void buzzerOff() { digitalWrite(BUZZER_PIN, LOW);  } // LOW  = silencio
/* ================================================================
 *  PANTALLAS
 * ================================================================ */
void pantEspera() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20, 5); display.print("INSTITUTO MUJER");
  display.drawLine(0, 15, 128, 15, SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(20, 25); display.print("ESPERA...");
  display.setTextSize(1);
  display.setCursor(0, 50); display.print("915MHz SF8");
  if (millis() % 2000 < 1000) display.fillCircle(120, 55, 3, SSD1306_WHITE);
  else                        display.drawCircle(120, 55, 3, SSD1306_WHITE);
  display.display();
}

void pantSignalLost() {
  display.clearDisplay();
  bool blinking = (millis() % 800 < 400);
  if (blinking) {
    display.fillRect(0, 0, 128, 13, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
    display.drawRect(0, 0, 128, 13, SSD1306_WHITE);
  }
  display.setTextSize(1);
  display.setCursor(16, 3); display.print("! SENAL PERDIDA !");
  display.setTextColor(SSD1306_WHITE);
  display.drawLine(0, 13, 128, 13, SSD1306_WHITE);
  display.setCursor(6, 18); display.print("Sin datos de pulsera");
  unsigned long segs = (millis() - ultimoTiempoRX) / 1000;
  unsigned long mins = segs / 60;
  unsigned long secs = segs % 60;
  display.setCursor(6, 30);
  display.print("Hace: ");
  if (mins > 0) { display.print(mins); display.print("m "); display.print(secs); display.print("s"); }
  else          { display.print(secs); display.print("s"); }
  display.setCursor(6, 42); display.print("Ult paquete: #"); display.print(ultimo.numero);
  display.setCursor(6, 54); display.print("915MHz SF8 | OK");
  display.display();
}

void pant_alerta() {
  bool esAlerta = (ultimo.tipo == "PANICO" || ultimo.tipo == "CAIDA");
  display.clearDisplay();

  if (esAlerta) {
    static bool inv = false;
    static unsigned long tI = 0;
    if (millis() - tI > 400) { inv = !inv; tI = millis(); }
    if (inv) {
      display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setTextSize(1);
    display.setCursor(2, 2);
    display.print("!! " + ultimo.tipo + " !!");
    display.setTextColor(SSD1306_WHITE);
  } else {
    drawHdr("DATOS RECIBIDOS");
  }

  display.setTextSize(1);
  display.drawXBitmap(0, 15, HEART_BITS, 12, 12, SSD1306_WHITE);
  display.setCursor(14, 16); display.print(String(ultimo.bpm) + " BPM");
  display.drawXBitmap(64, 15, DROP_BITS, 8, 12, SSD1306_WHITE);
  display.setCursor(76, 16); display.print(String(ultimo.spo2) + "%");
  display.drawXBitmap(0, 28, WAVE_BITS, 10, 8, SSD1306_WHITE);
  display.setCursor(14, 29); display.print("G:" + String(ultimo.acel, 2));
  display.drawXBitmap(75, 28, SIGNAL_BITS, 10, 10, SSD1306_WHITE);
  display.setCursor(87, 29); display.print(String((int)ultimo.rssi) + "dB");
  display.drawXBitmap(0, 41, PIN_BITS, 8, 12, SSD1306_WHITE);
  display.setCursor(12, 42);
  if (ultimo.fix) display.print("Lat:" + String(ultimo.lat, 5));
  else            display.print("Lat: --");
  display.setCursor(12, 54);
  if (ultimo.fix) display.print("Lon:" + String(ultimo.lon, 5));
  else            display.print("Lon: --");

  drawDots();
  display.display();
}

void pant_gps() {
  display.clearDisplay();
  drawHdr("DETALLES GPS");
  display.setTextSize(1);
  display.drawXBitmap(0, 16, PIN_BITS, 8, 12, SSD1306_WHITE);
  display.setCursor(12, 16); display.print("Sats: " + String(ultimo.sats));
  int bw = (ultimo.sats > 12) ? 110 : (ultimo.sats * 110) / 12;
  display.drawRect(0, 28, 128, 6, SSD1306_WHITE);
  if (bw > 0) display.fillRect(1, 29, bw, 4, SSD1306_WHITE);
  if (ultimo.fix) {
    display.setCursor(0, 38); display.print("Lat: " + String(ultimo.lat, 5));
    display.setCursor(0, 50); display.print("Lon: " + String(ultimo.lon, 5));
  } else {
    display.setCursor(30, 42); display.print("Sin fix GPS");
  }
  drawDots();
  display.display();
}

void pant_stats() {
  display.clearDisplay();
  drawHdr("ESTADISTICAS");
  display.setTextSize(1);
  int ok  = totalRecibidos - totalCorruptos;
  int pct = totalRecibidos > 0 ? (ok * 100) / totalRecibidos : 0;
  display.setCursor(0, 16); display.print("Recibidos: " + String(totalRecibidos));
  display.setCursor(0, 28); display.print("Perdidos : " + String(totalCorruptos));
  display.setCursor(0, 42); display.print("Tasa Exito: " + String(pct) + "%");
  drawBarra(0, 54, 128, 8, pct);
  drawDots();
  display.display();
}

void pant_historial() {
  display.clearDisplay();
  drawHdr("HISTORIAL");
  display.setTextSize(1);
  int count = min(totalRecibidos, HIST_SIZE);
  for (int i = 0; i < count; i++) {
    int idx = (totalRecibidos - 1 - i + HIST_SIZE) % HIST_SIZE;
    String lin = "#" + String(historial[idx].numero) + " " + historial[idx].tipo + " " + String(historial[idx].acel, 1) + "g";
    display.setCursor(0, 14 + i * 10); display.print(lin);
  }
  if (count == 0) { display.setCursor(35, 30); display.print("Vacio"); }
  drawDots();
  display.display();
}

void dibujarPantalla() {
  if (totalRecibidos == 0) { pantEspera();     return; }
  if (senalPerdida)        { pantSignalLost(); return; }
  switch (pantActual) {
    case 0: pant_alerta();    break;
    case 1: pant_gps();       break;
    case 2: pant_stats();     break;
    case 3: pant_historial(); break;
  }
}

/* ================================================================
 *  LOGICA BUZZER — CORREGIDA
 * ================================================================ */
void checkBuzzerAlert() {
  // Condiciones de silencio absoluto
  if (totalRecibidos == 0 || senalPerdida) {
    buzzerOff();
    alertaActiva = false;
    return;
  }

  // Auto-cancelar si pasaron 30s sin recibir otro paquete de alerta
  if (alertaActiva && millis() - tiempoAlerta > DURACION_ALERTA) {
    alertaActiva = false;
    Serial.println(F("BUZZER: Timeout alerta, cancelando."));
  }

  if (!alertaActiva) {
    buzzerOff();
    return;
  }

  // Dip-Dip cada 5 segundos
  unsigned long cycle = millis() % 5000;
  if      (cycle < 100) buzzerOn();
  else if (cycle < 200) buzzerOff();
  else if (cycle < 300) buzzerOn();
  else                  buzzerOff();
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

  // ── Actualizar flag de alerta ─────────────────────────────────
  if (ev.tipo == "PANICO" || ev.tipo == "CAIDA") {
    if (!alertaActiva) Serial.println(F("BUZZER: Alerta activada."));
    alertaActiva = true;
    tiempoAlerta = millis(); // Reinicia el timeout de 30s
  } else {
    if (alertaActiva) Serial.println(F("BUZZER: Paquete NORMAL, alerta cancelada."));
    alertaActiva = false;   // Un paquete NORMAL cancela inmediatamente
  }
  // ─────────────────────────────────────────────────────────────

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


void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(0, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

 pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // silencio desde el primer microsegundo
  buzzerOff();

  Wire.begin(4, 15);
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);  delay(50);
  digitalWrite(OLED_RESET, HIGH); delay(50);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED ERROR"));
    while (true) delay(1000);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(30, 20); display.print("INICIANDO...");
  display.display();
  delay(1000);

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

void loop() {
  leerBoton();
  recibirLoRa();

  if (totalRecibidos > 0) {
    if (millis() - ultimoTiempoRX > TIMEOUT_SENAL) {
      if (!senalPerdida) {
        senalPerdida = true;
        hayDato      = true;
        Serial.println(F("ALERTA: Senal perdida (Timeout)"));
      }
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
