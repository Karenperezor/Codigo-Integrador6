/**
 * CODIGO FINAL
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
 *  INSTANCIAS Y PINES
 * ================================================================ */

SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
SX1262 radio = new Module(8, 14, 12, 13);

#define FREQUENCY     915.0
#define BANDWIDTH     125.0
#define SPREAD_FACTOR 8
#define CODING_RATE   5
#define TX_POWER      22

byte dirLocal   = 0xC1;
byte dirDestino = 0xD3;
byte idMsg      = 0;
byte bufTx[256];

// Bus I2C compartido (Controlador 1)
TwoWire busI2C = TwoWire(1);

// Pines de cada sensor
#define MPU_SDA 41
#define MPU_SCL 42
#define RTC_SDA 38
#define RTC_SCL 39
#define MAX_SDA  1
#define MAX_SCL  2

MPU6050   sensor(0x68, &busI2C);
RTC_DS3231 rtc;
MAX30105  particleSensor;

bool mpuOK = false;
bool rtcOK = false;
bool maxOK = false;
#define FORZAR_AJUSTE false

// --- Variables MPU6050 (Acelerómetro y Giroscopio) ---
float ax_g = 0, ay_g = 0, az_g = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;
float mag = 1.0;
DateTime ultimaHora;
const float LSB_POR_G    = 16384.0f;
const float LSB_POR_GYRO =   131.0f;

// --- Umbrales de Alerta ---
const float UMBRAL_G_FORCEJEO = 2.0f;
const float UMBRAL_G_CAIDA    = 3.5f;
const int   UMBRAL_BPM_ALTO   = 110;
const int   UMBRAL_SPO2_BAJO  =  92;

// --- Variables MAX30102 ---
const byte RATE_SIZE = 4;
byte  rates[RATE_SIZE];
byte  rateSpot = 0;
long  lastBeat = 0;
float beatsPerMinute = 0;
int   beatAvg = 0;
int   spo2    = 0;

HardwareSerial neogps(1);
TinyGPSPlus    gps;
float lat = 0, lon = 0;
bool  gpsValido   = false;
bool  sincronizado = false;

enum Estado { REPOSO, PANICO };
Estado estadoActual = REPOSO;
unsigned long tPanico      = 0;
unsigned long tUltimoEnvio = 0;
int  numEvidencia  = 0;
String tipoMovimiento = "REPOSO";

#define DURACION_PANICO 3600000UL
#define INTERVALO_ENVIO    2000UL

#define BTN_PANICO 0
bool          botonPresionado = false;
unsigned long tBoton          = 0;

bool          enAlerta    = false;
unsigned long tiempoAlerta = 0;
String        tipoAlerta  = "";
float         magAlerta   = 0;
const unsigned long DUR_ALERTA = 3000;

// --- Variables para animación visual ---
bool blinkState = false;
unsigned long lastBlink = 0;

/* ================================================================
 *  ICONOS XBM (Diseño gráfico proporcionado)
 * ================================================================ */

// ❤ CORAZÓN 12x12
static const uint8_t HEART_BITS[] = {
  0x36, 0x00, 0x7F, 0x00, 0xFF, 0x00, 0xFF, 0x00,
  0x7E, 0x00, 0x3C, 0x00, 0x18, 0x00, 0x08, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// GOTA 8x10
static const uint8_t DROP_BITS[] = {
  0x0C, 0x1E, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1E,
  0x0C, 0x00, 0x00, 0x00,
};

// RELOJ 10x10
static const uint8_t CLOCK_BITS[] = {
  0x7C, 0x00, 0x82, 0x00, 0x9A, 0x00, 0xBA, 0x00,
  0x9A, 0x00, 0x82, 0x00, 0x82, 0x00, 0x7C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// PIN MAPA 8x12
static const uint8_t PIN_BITS[] = {
  0x3E, 0x7F, 0x7F, 0x7F, 0x7F, 0x3E, 0x1C, 0x08,
  0x08, 0x00, 0x00, 0x00,
};

// ≈ ONDA (acelerómetro) 10x8
static const uint8_t WAVE_BITS[] = {
  0x00, 0x00, 0x22, 0x00, 0x55, 0x00, 0x88, 0x00,
  0x55, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* ================================================================
 *  GESTIÓN DE BUS I2C
 * ================================================================ */

int active_sda = -1;
int active_scl = -1;

void cambiarBusI2C(int nuevo_sda, int nuevo_scl) {
  if (active_sda == nuevo_sda && active_scl == nuevo_scl) return;
  busI2C.end();
  delay(1);
  if (active_sda != -1) {
    pinMode(active_sda, INPUT);
    pinMode(active_scl, INPUT);
  }
  delay(5);
  busI2C.begin(nuevo_sda, nuevo_scl);
  busI2C.setClock(50000);
  busI2C.setTimeOut(50);
  active_sda = nuevo_sda;
  active_scl = nuevo_scl;
}

void seleccionarMPU() { cambiarBusI2C(MPU_SDA, MPU_SCL); }
void seleccionarRTC() { cambiarBusI2C(RTC_SDA, RTC_SCL); }
void seleccionarMAX() { cambiarBusI2C(MAX_SDA, MAX_SCL); }

/* ================================================================
 *  UTILIDADES
 * ================================================================ */

uint32_t hashFNV(const String &s) {
  uint32_t h = 0x811c9dc5UL;
  for (size_t i = 0; i < s.length(); i++) { h ^= (uint8_t)s[i]; h *= 0x01000193UL; }
  return h;
}

String hashHex(const String &s) {
  char buf[9];
  snprintf(buf, 9, "%08X", hashFNV(s));
  return String(buf);
}

String tiempoRestante() {
  long ms = (long)(DURACION_PANICO - (millis() - tPanico));
  if (ms < 0) ms = 0;
  char buf[8];
  snprintf(buf, 8, "%02d:%02d", (int)(ms / 60000), (int)((ms % 60000) / 1000));
  return String(buf);
}

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
    int hLocal = (gps.time.hour() - 6 + 24) % 24;
    char buf[20];
    snprintf(buf, 20, "%04d-%02d-%02dT%02d:%02d:%02d",
             gps.date.year(), gps.date.month(), gps.date.day(),
             hLocal, gps.time.minute(), gps.time.second());
    return String(buf);
  }
  return String(millis());
}

String clasificarEvento(float m, int bpm, int ox) {
  if (m > UMBRAL_G_CAIDA)                        return "CAIDA DETECTADA";
  if (m > UMBRAL_G_FORCEJEO)                     return "FORCEJEO";
  if (bpm > UMBRAL_BPM_ALTO && m < 1.5f)         return "ESTRES/MIEDO";
  if (ox < UMBRAL_SPO2_BAJO && ox > 0)           return "ALERTA SALUD";
  return "MOV BRUSCO";
}

/* ================================================================
 *  FUNCIONES DE DIBUJO (NUEVO DISEÑO)
 * ================================================================ */

// Dibuja campana animada
void drawBell(int cx, int cy, bool ring) {
  // Asa superior
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
  // Badajo
  int bx = ring ? cx+3 : cx;
  oled.drawLine(cx, cy+5, bx, cy+9);
  oled.fillCircle(bx, cy+12, 3);
  // Ondas de sonido
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

// PANTALLA REPOSO: Diseño horizontal limpio
void pantallaUnificada() {
  oled.clear();

  // --- FECHA (arriba, centrada) ---
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  if (rtcOK) {
    char buf[11];
    snprintf(buf, 11, "%04d/%02d/%02d", ultimaHora.year(), ultimaHora.month(), ultimaHora.day());
    oled.drawString(64, 0, String(buf));
  }

  oled.drawLine(0, 12, 128, 12);

  // --- HORA GRANDE ---
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

  oled.drawString(52, 16, String(hhStr)); 

  // Dos puntos (usamos blinkState para parpadeo)
  if (blinkState) {
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    oled.drawString(64, 14, ":");
  }

  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(68, 16, String(mmStr));

  // Segundos pequeños
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(104, 28, String(ssStr));

  oled.drawLine(0, 50, 128, 50);

  // --- BARRA INFERIOR: BPM | SpO2 | MOV ---
  // Sección 1: BPM
  oled.drawXbm(2, 53, 12, 8, HEART_BITS);
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(16, 53, String(beatAvg));

  oled.drawLine(42, 51, 42, 63);

  // Sección 2: SpO2
  oled.drawXbm(44, 54, 6, 8, DROP_BITS);
  oled.drawString(52, 53, String(spo2) + "%");

  oled.drawLine(84, 51, 84, 63);

  // Sección 3: Acelerómetro (MOV)
  oled.drawXbm(86, 53, 10, 8, WAVE_BITS);
  oled.drawString(98, 53, mag > 1.1 ? "MOV" : "OK");

  oled.display();
}

// PANTALLA PANICO: Diseño con Campana y Marco
void pantallaPanico() {
  oled.clear();

  // Marco doble SIEMPRE visible
  oled.drawRect(0, 0, 128, 64);
  oled.drawRect(2, 2, 124, 60);

  // Campana (parpadea con blinkState)
  drawBell(26, 28, blinkState);

  // Texto PÁNICO
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  oled.drawString(26, 51, "PANICO");

  // Divisor vertical
  oled.drawLine(52, 4, 52, 60);

  // Columna derecha
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);

  // !EMERGENCIA!!
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

  // Localización
  oled.drawXbm(56, 51, 8, 12, PIN_BITS);
  oled.drawString(70, 52, gpsValido ? "Loc.enviada" : "Sin GPS");

  oled.display();
}

// Dibuja el estado/alerta (Se mantiene por si se usa, pero en REPOSO se usa pantallaUnificada)
void dibujarEstado() {
  // Esta función ya no sobrescribe la pantalla unificada para no romper el diseño
  // Se puede dejar vacía o usar solo para depuración, pero la lógica visual principal
  // está en pantallaUnificada y pantallaPanico.
}

// Pantalla de alerta física breve
void mostrarAlerta(const String &tipo, float m) {
  oled.clear();
  oled.drawRect(0, 0, 128, 64);
  oled.drawRect(2, 2, 124, 60);
  oled.setFont(ArialMT_Plain_16);
  oled.drawString(50, 4, "!!");
  oled.setFont(ArialMT_Plain_10);
  int w = oled.getStringWidth(tipo);
  oled.drawString((128 - w) / 2, 26, tipo);
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f g", m);
  oled.drawString((128 - oled.getStringWidth(buf)) / 2, 40, buf);
  oled.display();
}

/* ================================================================
 *  LÓGICA PRINCIPAL
 * ================================================================ */

void leerBoton() {
  bool btn = (digitalRead(BTN_PANICO) == LOW);
  if (btn && !botonPresionado && millis() - tBoton > 400) {
    botonPresionado = true;
    tBoton = millis();
    if (estadoActual == PANICO) return;
    estadoActual  = PANICO;
    tPanico       = millis();
    tUltimoEnvio  = 0;
    numEvidencia  = 0;
    tipoMovimiento = "PANICO MANUAL";
    Serial.println(F("PANICO ACTIVADO"));
  }
  if (!btn) botonPresionado = false;
}

void enviarEvidencia() {
  numEvidencia++;

  // Clasificar evento
  if (tipoMovimiento != "PANICO MANUAL") {
    if      (mag > UMBRAL_G_CAIDA)       tipoMovimiento = "CAIDA";
    else if (mag > UMBRAL_G_FORCEJEO)    tipoMovimiento = "FORCEJEO";
    else if (beatAvg > UMBRAL_BPM_ALTO)  tipoMovimiento = "ESTRES";
    else                                 tipoMovimiento = "ALERTA";
  }

  String ts = obtenerTimestamp();

  // JSON con todos los campos (INTACTO)
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

  int i = 0;
  bufTx[i++] = dirDestino;
  bufTx[i++] = dirLocal;
  bufTx[i++] = idMsg++;
  bufTx[i++] = (byte)json.length();
  for (size_t j = 0; j < json.length(); j++) bufTx[i++] = (byte)json[j];

  int r = radio.transmit(bufTx, i);
  Serial.println(r == RADIOLIB_ERR_NONE
                 ? "TX OK: " + json
                 : "TX err: " + String(r));
}

/* ================================================================
 *  SETUP
 * ================================================================ */

void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 2000);   

  pinMode(BTN_PANICO, INPUT_PULLUP);

  // Encender pantalla
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(300);

  oled.init();
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "Iniciando...");
  oled.display();
  delay(500);

  // --- RTC ---
  oled.drawString(0, 14, "DS3231...");
  oled.display();
  seleccionarRTC();
  rtcOK = rtc.begin(&busI2C);
  if (rtcOK) {
    if (rtc.lostPower() || FORZAR_AJUSTE)
      rtc.adjust(DateTime(2026, 3, 1, 23, 26, 0));
    ultimaHora = rtc.now();
  }

  // --- MPU6050 ---
  oled.drawString(0, 28, "MPU6050...");
  oled.display();
  seleccionarMPU();
  sensor.initialize();
  mpuOK = sensor.testConnection();
  Serial.println(mpuOK ? "MPU6050: OK" : "MPU6050: ERROR");

  // --- MAX30102 ---
  oled.drawString(0, 42, "MAX30102...");
  oled.display();
  seleccionarMAX();
  delay(300);
  if (particleSensor.begin(busI2C, I2C_SPEED_STANDARD)) {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeGreen(0);
    maxOK = true;
  }

  // --- GPS ---
  neogps.begin(9600, SERIAL_8N1, 45, 46);

  // --- LoRa ---
  SPI.begin(9, 11, 10, 8);
  delay(200);
  int st = radio.begin(FREQUENCY, BANDWIDTH, SPREAD_FACTOR, CODING_RATE);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setOutputPower(TX_POWER);
    radio.setCRC(true);
  }

  // --- Resumen en pantalla ---
  oled.clear();
  oled.drawString(0,  0, rtcOK ? "DS3231:   OK" : "DS3231:   ERROR");
  oled.drawString(0, 14, mpuOK ? "MPU6050:  OK" : "MPU6050:  ERROR");
  oled.drawString(0, 28, maxOK ? "MAX30102: OK" : "MAX30102: ERROR");
  oled.display();
  delay(3000);

  seleccionarMPU();
}

/* ================================================================
 *  LOOP
 * ================================================================ */

void loop() {
  // Animación parpadeo global
  if (millis() - lastBlink >= 400) {
    lastBlink = millis();
    blinkState = !blinkState;
  }

  leerBoton();

  // --- GPS ---
  while (neogps.available()) gps.encode(neogps.read());
  if (gps.location.isValid()) {
    gpsValido = true;
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  // Sincronizar RTC con GPS (una sola vez)
  if (!sincronizado && rtcOK &&
      gps.location.isValid() && gps.time.isValid() && gps.date.isValid()) {
    seleccionarRTC();
    DateTime utcGPS(gps.date.year(), gps.date.month(), gps.date.day(),
                    gps.time.hour(), gps.time.minute(), gps.time.second());
    DateTime localGPS(utcGPS.unixtime() - 6UL * 3600UL);
    rtc.adjust(localGPS);
    sincronizado = true;
    Serial.println("RTC sincronizado GPS");
  }

  // --- MAX30102: BPM y SpO2 ---
  if (maxOK) {
    seleccionarMAX();
    long ir  = particleSensor.getIR();
    long red = particleSensor.getRed();

    if (ir < 50000) {
      beatAvg = 0;
      spo2    = 0;
    } else {
      // BPM
      if (checkForBeat(ir)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        beatsPerMinute = 60.0 / (delta / 1000.0);
        if (beatsPerMinute > 20 && beatsPerMinute < 255) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }
      // SpO2
      if (ir > 0 && red > 0) {
        float ratio = (float)red / (float)ir;
        spo2 = constrain((int)(110 - 25 * ratio), 70, 100);
      }
    }
  }

  // --- MPU6050 ---
  static unsigned long tAlterna = 0;
  if (millis() - tAlterna > 200) {
    tAlterna = millis();

    seleccionarMPU();
    if (!sensor.testConnection()) {
      sensor.initialize();
      mpuOK = sensor.testConnection();
    }

    if (mpuOK) {
      int16_t axr, ayr, azr, gxr, gyr, gzr;
      sensor.getMotion6(&axr, &ayr, &azr, &gxr, &gyr, &gzr);

      ax_g = axr / LSB_POR_G;
      ay_g = ayr / LSB_POR_G;
      az_g = azr / LSB_POR_G;
      mag  = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

      gx_dps = gxr / LSB_POR_GYRO;
      gy_dps = gyr / LSB_POR_GYRO;
      gz_dps = gzr / LSB_POR_GYRO;

      // Detección de alertas automáticas (solo en REPOSO)
      if (estadoActual == REPOSO) {
        if (mag > UMBRAL_G_FORCEJEO) {
          enAlerta    = true;
          tiempoAlerta = millis();
          tipoAlerta  = clasificarEvento(mag, beatAvg, spo2);
          magAlerta   = mag;
          Serial.printf("ALERTA FISICA: %s | %.2f g\n",
                        tipoAlerta.c_str(), mag);
        } else if (beatAvg > UMBRAL_BPM_ALTO ||(spo2 < UMBRAL_SPO2_BAJO && spo2 > 0 && beatAvg > 20)) {
          enAlerta    = true;
          tiempoAlerta = millis();
          tipoAlerta  = clasificarEvento(mag, beatAvg, spo2);
          magAlerta   = mag;
          Serial.printf("ALERTA BIO: %s | BPM:%d SpO2:%d\n",
                        tipoAlerta.c_str(), beatAvg, spo2);
        }
      }
    }

    // Actualizar RTC
    static bool turnoRTC = false;
    if (turnoRTC && rtcOK) {
      seleccionarRTC();
      ultimaHora = rtc.now();
    }
    turnoRTC = !turnoRTC;
  }

  // --- MÁQUINA DE ESTADOS ---
  switch (estadoActual) {

    case REPOSO:
      if (enAlerta) {
        mostrarAlerta(tipoAlerta, magAlerta);
        if (millis() - tiempoAlerta >= DUR_ALERTA) enAlerta = false;
      } else {
        pantallaUnificada(); // Pantalla reloj limpio
        // dibujarEstado(); // No se usa para mantener diseño limpio
      }
      break;

    case PANICO:
      if (millis() - tUltimoEnvio >= INTERVALO_ENVIO) {
        enviarEvidencia();
        tUltimoEnvio = millis();
      }
      pantallaPanico(); // Pantalla pánico con campana
      if (millis() - tPanico >= DURACION_PANICO) {
        estadoActual = REPOSO;
        numEvidencia = 0;
      }
      break;
  }

  delay(10);
}
