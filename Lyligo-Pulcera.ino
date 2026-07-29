/**
 * NODO EMISOR - PULSERA INTELIGENTE PARA SEGURIDAD DE MUJERES
 * Placa: LilyGO T-SIM7000G (ESP32 WROVER-B)
 *
 * Disparo de pánico: EXCLUSIVAMENTE por botón físico (GPIO33). No hay
 * detección automática por acelerómetro ni por signos vitales -- el
 * MPU6050 y el MAX30102 se leen y se envían como datos de contexto
 * (movimiento, BPM, SpO2) junto con cada evidencia, pero no deciden por
 * sí solos activar una alerta.
 *
 * El módem y el GPS del SIM7000G se dejan siempre encendidos (nunca se
 * apagan, ni siquiera en deep sleep) para poder responder al pánico sin
 * tener que re-registrarse en la red desde cero. El ahorro de batería en
 * reposo viene de apagar el MPU6050, el MAX30102 y la pantalla OLED
 * durante los periodos de inactividad, usando deep sleep del ESP32.
 *
 * ============================================================================
 *  ⚠️  COSAS QUE VERIFICAR FÍSICAMENTE ANTES DE ARMAR EL CABLEADO FINAL  ⚠️
 * ============================================================================
 *
 * 1) MPU6050, dirección I2C 0x69: el código asume AD0 conectado a 3.3V
 *    (no a GND). Si en tu montaje AD0 quedó flotando o a GND, la
 *    dirección real del sensor sería 0x68, y el código no lo encontraría.
 *
 * 2) MAX30102 en bus I2C separado (SDA=18, SCL=19, segunda instancia de
 *    hardware I2C del ESP32). En algunas revisiones de la LilyGO
 *    T-SIM7000G, GPIO18/19 están reservados para el bus SPI de la
 *    tarjeta SD. Antes de soldar, confirma contra el esquemático oficial
 *    de tu revisión específica que esos pines estén libres.
 *
 * 3) El comando AT+SGPIO=0,4,1,1 usado para activar el GPS interno solo
 *    está confirmado para la revisión de placa V1.1 (20200415). Si tu
 *    placa es otra revisión, este comando podría no aplicar igual.
 *
 * 4) La hora (horaSincronizada) se resincroniza con AT+CCLK? cada vez que
 *    la pulsera despierta o se activa el botón, en vez de guardarse en
 *    memoria RTC entre ciclos de deep sleep. Esto es intencional: millis()
 *    se reinicia en cada deep sleep, así que cualquier offset de hora
 *    guardado quedaría desfasado por el tiempo que la pulsera estuvo
 *    dormida (tiempo que no se puede medir con millis()). Pedir la hora
 *    fresca al módem es más confiable que intentar reconstruirla.
 *
 * 5) celularOK_rtc SÍ se guarda en memoria RTC (RTC_DATA_ATTR), porque a
 *    diferencia de la hora, aquí no hay drift que corregir: es solo una
 *    pista de si la última vez que se durmió, había conexión a red/MQTT.
 *    Sirve para decidir si vale la pena intentar una reconexión rápida
 *    (timeout corto) antes de caer al proceso completo (60s). No es una
 *    garantía de que el módem siga registrado -- eso puede cambiar por
 *    razones fuera del control del código (cobertura, alimentación).
 *
 * 6) Diagnóstico de red incluido (imprimirDiagnosticoRed): antes de cada
 *    intento de conexión se reporta por Serial el estado de celularOK_rtc,
 *    la calidad de señal (AT+CSQ) y el estado de registro (AT+CREG?), más
 *    cuánto tardó el intento. Es una herramienta de buenas prácticas para
 *    poder correlacionar fallas de conexión con causas reales (señal,
 *    registro, alimentación) en vez de adivinar.
 *
 * Autor: Equipo Shero
 * Fecha: Julio 2026
 * Versión: 3.0 (solo botón de pánico manual, sin detección automática)
 */

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024

#include <Wire.h>
#include <SSD1306Wire.h>      // Librería genérica ThingPulse/Squix
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "MPU6050.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "driver/rtc_io.h"     // rtc_gpio_pullup_en / rtc_gpio_hold (deep sleep)

/* ================================================================
 *  PINES CONFIRMADOS DEL MÓDEM/PLACA
 * ================================================================ */
#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4
#define LED_PIN     12

/* ================================================================
 *  PINES DE PERIFÉRICOS
 *  - OLED y MPU6050 comparten el bus I2C original (Wire)
 *  - MAX30102 usa un bus I2C separado (segundo periférico I2C del ESP32)
 * ================================================================ */
#define I2C_SDA      21   // OLED + MPU6050 (bus Wire)
#define I2C_SCL      22   // OLED + MPU6050 (bus Wire)
#define I2C_SDA_MAX  18   // MAX30102 (bus separado)
#define I2C_SCL_MAX  19   // MAX30102 (bus separado)
#define BTN_PANICO   33

/* ================================================================
 *  DEEP SLEEP - Control de inactividad
 * ================================================================ */
unsigned long tUltimaActividad = 0;
#define TIEMPO_INACTIVIDAD 30000UL  // 30 s sin actividad en REPOSO -> dormir
                                      // (ajústalo según cuánto quieras que
                                      // dure la pantalla/sensores prendidos
                                      // entre usos; no afecta al módem/GPS,
                                      // que quedan siempre activos)

/* ================================================================
 *  RECONEXIÓN A RED TRAS DEEP SLEEP
 * ================================================================ */
// Buena práctica: cualquier estado que deba persistir entre ciclos de
// deep sleep debe declararse con RTC_DATA_ATTR, ya que el deep sleep es
// en la práctica un reinicio completo del ESP32 -- solo la memoria RTC
// sobrevive. Aquí se usa para recordar si la última vez que se durmió
// había conexión a red/MQTT, y así decidir si conviene intentar primero
// una reconexión rápida.
RTC_DATA_ATTR bool celularOK_rtc = false;

#define RED_TIMEOUT_RAPIDO 5000UL     // timeout corto cuando celularOK_rtc sugiere reconexión probable
#define RED_TIMEOUT_COMPLETO 60000UL  // timeout completo para registro desde cero

/* ================================================================
 *  CREDENCIALES DE RED
 * ================================================================ */
const char apn[]      = "internet.itelcel.com";
const char gprsUser[] = "webgprs";
const char gprsPass[] = "webgprs2002";

const char* broker = "broker.hivemq.com";
const int   port   = 1883;
const char* topic  = "instituto/mujer/alertas";

/* ================================================================
 *  INSTANCIAS GLOBALES
 * ================================================================ */
HardwareSerial ModemSerial(1);
TinyGsm        modem(ModemSerial);
TinyGsmClient  gsmClient(modem);
PubSubClient   mqtt(gsmClient);

SSD1306Wire oled(0x3c, I2C_SDA, I2C_SCL, GEOMETRY_128_64);

// Bus I2C separado para el MAX30102 (usa el periférico I2C #1 del ESP32,
// distinto del Wire por defecto que usan OLED y MPU6050)
TwoWire I2C_MAX = TwoWire(1);

MPU6050    sensor(0x69);     // AD0 en 3.3V -> dirección real 0x69
MAX30105   particleSensor;   // 0x57, en I2C_MAX

bool mpuOK = false;
bool maxOK = false;
bool celularOK = false;

/* ================================================================
 *  VARIABLES DE SENSORES
 *  Se leen y se envían como datos de contexto (movimiento, BPM, SpO2)
 *  junto con cada evidencia de pánico. No disparan ninguna alerta por
 *  sí solos -- el único disparador de pánico es el botón físico.
 * ================================================================ */
float ax_g = 0, ay_g = 0, az_g = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;
float mag = 1.0;
const float LSB_POR_G    = 16384.0f;
const float LSB_POR_GYRO = 131.0f;

const byte RATE_SIZE = 4;
byte  rates[RATE_SIZE];
byte  rateSpot = 0;
long  lastBeat = 0;
float beatsPerMinute = 0;
int   beatAvg = 0;
int   spo2    = 0;

/* ================================================================
 *  GPS (vía módem, siempre activo -> ver nota de diseño arriba)
 * ================================================================ */
float lat = 0, lon = 0;
bool  gpsValido = false;

/* ================================================================
 *  MÁQUINA DE ESTADOS Y CONTROL DE PÁNICO
 * ================================================================ */
enum Estado { REPOSO, PANICO };
Estado estadoActual = REPOSO;
unsigned long tPanico      = 0;
unsigned long tUltimoEnvio = 0;
int  numEvidencia  = 0;

#define DURACION_PANICO 3600000UL
#define INTERVALO_ENVIO 2000UL

bool          botonPresionado = false;
unsigned long tBoton          = 0;

bool blinkState = false;
unsigned long lastBlink = 0;

/* ================================================================
 *  HORA POR RED CELULAR
 * ================================================================ */
int redYY = 0, redMM = 0, redDD = 0, redHH = 0, redMI = 0, redSS = 0;
unsigned long tSincroHora = 0;
bool horaSincronizada = false;
// Buena práctica: estas variables se dejan como globales normales A
// PROPÓSITO (no RTC_DATA_ATTR) -- ver nota 4 al inicio del archivo sobre
// por qué no conviene persistir el offset de hora entre ciclos de sleep.

// Consulta la hora de red (NITZ) al módem vía AT+CCLK? y la guarda como
// punto de referencia (redHH/redMI/redSS + millis() de referencia).
bool sincronizarHoraRed() {
  String resp;
  modem.sendAT("+CCLK?");
  if (modem.waitResponse(5000, resp) != 1) return false;

  int idx = resp.indexOf("+CCLK: \"");
  if (idx == -1) return false;
  String data = resp.substring(idx + 8);
  if (data.length() < 17) return false;

  redYY = data.substring(0, 2).toInt();
  redMM = data.substring(3, 5).toInt();
  redDD = data.substring(6, 8).toInt();
  redHH = data.substring(9, 11).toInt();
  redMI = data.substring(12, 14).toInt();
  redSS = data.substring(15, 17).toInt();

  tSincroHora = millis();
  horaSincronizada = true;
  return true;
}

// Calcula la hora actual sumando el tiempo transcurrido (millis) desde la
// última sincronización NITZ a la hora de referencia guardada.
void obtenerHoraActual(int &yy, int &mm, int &dd, int &hh, int &mi, int &ss) {
  yy = redYY; mm = redMM; dd = redDD;
  long totalSeg = redHH * 3600L + redMI * 60L + redSS;
  totalSeg += (millis() - tSincroHora) / 1000UL;
  totalSeg %= 86400L;
  hh = totalSeg / 3600L;
  mi = (totalSeg % 3600L) / 60L;
  ss = totalSeg % 60L;
}

/* ================================================================
 *  FUNCIONES DE CONTROL DEL MÓDEM
 * ================================================================ */
// Pulso de encendido del PWRKEY del SIM7000G. Solo debe llamarse en un
// arranque en frío (ver bloque condicional en setup()); repetir el pulso
// con el módem ya encendido puede apagarlo en vez de encenderlo.
void encenderModem() {
  Serial.println("[PWR] Encendiendo módem...");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);
  Serial.println("[PWR] Listo");
}

// Diagnóstico de red: reporta por Serial, antes de cada intento de
// conexión, el estado de celularOK_rtc, la calidad de señal (AT+CSQ) y
// el estado de registro (AT+CREG?). Buena práctica para poder
// correlacionar fallas de conexión con su causa real en vez de adivinar.
//
// AT+CSQ responde "+CSQ: <rssi>,<ber>", rssi de 0-31 (más alto = mejor
// señal), 99 = no se puede medir/sin señal. Como referencia orientativa,
// no un estándar estricto: <10 suele ser señal pobre, 10-20 aceptable,
// >20 buena.
//
// AT+CREG? responde "+CREG: <n>,<stat>": stat=1 registrado (red local),
// 5 registrado (roaming), 0 no registrado y no buscando, 2 no registrado
// pero buscando, 3 registro denegado.
//
// No está confirmado que el firmware AT de tu módulo específico devuelva
// exactamente este formato -- compáralo contra el manual AT del
// SIM7000G si ves algo distinto en el monitor serial.
void imprimirDiagnosticoRed(bool intentoRapido) {
  Serial.println("---- [DIAG] Estado antes de conectar ----");
  Serial.print("[DIAG] celularOK_rtc: ");
  Serial.println(celularOK_rtc ? "true (se intentará camino rápido)" : "false (camino completo)");
  Serial.print("[DIAG] modo de este intento: ");
  Serial.println(intentoRapido ? "RAPIDO" : "COMPLETO");

  String respCSQ;
  modem.sendAT("+CSQ");
  if (modem.waitResponse(2000, respCSQ) == 1) {
    Serial.print("[DIAG] CSQ crudo: ");
    Serial.println(respCSQ);
  } else {
    Serial.println("[DIAG] CSQ: sin respuesta del módem");
  }

  String respCREG;
  modem.sendAT("+CREG?");
  if (modem.waitResponse(2000, respCREG) == 1) {
    Serial.print("[DIAG] CREG crudo: ");
    Serial.println(respCREG);
  } else {
    Serial.println("[DIAG] CREG: sin respuesta del módem");
  }
  Serial.println("------------------------------------------");
}

// Registra el módem en la red celular, abre el contexto GPRS y conecta
// al broker MQTT.
//
// intentoRapido:
//   false (default) -> timeout completo (RED_TIMEOUT_COMPLETO).
//   true  -> se usa cuando celularOK_rtc sugiere que probablemente
//            seguimos registrados tras el deep sleep. Usa un timeout
//            corto (RED_TIMEOUT_RAPIDO) y, si falla, cae automáticamente
//            al proceso completo (recursión con intentoRapido=false).
//            Así nunca se pierde la posibilidad de conectar, solo se
//            intenta primero el camino corto.
bool conectarCelularYMqtt(bool intentoRapido = false) {
  unsigned long timeoutRed = intentoRapido ? RED_TIMEOUT_RAPIDO : RED_TIMEOUT_COMPLETO;
  unsigned long tInicioIntento = millis();  // para medir cuánto tarda este intento

  imprimirDiagnosticoRed(intentoRapido);

  Serial.println(intentoRapido ? "[RED] Intento rápido (post-sleep)..." : "[RED] Conectando a red celular...");

  if (!modem.waitForNetwork(timeoutRed)) {
    Serial.print("[DIAG] Tiempo hasta fallo de registro: ");
    Serial.print(millis() - tInicioIntento);
    Serial.println(" ms");

    if (intentoRapido) {
      Serial.println("[RED] Intento rápido falló, probando proceso completo...");
      return conectarCelularYMqtt(false);
    }
    Serial.println("[RED] Sin registro de red");
    celularOK_rtc = false;
    return false;
  }

  Serial.print("[DIAG] Tiempo hasta registro de red: ");
  Serial.print(millis() - tInicioIntento);
  Serial.println(" ms");

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println("[RED] GPRS falló");
    celularOK_rtc = false;
    return false;
  }

  mqtt.setServer(broker, port);
  String clientId = "Pulsera-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  int intentos = 0;
  int maxIntentos = intentoRapido ? 2 : 5;
  while (!mqtt.connected() && intentos < maxIntentos) {
    if (mqtt.connect(clientId.c_str())) {
      Serial.print("[MQTT] Conectado. Tiempo TOTAL del intento: ");
      Serial.print(millis() - tInicioIntento);
      Serial.println(" ms");
      celularOK_rtc = true;
      return true;
    }
    delay(intentoRapido ? 500 : 2000);
    intentos++;
  }
  Serial.print("[DIAG] MQTT no conectó tras registro de red. Tiempo total: ");
  Serial.print(millis() - tInicioIntento);
  Serial.println(" ms");
  celularOK_rtc = false;
  return false;
}

/* ================================================================
 *  GPS VÍA MÓDEM
 *  Se activa UNA sola vez (en el arranque en frío) y se deja
 *  corriendo. Al despertar del deep sleep NO se vuelve a activar,
 *  porque nunca se apagó: solo se le vuelve a preguntar su posición.
 * ================================================================ */
// Enciende la alimentación del GPS interno del módem y activa el motor
// NMEA/GNSS. Solo se llama una vez, en el arranque en frío.
void activarGPS() {
  // ⚠️ +SGPIO=0,4,1,1 solo confirmado para revisión de placa V1.1 (20200415)
  modem.sendAT("+SGPIO=0,4,1,1");
  modem.waitResponse(10000L);
  modem.enableGPS();
}

// Pide al módem la última posición vía AT+CGNSINF (TinyGSM la parsea) y
// actualiza lat/lon/gpsValido solo si la respuesta trae un fix válido.
void leerGPS() {
  float latTmp, lonTmp;
  if (modem.getGPS(&latTmp, &lonTmp)) {
    lat = latTmp;
    lon = lonTmp;
    gpsValido = true;
  }
}

/* ================================================================
 *  TIMESTAMP
 * ================================================================ */
// Devuelve la marca de tiempo en formato ISO 8601 si ya hay hora
// sincronizada por red; si no, cae de vuelta a millis() como valor
// provisional (útil solo para depuración, no representa fecha/hora real).
String obtenerTimestamp() {
  if (horaSincronizada) {
    int yy, mm, dd, hh, mi, ss;
    obtenerHoraActual(yy, mm, dd, hh, mi, ss);
    char buf[20];
    snprintf(buf, 20, "20%02d-%02d-%02dT%02d:%02d:%02d", yy, mm, dd, hh, mi, ss);
    return String(buf);
  }
  return String(millis());
}

/* ================================================================
 *  DEEP SLEEP - Apagado y encendido de sensores
 *  (el módem y el GPS NO se tocan aquí, quedan siempre activos)
 * ================================================================ */

/**
 * Apaga MPU6050, MAX30102 y "apaga" lógicamente la OLED.
 * El módem/GPS se dejan intencionalmente encendidos.
 */
void apagarSensores() {
  // MPU6050 -> sleep por escritura directa a PWR_MGMT_1 (0x6B), bit6=1
  // Dirección real 0x69 (ver nota AD0 arriba). Sigue en el bus Wire (21/22).
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x40);
  Wire.endTransmission();
  Serial.println("[SLEEP] MPU6050: dormido via registro directo");

  if (maxOK) {
    particleSensor.shutDown();
    Serial.println("[SLEEP] MAX30102: apagado (bus I2C_MAX, 18/19)");
  }

  // OLED: apagado lógico (pantalla en negro). Si tu librería instalada
  // trae oled.displayOff(), pruébala aquí en vez de clear()+display()
  // para un ahorro real de corriente:
  // oled.displayOff();
  oled.clear();
  oled.display();
  Serial.println("[SLEEP] OLED: en negro (apagado lógico)");

  Serial.println("[SLEEP] Módem y GPS: se dejan encendidos (decisión de diseño)");
}

/**
 * Reactiva MPU6050, MAX30102 y la OLED después de despertar.
 * El módem/GPS nunca se apagaron, así que no se reinicializan aquí;
 * solo se vuelve a abrir el UART hacia el módem en setup().
 */
void encenderSensores() {
  oled.init();
  oled.clear();
  oled.display();
  Serial.println("[WAKE] OLED: encendida");

  if (maxOK) {
    particleSensor.wakeUp();
    Serial.println("[WAKE] MAX30102: despertado (bus I2C_MAX, 18/19)");
  }

  // MPU6050 -> despertar limpiando el bit6 de PWR_MGMT_1 (0x6B)
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
  sensor.initialize();
  mpuOK = sensor.testConnection();
  Serial.println(mpuOK ? "[WAKE] MPU6050: OK" : "[WAKE] MPU6050: ERROR");
}

/**
 * Secuencia de entrada a deep sleep:
 * 1. Apaga MPU6050 / MAX30102 / OLED (no toca módem/GPS)
 * 2. Configura despertar SOLO por el botón de pánico (GPIO33, LOW)
 * 3. Habilita pull-up en el dominio RTC_GPIO (necesario porque
 *    INPUT_PULLUP normal no sobrevive el deep sleep)
 * 4. Entra a deep sleep
 */
void entrarDeepSleep() {
  Serial.println("[SLEEP] Preparando deep sleep...");

  apagarSensores();
  delay(100);

  // ⚠️ Verificar físicamente: GPIO33 como ext0 wakeup + pull-up RTC.
  rtc_gpio_pullup_en(GPIO_NUM_33);
  rtc_gpio_pulldown_dis(GPIO_NUM_33);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, LOW);

  Serial.println("[SLEEP] Entrando a deep sleep. Presiona el botón para despertar.");
  delay(100);

  esp_deep_sleep_start();
  // No continúa desde aquí; al despertar reinicia por setup()
}

/* ================================================================
 *  BOTÓN DE PÁNICO
 *  Único disparador de la máquina de estados hacia PANICO.
 * ================================================================ */
// Lee el botón de pánico con antirrebote por tiempo (400 ms) y, si se
// detecta una pulsación válida, cambia la máquina de estados a PANICO
// e intenta levantar conexión celular/MQTT y hora de red si aún no las hay.
void leerBoton() {
  bool btn = (digitalRead(BTN_PANICO) == LOW);
  if (btn && !botonPresionado && millis() - tBoton > 400) {
    botonPresionado = true;
    tBoton = millis();
    tUltimaActividad = millis();

    if (estadoActual == PANICO) return;

    estadoActual  = PANICO;
    tPanico       = millis();
    tUltimoEnvio  = 0;
    numEvidencia  = 0;
    Serial.println(F("PANICO ACTIVADO"));

    // Si celularOK_rtc sugiere que seguíamos registrados, se intenta
    // primero el camino rápido (ver conectarCelularYMqtt).
    if (!celularOK) celularOK = conectarCelularYMqtt(celularOK_rtc);
    if (!horaSincronizada) sincronizarHoraRed();
    // GPS ya está corriendo de forma continua; no hace falta activarGPS() aquí.
  }
  if (!btn) botonPresionado = false;
}

/* ================================================================
 *  ENVÍO DE EVIDENCIA
 * ================================================================ */
// Arma el paquete JSON de evidencia (GPS + signos vitales + IMU +
// timestamp) y lo publica por MQTT. BPM/SpO2/movimiento van como datos
// de contexto para quien reciba la alerta -- no determinan si se envía
// o no, eso ya lo decidió el botón de pánico.
void enviarEvidencia() {
  numEvidencia++;

  leerGPS();

  String ts = obtenerTimestamp();
  String fecha = ts.substring(0, 10);
  String hora  = ts.length() > 11 ? ts.substring(11) : "00:00:00";

  String json = "{";
  json += "\"SOS\":1,";
  json += "\"lat\":"   + String(lat, 6)    + ",";
  json += "\"lon\":"   + String(lon, 6)    + ",";
  json += "\"BPM\":"   + String(beatAvg)   + ",";
  json += "\"SpO2\":"  + String(spo2)      + ",";
  json += "\"AccX\":"  + String(ax_g, 2)   + ",";
  json += "\"AccY\":"  + String(ay_g, 2)   + ",";
  json += "\"AccZ\":"  + String(az_g, 2)   + ",";
  json += "\"GyroX\":" + String(gx_dps, 1) + ",";
  json += "\"GyroY\":" + String(gy_dps, 1) + ",";
  json += "\"GyroZ\":" + String(gz_dps, 1) + ",";
  json += "\"fecha\":\"" + fecha + "\",";
  json += "\"hora\":\""  + hora  + "\"";
  json += "}";

  if (!mqtt.connected()) {
    celularOK = conectarCelularYMqtt(celularOK_rtc);
  }
  if (celularOK && mqtt.connected()) {
    bool ok = mqtt.publish(topic, json.c_str());
    Serial.println(ok ? "TX OK: " + json : "TX err (publish false)");
  } else {
    Serial.println("Sin conexión celular/MQTT, evidencia no enviada: " + json);
  }
  mqtt.loop();
}

/* ================================================================
 *  ICONOS XBM
 * ================================================================ */
static const uint8_t HEART_BITS[] = {
  0x36, 0x00, 0x7F, 0x00, 0xFF, 0x00, 0xFF, 0x00,
  0x7E, 0x00, 0x3C, 0x00, 0x18, 0x00, 0x08, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t DROP_BITS[] = {
  0x0C, 0x1E, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1E,
  0x0C, 0x00, 0x00, 0x00,
};
static const uint8_t CLOCK_BITS[] = {
  0x7C, 0x00, 0x82, 0x00, 0x9A, 0x00, 0xBA, 0x00,
  0x9A, 0x00, 0x82, 0x00, 0x82, 0x00, 0x7C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t PIN_BITS[] = {
  0x3E, 0x7F, 0x7F, 0x7F, 0x7F, 0x3E, 0x1C, 0x08,
  0x08, 0x00, 0x00, 0x00,
};
static const uint8_t WAVE_BITS[] = {
  0x00, 0x00, 0x22, 0x00, 0x55, 0x00, 0x88, 0x00,
  0x55, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// Dibuja el ícono de campana de emergencia; si ring=true agrega las
// "ondas de sonido" a los lados para indicar que está sonando (parpadeo).
void drawBell(int cx, int cy, bool ring) {
  oled.drawLine(cx-3, cy-13, cx+3, cy-13);
  oled.drawLine(cx,   cy-16, cx,   cy-13);
  oled.drawCircle(cx, cy-4, 8);
  oled.drawLine(cx-8, cy-4, cx-8, cy+4);
  oled.drawLine(cx+8, cy-4, cx+8, cy+4);
  oled.drawLine(cx-10, cy+5, cx+10, cy+5);
  oled.drawLine(cx-10, cy+4, cx-8,  cy+4);
  oled.drawLine(cx+10, cy+4, cx+8,  cy+4);

  int bx = ring ? cx+3 : cx;
  oled.drawLine(cx, cy+5, bx, cy+9);
  oled.fillCircle(bx, cy+12, 3);

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

// Pantalla principal en estado REPOSO: fecha, reloj (con ":" parpadeante)
// y una barra inferior con BPM, SpO2 y estado de movimiento.
void pantallaUnificada() {
  oled.clear();

  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  int yy, mm, dd, hh, mi, ss;
  if (horaSincronizada) {
    obtenerHoraActual(yy, mm, dd, hh, mi, ss);
    char buf[11];
    snprintf(buf, 11, "20%02d/%02d/%02d", yy, mm, dd);
    oled.drawString(64, 0, String(buf));
  }
  oled.drawLine(0, 12, 128, 12);

  oled.setFont(ArialMT_Plain_24);
  oled.setTextAlignment(TEXT_ALIGN_RIGHT);
  char hhStr[3], mmStr[3];
  if (horaSincronizada) {
    snprintf(hhStr, 3, "%02d", hh);
    snprintf(mmStr, 3, "%02d", mi);
  } else {
    snprintf(hhStr, 3, "--");
    snprintf(mmStr, 3, "--");
  }
  oled.drawString(52, 16, String(hhStr));

  if (blinkState) {
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    oled.drawString(64, 14, ":");
  }

  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(68, 16, String(mmStr));

  oled.drawLine(0, 50, 128, 50);

  oled.drawXbm(2, 53, 12, 8, HEART_BITS);
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(16, 53, String(beatAvg));

  oled.drawLine(42, 51, 42, 63);

  oled.drawXbm(44, 54, 6, 8, DROP_BITS);
  oled.drawString(52, 53, String(spo2) + "%");

  oled.drawLine(84, 51, 84, 63);

  oled.drawXbm(86, 53, 10, 8, WAVE_BITS);
  oled.drawString(98, 53, mag > 1.1 ? "MOV" : "OK");

  oled.display();
}

// Pantalla de estado PANICO: campana animada, BPM/SpO2, hora y si ya se
// consiguió fix de GPS para confirmar que la ubicación se está enviando.
void pantallaPanico() {
  oled.clear();

  oled.drawRect(0, 0, 128, 64);
  oled.drawRect(2, 2, 124, 60);

  drawBell(26, 28, blinkState);

  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  oled.drawString(26, 51, "PANICO");

  oled.drawLine(52, 4, 52, 60);

  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setFont(ArialMT_Plain_10);

  oled.drawString(55, 2, "!EMERGENCIA!!");
  oled.drawLine(53, 13, 125, 13);

  oled.drawXbm(55, 15, 12, 12, HEART_BITS);
  oled.drawString(70, 15, String(beatAvg) + " BPM");

  oled.drawXbm(56, 28, 8, 12, DROP_BITS);
  oled.drawString(70, 28, "SpO2:" + String(spo2) + "%");

  if (horaSincronizada) {
    int yy, mm, dd, hh, mi, ss;
    obtenerHoraActual(yy, mm, dd, hh, mi, ss);
    char buf[6];
    snprintf(buf, 6, "%02d:%02d", hh, mi);
    oled.drawXbm(55, 41, 10, 12, CLOCK_BITS);
    oled.drawString(70, 41, String(buf));
  }

  oled.drawXbm(56, 51, 8, 12, PIN_BITS);
  oled.drawString(70, 52, gpsValido ? "Loc.enviada" : "Sin GPS");

  oled.display();
}

/**
 * Pantalla breve que se muestra al despertar del deep sleep.
 */
void mostrarPantallaWake() {
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  oled.drawString(64, 10, "SISTEMA");
  oled.drawString(64, 24, "ACTIVADO");
  oled.drawString(64, 40, gpsValido ? "GPS: con fix" : "GPS: buscando...");
  oled.display();
  delay(1500);
}

/* ================================================================
 *  SETUP
 * ================================================================ */
void setup() {
  Serial.begin(115200);
  delay(200);

  // Detectar causa de arranque: ¿venimos de deep sleep por botón, o de
  // un power-on / reset normal?
  esp_sleep_wakeup_cause_t causa = esp_sleep_get_wakeup_cause();
  bool despertoDeSleep = (causa == ESP_SLEEP_WAKEUP_EXT0);

  if (despertoDeSleep) {
    Serial.println("[WAKE] Despertó por botón de pánico");
    rtc_gpio_hold_dis(GPIO_NUM_33); // liberar el hold del pin tras despertar
  } else {
    Serial.println("[BOOT] Arranque normal");
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(BTN_PANICO, INPUT_PULLUP);

  // Bus I2C #1: OLED + MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  // Bus I2C #2: MAX30102, en pines separados (18/19) para no compartir
  // bus con OLED/MPU6050
  I2C_MAX.begin(I2C_SDA_MAX, I2C_SCL_MAX);

  oled.init();
  oled.clear();

  // El UART hacia el módem se reabre siempre, tanto en arranque en frío
  // como al despertar (el módem físicamente sigue ahí, pero el ESP32
  // necesita reabrir su lado de la comunicación tras el reinicio).
  ModemSerial.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  if (despertoDeSleep) {
    mostrarPantallaWake();
    // Reactivar MPU6050 / MAX30102 / OLED (el módem/GPS nunca se apagaron)
    encenderSensores();

    estadoActual   = PANICO;
    tPanico        = millis();
    tUltimoEnvio   = 0;
    numEvidencia   = 0;
    tUltimaActividad = millis();
    Serial.println("[WAKE] PANICO ACTIVADO AUTOMATICAMENTE");

    // Buena práctica: darle tiempo al módem para estabilizarse antes de
    // mandarle comandos AT, igual que en el arranque en frío (ver bloque
    // !despertoDeSleep más abajo). El valor de 3000 ms se tomó del
    // arranque en frío como punto de partida; ajústalo según lo que
    // observes con el diagnóstico de red si hace falta.
    Serial.println("[WAKE] Esperando estabilización del módem antes de AT...");
    delay(3000);
    modem.init();

    // Intento de reconexión antes de entrar al loop(): si celularOK_rtc
    // quedó en true la última vez que dormimos, se usa el camino rápido
    // (timeout corto); si no, va directo al proceso completo. Arrancar
    // esto aquí, en vez de esperar al primer paso de leerBoton() dentro
    // del loop, gana el tiempo del primer ciclo.
    celularOK = conectarCelularYMqtt(celularOK_rtc);

    // La hora se vuelve a pedir fresca al despertar (ver nota 4 al
    // inicio del archivo).
    sincronizarHoraRed();

    // GPS nunca se apagó; pedimos una lectura fresca de posición.
    leerGPS();
  } else {
    oled.setFont(ArialMT_Plain_10);
    oled.drawString(0, 0, "Iniciando...");
    oled.display();

    // --- Módem: SOLO se enciende (pulso PWRKEY) en arranque en frío ---
    // Pulsar PWRKEY con el módem ya encendido puede apagarlo, por eso
    // este bloque está condicionado a !despertoDeSleep.
    encenderModem();
    delay(3000);
    modem.init();

    if (modem.waitForNetwork(15000L)) {
      sincronizarHoraRed();
    }

    // GPS se activa UNA sola vez aquí y se deja corriendo indefinidamente
    activarGPS();
  }

  sensor.initialize();
  mpuOK = sensor.testConnection();

  // MAX30102 se inicializa en el bus I2C_MAX (18/19), no en Wire
  if (particleSensor.begin(I2C_MAX, I2C_SPEED_STANDARD)) {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeGreen(0);
    maxOK = true;
  }

  if (!despertoDeSleep) {
    oled.clear();
    oled.drawString(0,  0, horaSincronizada ? "Hora red: OK" : "Hora red: pendiente");
    oled.drawString(0, 14, mpuOK ? "MPU6050:  OK" : "MPU6050:  ERROR");
    oled.drawString(0, 28, maxOK ? "MAX30102: OK" : "MAX30102: ERROR");
    oled.drawString(0, 42, "Modem:    OK");
    oled.display();
    delay(3000);
  }

  tUltimaActividad = millis();
}

/* ================================================================
 *  LOOP
 * ================================================================ */
void loop() {

  // Deep sleep por inactividad: solo en REPOSO.
  if (estadoActual == REPOSO) {
    if (millis() - tUltimaActividad > TIEMPO_INACTIVIDAD) {
      entrarDeepSleep();
    }
  }

  if (millis() - lastBlink >= 400) {
    lastBlink = millis();
    blinkState = !blinkState;
  }

  leerBoton();

  // Lectura de pulso/oxigenación (MAX30102). Si no hay dedo puesto
  // (ir < 50000) se reportan ceros en vez de mantener el último valor.
  // Estos valores son solo datos de contexto -- no disparan ninguna
  // alerta por sí solos.
  if (maxOK) {
    long ir  = particleSensor.getIR();
    long red = particleSensor.getRed();
    if (ir < 50000) {
      beatAvg = 0;
      spo2    = 0;
    } else {
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
      if (ir > 0 && red > 0) {
        float ratio = (float)red / (float)ir;
        spo2 = constrain((int)(110 - 25 * ratio), 70, 100);
      }
    }
  }

  // Lectura de IMU (MPU6050) cada 200 ms: calcula magnitud de
  // aceleración para mostrarla como dato de contexto en pantalla y en
  // el JSON de evidencia. No dispara ninguna alerta por sí sola.
  static unsigned long tAlterna = 0;
  if (millis() - tAlterna > 200) {
    tAlterna = millis();
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
    }
  }

  switch (estadoActual) {
    case REPOSO:
      pantallaUnificada();
      break;

    case PANICO:
      if (millis() - tUltimoEnvio >= INTERVALO_ENVIO) {
        enviarEvidencia();
        tUltimoEnvio = millis();
      }
      pantallaPanico();
      if (millis() - tPanico >= DURACION_PANICO) {
        estadoActual = REPOSO;
        numEvidencia = 0;
        tUltimaActividad = millis();
      }
      break;
  }

  delay(10);
}
