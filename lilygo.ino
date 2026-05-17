/**
 * LILYGO T-SIM + ESP-NOW + HIVEMQ
 * EL GATEWAY DEFINITIVO - DATOS REALES
 */

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024

#include <WiFi.h>         // <-- Para ESP-NOW
#include <esp_now.h>      // <-- Para ESP-NOW
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// ─── Pines ────────────────────────────────────────────────────────────────────
#define MODEM_TX   27
#define MODEM_RX   26
#define MODEM_PWR  4
#define LED_PIN    12

// ─── APN Telcel ───────────────────────────────────────────────────────────────
const char apn[]      = "internet.itelcel.com";
const char gprsUser[] = "webgprs";
const char gprsPass[] = "webgprs2002";

// ─── MQTT HiveMQ público ──────────────────────────────────────────────────────
const char* broker = "broker.hivemq.com";
const int   port   = 1883;
const char* topic  = "instituto/mujer/alertas"; // <-- TU TÓPICO REAL

// ─── Objetos ──────────────────────────────────────────────────────────────────
HardwareSerial ModemSerial(1);
TinyGsm        modem(ModemSerial);
TinyGsmClient  gsmClient(modem);
PubSubClient   mqtt(gsmClient);

// ─── Variables ESP-NOW (DATOS REALES) ─────────────────────────────────────────
String datosPendientes = "";
bool hayDatosNuevos = false;

// ─── Función que se activa al recibir datos de la placa LoRa ──────────────────
// Nota: Si te da error de compilación aquí, cambia "const esp_now_recv_info *info" 
// por "const uint8_t * mac" (depende de la versión de tu ESP32)
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (!hayDatosNuevos) {
    datosPendientes = "";
    for (int i = 0; i < len; i++) datosPendientes += (char)data[i];
    hayDatosNuevos = true;
  }
}

// ─── Encender módem ───────────────────────────────────────────────────────────
void encenderModem() {
    Serial.println("[PWR] Encendiendo módem...");
    pinMode(MODEM_PWR, OUTPUT);
    digitalWrite(MODEM_PWR, LOW);  delay(1000);
    digitalWrite(MODEM_PWR, HIGH);
    Serial.println("[PWR] Listo");
}

// ─── Conectar MQTT con reintentos ─────────────────────────────────────────────
bool mqttConectar() {
    int intentos = 0;
    while (!mqtt.connected() && intentos < 5) {
        Serial.printf("[MQTT] Intento %d/5 conectando a %s...\n", intentos + 1, broker);
        String clientId = "LilyGO-" + String((uint32_t)ESP.getEfuseMac(), HEX);
        if (mqtt.connect(clientId.c_str())) {
            Serial.println("[MQTT] Conectado!");
            return true;
        } else {
            Serial.printf("[MQTT] Falló (rc=%d), esperando 3s...\n", mqtt.state());
            delay(3000);
        }
        intentos++;
    }
    return false;
}

// ─── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // LED apagado

        // ↓ AGREGA ESTA LÍNEA AQUÍ ↓
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());  // <--- AQUÍ
    // ↑ AGREGA ESTA LÍNEA AQUÍ ↑

    Serial.println("\n========================================");
    Serial.println("  INICIANDO GATEWAY CELULAR + ESP-NOW");
    Serial.println("========================================\n");

    // --- 0. INICIAR ESP-NOW PRIMERO ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnDataRecv);
        Serial.println("📡 [ESP-NOW] Iniciado y escuchando a la receptora LoRa");
    } else {
        Serial.println("❌ [ESP-NOW] Error al iniciar");
    }

    // --- 1. Encender módem ---
    encenderModem();
    ModemSerial.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(3000);

    // --- 2. Esperar que el módem responda ---
    Serial.println("[1/4] Esperando módem...");
    uint32_t t = millis();
    while (!modem.testAT()) {
        Serial.print(".");
        if (millis() - t > 60000) {
            Serial.println("\n[!] Módem no responde. Revisa conexión física.");
            while (1);
        }
        delay(500);
    }
    Serial.println("\n Módem online");

    // --- 3. Esperar SIM ---
    Serial.println("[2/4] Verificando SIM Telcel...");
    t = millis();
    while (modem.getSimStatus() != SIM_READY) {
        Serial.print(".");
        if (millis() - t > 30000) {
            Serial.println("\n[!] SIM no detectada. ¿Está bien insertada?");
            while (1);
        }
        delay(500);
    }
    Serial.println("\n SIM lista");

    // --- 4. Registrar en red ---
    Serial.println("[3/4] Conectando a red Telcel (puede tardar ~1 min)...");
    modem.sendAT("+CBAND=ALL_MODE"); modem.waitResponse();
    modem.setPreferredMode(3);  // CAT-M + NB-IoT
    modem.setNetworkMode(2);    // Automático

    SIM70xxRegStatus status;
    do {
        status = modem.getRegistrationStatus();
        if (status == REG_DENIED) {
            Serial.println("[!] Red rechazó la SIM.");
            while (1);
        }
        delay(1000);
    } while (status != REG_OK_HOME && status != REG_OK_ROAMING);
    Serial.println(" Registrado en red Telcel");

    // --- 5. GPRS ---
    Serial.println("[4/4] Activando GPRS...");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println("[!] GPRS falló. Verifica APN de Telcel.");
        while (1);
    }
    Serial.println(" GPRS conectado");

    // --- 6. MQTT ---
    mqtt.setServer(broker, port);
    mqtt.setKeepAlive(60);
    if (!mqttConectar()) {
        Serial.println("[!] No se pudo conectar al broker MQTT.");
        while (1);
    }

    Serial.println("\n========================================");
    Serial.println(" ✅ SISTEMA LISTO Y ESPERANDO DATOS LORA");
    Serial.println("========================================\n");
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────
void loop() {
    // Reconectar MQTT y GPRS si se cae la red
    if (!mqtt.connected()) {
        Serial.println("[MQTT] Desconectado, reconectando...");
        if (!modem.isGprsConnected()) {
            Serial.println("[GPRS] Reconectando GPRS...");
            modem.gprsConnect(apn, gprsUser, gprsPass);
        }
        mqttConectar();
        delay(2000);
        return;
    }
    mqtt.loop();


    // ─── AQUÍ OCURRE LA MAGIA: Cuando llega un dato real por ESP-NOW ───
    if (hayDatosNuevos) {
        Serial.println("\n🚨 ¡ALERTA RECIBIDA DE LA PULSERA!");
        Serial.println("Datos: " + datosPendientes);

        // Enviamos la alerta real a HiveMQ
        if (mqtt.publish(topic, datosPendientes.c_str(), true)) {
            Serial.println("🚀 ¡DATOS REALES PUBLICADOS EN LA NUBE CON ÉXITO!");
            // Parpadeo LED como confirmación física
            digitalWrite(LED_PIN, LOW);  delay(200);
            digitalWrite(LED_PIN, HIGH);
        } else {
            Serial.println("❌ Error al enviar a la nube");
        }
        
        hayDatosNuevos = false; // Limpiamos para esperar la siguiente alerta
    }
}