/**
 * @file gateway_celular.ino
 * @brief LILYGO T-SIM7000G - Gateway Celular + ESP-NOW + HiveMQ
 * 
 * @details
 *   Este dispositivo actúa como puente (Gateway) entre la red local ESP-NOW
 *   (donde opera la Estación Base LoRa) y la nube (Internet vía 4G/LTE).
 *   Recibe tramas JSON por ESP-NOW y las reenvía íntegramente a un broker
 *   MQTT público (HiveMQ) para visualización en dashboards (Node-RED, etc).
 * 
 * @author Equipo Shero
 * @date Mayo 2026
 * @version 2.0 (Gateway Definitivo)
 */

// Definición de módem para la librería TinyGSM (específico para SIM7000G)
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024  // Buffer ampliado para recepción de datos TCP

// Librerías principales
#include <WiFi.h>         // Necesario para la radio WiFi subyacente de ESP-NOW
#include <esp_now.h>      // Protocolo de comunicación P2P de bajo consumo
#include <TinyGsmClient.h> // Driver para módulos SIMcom (SIM7000)
#include <PubSubClient.h> // Cliente MQTT ligero

/* ================================================================
 *  CONFIGURACIÓN DE HARDWARE Y CONECTIVIDAD
 * ================================================================ */

/**
 * @defgroup PinsConfig Configuración de Pines
 * @brief Definiciones de GPIO para el T-SIM7000G.
 * @{
 */
#define MODEM_TX   27  ///< TX del ESP32 conectado a RX del SIM7000
#define MODEM_RX   26  ///< RX del ESP32 conectado a TX del SIM7000
#define MODEM_PWR  4   ///< Pin de encendido (PWRKEY) del módem
#define LED_PIN    12  ///< LED indicador físico (Active LOW en esta placa)
/** @} */

/**
 * @defgroup NetworkConfig Credenciales de Red
 * @brief APN de Telcel y configuración MQTT.
 * @{
 */
const char apn[]      = "internet.itelcel.com"; ///< APN para datos Telcel
const char gprsUser[] = "webgprs";              ///< Usuario GPRS (genérico)
const char gprsPass[] = "webgprs2002";          ///< Contraseña GPRS

const char* broker = "broker.hivemq.com";       ///< Broker MQTT público
const int   port   = 1883;                      ///< Puerto MQTT estándar (sin TLS)
const char* topic  = "instituto/mujer/alertas"; ///< Tópico de publicación
/** @} */

/* ================================================================
 *  INSTANCIAS GLOBALES
 * ================================================================ */

// Puerto serial dedicado al módem
HardwareSerial ModemSerial(1);

// Objetos de la librería TinyGSM
TinyGsm        modem(ModemSerial);
TinyGsmClient  gsmClient(modem);
PubSubClient   mqtt(gsmClient);

/* ================================================================
 *  VARIABLES DE ESTADO
 * ================================================================ */

String datosPendientes = "";  ///< Buffer para almacenar el último JSON recibido
bool hayDatosNuevos = false;  ///< Flag que indica llegada de datos por ESP-NOW

/* ================================================================
 *  CALLBACK ESP-NOW (Recepción de Datos)
 * ================================================================ */

/**
 * @brief Función de callback ejecutada al recibir datos ESP-NOW.
 * 
 * @details
 *   Se activa de forma asíncrona cuando la Estación Base LoRa envía datos.
 *   Almacena el payload en un buffer String para ser procesado en el loop principal.
 * 
 * @param info Estructura con información del emisor (MAC, etc).
 * @param data Puntero al array de bytes recibidos.
 * @param len Longitud de los datos recibidos.
 * @note La firma de esta función varía según la versión del core ESP32.
 *       Si da error, cambiar "const esp_now_recv_info *info" por "const uint8_t * mac".
 */
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  // Solo procesamos si el buffer anterior ya fue enviado (evitar sobrescribir)
  if (!hayDatosNuevos) {
    datosPendientes = "";
    for (int i = 0; i < len; i++) datosPendientes += (char)data[i];
    hayDatosNuevos = true;
  }
}

/* ================================================================
 *  FUNCIONES DE CONTROL DE HARDWARE
 * ================================================================ */

/**
 * @brief Secuencia de encendido del SIM7000G.
 * 
 * @details
 *   El módulo SIM7000 requiere una secuencia específica en el pin PWRKEY:
 *   - LOW durante ~1 segundo para iniciar el encendido.
 *   - Luego se libera (HIGH) para completar la secuencia.
 */
void encenderModem() {
    Serial.println("[PWR] Encendiendo módem...");
    pinMode(MODEM_PWR, OUTPUT);
    digitalWrite(MODEM_PWR, LOW);  delay(1000); // Pulso LOW
    digitalWrite(MODEM_PWR, HIGH);              // Liberar
    Serial.println("[PWR] Listo");
}

/* ================================================================
 *  FUNCIONES DE CONEXIÓN MQTT
 * ================================================================ */

/**
 * @brief Intenta conectar al broker MQTT con reintentos limitados.
 * 
 * @details
 *   Genera un ClientID único basado en la MAC del ESP32 para evitar
 *   conflictos de sesión con otros dispositivos.
 * 
 * @return true si la conexión fue exitosa, false si se agotaron los intentos.
 */
bool mqttConectar() {
    int intentos = 0;
    while (!mqtt.connected() && intentos < 5) {
        Serial.printf("[MQTT] Intento %d/5 conectando a %s...\n", intentos + 1, broker);
        
        // Generar ID único: "LilyGO-<MAC>"
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

/* ================================================================
 *  INICIALIZACIÓN (SETUP)
 * ================================================================ */

void setup() {
    Serial.begin(115200);
    delay(200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Estado inicial: LED apagado (Lógica invertida)

    // Bloque de inicialización WiFi (Parche para visualizar MAC)
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());  // Imprime MAC para configurar en el receptor LoRa
    // -------------------------------------------------------

    Serial.println("\n========================================");
    Serial.println("  INICIANDO GATEWAY CELULAR + ESP-NOW");
    Serial.println("========================================\n");

    // --- PASO 0: Inicializar ESP-NOW ---
    // Es importante hacerlo antes de bloquear el código con el módem
    WiFi.mode(WIFI_STA);       // Modo Estación obligatorio para ESP-NOW
    WiFi.disconnect();         // Desconectar de cualquier WiFi previo
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(OnDataRecv);
        Serial.println("📡 [ESP-NOW] Iniciado y escuchando a la receptora LoRa");
    } else {
        Serial.println("❌ [ESP-NOW] Error al iniciar");
    }

    // --- PASO 1: Encender Módem ---
    encenderModem();
    ModemSerial.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(3000); // Espera necesaria para que el módem inicie sus periféricos

    // --- PASO 2: Sincronización AT ---
    Serial.println("[1/4] Esperando módem...");
    uint32_t t = millis();
    while (!modem.testAT()) {
        Serial.print(".");
        if (millis() - t > 60000) {
            Serial.println("\n[!] Módem no responde. Revisa conexión física.");
            while (1); // Bloqueo fatal
        }
        delay(500);
    }
    Serial.println("\n Módem online");

    // --- PASO 3: Verificar SIM ---
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

    // --- PASO 4: Conexión a Red Celular ---
    Serial.println("[3/4] Conectando a red Telcel (puede tardar ~1 min)...");
    modem.sendAT("+CBAND=ALL_MODE"); modem.waitResponse(); // Permitir todas las bandas
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

    // --- PASO 5: Conexión GPRS (Datos) ---
    Serial.println("[4/4] Activando GPRS...");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println("[!] GPRS falló. Verifica APN de Telcel.");
        while (1);
    }
    Serial.println(" GPRS conectado");

    // --- PASO 6: Conexión MQTT ---
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

/* ================================================================
 *  CICLO PRINCIPAL (LOOP)
 * ================================================================ */

void loop() {
    // --- Mantenimiento de Conexión ---
    // Verifica que MQTT esté activo. Si se cayó, intenta reconectar.
    if (!mqtt.connected()) {
        Serial.println("[MQTT] Desconectado, reconectando...");
        
        // Si GPRS también cayó (común en móviles), reactivar
        if (!modem.isGprsConnected()) {
            Serial.println("[GPRS] Reconectando GPRS...");
            modem.gprsConnect(apn, gprsUser, gprsPass);
        }
        
        mqttConectar();
        delay(2000);
        return; // Reiniciar loop para asegurar conexión estable
    }
    mqtt.loop(); // Procesar paquetes MQTT entrantes/salientes (KeepAlive)


    // --- Procesamiento de Alertas ESP-NOW ---
    // Si el flag se activó en la interrupción (callback), procesamos aquí
    if (hayDatosNuevos) {
        Serial.println("\n🚨 ¡ALERTA RECIBIDA DE LA PULSERA!");
        Serial.println("Datos: " + datosPendientes);

        // Intentar publicación MQTT con QoS 0 (no asegurado, pero rápido)
        // .c_str() convierte el String a puntero char const requerido por PubSubClient
        if (mqtt.publish(topic, datosPendientes.c_str(), true)) { // Retained = true
            Serial.println("🚀 ¡DATOS REALES PUBLICADOS EN LA NUBE CON ÉXITO!");
            
            // Feedback visual local
            digitalWrite(LED_PIN, LOW);  delay(200); // LED ON
            digitalWrite(LED_PIN, HIGH);             // LED OFF
        } else {
            Serial.println("❌ Error al enviar a la nube");
        }
        
        // Limpiar flag para recibir la siguiente alerta
        hayDatosNuevos = false;
    }
}