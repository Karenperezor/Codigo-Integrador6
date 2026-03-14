/**
 * Receptor LoRa V2.1 — Instituto de la Mujer
 *
 * Este programa corre en una estación BASE (receptora).
 * Recibe paquetes de radio LoRa enviados por una PULSERA (transmisor)
 * que porta la usuaria. Cada paquete contiene datos como:
 *   - Tipo de evento (PANICO, CAIDA, FORCEJEO, o estado normal)
 *   - Aceleración del sensor IMU
 *   - Ritmo cardíaco (BPM)
 *   - Coordenadas GPS y número de satélites
 *   - Timestamp y número de paquete
 *   - Hash de integridad para detectar datos corruptos
 *
 * Los datos recibidos se muestran en una pantalla OLED de 128x64 px
 * organizada en 4 pantallas navegables con un botón físico.
 */

// ── Librerías necesarias ───────────────────────────────────────────
#include <RadioLib.h>          // Manejo del módulo de radio LoRa SX1276
#include <Wire.h>              // Comunicación I2C (para la pantalla OLED)
#include <Adafruit_GFX.h>      // Gráficos base para pantallas Adafruit
#include <Adafruit_SSD1306.h>  // Driver para pantalla OLED SSD1306

/* ================================================================
 *  CONFIGURACIÓN OLED
 *  Pantalla de 128x64 píxeles conectada por I2C.
 *  El pin OLED_RESET (GPIO 16) se usa para hacer reset por hardware.
 * ================================================================ */
#define SCREEN_WIDTH  128   // Ancho de la pantalla en píxeles
#define SCREEN_HEIGHT 64    // Alto de la pantalla en píxeles
#define OLED_RESET     16   // Pin GPIO para resetear la pantalla

// Objeto global de la pantalla OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ================================================================
 *  CONFIGURACIÓN LORA SX1276
 *  Módulo de radio conectado por SPI.
 *  Pines: NSS=18, DIO0=26, RESET=14, DIO1=35
 * ================================================================ */
SX1276 radio = new Module(18, 26, 14, 35);

#define FREQUENCY      915.0  // Frecuencia de radio en MHz (banda ISM 915)
#define BANDWIDTH      125.0  // Ancho de banda en kHz
#define SPREAD_FACTOR  8      // Factor de dispersión LoRa (rango vs velocidad)
#define CODING_RATE    5      // Tasa de codificación para corrección de errores
#define LED_PIN        25     // GPIO del LED indicador de actividad

/* ================================================================
 *  DIRECCIONES LORA
 *  Cada dispositivo tiene una dirección única de 1 byte.
 *  El receptor solo acepta paquetes que vengan de la pulsera
 *  y que estén dirigidos a él mismo.
 * ================================================================ */
byte dirLocal   = 0xD3; // Dirección de ESTE receptor (base)
byte dirPulsera = 0xC1; // Dirección de la pulsera (transmisor)

/* ================================================================
 *  ESTRUCTURA DE DATOS — Evidencia
 *  Representa un paquete de datos recibido desde la pulsera.
 *  Se llama "Evidencia" porque cada paquete queda registrado
 *  como evidencia de un evento (alerta o estado normal).
 * ================================================================ */
struct Evidencia {
  String tipo;    // Tipo de evento: "NORMAL", "PANICO", "CAIDA", "FORCEJEO", etc.
  float  acel;    // Magnitud de la aceleración medida por el IMU (en g)
  int    bpm;     // Ritmo cardíaco en latidos por minuto
  double lat;     // Latitud GPS (grados decimales)
  double lon;     // Longitud GPS (grados decimales)
  int    sats;    // Número de satélites GPS visibles
  bool   fix;     // true = el GPS tiene posición válida (fix), false = sin fix
  String ts;      // Timestamp (fecha/hora) del evento como texto
  int    numero;  // Número de secuencia del paquete (para detectar pérdidas)
  String hash;    // Hash FNV-1a recibido (para verificar integridad)
  float  rssi;    // RSSI de la señal recibida (dBm) — calidad de señal
  float  snr;     // SNR de la señal recibida (dB) — relación señal/ruido
  bool   hashOK;  // true si el hash calculado coincide con el recibido
};

// Historial circular de los últimos 5 paquetes recibidos
#define HIST_SIZE 5
Evidencia historial[HIST_SIZE];

// Último paquete recibido (se muestra en la pantalla principal)
Evidencia ultimo;

// Contadores globales de estadísticas
int  totalRecibidos = 0;  // Total de paquetes recibidos (incluyendo corruptos)
int  totalCorruptos = 0;  // Paquetes con parse fallido o hash incorrecto
bool hayDato        = false; // Bandera: true cuando hay datos nuevos que mostrar

/* ================================================================
 *  CONTROL DE PANTALLA
 *  El usuario navega entre 4 pantallas usando el botón físico (GPIO 0).
 *  Un punto en la parte inferior indica la pantalla actual.
 * ================================================================ */
int pantActual = 0;       // Índice de la pantalla actualmente visible (0 a 3)
#define N_PANTALLAS 4     // Número total de pantallas disponibles

bool          btnPresionado = false; // Estado anterior del botón (para detectar flanco)
unsigned long tBtn          = 0;     // Tiempo del último cambio de pantalla (debounce)
unsigned long tBlink        = 0;     // Tiempo del último parpadeo del LED

/* ================================================================
 *  FUNCIÓN: hashFNV
 *  Calcula un hash FNV-1a de 32 bits sobre un String.
 *  Este algoritmo es simple y rápido, adecuado para microcontroladores.
 *  Se usa para verificar que el contenido del paquete no fue alterado.
 *
 *  Parámetros:
 *    s — String de entrada
 *  Retorna:
 *    Valor hash de 32 bits sin signo
 * ================================================================ */
uint32_t hashFNV(const String &s) {
  uint32_t h = 0x811c9dc5UL; // Valor inicial FNV (constante del algoritmo)
  for (size_t i = 0; i < s.length(); i++) {
    h ^= (uint8_t)s[i];      // XOR con el byte actual
    h *= 0x01000193UL;        // Multiplica por el primo FNV
  }
  return h;
}

/* ================================================================
 *  FUNCIÓN: hashHex
 *  Convierte el hash FNV-1a a una cadena hexadecimal de 8 caracteres.
 *  Ejemplo: hashHex("hola") → "A3B2C1D0"
 *
 *  Parámetros:
 *    s — String de entrada
 *  Retorna:
 *    String con el hash en formato hexadecimal de 8 dígitos (mayúsculas)
 * ================================================================ */
String hashHex(const String &s) {
  char b[9]; // 8 caracteres hex + terminador nulo
  snprintf(b, 9, "%08X", hashFNV(s));
  return String(b);
}

/* ================================================================
 *  FUNCIÓN: parsearJSON
 *  Extrae los campos de un JSON recibido por radio y los guarda
 *  en una estructura Evidencia. También verifica la integridad
 *  del paquete comparando el hash recibido con el calculado.
 *
 *  El JSON esperado tiene esta forma:
 *  {"tipo":"NORMAL","acel":0.98,"bpm":72,"lat":20.1,"lon":-98.7,
 *   "sats":6,"fix":true,"ts":"2024-01-01T12:00:00","num":1,"hash":"AABBCCDD"}
 *
 *  Parámetros:
 *    raw — String con el JSON recibido sin procesar
 *    ev  — Referencia a la estructura donde se guardarán los datos
 *  Retorna:
 *    true si el parsing fue exitoso (campo "tipo" presente)
 *    false si el JSON está malformado o vacío
 * ================================================================ */
bool parsearJSON(const String &raw, Evidencia &ev) {

  // ── Función interna para extraer valores de tipo String ──────────
  // Busca el patrón "clave":"valor" y devuelve el valor.
  auto extraerStr = [&](const String &key) -> String {
    String k = "\"" + key + "\":\""; // Busca: "clave":"
    int i = raw.indexOf(k);
    if (i < 0) return "";            // Clave no encontrada
    i += k.length();                 // Avanza hasta el inicio del valor
    int j = raw.indexOf("\"", i);    // Busca la comilla de cierre
    return j < 0 ? "" : raw.substring(i, j);
  };

  // ── Función interna para extraer valores numéricos o booleanos ───
  // Busca el patrón "clave":valor y devuelve el valor como String.
  auto extraerNum = [&](const String &key) -> String {
    String k = "\"" + key + "\":"; // Busca: "clave":
    int i = raw.indexOf(k);
    if (i < 0) return "0";         // Clave no encontrada, devuelve "0"
    i += k.length();               // Avanza hasta el inicio del valor
    int j = i;
    // Avanza hasta encontrar una coma o llave de cierre (fin del valor)
    while (j < (int)raw.length() && raw[j] != ',' && raw[j] != '}') j++;
    return raw.substring(i, j);
  };

  // ── Extraer todos los campos del JSON ────────────────────────────
  ev.tipo   = extraerStr("tipo");
  ev.acel   = extraerNum("acel").toFloat();
  ev.bpm    = extraerNum("bpm").toInt();
  ev.lat    = extraerNum("lat").toDouble();
  ev.lon    = extraerNum("lon").toDouble();
  ev.sats   = extraerNum("sats").toInt();
  ev.fix    = (extraerNum("fix") == "true"); // Convierte "true"/"false" a bool
  ev.ts     = extraerStr("ts");
  ev.numero = extraerNum("num").toInt();
  ev.hash   = extraerStr("hash");

  // Si no se pudo extraer el tipo, el JSON es inválido
  if (ev.tipo.length() == 0) return false;

  // ── Verificación de integridad con hash ──────────────────────────
  // Se reconstruye el mismo JSON base que usó la pulsera para calcular
  // su hash, respetando el MISMO orden de campos y formato.
  // El campo "hash" en sí NO se incluye en el cálculo.
  String base = "{";
  base += "\"tipo\":\""  + ev.tipo            + "\",";
  base += "\"acel\":"    + extraerNum("acel") + ",";
  base += "\"bpm\":"     + extraerNum("bpm")  + ",";
  base += "\"lat\":"     + extraerNum("lat")  + ",";
  base += "\"lon\":"     + extraerNum("lon")  + ",";
  base += "\"sats\":"    + extraerNum("sats") + ",";
  base += "\"fix\":"     + extraerNum("fix")  + ",";
  base += "\"ts\":\""    + ev.ts              + "\",";
  base += "\"num\":"     + extraerNum("num")  + ",";
  // Nota: La pulsera también coloca una coma al final antes del hash.

  // Compara el hash calculado localmente contra el recibido en el paquete
  ev.hashOK = (hashHex(base) == ev.hash);

  return true;
}

/* ================================================================
 *  HELPERS DE PANTALLA OLED
 * ================================================================ */

/**
 * drawHdr — Dibuja la barra de encabezado en la parte superior.
 * Fondo blanco con texto negro y centrado, seguido de una línea divisoria.
 *
 * @param titulo  Texto a mostrar como título de la pantalla
 */
void drawHdr(const char *titulo) {
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE); // Barra blanca
  display.setTextColor(SSD1306_BLACK);             // Texto negro sobre blanco
  display.setTextSize(1);
  // Calcular el ancho del texto para centrarlo
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(titulo, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 2); // Centrar horizontalmente
  display.print(titulo);
  display.setTextColor(SSD1306_WHITE); // Restaurar color blanco
  display.drawLine(0, 12, 127, 12, SSD1306_WHITE); // Línea separadora
}

/**
 * drawDots — Dibuja los puntos de navegación en la parte inferior.
 * El punto de la pantalla actual se muestra sólido (relleno),
 * los demás se muestran como contorno vacío.
 */
void drawDots() {
  int dotW = 6, gap = 4; // Ancho de cada punto y separación entre ellos
  // Calcular la posición X de inicio para centrar todos los puntos
  int startX = (128 - (N_PANTALLAS * dotW + (N_PANTALLAS - 1) * gap)) / 2;
  for (int i = 0; i < N_PANTALLAS; i++) {
    int x = startX + i * (dotW + gap);
    if (i == pantActual)
      display.fillRect(x, 61, dotW, 3, SSD1306_WHITE); // Punto activo: sólido
    else
      display.drawRect(x, 61, dotW, 3, SSD1306_WHITE); // Punto inactivo: vacío
  }
}

/**
 * drawBarra — Dibuja una barra de progreso horizontal.
 * Se usa en la pantalla de estadísticas para mostrar el % de integridad.
 *
 * @param x, y  Posición superior-izquierda de la barra
 * @param w, h  Ancho y alto de la barra
 * @param pct   Porcentaje de relleno (0 a 100)
 */
void drawBarra(int x, int y, int w, int h, int pct) {
  display.drawRect(x, y, w, h, SSD1306_WHITE); // Borde exterior
  int fill = (pct * (w - 2)) / 100;            // Ancho de relleno proporcional
  if (fill > 0)
    display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE); // Relleno interior
}

/* ================================================================
 *  FUNCIONES DE PANTALLA
 *  Cada función dibuja una de las 4 pantallas disponibles.
 * ================================================================ */

/**
 * pantEspera — Pantalla 0 (inicial).
 * Se muestra cuando aún no se ha recibido ningún paquete.
 * Indica la configuración de radio activa y espera a la pulsera.
 */
void pantEspera() {
  display.clearDisplay();
  drawHdr("BASE LISTA");
  display.setTextSize(1);
  display.setCursor(0, 16); display.print("915MHz SF8 125kHz");
  display.setCursor(0, 28); display.print("Esperando pulsera...");
  display.setCursor(0, 40); display.print("Actualizado V2.1");
  drawDots();
  display.display();
}

/**
 * pant_alerta — Pantalla 0 (principal, post-recepción).
 * Muestra el resumen del ÚLTIMO paquete recibido:
 *   - Tipo de evento con animación parpadeante si es alerta
 *   - Número de paquete y estado del hash
 *   - Aceleración (G) y ritmo cardíaco (BPM)
 *   - Estado GPS (satélites y fix)
 *   - RSSI y SNR de la señal LoRa
 */
void pant_alerta() {
  // Determinar si el último evento es una alerta crítica
  bool esAlerta = ultimo.tipo.indexOf("PANICO")   >= 0 ||
                  ultimo.tipo.indexOf("CAIDA")    >= 0 ||
                  ultimo.tipo.indexOf("FORCEJEO") >= 0;

  display.clearDisplay();

  if (esAlerta) {
    // Animación de parpadeo: invierte el encabezado cada 450 ms
    static bool inv = false;
    static unsigned long tI = 0;
    if (millis() - tI > 450) { inv = !inv; tI = millis(); }

    if (inv) {
      // Estado invertido: fondo blanco con texto negro
      display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setTextSize(1);
    display.setCursor(2, 2);
    String h = "!! " + ultimo.tipo + " !!";         // Formato: !! TIPO !!
    if (h.length() > 21) h = h.substring(0, 21);    // Truncar si no cabe
    display.print(h);
    display.setTextColor(SSD1306_WHITE);
    display.drawLine(0, 12, 127, 12, SSD1306_WHITE);
  } else {
    // Sin alerta: encabezado normal con el tipo de evento
    drawHdr(ultimo.tipo.c_str());
  }

  display.setTextSize(1);

  // Línea 1: Número de paquete y resultado de verificación de hash
  display.setCursor(0, 15);
  display.print("#" + String(ultimo.numero) + "  " +
                (ultimo.hashOK ? "HASH OK" : "! HASH ERR"));

  // Línea 2: Aceleración en g y ritmo cardíaco en BPM
  display.setCursor(0, 25);
  display.print("G:" + String(ultimo.acel, 2) + "g  BPM:" + String(ultimo.bpm));

  // Línea 3: Estado GPS (satélites visibles y si tiene fix)
  display.setCursor(0, 35);
  display.print("GPS:" + String(ultimo.sats) + "sat " +
                (ultimo.fix ? "FIX" : "---"));

  // Línea 4: Calidad de señal LoRa
  display.setCursor(0, 45);
  display.print("RSSI:" + String((int)ultimo.rssi) +
                " SNR:" + String((int)ultimo.snr));

  drawDots();
  display.display();
}

/**
 * pant_gps — Pantalla 1.
 * Muestra información detallada de GPS:
 *   - Número de satélites y barra de progreso
 *   - Coordenadas lat/lon (si hay fix)
 *   - Mensaje de ayuda si no hay fix GPS
 */
void pant_gps() {
  display.clearDisplay();
  drawHdr("GPS");
  display.setTextSize(1);

  // Fix válido requiere señal Y coordenadas distintas de cero
  bool fix = ultimo.fix && (ultimo.lat != 0.0 || ultimo.lon != 0.0);

  // Mostrar número de satélites y estado de fix
  char ss[24];
  snprintf(ss, sizeof(ss), "Sats: %d  [%s]",
           ultimo.sats, fix ? "FIX" : "---");
  display.setCursor(0, 15);
  display.print(ss);

  // Barra de calidad GPS: se llena conforme aumentan los satélites (máx 12)
  int bw = (ultimo.sats > 12) ? 126 : (ultimo.sats * 126) / 12;
  display.drawRect(0, 25, 128, 6, SSD1306_WHITE);
  if (bw > 0) display.fillRect(1, 26, bw, 4, SSD1306_WHITE);

  if (fix) {
    // Con fix: mostrar coordenadas con 5 decimales de precisión (~1m)
    char ls[22], os[22];
    snprintf(ls, sizeof(ls), "Lat:%.5f", ultimo.lat);
    snprintf(os, sizeof(os), "Lon:%.5f", ultimo.lon);
    display.setCursor(0, 34); display.print(ls);
    display.setCursor(0, 44); display.print(os);
  } else {
    // Sin fix: sugerir salir al exterior para mejorar señal GPS
    display.setCursor(14, 34); display.print("Sin fix GPS");
    display.setCursor(14, 44); display.print("Sal al exterior");
  }

  drawDots();
  display.display();
}

/**
 * pant_stats — Pantalla 2.
 * Muestra estadísticas acumuladas de la sesión:
 *   - Total de paquetes recibidos
 *   - Paquetes corruptos (parse fallido o hash incorrecto)
 *   - Porcentaje de integridad con barra de progreso
 */
void pant_stats() {
  display.clearDisplay();
  drawHdr("ESTADISTICAS");
  display.setTextSize(1);

  int ok  = totalRecibidos - totalCorruptos;
  int pct = totalRecibidos > 0 ? (ok * 100) / totalRecibidos : 0;

  display.setCursor(0, 15); display.print("Recibidos : " + String(totalRecibidos));
  display.setCursor(0, 25); display.print("Corruptos : " + String(totalCorruptos));
  display.setCursor(0, 35); display.print("Integridad: " + String(pct) + "%");

  // Barra de progreso visual para el porcentaje de integridad
  drawBarra(0, 45, 128, 8, pct);

  drawDots();
  display.display();
}

/**
 * pant_historial — Pantalla 3.
 * Muestra los últimos eventos recibidos (hasta HIST_SIZE = 5).
 * Los eventos de alerta se marcan con "!" al inicio de la línea.
 * Los más recientes aparecen primero.
 */
void pant_historial() {
  display.clearDisplay();
  drawHdr("HISTORIAL");
  display.setTextSize(1);

  int count = min(totalRecibidos, HIST_SIZE); // Cuántos hay disponibles
  for (int i = 0; i < count; i++) {
    // Calcular el índice en el arreglo circular (más reciente primero)
    int idx = (totalRecibidos - 1 - i + HIST_SIZE) % HIST_SIZE;

    // Verificar si ese evento fue una alerta
    bool al = historial[idx].tipo.indexOf("PANICO")   >= 0 ||
              historial[idx].tipo.indexOf("CAIDA")    >= 0 ||
              historial[idx].tipo.indexOf("FORCEJEO") >= 0;

    // Truncar el tipo a 7 caracteres para que quepa en pantalla
    String t = historial[idx].tipo;
    if (t.length() > 7) t = t.substring(0, 7);

    // Formato: [!] #NUM TIPO G:X.X
    String lin = (al ? "!" : " ") +
                 String("#") + String(historial[idx].numero) +
                 " " + t +
                 " G:" + String(historial[idx].acel, 1);

    display.setCursor(0, 14 + i * 10); // 10 px de separación entre líneas
    display.print(lin);
  }

  // Si no hay ningún evento aún
  if (count == 0) {
    display.setCursor(20, 30);
    display.print("Sin eventos");
  }

  drawDots();
  display.display();
}

/**
 * dibujarPantalla — Punto de despacho de pantallas.
 * Decide qué función de pantalla llamar según el estado actual:
 *   - Sin datos → pantalla de espera
 *   - Con datos → pantalla según pantActual
 */
void dibujarPantalla() {
  if (totalRecibidos == 0) { pantEspera(); return; }
  switch (pantActual) {
    case 0: pant_alerta();    break; // Pantalla principal / alerta
    case 1: pant_gps();       break; // Detalles GPS
    case 2: pant_stats();     break; // Estadísticas de recepción
    case 3: pant_historial(); break; // Historial de eventos
  }
}

/* ================================================================
 *  BOTÓN Y LED
 * ================================================================ */

/**
 * leerBoton — Detecta pulsación del botón físico (GPIO 0).
 * Cambia a la siguiente pantalla con debounce de 300 ms para
 * evitar múltiples cambios por una sola pulsación.
 * El botón usa INPUT_PULLUP, por lo que LOW = presionado.
 */
void leerBoton() {
  bool btn = (digitalRead(0) == LOW); // LOW = botón presionado
  if (btn && !btnPresionado && millis() - tBtn > 300) {
    btnPresionado = true;
    tBtn          = millis();
    pantActual    = (pantActual + 1) % N_PANTALLAS; // Avanza a la siguiente pantalla (circular)
    hayDato       = true; // Forzar redibujo inmediato
  }
  if (!btn) btnPresionado = false; // Liberar cuando se suelta el botón
}

/**
 * parpadearLED — Parpadeo de "heartbeat" del LED cuando no hay datos.
 * Da un destello breve cada 2.5 segundos para indicar que el sistema
 * está encendido y activo aunque no haya recibido nada aún.
 * Se detiene automáticamente una vez que se recibe el primer paquete.
 */
void parpadearLED() {
  if (totalRecibidos == 0 && millis() - tBlink > 2500) {
    tBlink = millis();
    digitalWrite(LED_PIN, HIGH); delay(25); // Destello de 25 ms
    digitalWrite(LED_PIN, LOW);
  }
}

/* ================================================================
 *  RECEPCIÓN LORA
 * ================================================================ */

/**
 * recibirLoRa — Intenta recibir un paquete LoRa y procesarlo.
 *
 * Flujo:
 *   1. Intentar recibir (no bloqueante); salir si no hay nada.
 *   2. Validar longitud mínima del paquete (4 bytes de encabezado).
 *   3. Verificar que el paquete va dirigido a ESTE receptor
 *      y que proviene de la pulsera esperada.
 *   4. Extraer el payload JSON del paquete.
 *   5. Parsear y validar el JSON.
 *   6. Guardar en historial y actualizar estadísticas.
 *   7. Parpadear el LED: 5 veces (alerta) o 2 veces (normal).
 *
 * Formato del paquete LoRa (bytes):
 *   [0] Dirección destino  (debe ser dirLocal   = 0xD3)
 *   [1] Dirección origen   (debe ser dirPulsera  = 0xC1)
 *   [2] Flags / reservado
 *   [3] Longitud del payload
 *   [4..n] Payload JSON
 */
void recibirLoRa() {
  byte buf[256];
  // Intentar recibir sin bloquear; si no hay paquete, retornar
  if (radio.receive(buf, 0) != RADIOLIB_ERR_NONE) return;

  int len = radio.getPacketLength();
  if (len < 4) return; // Paquete demasiado corto para ser válido

  // Filtrar por direcciones: descartar paquetes que no son para este receptor
  if (buf[0] != dirLocal)   return; // No es para nosotros
  if (buf[1] != dirPulsera) return; // No viene de la pulsera esperada

  // Extraer y validar la longitud declarada del payload
  byte payLen = buf[3];
  if (len < (int)(4 + payLen)) return; // El paquete está truncado

  // Construir el String del payload copiando los bytes del JSON
  String payload = "";
  payload.reserve(payLen);
  for (int i = 4; i < 4 + payLen; i++) payload += (char)buf[i];

  Serial.println(F("─────────────────────────────────────────"));
  Serial.print(F("RX: ")); Serial.println(payload);

  // Crear una nueva Evidencia y leer la calidad de señal del módulo
  Evidencia ev;
  ev.rssi = radio.getRSSI(); // Nivel de señal recibida (dBm, valores negativos)
  ev.snr  = radio.getSNR();  // Relación señal/ruido (dB)

  // Intentar parsear el JSON; si falla, contar como corrupto y salir
  if (!parsearJSON(payload, ev)) {
    totalCorruptos++;
    Serial.println(F("Parse fallido"));
    return;
  }

  // Guardar como último evento y actualizar estadísticas
  ultimo = ev;
  totalRecibidos++;
  hayDato = true;
  historial[(totalRecibidos - 1) % HIST_SIZE] = ev; // Guardar en historial circular

  // Si el hash falló, también contar como corrupto (datos alterados)
  if (!ev.hashOK) totalCorruptos++;

  // Imprimir resumen por Serial para depuración
  Serial.printf("✓ #%d [%s] G:%.2f BPM:%d GPS:%d Fix:%s Hash:%s RSSI:%.0f\n",
    ev.numero, ev.tipo.c_str(), ev.acel, ev.bpm, ev.sats,
    ev.fix    ? "SI"  : "NO",
    ev.hashOK ? "OK"  : "ERR",
    ev.rssi);
  Serial.println(F("─────────────────────────────────────────"));

  // Indicar visualmente en el LED: alerta = 5 parpadeos largos, normal = 2 rápidos
  bool alerta = ev.tipo.indexOf("PANICO")   >= 0 ||
                ev.tipo.indexOf("CAIDA")    >= 0 ||
                ev.tipo.indexOf("FORCEJEO") >= 0;
  int blinks = alerta ? 5 : 2;
  for (int k = 0; k < blinks; k++) {
    digitalWrite(LED_PIN, HIGH); delay(alerta ? 120 : 50); // Duración del destello
    digitalWrite(LED_PIN, LOW);  delay(50);                 // Pausa entre destellos
  }
}

/* ================================================================
 *  SETUP — Se ejecuta UNA VEZ al encender o resetear el dispositivo
 * ================================================================ */
void setup() {
  // Iniciar comunicación serial para depuración
  Serial.begin(115200);
  delay(500);

  // Configurar pines de entrada/salida
  pinMode(0, INPUT_PULLUP);        // Botón de navegación (activo en LOW)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);      // LED apagado al inicio

  // Iniciar I2C en pines SDA=4, SCL=15 (pines no estándar en este hardware)
  Wire.begin(4, 15);

  // Reset manual de la pantalla OLED por hardware
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);  delay(50);  // Mantener en LOW para reset
  digitalWrite(OLED_RESET, HIGH); delay(50);  // Liberar para operar

  // Inicializar la pantalla OLED (dirección I2C: 0x3C)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED ERROR"));
    while (true) delay(1000); // Bloquear si la pantalla no responde
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // ── Pantalla de bienvenida ────────────────────────────────────────
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE); // Marco exterior
  display.drawRect(2, 2, 124, 60, SSD1306_WHITE); // Marco interior
  display.setTextSize(2);
  display.setCursor(10, 8);  display.print("INSTITUTO");
  display.setCursor(5, 28);  display.print("DE LA MUJER");
  display.setTextSize(1);
  display.setCursor(15, 52); display.print("Receptor LoRa V2.1");
  display.display();
  delay(1800); // Mostrar bienvenida por 1.8 segundos

  // ── Inicializar SPI y el módulo LoRa ─────────────────────────────
  // Pines SPI: SCK=5, MISO=19, MOSI=27, NSS=18
  SPI.begin(5, 19, 27, 18);
  Serial.print(F("LoRa SX1276 915MHz... "));
  int st = radio.begin(FREQUENCY, BANDWIDTH, SPREAD_FACTOR, CODING_RATE);

  if (st == RADIOLIB_ERR_NONE) {
    radio.setCRC(true); // Activar CRC de hardware para detección de errores
    Serial.println(F("OK"));
  } else {
    // Error al inicializar LoRa: mostrar código de error y detener
    Serial.printf("ERROR %d\n", st);
    display.clearDisplay();
    display.setCursor(0, 20);
    display.print("ERROR LORA: " + String(st));
    display.display();
    while (true) delay(1000);
  }

  Serial.println(F("Escuchando 915MHz SF8\n"));
  pantEspera(); // Mostrar pantalla de espera mientras no haya datos
}

/* ================================================================
 *  LOOP — Se ejecuta continuamente después de setup()
 *
 *  El ciclo principal hace tres cosas en orden:
 *    1. Leer el botón de navegación
 *    2. Intentar recibir un paquete LoRa
 *    3. Redibujar la pantalla si hay datos nuevos o cada 500 ms
 *    4. Parpadear el LED de heartbeat si no hay datos aún
 * ================================================================ */
void loop() {
  leerBoton();    // Detectar pulsación del botón para cambiar pantalla
  recibirLoRa();  // Intentar recibir y procesar un paquete LoRa

  // Redibujar la pantalla si llegó dato nuevo O han pasado 500 ms
  static unsigned long tRef = 0;
  if (hayDato || millis() - tRef > 500) {
    tRef    = millis();
    hayDato = false;
    dibujarPantalla();
  }

  parpadearLED(); // Heartbeat del LED cuando no hay datos
  delay(10);      // Pequeña pausa para no saturar el procesador
}
