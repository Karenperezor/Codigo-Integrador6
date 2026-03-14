# Codigo-Integrador6

IoT Implementation for Women’s Safety

## Arquitectura del Sistema
El sistema se basa en una arquitectura IoT compuesta por:

Dispositivo Portátil (Wearable): Integra sensores y módulos de comunicación.
Sensores: Recopilan datos del entorno y estado físico.
Comunicación LoRa: Transmisión de largo alcance y bajo consumo.
Sistema Receptor: Dispositivo o servidor que recibe las alertas.


# Tabla de Conexiones General

| Sensor / Módulo | SDA / RX | SCL / TX | Bus / Notas |
|-----------------|----------|----------|-------------|
| MPU6050 | GPIO 41 | GPIO 42 | Wire2 (I2C) |
| RTC DS3231 | GPIO 38 | GPIO 39 | Wire3 (I2C) |
| MAX30102 | GPIO 1 | GPIO 2 | Wire (I2C) |
| GPS NEO-6M | GPIO 45 | GPIO 46 | Serial1 |
| Botón de Pánico | GPIO 0 | GND | Pull-up Interno |

---

# Especificaciones Técnicas de Sensores

A continuación se detallan los pines de conexión, datos capturados y especificaciones técnicas de cada componente.

---

## 1. MPU6050 (Acelerómetro / Giroscopio)

Detector de movimientos bruscos, caídas y forcejeos.
**Dirección I2C:** `0x69` (AD0 conectado a 3.3V)

### Datos Técnicos

| Dato Capturado | Unidad | Rango | Condición de Alerta |
|----------------|------|------|----------------|
| Aceleración (X, Y, Z) | g (gravedad) | ±2g a ±16g | Acc > 2g (Forcejeo/Caída) |
| Velocidad Angular (X, Y, Z) | °/s | ±250 a ±2000 °/s | Cambios bruscos de orientación |

⚠️ **Nota:** Si AD0 va a GND, la dirección cambia a `0x68`.

---

## 2. MAX30102 (Sensor Biométrico)

Monitor de frecuencia cardíaca y oxigenación para detectar estrés físico.
**Dirección I2C:** `0x57`

### Datos Técnicos

| Dato Capturado | Unidad | Rango | Condición de Alerta |
|----------------|------|------|----------------|
| Frecuencia Cardíaca (BPM) | BPM | 30 – 240 | BPM > 110 (Estrés/Miedo) |
| Oxígeno en Sangre (SpO₂) | % | 70 – 100% | SpO₂ < 92% (Alerta médica) |

---

## 3. GPS NEO-6M (Geolocalización)

Obtención de coordenadas exactas para la respuesta de emergencia.
**Comunicación:** UART (Serial) a **9600 baudios**


### Datos Técnicos

| Dato Capturado | Unidad | Precisión | Uso |
|----------------|--------|-----------|-----|
| Latitud | Grados | ~2.5 m | Ubicación del evento |
| Longitud | Grados | ~2.5 m | Ubicación del evento |
| Hora UTC | hh:mm:ss | Alta | Sincronización |

 **Importante:** Cruzar conexiones (**TX → RX**).  
Puede tardar **1–5 minutos** en obtener **FIX** la primera vez.

---

## 4. DS3231 (Módulo de Tiempo Real - RTC)

Registro preciso de fecha y hora para evidencia legal.
**Dirección I2C:** `0x68`

### Datos Técnicos

| Dato Capturado | Formato | Función |
|---------------|--------|--------|
| Fecha | AAAA-MM-DD | Marca temporal histórica |
| Hora | hh:mm:ss | Registro exacto del incidente |

---

## 5. Botón de Pánico (SOS)

Activación manual de la alerta.

### Datos Técnicos

| Dato Capturado | Valor | Significado |
|---------------|------|-------------|
| Estado SOS | 0 | Estado Normal |
| Estado SOS | 1 | ALERTA ACTIVADA |

---

# Formato de Datos de Salida

El dispositivo transmite un paquete de datos estructurado (**JSON**) al detectar una emergencia o al presionar el botón.

```json
{
  "SOS": 1,
  "lat": 20.123456,
  "lon": -99.213456,
  "BPM": 110,
  "SpO2": 95,
  "AccX": 2.2,
  "AccY": 1.3,
  "AccZ": 0.8,
  "GyroX": 45,
  "GyroY": 12,
  "GyroZ": 8,
  "fecha": "2026-03-12",
  "hora": "21:45:20"
}
````

---

#  Impacto Esperado

El proyecto busca contribuir al desarrollo de herramientas tecnológicas orientadas a la protección y seguridad personal, proporcionando mecanismos que permitan:

* Actuar con mayor rapidez ante situaciones de riesgo.
* Registrar información objetiva sobre incidentes.
* Fortalecer el apoyo institucional en casos de emergencia.
* Reducir la revictimización mediante evidencia digital irrefutable.

```
```
