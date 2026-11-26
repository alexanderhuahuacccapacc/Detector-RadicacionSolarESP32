// ESP32 DevKit/WROOM clasico. Pines segun Esquemas_registrador_solar_ESP32.pdf.
// Resistencia total del shunt usada para el calculo: 1.5 ohmios.
// La lectura electrica NO equivale a una calibracion de irradiancia.
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_ADS1X15.h>
#include <BH1750.h>
#include <RTClib.h>
#include <DHT.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <atomic>
#include <esp_random.h>
#include <math.h>
#include <sys/time.h>
#include "CertificadoMawi.h"

#define SD_CS 13
#define SD_SCK 14
#define SD_MOSI 27
#define SD_MISO 25
#define DHTPIN 4
#define DHTTYPE DHT22

const float R_SHUNT = 1.5f;         // Medicion del conjunto en R1.
const float P_SHUNT_NOM_W = 0.25f;  // Potencia nominal de la resistencia.
const float VOLT_MAX = 0.95f;       // Aviso; NO es proteccion electrica.
const float RUIDO_V = 0.002f;       // Ruido de 2 mV: se trata como 0.
const int RAW_SATURADO = 32000;     // Umbral conservador cercano a 1 V.
const int MUESTRAS = 8;
const unsigned long INTERVALO_MS = 5000;
const unsigned long TIMEOUT_ADC_MS = 100;
const unsigned long REINTENTO_WIFI_MS = 5000;
const char* const CLAVES_RED[] = {"wifi", "wifi2", "wifi3", "wifi4"};
const char* URL_MAWI = "https://mawi-radiacion-solar.keentangerine256.chatgpt.site/api/lectura";
const int MAX_PENDIENTES = 60;
const time_t PRIMERA_FECHA_TLS = 1704067200;
const int DIFERENCIA_PET_SEG = 5 * 3600;

struct LecturaNube {
  char json[384];
};

// ---- Calibracion ----
// ISC_EQ_1000_A = (I_medida_A - I_CERO_A) * 1000 / G_referencia_Wm2.
// Calibracion PRELIMINAR (1 punto): 0.13 A medidos con multimetro (rango 10 A)
// y 669.5 W/m2 de referencia Open-Meteo -> 0.13 * 1000 / 669.5 = 0.194 A.
// Mejorar con mas mediciones a distintas horas y promediar.
const bool CALIBRADO = true;
// Valores de fabrica. Se pueden recalibrar por USB con los comandos
// "cero" y "calibrar W/m2"; lo guardado en la memoria del ESP32 tiene prioridad.
const float ISC_EQ_1000_DEF_A = 0.194f;
const float I_CERO_DEF_A = 0.0f;
float iscEq1000A = ISC_EQ_1000_DEF_A;
float iCeroA = I_CERO_DEF_A;
// ---- Respaldo: radiacion estimada con el luxometro (BH1750) + DHT22 ----
// Eficacia luminosa global tipica: 105-128 lm/W, ~110 lm/W para cualquier cielo
// (Muneer y Angus). W/m2 = lux / K. El punto de rocio (de T y HR del DHT22) da el
// agua precipitable (Perez: W = exp(0.07*Td - 0.075) cm) y ajusta K unos pocos %.
// Es una ESTIMACION (+-20% por el sensor y el cielo). Se usa solo si el panel marca
// casi 0 mientras el luxometro indica luz, y queda anotada como ESTIMADO_POR_LUX.
const bool USAR_ESTIMADO_LUX = true;
const float K_LUZ_LM_W = 110.0f;
// K en uso: 110 lm/W de fabrica, o la calibrada en el lugar con el comando "kluz W/m2".
float kLuz = K_LUZ_LM_W;
// Valor PROVISIONAL (solo se usa si CALIBRADO = false).
const float ISC_PROVISIONAL_A = 0.30f;

// Poner la hora: escribe en el monitor serie:  hora 2026-11-17 16:20:00
// Buscar dispositivos I2C: escribe en el monitor serie:  i2c
const bool AJUSTAR_RTC_AL_COMPILAR = false;
const char* ARCHIVO = "/datos_v2.csv";
const char* CABECERA =
  "fecha_hora_PET,tiempo_ms,raw_promedio,raw_max,tension_ADC_mV,corriente_mA,"
  "potencia_shunt_W,irradiancia_Wm2,estado_panel,lux,estado_lux,"
  "temp_C,humedad_pct,estado_DHT";

SPIClass sdSPI(HSPI);
Adafruit_ADS1115 ads;
BH1750 luxometro;
RTC_DS3231 rtc;
DHT dht(DHTPIN, DHTTYPE);
bool adsOk = false, luxOk = false, rtcOk = false, sdOk = false;
unsigned long ultimo = 0;
std::atomic<bool> redesCambiaron{true};
// Red que el usuario acaba de aceptar: la tarea de red intenta esa primero.
std::atomic<bool> hayPreferida{false};
char redPreferida[33] = "";
// Evita que un escaneo manual choque con el intento de conexion de tareaRed.
SemaphoreHandle_t mutexWiFi = nullptr;
QueueHandle_t colaNube = nullptr;
uint32_t sesionNube[2];
uint32_t secuenciaNube = 0;

String num(float v, int d) {
  return isfinite(v) ? String(v, d) : String("");
}

String dato(float v, int d, const char* unidad) {
  return isfinite(v) ? String(v, d) + " " + unidad : String("sin dato");
}

bool presente(uint8_t direccion) {
  Wire.beginTransmission(direccion);
  return Wire.endTransmission() == 0;
}

// Lista los dispositivos que responden en el bus I2C.
void escanearI2C() {
  Serial.println("Buscando dispositivos I2C...");
  int total = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (!presente(a)) continue;
    total++;
    Serial.print("  Encontrado: 0x");
    Serial.print(a, HEX);
    if (a == 0x23) Serial.print("  (BH1750)");
    else if (a == 0x48) Serial.print("  (ADS1115)");
    else if (a == 0x68) Serial.print("  (DS3231 reloj)");
    else if (a == 0x57) Serial.print("  (EEPROM del reloj)");
    Serial.println();
  }
  if (total == 0) Serial.println("  Ninguno. Revisa SDA(D21), SCL(D22), 3V3 y GND.");
  if (!presente(0x48)) Serial.println("  FALTA el ADS1115 (0x48): revisa sus cables.");
  Serial.println("Fin de la busqueda.");
}

String fechaISO() {
  if (!rtcOk || !presente(0x68) || rtc.lostPower()) return "sin_reloj_valido";
  DateTime ahora = rtc.now();
  return ahora.isValid() ? ahora.timestamp() : String("sin_reloj_valido");
}

bool iniciarADS() {
  bool ok = ads.begin(0x48, &Wire);
  if (ok) {
    ads.setGain(GAIN_FOUR);  // Rango de medida +-1.024 V, no limite de proteccion.
    ads.setDataRate(RATE_ADS1115_128SPS);
  }
  return ok;
}

void iniciarSensores() {
  adsOk = iniciarADS();
  luxOk = luxometro.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  rtcOk = rtc.begin(&Wire);
  if (rtcOk && AJUSTAR_RTC_AL_COMPILAR) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("RTC ajustado a la hora de compilacion.");
  }
  if (rtcOk && rtc.lostPower()) {
    Serial.println("RTC sin hora fiable: escribe  hora AAAA-MM-DD HH:MM:SS");
  }
  dht.begin();
}

void iniciarSD() {
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk = SD.begin(SD_CS, sdSPI);
}

bool cargarRedes(WiFiMulti &redes) {
  redes.APlistClean();
  Preferences ajustes;
  if (!ajustes.begin("registrador", true)) return false;
  bool configurado = false;
  for (const char* llave : CLAVES_RED) {
    if (!ajustes.isKey(llave)) continue;
    String credenciales = ajustes.getString(llave);
    int separador = credenciales.indexOf('\t');
    if (separador < 1) continue;
    String ssid = credenciales.substring(0, separador);
    String clave = credenciales.substring(separador + 1);
    if (redes.addAP(ssid.c_str(), clave.isEmpty() ? nullptr : clave.c_str())) configurado = true;
  }
  return configurado;
}

void configurarWiFi(const String &credenciales) {
  int separador = credenciales.indexOf('\t');
  if (separador < 1 || separador > 32 || credenciales.length() - separador - 1 > 64) {
    Serial.println("Formato Wi-Fi: wifi RED<TAB>CLAVE");
    return;
  }
  Preferences ajustes;
  if (!ajustes.begin("registrador")) {
    Serial.println("Wi-Fi: error al guardar la configuracion.");
    return;
  }
  const char* ranura = nullptr;
  const String ssid = credenciales.substring(0, separador);
  for (const char* llave : CLAVES_RED) {
    if (!ajustes.isKey(llave)) {
      if (!ranura) ranura = llave;
      continue;
    }
    const String existente = ajustes.getString(llave);
    if (existente.startsWith(ssid + "\t")) {
      ranura = llave;
      break;
    }
  }
  if (!ranura) {
    Serial.println("Wi-Fi: las cuatro ranuras estan ocupadas.");
    return;
  }
  if (ajustes.putString(ranura, credenciales) != credenciales.length()) {
    Serial.println("Wi-Fi: error al guardar la configuracion.");
    return;
  }
  strlcpy(redPreferida, ssid.c_str(), sizeof(redPreferida));
  hayPreferida = true;
  redesCambiaron = true;
  Serial.println("Wi-Fi: red guardada " + ssid);
}

// Busca la clave guardada de un SSID. Devuelve false si no esta registrado.
bool credencialesDe(const String &ssid, String &clave) {
  Preferences ajustes;
  if (!ajustes.begin("registrador", true)) return false;
  for (const char* llave : CLAVES_RED) {
    if (!ajustes.isKey(llave)) continue;
    const String guardada = ajustes.getString(llave);
    if (guardada.startsWith(ssid + "\t")) {
      clave = guardada.substring(ssid.length() + 1);
      return true;
    }
  }
  return false;
}

void borrarWiFi(const String &ssid) {
  Preferences ajustes;
  if (ssid.isEmpty() || !ajustes.begin("registrador")) {
    Serial.println("Wi-Fi: error al guardar la configuracion.");
    return;
  }
  for (const char* llave : CLAVES_RED) {
    if (!ajustes.isKey(llave)) continue;
    if (!ajustes.getString(llave).startsWith(ssid + "\t")) continue;
    ajustes.remove(llave);
    if (ssid == redPreferida) hayPreferida = false;
    redesCambiaron = true;
    Serial.println("Wi-Fi: red borrada " + ssid);
    return;
  }
  Serial.println("Wi-Fi: red no encontrada " + ssid);
}

// Respuesta: WIFI_ACTUAL (si hay conexion) y una linea WIFI_GUARDADA por ranura.
// Nunca se envian las claves.
void listarWiFi() {
  Serial.println("WIFI_LISTA_INI");
  if (WiFi.status() == WL_CONNECTED) Serial.println("WIFI_ACTUAL\t" + WiFi.SSID());
  Preferences ajustes;
  if (ajustes.begin("registrador", true)) {
    for (int i = 0; i < 4; i++) {
      if (!ajustes.isKey(CLAVES_RED[i])) continue;
      const String guardada = ajustes.getString(CLAVES_RED[i]);
      const int separador = guardada.indexOf('\t');
      if (separador < 1) continue;
      Serial.printf("WIFI_GUARDADA\t%d\t%s\n", i + 1, guardada.substring(0, separador).c_str());
    }
  }
  Serial.println("WIFI_LISTA_FIN");
}

// Respuesta: una linea WIFI_RED<TAB>dBm<TAB>abierta(0/1)<TAB>SSID por red distinta.
void escanearWiFi() {
  Serial.println("WIFI_SCAN_INI");
  if (mutexWiFi) xSemaphoreTake(mutexWiFi, portMAX_DELAY);
  const int total = WiFi.scanNetworks(false, false);
  if (mutexWiFi) xSemaphoreGive(mutexWiFi);
  String vistas = "\n";
  int mostradas = 0;
  for (int i = 0; i < total && mostradas < 15; i++) {  // Llegan ordenadas por senal.
    const String ssid = WiFi.SSID(i);
    if (ssid.isEmpty() || ssid.indexOf('\t') >= 0 || ssid.indexOf('\n') >= 0 ||
        ssid.indexOf('\r') >= 0 || vistas.indexOf("\n" + ssid + "\n") >= 0) continue;
    vistas += ssid + "\n";
    Serial.printf("WIFI_RED\t%d\t%d\t%s\n", (int)WiFi.RSSI(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? 1 : 0, ssid.c_str());
    mostradas++;
  }
  WiFi.scanDelete();
  Serial.println("WIFI_SCAN_FIN");
}

// Conecta de inmediato a la red recien aceptada, sin esperar a WiFiMulti.
void conectarPreferida() {
  const String ssid = redPreferida;
  String clave;
  if (ssid.isEmpty() || !credencialesDe(ssid, clave)) return;
  if (mutexWiFi) xSemaphoreTake(mutexWiFi, portMAX_DELAY);
  WiFi.begin(ssid.c_str(), clave.isEmpty() ? nullptr : clave.c_str());
  const unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 10000) vTaskDelay(pdMS_TO_TICKS(200));
  if (mutexWiFi) xSemaphoreGive(mutexWiFi);
  if (WiFi.status() != WL_CONNECTED) Serial.println("Wi-Fi: no se pudo conectar a " + ssid);
}

void configurarNube(const String &token) {
  if (token.length() != 64 || strspn(token.c_str(), "0123456789abcdef") != 64) {
    Serial.println("Nube: clave invalida.");
    return;
  }
  Preferences ajustes;
  if (!ajustes.begin("registrador") || ajustes.putString("nube_token", token) != token.length()) {
    Serial.println("Nube: error al guardar la clave.");
    return;
  }
  Serial.println("Nube: clave guardada.");
}

String claveNube() {
  Preferences ajustes;
  if (!ajustes.begin("registrador", true) || !ajustes.isKey("nube_token")) return "";
  return ajustes.getString("nube_token");
}

int publicarNube(const LecturaNube &lectura, const String &token) {
  NetworkClientSecure cliente;
  cliente.setCACert(CERTIFICADO_MAWI);
  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(4000);
  if (!http.begin(cliente, URL_MAWI)) return -1;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + token);
  const int respuesta = http.POST(String(lectura.json));
  http.end();
  return respuesta;
}

void tareaRed(void*) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFiMulti redes;
  bool configurado = false;
  bool conectado = false;
  bool tienePendiente = false;
  LecturaNube pendiente;
  int ultimoHTTP = 0;

  for (;;) {
    if (redesCambiaron.exchange(false)) {
      configurado = cargarRedes(redes);
      if (conectado) WiFi.disconnect();
      conectado = false;
      if (!configurado) Serial.println("Wi-Fi sin configurar. Por USB envia: wifi RED<TAB>CLAVE");
      else if (hayPreferida.exchange(false)) conectarPreferida();
    }
    bool enlazado = configurado && WiFi.status() == WL_CONNECTED;
    if (configurado && !enlazado) {
      if (mutexWiFi) xSemaphoreTake(mutexWiFi, portMAX_DELAY);
      enlazado = redes.run(4000) == WL_CONNECTED;
      if (mutexWiFi) xSemaphoreGive(mutexWiFi);
    }
    if (!enlazado) {
      if (conectado) Serial.println("Wi-Fi desconectado. El registro continua.");
      conectado = false;
      vTaskDelay(pdMS_TO_TICKS(REINTENTO_WIFI_MS));
      continue;
    }
    if (!conectado) {
      conectado = true;
      Serial.print("Wi-Fi conectado. IP: ");
      Serial.println(WiFi.localIP());
      configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
    }
    const String token = claveNube();
    if (token.isEmpty() || time(nullptr) < PRIMERA_FECHA_TLS) {
      vTaskDelay(pdMS_TO_TICKS(REINTENTO_WIFI_MS));
      continue;
    }
    if (!tienePendiente) {
      tienePendiente = xQueueReceive(colaNube, &pendiente, pdMS_TO_TICKS(300)) == pdTRUE;
    }
    if (!tienePendiente) continue;
    const int respuesta = publicarNube(pendiente, token);
    if (respuesta != ultimoHTTP) {
      Serial.printf("Nube: HTTP %d\n", respuesta);
      ultimoHTTP = respuesta;
    }
    if (respuesta == 204 || respuesta == 400 || respuesta == 413) {
      tienePendiente = false;
      vTaskDelay(pdMS_TO_TICKS(100));
    } else {
      vTaskDelay(pdMS_TO_TICKS(REINTENTO_WIFI_MS));
    }
  }
}

void iniciarRed() {
  if (rtcOk && !rtc.lostPower()) {
    const DateTime ahora = rtc.now();
    if (ahora.isValid() && ahora.year() >= 2024) {
      struct timeval horaUTC = {time_t(ahora.unixtime()) + DIFERENCIA_PET_SEG, 0};
      settimeofday(&horaUTC, nullptr);
    }
  }
  esp_fill_random(sesionNube, sizeof(sesionNube));
  mutexWiFi = xSemaphoreCreateMutex();
  colaNube = xQueueCreate(MAX_PENDIENTES, sizeof(LecturaNube));
  if (!colaNube || xTaskCreate(tareaRed, "mawi-red", 10240, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("Nube: memoria insuficiente para iniciar el envio.");
  }
}

String numeroJSON(float valor, int decimales) {
  return isfinite(valor) ? String(valor, decimales) : String("null");
}

void encolarNube(const String &fecha, unsigned long tiempoMs, float tension, float radiacion,
                 float lux, float temperatura,
                 float humedad, const String &estadoPanel, const String &estadoLux,
                 const String &estadoDHT) {
  if (!colaNube) return;
  char id[26];
  snprintf(id, sizeof(id), "%08lx%08lx-%08lx", (unsigned long)sesionNube[0],
           (unsigned long)sesionNube[1], (unsigned long)++secuenciaNube);
  const String captura = fecha == "sin_reloj_valido" ? String("null") : "\"" + fecha + "\"";
  String json = String("{\"id\":\"") + id + "\",\"capturado\":" + captura +
    ",\"tiempo_ms\":" + String(tiempoMs) +
    ",\"tension_mv\":" + numeroJSON(tension * 1000, 3) +
    ",\"irradiancia_wm2\":" + numeroJSON(radiacion, 1) +
    ",\"lux\":" + numeroJSON(lux, 1) + ",\"temperatura_c\":" + numeroJSON(temperatura, 1) +
    ",\"humedad_pct\":" + numeroJSON(humedad, 1) +
    ",\"estado_panel\":\"" + estadoPanel + "\",\"estado_lux\":\"" + estadoLux +
    "\",\"estado_dht\":\"" + estadoDHT + "\"}";
  LecturaNube lectura;
  if (json.length() >= sizeof(lectura.json)) return;
  json.toCharArray(lectura.json, sizeof(lectura.json));
  static bool aviso = false;
  const bool llena = xQueueSend(colaNube, &lectura, 0) != pdTRUE;
  if (llena && !aviso) Serial.println("Nube: cola llena. El registro local sigue.");
  aviso = llena;
}

// Comandos por monitor serie:
//   hora 2026-11-17 16:20:00
//   i2c
//   wifi RED<TAB>CLAVE   guarda la red y conecta de inmediato
//   borrar RED           libera la ranura de esa red
//   escanear             lista las redes cercanas (WIFI_RED ...)
//   listar               lista las redes guardadas (WIFI_GUARDADA ...)
//   cero                 panel tapado: guarda el desfase de reposo (I_cero)
//   calibrar W/m2        panel al sol: guarda I_sc con esa referencia
//   calibracion          muestra los valores de calibracion en uso
//   nube CLAVE
void revisarComandoSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  if (cmd.endsWith("\r")) cmd.remove(cmd.length() - 1);
  if (cmd.startsWith("wifi ")) {
    configurarWiFi(cmd.substring(5));
    return;
  }
  if (cmd.startsWith("borrar ")) {
    borrarWiFi(cmd.substring(7));
    return;
  }
  cmd.trim();
  if (cmd == "escanear") {
    escanearWiFi();
    return;
  }
  if (cmd == "listar") {
    listarWiFi();
    return;
  }
  if (cmd.startsWith("kluz ")) {
    calibrarKLuz(cmd.substring(5).toFloat());
    return;
  }
  if (cmd == "diag") {
    diagnostico();
    return;
  }
  if (cmd == "calibracion") {
    mostrarCalibracion();
    return;
  }
  if (cmd == "cero") {
    calibrarCero();
    return;
  }
  if (cmd.startsWith("calibrar ")) {
    calibrarRadiacion(cmd.substring(9).toFloat());
    return;
  }
  if (cmd.startsWith("nube ")) {
    configurarNube(cmd.substring(5));
    return;
  }
  if (cmd == "i2c") {
    escanearI2C();
    return;
  }
  if (!cmd.startsWith("hora ")) return;
  int a, mo, d, h, mi, s;
  if (rtcOk && sscanf(cmd.c_str() + 5, "%d-%d-%d %d:%d:%d", &a, &mo, &d, &h, &mi, &s) == 6) {
    rtc.adjust(DateTime(a, mo, d, h, mi, s));
    Serial.println("Hora ajustada: " + rtc.now().timestamp());
  } else {
    Serial.println("Formato: hora 2026-11-17 16:20:00");
  }
}

// Salidas invalidas se mantienen en NAN: un fallo no se convierte en cero.
bool leerPanel(float &volts, float &corriente, float &potencia, float &radiacion,
               float &rawPromedio, int16_t &rawMax, String &estado) {
  volts = corriente = potencia = radiacion = rawPromedio = NAN;
  rawMax = INT16_MIN;
  estado = "SIN_ADS";
  if (!adsOk || !presente(0x48)) {
    adsOk = false;  // Se reintenta iniciar en el siguiente ciclo.
    return false;
  }

  int32_t suma = 0;
  float sumaV2 = 0;
  bool saturado = false;
  bool potenciaAlta = false;
  for (int i = 0; i < MUESTRAS; ++i) {
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
    unsigned long inicio = millis();
    while (!ads.conversionComplete()) {
      if (millis() - inicio >= TIMEOUT_ADC_MS) {
        estado = "TIMEOUT_ADS";
        return false;
      }
      delay(1);
    }
    int16_t raw = ads.getLastConversionResults();
    if (!presente(0x48)) {
      estado = "ERROR_I2C_ADS";
      adsOk = false;
      return false;
    }
    suma += raw;
    if (raw > rawMax) rawMax = raw;
    if (raw >= RAW_SATURADO) saturado = true;
    float v = ads.computeVolts(raw);
    sumaV2 += v * v;
    if (v * v / R_SHUNT >= 0.8f * P_SHUNT_NOM_W) potenciaAlta = true;
  }
  rawPromedio = float(suma) / MUESTRAS;
  volts = rawPromedio * ads.computeVolts(1);
  if (saturado) {
    estado = "SATURACION_ADC";
    return false;  // Tension recortada; I y G no son validas.
  }
  corriente = volts / R_SHUNT;
  potencia = sumaV2 / (MUESTRAS * R_SHUNT);
  estado = "OK_ELECTRICO";
  if (volts < -RUIDO_V) estado += "|TENSION_NEGATIVA_REVISAR";
  if (ads.computeVolts(rawMax) >= VOLT_MAX) estado += "|CERCA_LIMITE_ADC";
  if (potenciaAlta) estado += "|POTENCIA_SHUNT_ALTA";

  bool calibracionValida = CALIBRADO && isfinite(iscEq1000A) &&
                           iscEq1000A > 0 && isfinite(iCeroA);
  float isc1000 = calibracionValida ? iscEq1000A : ISC_PROVISIONAL_A;
  float cero = calibracionValida ? iCeroA : 0.0f;
  if (!calibracionValida) estado += "|PROVISIONAL";
  if (volts >= -RUIDO_V && !potenciaAlta) {
    float corregida = corriente - cero;
    if (corregida < 0) {
      estado += "|POR_DEBAJO_DEL_CERO";
      radiacion = 0;
    } else {
      radiacion = 1000.0f * corregida / isc1000;
    }
  }
  return true;
}

// ---- Calibracion en campo (queda guardada en la memoria del ESP32) ----
// Promedio de 10 mediciones de corriente. Falla si el ADS no responde o satura.
bool promedioCorriente(float &media) {
  float suma = 0;
  int validas = 0;
  for (int i = 0; i < 10; i++) {
    float v, c, p, g, raw;
    int16_t rawMax;
    String estado;
    if (leerPanel(v, c, p, g, raw, rawMax, estado) && isfinite(c)) {
      suma += c;
      validas++;
    }
    delay(100);
  }
  if (validas < 8) return false;
  media = suma / validas;
  return true;
}

void cargarCalibracion() {
  Preferences ajustes;
  if (!ajustes.begin("registrador", true)) return;
  const float k = ajustes.getFloat("isc1000", ISC_EQ_1000_DEF_A);
  const float cero = ajustes.getFloat("i0", I_CERO_DEF_A);
  if (isfinite(k) && k > 0.01f && k < 5.0f) iscEq1000A = k;
  if (isfinite(cero) && fabsf(cero) < 0.05f) iCeroA = cero;
  const float kl = ajustes.getFloat("kluz", K_LUZ_LM_W);
  if (isfinite(kl) && kl >= 60.0f && kl <= 200.0f) kLuz = kl;
}

void mostrarCalibracion() {
  Serial.printf("Calibracion: I_sc a 1000 W/m2 = %.4f A | I_cero = %.5f A | K lux = %.1f lm/W\n",
                iscEq1000A, iCeroA, kLuz);
}

bool guardarCalibracion(const char* clave, float valor) {
  Preferences ajustes;
  if (!ajustes.begin("registrador") || ajustes.putFloat(clave, valor) != sizeof(float)) {
    Serial.println("Calibracion: error al guardar.");
    return false;
  }
  return true;
}

// Panel TAPADO por completo: mide la corriente de reposo (desfase del ADC).
void calibrarCero() {
  float media;
  if (!promedioCorriente(media)) {
    Serial.println("Calibracion: lectura del ADS no valida.");
    return;
  }
  if (fabsf(media) > 0.01f) {
    Serial.printf("Calibracion: hay %.1f mA; tapa el panel por completo y repite.\n", media * 1000);
    return;
  }
  if (!guardarCalibracion("i0", media)) return;
  iCeroA = media;
  mostrarCalibracion();
}

// Panel al SOL: recibe los W/m2 reales de una referencia y calcula I_sc a 1000 W/m2.
void calibrarRadiacion(float referencia) {
  if (!(referencia >= 50.0f && referencia <= 1500.0f)) {
    Serial.println("Formato: calibrar W/m2   (referencia entre 50 y 1500)");
    return;
  }
  float media;
  if (!promedioCorriente(media)) {
    Serial.println("Calibracion: lectura del ADS no valida o saturada.");
    return;
  }
  const float neta = media - iCeroA;
  if (neta < 0.005f) {
    Serial.printf("Calibracion: corriente muy baja (%.2f mA). Apunta el panel al sol.\n", media * 1000);
    return;
  }
  const float k = neta * 1000.0f / referencia;
  if (k < 0.01f || k > 5.0f) {
    Serial.println("Calibracion: resultado fuera de rango; revisa la referencia.");
    return;
  }
  if (!guardarCalibracion("isc1000", k)) return;
  iscEq1000A = k;
  Serial.printf("Calibracion guardada con %.1f W/m2 de referencia y %.2f mA medidos.\n",
                referencia, media * 1000);
  mostrarCalibracion();
}

// Ajuste pequeno (unos pocos %) por vapor de agua, con el punto de rocio del DHT22.
float ajusteHumedad(float temp, float hum) {
  if (!isfinite(temp) || !isfinite(hum) || hum <= 1.0f) return 0.0f;
  const float g = logf(hum / 100.0f) + 17.62f * temp / (243.12f + temp);  // Magnus
  const float rocio = 243.12f * g / (17.62f - g);
  const float aguaCm = expf(0.07f * rocio - 0.075f);                      // agua precipitable
  return constrain(aguaCm - 2.0f, -3.0f, 4.0f);
}

// Convierte lux en W/m2 con la eficacia luminosa K y el ajuste por humedad.
float estimarRadiacionLux(float lux, float temp, float hum) {
  return lux / (kLuz + ajusteHumedad(temp, hum));
}

// Autorango: sensibilidad normal (MTreg 69) y, con luz muy fuerte, rango ampliado
// (MTreg 31, hasta ~120 000 lx) para no saturar al sol.
float leerLux(String &estado) {
  static uint8_t mtreg = 69;
  estado = "SIN_BH1750";
  if (!luxOk) return NAN;
  float lux = luxometro.readLightLevel();
  if (!isfinite(lux) || lux < 0) {
    estado = "ERROR_BH1750";
    return NAN;
  }
  if (mtreg == 69 && lux >= 40000.0f) {
    if (luxometro.setMTreg(31)) mtreg = 31;
    if (lux >= 54600.0f) {
      estado = "CAMBIO_RANGO_BH1750";  // Esta lectura pudo saturarse; la siguiente ya es valida.
      return NAN;
    }
  } else if (mtreg == 31 && lux < 15000.0f) {
    if (luxometro.setMTreg(69)) mtreg = 69;
  }
  if (lux >= 120000.0f) {
    estado = "POSIBLE_SATURACION_BH1750";
    return NAN;
  }
  estado = "OK";
  return lux;
}

// Calibra K con una referencia real de radiacion (piranometro o celda calibrada).
// Con el sensor al aire libre y apuntando al cielo: "kluz 650" -> K = lux / 650.
void calibrarKLuz(float referencia) {
  if (!(referencia >= 50.0f && referencia <= 1500.0f)) {
    Serial.println("Formato: kluz W/m2   (referencia entre 50 y 1500)");
    return;
  }
  float suma = 0;
  int validas = 0;
  String estado;
  for (int i = 0; i < 10; i++) {
    const float lux = leerLux(estado);
    if (isfinite(lux)) {
      suma += lux;
      validas++;
    }
    delay(250);
  }
  if (validas < 8) {
    Serial.println("Calibracion lux: lectura no valida (" + estado + ").");
    return;
  }
  const float lux = suma / validas;
  if (lux < 3000.0f) {
    Serial.printf("Calibracion lux: solo %.0f lux. Saca el sensor al aire libre, apuntando al cielo.\n", lux);
    return;
  }
  const float k = lux / referencia - ajusteHumedad(dht.readTemperature(), dht.readHumidity());
  if (!(k >= 60.0f && k <= 200.0f)) {
    Serial.printf("Calibracion lux: K=%.1f fuera de rango (60-200); revisa la referencia.\n", k);
    return;
  }
  if (!guardarCalibracion("kluz", k)) return;
  kLuz = k;
  Serial.printf("Calibracion lux guardada: %.0f lux con %.0f W/m2 -> K = %.1f lm/W\n", lux, referencia, k);
}

// Diagnostico sin multimetro: canales del ADS, SD y sensores.
void diagnostico() {
  Serial.println("DIAG --------------------------------");
  if (!presente(0x48)) {
    Serial.println("DIAG ADS1115: no responde en 0x48.");
  } else {
    for (int canal = 0; canal < 4; canal++) {
      const int16_t raw = ads.readADC_SingleEnded(canal);
      Serial.printf("DIAG ADS A%d = %.3f mV (raw %d)%s\n", canal, ads.computeVolts(raw) * 1000.0f, raw,
                    canal == 0 ? "  <- entrada del panel/shunt" : "");
    }
  }
  SD.end();
  sdSPI.end();
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk = SD.begin(SD_CS, sdSPI, 1000000);  // 1 MHz: mas tolerante a cables largos
  if (sdOk) {
    const char* tipos[] = {"ninguna", "MMC", "SD", "SDHC"};
    const uint8_t t = SD.cardType();
    Serial.printf("DIAG SD: OK, tarjeta %s, %llu MB\n", t < 4 ? tipos[t] : "?", SD.cardSize() / (1024 * 1024));
  } else {
    Serial.println("DIAG SD: sin respuesta. Revisa capuchon en JP1, puentes W5/W6, orden de pines del modulo y que la tarjeta sea FAT32.");
  }
  String estadoLux;
  const float lux = leerLux(estadoLux);
  Serial.println("DIAG Lux: " + dato(lux, 1, "lux") + " | " + estadoLux);
  Serial.println("DIAG DHT22: " + dato(dht.readTemperature(), 1, "C") + " | " + dato(dht.readHumidity(), 1, "%"));
  Serial.println("DIAG fin");
}

bool guardarSD(const String &linea) {
  if (!sdOk) return false;
  File f = SD.open(ARCHIVO, FILE_APPEND);
  if (!f) return false;
  bool ok = true;
  if (f.size() == 0) {
    ok = f.println(CABECERA) == strlen(CABECERA) + 2;
  }
  if (ok) ok = f.println(linea) == linea.length() + 2;
  f.flush();
  ok = ok && f.getWriteError() == 0;
  f.close();
  return ok;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setTimeOut(50);
  delay(1500);
  escanearI2C();
  cargarCalibracion();
  iniciarSensores();
  iniciarSD();
  iniciarRed();
  Serial.printf("Lecturas cada %lu segundos.\n", INTERVALO_MS / 1000);
  Serial.println("Inicio. Para poner la hora escribe:  hora 2026-11-17 16:20:00");
  Serial.println("Para buscar dispositivos I2C escribe:  i2c");
  Serial.println("Hora del reloj al iniciar: " + fechaISO());
  Serial.println(String("Detectados: ADS=") + adsOk + " BH1750=" + luxOk +
                 " RTC=" + rtcOk + " SD=" + sdOk);
}

void loop() {
  revisarComandoSerial();
  unsigned long inicio = millis();
  if (inicio - ultimo < INTERVALO_MS) return;
  ultimo = inicio;

  // Si el ADS dejo de responder, reinicia el bus I2C cada 3 intentos
  // y vuelve a iniciarlo cuando reaparezca.
  static int fallosADS = 0;
  if (!adsOk) {
    if (!presente(0x48)) {
      fallosADS++;
      if (fallosADS % 3 == 0) {
        Wire.end();
        delay(20);
        Wire.begin(21, 22);
        Wire.setTimeOut(50);
      }
    }
    if (presente(0x48)) {
      adsOk = iniciarADS();
      fallosADS = 0;
    }
  }

  String fecha = fechaISO();

  float volts, corriente, potencia, radiacion, rawPromedio;
  int16_t rawMax;
  String estadoPanel, estadoLux;
  leerPanel(volts, corriente, potencia, radiacion, rawPromedio, rawMax, estadoPanel);
  float lux = leerLux(estadoLux);
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isfinite(t) || t < -40 || t > 80) t = NAN;
  if (!isfinite(h) || h < 0 || h > 100) h = NAN;
  String estadoDHT = isfinite(t) && isfinite(h) ? "OK" : "ERROR_DHT";

  // Respaldo: si el panel marca casi 0 pero el luxometro indica luz, estima con lux + DHT22.
  if (USAR_ESTIMADO_LUX && isfinite(lux)) {
    const float estimada = estimarRadiacionLux(lux, t, h);
    if (estimada > 30.0f && (!isfinite(radiacion) || radiacion < 0.2f * estimada)) {
      radiacion = estimada;
      estadoPanel += "|ESTIMADO_POR_LUX";
    }
  }

  String linea = fecha + "," + String(inicio) + "," + num(rawPromedio, 2) + "," +
    (isfinite(rawPromedio) ? String(rawMax) : String("")) + "," +
    num(volts * 1000, 3) + "," + num(corriente * 1000, 3) + "," +
    num(potencia, 5) + "," + num(radiacion, 1) + "," + estadoPanel + "," +
    num(lux, 1) + "," + estadoLux + "," + num(t, 1) + "," + num(h, 1) + "," + estadoDHT;

  bool guardado = guardarSD(linea);
  encolarNube(fecha, inicio, volts, radiacion, lux, t, h, estadoPanel, estadoLux, estadoDHT);

  String f = fecha; f.replace('T', ' ');
  Serial.println("--------------------------------");
  Serial.println("Fecha:            " + f);
  Serial.println("Tension ADC:      " + dato(volts * 1000, 3, "mV"));
  Serial.println("Corriente shunt:  " + dato(corriente * 1000, 3, "mA"));
  Serial.println("Lux BH1750:       " + dato(lux, 1, "lux") + " | " + estadoLux);
  Serial.println("Radiacion solar:  " + dato(radiacion, 1, "W/m2") +
                 (CALIBRADO ? "  (calibracion preliminar, 1 punto)"
                            : "  (PROVISIONAL, sin calibrar)"));
  Serial.println("Estado:           " + estadoPanel + " | SD: " +
                 (!sdOk ? "NO DETECTADA" : (guardado ? "guardado" : "ERROR AL GUARDAR")));
}
