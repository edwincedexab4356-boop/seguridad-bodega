/*
  SEGURIDAD PARA BODEGAS — Arduino UNO R4 WiFi
  ------------------------------------------------------------
  Zonas:  PIR (movimiento) · ultrasónico (alguien cerca de la puerta)
  Al detectar con el sistema ARMADO:
    - PIR o ultrasónico -> CUENTA regresiva 9..0 (display + matriz) para poner la clave
  ALARMA: sirena y el motor CIERRA la puerta. Cada evento sale por el
          Bluetooth con fecha y hora (RTC).
  El RTC se detecta solo: si no está, el resto sigue funcionando.
  (Versión SIN láser, SIN cámara y SIN microSD: D3, D10-D13 y A0 quedan libres.)

  Control: celular por Bluetooth (HC-05) o Monitor Serie a 115200. Escribe AYUDA.
  Todo lo que responde el Arduino sale por los dos.

  PINES
    D0/D1  Bluetooth HC-05 (Serial1, 9600)  TXD -> D0 · D1 -> 220 -> RXD, RXD -> 220 -> 220 -> GND
    D2  PIR OUT          D4  buzzer (+)
    D5  L298N ENA        D8  L298N IN1              D9  L298N IN2
    D6  HC-SR04 TRIG     D7  HC-SR04 ECHO
    A1  74HC595 DS (14)  A2  SH_CP (11)  A3  ST_CP (12)
    SDA/SCL  RTC DS3231
*/
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>
#include "Arduino_LED_Matrix.h"

// Imprime a la vez en el USB y en el Bluetooth
class Doble : public Print {
 public:
  size_t write(uint8_t c) override { Serial.write(c); Serial1.write(c); return 1; }
  size_t write(const uint8_t* b, size_t n) override { Serial.write(b, n); Serial1.write(b, n); return n; }
};
Doble out;

// ---------- pines ----------
const uint8_t P_PIR = 2, P_BUZ = 4;
const uint8_t P_ENA = 5, P_TRIG = 6, P_ECHO = 7, P_IN1 = 8, P_IN2 = 9;
const uint8_t P_DS = A1, P_SH = A2, P_ST = A3;

// ---------- ajustes guardados ----------
struct Ajustes {
  uint8_t  firma;
  uint8_t  dist;        // cm: alguien más cerca que esto = intruso
  uint8_t  cuenta;      // segundos de la cuenta regresiva (1..9)
  uint16_t motorMs;     // tiempo que gira el motor para cerrar/abrir
  uint8_t  anodo;       // 1 = display ánodo común, 0 = cátodo común
  uint8_t  zonas;       // bit0 PIR, bit1 ultrasónico
  uint8_t  armado;      // recordar si quedó armado
  char     clave[7];    // 4..6 dígitos
  uint8_t  alarmaSeg;   // duración de la sirena en segundos
};
const uint8_t FIRMA = 0xD2;
Ajustes cfg;

void ajustesFabrica() {
  cfg.firma = FIRMA; cfg.dist = 30; cfg.cuenta = 9; cfg.motorMs = 1500;
  cfg.anodo = 0; cfg.zonas = 0b11; cfg.armado = 0;
  strcpy(cfg.clave, "1234"); cfg.alarmaSeg = 60;
}
void guardar() { EEPROM.put(0, cfg); }

// ---------- estado ----------
enum Estado { DESARMADO, SALIDA, ARMADO, CUENTA, ALARMA };
Estado estado = DESARMADO;
const char* NOMBRE[] = {"DESARMADO", "SALIDA", "ARMADO", "CUENTA", "ALARMA"};

unsigned long tEstado = 0;           // cuándo empezó el estado actual
int  segRestantes = 0;
char origen[12] = "-";               // qué disparó
bool puertaCerrada = false;
unsigned long motorHasta = 0;
const unsigned long PIR_CALENTAR = 30000;
uint8_t clavesMalas = 0;

bool hayRTC = false;
RTC_DS3231 rtc;
ArduinoLEDMatrix matriz;

int  distancia = -1;                 // cm, -1 = sin eco
bool pirAct = false, ultraAct = false;
uint8_t ultraCerca = 0;

// ---------- 7 segmentos (74HC595: Q0=a ... Q6=g, Q7=punto) ----------
const uint8_t DIG[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
const uint8_t SEG_GUION = 0x40, SEG_A = 0x77, SEG_E = 0x79, SEG_NADA = 0x00;
uint8_t segActual = 0xFF;
void display7(uint8_t b) {
  if (b == segActual) return;
  segActual = b;
  uint8_t s = cfg.anodo ? (uint8_t)~b : b;
  digitalWrite(P_ST, LOW);
  shiftOut(P_DS, P_SH, MSBFIRST, s);
  digitalWrite(P_ST, HIGH);
}

// ---------- matriz 12x8 del R4 ----------
const uint8_t FUENTE[][7] = {
  {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
  {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},{0x1F,0x02,0x04,0x02,0x01,0x11,0x0E},
  {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},{0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
  {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},{0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
  {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},{0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
  {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},   // 10 = A (armado)
  {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},   // 11 = - (desarmado)
  {0x04,0x04,0x04,0x04,0x04,0x00,0x04},   // 12 = ! (alarma)
};
uint8_t marco[8][12];
int matrizActual = -1;
void matrizSimbolo(int s, bool invertido) {
  int clave = s * 2 + (invertido ? 1 : 0);
  if (clave == matrizActual) return;
  matrizActual = clave;
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 12; c++) marco[r][c] = invertido ? 1 : 0;
  for (int r = 0; r < 7; r++)
    for (int c = 0; c < 5; c++)
      if (FUENTE[s][r] & (0x10 >> c)) marco[r][c + 4] = invertido ? 0 : 1;
  matriz.renderBitmap(marco, 8, 12);
}

// ---------- utilidades ----------
String ahora() {
  if (hayRTC) {
    DateTime t = rtc.now();
    char b[20];
    snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d",
             t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
    return String(b);
  }
  return "t+" + String(millis() / 1000) + "s";
}

void registrar(const String& ev) {
  String linea = ahora() + "," + ev;
  out.println("EV," + linea);
}

void pitido(int hz, int ms) { tone(P_BUZ, hz, ms); }

void motor(int sentido, unsigned long ms) {   // 1 cerrar, -1 abrir, 0 parar
  if (sentido == 0) {
    digitalWrite(P_IN1, LOW); digitalWrite(P_IN2, LOW); analogWrite(P_ENA, 0);
    motorHasta = 0; return;
  }
  digitalWrite(P_IN1, sentido > 0 ? HIGH : LOW);
  digitalWrite(P_IN2, sentido > 0 ? LOW : HIGH);
  analogWrite(P_ENA, 255);
  motorHasta = millis() + ms;
}
void cerrarPuerta() {
  if (puertaCerrada) return;
  motor(1, cfg.motorMs); puertaCerrada = true; registrar("PUERTA_CERRADA");
}
void abrirPuerta() {
  if (!puertaCerrada) { out.println("La puerta ya está abierta"); return; }
  motor(-1, cfg.motorMs); puertaCerrada = false; registrar("PUERTA_ABIERTA");
}

void cambiar(int e) {
  estado = (Estado)e; tEstado = millis();
  out.print("ESTADO -> "); out.println(NOMBRE[e]);
  out.print("SE,"); out.println(e);          // línea corta para una app
}

// ---------- acciones del sistema ----------
void armar() {
  if (estado != DESARMADO) { out.println("Ya está armado"); return; }
  cfg.armado = 1; guardar();
  segRestantes = cfg.cuenta;
  cambiar(SALIDA);
  registrar("ARMADO (salida " + String(cfg.cuenta) + " s)");
  pitido(1800, 120);
}

void desarmar(const char* como) {
  noTone(P_BUZ);
  cfg.armado = 0; guardar();
  clavesMalas = 0;
  cambiar(DESARMADO);
  registrar(String("DESARMADO por ") + como);
  pitido(2200, 80); delay(120); pitido(2200, 80);
}

void disparo(const char* quien, bool inmediato) {
  if (estado != ARMADO && !(estado == CUENTA && inmediato)) return;
  strncpy(origen, quien, sizeof(origen) - 1); origen[sizeof(origen) - 1] = 0;
  if (inmediato) {
    cambiar(ALARMA);
    registrar(String("ALARMA por ") + quien);
    cerrarPuerta();
  } else {
    segRestantes = cfg.cuenta;
    cambiar(CUENTA);
    registrar(String("DETECCION ") + quien + " - cuenta regresiva");
  }
}

// ---------- sensores ----------
unsigned long tUltra = 0;
void leerSensores() {
  bool pirListo = millis() > PIR_CALENTAR;
  pirAct = pirListo && digitalRead(P_PIR) == HIGH;

  if (millis() - tUltra >= 100) {
    tUltra = millis();
    digitalWrite(P_TRIG, LOW); delayMicroseconds(3);
    digitalWrite(P_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(P_TRIG, LOW);
    unsigned long us = pulseIn(P_ECHO, HIGH, 25000UL);
    distancia = us ? (int)(us / 58) : -1;
    if (distancia > 0 && distancia < cfg.dist) { if (ultraCerca < 10) ultraCerca++; }
    else ultraCerca = 0;
    ultraAct = ultraCerca >= 2;                    // 2 lecturas seguidas
  }

  if (estado == ARMADO) {
    if ((cfg.zonas & 1) && pirAct)        disparo("PIR", false);
    else if ((cfg.zonas & 2) && ultraAct) disparo("ULTRASONICO", false);
  }
}

// ---------- pantalla y sonido según estado ----------
unsigned long tSirena = 0; bool sirenaAlta = false;
void actualizarSalidas() {
  unsigned long t = millis() - tEstado;
  bool parpadeo = (millis() / 300) % 2;
  switch (estado) {
    case DESARMADO:
      display7(SEG_GUION); matrizSimbolo(11, false); break;

    case SALIDA:
    case CUENTA: {
      int r = cfg.cuenta - (int)(t / 1000);
      if (r != segRestantes) {
        segRestantes = r;
        if (r >= 0) pitido(estado == CUENTA ? 2500 : 1500, estado == CUENTA ? 150 : 60);
      }
      if (r < 0) {
        if (estado == SALIDA) { cambiar(ARMADO); registrar("VIGILANDO"); pitido(1800, 300); }
        else { cambiar(ALARMA); registrar(String("ALARMA por ") + origen + " (no pusieron la clave)");
               cerrarPuerta(); }
        break;
      }
      display7(DIG[r]); matrizSimbolo(r, estado == CUENTA && parpadeo);
      break;
    }

    case ARMADO:
      display7((millis() / 1000) % 2 ? SEG_A : (SEG_A | 0x80));
      matrizSimbolo(10, false); break;

    case ALARMA:
      display7(parpadeo ? SEG_E : SEG_NADA); matrizSimbolo(12, parpadeo);
      if (millis() - tSirena > 300) {
        tSirena = millis(); sirenaAlta = !sirenaAlta;
        tone(P_BUZ, sirenaAlta ? 1600 : 1000);
      }
      if (t > (unsigned long)cfg.alarmaSeg * 1000UL) {
        noTone(P_BUZ); registrar("FIN DE SIRENA - sigue vigilando");
        cambiar(ARMADO);
      }
      break;
  }
  if (motorHasta && (long)(millis() - motorHasta) >= 0) motor(0, 0);
}

// ---------- órdenes ----------
void estadoCompleto() {
  out.println("---------------- ESTADO ----------------");
  out.print("Sistema: "); out.print(NOMBRE[estado]);
  if (estado == SALIDA || estado == CUENTA) { out.print(" ("); out.print(segRestantes); out.print(" s)"); }
  out.println();
  out.print("Hora: "); out.println(ahora());
  out.print("PIR: "); out.println(millis() < PIR_CALENTAR ? "calentando" : (digitalRead(P_PIR) ? "MOVIMIENTO" : "quieto"));
  out.print("Ultrasónico: "); if (distancia < 0) out.println("sin eco (¿conectado?)");
  else { out.print(distancia); out.print(" cm (límite "); out.print(cfg.dist); out.println(" cm)"); }
  out.print("Puerta: "); out.println(puertaCerrada ? "CERRADA" : "abierta");
  out.print("Zonas: PIR="); out.print(cfg.zonas & 1 ? 1 : 0);
  out.print(" ULTRA="); out.println(cfg.zonas & 2 ? 1 : 0);
  out.print("Reloj RTC: "); out.println(hayRTC ? "sí" : "no");
  out.print("Cuenta "); out.print(cfg.cuenta); out.print(" s · motor "); out.print(cfg.motorMs);
  out.print(" ms · sirena "); out.print(cfg.alarmaSeg); out.print(" s · display ");
  out.println(cfg.anodo ? "ánodo común" : "cátodo común");
  out.println("----------------------------------------");
  enviarST();
}

// línea corta para la página: ST,estado,seg,pir,dist,puerta,rtc,pirListo,zonas,distLimite
void enviarST() {
  out.print("ST,"); out.print(estado); out.print(','); out.print(segRestantes); out.print(',');
  out.print(pirAct); out.print(','); out.print(distancia); out.print(',');
  out.print(puertaCerrada); out.print(','); out.print(hayRTC); out.print(',');
  out.print(millis() > PIR_CALENTAR); out.print(','); out.print(cfg.zonas); out.print(','); out.println(cfg.dist);
}

void ayuda() {
  out.println(F("ÓRDENES (celular por Bluetooth o Monitor Serie 115200):"));
  out.println(F("  ON                 armar (tienes la cuenta para salir)"));
  out.println(F("  CODIGO:1234        desarmar, parar la alarma y ABRIR la puerta"));
  out.println(F("  ESTADO             ver todo   ·  PRUEBA  probar display, buzzer y motor"));
  out.println(F("  CERRAR             cerrar la puerta (abrir solo con CODIGO)"));
  out.println(F("  SILENCIAR          calla la sirena (sigue armado)"));
  out.println(F("  ALERTA             alarma manual"));
  out.println(F("  DIST:30            límite del ultrasónico en cm"));
  out.println(F("  CUENTA:9           segundos de la cuenta (1-9)"));
  out.println(F("  MOTOR:1500         ms que gira el motor"));
  out.println(F("  SIRENA:60          segundos de sirena"));
  out.println(F("  ZONA:1:0           apagar zona (1 PIR, 2 ultrasónico; 1 = prender)"));
  out.println(F("  DISPLAY:A / DISPLAY:C   display ánodo o cátodo común"));
  out.println(F("  CLAVE:1234:5678    cambiar la clave (vieja:nueva)"));
  out.println(F("  HORA:2026-11-19 14:30:00   poner la hora del RTC"));
  out.println(F("  FABRICA            volver a los ajustes de fábrica"));
}

void prueba() {
  out.println("PRUEBA: display 9..0, pitidos y motor (cerrar y abrir)");
  for (int i = 9; i >= 0; i--) { display7(DIG[i]); matrizSimbolo(i, false); pitido(1200 + i * 100, 80); delay(250); }
  display7(0xFF); delay(400); segActual = 0xFF;
  motor(1, 0); delay(cfg.motorMs); motor(0, 0); delay(300);
  motor(-1, 0); delay(cfg.motorMs); motor(0, 0);
  matrizActual = -1; segActual = 0xFF;
  out.println("PRUEBA terminada");
}

void orden(String s) {
  s.trim();
  String u = s; u.toUpperCase();
  if (!u.length()) return;
  if (u == "AYUDA" || u == "?") ayuda();
  else if (u == "ON" || u == "ARMAR") armar();
  else if (u.startsWith("CODIGO:")) {
    if (s.substring(7) == cfg.clave) {          // clave buena: desarma Y abre la puerta
      if (estado != DESARMADO) desarmar("clave");
      else { clavesMalas = 0; out.println("Clave correcta"); }
      if (puertaCerrada) abrirPuerta();
    } else {
      clavesMalas++;
      registrar("CLAVE INCORRECTA (" + String(clavesMalas) + ")");
      pitido(400, 400);
      if (clavesMalas >= 3 && estado != ALARMA) {   // 3 claves malas = alarma (aunque esté desarmado)
        estado = ARMADO; disparo("CLAVE", true);
      }
    }
  }
  else if (u == "OFF") out.println("Para desarmar escribe CODIGO:tu_clave");
  else if (u == "ESTADO") estadoCompleto();
  else if (u == "ST") enviarST();
  else if (u == "PRUEBA") { if (estado == DESARMADO) prueba(); else out.println("Desarma primero"); }
  else if (u == "CERRAR") { if (puertaCerrada) out.println("Ya está cerrada"); else cerrarPuerta(); }
  else if (u == "ABRIR") out.println("La puerta solo se abre con la clave: CODIGO:tu_clave");
  else if (u == "SILENCIAR") { if (estado == ALARMA) { noTone(P_BUZ); registrar("SILENCIADA"); cambiar(ARMADO); } }
  else if (u == "ALERTA") { if (estado == DESARMADO) { cfg.armado = 1; guardar(); } estado = ARMADO; disparo("MANUAL", true); }
  else if (u.startsWith("DIST:")) { int v = s.substring(5).toInt(); if (v >= 3 && v <= 250) { cfg.dist = v; guardar(); out.println("OK distancia " + String(v) + " cm"); } }
  else if (u.startsWith("CUENTA:")) { int v = s.substring(7).toInt(); if (v >= 1 && v <= 9) { cfg.cuenta = v; guardar(); out.println("OK cuenta " + String(v) + " s"); } }
  else if (u.startsWith("MOTOR:")) { int v = s.substring(6).toInt(); if (v >= 100 && v <= 10000) { cfg.motorMs = v; guardar(); out.println("OK motor " + String(v) + " ms"); } }
  else if (u.startsWith("SIRENA:")) { int v = s.substring(7).toInt(); if (v >= 5 && v <= 250) { cfg.alarmaSeg = v; guardar(); out.println("OK sirena " + String(v) + " s"); } }
  else if (u.startsWith("ZONA:")) {
    int z = s.substring(5, 6).toInt(); int v = s.substring(7).toInt();
    if (z >= 1 && z <= 2) { uint8_t bit = 1 << (z - 1); if (v) cfg.zonas |= bit; else cfg.zonas &= ~bit; guardar(); out.println("OK zona " + String(z) + (v ? " prendida" : " apagada")); }
  }
  else if (u == "DISPLAY:A" || u == "DISPLAY:C") { cfg.anodo = (u == "DISPLAY:A"); guardar(); segActual = 0xFF; out.println(cfg.anodo ? "OK ánodo común" : "OK cátodo común"); }
  else if (u.startsWith("CLAVE:")) {
    int p = s.indexOf(':', 6);
    String vieja = s.substring(6, p), nueva = s.substring(p + 1);
    bool soloNum = nueva.length() >= 4 && nueva.length() <= 6;
    for (unsigned i = 0; i < nueva.length(); i++) if (!isDigit(nueva[i])) soloNum = false;
    if (p < 0 || vieja != cfg.clave) out.println("Clave vieja incorrecta");
    else if (!soloNum) out.println("La nueva debe tener 4 a 6 números");
    else { nueva.toCharArray(cfg.clave, sizeof(cfg.clave)); guardar(); registrar("CLAVE CAMBIADA"); }
  }
  else if (u.startsWith("HORA:")) {
    if (!hayRTC) out.println("No hay RTC conectado");
    else {
      String h = s.substring(5);   // AAAA-MM-DD HH:MM:SS
      rtc.adjust(DateTime(h.substring(0,4).toInt(), h.substring(5,7).toInt(), h.substring(8,10).toInt(),
                          h.substring(11,13).toInt(), h.substring(14,16).toInt(), h.substring(17,19).toInt()));
      out.println("Hora puesta: " + ahora());
    }
  }
  else if (u == "FABRICA") { ajustesFabrica(); guardar(); out.println("Ajustes de fábrica (clave 1234)"); }
  else out.println("No entiendo: " + s + "  (escribe AYUDA)");
}

String bufUSB, bufBT;
void leerSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { orden(bufUSB); bufUSB = ""; }
    else if (bufUSB.length() < 60) bufUSB += c;
  }
  while (Serial1.available()) {             // órdenes desde el celular
    char c = Serial1.read();
    if (c == '\n' || c == '\r') { orden(bufBT); bufBT = ""; }
    else if (bufBT.length() < 60) bufBT += c;
  }
}

// ---------- arranque ----------
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);                   // HC-05 viene a 9600
  pinMode(P_PIR, INPUT); pinMode(P_BUZ, OUTPUT);
  pinMode(P_ENA, OUTPUT); pinMode(P_IN1, OUTPUT); pinMode(P_IN2, OUTPUT);
  pinMode(P_TRIG, OUTPUT); pinMode(P_ECHO, INPUT);
  pinMode(P_DS, OUTPUT); pinMode(P_SH, OUTPUT); pinMode(P_ST, OUTPUT);
  motor(0, 0);
  matriz.begin();

  EEPROM.get(0, cfg);
  if (cfg.firma != FIRMA) { ajustesFabrica(); guardar(); }
  cfg.clave[sizeof(cfg.clave) - 1] = 0;

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) {}

  Wire.begin();
  hayRTC = rtc.begin();
  if (hayRTC && rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  out.println();
  out.println("=== SEGURIDAD PARA BODEGAS ===");
  out.print("RTC: "); out.print(hayRTC ? "sí" : "no");
  out.println(" · Bluetooth HC-05 en D0/D1 (9600)");
  out.println("El PIR tarda 30 s en calentar. Escribe AYUDA para ver las órdenes.");
  registrar("ENCENDIDO");
  pitido(1500, 100);

  if (cfg.armado) { estado = DESARMADO; armar(); }   // si se fue la luz armado, vuelve a armarse
}

void loop() {
  leerSerie();
  leerSensores();
  actualizarSalidas();
}
