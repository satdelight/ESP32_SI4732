#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "EEPROM.h"
#include <SI4735.h>
#include "DSEG7_Classic_Regular_16.h"
#include "Rotary.h"
#include <patch_ssb_compressed.h>

const uint16_t size_content = sizeof ssb_patch_content; // See ssb_patch_content.h
const uint16_t cmd_0x15_size = sizeof cmd_0x15;         // Array of lines where the 0x15 command occurs in the patch content.


// --- SSB-Debug einschalten (1=an, 0=aus) ---
#define DEBUG_SSB 1

// Optionen für TFT-Anzeige
#ifndef TFT_SHOW_FM_DETAILS
#define TFT_SHOW_FM_DETAILS 1
#endif
#ifndef TFT_SHOW_FM_BANDLABEL
#define TFT_SHOW_FM_BANDLABEL 0
#endif

// ------------ TFT ST7735 ------------
#define USE_TFT_ST7735 1
#if USE_TFT_ST7735
#include <SPI.h>
#include <Adafruit_ST7735.h>
#include <Fonts/FreeMonoBold18pt7b.h>

#define TFT_CS   5
#define TFT_DC   26
#define TFT_RST  33
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define TCLR_BUFFER(x) (x[0] = '\0')
char tftBufferFreq[15];
char tftBufferUnt[6];
char tftBufferBand[24];
char tftBufferAGC[16];
char tftBufferBW[20];
char tftBufferStepVFO[20];
char tftBufferStereo[10];
char tftBufferRssi[10];
char tftBufferBFO[10];
// RDS-Puffer für TFT
char tftBufferRdsPS[26];
char tftBufferRdsRT[66];

static char* tftConvertToChar(uint16_t value, char *strValue, uint8_t len, uint8_t dot, uint8_t separator) {
  char d;
  for (int i = (len - 1); i >= 0; i--) { d = value % 10; value = value / 10; strValue[i] = d + 48; }
  strValue[len] = '\0';
  if (dot > 0) { for (int i = len; i >= dot; i--) strValue[i + 1] = strValue[i]; strValue[dot] = separator; }
  if (strValue[0] == '0') { strValue[0] = ' '; if (strValue[1] == '0') strValue[1] = ' '; }
  return strValue;
}

static void tftPrintValue(int col, int line, char *oldValue, char *newValue, uint8_t space, uint16_t color, uint8_t txtSize) {
  int c = col;
  char *pOld = oldValue, *pNew = newValue;
  tft.setTextSize(txtSize);
  while (*pOld && *pNew) {
    if (*pOld != *pNew) {
      tft.setTextColor(ST77XX_BLACK); tft.setCursor(c, line); tft.print(*pOld);
      tft.setTextColor(color);        tft.setCursor(c, line); tft.print(*pNew);
    }
    pOld++; pNew++; c += space;
  }
  tft.setTextColor(ST77XX_BLACK);
  while (*pOld) { tft.setCursor(c, line); tft.print(*pOld); pOld++; c += space; }
  tft.setTextColor(color);
  while (*pNew) { tft.setCursor(c, line); tft.print(*pNew); pNew++; c += space; }
  strcpy(oldValue, newValue);
}
#endif
// -------------------------------------

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

#define RESET_PIN 12
#define ENCODER_PIN_A 13
#define ENCODER_PIN_B 14
#define ENCODER_PUSH_BUTTON 27

#define ESP32_I2C_SDA 21
#define ESP32_I2C_SCL 22

// Externer Ref‑Clock
#define PIN_RCLK_OUT 25
#define LEDC_CH         0
#define LEDC_TIMERBIT   11   // bei 8 Bit belassen
#define REFCLK_HZ       32768.0
#define REFCLK_TRIM_HZ  (0)   // z.B. -200, falls du fix 200 Hz abziehen willst

#define MIN_ELAPSED_TIME 300
#define MIN_ELAPSED_RSSI_TIME 200
#define ELAPSED_COMMAND 2000
#define ELAPSED_CLICK 1500
#define DEFAULT_VOLUME 35

#define FM 0
#define LSB 1
#define USB 2
#define AM 3
#define LW 4

#define EEPROM_SIZE 512
#define STORE_TIME 10000

// app_id erhöht wegen neuem ANTCAP-Menü + EEPROM-Layout (+ANTCAP Flag)
const uint8_t app_id = 56;
const int eeprom_address = 0;
long storeTime = millis();
bool itIsTimeToSave = false;

bool bfoOn = false;
bool ssbLoaded = false;

int8_t agcIdx = 0;      // 0..35 (0/1 => AGC ON; >1 => ATT=agcIdx-1)
uint8_t disableAgc = 0; // 0 oder 1
int8_t agcNdx = 0;      // 0..34
int8_t softMuteMaxAttIdx = 24; // guter Startwert gegen UKW-Rauschen

uint8_t countClick = 0;
uint8_t seekDirection = 1;

bool cmdBand = false;
bool cmdVolume = false;
bool cmdAgc = false;
bool cmdBandwidth = false;
bool cmdStep = false;
bool cmdMode = false;
bool cmdMenu = false;
bool cmdSoftMuteMaxAtt = false;
bool cmdRds = false;     // RDS-Editmodus
bool cmdRegion = false;  // Region (AM 9/10 kHz)
bool cmdAntcap = false;  // ANTCAP Auto/Hold

// OLED Edit-Bildschirm aktiv?
bool oledEdit = false;

// RDS standardmäßig aktiv
bool fmRDS = true;

// ANTCAP: true=Auto (Chip stimmt ab), false=Hold (nicht abstimmen)
bool antcapAuto = true;

int16_t currentBFO = 0;
long elapsedRSSI = millis();
long elapsedButton = millis();
long elapsedClick = millis();
long elapsedCommand = millis();
// RDS-Scan kann jederzeit laufen (kein separater Timer nötig)
volatile int encoderCount = 0;
uint16_t currentFrequency;

const uint8_t currentBFOStep = 10;

// ========= AM Region (9/10 kHz) =========
enum AMRegion : uint8_t { REGION_9KHZ = 0, REGION_10KHZ = 1 };
uint8_t amRegion = REGION_9KHZ; // Default: EU 9 kHz
inline uint8_t getAmSpacing() { return (amRegion == REGION_9KHZ) ? 9 : 10; }
static uint16_t alignMwToRegion(uint16_t f, uint16_t minF, uint16_t maxF, uint8_t region) {
  const uint8_t sp = (region == REGION_9KHZ) ? 9 : 10;
  const int base = (region == REGION_9KHZ) ? 531 : 530;
  long n = lround((f - base) / (double)sp);
  long aligned = base + n * (long)sp;
  if (aligned < minF) aligned = minF;
  if (aligned > maxF) aligned = maxF;
  return (uint16_t)aligned;
}
// Allgemeines Raster-Align (für CB etc.)
static uint16_t alignToGrid(uint16_t f, uint16_t base, uint8_t step, uint16_t minF, uint16_t maxF) {
  long n = lround((f - base) / (double)step);
  long aligned = base + n * (long)step;
  if (aligned < minF) aligned = minF;
  if (aligned > maxF) aligned = maxF;
  return (uint16_t)aligned;
}

// Menü inkl. Region + RDS + ANTCAP
const char *menu[] = { "Volume", "Step", "Mode", "BFO", "BW", "AGC/Att", "SoftMute", "Region", "Seek Up", "Seek Down", "RDS", "ANTCAP" };
int8_t menuIdx = 0;
const int lastMenu = 11; // letzter Eintrag: ANTCAP (Index 11)
int8_t currentMenuCmd = -1;

typedef struct { uint8_t idx; const char *desc; } Bandwidth;

int8_t bwIdxSSB = 4;
const int8_t maxSsbBw = 5;
Bandwidth bandwidthSSB[] = {
  {4, "0.5"}, {5, "1.0"}, {0, "1.2"}, {1, "2.2"}, {2, "3.0"}, {3, "4.0"}
};

int8_t bwIdxAM = 4;
const int8_t maxAmBw = 6;
Bandwidth bandwidthAM[] = {
  {4, "1.0"}, {5, "1.8"}, {3, "2.0"}, {6, "2.5"}, {2, "3.0"}, {1, "4.0"}, {0, "6.0"}
};

int8_t bwIdxFM = 0;
const int8_t maxFmBw = 4;
Bandwidth bandwidthFM[] = {
  {0, "AUT"}, {1, "110"}, {2, " 84"}, {3, " 60"}, {4, " 40"}
};

int tabAmStep[] = {1, 5, 9, 10, 50, 100};
const int lastAmStep = (sizeof tabAmStep / sizeof(int)) - 1;
int idxAmStep = 3;

int tabFmStep[] = {5, 10, 20};
const int lastFmStep = (sizeof tabFmStep / sizeof(int)) - 1;
int idxFmStep = 1;

uint16_t currentStepIdx = 1;

const char *bandModeDesc[] = {"FM ", "LSB", "USB", "AM "};
uint8_t currentMode = FM;

typedef struct {
  const char *bandName; uint8_t bandType;
  uint16_t minimumFreq, maximumFreq, currentFreq;
  int8_t currentStepIdx, bandwidthIdx;
} Band;

// Aufgeräumte Bandliste (inkl. LW, MW EU/NA, Ham- & SW-Broadcast-Bänder)
// Hinzugefügt: fehlende Rundfunkbänder (120m, 90m, 75m) und Amateurfunkbänder (630m, 60m, 30m, 17m)
// inkl. CB (40ch) und CB-DE (80ch)
Band band[] = {
  // FM
  {"FM ",  FM_BAND_TYPE,   8750, 10800, 10130, 1, 0},

  // LW / MW (Broadcast)
  {"LW ",  LW_BAND_TYPE,    153,   279,   198,  3, 4},
  {"MW-EU",MW_BAND_TYPE,    531,  1602,   999,  2, 4}, // 9 kHz Raster
  {"MW-NA",MW_BAND_TYPE,    530,  1700,  1000,  3, 4}, // 10 kHz Raster

  // Amateurbänder
  {"630M", MW_BAND_TYPE,    472,   479,   475,  0, 4},  // 472–479 kHz
  {"160M", MW_BAND_TYPE,   1800,  2000,  1840,  0, 4},
  {"80M",  MW_BAND_TYPE,   3500,  4000,  3700,  0, 4},
  {"60M",  SW_BAND_TYPE,   5351,  5367,  5363,  0, 4},  // WRC-15 5351.5–5366.5 (gerundet)
  {"40M",  SW_BAND_TYPE,   7000,  7300,  7100,  0, 4},
  {"30M",  SW_BAND_TYPE,  10100, 10150, 10120,  0, 4},
  {"20M",  SW_BAND_TYPE,  14000, 14350, 14200,  0, 4},
  {"17M",  SW_BAND_TYPE,  18068, 18168, 18100,  0, 4},
  {"15M",  SW_BAND_TYPE,  21000, 21450, 21100,  0, 4},
  {"12M",  SW_BAND_TYPE,  24890, 24990, 24940,  0, 4},
  {"10M",  SW_BAND_TYPE,  28000, 29700, 28400,  0, 4},

  // CB-Funk
  {"CB ",    SW_BAND_TYPE, 26965, 27405, 27285, 3, 4}, // 40 Kanäle, Basis 26.965 MHz, 10 kHz Raster
  {"CB-DE",  SW_BAND_TYPE, 26565, 27405, 27285, 3, 4}, // 80 Kanäle DE: 26.565–27.405 MHz, 10 kHz Raster

  // SW Broadcast (Rundfunk)
  {"120m", SW_BAND_TYPE,   2300,  2495,  2400,  1, 4},
  {"90m",  SW_BAND_TYPE,   3200,  3400,  3300,  1, 4},
  {"75m",  SW_BAND_TYPE,   3900,  4000,  3950,  1, 4},
  {"60m",  SW_BAND_TYPE,   4750,  5060,  4900,  1, 4},
  {"49m",  SW_BAND_TYPE,   5800,  6200,  6000,  1, 4},
  {"41m",  SW_BAND_TYPE,   7200,  7600,  7400,  1, 4},
  {"31m",  SW_BAND_TYPE,   9400,  9900,  9700,  1, 4},
  {"25m",  SW_BAND_TYPE,  11600, 12100, 11900,  1, 4},
  {"22m",  SW_BAND_TYPE,  13570, 13870, 13700,  1, 4},
  {"19m",  SW_BAND_TYPE,  15100, 15800, 15300,  1, 4},
  {"16m",  SW_BAND_TYPE,  17480, 17900, 17600,  1, 4},
  {"15m",  SW_BAND_TYPE,  18900, 19020, 18950,  1, 4},
  {"13m",  SW_BAND_TYPE,  21450, 21850, 21600,  1, 4},
  {"11m",  SW_BAND_TYPE,  25670, 26100, 25800,  1, 4},

  // Catch-all
  {"ALL", SW_BAND_TYPE,     150, 30000, 15000,  1,  4}
};

const int lastBand = (sizeof band / sizeof(Band)) - 1;

// bandIdx muss vor Funktionen deklariert sein, die ihn verwenden
int bandIdx = 0;

// Erkennung bestimmter Bänder
static bool isBroadcastMWBandIdx(int i) {
  const char* n = band[i].bandName;
  return (strcmp(n, "MW-EU") == 0 || strcmp(n, "MW-NA") == 0);
}
static bool isCBBandIdx(int i) {
  const char* n = band[i].bandName;
  return (strncmp(n, "CB", 2) == 0); // "CB " oder "CB-DE"
}
// Seek-Spacing je Band
static uint8_t getSeekSpacingForCurrentBand() {
  if (isBroadcastMWBandIdx(bandIdx)) return getAmSpacing(); // 9/10 kHz je Region
  if (isCBBandIdx(bandIdx)) return 10;                      // CB: 10 kHz
  return 5;                                                 // SW/HAM: 5 kHz typisch
}

int tabStep[] = {1, 5, 10, 50, 100, 500, 1000};
const int lastStep = (sizeof tabStep / sizeof(int)) - 1;

uint8_t rssi = 0;
uint8_t snr = 0;
uint8_t volume = DEFAULT_VOLUME;

// Geräte
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);
SI4735 rx;

// Refclock
double g_realRefHz = 0.0;
void startRefClock() {
  g_realRefHz = ledcSetup(LEDC_CH, REFCLK_HZ, LEDC_TIMERBIT);
  ledcAttachPin(PIN_RCLK_OUT, LEDC_CH);
  uint32_t duty = 1U << (LEDC_TIMERBIT - 1);
  ledcWrite(LEDC_CH, duty);
}

// Vorwärtsdeklarationen
void saveAllReceiverInformation();
void readAllReceiverInformation();
void resetEepromDelay();
void disableCommands();

void oledShowFrequency();
void oledShowBandMode();
void oledShowRSSI();
void oledShowFrequencyScreen();

#if USE_TFT_ST7735
static void tftShowTemplate();
static void tftResetDynamicAreas();
void tftShowStatus();
void tftShowFrequency();
void tftShowRSSI();
void tftShowBandwidth();
void tftShowAgcAtt();
void tftShowStep();
void tftShowBFO();
void tftShowBandMode();
void tftShowRdsPS(const char* txt);
void tftShowRdsRTWindow(const char* txt, int startIdx);
void tftClearRDS();
#endif

void showCommandStatus(char * currentCmd);
void showMenu();
void doMenu(int8_t v);
void doCurrentMenuCmd();
bool isMenuMode();
void setBand(int8_t up_down);
void useBand();
void loadSSB();
void doBandwidth(int8_t v);
void doAgc(int8_t v);
void doStep(int8_t v);
void doMode(int8_t v);
void doVolume(int8_t v);
void showFrequencySeek(uint16_t freq);
void doSeek();
void doSoftMute(int8_t v);

// OLED Edit-Bildschirm
void oledEditShow(const char* title, const char* value);
void oledEditShowBW();
void oledEditShowStep();
void oledEditShowAGC();
void oledEditShowVolume();
void oledEditShowSoftMute();
void oledEditShowBFO();
void oledEditShowRDS();
void oledEditShowRegion();
void oledEditShowAntcap(); // NEU

// Region
void doRegion(int8_t v);

// ============ RDS nach PU2CLR-Beispiel ============
static void sanitizeAscii(char* s) { if (!s) return; for (uint8_t i = 0; s[i]; i++) if ((uint8_t)s[i] < 32) s[i] = ' '; }

// PS (Stationsname) Debounce
char rdsPSShown[9] = "";
char rdsPSCand[9] = "";
unsigned long rdsPSCandSince = 0;

void rdsResetTop() {
#if USE_TFT_ST7735
  tftPrintValue(5, 80, tftBufferRdsPS, (char*)"", 6, ST77XX_WHITE, 1);
#endif
  rdsPSShown[0] = 0; rdsPSCand[0] = 0; rdsPSCandSince = 0;
}
void rdsTopRender(const char* txt8) {
  if (!txt8) return;
  char eight[9]; strncpy(eight, txt8, 8); eight[8] = '\0'; sanitizeAscii(eight);
  if (strncmp(eight, rdsPSShown, 8) != 0) {
#if USE_TFT_ST7735
    tftPrintValue(5, 80, tftBufferRdsPS, eight, 6, ST77XX_WHITE, 1);
#endif
    strncpy(rdsPSShown, eight, 9);
    rdsPSShown[8] = '\0';
  }
}
void rdsPollTop() {
  if (!rx.isCurrentTuneFM()) return;
  if (!rx.getRdsReceived()) return;
  if (!rx.getRdsSync() || rx.getNumRdsFifoUsed() == 0) return;
  char* ps = rx.getRdsStationName();
  if (!ps || !ps[0]) return;

  char tmp[9]; uint8_t i=0; for (; i<8 && ps[i]; i++) tmp[i]=ps[i]; tmp[i]='\0'; sanitizeAscii(tmp);
  unsigned long now = millis();
  if (strcmp(tmp, rdsPSCand) != 0) {
    strncpy(rdsPSCand, tmp, 9); rdsPSCand[8]='\0'; rdsPSCandSince = now; return;
  }
  if (rdsPSCandSince && (now - rdsPSCandSince) >= 800UL) {
    rdsTopRender(rdsPSCand);
    rdsPSCandSince = 0;
  }
}

// RT (Radiotext) Debounce + Scroll‑Fenster
char rdsRT[65] = "";
char rdsRTCand[65] = "";
unsigned long rdsRTCandSince = 0;
int rdsRTIndex = 0;
unsigned long rdsRTNextScrollMs = 0;
const int RT_WINDOW = 20;

void rdsResetBottom() {
#if USE_TFT_ST7735
  tftPrintValue(5, 92, tftBufferRdsRT, (char*)"", 6, ST77XX_WHITE, 1);
#endif
  rdsRT[0]=0; rdsRTCand[0]=0; rdsRTCandSince=0; rdsRTIndex=0; rdsRTNextScrollMs=0;
}
void rdsBottomRenderWindow() {
  char win[RT_WINDOW+1];
  int len = strnlen(rdsRT, 64);
  if (len <= RT_WINDOW) {
    int i=0; for (; i<len; i++) win[i]=rdsRT[i]; for (; i<RT_WINDOW; i++) win[i]=' '; win[RT_WINDOW]='\0';
  } else {
    if (rdsRTIndex > (len - RT_WINDOW)) rdsRTIndex = 0;
    memcpy(win, rdsRT + rdsRTIndex, RT_WINDOW);
    win[RT_WINDOW] = '\0';
  }
#if USE_TFT_ST7735
  tftPrintValue(5, 92, tftBufferRdsRT, win, 6, ST77XX_WHITE, 1);
#endif
}
void rdsPollBottom() {
  if (!rx.isCurrentTuneFM()) return;
  if (!rx.getRdsReceived()) return;
  if (!rx.getRdsSync() || rx.getNumRdsFifoUsed() == 0) return;

  char* rt = rx.getRdsProgramInformation();
  if (rt && rt[0]) {
    char tmp[65]; uint8_t i=0; for (; i<64 && rt[i]; i++) tmp[i] = (rt[i] >= 32 ? rt[i] : ' '); tmp[i]='\0';
    unsigned long now = millis();
    if (strcmp(tmp, rdsRTCand) != 0) {
      strncpy(rdsRTCand, tmp, 65); rdsRTCand[64]='\0'; rdsRTCandSince = now; return;
    }
    if (rdsRTCandSince && (now - rdsRTCandSince) >= 800UL) {
      strncpy(rdsRT, rdsRTCand, 65); rdsRT[64]='\0';
      rdsRTIndex = 0; rdsBottomRenderWindow(); rdsRTCandSince = 0; rdsRTNextScrollMs = now + 700UL;
      return;
    }
  }
  // Scrollen
  if (rdsRT[0]) {
    unsigned long now = millis();
    if (now >= rdsRTNextScrollMs) {
      int len = strnlen(rdsRT, 64);
      if (len > RT_WINDOW) { rdsRTIndex += 1; rdsBottomRenderWindow(); }
      rdsRTNextScrollMs = now + 700UL;
    }
  }
}
// ================================================

// ISR: ESP32 -> IRAM_ATTR
void IRAM_ATTR rotaryEncoder() {
  uint8_t encoderStatus = encoder.process();
  if (encoderStatus) encoderCount = (encoderStatus == DIR_CW) ? 1 : -1;
}

void setup() {
  pinMode(ENCODER_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

#if USE_TFT_ST7735
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tftShowTemplate();
  tftResetDynamicAreas();
#endif

  EEPROM.begin(EEPROM_SIZE);

  if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
    EEPROM.write(eeprom_address, 0);
    EEPROM.commit();
    oled.setTextSize(2);
    oled.setCursor(0,0); oled.print("EEPROM");
    oled.setCursor(0,16); oled.print("RESET");
    oled.display();
    delay(1500);
    oled.clearDisplay();
  }

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);

  rx.setI2CFastModeCustom(100000);

  startRefClock();
  delay(20);
  
  Serial.begin(115200);
  Serial.printf("ssb_patch_content bytes = %u\n", (unsigned)sizeof(ssb_patch_content));
  Serial.printf("LEDC ref: %.3f Hz\n", g_realRefHz);
#if DEBUG_SSB
  Serial.println("[SSB] Debug enabled");
#endif

  rx.getDeviceI2CAddress(RESET_PIN);
  double ref_trimmed = g_realRefHz + REFCLK_TRIM_HZ;
  rx.setRefClock((uint32_t)round(ref_trimmed));
  Serial.printf("LEDC trimmed: %.3f Hz\n", ref_trimmed);
  rx.setRefClockPrescaler(1);
  rx.setup(RESET_PIN, 0, MW_BAND_TYPE, SI473X_ANALOG_AUDIO, XOSCEN_RCLK);

  delay(250);

  if (EEPROM.read(eeprom_address) == app_id) {
    readAllReceiverInformation();
  } else {
    rx.setVolume(volume);
  }

  useBand();

  oledShowFrequencyScreen();
#if USE_TFT_ST7735
  tftShowStatus();
#endif
}

void saveAllReceiverInformation() {
  int addr_offset;
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(eeprom_address, app_id);
  EEPROM.write(eeprom_address + 1, rx.getVolume());
  EEPROM.write(eeprom_address + 2, bandIdx);
  EEPROM.write(eeprom_address + 3, fmRDS);
  EEPROM.write(eeprom_address + 4, currentMode);
  EEPROM.write(eeprom_address + 5, currentBFO >> 8);
  EEPROM.write(eeprom_address + 6, currentBFO & 0XFF);
  // SoftMute speichern (+7)
  uint8_t sm = (softMuteMaxAttIdx < 0) ? 0 : (softMuteMaxAttIdx > 32 ? 32 : (uint8_t)softMuteMaxAttIdx);
  EEPROM.write(eeprom_address + 7, sm);
  // AGC-Index speichern (+8)
  uint8_t agcStore = (agcIdx < 0) ? 0 : (agcIdx > 35 ? 35 : (uint8_t)agcIdx);
  EEPROM.write(eeprom_address + 8, agcStore);
  // AM-Region (+9)
  EEPROM.write(eeprom_address + 9, amRegion);
  // ANTCAP Auto/Hold (+10)
  EEPROM.write(eeprom_address + 10, antcapAuto ? 1 : 0);

  // Bänder beginnen jetzt ab +11
  addr_offset = 11;
  band[bandIdx].currentFreq = currentFrequency;
  for (int i = 0; i <= lastBand; i++) {
    EEPROM.write(addr_offset++, (band[i].currentFreq >> 8));
    EEPROM.write(addr_offset++, (band[i].currentFreq & 0xFF));
    EEPROM.write(addr_offset++, band[i].currentStepIdx);
    EEPROM.write(addr_offset++, band[i].bandwidthIdx);
  }
  EEPROM.commit();
  EEPROM.end();
}

void readAllReceiverInformation() {
  uint8_t vol; int addr_offset; int bwIdx;
  EEPROM.begin(EEPROM_SIZE);

  vol = EEPROM.read(eeprom_address + 1);
  bandIdx = EEPROM.read(eeprom_address + 2);
  fmRDS = EEPROM.read(eeprom_address + 3);
  currentMode = EEPROM.read(eeprom_address + 4);
  currentBFO = EEPROM.read(eeprom_address + 5) << 8;
  currentBFO |= EEPROM.read(eeprom_address + 6);
  // SoftMute laden (+7)
  uint8_t sm = EEPROM.read(eeprom_address + 7);
  if (sm > 32) sm = 24;
  softMuteMaxAttIdx = (int8_t)sm;
  // AGC-Index laden (+8)
  uint8_t agcLoad = EEPROM.read(eeprom_address + 8);
  if (agcLoad > 35) agcLoad = 0;
  agcIdx = (int8_t)agcLoad;
  disableAgc = (agcIdx > 0);
  agcNdx = (agcIdx > 1) ? (agcIdx - 1) : 0;
  // AM-Region (+9)
  amRegion = EEPROM.read(eeprom_address + 9);
  if (amRegion > REGION_10KHZ) amRegion = REGION_9KHZ;
  // ANTCAP (+10)
  uint8_t ac = EEPROM.read(eeprom_address + 10);
  antcapAuto = (ac == 0xFF) ? true : (ac ? true : false); // Default true, falls uninitialisiert

  // Bänder ab +11
  addr_offset = 11;
  for (int i = 0; i <= lastBand; i++) {
    band[i].currentFreq = EEPROM.read(addr_offset++) << 8;
    band[i].currentFreq |= EEPROM.read(addr_offset++);
    band[i].currentStepIdx = EEPROM.read(addr_offset++);
    band[i].bandwidthIdx = EEPROM.read(addr_offset++);
  }

  EEPROM.end();

  currentFrequency = band[bandIdx].currentFreq;

  if (band[bandIdx].bandType == FM_BAND_TYPE) {
    currentStepIdx = idxFmStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabFmStep[currentStepIdx]);
  } else {
    currentStepIdx = idxAmStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabAmStep[currentStepIdx]);
  }

  bwIdx = band[bandIdx].bandwidthIdx;

  if (currentMode == LSB || currentMode == USB) {
    loadSSB();
    bwIdxSSB = (bwIdx > 5) ? 5 : bwIdx;
    rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    if (bandwidthSSB[bwIdxSSB].idx == 0 || bandwidthSSB[bwIdxSSB].idx == 4 || bandwidthSSB[bwIdxSSB].idx == 5)
      rx.setSSBSidebandCutoffFilter(0);
    else
      rx.setSSBSidebandCutoffFilter(1);
  } else if (currentMode == AM) {
    bwIdxAM = bwIdx;
    rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
  } else {
    bwIdxFM = bwIdx;
    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
  }

  delay(50);
  rx.setVolume(vol);

  // Seek-Raster je Band anwenden
  rx.setSeekAmSpacing(getSeekSpacingForCurrentBand());
}

void resetEepromDelay() {
  elapsedCommand = storeTime = millis();
  itIsTimeToSave = true;
}

void disableCommands() {
  cmdBand = false; bfoOn = false; cmdVolume = false; cmdAgc = false;
  cmdBandwidth = false; cmdStep = false; cmdMode = false;
  cmdMenu = false; cmdSoftMuteMaxAtt = false; cmdRds = false; cmdRegion = false; cmdAntcap = false; countClick = 0;
  oledEdit = false;
}

// ---------------- OLED normal ----------------
void oledShowFrequency() {
  oled.fillRect(0, 8, 128, 17, SSD1306_BLACK);

  char tmp[8], out[8];
  sprintf(tmp, "%5.5u", currentFrequency);
  out[0] = (tmp[0] == '0') ? ' ' : tmp[0];
  out[1] = tmp[1];
  const char* unit;
  if (rx.isCurrentTuneFM()) {
    out[2] = tmp[2]; out[3] = '.'; out[4] = tmp[3];
    unit = "MHz";
  } else {
    if (currentFrequency < 1000) { out[1] = ' '; out[2] = tmp[2]; out[3] = tmp[3]; out[4] = tmp[4]; }
    else { out[2] = tmp[2]; out[3] = tmp[3]; out[4] = tmp[4]; }
    unit = "kHz";
  }
  out[5] = '\0';

  oled.setFont(&DSEG7_Classic_Regular_16);
  oled.setCursor(20, 24);
  oled.print(out);

  oled.setFont(NULL);
  oled.setTextSize(1);
  oled.setCursor(90, 15);
  oled.print(unit);

  oled.display();
}

void oledShowBandMode() {
  const char* modeStr =
    (band[bandIdx].bandType == LW_BAND_TYPE) ? "LW  " : bandModeDesc[currentMode];
  oled.fillRect(0, 0, 128, 8, SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.print(modeStr);
  oled.setCursor(90, 0); oled.print(band[bandIdx].bandName);
  oled.display();
}

void oledShowRSSI() {
  char sMeter[10];
  sprintf(sMeter, "S:%d ", rssi);
  oled.fillRect(0, 25, 128, 10, SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(80, 25); oled.print(sMeter);
  if (currentMode == FM) { oled.setCursor(0, 25); oled.print(rx.getCurrentPilot() ? "ST" : "MO"); }
  oled.display();
}

void oledShowFrequencyScreen() {
  oledShowBandMode();
  oledShowFrequency();
  oledShowRSSI();
}

// ---------------- OLED Edit-Bildschirm ----------------
void oledEditShow(const char* title, const char* value) {
  oled.clearDisplay(); // Voll schwarz
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(title);
  oled.setCursor(0, 16);
  oled.print(value);
  oled.display();
}

void oledEditShowBW() {
  char val[20];
  if (currentMode == AM)       snprintf(val, sizeof(val), "BW: %s", bandwidthAM[bwIdxAM].desc);
  else if (currentMode == LSB || currentMode == USB)
                               snprintf(val, sizeof(val), "BW: %s", bandwidthSSB[bwIdxSSB].desc);
  else                         snprintf(val, sizeof(val), "BW: %s", bandwidthFM[bwIdxFM].desc);
  oledEditShow("BW", val);
}
void oledEditShowStep() {
  char val[20];
  sprintf(val, "Stp:%4d", (currentMode == FM)? (tabFmStep[currentStepIdx]*10) : tabAmStep[currentStepIdx]);
  oledEditShow("Step", val);
}
void oledEditShowAGC() {
  char val[16];
  if (agcNdx == 0 && agcIdx == 0) strcpy(val, "AGC ON");
  else snprintf(val, sizeof(val), "ATT: %2.2d", agcNdx);
  oledEditShow("AGC/Att", val);
}
void oledEditShowVolume() {
  char val[16];
  snprintf(val, sizeof(val), "VOL: %2u", rx.getVolume());
  oledEditShow("Volume", val);
}
void oledEditShowSoftMute() {
  char val[20];
  snprintf(val, sizeof(val), "SMute: %2d", softMuteMaxAttIdx);
  oledEditShow("SoftMute", val);
}
void oledEditShowBFO() {
  char val[18];
  if (currentBFO >= 0) snprintf(val, sizeof(val), "+%4.4d", currentBFO);
  else snprintf(val, sizeof(val), "%4.4d", currentBFO);
  oledEditShow("BFO", val);
}
void oledEditShowRDS() {
  oledEditShow("RDS", fmRDS ? "ON" : "OFF");
}
void oledEditShowRegion() {
  oledEditShow("Region", (amRegion == REGION_9KHZ) ? "9 kHz" : "10 kHz");
}
void oledEditShowAntcap() {
  oledEditShow("ANTCAP", antcapAuto ? "Auto" : "Hold");
}

// ---------------- TFT (Vollanzeige) ----------------
#if USE_TFT_ST7735
static void tftShowTemplate() {
  int maxX1 = tft.width() - 2;
  int maxY1 = tft.height() - 5;
  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(2, 2, maxX1, maxY1, ST77XX_BLUE);
  tft.drawLine(2, 40, maxX1, 40, ST77XX_BLUE);
  tft.drawLine(2, 60, maxX1, 60, ST77XX_BLUE);
}

static void tftResetDynamicAreas() {
  int maxX1 = tft.width() - 5;
  tft.fillRect(3, 3,  maxX1, 36, ST77XX_BLACK);
  tft.fillRect(3, 61, maxX1, 60, ST77XX_BLACK);
  TCLR_BUFFER(tftBufferFreq); TCLR_BUFFER(tftBufferUnt); TCLR_BUFFER(tftBufferBand);
  TCLR_BUFFER(tftBufferAGC);  TCLR_BUFFER(tftBufferBW);  TCLR_BUFFER(tftBufferStepVFO);
  TCLR_BUFFER(tftBufferStereo); TCLR_BUFFER(tftBufferRssi); TCLR_BUFFER(tftBufferBFO);
  TCLR_BUFFER(tftBufferRdsPS); TCLR_BUFFER(tftBufferRdsRT);
}

void tftShowFrequency() {
  uint16_t color;
  char localBuff[8];
  if (rx.isCurrentTuneFM()) { tftConvertToChar(currentFrequency, localBuff, 5, 3, '.'); color = ST77XX_CYAN; }
  else { tftConvertToChar(currentFrequency, localBuff, 5, 0, '.'); color = (bfoOn && (currentMode == LSB || currentMode == USB)) ? ST77XX_WHITE : ST77XX_YELLOW; }
  localBuff[5] = '\0';
  tft.setFont(&FreeMonoBold18pt7b);
  tftPrintValue(22, 32, tftBufferFreq, localBuff, 22, color, 1);
  tft.setFont(NULL);
  const char* unt = rx.isCurrentTuneFM() ? "MHz" : "kHz";
  tftPrintValue(140, 5, tftBufferUnt, (char*)unt, 6, ST77XX_GREEN, 1);
}

void tftShowBFO() {
  char buff[6], show[6];
  uint16_t auxBfo = (currentBFO < 0) ? (uint16_t)(~currentBFO + 1) : (uint16_t)currentBFO;
  char sign = (currentBFO < 0) ? '-' : (currentBFO > 0 ? '+' : ' ');
  tftConvertToChar(auxBfo, buff, 4, 0, '.');
  show[0] = sign; memcpy(&show[1], buff, 4); show[5] = '\0';
  tftPrintValue(120, 30, tftBufferBFO, show, 6, ST77XX_CYAN, 1);
}

void tftShowRSSI() {
  if (currentMode == FM) tftPrintValue(4, 4, tftBufferStereo, (char*)(rx.getCurrentPilot() ? "ST" : "MO"), 6, ST77XX_GREEN, 1);
  else tftPrintValue(4, 4, tftBufferStereo, (char*)"", 6, ST77XX_GREEN, 1);

  uint8_t rssiAux;
  if (rssi < 2) rssiAux = 4;
  else if (rssi < 4) rssiAux = 5;
  else if (rssi < 12) rssiAux = 6;
  else if (rssi < 25) rssiAux = 7;
  else if (rssi < 50) rssiAux = 8;
  else rssiAux = 9;

  char sRssi[10];
  sRssi[0] = 'S'; sRssi[1] = rssiAux + 48; sRssi[2] = (rssiAux == 9) ? '+' : ' '; sRssi[3] = '\0';
  tftPrintValue(5, 31, tftBufferRssi, sRssi, 6, ST77XX_GREEN, 1);

  int maxAux = tft.width() - 10;
  int rssiLevel = map(rssiAux, 0, 10, 0, maxAux);
  int snrLevel  = map(snr,     0, 127, 0, maxAux);

  tft.fillRect(5, 42, maxAux, 6, ST77XX_BLACK);
  tft.fillRect(5, 43, rssiLevel, 6, ST77XX_ORANGE);

  tft.fillRect(5, 51, maxAux, 6, ST77XX_BLACK);
  tft.fillRect(5, 51, snrLevel, 6, ST77XX_WHITE);
}

void tftShowBandwidth() {
  if (currentMode == LSB || currentMode == USB || currentMode == AM) {
    char t[20];
    const char* bw = (currentMode == AM) ? bandwidthAM[bwIdxAM].desc : bandwidthSSB[bwIdxSSB].desc;
    snprintf(t, sizeof(t), "BW: %skHz", bw);
    tftPrintValue(5, 110, tftBufferBW, t, 6, ST77XX_GREEN, 1);
  } else {
#if TFT_SHOW_FM_DETAILS
    char t[20];
    snprintf(t, sizeof(t), "BW: %s", bandwidthFM[bwIdxFM].desc);
    tftPrintValue(5, 110, tftBufferBW, t, 6, ST77XX_GREEN, 1);
#else
    tftPrintValue(5, 110, tftBufferBW, (char*)"", 6, ST77XX_GREEN, 1);
    TCLR_BUFFER(tftBufferBW);
#endif
  }
}

void tftShowAgcAtt() {
  char sAgc[15];
  if (agcNdx == 0 && agcIdx == 0) strcpy(sAgc, "AGC ON");
  else { sprintf(sAgc, "ATT: %2.2d", agcNdx); }
  tftPrintValue(110, 110, tftBufferAGC, sAgc, 6, ST77XX_GREEN, 1);
}

void tftShowStep() {
  char sStep[20];
  sprintf(sStep, "Stp:%4d", (currentMode == FM )? ( tabFmStep[currentStepIdx] * 10) : tabAmStep[currentStepIdx]);
  tftPrintValue(110, 65, tftBufferStepVFO, sStep, 6, ST77XX_GREEN, 1);
}

void tftShowBandMode() {
  char bdisp[24];
#if TFT_SHOW_FM_BANDLABEL
  if (rx.isCurrentTuneFM()) snprintf(bdisp, sizeof(bdisp), "%s FM", band[bandIdx].bandName);
  else snprintf(bdisp, sizeof(bdisp), "%s %s", band[bandIdx].bandName, bandModeDesc[currentMode]);
#else
  if (rx.isCurrentTuneFM()) snprintf(bdisp, sizeof(bdisp), "FM ");
  else snprintf(bdisp, sizeof(bdisp), "%s %s", band[bandIdx].bandName, bandModeDesc[currentMode]);
#endif
  tftPrintValue(5, 65, tftBufferBand, bdisp, 6, ST77XX_CYAN, 1);
}

void tftShowRdsPS(const char* txt) {
  if (!txt) return;
  char win[26];
  strncpy(win, txt, 24);
  win[24] = '\0';
  tftPrintValue(5, 80, tftBufferRdsPS, win, 6, ST77XX_WHITE, 1);
}

void tftShowRdsRTWindow(const char* txt, int startIdx) {
  if (!txt) return;
  const int W = 20;
  char win[W+1];
  int len = strlen(txt);
  if (len == 0) { win[0] = 0; tftPrintValue(5, 92, tftBufferRdsRT, win, 6, ST77XX_WHITE, 1); return; }
  if (startIdx >= len) startIdx = 0;
  int n = min(W, len - startIdx);
  memcpy(win, txt + startIdx, n);
  if (n < W) memset(win + n, ' ', W - n);
  win[W] = '\0';
  tftPrintValue(5, 92, tftBufferRdsRT, win, 6, ST77XX_WHITE, 1);
}

void tftClearRDS() {
  tftPrintValue(5, 80, tftBufferRdsPS, (char*)"", 6, ST77XX_WHITE, 1);
  tftPrintValue(5, 92, tftBufferRdsRT, (char*)"", 6, ST77XX_WHITE, 1);
}

void tftShowStatus() {
  tftResetDynamicAreas();
  tftShowFrequency();
  tftShowBandMode();
  if (!rx.isCurrentTuneFM()) {
    tftShowStep();
    tftShowAgcAtt();
    if (ssbLoaded) tftShowBFO();
    tftClearRDS();
  } else {
#if TFT_SHOW_FM_DETAILS
    tftShowStep();
#endif
    tftClearRDS();
  }
  tftShowBandwidth();
  tftShowRSSI();
}
#endif
// -------------------------------------------------

void showCommandStatus(char * currentCmd) {
  oled.fillRect(40, 0, 50, 8, SSD1306_BLACK);
  oled.setCursor(40, 0);
  oled.setTextSize(1);
  oled.print(currentCmd);
  oled.display();
}

void showMenu() {
  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.setTextSize(1);
  oled.print(menu[menuIdx]);
  oled.display();
  showCommandStatus((char *) "Menu");
}

void loadSSB() {
  rx.setI2CFastModeCustom(200000);
  rx.queryLibraryId(); // Is it really necessary here? I will check it.
  rx.patchPowerUp();
  delay(50);
  rx.downloadCompressedPatch(ssb_patch_content, size_content, cmd_0x15, cmd_0x15_size);
  rx.setSSBConfig(bandwidthSSB[bwIdxSSB].idx, 1, 0, 1, 0, 1);
  rx.setI2CStandardMode();
  ssbLoaded = true;
  // oled.clear();
//  cleanBfoRdsInfo();
}

void setBand(int8_t up_down) {
  band[bandIdx].currentFreq = currentFrequency;
  band[bandIdx].currentStepIdx = currentStepIdx;
  if (up_down == 1) bandIdx = (bandIdx < lastBand) ? (bandIdx + 1) : 0;
  else bandIdx = (bandIdx > 0) ? (bandIdx - 1) : lastBand;
  useBand();
  delay(MIN_ELAPSED_TIME);
  elapsedCommand = millis();
}

void useBand() {
  if (band[bandIdx].bandType == FM_BAND_TYPE) {
    currentMode = FM;
    rx.setTuneFrequencyAntennaCapacitor(0); // FM: ohne Wirkung, aber ok
    rx.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabFmStep[band[bandIdx].currentStepIdx]);
    rx.setSeekFmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
    bfoOn = ssbLoaded = false;
    bwIdxFM = band[bandIdx].bandwidthIdx;
    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
    rx.setFmSoftMuteMaxAttenuation(softMuteMaxAttIdx);
    rx.setRdsConfig(3, 3, 3, 3, 3);
    rx.setFifoCount(1);
#if USE_TFT_ST7735
    TCLR_BUFFER(tftBufferRdsPS);
    TCLR_BUFFER(tftBufferRdsRT);
    tftClearRDS();
#endif
    rdsResetTop();
    rdsResetBottom();
  } else {
#if USE_TFT_ST7735
    tftClearRDS();
#endif
    // ANTCAP anwenden: Auto (0) oder Hold (1)
    rx.setTuneFrequencyAntennaCapacitor(antcapAuto ? 0 : 1);

    if (ssbLoaded) {
      rx.setSSB(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabAmStep[band[bandIdx].currentStepIdx], currentMode);
      rx.setSSBAutomaticVolumeControl(1);
      rx.setSsbSoftMuteMaxAttenuation(softMuteMaxAttIdx);
      bwIdxSSB = band[bandIdx].bandwidthIdx;
      rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
      rx.setSSBBfo(currentBFO); // BFO-Wert anwenden
#if DEBUG_SSB
      const char* m = (currentMode==LSB)?"LSB":"USB";
      Serial.printf("[SSB] useBand->setSSB: mode=%s, f=%u kHz, step=%d kHz, BW=%s, BFO=%d, AGC=%s, ATT=%d, SoftMuteMax=%d, ANTCAP=%s\n",
                    m,
                    band[bandIdx].currentFreq,
                    tabAmStep[band[bandIdx].currentStepIdx],
                    bandwidthSSB[bwIdxSSB].desc,
                    (int)currentBFO,
                    disableAgc ? "OFF" : "ON",
                    (int)agcNdx,
                    (int)softMuteMaxAttIdx,
                    antcapAuto ? "Auto" : "Hold");
#endif
    } else {
#if DEBUG_SSB
      if (currentMode == LSB || currentMode == USB) {
        Serial.println("[SSB] WARN: currentMode is SSB but patch not loaded; falling back to AM");
      }
#endif
      currentMode = AM;
      rx.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabAmStep[band[bandIdx].currentStepIdx]);
      bfoOn = false;
      bwIdxAM = band[bandIdx].bandwidthIdx;
      rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
      rx.setAmSoftMuteMaxAttenuation(softMuteMaxAttIdx);
    }
    // AGC/Att immer für AM/SSB anwenden
    rx.setAutomaticGainControl(disableAgc, agcNdx);
    rx.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
    rx.setSeekAmSpacing(getSeekSpacingForCurrentBand());

    // Raster-Ausrichtung je Band - NUR in AM rastern, NICHT in SSB
    if (currentMode == AM) {
      if (isBroadcastMWBandIdx(bandIdx)) {
        uint16_t fNew = alignMwToRegion(band[bandIdx].currentFreq, band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, amRegion);
        if (fNew != band[bandIdx].currentFreq) {
          band[bandIdx].currentFreq = fNew;
          currentFrequency = fNew;
          rx.setFrequency(currentFrequency);
        }
      } else if (isCBBandIdx(bandIdx)) {
        // CB 40ch Basis 26965; CB-DE 80ch Basis 26565; beide 10 kHz Raster
        uint16_t base = (strcmp(band[bandIdx].bandName, "CB-DE") == 0) ? 26565 : 26965;
        uint16_t fNew = alignToGrid(band[bandIdx].currentFreq, base, 10, band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
        if (fNew != band[bandIdx].currentFreq) {
          band[bandIdx].currentFreq = fNew;
          currentFrequency = fNew;
          rx.setFrequency(currentFrequency);
        }
      }
    }

    rdsResetTop();
    rdsResetBottom();
  }
  delay(100);
  currentFrequency = band[bandIdx].currentFreq;
  currentStepIdx = band[bandIdx].currentStepIdx;

  rssi = 0;

  oledShowFrequencyScreen();
#if USE_TFT_ST7735
  tftShowStatus();
#endif

  showCommandStatus((char *) "Band");
}

void doBandwidth(int8_t v) {
  if (currentMode == LSB || currentMode == USB) {
    bwIdxSSB = (v == 1) ? bwIdxSSB + 1 : bwIdxSSB - 1;
    if (bwIdxSSB > maxSsbBw) bwIdxSSB = 0;
    else if (bwIdxSSB < 0)   bwIdxSSB = maxSsbBw;
    rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    if (bandwidthSSB[bwIdxSSB].idx == 0 || bandwidthSSB[bwIdxSSB].idx == 4 || bandwidthSSB[bwIdxSSB].idx == 5)
      rx.setSSBSidebandCutoffFilter(0);
    else
      rx.setSSBSidebandCutoffFilter(1);
    band[bandIdx].bandwidthIdx = bwIdxSSB;
  } else if (currentMode == AM) {
    bwIdxAM = (v == 1) ? bwIdxAM + 1 : bwIdxAM - 1;
    if (bwIdxAM > maxAmBw) bwIdxAM = 0;
    else if (bwIdxAM < 0)  bwIdxAM = maxAmBw;
    rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
    band[bandIdx].bandwidthIdx = bwIdxAM;
  } else {
    bwIdxFM = (v == 1) ? bwIdxFM + 1 : bwIdxFM - 1;
    if (bwIdxFM > maxFmBw) bwIdxFM = 0;
    else if (bwIdxFM < 0)  bwIdxFM = maxFmBw;
    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
    band[bandIdx].bandwidthIdx = bwIdxFM;
  }
#if USE_TFT_ST7735
  tftShowBandwidth();
#endif
  elapsedCommand = millis();
}

void doAgc(int8_t v) {
  agcIdx = (v == 1) ? agcIdx + 1 : agcIdx - 1;
  if (agcIdx < 0 ) agcIdx = 35;
  else if ( agcIdx > 35) agcIdx = 0;
  disableAgc = (agcIdx > 0);
  agcNdx = (agcIdx > 1) ? (agcIdx - 1) : 0;
  rx.setAutomaticGainControl(disableAgc, agcNdx);
#if USE_TFT_ST7735
  tftShowAgcAtt();
#endif
  elapsedCommand = millis();
}

void doStep(int8_t v) {
  if ( currentMode == FM ) {
    idxFmStep = (v == 1) ? idxFmStep + 1 : idxFmStep - 1;
    if (idxFmStep > lastFmStep) idxFmStep = 0;
    else if (idxFmStep < 0)     idxFmStep = lastFmStep;
    currentStepIdx = idxFmStep;
    rx.setFrequencyStep(tabFmStep[currentStepIdx]);
  } else {
    idxAmStep = (v == 1) ? idxAmStep + 1 : idxAmStep - 1;
    if (idxAmStep > lastAmStep) idxAmStep = 0;
    else if (idxAmStep < 0)     idxAmStep = lastAmStep;
    currentStepIdx = idxAmStep;
    rx.setFrequencyStep(tabAmStep[currentStepIdx]);
    rx.setSeekAmSpacing(getSeekSpacingForCurrentBand());
  }
  band[bandIdx].currentStepIdx = currentStepIdx;
#if USE_TFT_ST7735
  tftShowStep();
#endif
  elapsedCommand = millis();
}

void doMode(int8_t v) {
  if (currentMode != FM) {
    if (v == 1)  {
      if (currentMode == AM) { loadSSB(); ssbLoaded = true; currentMode = LSB; }
      else if (currentMode == LSB) currentMode = USB;
      else if (currentMode == USB) { currentMode = AM; bfoOn = ssbLoaded = false; }
    } else {
      if (currentMode == AM) { loadSSB(); ssbLoaded = true; currentMode = USB; }
      else if (currentMode == USB) currentMode = LSB;
      else if (currentMode == LSB) { currentMode = AM; bfoOn = ssbLoaded = false; }
    }
#if DEBUG_SSB
    Serial.printf("[SSB] doMode: mode=%s, f=%u kHz, BFO=%d, ssbLoaded=%s\n",
                  (currentMode==LSB)?"LSB":(currentMode==USB)?"USB":"AM",
                  currentFrequency, (int)currentBFO, ssbLoaded?"true":"false");
#endif
    band[bandIdx].currentFreq = currentFrequency;
    band[bandIdx].currentStepIdx = currentStepIdx;
    useBand();
  }
  elapsedCommand = millis();
}

void doVolume( int8_t v ) {
  if ( v == 1) rx.volumeUp(); else rx.volumeDown();
  elapsedCommand = millis();
}

void showFrequencySeek(uint16_t freq) {
  currentFrequency = freq;
  if (!oledEdit && !isMenuMode()) oledShowFrequencyScreen();
#if USE_TFT_ST7735
  tftShowFrequency();
#endif
}

void doSeek() {
  if ((currentMode == LSB || currentMode == USB)) return;
  rx.seekStationProgress(showFrequencySeek, seekDirection);
  currentFrequency = rx.getFrequency();

  if (rx.isCurrentTuneFM()) {
    rdsResetTop();
    rdsResetBottom();
  }
}

void doSoftMute(int8_t v) {
  softMuteMaxAttIdx = (v == 1) ? softMuteMaxAttIdx + 1 : softMuteMaxAttIdx - 1;
  if (softMuteMaxAttIdx > 32) softMuteMaxAttIdx = 0;
  else if (softMuteMaxAttIdx < 0) softMuteMaxAttIdx = 32;

  if (currentMode == FM) {
    rx.setFmSoftMuteMaxAttenuation(softMuteMaxAttIdx);
  } else if (currentMode == LSB || currentMode == USB) {
    rx.setSsbSoftMuteMaxAttenuation(softMuteMaxAttIdx);
  } else {
    rx.setAmSoftMuteMaxAttenuation(softMuteMaxAttIdx);
  }

  elapsedCommand = millis();
}

void doRegion(int8_t v) {
  // Toggle 9k ↔ 10k pro Rastung
  if (v == 1 || v == -1) {
    amRegion = (amRegion == REGION_9KHZ) ? REGION_10KHZ : REGION_9KHZ;
  }

  // Seek-Raster umstellen (nur für MW relevant; SW/CB wird über getSeekSpacingForCurrentBand() auf 5/10 gesetzt)
  rx.setSeekAmSpacing(getSeekSpacingForCurrentBand());

  // MW-Broadcast-Frequenz ans Raster anpassen (Ham-Bänder nicht)
  if (isBroadcastMWBandIdx(bandIdx)) {
    uint16_t fNew = alignMwToRegion(currentFrequency, band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, amRegion);
    if (fNew != currentFrequency) {
      currentFrequency = fNew;
      rx.setFrequency(currentFrequency);
      oledShowFrequencyScreen();
#if USE_TFT_ST7735
      tftShowFrequency();
#endif
    }
  }

  if (oledEdit) oledEditShowRegion();
  elapsedCommand = millis();
  resetEepromDelay();
}

void doMenu( int8_t v) {
  menuIdx = (v == 1) ? menuIdx + 1 : menuIdx - 1;
  if (menuIdx > lastMenu) menuIdx = 0;
  else if (menuIdx < 0)   menuIdx = lastMenu;
  showMenu();
  delay(MIN_ELAPSED_TIME);
  elapsedCommand = millis();
}

void enterEditScreen() { oledEdit = true; }
void leaveEditScreen() {
  oledEdit = false;
  disableCommands();
#if USE_TFT_ST7735
  tftShowStatus();
#endif
  oledShowFrequencyScreen();
}

void doCurrentMenuCmd() {
  disableCommands(); // Menü verlassen, spezifische Edit-Flags gleich setzen
  switch (currentMenuCmd) {
    case 0: // Volume
      cmdVolume = true;
      enterEditScreen();
      oledEditShowVolume();
      break;
    case 1: // Step
      cmdStep = true;
#if USE_TFT_ST7735
      tftShowStep();
#endif
      enterEditScreen();
      oledEditShowStep();
      break;
    case 2: // Mode
      cmdMode = true;
      enterEditScreen();
      oledEditShow("Mode", (currentMode==FM)?"FM":(currentMode==AM)?"AM":(currentMode==LSB)?"LSB":"USB");
      break;
    case 3: // BFO
      bfoOn = true;
#if USE_TFT_ST7735
      if ((currentMode == LSB || currentMode == USB)) tftShowBFO();
#endif
      enterEditScreen();
      oledEditShowBFO();
      break;
    case 4: // BW
      cmdBandwidth = true;
#if USE_TFT_ST7735
      tftShowBandwidth();
#endif
      enterEditScreen();
      oledEditShowBW();
      break;
    case 5: // AGC
      cmdAgc = true;
#if USE_TFT_ST7735
      tftShowAgcAtt();
#endif
      enterEditScreen();
      oledEditShowAGC();
      break;
    case 6: // SoftMute
      cmdSoftMuteMaxAtt = true;
      enterEditScreen();
      oledEditShowSoftMute();
      break;
    case 7: // Region
      cmdRegion = true;
      enterEditScreen();
      oledEditShowRegion();
      break;
    case 8: // Seek Up (kein Edit)
      seekDirection = 1;
      doSeek();
      oledShowFrequencyScreen();
      break;
    case 9: // Seek Down (kein Edit)
      seekDirection = 0;
      doSeek();
      oledShowFrequencyScreen();
      break;
    case 10: // RDS
      cmdRds = true;
      enterEditScreen();
      oledEditShowRDS();
      break;
    case 11: // ANTCAP
      cmdAntcap = true;
      enterEditScreen();
      oledEditShowAntcap();
      break;
    default:
      break;
  }
  currentMenuCmd = -1;
  elapsedCommand = millis();
}

// Band-/Einstell-Modus für Timeout berücksichtigen (logische Operatoren)
bool isMenuMode() {
  return (cmdMenu || cmdStep || cmdBandwidth || cmdAgc || cmdVolume || cmdSoftMuteMaxAtt || cmdMode || cmdRds || cmdRegion || cmdAntcap || bfoOn || cmdBand);
}

void loop() {
  if (encoderCount != 0) {
    if (bfoOn && (currentMode == LSB || currentMode == USB)) {
      currentBFO = (encoderCount == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
      rx.setSSBBfo(currentBFO);
#if DEBUG_SSB
      Serial.printf("[SSB] BFO=%d (step=%u Hz) @ f=%u kHz\n", (int)currentBFO, (unsigned)currentBFOStep, currentFrequency);
#endif
#if USE_TFT_ST7735
      tftShowBFO();
      tftShowFrequency();
#endif
      if (oledEdit) oledEditShowBFO();
    } else if (cmdMenu) {
      doMenu(encoderCount);
    } else if (cmdMode) {
      doMode(encoderCount);
      if (oledEdit) oledEditShow("Mode", (currentMode==FM)?"FM":(currentMode==AM)?"AM":(currentMode==LSB)?"LSB":"USB");
    } else if (cmdStep) {
      doStep(encoderCount);
      if (oledEdit) oledEditShowStep();
    } else if (cmdAgc) {
      doAgc(encoderCount);
      if (oledEdit) oledEditShowAGC();
    } else if (cmdBandwidth) {
      doBandwidth(encoderCount);
      if (oledEdit) oledEditShowBW();
    } else if (cmdVolume) {
      doVolume(encoderCount);
      if (oledEdit) oledEditShowVolume();
    } else if (cmdSoftMuteMaxAtt) {
      doSoftMute(encoderCount);
      if (oledEdit) oledEditShowSoftMute();
    } else if (cmdRegion) {
      doRegion(encoderCount);
    } else if (cmdRds) {
      // RDS toggeln bei jeder Rastung
      fmRDS = !fmRDS;
      if (!fmRDS) {
#if USE_TFT_ST7735
        tftClearRDS();
#endif
        rdsResetTop();
        rdsResetBottom();
      } else {
        rdsResetTop();
        rdsResetBottom();
      }
      if (oledEdit) oledEditShowRDS();
      resetEepromDelay();
      elapsedCommand = millis();
    } else if (cmdAntcap) {
      // ANTCAP toggeln bei jeder Rastung
      antcapAuto = !antcapAuto;
      // Für AM/SW/CB: Einstellung sofort anwenden durch Retune
      if (!rx.isCurrentTuneFM()) {
        rx.setTuneFrequencyAntennaCapacitor(antcapAuto ? 0 : 1);
        rx.setFrequency(currentFrequency); // retune, damit ANTCAP greift
      }
      if (oledEdit) oledEditShowAntcap();
      resetEepromDelay();
      elapsedCommand = millis();
    } else if (cmdBand) {
      setBand(encoderCount);
    } else {
      // VFO
      if (encoderCount == 1) rx.frequencyUp();
      else                   rx.frequencyDown();
      uint16_t prev = currentFrequency;
      currentFrequency = rx.getFrequency();
      if (rx.isCurrentTuneFM() && currentFrequency != prev) {
        rdsResetTop();
        rdsResetBottom();
      }
      if (!isMenuMode() && !oledEdit) {
        oledShowFrequencyScreen();
      }
#if USE_TFT_ST7735
      tftShowFrequency();
#endif
    }
    encoderCount = 0;
    resetEepromDelay();
  } else {
    // Button-Klick-Logik: Doppel-Klick -> Menü; Einzelklick -> Aktion
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW) {
      countClick++;
      if (cmdMenu) {
        currentMenuCmd = menuIdx;
        doCurrentMenuCmd();
      } else if (countClick == 1) {
        if (oledEdit || isMenuMode()) {
          // Edit/Einstellmodus beenden -> zurück zur Frequenzanzeige
          leaveEditScreen();
        } else if (bfoOn) {
          bfoOn = false;
#if USE_TFT_ST7735
          tftPrintValue(120, 30, tftBufferBFO, (char*)"", 6, ST77XX_CYAN, 1);
          TCLR_BUFFER(tftBufferBFO);
          tftShowFrequency();
#endif
          oledShowFrequencyScreen();
        } else {
          cmdBand = !cmdBand;
          showCommandStatus((char *)"Band");
        }
      } else {
        // Doppel-Klick -> Menü toggeln (beim Schließen sofort neu zeichnen)
        cmdMenu = !cmdMenu;
        if (cmdMenu) {
          showMenu();
        } else {
          oledShowFrequencyScreen();
#if USE_TFT_ST7735
          tftShowStatus();
#endif
        }
      }
      delay(MIN_ELAPSED_TIME);
      elapsedCommand = millis();
    }
  }

  // RSSI/SNR gemeinsam aktualisieren, wenn sich etwas ändert
  if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 6) {
    rx.getCurrentReceivedSignalQuality();
    uint8_t newRssi = rx.getCurrentRSSI();
    uint8_t newSnr  = rx.getCurrentSNR();
    if ((rssi != newRssi || snr != newSnr) && !isMenuMode() && !oledEdit) {
      rssi = newRssi;
      snr  = newSnr;
      oledShowRSSI();
#if USE_TFT_ST7735
      tftShowRSSI();
#endif
    }
    elapsedRSSI = millis();
  }

  // RDS Poll – nur einmal pro Loop
  if (rx.isCurrentTuneFM() && fmRDS) {
    rx.getRdsStatus();
    rdsPollTop();
    rdsPollBottom();
  }

  // Timeout: Edit/Einstellmodus und Band-Modus beenden
  if ((millis() - elapsedCommand) > ELAPSED_COMMAND) {
    if (oledEdit || isMenuMode()) {
      leaveEditScreen();
    }
    elapsedCommand = millis();
  }

  // Doppelklick-Fenster
  if ((millis() - elapsedClick) > ELAPSED_CLICK) {
    countClick = 0;
    elapsedClick = millis();
  }

  // EEPROM nach Ruhe
  if (itIsTimeToSave) {
    if ((millis() - storeTime) > STORE_TIME) {
      saveAllReceiverInformation();
      storeTime = millis();
      itIsTimeToSave = false;
    }
  }

  delay(5);
}
