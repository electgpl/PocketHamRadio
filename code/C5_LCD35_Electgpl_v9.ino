/*
 * Electgpl - ESP32-C5-Touch-LCD-3.5 (Waveshare) - Peripheral demo v9 (launcher + 16 apps)
 *
 *  HOME  : PalmOS-style launcher, 3x3 icon grid. BOOT = HOME, PWR (short) = BACK, BOOT long = backlight,
 *          tap on the Electgpl logo = HOME
 *  SENS  : SHTC3 (T/RH), PCF85063A (RTC), QMI8658 (IMU + bubble level), AXP2101 (PMIC), microSD
 *  AUDIO : ES8311 + NS4150B - level meter, REC/PLAY/STOP (5 s in PSRAM), WAV to microSD, test tones
 *  CAM   : BF3901 over PARLIO RX 2-bit, live 240x320 RGB565
 *  AX.25 : AFSK 1200 Bd (Bell 202) software modem - microphone receiver with terminal,
 *          speaker transmitter (UI test frame) for acoustic loopback
 *  RADIO : Wi-Fi 6 dual-band (2.4/5 GHz) scanner and BLE advertisement scanner
 *  PAINT : finger painting, 10 colors, 3 brush sizes, PSRAM shadow buffer, SAVE as 24-bit BMP on microSD
 *  NOTES : .txt/.md/.log/.csv reader from microSD (built-in demo text without card), word wrap, drag scroll
 *  TETRIS: 10x20 board, 7-bag, ghost piece, wall kicks, levels, auto-repeat controls
 *  RADIO : + ESP-NOW mode (promiscuous sniffer with channel hopping; two-board symmetric range test)
 *  RADIO : + IEEE 802.15.4 mode (energy-detect sweep ch 11-26 + promiscuous MAC frame sniffer)
 *  FOX   : Wi-Fi RSSI tracker on one BSSID (promiscuous RX), 30 s chart, stats, Geiger-style audio
 *  SPEC  : audio spectrum 0-8 kHz, fixed-point FFT 512 + Hann, waterfall, peak with interpolation
 *  SETUP : persistent settings in NVS (backlight, sleep, volume, mic gain, AX.25 callsign, camera flip)
 *  APRS  : APRS decoder on the AX.25 receiver (uncompressed, compressed, Mic-E, objects, items, messages),
 *          station list, detail, radar map with distance/bearing from MYPOS, position BEACON
 *  MAZE  : tilt maze, IMU-driven ball physics (circle vs wall collision, sub-stepping), random DFS mazes,
 *          3-step IMU axis-mapping wizard stored in NVS, EMA filter + dead zone
 *  SSTV  : RX with VIS auto-detect (Robot 36, Martin M1, Scottie S1, PD 120), per-line sync + slant tracking,
 *          TX Robot 36 from the camera or a test pattern, BMP save
 *  RTTY  : ITA2 Baudot RX/TX, 45.45/50/75 Bd, 170/200/425/850 Hz shift, NOR/REV, waterfall tuning, CQ/RYRY/INFO macros
 *  Header: clock + smartphone-style battery icon (% and charging bolt); NTP time sync in Settings
 *  CALC  : 4-function calculator with %, +/-, backspace, chained operations
 *  Power : on battery the backlight drops to minimum and, after BATT_IDLE_SLEEP_MS without
 *          user activity, the board enters deep sleep (or AXP2101 power-off, see SLEEP_MODE)
 *
 * Toolchain: esp32 by Espressif 3.3.10 (ESP-IDF v5.5.4), board "ESP32C5 Dev Module"
 * Options  : USB CDC On Boot = Enabled  |  PSRAM = Enabled  (REQUIRED)
 * Files    : C5_LCD35_Electgpl_v9.ino, font8x13.h (public domain), bf3901_init.h (Apache-2.0, Espressif)
 *
 * References: Waveshare BSP waveshare__esp32_c5_touch_lcd_3_5 and 01_factory firmware (IDF 5.5.4),
 * board schematic ESP32-C5-Touch-LCD-3.5.pdf, esp_codec_dev/es8311.c, esp_cam_sensor (bf3901.c,
 * esp_cam_ctlr_spi_cam.c), XPowersLib (AXP2101), SHTC3 / PCF85063A / QMI8658A datasheets,
 * AX.25 Link Access Protocol v2.2, Bell 202 / APRS 1.0.1 AFSK conventions.
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ESP_I2S.h>
#include <math.h>
#include <algorithm>
#include "driver/gpio.h"
#include "driver/parlio_rx.h"
#include "esp_private/parlio_rx_private.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_private/esp_cache_private.h"   // esp_cache_get_alignment()
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_wifi.h"
#include "esp_ieee802154.h"
#include "esp_now.h"
#include <Preferences.h>
#include "font8x13.h"
#include "bf3901_init.h"

// ================================ CONFIGURATION ================================
// Main I2C bus (all peripherals, including the camera SCCB)
static const int      PIN_I2C_SDA   = 27;
static const int      PIN_I2C_SCL   = 26;
static const uint32_t I2C_FREQ_HZ   = 400000;
static const uint32_t SCCB_FREQ_HZ  = 100000;
// SPI bus shared by LCD and microSD
static const int      PIN_SPI_SCLK  = 6;
static const int      PIN_SPI_MOSI  = 7;
static const int      PIN_SPI_MISO  = 2;
static const int      PIN_LCD_CS    = 8;
static const int      PIN_LCD_DC    = 5;
static const int      PIN_SD_CS     = 9;
static const uint32_t LCD_SPI_HZ    = 40000000;
static const uint32_t SD_SPI_HZ     = 20000000;
static const uint16_t LCD_W = 320, LCD_H = 480;
static const uint8_t  LCD_MADCTL    = 0x48;
static const bool     LCD_INVERT    = true;
// ES8311 I2S (no MCLK line: the codec derives its clock from BCLK)
static const int      PIN_I2S_BCLK  = 23;
static const int      PIN_I2S_WS    = 10;
static const int      PIN_I2S_DOUT  = 25;
static const int      PIN_I2S_DIN   = 24;
static const uint32_t AUDIO_FS      = 16000;
static const int      AUDIO_BLOCK   = 256;
static const int      REC_SECONDS   = 5;
static const float    TONE_AMPL_FS  = 0.30f;
static const uint8_t  DAC_VOL_REG   = 0xA7;   // 0xBF = 0 dB, 0.5 dB/step -> 0xA7 = -12 dB
static const uint8_t  MIC_DIG_GAIN  = 3;      // 0..7 -> 0..42 dB in 6 dB steps
static const bool     MIC_USE_RIGHT = false;
// AX.25 / AFSK
static const char    *AX25_MYCALL   = "NOCALL";   // set your own callsign before transmitting
static const uint8_t  AX25_MYSSID   = 0;
static const char    *AX25_DEST     = "APZEGL";   // APZxxx = experimental (APRS tocall table)
static const int      AX25_TXDELAY_FLAGS = 40;    // ~267 ms of preamble flags
static const int      AX25_TXTAIL_FLAGS  = 4;
static const float    AFSK_TX_AMPL  = 0.35f;
// BF3901 camera (PARLIO RX)
static const int      PIN_CAM_D0    = 0;
static const int      PIN_CAM_D1    = 1;
static const int      PIN_CAM_PCLK  = 11;     // shared with U0TXD (0R selector on the board)
static const int      PIN_CAM_VSYNC = 12;     // shared with U0RXD
static const bool     CAM_HMIRROR   = false;  // factory firmware: both false
static const bool     CAM_VFLIP     = false;
static const bool     CAM_SWAP_BYTES= true;   // sensor RGB565 LE -> ST7796 expects BE
// Radio scanner
static const uint32_t BLE_SCAN_SECONDS = 5;
// Battery power management
static const bool     PM_ENABLE          = true;
static const uint8_t  BATT_BACKLIGHT_PCT = 5;        // backlight while running on battery
static const uint32_t BATT_IDLE_SLEEP_MS = 30000;    // idle time on battery before sleeping
enum SleepMode { SLEEP_DEEP = 0, SLEEP_POWEROFF = 1 };
// SLEEP_DEEP    : ESP32-C5 deep sleep. Wake: touch screen (FT6336 INT on GPIO3) or RESET.
// SLEEP_POWEROFF: AXP2101 shutdown (lowest drain). Wake: PWR button or USB plug-in.
// BOOT (GPIO28) cannot wake the chip: only LP GPIO0..GPIO6 are deep-sleep wake capable.
static const SleepMode SLEEP_MODE        = SLEEP_DEEP;
static const bool     WAKE_ON_GPIO4      = false;    // EXIO_INT (CH32V006) via 0R selector R60, unverified
// Time sync / games
static const int8_t   DEF_TZ_HOURS  = -3;        // UTC offset for NTP (Argentina: UTC-3, no DST)
static const char    *NTP_SERVER1   = "pool.ntp.org";
static const char    *NTP_SERVER2   = "time.google.com";
// Misc
static const int      PIN_TP_INT    = 3;
static const int      PIN_BOOT      = 28;
// I2C addresses (7-bit)
static const uint8_t  A_ES8311 = 0x18, A_EXIO = 0x24, A_AXP = 0x34, A_FT6336 = 0x38;
static const uint8_t  A_PCF85063 = 0x51, A_QMI8658 = 0x6B, A_BF3901 = 0x6E, A_SHTC3 = 0x70;
// CH32V006 I/O expander
static const uint8_t  EXIO_REG_MODE = 0x02, EXIO_REG_OUT = 0x03, EXIO_REG_PWM = 0x05;
static const uint8_t  EXIO_RST_MASK = 0x03, EXIO_PA_BIT = 5;
// ==============================================================================

// Types used as function parameters must be declared BEFORE any function:
// the Arduino preprocessor inserts automatic prototypes ahead of the first function.
struct ImuData   { float ax, ay, az, gx, gy, gz, t; };
struct RtcTime   { uint8_t sec, min, hour, day, wday, mon; uint16_t year; bool osStop; };
struct PmicData  { bool vbusGood, batPresent; uint8_t dir, chg; uint16_t vbat, vbus, vsys; float tdie; int pct; };
struct Button    { int16_t x, y, w, h; const char *label; };
static const int AX25_MAX_FRAME = 332;
struct Ax25Frame { uint16_t len; uint8_t data[AX25_MAX_FRAME]; };
static const int TERM_COLS = 38, TERM_ROWS = 23;
struct TermLine  { char txt[TERM_COLS + 1]; uint16_t color; };
static const int MAX_AP = 40, MAX_BLE = 40;
struct ApInfo    { char ssid[33]; int8_t rssi; uint8_t ch; uint8_t auth; uint8_t bssid[6]; };
struct BleInfo   { char addr[18]; char name[24]; int8_t rssi; };
enum   Page      { PG_HOME = 0, PG_SENSOR, PG_AUDIO, PG_CAM, PG_AX25, PG_RADIO, PG_PAINT, PG_NOTES, PG_TETRIS, PG_CALC,
                   PG_FOX, PG_SPEC, PG_SET, PG_APRS, PG_MAZE, PG_SSTV, PG_RTTY };
struct EnDev     { uint8_t mac[6]; int8_t rssi; uint8_t ch, len; bool bc, enc; uint32_t n, t; };
struct EnPeer    { uint8_t mac[6]; char call[8]; int8_t rssi, remote; float ema, per; uint32_t first, last, recv, t, seqW, recvW;
                   int8_t hist[300]; int head, mn, mx; };
struct __attribute__((packed)) EnPkt { char magic[4]; uint8_t ver; uint32_t seq; int8_t heard; char call[7]; };
struct SstvMode  { const char *name; uint8_t vis, kind; uint16_t lines; float lineMs, syncMs, firstSyncMs; };
enum   StView    { SV_MAIN = 0, SV_CALL, SV_TIME, SV_NET, SV_PASS };
struct AprsSta   { char call[10]; float lat, lon; bool pos, hasCs, hasAlt; char symT, symC, kind; int16_t crs, spd; int32_t alt;
                   uint32_t heard; uint16_t pkts; uint8_t hh, mm; char comment[44]; char path[36]; };
struct IeeeRx    { uint8_t len, ch; int8_t rssi; uint8_t lqi; uint8_t d[127]; };
struct Settings  { uint8_t blUsb, blBatt, sleepIdx, sleepMode; int8_t volDb; uint8_t micGain; char call[7]; uint8_t ssid, hmirror, vflip;
                   int8_t tz; char wifiSsid[33]; char wifiPass[65]; float myLat, myLon; uint8_t hasPos;     // v2 fields
                   uint8_t mzAxX, mzAxY; int8_t mzSgX, mzSgY; float mzOffX, mzOffY; uint8_t mzCal; };        // v3 fields

// ============================ Persistent settings (NVS) ========================
static const uint16_t SLEEP_OPTS[6] = {15, 30, 60, 120, 300, 0};   // seconds, 0 = never
static const uint8_t  BATT_BL_OPTS[7] = {1, 2, 5, 10, 20, 30, 50};
static const uint8_t  CFG_VER = 3;
static Settings cfg;
static Preferences prefs;
static uint8_t dacVolReg() { return (uint8_t)constrain(0xBF + 2 * (int)cfg.volDb, 0, 0xFF); }   // 0.5 dB/LSB
static void settingsDefaults() {                     // factory defaults = compile-time constants
  cfg.blUsb = 100; cfg.blBatt = BATT_BACKLIGHT_PCT; cfg.sleepIdx = 1;
  for (int i = 0; i < 6; i++) if ((uint32_t)SLEEP_OPTS[i] * 1000 == BATT_IDLE_SLEEP_MS) cfg.sleepIdx = (uint8_t)i;
  cfg.sleepMode = (uint8_t)SLEEP_MODE; cfg.volDb = (int8_t)(((int)DAC_VOL_REG - 0xBF) / 2); cfg.micGain = MIC_DIG_GAIN;
  snprintf(cfg.call, sizeof(cfg.call), "%s", AX25_MYCALL); cfg.ssid = AX25_MYSSID; cfg.hmirror = CAM_HMIRROR; cfg.vflip = CAM_VFLIP;
  cfg.tz = DEF_TZ_HOURS; cfg.wifiSsid[0] = 0; cfg.wifiPass[0] = 0; cfg.myLat = cfg.myLon = 0; cfg.hasPos = 0;
  cfg.mzAxX = 0; cfg.mzSgX = 1; cfg.mzAxY = 1; cfg.mzSgY = 1; cfg.mzOffX = cfg.mzOffY = 0; cfg.mzCal = 0;   // v6 behaviour until SETUP
}
static void settingsLoad() {
  settingsDefaults();
  if (prefs.begin("electgpl", true)) {
    const uint8_t ver = prefs.getUChar("ver", 0); const size_t len = prefs.getBytesLength("cfg");
    if (ver == CFG_VER && len == sizeof(cfg)) prefs.getBytes("cfg", &cfg, sizeof(cfg));
    else if (ver == 2 && len >= offsetof(Settings, mzAxX)) prefs.getBytes("cfg", &cfg, offsetof(Settings, mzAxX));   // migrate v2 blob
    else if (ver == 1 && len >= offsetof(Settings, tz)) prefs.getBytes("cfg", &cfg, offsetof(Settings, tz));         // migrate v1 blob
    prefs.end();
  }
  cfg.call[6] = 0; cfg.sleepIdx %= 6; cfg.sleepMode = cfg.sleepMode ? 1 : 0; cfg.micGain &= 7; cfg.ssid &= 15;
  cfg.blUsb = (uint8_t)constrain(cfg.blUsb, 10, 100); cfg.blBatt = (uint8_t)constrain(cfg.blBatt, 1, 50);
  cfg.volDb = (int8_t)constrain(cfg.volDb, -30, 6); cfg.tz = (int8_t)constrain(cfg.tz, -12, 14);
  cfg.wifiSsid[32] = 0; cfg.wifiPass[64] = 0; if (!isfinite(cfg.myLat) || !isfinite(cfg.myLon)) cfg.hasPos = 0;
  cfg.mzAxX &= 1; cfg.mzAxY &= 1; cfg.mzSgX = cfg.mzSgX < 0 ? -1 : 1; cfg.mzSgY = cfg.mzSgY < 0 ? -1 : 1;
  if (!isfinite(cfg.mzOffX) || !isfinite(cfg.mzOffY) || fabsf(cfg.mzOffX) > 1.5f || fabsf(cfg.mzOffY) > 1.5f) { cfg.mzOffX = cfg.mzOffY = 0; cfg.mzCal = 0; }
}
static void settingsSave() {
  if (prefs.begin("electgpl", false)) { prefs.putUChar("ver", CFG_VER); prefs.putBytes("cfg", &cfg, sizeof(cfg)); prefs.end(); }
}

enum   AudioMode { AM_IDLE = 0, AM_TONE = 1, AM_REC = 2, AM_PLAY = 3 };

// RGB565 colors
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_BLUE    0x435F
#define C_YELLOW  0xFFE0
#define C_CYAN    0x07FF
#define C_ORANGE  0xFD20
#define C_GRAY    0x8410
#define C_DGRAY   0x2104
#define C_BG      0x0841
#define C_ACCENT  0xFC60   // UI accent (tabs, separators)
#define C_BRAND_E 0xF800   // red "E" of Electgpl

// ================================= I2C helpers ================================
static bool i2cPresent(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }
static bool i2cW8(uint8_t a, uint8_t r, uint8_t v) {
  Wire.beginTransmission(a); Wire.write(r); Wire.write(v); return Wire.endTransmission() == 0;
}
static bool i2cRd(uint8_t a, uint8_t r, uint8_t *b, size_t n, bool repStart = true) {
  Wire.beginTransmission(a); Wire.write(r);
  if (Wire.endTransmission(!repStart) != 0) return false;      // SCCB needs STOP: repStart=false
  if (Wire.requestFrom((int)a, (int)n) != n) return false;
  for (size_t i = 0; i < n; i++) b[i] = (uint8_t)Wire.read();
  return true;
}
static int i2cR8(uint8_t a, uint8_t r) { uint8_t v; return i2cRd(a, r, &v, 1) ? v : -1; }
static bool i2cCmd16(uint8_t a, uint16_t c) {
  Wire.beginTransmission(a); Wire.write((uint8_t)(c >> 8)); Wire.write((uint8_t)c);
  return Wire.endTransmission() == 0;
}

// =================================== CH32V006 =================================
static bool exioOk = false;
static bool backlightSet(uint8_t pct) {
  if (pct > 100) pct = 100;
  return i2cW8(A_EXIO, EXIO_REG_PWM, (uint8_t)((uint16_t)pct * 255U / 100U));
}
static bool exioInit() {
  const uint8_t dir = EXIO_RST_MASK | (1U << EXIO_PA_BIT);
  bool ok = i2cW8(A_EXIO, EXIO_REG_MODE, 0xFF) && i2cW8(A_EXIO, EXIO_REG_OUT, 0) &&
            i2cW8(A_EXIO, EXIO_REG_PWM, 0) && i2cW8(A_EXIO, EXIO_REG_MODE, dir);
  if (!ok) return false;
  ok  = i2cW8(A_EXIO, EXIO_REG_OUT, EXIO_RST_MASK);                         delay(50);   // LCD/TP reset high
  ok &= i2cW8(A_EXIO, EXIO_REG_OUT, 0);                                     delay(50);   // reset low
  ok &= i2cW8(A_EXIO, EXIO_REG_OUT, EXIO_RST_MASK | (1U << EXIO_PA_BIT));   delay(200);  // release + PA on
  return ok;
}

// =================================== AXP2101 ==================================
static bool axpOk = false;
static bool axpLdo(uint8_t volReg, uint8_t enBit, uint16_t mV) {   // LDO: (mV-500)/100 in [4:0], enable in 0x90
  int v = i2cR8(A_AXP, volReg), e = i2cR8(A_AXP, 0x90);
  if (v < 0 || e < 0) return false;
  return i2cW8(A_AXP, volReg, (uint8_t)((v & 0xE0) | ((mV - 500) / 100))) &&
         i2cW8(A_AXP, 0x90, (uint8_t)(e | (1U << enBit)));
}
static bool axpInit() {
  bool ok = axpLdo(0x92, 0, 3300) &&   // ALDO1 -> A3V3
            axpLdo(0x93, 1, 3300) &&   // ALDO2 -> AL2_3V3
            axpLdo(0x95, 3, 1800) &&   // ALDO4 -> 1V8 (camera DOVDD)
            axpLdo(0x97, 5, 2800);     // BLDO2 -> 2V8 (camera AVDD)
  int adc = i2cR8(A_AXP, 0x30);        // ADC enable: b0 VBAT, b2 VBUS, b3 VSYS, b4 TDIE
  int det = i2cR8(A_AXP, 0x68);        // battery detection enable
  ok &= adc >= 0 && i2cW8(A_AXP, 0x30, (uint8_t)(adc | 0x1D));
  ok &= det >= 0 && i2cW8(A_AXP, 0x68, (uint8_t)(det | 0x01));
  delay(20);
  return ok;
}
static uint16_t axpH(uint8_t hi, uint8_t lo, uint8_t mask) {
  int h = i2cR8(A_AXP, hi), l = i2cR8(A_AXP, lo);
  return (h < 0 || l < 0) ? 0 : (uint16_t)(((h & mask) << 8) | l);
}
static bool axpRead(PmicData &p) {
  int s1 = i2cR8(A_AXP, 0x00), s2 = i2cR8(A_AXP, 0x01);
  if (s1 < 0 || s2 < 0) return false;
  p.vbusGood   = s1 & 0x20;
  p.batPresent = s1 & 0x08;
  p.dir  = (uint8_t)((s2 >> 5) & 0x03);   // 0 standby, 1 charge, 2 discharge
  p.chg  = (uint8_t)(s2 & 0x07);          // charger state
  const bool vbusIn = p.vbusGood && !(s2 & 0x08);
  p.vbat = p.batPresent ? axpH(0x34, 0x35, 0x1F) : 0;
  p.vbus = vbusIn ? axpH(0x38, 0x39, 0x3F) : 0;
  p.vsys = axpH(0x3A, 0x3B, 0x3F);
  p.tdie = 22.0f + (7274.0f - (float)axpH(0x3C, 0x3D, 0x3F)) / 20.0f;
  p.pct  = p.batPresent ? i2cR8(A_AXP, 0xA4) : -1;
  return true;
}

// =================================== ST7796 ===================================
static SPISettings lcdSpi(LCD_SPI_HZ, MSBFIRST, SPI_MODE0);
static uint8_t lineBuf[LCD_W * 2];
static uint8_t glyphBuf[2 * (FONT_W * 3) * (FONT_H * 3)];

static inline void lcdCmdRaw(uint8_t c) { digitalWrite(PIN_LCD_DC, LOW); SPI.transfer(c); digitalWrite(PIN_LCD_DC, HIGH); }
static void lcdCmd(uint8_t c, const uint8_t *d = nullptr, size_t n = 0) {
  SPI.beginTransaction(lcdSpi); digitalWrite(PIN_LCD_CS, LOW);
  lcdCmdRaw(c); if (n) SPI.writeBytes(d, n);
  digitalWrite(PIN_LCD_CS, HIGH); SPI.endTransaction();
}
static void lcdInit() {   // same ST7796 sequence as the Waveshare IDF BSP
  lcdCmd(0x01); delay(120); lcdCmd(0x11); delay(120);
  { uint8_t d[] = {LCD_MADCTL}; lcdCmd(0x36, d, 1); }
  { uint8_t d[] = {0x55}; lcdCmd(0x3A, d, 1); }
  { uint8_t d[] = {0xC3}; lcdCmd(0xF0, d, 1); }
  { uint8_t d[] = {0x96}; lcdCmd(0xF0, d, 1); }
  { uint8_t d[] = {0x01}; lcdCmd(0xB4, d, 1); }
  { uint8_t d[] = {0xC6}; lcdCmd(0xB7, d, 1); }
  { uint8_t d[] = {0x40,0x8A,0x00,0x00,0x29,0x19,0xA5,0x33}; lcdCmd(0xE8, d, sizeof(d)); }
  { uint8_t d[] = {0x06}; lcdCmd(0xC1, d, 1); }
  { uint8_t d[] = {0xA7}; lcdCmd(0xC2, d, 1); }
  { uint8_t d[] = {0x18}; lcdCmd(0xC5, d, 1); }
  { uint8_t d[] = {0xF0,0x09,0x0B,0x06,0x04,0x15,0x2F,0x54,0x42,0x3C,0x17,0x14,0x18,0x1B}; lcdCmd(0xE0, d, sizeof(d)); }
  { uint8_t d[] = {0xF0,0x09,0x0B,0x06,0x04,0x03,0x2D,0x43,0x42,0x3B,0x16,0x14,0x17,0x1B}; lcdCmd(0xE1, d, sizeof(d)); }
  { uint8_t d[] = {0x3C}; lcdCmd(0xF0, d, 1); }
  { uint8_t d[] = {0x69}; lcdCmd(0xF0, d, 1); }
  lcdCmd(LCD_INVERT ? 0x21 : 0x20);
  lcdCmd(0x29); delay(20);
}
static inline void lcdWindowRaw(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t ca[4] = {(uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1};
  uint8_t ra[4] = {(uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1};
  lcdCmdRaw(0x2A); SPI.writeBytes(ca, 4); lcdCmdRaw(0x2B); SPI.writeBytes(ra, 4); lcdCmdRaw(0x2C);
}
static void lcdBlit(int x, int y, int w, int h, const uint8_t *px) {
  if (x < 0 || y < 0 || w <= 0 || h <= 0 || x + w > LCD_W || y + h > LCD_H) return;
  SPI.beginTransaction(lcdSpi); digitalWrite(PIN_LCD_CS, LOW);
  lcdWindowRaw(x, y, x + w - 1, y + h - 1); SPI.writeBytes(px, (uint32_t)w * h * 2);
  digitalWrite(PIN_LCD_CS, HIGH); SPI.endTransaction();
}
static void lcdFill(int x, int y, int w, int h, uint16_t c) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > LCD_W) w = LCD_W - x;
  if (y + h > LCD_H) h = LCD_H - y;
  if (w <= 0 || h <= 0) return;
  for (int i = 0; i < w; i++) { lineBuf[2*i] = (uint8_t)(c >> 8); lineBuf[2*i+1] = (uint8_t)c; }
  SPI.beginTransaction(lcdSpi); digitalWrite(PIN_LCD_CS, LOW);
  lcdWindowRaw(x, y, x + w - 1, y + h - 1);
  for (int r = 0; r < h; r++) SPI.writeBytes(lineBuf, (uint32_t)w * 2);
  digitalWrite(PIN_LCD_CS, HIGH); SPI.endTransaction();
}
static void lcdRect(int x, int y, int w, int h, uint16_t c) {
  lcdFill(x, y, w, 1, c); lcdFill(x, y + h - 1, w, 1, c); lcdFill(x, y, 1, h, c); lcdFill(x + w - 1, y, 1, h, c);
}
static int drawChar(int x, int y, char ch, uint16_t fg, uint16_t bg, uint8_t s) {
  if (s < 1) s = 1;
  if (s > 3) s = 3;
  if ((uint8_t)ch < FONT_FIRST || (uint8_t)ch > FONT_LAST) ch = '?';
  const uint8_t *g = font8x13[(uint8_t)ch - FONT_FIRST];
  uint8_t *p = glyphBuf;
  for (int r = 0; r < FONT_H; r++)
    for (int sy = 0; sy < s; sy++)
      for (int c = 0; c < FONT_W; c++) {
        const uint16_t col = (g[r] & (0x80 >> c)) ? fg : bg;
        for (int sx = 0; sx < s; sx++) { *p++ = (uint8_t)(col >> 8); *p++ = (uint8_t)col; }
      }
  lcdBlit(x, y, FONT_W * s, FONT_H * s, glyphBuf);
  return x + FONT_W * s;
}
static int drawCharWH(int x, int y, char ch, uint16_t fg, uint16_t bg, int cw, int chh) {   // any cell size <= 24x39
  if ((uint8_t)ch < FONT_FIRST || (uint8_t)ch > FONT_LAST) ch = '?';
  const uint8_t *g = font8x13[(uint8_t)ch - FONT_FIRST];
  uint8_t *p = glyphBuf;
  for (int oy = 0; oy < chh; oy++) {
    const uint8_t row = g[oy * FONT_H / chh];
    for (int ox = 0; ox < cw; ox++) {
      const uint16_t col = (row & (0x80 >> (ox * FONT_W / cw))) ? fg : bg;
      *p++ = (uint8_t)(col >> 8); *p++ = (uint8_t)col;
    }
  }
  lcdBlit(x, y, cw, chh, glyphBuf);
  return x + cw;
}
static int drawText(int x, int y, const char *s, uint16_t fg, uint16_t bg = C_BG, uint8_t sc = 1) {
  while (*s) x = drawChar(x, y, *s++, fg, bg, sc);
  return x;
}
static int drawTextf(int x, int y, uint16_t fg, uint16_t bg, uint8_t sc, const char *fmt, ...)
    __attribute__((format(printf, 6, 7)));
static int drawTextf(int x, int y, uint16_t fg, uint16_t bg, uint8_t sc, const char *fmt, ...) {
  char b[48]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  return drawText(x, y, b, fg, bg, sc);
}
static void drawButton(const Button &b, bool on, uint16_t onColor, int cw = 16, int chh = 26) {
  const uint16_t bg = on ? onColor : C_DGRAY, fg = on ? C_BLACK : C_WHITE;
  lcdFill(b.x, b.y, b.w, b.h, bg);
  lcdRect(b.x, b.y, b.w, b.h, on ? C_WHITE : C_GRAY);
  int x = b.x + (b.w - (int)strlen(b.label) * cw) / 2;
  const int y = b.y + (b.h - chh) / 2;
  for (const char *c = b.label; *c; c++) x = drawCharWH(x, y, *c, fg, bg, cw, chh);
}
static bool hit(const Button &b, uint16_t x, uint16_t y) { return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h; }

// =================================== FT6336 ===================================
static bool touchOk = false;
static bool touchRead(uint16_t &x, uint16_t &y) {
  uint8_t b[5];
  if (!i2cRd(A_FT6336, 0x02, b, 5)) return false;
  const uint8_t n = b[0] & 0x0F;
  if (n == 0 || n > 2) return false;
  const uint16_t rx = ((uint16_t)(b[1] & 0x0F) << 8) | b[2], ry = ((uint16_t)(b[3] & 0x0F) << 8) | b[4];
  if (rx >= LCD_W || ry >= LCD_H) return false;
  x = rx; y = ry; return true;
}

// =================================== QMI8658 ==================================
static bool imuOk = false;
static bool imuInit() {
  i2cW8(A_QMI8658, 0x60, 0xB0); delay(30);                                   // soft reset
  if (i2cR8(A_QMI8658, 0x00) != 0x05) return false;                          // WHO_AM_I
  bool ok = i2cW8(A_QMI8658, 0x02, 0x60) && i2cW8(A_QMI8658, 0x03, 0x25) &&   // auto-inc; +/-8 g @ 250 Hz
            i2cW8(A_QMI8658, 0x04, 0x55) && i2cW8(A_QMI8658, 0x06, 0x00) &&   // +/-512 dps @ 250 Hz; no LPF
            i2cW8(A_QMI8658, 0x08, 0x03);                                       // aEN | gEN
  delay(50);
  return ok;
}
static bool imuRead(ImuData &d) {
  uint8_t b[14];
  if (!i2cRd(A_QMI8658, 0x33, b, 14)) return false;                          // TEMP, AX..AZ, GX..GZ (LE)
  auto s16 = [&](int i) { return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8)); };
  d.t = s16(0) / 256.0f;
  d.ax = s16(2) / 4096.0f; d.ay = s16(4) / 4096.0f; d.az = s16(6) / 4096.0f;
  d.gx = s16(8) / 64.0f;   d.gy = s16(10) / 64.0f;  d.gz = s16(12) / 64.0f;
  return true;
}

// ==================================== SHTC3 ===================================
static bool shtOk = false;
static uint8_t crc8(const uint8_t *d, int n) {   // CRC-8, poly 0x31, init 0xFF
  uint8_t c = 0xFF;
  for (int i = 0; i < n; i++) { c ^= d[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1); }
  return c;
}
static bool shtRead(float &t, float &rh) {
  if (!i2cCmd16(A_SHTC3, 0x3517)) return false;   // wake-up
  delayMicroseconds(300);
  if (!i2cCmd16(A_SHTC3, 0x7866)) return false;   // T first, no clock stretching, normal mode
  delay(15);                                        // tMEAS max 12.1 ms
  uint8_t b[6];
  bool ok = Wire.requestFrom((int)A_SHTC3, 6) == 6;
  for (int i = 0; ok && i < 6; i++) b[i] = (uint8_t)Wire.read();
  i2cCmd16(A_SHTC3, 0xB098);                        // sleep
  if (!ok || crc8(b, 2) != b[2] || crc8(b + 3, 2) != b[5]) return false;
  t  = -45.0f + 175.0f * (float)((b[0] << 8) | b[1]) / 65535.0f;
  rh = 100.0f * (float)((b[3] << 8) | b[4]) / 65535.0f;
  return true;
}

// ================================== PCF85063A =================================
static bool rtcOk = false, rtcWasSet = false;
static uint8_t bcd2b(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static uint8_t b2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static bool rtcRead(RtcTime &t) {
  uint8_t b[7];
  if (!i2cRd(A_PCF85063, 0x04, b, 7)) return false;
  t.osStop = b[0] & 0x80;                            // oscillator-stop flag
  t.sec = bcd2b(b[0] & 0x7F); t.min = bcd2b(b[1] & 0x7F); t.hour = bcd2b(b[2] & 0x3F);
  t.day = bcd2b(b[3] & 0x3F); t.wday = b[4] & 0x07; t.mon = bcd2b(b[5] & 0x1F); t.year = 2000 + bcd2b(b[6]);
  return true;
}
static uint8_t dayOfWeek(int y, int m, int d) { static const int t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4}; if (m < 3) y--; return (uint8_t)((y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7); }
static bool rtcWrite(int y, int mo, int d, int h, int mi, int s) {  // local time, clears the OS flag
  Wire.beginTransmission(A_PCF85063);
  Wire.write(0x04);
  Wire.write(b2bcd((uint8_t)s)); Wire.write(b2bcd((uint8_t)mi)); Wire.write(b2bcd((uint8_t)h)); Wire.write(b2bcd((uint8_t)d));
  Wire.write(dayOfWeek(y, mo, d)); Wire.write(b2bcd((uint8_t)mo)); Wire.write(b2bcd((uint8_t)(y - 2000)));
  return Wire.endTransmission() == 0;
}
static bool rtcSetFromBuild() {   // __DATE__ = "Mmm dd yyyy", __TIME__ = "hh:mm:ss" (PC local time at compile)
  static const char *M = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *d = __DATE__, *tm = __TIME__;
  char mm[4] = {d[0], d[1], d[2], 0};
  const uint8_t mon = (uint8_t)((strstr(M, mm) - M) / 3 + 1);
  Wire.beginTransmission(A_PCF85063);
  Wire.write(0x04);
  Wire.write(b2bcd((uint8_t)atoi(tm + 6)));          // seconds, clears OS
  Wire.write(b2bcd((uint8_t)atoi(tm + 3)));
  Wire.write(b2bcd((uint8_t)atoi(tm)));
  Wire.write(b2bcd((uint8_t)atoi(d + 4)));
  Wire.write(0);                                       // weekday (unused)
  Wire.write(b2bcd(mon));
  Wire.write(b2bcd((uint8_t)(atoi(d + 7) - 2000)));
  return Wire.endTransmission() == 0;
}
static bool rtcInit() {
  RtcTime t;
  if (!rtcRead(t)) return false;
  if (t.osStop || t.year < 2025 || t.mon < 1 || t.mon > 12) rtcWasSet = rtcSetFromBuild();
  return true;
}

// =================================== ES8311 ===================================
static bool codecOk = false;
static bool esW(uint8_t r, uint8_t v) { return i2cW8(A_ES8311, r, v); }
static int  esR(uint8_t r) { return i2cR8(A_ES8311, r); }
static bool es8311Init() {   // equivalent to esp_codec_dev/es8311.c: slave, use_mclk=false, 16 kHz / 16 bit
  bool ok = true;
  ok &= esW(0x0D, 0xFA); ok &= esW(0x44, 0x08); ok &= esW(0x44, 0x08);
  ok &= esW(0x01, 0x30); ok &= esW(0x02, 0x00); ok &= esW(0x03, 0x10); ok &= esW(0x16, 0x24);
  ok &= esW(0x04, 0x10); ok &= esW(0x05, 0x00); ok &= esW(0x0B, 0x00); ok &= esW(0x0C, 0x00);
  ok &= esW(0x10, 0x1F); ok &= esW(0x11, 0x7F); ok &= esW(0x00, 0x80); ok &= esW(0x01, 0xBF);   // MCLK <- SCLK
  int r = esR(0x06); ok &= r >= 0 && esW(0x06, (uint8_t)(r & ~0x20));
  ok &= esW(0x13, 0x10); ok &= esW(0x1B, 0x0A); ok &= esW(0x1C, 0x6A); ok &= esW(0x44, 0x08);
  int r9 = esR(0x09), rA = esR(0x0A); if (r9 < 0 || rA < 0) return false;
  ok &= esW(0x09, (uint8_t)((r9 | 0x0C) & 0xFC)); ok &= esW(0x0A, (uint8_t)((rA | 0x0C) & 0xFC));  // 16 bit, I2S
  ok &= esW(0x02, 0x18); ok &= esW(0x05, 0x00); ok &= esW(0x03, 0x10); ok &= esW(0x04, 0x20);       // 512 kHz x8
  ok &= esW(0x07, 0x00); ok &= esW(0x08, 0xFF);
  r = esR(0x06); ok &= r >= 0 && esW(0x06, (uint8_t)((r & 0xE0) | 0x03));
  ok &= esW(0x00, 0x80); ok &= esW(0x01, 0xBF);
  r9 = esR(0x09); rA = esR(0x0A); if (r9 < 0 || rA < 0) return false;
  ok &= esW(0x09, (uint8_t)(r9 & 0xBF)); ok &= esW(0x0A, (uint8_t)(rA & 0xBF));
  ok &= esW(0x17, 0xBF); ok &= esW(0x0E, 0x02); ok &= esW(0x12, 0x00); ok &= esW(0x14, 0x1A);
  ok &= esW(0x0D, 0x01); ok &= esW(0x15, 0x40); ok &= esW(0x37, 0x08); ok &= esW(0x45, 0x00);
  ok &= esW(0x16, cfg.micGain & 0x07); ok &= esW(0x32, dacVolReg());
  r = esR(0x31); ok &= r >= 0 && esW(0x31, (uint8_t)(r & 0x9F));   // DAC unmute
  return ok;
}

// ========================= AFSK 1200 demodulator (RX) ==========================
// Bell 202: mark 1200 Hz = '1', space 2200 Hz = '0', 1200 Bd, NRZI, HDLC framing, FCS CRC-16/X.25.
// Fixed-point quadrature correlators over one bit period, energy comparison, DPLL bit sync.
static const int      DM_N        = 13;                        // ~1 bit at 16 kHz (13.33 samples)
static const uint32_t DM_INC_MARK = 322122547u;                // 1200 / 16000 * 2^32
static const uint32_t DM_INC_SPC  = 590558003u;                // 2200 / 16000 * 2^32
static const uint32_t DM_PLL_STEP = 322122547u;                // 1200 Bd / 16000 * 2^32
static int16_t  loTab[256];                                     // unit sine, Q14
static QueueHandle_t ax25Queue = nullptr;
static volatile bool     ax25RxEnabled = false;
static volatile uint32_t ax25Ok = 0, ax25Bad = 0, ax25LastFlagMs = 0;

static struct {
  int32_t  hpX, hpY;                                            // DC-blocking high-pass state
  uint32_t phM, phS;                                            // local oscillators
  int32_t  rMi[DM_N], rMq[DM_N], rSi[DM_N], rSq[DM_N];          // product rings
  int32_t  sMi, sMq, sSi, sSq;                                  // running sums
  int      ri;
  int32_t  pll;
  uint8_t  lastLevel, lastSampled;
  uint32_t bitStream;                                           // last received bits (LSB = newest)
  uint32_t bitBuf;                                              // byte assembler with sentinel
  bool     inFrame;
  uint16_t len;
  uint8_t  buf[AX25_MAX_FRAME];
} dm;

static uint16_t crcX25(const uint8_t *d, int n) {              // CRC-16/X.25: refl. 0x8408, init/xorout 0xFFFF
  uint16_t c = 0xFFFF;
  for (int i = 0; i < n; i++) { c ^= d[i]; for (int b = 0; b < 8; b++) c = (c & 1) ? (uint16_t)((c >> 1) ^ 0x8408) : (uint16_t)(c >> 1); }
  return (uint16_t)~c;
}
static void hdlcBit(uint8_t bit) {                              // bit after NRZI decoding
  dm.bitStream = (dm.bitStream << 1) | bit;
  if ((dm.bitStream & 0xFF) == 0x7E) {                          // flag 01111110
    ax25LastFlagMs = millis();
    if (dm.inFrame && dm.len >= 17) {                           // 14 addr + ctrl + 2 FCS minimum
      const uint16_t fcs = (uint16_t)(dm.buf[dm.len - 2] | (dm.buf[dm.len - 1] << 8));
      if (crcX25(dm.buf, dm.len - 2) == fcs) {
        Ax25Frame f; f.len = dm.len - 2; memcpy(f.data, dm.buf, f.len);
        if (ax25Queue) xQueueSend(ax25Queue, &f, 0);
        ax25Ok = ax25Ok + 1;
      } else ax25Bad = ax25Bad + 1;
    }
    dm.inFrame = true; dm.len = 0; dm.bitBuf = 0x80;
    return;
  }
  if ((dm.bitStream & 0x7F) == 0x7F) { dm.inFrame = false; return; }   // 7 ones: abort / idle
  if (!dm.inFrame) return;
  if ((dm.bitStream & 0x3F) == 0x3E) return;                   // 0 after five 1s: stuffed bit
  if (bit) dm.bitBuf |= 0x100;
  if (dm.bitBuf & 1) {                                           // sentinel reached: byte complete
    if (dm.len < AX25_MAX_FRAME) dm.buf[dm.len++] = (uint8_t)(dm.bitBuf >> 1);
    else dm.inFrame = false;
    dm.bitBuf = 0x80;
    return;
  }
  dm.bitBuf >>= 1;
}
static void afskDemodSample(int16_t x) {
  // 1) DC block: y = x - x[-1] + 0.98 y[-1]
  const int32_t y = x - dm.hpX + ((dm.hpY * 32113) >> 15);
  dm.hpX = x; dm.hpY = y;
  // 2) Quadrature mixing with mark/space LOs, sliding sums over one bit
  const int32_t cm = loTab[(uint8_t)((dm.phM >> 24) + 64)], sm = loTab[dm.phM >> 24];
  const int32_t cs = loTab[(uint8_t)((dm.phS >> 24) + 64)], ss = loTab[dm.phS >> 24];
  dm.phM += DM_INC_MARK; dm.phS += DM_INC_SPC;
  const int32_t pMi = (y * cm) >> 14, pMq = (y * sm) >> 14, pSi = (y * cs) >> 14, pSq = (y * ss) >> 14;
  dm.sMi += pMi - dm.rMi[dm.ri]; dm.rMi[dm.ri] = pMi;
  dm.sMq += pMq - dm.rMq[dm.ri]; dm.rMq[dm.ri] = pMq;
  dm.sSi += pSi - dm.rSi[dm.ri]; dm.rSi[dm.ri] = pSi;
  dm.sSq += pSq - dm.rSq[dm.ri]; dm.rSq[dm.ri] = pSq;
  if (++dm.ri == DM_N) dm.ri = 0;
  const int64_t eM = (int64_t)dm.sMi * dm.sMi + (int64_t)dm.sMq * dm.sMq;
  const int64_t eS = (int64_t)dm.sSi * dm.sSi + (int64_t)dm.sSq * dm.sSq;
  const uint8_t level = eM > eS ? 1 : 0;
  // 3) DPLL: sample when the 32-bit phase wraps positive -> negative; pull toward transitions
  const int32_t prev = dm.pll;
  dm.pll = (int32_t)((uint32_t)dm.pll + DM_PLL_STEP);
  if (prev > 0 && dm.pll < 0) {
    hdlcBit(level == dm.lastSampled ? 1 : 0);                   // NRZI: no change = 1
    dm.lastSampled = level;
  }
  if (level != dm.lastLevel) {
    dm.pll = (int32_t)(((int64_t)dm.pll * (dm.inFrame ? 190 : 128)) >> 8);   // inertia 0.74 / 0.50
    dm.lastLevel = level;
  }
}

// ========================= AFSK 1200 modulator (TX) ============================
static int16_t *txBuf = nullptr;
static const uint32_t TX_MAX_SAMPLES = 24000;                  // 1.5 s
static uint32_t txCount = 0;
static struct { uint32_t phase, bitAcc, n; bool mark; int ones; } md;

static void modBitRaw(uint8_t nrzBit) {                         // one channel bit (after stuffing), NRZI
  if (!nrzBit) md.mark = !md.mark;                              // '0' toggles the tone
  const uint32_t inc = md.mark ? DM_INC_MARK : DM_INC_SPC;
  do {
    if (md.n < TX_MAX_SAMPLES) txBuf[md.n++] = (int16_t)lrintf(AFSK_TX_AMPL * 2.0f * loTab[md.phase >> 24]);
    md.phase += inc;
    md.bitAcc += 1200;
  } while (md.bitAcc < AUDIO_FS);
  md.bitAcc -= AUDIO_FS;
}
static void modFlag() { for (int i = 0; i < 8; i++) modBitRaw((0x7E >> i) & 1); md.ones = 0; }
static void modByte(uint8_t b) {                                // LSB first with bit stuffing
  for (int i = 0; i < 8; i++) {
    const uint8_t bit = (b >> i) & 1;
    modBitRaw(bit);
    if (bit) { if (++md.ones == 5) { modBitRaw(0); md.ones = 0; } } else md.ones = 0;
  }
}
static int ax25Addr(uint8_t *p, const char *call, uint8_t ssid, bool last) {
  for (int i = 0; i < 6; i++) p[i] = (uint8_t)((*call ? *call++ : ' ') << 1);
  p[6] = (uint8_t)(0x60 | ((ssid & 0x0F) << 1) | (last ? 1 : 0));
  return 7;
}
static bool ax25BuildTx(const char *info) {                     // UI frame, no digipeater path
  if (!txBuf) return false;
  uint8_t f[AX25_MAX_FRAME]; int n = 0;
  n += ax25Addr(f + n, AX25_DEST, 0, false);
  n += ax25Addr(f + n, cfg.call, cfg.ssid, true);
  f[n++] = 0x03; f[n++] = 0xF0;                                  // UI, no layer 3
  for (; *info && n < AX25_MAX_FRAME - 2; info++) f[n++] = (uint8_t)*info;
  const uint16_t fcs = crcX25(f, n);
  f[n++] = (uint8_t)fcs; f[n++] = (uint8_t)(fcs >> 8);
  md.phase = 0; md.bitAcc = 0; md.n = 0; md.mark = true; md.ones = 0;
  for (int i = 0; i < AX25_TXDELAY_FLAGS; i++) modFlag();
  for (int i = 0; i < n; i++) modByte(f[i]);
  for (int i = 0; i < AX25_TXTAIL_FLAGS; i++) modFlag();
  txCount = md.n;
  return md.n < TX_MAX_SAMPLES;
}

// ===================== SSTV / RTTY DSP cores (run inside audioTask) ===============
// SSTV: complex mix to 1700 Hz, 2x boxcar-6 low-pass (image rejection ~28 dB, ~0.7 ms span),
// instantaneous frequency from the phase difference of consecutive samples -> int16 Hz ring.
static const uint32_t SS_RING = 65536;                            // 4.1 s of frequency samples
static int16_t *ssRing = nullptr;
static volatile uint32_t ssW = 0;
static volatile bool sstvRxOn = false;
static const int SS_L = 6;
static struct { int32_t bi1[SS_L], bq1[SS_L], bi2[SS_L], bq2[SS_L], si1, sq1, si2, sq2; int k; int64_t pi, pq; uint32_t ph; } se;
static void sstvFreqSample(int16_t x) {
  const int32_t c = loTab[(uint8_t)((se.ph >> 24) + 64)], s = loTab[se.ph >> 24];
  se.ph += 456340275u;                                             // 1700 / 16000 * 2^32
  const int32_t i0 = (x * c) >> 14, q0 = -((x * s) >> 14);
  se.si1 += i0 - se.bi1[se.k]; se.bi1[se.k] = i0; se.sq1 += q0 - se.bq1[se.k]; se.bq1[se.k] = q0;
  se.si2 += se.si1 - se.bi2[se.k]; se.bi2[se.k] = se.si1; se.sq2 += se.sq1 - se.bq2[se.k]; se.bq2[se.k] = se.sq1;
  if (++se.k == SS_L) se.k = 0;
  const int64_t I = se.si2, Q = se.sq2;
  const float cr = (float)(se.pi * Q - se.pq * I), dt = (float)(se.pi * I + se.pq * Q);
  se.pi = I; se.pq = Q;
  const float f = 1700.0f + atan2f(cr, dt) * (16000.0f / 6.2831853f);
  if (!ssRing) return;
  const uint32_t w = ssW;
  ssRing[w & (SS_RING - 1)] = (int16_t)f;
  ssW = w + 1;
}
// RTTY: mark/space quadrature correlators over one bit, energy decision, async UART (start, 5 data, stop)
static const float    RTTY_BAUDS[3] = {45.45f, 50.0f, 75.0f};
static const uint16_t RTTY_SHIFTS[4] = {170, 200, 425, 850};
static volatile int   rtBaudI = 0, rtShiftI = 0;
static volatile bool  rtRev = false, rttyRxOn = false;
static volatile float rtLow = 2125.0f, rtQ = 0;                    // lower tone (Hz), signal quality 0..1
static volatile uint32_t rtCfgSeq = 1;
static QueueHandle_t  rttyQ = nullptr;
static const int RT_NMAX = 352;
static const char RT_LTR[32] = {0, 'E', '\n', 'A', ' ', 'S', 'I', 'U', '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K',
                                'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q', 'O', 'B', 'G', 0, 'M', 'X', 'V', 0};
static const char RT_FIG[32] = {0, '3', '\n', '-', ' ', '\'', '8', '7', '\r', '$', '4', 0, ',', '!', ':', '(',
                                '5', '+', ')', '2', '#', '6', '0', '1', '9', '?', '&', 0, '.', '/', ';', 0};
static struct { uint32_t seq; int N; uint32_t incM, incS, phM, phS; int32_t rMi[RT_NMAX], rMq[RT_NMAX], rSi[RT_NMAX], rSq[RT_NMAX];
                int32_t sMi, sMq, sSi, sSq; int ri, st, cnt, bit; uint8_t lvl, last, sh; bool figs; float q; } rd;
static void rttyCfg() {
  rd.seq = rtCfgSeq;
  rd.N = (int)lrintf(16000.0f / RTTY_BAUDS[rtBaudI % 3]);
  const float lo = rtLow, hi = rtLow + RTTY_SHIFTS[rtShiftI % 4];
  const float mark = rtRev ? hi : lo, space = rtRev ? lo : hi;
  rd.incM = (uint32_t)((double)mark * 4294967296.0 / 16000.0); rd.incS = (uint32_t)((double)space * 4294967296.0 / 16000.0);
  memset(rd.rMi, 0, sizeof(rd.rMi)); memset(rd.rMq, 0, sizeof(rd.rMq)); memset(rd.rSi, 0, sizeof(rd.rSi)); memset(rd.rSq, 0, sizeof(rd.rSq));
  rd.sMi = rd.sMq = rd.sSi = rd.sSq = 0; rd.ri = 0; rd.st = 0; rd.figs = false;
}
static void rttyEmit(uint8_t code) {
  if (code == 0x1F) { rd.figs = false; return; }
  if (code == 0x1B) { rd.figs = true; return; }
  char c = rd.figs ? RT_FIG[code & 31] : RT_LTR[code & 31];
  if (c == ' ') rd.figs = false;                                   // unshift on space (USOS)
  if (c && rttyQ) xQueueSend(rttyQ, &c, 0);
}
static void rttyDemodSample(int16_t x) {
  if (rd.seq != rtCfgSeq) rttyCfg();
  const int32_t cm = loTab[(uint8_t)((rd.phM >> 24) + 64)], sm = loTab[rd.phM >> 24];
  const int32_t cs = loTab[(uint8_t)((rd.phS >> 24) + 64)], ss = loTab[rd.phS >> 24];
  rd.phM += rd.incM; rd.phS += rd.incS;
  const int32_t pMi = (x * cm) >> 14, pMq = (x * sm) >> 14, pSi = (x * cs) >> 14, pSq = (x * ss) >> 14;
  rd.sMi += pMi - rd.rMi[rd.ri]; rd.rMi[rd.ri] = pMi; rd.sMq += pMq - rd.rMq[rd.ri]; rd.rMq[rd.ri] = pMq;
  rd.sSi += pSi - rd.rSi[rd.ri]; rd.rSi[rd.ri] = pSi; rd.sSq += pSq - rd.rSq[rd.ri]; rd.rSq[rd.ri] = pSq;
  if (++rd.ri >= rd.N) rd.ri = 0;
  const int64_t eM = (int64_t)rd.sMi * rd.sMi + (int64_t)rd.sMq * rd.sMq, eS = (int64_t)rd.sSi * rd.sSi + (int64_t)rd.sSq * rd.sSq;
  const float qi = (eM + eS) > 0 ? fabsf((float)(eM - eS)) / (float)(eM + eS) : 0.0f;
  rd.q += (qi - rd.q) / rd.N; rtQ = rd.q;
  rd.lvl = eM > eS ? 1 : 0;
  rd.cnt++;
  switch (rd.st) {
    case 0: if (rd.last == 1 && rd.lvl == 0 && rd.q > 0.3f) { rd.st = 1; rd.cnt = 0; } break;   // start-bit edge
    case 1: if (rd.cnt >= rd.N / 2) { if (rd.lvl == 0) { rd.st = 2; rd.cnt = 0; rd.bit = 0; rd.sh = 0; } else rd.st = 0; } break;
    case 2: if (rd.cnt >= rd.N) { rd.cnt = 0; rd.sh |= (uint8_t)(rd.lvl << rd.bit); if (++rd.bit == 5) rd.st = 3; } break;
    case 3: if (rd.cnt >= rd.N) { if (rd.lvl == 1) rttyEmit(rd.sh); rd.st = 0; } break;       // stop bit must be mark
  }
  rd.last = rd.lvl;
}

// ================================ Audio task ==================================
static I2SClass i2s;
static int16_t  sineLut[256];
static int16_t *recBuf = nullptr;
static const uint32_t REC_SAMPLES = AUDIO_FS * REC_SECONDS;
static volatile AudioMode audioMode = AM_IDLE;
static const int16_t * volatile playSrc = nullptr;
static volatile uint32_t  tonePhaseInc = 0, recPos = 0, recLen = 0, playPos = 0, playLen = 0;
static volatile bool      recDoneFlag = false, playDoneFlag = false;
static volatile float     micRmsDb = -120.0f, micPeakDb = -120.0f;
static const uint32_t     SPEC_RING = 2048;                       // power of two
static int16_t            specRing[SPEC_RING];
static volatile uint32_t  specW = 0;
static volatile bool      specEnabled = false;

static uint32_t phaseIncFor(float f) { return (uint32_t)((double)f * 4294967296.0 / AUDIO_FS); }
static void startPlayback(const int16_t *src, uint32_t len) {
  audioMode = AM_IDLE; playSrc = src; playLen = len; playPos = 0; audioMode = AM_PLAY;
}

static void audioTask(void *) {
  static int16_t tx[AUDIO_BLOCK * 2], rx[AUDIO_BLOCK * 2];
  uint32_t phase = 0;
  const int slot = MIC_USE_RIGHT ? 1 : 0;
  for (;;) {
    const AudioMode m = audioMode;
    const uint32_t inc = tonePhaseInc;
    for (int i = 0; i < AUDIO_BLOCK; i++) {
      int16_t s = 0;
      if (m == AM_TONE) { s = sineLut[phase >> 24]; phase += inc; }
      else if (m == AM_PLAY) {
        if (playPos < playLen && playSrc) { s = playSrc[playPos]; playPos = playPos + 1; }
        else if (audioMode == AM_PLAY) { audioMode = AM_IDLE; playDoneFlag = true; }
      }
      tx[2 * i] = s; tx[2 * i + 1] = s;
    }
    i2s.write((const uint8_t *)tx, sizeof(tx));
    const size_t n = i2s.readBytes((char *)rx, sizeof(rx));
    const int frames = (int)(n / 4);
    if (frames <= 0) { vTaskDelay(1); continue; }
    if (audioMode == AM_REC && recBuf) {
      uint32_t rp = recPos;
      for (int i = 0; i < frames && rp < REC_SAMPLES; i++) recBuf[rp++] = rx[2 * i + slot];
      recPos = rp;
      if (rp >= REC_SAMPLES) { recLen = rp; audioMode = AM_IDLE; recDoneFlag = true; }
    }
    if (ax25RxEnabled) for (int i = 0; i < frames; i++) afskDemodSample(rx[2 * i + slot]);
    if (sstvRxOn) for (int i = 0; i < frames; i++) sstvFreqSample(rx[2 * i + slot]);
    if (rttyRxOn) for (int i = 0; i < frames; i++) rttyDemodSample(rx[2 * i + slot]);
    if (specEnabled) { uint32_t w = specW; for (int i = 0; i < frames; i++) specRing[(w++) & (SPEC_RING - 1)] = rx[2 * i + slot]; specW = w; }
    int32_t sum = 0;
    for (int i = 0; i < frames; i++) sum += rx[2 * i + slot];
    const int32_t mean = sum / frames;
    float acc = 0; int32_t pk = 0;
    for (int i = 0; i < frames; i++) {
      int32_t v = rx[2 * i + slot] - mean;
      acc += (float)v * (float)v;
      if (v < 0) v = -v;
      if (v > pk) pk = v;
    }
    micRmsDb  = 20.0f * log10f(sqrtf(acc / frames) / 32768.0f + 1e-6f);
    micPeakDb = 20.0f * log10f((float)pk / 32768.0f + 1e-6f);
  }
}
static bool audioInit() {
  for (int i = 0; i < 256; i++) {
    const float s = sinf(2.0f * (float)M_PI * i / 256.0f);
    sineLut[i] = (int16_t)lrintf(TONE_AMPL_FS * 32767.0f * s);
    loTab[i]   = (int16_t)lrintf(16384.0f * s);
  }
  if (psramFound()) {
    recBuf = (int16_t *)heap_caps_malloc(REC_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    txBuf  = (int16_t *)heap_caps_malloc(TX_MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!txBuf) txBuf = (int16_t *)malloc(TX_MAX_SAMPLES * sizeof(int16_t));   // AX.25 TX works without PSRAM
  ax25Queue = xQueueCreate(4, sizeof(Ax25Frame));
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN, -1);
  if (!i2s.begin(I2S_MODE_STD, AUDIO_FS, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) return false;
  delay(10);                                        // BCLK running before configuring the codec
  if (!i2cPresent(A_ES8311)) return false;
  codecOk = es8311Init();
  xTaskCreate(audioTask, "audio", 4096, nullptr, 5, nullptr);
  return codecOk;
}

// =============================== BF3901 camera ================================
static const int    CAM_W = 240, CAM_H = 320, CAM_FHDR = 4, CAM_LHDR = 6;
static const size_t CAM_LINE  = CAM_W * 2 + CAM_LHDR;            // 486 bytes
static const size_t CAM_FRAME = CAM_LINE * CAM_H + CAM_FHDR;      // 155524 bytes
static const int    CAM_NBUF = 3;
static const int    CAM_X = 40, CAM_Y = 58;
enum { CB_FREE = 0, CB_CAPT = 1, CB_READY = 2, CB_DRAW = 3 };
static uint8_t  *camBuf[CAM_NBUF];
static volatile uint8_t  camSt[CAM_NBUF];
static volatile uint32_t camSeq[CAM_NBUF];
static size_t   camBufSize = 0;
static parlio_rx_unit_handle_t      camRx = nullptr;
static parlio_rx_delimiter_handle_t camDelim = nullptr;
static SemaphoreHandle_t camDoneSem = nullptr;
static portMUX_TYPE camMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool     camInflight = false, camRunning = false;
static volatile int      camCapIdx = -1;
static volatile uint32_t camSeqCtr = 0, camDropFirst = 0, camFramesOk = 0, camFramesBad = 0, camOverrun = 0;
static bool     camInited = false;
static char     camErr[41] = "";

static bool camRxDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *) {
  if (!camInflight) return false;          // forced EOF with no transaction in progress
  camInflight = false;
  BaseType_t w = pdFALSE;
  xSemaphoreGiveFromISR(camDoneSem, &w);
  return w == pdTRUE;
}
static void camVsyncIsr(void *) {          // VSYNC/CS rising edge = end of frame
  bool y = false;
  parlio_rx_unit_trigger_fake_eof(camRx, &y);
  if (y) portYIELD_FROM_ISR();
}
static bool camSubmit(int idx) {
  parlio_receive_config_t rc = {};
  rc.delimiter = camDelim;
  camCapIdx = idx; camSt[idx] = CB_CAPT; camInflight = true;
  if (parlio_rx_unit_receive(camRx, camBuf[idx], camBufSize, &rc) != ESP_OK) {
    camInflight = false; camSt[idx] = CB_FREE; return false;
  }
  return true;
}
static void camTask(void *) {
  for (;;) {
    if (xSemaphoreTake(camDoneSem, pdMS_TO_TICKS(250)) == pdTRUE) {
      const int idx = camCapIdx;
      esp_cache_msync(camBuf[idx], camBufSize, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      taskENTER_CRITICAL(&camMux);
      if (camDropFirst) { camDropFirst = camDropFirst - 1; camSt[idx] = CB_FREE; }
      else { camSeqCtr = camSeqCtr + 1; camSt[idx] = CB_READY; camSeq[idx] = camSeqCtr; }
      taskEXIT_CRITICAL(&camMux);
    }
    if (!camRunning || camInflight) continue;
    int pick = -1;
    taskENTER_CRITICAL(&camMux);
    for (int i = 0; i < CAM_NBUF && pick < 0; i++) if (camSt[i] == CB_FREE) pick = i;
    if (pick < 0) {                        // no free buffer: recycle the oldest ready frame
      uint32_t best = UINT32_MAX;
      for (int i = 0; i < CAM_NBUF; i++) if (camSt[i] == CB_READY && camSeq[i] < best) { best = camSeq[i]; pick = i; }
      if (pick >= 0) { camSt[pick] = CB_FREE; camOverrun = camOverrun + 1; }
    }
    taskEXIT_CRITICAL(&camMux);
    if (pick >= 0) camSubmit(pick);
  }
}
static bool sccbW(uint8_t r, uint8_t v) { return i2cW8(A_BF3901, r, v); }
static int  sccbR(uint8_t r) { uint8_t v; return i2cRd(A_BF3901, r, &v, 1, false) ? v : -1; }

static bool camInit() {
  if (!psramFound()) { snprintf(camErr, sizeof(camErr), "PSRAM disabled"); return false; }
  // 1) Sensor over SCCB
  Wire.setClock(SCCB_FREQ_HZ);
  const int h = sccbR(0xFC), l = sccbR(0xFD);
  if (h < 0 || l < 0 || ((h << 8) | l) != 0x3901) {
    Wire.setClock(I2C_FREQ_HZ);
    snprintf(camErr, sizeof(camErr), "BF3901 not found (PID %02X%02X)", h & 0xFF, l & 0xFF);
    return false;
  }
  bool ok = true;
  for (size_t i = 0; i < sizeof(bf3901_2bit_240x320_rgb565) / sizeof(Bf3901Reg) && ok; i++) {
    const Bf3901Reg &r = bf3901_2bit_240x320_rgb565[i];
    if (r.reg == BF3901_REG_DELAY) delay(r.val); else ok = sccbW(r.reg, r.val);
  }
  int r1e = sccbR(0x1E);                     // bit5 = H mirror, bit4 = V flip
  if (r1e >= 0) sccbW(0x1E, (uint8_t)((r1e & ~0x30) | (cfg.hmirror ? 0x20 : 0) | (cfg.vflip ? 0x10 : 0)));
  sccbW(0x09, 0x13);                         // standby until the CAM page is opened
  Wire.setClock(I2C_FREQ_HZ);
  if (!ok) { snprintf(camErr, sizeof(camErr), "SCCB register write failed"); return false; }
  // 2) Pins as inputs (GPIO11/12 come up as UART0 console pins)
  gpio_config_t io = {};
  io.pin_bit_mask = (1ULL << PIN_CAM_D0) | (1ULL << PIN_CAM_D1) | (1ULL << PIN_CAM_PCLK) | (1ULL << PIN_CAM_VSYNC);
  io.mode = GPIO_MODE_INPUT; io.pull_up_en = GPIO_PULLUP_ENABLE; io.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&io) != ESP_OK) { snprintf(camErr, sizeof(camErr), "gpio_config failed"); return false; }
  // 3) Cache-line aligned PSRAM buffers
  size_t align = 64;
  esp_cache_get_alignment(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA, &align);
  if (align < 4) align = 4;
  camBufSize = (CAM_FRAME + align - 1) / align * align;
  for (int i = 0; i < CAM_NBUF; i++) {
    camBuf[i] = (uint8_t *)heap_caps_aligned_alloc(align, camBufSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!camBuf[i]) camBuf[i] = (uint8_t *)heap_caps_aligned_alloc(align, camBufSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!camBuf[i]) { snprintf(camErr, sizeof(camErr), "out of PSRAM"); return false; }
    esp_cache_msync(camBuf[i], camBufSize, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    camSt[i] = CB_FREE;
  }
  // 4) PARLIO RX: 2 data bits, external 24 MHz PCLK, VSYNC as active-low valid signal
  parlio_rx_unit_config_t uc = {};
  uc.trans_queue_depth = 1;
  uc.max_recv_size = camBufSize;
  uc.data_width = 2;
  uc.clk_src = PARLIO_CLK_SRC_EXTERNAL;
  uc.ext_clk_freq_hz = 24000000;
  uc.exp_clk_freq_hz = 24000000;
  uc.clk_in_gpio_num = (gpio_num_t)PIN_CAM_PCLK;
  uc.clk_out_gpio_num = GPIO_NUM_NC;
  uc.valid_gpio_num = (gpio_num_t)PIN_CAM_VSYNC;
  for (int i = 0; i < PARLIO_RX_UNIT_MAX_DATA_WIDTH; i++) uc.data_gpio_nums[i] = GPIO_NUM_NC;
  uc.data_gpio_nums[0] = (gpio_num_t)PIN_CAM_D0;
  uc.data_gpio_nums[1] = (gpio_num_t)PIN_CAM_D1;
  esp_err_t e = parlio_new_rx_unit(&uc, &camRx);
  if (e != ESP_OK) { snprintf(camErr, sizeof(camErr), "parlio_new_rx_unit %s", esp_err_to_name(e)); return false; }
  parlio_rx_level_delimiter_config_t dc = {};
  dc.valid_sig_line_id = PARLIO_RX_UNIT_MAX_DATA_WIDTH - 1;
  dc.sample_edge = PARLIO_SAMPLE_EDGE_POS;
  dc.bit_pack_order = PARLIO_BIT_PACK_ORDER_MSB;
  dc.eof_data_len = 0;
  dc.timeout_ticks = 0;
  dc.flags.active_low_en = 1;
  e = parlio_new_rx_level_delimiter(&dc, &camDelim);
  if (e != ESP_OK) { snprintf(camErr, sizeof(camErr), "delimiter %s", esp_err_to_name(e)); return false; }
  parlio_rx_event_callbacks_t cb = {};
  cb.on_receive_done = camRxDone;
  parlio_rx_unit_register_event_callbacks(camRx, &cb, nullptr);
  camDoneSem = xSemaphoreCreateBinary();
  e = parlio_rx_unit_enable(camRx, true);
  if (e != ESP_OK) { snprintf(camErr, sizeof(camErr), "parlio enable %s", esp_err_to_name(e)); return false; }
  // 5) End-of-frame ISR on VSYNC
  e = gpio_install_isr_service(0);
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) { snprintf(camErr, sizeof(camErr), "gpio isr service"); return false; }
  gpio_set_intr_type((gpio_num_t)PIN_CAM_VSYNC, GPIO_INTR_POSEDGE);
  gpio_isr_handler_add((gpio_num_t)PIN_CAM_VSYNC, camVsyncIsr, nullptr);
  gpio_intr_disable((gpio_num_t)PIN_CAM_VSYNC);
  xTaskCreate(camTask, "cam", 4096, nullptr, 6, nullptr);
  camInited = true;
  return true;
}
static void camStart() {
  if (!camInited) return;
  camDropFirst = 2;                          // 1st packet is not image data + 1st frame may be partial
  gpio_intr_enable((gpio_num_t)PIN_CAM_VSYNC);
  Wire.setClock(SCCB_FREQ_HZ);
  const int r1e = sccbR(0x1E);                                     // apply mirror/flip from settings
  if (r1e >= 0) sccbW(0x1E, (uint8_t)((r1e & ~0x30) | (cfg.hmirror ? 0x20 : 0) | (cfg.vflip ? 0x10 : 0)));
  sccbW(0x09, 0x03); Wire.setClock(I2C_FREQ_HZ);                   // stream on
  camRunning = true;
}
static void camStop() {
  if (!camInited) return;
  camRunning = false;
  Wire.setClock(SCCB_FREQ_HZ); sccbW(0x09, 0x13); Wire.setClock(I2C_FREQ_HZ);   // standby
  gpio_intr_disable((gpio_num_t)PIN_CAM_VSYNC);
}
static const uint8_t CAM_FH[4] = {0xFF, 0xFF, 0xFF, 0x00}, CAM_LH[4] = {0xFF, 0xFF, 0xFF, 0x40};
static bool camDraw(const uint8_t *f) {
  if (memcmp(f, CAM_FH, 4) != 0) return false;
  const uint8_t *p = f + CAM_FHDR;
  for (int i = 0; i < CAM_H; i++) if (memcmp(p + i * CAM_LINE, CAM_LH, 4) != 0) return false;
  SPI.beginTransaction(lcdSpi); digitalWrite(PIN_LCD_CS, LOW);
  lcdWindowRaw(CAM_X, CAM_Y, CAM_X + CAM_W - 1, CAM_Y + CAM_H - 1);
  for (int i = 0; i < CAM_H; i++) {
    const uint8_t *s = p + i * CAM_LINE + CAM_LHDR;
    if (CAM_SWAP_BYTES) { for (int x = 0; x < CAM_W * 2; x += 2) { lineBuf[x] = s[x + 1]; lineBuf[x + 1] = s[x]; } SPI.writeBytes(lineBuf, CAM_W * 2); }
    else SPI.writeBytes(s, CAM_W * 2);
  }
  digitalWrite(PIN_LCD_CS, HIGH); SPI.endTransaction();
  return true;
}

// =================================== microSD ==================================
static bool sdOk = false;
static bool saveWav(const char *path, const int16_t *pcm, uint32_t n) {
  if (!sdOk) return false;
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  const uint32_t data = n * 2;
  uint8_t h[44] = {'R','I','F','F',0,0,0,0,'W','A','V','E','f','m','t',' ',16,0,0,0,1,0,1,0,
                   0,0,0,0,0,0,0,0,2,0,16,0,'d','a','t','a',0,0,0,0};
  auto put32 = [&](int o, uint32_t v) { h[o] = (uint8_t)v; h[o+1] = (uint8_t)(v >> 8); h[o+2] = (uint8_t)(v >> 16); h[o+3] = (uint8_t)(v >> 24); };
  put32(4, 36 + data); put32(24, AUDIO_FS); put32(28, AUDIO_FS * 2); put32(40, data);
  bool ok = f.write(h, 44) == 44 && f.write((const uint8_t *)pcm, data) == data;
  f.close();
  return ok;
}

// ===================================== UI =====================================
static Page page = PG_HOME;
static bool onBattery = false;
static bool foxTrack = false;
static uint8_t stView = SV_MAIN;
static char wakeMsg[41] = "Power-on";
static const Button bWifi = {4, 382, 76, 44, "WIFI"}, bBle = {82, 382, 76, 44, "BLE"}, bIeee = {160, 382, 76, 44, "15.4"}, bEspNow = {238, 382, 78, 44, "ESPNOW"};
static const Button bRec  = {6, 122, 100, 52, "REC"}, bPlay = {110, 122, 100, 52, "PLAY"}, bStop = {214, 122, 100, 52, "STOP"};
static const Button bTone[4] = {{6, 352, 74, 46, "440"}, {84, 352, 74, 46, "1k"}, {162, 352, 74, 46, "3k"}, {240, 352, 74, 46, "OFF"}};
static const Button bSend = {6, 380, 150, 48, "SEND"}, bClear = {164, 380, 150, 48, "CLEAR"};
static const float  toneFreq[4] = {440.0f, 1000.0f, 3000.0f, 0.0f};
static int    toneSel = 3;
static char   audioMsg[41] = "Ready";
static bool   audioMsgDirty = true, waveValid = false;
static float  recGain = 1.0f, recPeakDbRaw = -120.0f;
static uint32_t camFpsT0 = 0, camFpsN = 0;
static uint32_t txSeq = 0;
static bool   txActive = false;

static void drawHeader() {
  lcdFill(0, 0, LCD_W, 36, C_BG);
  int x = drawText(8, 5, "E", C_BRAND_E, C_BG, 2);         // "Electgpl": red E, white "lectgpl"
  drawText(x, 5, "lectgpl", C_WHITE, C_BG, 2);
  lcdFill(0, 34, LCD_W, 2, C_ACCENT);
}
// ------------------------------------------------ header: clock + battery icon
static uint32_t hdrMsgUntil = 0;
static int      hdrBatPct = -2, hdrBatState = -1;
static uint16_t batBuf[33 * 14];
static const int BAT_X = 283;
static bool inPoly(const float *px, const float *py, int n, float x, float y) {   // even-odd rule
  bool c = false;
  for (int i = 0, j = n - 1; i < n; j = i++)
    if (((py[i] > y) != (py[j] > y)) && (x < (px[j] - px[i]) * (y - py[i]) / (py[j] - py[i]) + px[i])) c = !c;
  return c;
}
static void drawBattery(bool force) {
  if (!axpOk) return;
  const int s1 = i2cR8(A_AXP, 0x00), s2 = i2cR8(A_AXP, 0x01);
  if (s1 < 0 || s2 < 0) return;
  const bool present = s1 & 0x08, vbus = s1 & 0x20;
  const int pct = present ? constrain(i2cR8(A_AXP, 0xA4), 0, 100) : -1;
  const int state = (present ? 1 : 0) | (vbus ? 2 : 0);
  if (!force && pct == hdrBatPct && state == hdrBatState) return;
  hdrBatPct = pct; hdrBatState = state;
  static const float bx[6] = {18, 11, 16, 14, 21, 16}, by[6] = {1, 8, 8, 13, 6, 6};   // lightning bolt
  const uint16_t outl = be16(present ? C_WHITE : C_GRAY), bg = be16(C_BG);
  const uint16_t fill = be16(vbus ? C_GREEN : pct > 50 ? C_GREEN : pct > 20 ? C_YELLOW : C_RED);
  const int fw = present ? (26 * pct + 50) / 100 : 0;
  for (int y = 0; y < 14; y++)
    for (int x = 0; x < 33; x++) {
      uint16_t c = bg;
      const bool body = x < 30, edge = body && (x == 0 || x == 29 || y == 0 || y == 13);
      if (edge || (x >= 30 && y >= 4 && y <= 9)) c = outl;
      else if (body && x >= 2 && x < 2 + fw && y >= 2 && y <= 11) c = fill;
      if (vbus && body) {
        const bool in = inPoly(bx, by, 6, x + 0.5f, y + 0.5f);
        const bool nb = inPoly(bx, by, 6, x - 0.5f, y + 0.5f) || inPoly(bx, by, 6, x + 1.5f, y + 0.5f) ||
                        inPoly(bx, by, 6, x + 0.5f, y - 0.5f) || inPoly(bx, by, 6, x + 0.5f, y + 1.5f);
        if (in) c = be16(C_WHITE); else if (nb) c = 0;                 // white bolt with black outline
      }
      batBuf[y * 33 + x] = c;
    }
  lcdBlit(BAT_X, 4, 33, 14, (const uint8_t *)batBuf);
  char t[6]; if (present) snprintf(t, sizeof(t), "%d%%", pct); else snprintf(t, sizeof(t), "USB");
  lcdFill(BAT_X - 8, 20, 41, 13, C_BG);
  drawText(BAT_X + 33 - (int)strlen(t) * FONT_W, 20, t, vbus ? C_GREEN : C_WHITE);
}
static void hdrMessage(const char *l1, const char *l2, uint16_t col) {   // 3 s notice over date/time
  drawTextf(150, 4, col, C_BG, 1, "%-16.16s", l1);
  drawTextf(150, 18, col, C_BG, 1, "%-16.16s", l2);
  hdrMsgUntil = millis() + 3000;
}
static void drawClock() {
  drawBattery(false);
  if (millis() < hdrMsgUntil) return;
  if (hdrMsgUntil) { lcdFill(150, 4, BAT_X - 158, 30, C_BG); hdrMsgUntil = 0; }
  RtcTime t;
  if (rtcOk && rtcRead(t)) {
    drawTextf(196, 4, C_WHITE, C_BG, 1, "%04u-%02u-%02u", t.year, t.mon, t.day);
    drawTextf(196, 18, C_WHITE, C_BG, 1, "%02u:%02u:%02u", t.hour, t.min, t.sec);
  } else drawText(196, 4, "RTC ?", C_RED);
}
static void section(int y, const char *title) {
  drawText(4, y, title, C_CYAN);
  const int w = LCD_W - 12 - (int)strlen(title) * FONT_W;
  if (w > 0) lcdFill(4 + (int)strlen(title) * FONT_W + 4, y + 6, w, 1, C_DGRAY);
}

// ------------------------------------------------------------------ SENS page
static const int BUB_X = 250, BUB_Y = 150, BUB_S = 66;
static void sensorStatic() {
  section(40, "SHTC3  temperature / humidity");
  section(88, "PCF85063A  real-time clock");
  section(134, "QMI8658  accel [g]  gyro [dps]");
  const char *ax[3] = {"X", "Y", "Z"}; const uint16_t axc[3] = {C_RED, C_GREEN, C_BLUE};
  for (int i = 0; i < 3; i++) drawText(4, 150 + i * 26, ax[i], axc[i], C_BG, 2);
  lcdRect(BUB_X, BUB_Y, BUB_S, BUB_S, C_GRAY);
  section(248, "AXP2101  PMIC");
  section(346, "MICROSD");
  section(384, "SYSTEM");
  drawText(4, 414, wakeMsg, C_GRAY);
}
static void sensorEnv() {
  float t, rh;
  if (shtOk && shtRead(t, rh)) {
    int x = drawText(4, 56, "T ", C_GRAY, C_BG, 2);
    x = drawTextf(x, 56, C_ORANGE, C_BG, 2, "%5.2f", t);
    x = drawText(x, 56, "C  RH ", C_GRAY, C_BG, 2);
    x = drawTextf(x, 56, C_CYAN, C_BG, 2, "%5.1f", rh);
    drawText(x, 56, "%", C_GRAY, C_BG, 2);
  } else drawText(4, 56, "SHTC3 not responding", C_RED, C_BG, 2);
  RtcTime r;
  if (rtcOk && rtcRead(r)) drawTextf(4, 104, C_WHITE, C_BG, 2, "%04u-%02u-%02u %02u:%02u:%02u", r.year, r.mon, r.day, r.hour, r.min, r.sec);
  else drawText(4, 104, "RTC not responding", C_RED, C_BG, 2);
}
static void sensorImu() {
  static int lbx = -1, lby = -1;
  ImuData d;
  if (!imuOk || !imuRead(d)) { drawText(28, 150, "QMI8658 not responding", C_RED); return; }
  const float a[3] = {d.ax, d.ay, d.az}, g[3] = {d.gx, d.gy, d.gz};
  for (int i = 0; i < 3; i++) {
    drawTextf(24, 150 + i * 26, C_WHITE, C_BG, 2, "%+6.3f", a[i]);
    drawTextf(124, 150 + i * 26, C_YELLOW, C_BG, 2, "%+7.1f", g[i]);
  }
  const float roll = atan2f(d.ay, d.az) * 57.29578f, pitch = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * 57.29578f;
  drawTextf(4, 230, C_GRAY, C_BG, 1, "Tdie %5.1f C  roll %+6.1f  pitch %+6.1f", d.t, roll, pitch);
  const int c = BUB_S / 2, rr = BUB_S / 2 - 6;
  const float fx = constrain(-d.ax / 0.5f, -1.0f, 1.0f), fy = constrain(d.ay / 0.5f, -1.0f, 1.0f);
  const int bx = BUB_X + c + (int)(fx * rr) - 4, by = BUB_Y + c + (int)(fy * rr) - 4;
  if (bx != lbx || by != lby) {
    if (lbx >= 0) lcdFill(lbx, lby, 9, 9, C_BG);
    lcdFill(BUB_X + c, BUB_Y + 2, 1, BUB_S - 4, C_DGRAY); lcdFill(BUB_X + 2, BUB_Y + c, BUB_S - 4, 1, C_DGRAY);
    lcdFill(bx, by, 9, 9, (fabsf(roll) < 2.0f && fabsf(pitch) < 2.0f) ? C_GREEN : C_ORANGE);
    lbx = bx; lby = by;
  }
}
static void sensorPmic() {
  PmicData p;
  if (!axpOk || !axpRead(p)) { drawText(4, 264, "AXP2101 not responding", C_RED); return; }
  static const char *dirTxt[4] = {"standby", "charging", "discharging", "?"};
  static const char *chgTxt[8] = {"trickle", "pre-charge", "CC", "CV", "done", "not charging", "?", "?"};
  drawTextf(4, 264, p.vbusGood ? C_GREEN : C_GRAY, C_BG, 1, "VBUS %5.3f V  %-8s", p.vbus / 1000.0f, p.vbusGood ? "present" : "absent");
  drawTextf(4, 280, C_WHITE, C_BG, 1, "VSYS %5.3f V   PMIC die %5.1f C ", p.vsys / 1000.0f, p.tdie);
  if (p.batPresent) {
    drawTextf(4, 296, C_WHITE, C_BG, 1, "VBAT %5.3f V   %3d %%            ", p.vbat / 1000.0f, p.pct);
    drawTextf(4, 312, C_YELLOW, C_BG, 1, "Battery: %s (%s)       ", dirTxt[p.dir & 3], chgTxt[p.chg & 7]);
    const int w = constrain(p.pct, 0, 100) * 300 / 100;
    const uint16_t col = p.pct > 50 ? C_GREEN : p.pct > 20 ? C_YELLOW : C_RED;
    lcdRect(9, 328, 302, 12, C_GRAY); lcdFill(10, 329, w, 10, col); lcdFill(10 + w, 329, 300 - w, 10, C_BLACK);
  } else {
    drawText(4, 296, "VBAT ---       no battery          ", C_GRAY);
    drawText(4, 312, "MX1.25 connector: 3.7 V Li-Po      ", C_GRAY);
    lcdFill(9, 328, 302, 12, C_BG);
  }
}
static void sensorSys() {
  if (sdOk) {
    static const char *ct[] = {"NONE", "MMC", "SDSC", "SDHC", "?"};
    const uint8_t t = SD.cardType();
    drawTextf(4, 362, C_GREEN, C_BG, 1, "%s %.1f GB  used %.1f MB     ", ct[t < 4 ? t : 4],
              SD.cardSize() / 1073741824.0, SD.usedBytes() / 1048576.0);
  } else drawText(4, 362, "no card / not mounted (FAT32)", C_GRAY);
  drawTextf(4, 398, C_GRAY, C_BG, 1, "Heap %lu  PSRAM %lu  up %lus   ", (unsigned long)ESP.getFreeHeap(),
            (unsigned long)ESP.getFreePsram(), (unsigned long)(millis() / 1000));
}

// ----------------------------------------------------------------- AUDIO page
static const int WAVE_X = 10, WAVE_Y = 226, WAVE_W = 300, WAVE_H = 96;
static void audioButtons() {
  const AudioMode m = audioMode;
  drawButton(bRec, m == AM_REC, C_RED);
  drawButton(bPlay, m == AM_PLAY && !txActive, C_GREEN);
  drawButton(bStop, false, C_GRAY);
  for (int i = 0; i < 4; i++) drawButton(bTone[i], i == toneSel && (toneFreq[i] == 0.0f || m == AM_TONE), toneFreq[i] > 0 ? C_ORANGE : C_GRAY);
}
static void drawWave() {
  lcdFill(WAVE_X, WAVE_Y, WAVE_W, WAVE_H, C_BLACK);
  lcdRect(WAVE_X - 1, WAVE_Y - 1, WAVE_W + 2, WAVE_H + 2, C_GRAY);
  lcdFill(WAVE_X, WAVE_Y + WAVE_H / 2, WAVE_W, 1, C_DGRAY);
  if (!waveValid || !recBuf || recLen == 0) return;
  const uint32_t per = recLen / WAVE_W;
  for (int c = 0; c < WAVE_W; c++) {
    int16_t mn = 32767, mx = -32768;
    for (uint32_t i = c * per; i < (uint32_t)(c + 1) * per; i++) { const int16_t v = recBuf[i]; if (v < mn) mn = v; if (v > mx) mx = v; }
    const int y0 = WAVE_Y + WAVE_H / 2 - mx * (WAVE_H / 2) / 32768, y1 = WAVE_Y + WAVE_H / 2 - mn * (WAVE_H / 2) / 32768;
    lcdFill(WAVE_X + c, y0, 1, max(1, y1 - y0 + 1), C_CYAN);
  }
}
static void audioStatic() {
  section(40, "MICROPHONE  ES8311  16 kHz 16-bit");
  lcdRect(9, 57, 302, 20, C_GRAY);
  section(104, "RECORDER  5 s in PSRAM");
  section(334, "TEST TONE  NS4150B");
  drawText(4, 404, codecOk ? "Codec OK" : "ES8311 codec not responding", codecOk ? C_GRAY : C_RED);
  if (!recBuf) drawText(4, 418, "PSRAM disabled: recorder unavailable", C_RED);
  audioButtons(); drawWave(); audioMsgDirty = true;
}
static int levelX(float db) { float f = (db + 72.0f) / 72.0f; f = constrain(f, 0.0f, 1.0f); return (int)(f * 300); }
static void audioMeter() {
  static float hold = -120.0f; static uint32_t ht = 0;
  const float rms = micRmsDb, pk = micPeakDb;
  if (pk > hold || millis() - ht > 1000) { hold = pk; ht = millis(); }
  const int xr = levelX(rms), x20 = levelX(-20.0f), x6 = levelX(-6.0f);
  lcdFill(10, 58, min(xr, x20), 18, C_GREEN);
  lcdFill(10 + x20, 58, constrain(xr, x20, x6) - x20, 18, C_YELLOW);
  lcdFill(10 + x6, 58, max(xr, x6) - x6, 18, C_RED);
  lcdFill(10 + xr, 58, 300 - xr, 18, C_BLACK);
  lcdFill(10 + min(levelX(hold), 298), 58, 2, 18, C_WHITE);
  drawTextf(4, 82, C_WHITE, C_BG, 1, "RMS %6.1f dBFS   peak %6.1f dBFS  ", rms, hold);
  const AudioMode m = audioMode;
  uint32_t pos = 0, len = REC_SAMPLES;
  if (m == AM_REC) pos = recPos; else if (m == AM_PLAY) { pos = playPos; len = playLen ? playLen : 1; }
  const int w = (int)((uint64_t)pos * 300 / len);
  lcdFill(10, 214, w, 6, m == AM_REC ? C_RED : C_GREEN); lcdFill(10 + w, 214, 300 - w, 6, C_DGRAY);
  if (m == AM_REC)  { snprintf(audioMsg, sizeof(audioMsg), "Recording %.1f / %d s", pos / (float)AUDIO_FS, REC_SECONDS); audioMsgDirty = true; }
  if (m == AM_PLAY) { snprintf(audioMsg, sizeof(audioMsg), "Playing %.1f s", pos / (float)AUDIO_FS); audioMsgDirty = true; }
  if (audioMsgDirty) { drawTextf(4, 194, C_YELLOW, C_BG, 1, "%-38s", audioMsg); audioMsgDirty = false; }
}
static void audioPostRecord() {   // remove DC, normalize to -1 dBFS (max gain x16), save WAV
  const uint32_t n = recLen;
  int64_t s = 0; for (uint32_t i = 0; i < n; i++) s += recBuf[i];
  const int32_t mean = (int32_t)(s / (int64_t)n);
  int32_t pk = 1; for (uint32_t i = 0; i < n; i++) { int32_t v = recBuf[i] - mean; if (v < 0) v = -v; if (v > pk) pk = v; }
  recPeakDbRaw = 20.0f * log10f(pk / 32768.0f);
  recGain = constrain(0.891f * 32767.0f / pk, 1.0f, 16.0f);
  for (uint32_t i = 0; i < n; i++) { float v = (recBuf[i] - mean) * recGain; recBuf[i] = (int16_t)constrain(v, -32768.0f, 32767.0f); }
  waveValid = true;
  if (page == PG_AUDIO) drawWave();
  const bool saved = saveWav("/electgpl_rec.wav", recBuf, n);
  snprintf(audioMsg, sizeof(audioMsg), "peak %.1f dBFS x%.1f %s", recPeakDbRaw, recGain, saved ? "SD:/electgpl_rec.wav" : "");
  audioMsgDirty = true;
  Serial.printf("Recording: %lu samples, peak %.1f dBFS, gain x%.2f, WAV %s\n", (unsigned long)n, recPeakDbRaw, recGain, saved ? "saved" : "not saved");
}

// ------------------------------------------------------------------- CAM page
static void camStatic() {
  section(40, "BF3901  240x320 RGB565  PARLIO 2-bit");
  lcdRect(CAM_X - 1, CAM_Y - 1, CAM_W + 2, CAM_H + 2, C_GRAY);
  if (!camInited) { drawText(4, 384, "Camera unavailable:", C_RED); drawText(4, 400, camErr, C_RED); }
}
static void camUpdate() {
  int pick = -1; uint32_t best = 0;
  taskENTER_CRITICAL(&camMux);
  for (int i = 0; i < CAM_NBUF; i++) if (camSt[i] == CB_READY && camSeq[i] >= best) { best = camSeq[i]; pick = i; }
  for (int i = 0; i < CAM_NBUF; i++) if (camSt[i] == CB_READY && i != pick) camSt[i] = CB_FREE;
  if (pick >= 0) camSt[pick] = CB_DRAW;
  taskEXIT_CRITICAL(&camMux);
  if (pick < 0) return;
  if (camDraw(camBuf[pick])) { camFramesOk = camFramesOk + 1; camFpsN++; } else camFramesBad = camFramesBad + 1;
  taskENTER_CRITICAL(&camMux); camSt[pick] = CB_FREE; taskEXIT_CRITICAL(&camMux);
  const uint32_t now = millis();
  if (now - camFpsT0 >= 1000) {
    const float fps = camFpsN * 1000.0f / (now - camFpsT0); camFpsN = 0; camFpsT0 = now;
    drawTextf(4, 384, C_WHITE, C_BG, 1, "%4.1f fps ok %lu err %lu drop %lu   ", fps,
              (unsigned long)camFramesOk, (unsigned long)camFramesBad, (unsigned long)camOverrun);
  }
}

// ----------------------------------------------------------------- AX.25 page
static const int TERM_X = 8, TERM_Y = 72;
static TermLine term[TERM_ROWS];
static int termCount = 0;
static void termDrawRow(int i) {
  char b[TERM_COLS + 1];
  snprintf(b, sizeof(b), "%-*s", TERM_COLS, i < termCount ? term[i].txt : "");
  drawText(TERM_X, TERM_Y + i * FONT_H, b, i < termCount ? term[i].color : C_WHITE, C_BLACK);
}
static void termRedraw() { for (int i = 0; i < TERM_ROWS; i++) termDrawRow(i); }
static void termPushLine(const char *s, uint16_t color) {
  bool scrolled = false;
  if (termCount == TERM_ROWS) { memmove(term, term + 1, sizeof(TermLine) * (TERM_ROWS - 1)); termCount--; scrolled = true; }
  strncpy(term[termCount].txt, s, TERM_COLS); term[termCount].txt[TERM_COLS] = 0;
  term[termCount].color = color;
  termCount++;
  if (page == PG_AX25) { if (scrolled) termRedraw(); else termDrawRow(termCount - 1); }
}
static void termWrite(const char *s, uint16_t color) {   // hard wrap at TERM_COLS, continuation indented
  char line[TERM_COLS + 1]; bool first = true;
  while (*s) {
    const int room = first ? TERM_COLS : TERM_COLS - 2;
    int n = (int)strnlen(s, room);
    snprintf(line, sizeof(line), "%s%.*s", first ? "" : "  ", n, s);
    termPushLine(line, color);
    s += n; first = false;
  }
}
static int ax25Call(const uint8_t *a, char *out) {           // "CALL-SSID" from a 7-byte address field
  int n = 0;
  for (int i = 0; i < 6; i++) { const char c = (char)(a[i] >> 1); if (c != ' ') out[n++] = c; }
  const uint8_t ssid = (a[6] >> 1) & 0x0F;
  if (ssid) n += sprintf(out + n, "-%u", ssid);
  out[n] = 0;
  return n;
}
static void ax25Display(const Ax25Frame &f) {
  int na = 0;
  while (na * 7 + 6 < f.len && na < 10) { if (f.data[na * 7 + 6] & 1) { na++; break; } na++; }
  if (na < 2 || na * 7 >= f.len) { termWrite("[malformed address field]", C_RED); return; }
  char hdr[160], c[12]; int n = 0;
  ax25Call(f.data + 7, c); n += sprintf(hdr + n, "%s>", c);                 // source
  ax25Call(f.data, c);     n += sprintf(hdr + n, "%s", c);                  // destination
  for (int i = 2; i < na; i++) {                                            // digipeaters, H-bit = '*'
    ax25Call(f.data + i * 7, c);
    n += sprintf(hdr + n, ",%s%s", c, (f.data[i * 7 + 6] & 0x80) ? "*" : "");
  }
  int p = na * 7;
  const uint8_t ctl = f.data[p++];
  const bool ui = (ctl & 0xEF) == 0x03;
  if (ui && p < f.len) p++;                                                 // skip PID
  else if (!ui) n += sprintf(hdr + n, " [ctl %02X]", ctl);
  sprintf(hdr + n, ":");
  RtcTime t; char ts[12] = "";
  if (rtcOk && rtcRead(t)) snprintf(ts, sizeof(ts), "%02u:%02u:%02u ", t.hour, t.min, t.sec);
  char line[200]; snprintf(line, sizeof(line), "%s%s", ts, hdr);
  termWrite(line, C_GREEN);
  char info[AX25_MAX_FRAME + 1]; int k = 0;
  for (; p < f.len; p++) { const uint8_t ch = f.data[p]; info[k++] = (ch >= 0x20 && ch < 0x7F) ? (char)ch : '.'; }
  info[k] = 0;
  if (k) termWrite(info, C_WHITE);
  Serial.printf("AX25 RX: %s%s\n", hdr, info);
}
static void ax25Static() {
  section(40, "AX.25  AFSK 1200 Bd  Bell 202");
  lcdRect(TERM_X - 3, TERM_Y - 3, TERM_COLS * FONT_W + 6, TERM_ROWS * FONT_H + 6, C_GRAY);
  termRedraw();
  drawButton(bSend, txActive, C_RED);
  drawButton(bClear, false, C_GRAY);
}
static void ax25Status() {
  const bool dcd = millis() - ax25LastFlagMs < 300;
  lcdFill(4, 55, 30, 13, dcd ? C_GREEN : C_DGRAY);
  drawText(7, 55, "DCD", dcd ? C_BLACK : C_GRAY, dcd ? C_GREEN : C_DGRAY);
  drawTextf(40, 55, C_WHITE, C_BG, 1, "%6.1f dBFS  ok %lu  crc %lu %s  ", (float)micRmsDb,
            (unsigned long)ax25Ok, (unsigned long)ax25Bad, txActive ? "TX" : "  ");
}
static void ax25Send() {
  if (txActive || !codecOk) return;
  char info[64];
  snprintf(info, sizeof(info), ">Electgpl ESP32-C5 AX.25 test #%lu", (unsigned long)++txSeq);
  if (!ax25BuildTx(info)) { termWrite("[TX buffer too small]", C_RED); return; }
  char line[80]; snprintf(line, sizeof(line), "TX %s-%u>%s:%s", cfg.call, cfg.ssid, AX25_DEST, info);
  termWrite(line, C_YELLOW);
  Serial.printf("AX25 TX: %lu samples (%.2f s)\n", (unsigned long)txCount, txCount / (float)AUDIO_FS);
  txActive = true;
  startPlayback(txBuf, txCount);
  drawButton(bSend, true, C_RED);
}

// ----------------------------------------------------------------- RADIO page
static ApInfo  aps[MAX_AP];
static BleInfo bles[MAX_BLE];
static int     apCount = 0, bleCount = 0;
static uint8_t radioMode = 0;                          // 0 = Wi-Fi list, 1 = BLE list
static bool    wifiScanning = false, bleInited = false;
static volatile bool bleScanning = false, bleDone = false;
static uint32_t scanT0 = 0, scanMs = 0;
static const int RL_Y = 72, RL_ROWS = 18, RL_H = 14;
static void foxDrawList();
static void stNetDraw();

static const char *authStr(uint8_t a) {
  switch ((wifi_auth_mode_t)a) {
    case WIFI_AUTH_OPEN: return "OPEN";            case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";          case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/2";   case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-E";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/3";
    case WIFI_AUTH_WAPI_PSK: return "WAPI";        case WIFI_AUTH_OWE: return "OWE";
    default: return "OTHER";
  }
}
static uint16_t rssiColor(int r) { return r > -60 ? C_GREEN : r > -75 ? C_YELLOW : C_RED; }
static void radioDrawList() {
  lcdFill(0, RL_Y - 16, LCD_W, 16 + RL_ROWS * RL_H, C_BG);
  if (radioMode == 0) {
    drawText(8, RL_Y - 15, "SSID              CH BD RSSI AUTH", C_GRAY);
    for (int i = 0; i < RL_ROWS && i < apCount; i++) {
      const ApInfo &a = aps[i]; const int y = RL_Y + i * RL_H;
      char ss[18];
      if (!a.ssid[0]) snprintf(ss, sizeof(ss), "<hidden>");
      else if (strlen(a.ssid) > 17) snprintf(ss, sizeof(ss), "%.16s~", a.ssid);
      else snprintf(ss, sizeof(ss), "%s", a.ssid);
      int x = drawTextf(8, y, a.ssid[0] ? C_WHITE : C_GRAY, C_BG, 1, "%-17s", ss);
      x = drawTextf(x, y, C_CYAN, C_BG, 1, " %3u", a.ch);
      x = drawText(x, y, a.ch > 14 ? " 5G" : " 2G", a.ch > 14 ? C_ORANGE : C_BLUE);
      x = drawTextf(x, y, rssiColor(a.rssi), C_BG, 1, " %4d", a.rssi);
      drawTextf(x, y, a.auth == WIFI_AUTH_OPEN ? C_RED : C_GRAY, C_BG, 1, " %-6s", authStr(a.auth));
    }
  } else {
    drawText(8, RL_Y - 15, "ADDRESS           NAME           RSSI", C_GRAY);
    for (int i = 0; i < RL_ROWS && i < bleCount; i++) {
      const BleInfo &b = bles[i]; const int y = RL_Y + i * RL_H;
      int x = drawTextf(8, y, C_CYAN, C_BG, 1, "%-17s ", b.addr);
      x = drawTextf(x, y, b.name[0] ? C_WHITE : C_GRAY, C_BG, 1, "%-14.14s ", b.name[0] ? b.name : "-");
      drawTextf(x, y, rssiColor(b.rssi), C_BG, 1, "%4d", b.rssi);
    }
  }
}
static void radioDrawSummary() {
  lcdFill(0, 326, LCD_W, 54, C_BG);
  if (radioMode == 0) {
    int n24 = 0, n5 = 0, perCh[14] = {0};
    for (int i = 0; i < apCount; i++) { if (aps[i].ch > 14) n5++; else { n24++; if (aps[i].ch >= 1 && aps[i].ch <= 13) perCh[aps[i].ch]++; } }
    drawTextf(4, 328, wifiScanning ? C_YELLOW : C_WHITE, C_BG, 1, wifiScanning ? "Scanning 2.4 + 5 GHz..." :
              "%d APs  2.4G %d  5G %d  %.1f s", apCount, n24, n5, scanMs / 1000.0f);
    int mx = 1; for (int c = 1; c <= 13; c++) mx = max(mx, perCh[c]);
    for (int c = 1; c <= 13; c++) {                         // 2.4 GHz channel occupancy
      const int h = perCh[c] * 26 / mx, x = 8 + (c - 1) * 23;
      lcdFill(x, 344, 19, 26 - h, C_DGRAY); if (h) lcdFill(x, 370 - h, 19, h, rssiColor(-50));
    }
    drawText(8, 371, "1", C_GRAY); drawText(8 + 12 * 23 + 2, 371, "13", C_GRAY);
  } else {
    drawTextf(4, 328, bleScanning ? C_YELLOW : C_WHITE, C_BG, 1, bleScanning ? "Scanning BLE advertisements..." :
              "%d BLE devices  %lu s active scan", bleCount, (unsigned long)BLE_SCAN_SECONDS);
    drawText(4, 344, "Sorted by RSSI, strongest first", C_GRAY);
  }
}
// --------------------------------------------- RADIO mode 2: IEEE 802.15.4
static bool ieeeOn = false, ieeeSweep = true, edBusy = false;
static volatile bool edPending = false;
static volatile int8_t edLast = -127;
static volatile uint8_t ieeeCurCh = 15;
static int8_t   edPow[16], edPeak[16];
static uint32_t ieeeCnt[16], ieeeTotal = 0, edT0 = 0, ieeeListenT0 = 0, ieeeSumT = 0;
static int      edIdx = 0, ieeeListen = 15, ieeeRowN = 0;
static QueueHandle_t ieeeQ = nullptr;
static char     ieeeRows[10][40];
static const int IE_X = 8, IE_PITCH = 19, IE_BW = 17, IE_Y = 60, IE_H = 100, IE_ROW_Y = 194;

// Driver callbacks (ISR context): copy the frame, release the driver buffer
void esp_ieee802154_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *info) {
  IeeeRx r;
  r.len = frame[0] > 127 ? 127 : frame[0];                 // PSDU length (FCS replaced by RSSI/LQI)
  memcpy(r.d, frame + 1, r.len);
  r.ch = ieeeCurCh; r.rssi = info->rssi; r.lqi = info->lqi;
  BaseType_t w = pdFALSE;
  if (ieeeQ) xQueueSendFromISR(ieeeQ, &r, &w);
  esp_ieee802154_receive_handle_done(frame);
  if (w) portYIELD_FROM_ISR();
}
void esp_ieee802154_energy_detect_done(int8_t power) { edLast = power; edPending = false; }

static void ieeeSetCh(uint8_t ch) { ieeeCurCh = ch; esp_ieee802154_set_channel(ch); }
static bool ieeeStart() {
  if (ieeeOn) return true;
  if (!ieeeQ) ieeeQ = xQueueCreate(16, sizeof(IeeeRx));
  if (WiFi.getMode() != WIFI_OFF) WiFi.mode(WIFI_OFF);   // free the 2.4 GHz radio
  if (esp_ieee802154_enable() != ESP_OK) return false;
  esp_ieee802154_set_promiscuous(true);
  esp_ieee802154_set_rx_when_idle(true);
  ieeeSetCh((uint8_t)ieeeListen);
  esp_ieee802154_receive();
  for (int i = 0; i < 16; i++) { edPow[i] = -100; edPeak[i] = -100; }
  ieeeOn = true; ieeeSweep = true; edIdx = 0; edBusy = false;
  return true;
}
static void ieeeStop() { if (!ieeeOn) return; esp_ieee802154_disable(); ieeeOn = false; edBusy = false; edPending = false; }
static void ieeeFmt(const IeeeRx &r, char *o, size_t n) {         // IEEE 802.15.4-2006 MAC header decode
  static const char *TYP[8] = {"BCN", "DATA", "ACK", "CMD", "RSV", "MP", "FRAG", "EXT"};
  const int lim = (int)r.len - 2;                                  // exclude FCS (RSSI/LQI)
  if (lim < 3) { snprintf(o, n, "%2u short frame            %4d", r.ch, r.rssi); return; }
  const uint16_t fcf = (uint16_t)(r.d[0] | (r.d[1] << 8));
  const uint8_t type = fcf & 7, dam = (fcf >> 10) & 3, sam = (fcf >> 14) & 3;
  const bool pidc = fcf & 0x40;
  const uint8_t seq = r.d[2];
  int p = 3; uint16_t pan = 0xFFFF; char da[8] = "-", sa[8] = "-";
  auto addr = [&](uint8_t mode, char *out) {
    if (mode == 2 && p + 2 <= lim) { snprintf(out, 8, "%04X", r.d[p] | (r.d[p + 1] << 8)); p += 2; }
    else if (mode == 3 && p + 8 <= lim) { snprintf(out, 8, "~%02X%02X", r.d[p + 1], r.d[p]); p += 8; }
  };
  if (dam) { if (p + 2 <= lim) { pan = (uint16_t)(r.d[p] | (r.d[p + 1] << 8)); p += 2; } addr(dam, da); }
  if (sam) { if (!pidc && p + 2 <= lim) { if (!dam) pan = (uint16_t)(r.d[p] | (r.d[p + 1] << 8)); p += 2; } addr(sam, sa); }
  snprintf(o, n, "%2u %-4s %3u %04X %5s>%-5s %4d", r.ch, TYP[type], seq, pan, da, sa, r.rssi);
}
static void ieeeDrawBars() {
  for (int i = 0; i < 16; i++) {
    const int x = IE_X + i * IE_PITCH;
    const int h = constrain(((int)edPow[i] + 100) * IE_H / 80, 0, IE_H), hp = constrain(((int)edPeak[i] + 100) * IE_H / 80, 0, IE_H);
    const uint16_t col = edPow[i] < -85 ? C_GREEN : edPow[i] < -70 ? C_YELLOW : C_RED;
    lcdFill(x, IE_Y, IE_BW, IE_H - h, C_BLACK);
    lcdFill(x, IE_Y + IE_H - h, IE_BW, h, col);
    if (hp > h) lcdFill(x, IE_Y + IE_H - hp, IE_BW, 2, C_WHITE);
    const bool sel = (11 + i) == ieeeListen;
    drawTextf(x + 1, IE_Y + IE_H + 3, sel ? C_BLACK : C_GRAY, sel ? C_YELLOW : C_BG, 1, "%02d", 11 + i);
  }
}
static void ieeeDrawRows() {
  for (int i = 0; i < 10; i++) drawTextf(8, IE_ROW_Y + i * FONT_H, i < ieeeRowN ? C_WHITE : C_GRAY, C_BLACK, 1, "%-38.38s", i < ieeeRowN ? ieeeRows[i] : "");
}
static void ieeeSummary() {
  drawTextf(4, 328, C_WHITE, C_BG, 1, "Listen ch %2d  frames %5lu  on ch %5lu  ", ieeeListen, (unsigned long)ieeeTotal, (unsigned long)ieeeCnt[ieeeListen - 11]);
  drawText(4, 344, "ED sweep 11-26 (4 ms/ch) + promiscuous RX", C_GRAY);
  drawText(4, 360, "Tap a bar to listen on that channel", C_GRAY);
}
static void ieeeStatic() {
  section(40, "IEEE 802.15.4  2.4 GHz  ch 11-26");
  if (!ieeeStart()) { drawText(8, 80, "esp_ieee802154_enable() failed", C_RED); return; }
  lcdRect(IE_X - 2, IE_Y - 2, 16 * IE_PITCH + 2, IE_H + 4, C_DGRAY);
  ieeeDrawBars();
  drawText(8, IE_ROW_Y - 15, "CH TYPE SEQ PAN    DST>SRC   RSSI", C_GRAY);
  ieeeDrawRows(); ieeeSummary();
}
static void ieeeTouch(uint16_t x, uint16_t y) {
  if (y < IE_Y || y > IE_Y + IE_H + 16 || x < IE_X || x >= IE_X + 16 * IE_PITCH) return;
  ieeeListen = 11 + (x - IE_X) / IE_PITCH;
  if (!ieeeSweep && !edBusy) ieeeSetCh((uint8_t)ieeeListen);
  ieeeDrawBars(); ieeeSummary();
}
static void ieeeUpdate(uint32_t now) {
  if (!ieeeOn) return;
  if (ieeeSweep) {                                                 // energy-detect sweep, one channel at a time
    if (edBusy && (!edPending || now - edT0 > 50)) {
      const int8_t p = edPending ? -100 : edLast;
      edPow[edIdx] = p; if (p > edPeak[edIdx]) edPeak[edIdx] = p;
      edIdx++; edBusy = false; edPending = false;
    }
    if (!edBusy) {
      if (edIdx >= 16) {                                           // sweep done: listen for 500 ms
        edIdx = 0; ieeeSweep = false; ieeeListenT0 = now;
        ieeeSetCh((uint8_t)ieeeListen); esp_ieee802154_receive();
        ieeeDrawBars();
      } else {
        ieeeSetCh((uint8_t)(11 + edIdx));
        edPending = true; edBusy = true; edT0 = now;
        if (esp_ieee802154_energy_detect(256) != ESP_OK) { edPending = false; }   // 256 symbols x 16 us
      }
    }
  } else if (now - ieeeListenT0 >= 500) ieeeSweep = true;
  IeeeRx r; bool newRows = false;
  while (ieeeQ && xQueueReceive(ieeeQ, &r, 0) == pdTRUE) {
    if (r.ch >= 11 && r.ch <= 26) ieeeCnt[r.ch - 11]++;
    ieeeTotal++;
    if (ieeeRowN == 10) { memmove(ieeeRows[0], ieeeRows[1], sizeof(ieeeRows[0]) * 9); ieeeRowN = 9; }
    ieeeFmt(r, ieeeRows[ieeeRowN++], sizeof(ieeeRows[0]));
    Serial.printf("15.4 RX ch %u len %u rssi %d lqi %u\n", r.ch, r.len, r.rssi, r.lqi);
    newRows = true;
  }
  if (newRows) ieeeDrawRows();
  if (newRows || now - ieeeSumT > 1000) { ieeeSumT = now; ieeeSummary(); }
}

// --------------------------------------------- RADIO mode 3: ESP-NOW
// SNIFF: promiscuous capture of Espressif vendor action frames (category 127, OUI 18:FE:34, type 4 = ESP-NOW),
//        sees broadcast AND third-party unicast traffic; protected (CCMP) action frames are listed as encrypted.
// RANGE: symmetric link test between two boards (broadcast "EGPL" packets every 100 ms, per-peer RSSI, loss,
//        and the RSSI at which the peer hears us, carried inside its own packets).
static const int EN_MAX = 20, EN_PEERS = 4, EN_HIST = 300, EN_CW = 300, EN_CH = 110, EN_CY = 220;
static const uint8_t EN_BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static EnDev    enDev[EN_MAX];
static EnPeer   enPeer[EN_PEERS];
static int      enDevN = 0, enPeerN = 0, enSel = 0;
static portMUX_TYPE enMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t  enMode = 0;                                        // 0 sniff, 1 range
static bool     enOn = false, enHop = true, enNowInit = false;
static int      enCh = 1;
static uint32_t enHopT = 0, enUiT = 0, enTxT = 0, enSeq = 0, enTxOk = 0, enStatT = 0, enFrames = 0;
static uint16_t *enImg = nullptr;
static char     enLabM[8], enLabA[6];
static Button enBtn(int i) {
  static const int16_t X[4] = {4, 110, 162, 262}, W[4] = {100, 48, 48, 54};
  const char *L[4] = {enLabM, "CH-", "CH+", enLabA};
  Button b = {X[i], 56, W[i], 28, L[i]}; return b;
}
static void enSniffRx(void *buf, wifi_promiscuous_pkt_type_t type) {   // Wi-Fi task context
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *f = p->payload; const int L = p->rx_ctrl.sig_len;
  if (L < 28 || (f[0] & 0xFC) != 0xD0) return;                    // management / action
  const bool prot = f[1] & 0x40;
  uint8_t dl = 0;
  if (!prot) {
    if (L < 40 || f[24] != 127 || f[25] != 0x18 || f[26] != 0xFE || f[27] != 0x34) return;   // vendor-specific, Espressif
    if (f[32] != 0xDD || f[34] != 0x18 || f[35] != 0xFE || f[36] != 0x34 || f[37] != 4) return;   // ESP-NOW element
    dl = f[33] >= 5 ? (uint8_t)(f[33] - 5) : 0;
  }
  const bool bc = f[4] == 0xFF && f[5] == 0xFF && f[6] == 0xFF;
  taskENTER_CRITICAL(&enMux);
  int i = 0; while (i < enDevN && memcmp(enDev[i].mac, f + 10, 6)) i++;
  if (i == enDevN && enDevN < EN_MAX) { memcpy(enDev[i].mac, f + 10, 6); enDev[i].n = 0; enDevN++; }
  if (i < enDevN) { EnDev &d = enDev[i]; d.rssi = (int8_t)p->rx_ctrl.rssi; d.ch = (uint8_t)p->rx_ctrl.channel; d.len = dl; d.bc = bc; d.enc = prot; d.n++; d.t = millis(); }
  enFrames++;
  taskEXIT_CRITICAL(&enMux);
}
static void enRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {   // Wi-Fi task context
  if (len < (int)sizeof(EnPkt) || memcmp(data, "EGPL", 4)) return;
  EnPkt pk; memcpy(&pk, data, sizeof(pk));
  const int8_t rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : -127;
  taskENTER_CRITICAL(&enMux);
  int i = 0; while (i < enPeerN && memcmp(enPeer[i].mac, info->src_addr, 6)) i++;
  if (i == enPeerN && enPeerN < EN_PEERS) {
    EnPeer &n = enPeer[i]; memset(&n, 0, sizeof(n)); memcpy(n.mac, info->src_addr, 6); n.first = n.last = pk.seq; n.seqW = pk.seq - 1; n.ema = rssi;
    n.mn = 0; n.mx = -127; for (int k = 0; k < EN_HIST; k++) n.hist[k] = -128; enPeerN++;
  }
  if (i < enPeerN) {
    EnPeer &e = enPeer[i];
    if (pk.seq < e.last && e.last - pk.seq > 1000) { e.first = pk.seq; e.recv = 0; e.recvW = 0; e.seqW = pk.seq; }   // peer rebooted
    e.last = max(e.last, pk.seq); e.recv++; e.rssi = rssi; e.remote = pk.heard; e.t = millis();
    memcpy(e.call, pk.call, 7); e.call[7] = 0;
  }
  taskEXIT_CRITICAL(&enMux);
}
static void enStop() {
  if (!enOn) return;
  esp_wifi_set_promiscuous(false);
  if (enNowInit) { esp_now_unregister_recv_cb(); esp_now_deinit(); enNowInit = false; }
  enOn = false;
}
static bool enStart() {
  enStop();
  if (ieeeOn) ieeeStop();
  if (WiFi.getMode() == WIFI_OFF) WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel((uint8_t)enCh, WIFI_SECOND_CHAN_NONE);
  if (enMode == 0) {
    wifi_promiscuous_filter_t flt = {}; flt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
    esp_wifi_set_promiscuous_filter(&flt); esp_wifi_set_promiscuous_rx_cb(enSniffRx); esp_wifi_set_promiscuous(true);
  } else {
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(enRecv);
    esp_now_peer_info_t pi = {}; memcpy(pi.peer_addr, EN_BCAST, 6); pi.channel = 0; pi.ifidx = WIFI_IF_STA; pi.encrypt = false;
    esp_now_add_peer(&pi);
    enNowInit = true;
  }
  enOn = true;
  return true;
}
static void enCtrl() {
  snprintf(enLabM, sizeof(enLabM), enMode ? "RANGE" : "SNIFF");
  snprintf(enLabA, sizeof(enLabA), enMode ? "RST" : (enHop ? "HOP" : "LOCK"));
  drawButton(enBtn(0), true, C_YELLOW, 8, 13); drawButton(enBtn(1), false, C_CYAN, 8, 13); drawButton(enBtn(2), false, C_CYAN, 8, 13);
  drawButton(enBtn(3), enMode == 0 && enHop, C_CYAN, 8, 13);
  drawTextf(212, 64, C_WHITE, C_BG, 1, "%2d", enCh);
}
static void enSniffDraw() {
  int ord[EN_MAX], n;
  EnDev snap[EN_MAX];
  taskENTER_CRITICAL(&enMux); n = enDevN; memcpy(snap, enDev, sizeof(EnDev) * n); taskEXIT_CRITICAL(&enMux);
  for (int i = 0; i < n; i++) ord[i] = i;
  std::sort(ord, ord + n, [&](int a, int b) { return snap[a].t > snap[b].t; });
  drawText(8, 92, "SRC MAC      CH RSSI DST   PKTS  LEN", C_GRAY);
  const uint32_t now = millis();
  for (int r = 0; r < 18; r++) {
    const int y = 106 + r * 14;
    if (r >= n) { drawTextf(8, y, C_GRAY, C_BG, 1, "%-38s", r == 0 ? "No ESP-NOW frames yet..." : ""); continue; }
    const EnDev &d = snap[ord[r]];
    const bool live = now - d.t < 3000;
    drawTextf(8, y, live ? C_WHITE : C_GRAY, C_BG, 1, "%02X%02X%02X%02X%02X%02X %2u %4d %s%s %5lu %4u  ", d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5],
              d.ch, d.rssi, d.bc ? "BC" : "UC", d.enc ? "*" : " ", (unsigned long)d.n, d.len);
  }
  drawTextf(4, 362, C_GRAY, C_BG, 1, "ch %2d %s  frames %lu  * = encrypted   ", enCh, enHop ? "hopping" : "locked ", (unsigned long)enFrames);
}
static void enRangeRender(const EnPeer &p) {
  const uint16_t bg = 0, grid = be16(0x2104);
  for (int i = 0; i < EN_CW * EN_CH; i++) enImg[i] = bg;
  for (int db = -30; db >= -90; db -= 20) { const int y = (-20 - db) * EN_CH / 80; for (int x = 0; x < EN_CW; x++) enImg[y * EN_CW + x] = grid; }
  int py = -1;
  for (int i = 0; i < EN_CW; i++) {
    const int8_t v = p.hist[(p.head + i) % EN_HIST];
    if (v == -128) { py = -1; continue; }
    const int y = constrain((-20 - v) * (EN_CH - 1) / 80, 0, EN_CH - 1), y0 = py < 0 ? y : min(py, y), y1 = py < 0 ? y : max(py, y);
    const uint16_t c = be16(rssiColor(v));
    for (int yy = y0; yy <= y1; yy++) enImg[yy * EN_CW + i] = c;
    py = y;
  }
  lcdBlit(10, EN_CY, EN_CW, EN_CH, (const uint8_t *)enImg);
  for (int db = -30; db >= -90; db -= 20) drawTextf(12, EN_CY + (-20 - db) * EN_CH / 80 - 13 + (db == -30 ? 13 : 0), C_GRAY, C_BLACK, 1, "%d", db);
}
static void enRangeDraw() {
  EnPeer snap[EN_PEERS]; int n;
  taskENTER_CRITICAL(&enMux); n = enPeerN; memcpy(snap, enPeer, sizeof(EnPeer) * n); taskEXIT_CRITICAL(&enMux);
  const uint32_t now = millis();
  if (!n) {
    drawTextf(8, 96, C_YELLOW, C_BG, 1, "%-38s", "Waiting for another Electgpl board");
    drawTextf(8, 112, C_GRAY, C_BG, 1, "%-38s", "running RADIO > ESPNOW > RANGE on the");
    drawTextf(8, 128, C_GRAY, C_BG, 1, "same channel %-25d", enCh);
  } else {
    const EnPeer &p = snap[min(enSel, n - 1)];
    const bool live = now - p.t < 1500;
    const int v = (int)lrintf(p.ema);
    if (live) drawTextf(8, 92, rssiColor(v), C_BG, 3, "%4d", v); else drawText(8, 92, " ---", C_GRAY, C_BG, 3);
    drawText(108, 104, "dBm", C_GRAY, C_BG, 2);
    drawTextf(172, 92, live ? C_WHITE : C_RED, C_BG, 1, "loss %5.1f %%  (2 s)  ", p.per);
    drawTextf(172, 106, C_WHITE, C_BG, 1, "rx %6lu  tx %6lu  ", (unsigned long)p.recv, (unsigned long)enTxOk);
    drawTextf(172, 120, C_CYAN, C_BG, 1, "peer hears me %4d dBm", p.remote);
    for (int i = 0; i < EN_PEERS; i++) {
      const int y = 140 + i * 14;
      if (i >= n) { drawTextf(8, y, C_GRAY, C_BG, 1, "%-38s", ""); continue; }
      const EnPeer &q = snap[i];
      drawTextf(8, y, i == enSel ? C_YELLOW : C_WHITE, C_BG, 1, "%c%02X%02X%02X %-7s %4d %4d %5.1f%% %6lu", i == enSel ? '>' : ' ', q.mac[3], q.mac[4], q.mac[5],
                q.call, q.rssi, q.remote, q.per, (unsigned long)q.recv);
    }
    if (enImg) enRangeRender(p);
    drawTextf(4, 334, C_GRAY, C_BG, 1, "min %4d  max %4d  dBm, 30 s history      ", p.mn, p.mx);
  }
  drawTextf(4, 350, C_GRAY, C_BG, 1, "ch %2d  10 pkt/s broadcast  tap peer row   ", enCh);
}
static void enStatic() {
  section(40, "ESP-NOW  2.4 GHz");
  if (!enImg && psramFound()) enImg = (uint16_t *)heap_caps_malloc(EN_CW * EN_CH * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  enCtrl();
  if (!enStart()) { drawText(8, 100, "ESP-NOW start failed", C_RED); return; }
  if (enMode == 1) { lcdRect(9, EN_CY - 1, EN_CW + 2, EN_CH + 2, C_GRAY); enRangeDraw(); } else enSniffDraw();
}
static void enUpdate(uint32_t now) {
  if (!enOn) return;
  if (enMode == 0 && enHop && now - enHopT >= 250) { enHopT = now; enCh = enCh % 13 + 1; esp_wifi_set_channel((uint8_t)enCh, WIFI_SECOND_CHAN_NONE); }
  if (enMode == 1 && now - enTxT >= 100) {                        // transmit our numbered packet
    enTxT = now;
    EnPkt pk = {}; memcpy(pk.magic, "EGPL", 4); pk.ver = 1; pk.seq = ++enSeq; snprintf(pk.call, sizeof(pk.call), "%s", cfg.call);
    taskENTER_CRITICAL(&enMux); pk.heard = enPeerN ? enPeer[min(enSel, enPeerN - 1)].rssi : -127; taskEXIT_CRITICAL(&enMux);
    if (esp_now_send(EN_BCAST, (const uint8_t *)&pk, sizeof(pk)) == ESP_OK) enTxOk++;
  }
  if (enMode == 1 && now - enStatT >= 100) {                      // per-peer statistics, 10 samples/s
    enStatT = now;
    static uint32_t perT = 0; const bool win = now - perT >= 2000; if (win) perT = now;
    taskENTER_CRITICAL(&enMux);
    for (int i = 0; i < enPeerN; i++) {
      EnPeer &p = enPeer[i]; const bool live = now - p.t < 1500;
      if (live) { p.ema = 0.7f * p.ema + 0.3f * p.rssi; p.mn = p.mn ? min(p.mn, (int)p.rssi) : p.rssi; p.mx = max(p.mx, (int)p.rssi); }
      p.hist[p.head] = live ? (int8_t)lrintf(p.ema) : (int8_t)-128; p.head = (p.head + 1) % EN_HIST;
      if (win) { const uint32_t exp = p.last - p.seqW, got = p.recv - p.recvW; p.per = exp ? constrain(100.0f * (1.0f - (float)got / exp), 0.0f, 100.0f) : (live ? 0 : 100);
                 p.seqW = p.last; p.recvW = p.recv; }
    }
    taskEXIT_CRITICAL(&enMux);
  }
  if (now - enUiT >= (enMode ? 250u : 500u)) { enUiT = now; if (enMode) enRangeDraw(); else enSniffDraw(); }
}
static void enTouch(uint16_t x, uint16_t y) {
  if (hit(enBtn(0), x, y)) { enMode ^= 1; lcdFill(0, 88, LCD_W, 290, C_BG); enStatic(); return; }
  if (hit(enBtn(1), x, y) || hit(enBtn(2), x, y)) {
    enCh = (enCh + (hit(enBtn(2), x, y) ? 1 : 12) - 1) % 13 + 1; enHop = false;
    esp_wifi_set_channel((uint8_t)enCh, WIFI_SECOND_CHAN_NONE); enCtrl(); return;
  }
  if (hit(enBtn(3), x, y)) {
    if (enMode == 0) enHop = !enHop;
    else { taskENTER_CRITICAL(&enMux); enPeerN = 0; taskEXIT_CRITICAL(&enMux); enTxOk = 0; lcdFill(0, 88, LCD_W, 290, C_BG); lcdRect(9, EN_CY - 1, EN_CW + 2, EN_CH + 2, C_GRAY); }
    enCtrl(); return;
  }
  if (enMode == 1 && y >= 140 && y < 140 + EN_PEERS * 14) { const int i = (y - 140) / 14; if (i < enPeerN) enSel = i; }
}

static void radioStatic() {
  lcdFill(0, 37, LCD_W, 343, C_BG);
  if (radioMode == 3) enStatic();
  else if (radioMode == 2) ieeeStatic();
  else { section(40, radioMode == 0 ? "WI-FI 6  2.4 / 5 GHz  scan" : "BLUETOOTH LE  advertisement scan"); radioDrawList(); radioDrawSummary(); }
  drawButton(bWifi, radioMode == 0, C_CYAN, 12, 20);
  drawButton(bBle, radioMode == 1, C_CYAN, 12, 20);
  drawButton(bIeee, radioMode == 2, C_CYAN, 12, 20);
  drawButton(bEspNow, radioMode == 3, C_CYAN, 12, 20);
}
static void wifiStartScan() {
  if (wifiScanning || bleScanning) return;
  if (ieeeOn) ieeeStop();
  if (enOn) enStop();
  WiFi.mode(WIFI_STA);
  WiFi.setBandMode(WIFI_BAND_MODE_AUTO);                    // ESP32-C5: scan both bands
  WiFi.disconnect();
  if (WiFi.scanNetworks(true, true) == WIFI_SCAN_FAILED) return;   // async, include hidden SSIDs
  wifiScanning = true; scanT0 = millis();
}
static void wifiPoll() {
  if (!wifiScanning) return;
  const int16_t n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;
  wifiScanning = false; scanMs = millis() - scanT0;
  apCount = 0;
  for (int i = 0; i < n && apCount < MAX_AP; i++) {
    ApInfo &a = aps[apCount++];
    snprintf(a.ssid, sizeof(a.ssid), "%s", WiFi.SSID(i).c_str());
    for (char *c = a.ssid; *c; c++) if (*c < 0x20 || *c > 0x7E) *c = '?';
    a.rssi = (int8_t)WiFi.RSSI(i); a.ch = (uint8_t)WiFi.channel(i); a.auth = (uint8_t)WiFi.encryptionType(i);
    { const uint8_t *b = WiFi.BSSID(i); if (b) memcpy(a.bssid, b, 6); else memset(a.bssid, 0, 6); }
  }
  WiFi.scanDelete();
  std::sort(aps, aps + apCount, [](const ApInfo &l, const ApInfo &r) { return l.rssi > r.rssi; });
  Serial.printf("Wi-Fi scan: %d APs in %lu ms\n", apCount, (unsigned long)scanMs);
  for (int i = 0; i < apCount; i++) Serial.printf("  %-32s ch %3u %4d dBm %s\n", aps[i].ssid, aps[i].ch, aps[i].rssi, authStr(aps[i].auth));
  if (page == PG_RADIO && radioMode == 0) { radioDrawList(); radioDrawSummary(); }
  if (page == PG_FOX && !foxTrack) foxDrawList();
  if (page == PG_SET && stView == SV_NET) stNetDraw();
}
static void bleScanDone(BLEScanResults res) {               // runs in the BLE host task
  int n = 0;
  for (int i = 0; i < res.getCount() && n < MAX_BLE; i++) {
    BLEAdvertisedDevice d = res.getDevice(i);
    BleInfo &b = bles[n++];
    snprintf(b.addr, sizeof(b.addr), "%s", d.getAddress().toString().c_str());
    snprintf(b.name, sizeof(b.name), "%s", d.haveName() ? d.getName().c_str() : "");
    for (char *c = b.name; *c; c++) if (*c < 0x20 || *c > 0x7E) *c = '?';
    b.rssi = (int8_t)d.getRSSI();
  }
  bleCount = n;
  bleScanning = false; bleDone = true;
}
static void bleStartScan() {
  if (wifiScanning || bleScanning) return;
  if (!bleInited) { BLEDevice::init("Electgpl-C5"); bleInited = true; }
  BLEScan *sc = BLEDevice::getScan();
  sc->setActiveScan(true); sc->setInterval(100); sc->setWindow(90);
  sc->clearResults();
  bleScanning = true; bleDone = false;
  if (!sc->start(BLE_SCAN_SECONDS, bleScanDone, false)) bleScanning = false;
}
static void blePoll() {
  if (!bleDone) return;
  bleDone = false;
  std::sort(bles, bles + bleCount, [](const BleInfo &l, const BleInfo &r) { return l.rssi > r.rssi; });
  BLEDevice::getScan()->clearResults();
  Serial.printf("BLE scan: %d devices\n", bleCount);
  for (int i = 0; i < bleCount; i++) Serial.printf("  %s %-24s %4d dBm\n", bles[i].addr, bles[i].name, bles[i].rssi);
  if (page == PG_RADIO && radioMode == 1) { radioDrawList(); radioDrawSummary(); }
}

// ======================== FOX HUNT (Wi-Fi RSSI tracker) =======================
static const int FOX_LIST_Y = 60, FOX_ROW_H = 26, FOX_ROWS = 12;
static const int FOX_CX = 10, FOX_CY = 146, FOX_CW = 300, FOX_CH = 156;
static uint16_t *foxImg = nullptr;
static uint8_t  foxBssid[6], foxChan = 1;
static char     foxSsid[33] = "";
static portMUX_TYPE foxMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t  foxSum = 0;
static volatile uint32_t foxCnt = 0;
static int8_t   foxHist[FOX_CW];
static int      foxHead = 0, foxMin = 0, foxMax = -127;
static float    foxEma = -100.0f;
static bool     foxHave = false, foxSound = false;
static double   foxAcc = 0;
static uint32_t foxAccN = 0, foxPps = 0, foxPktSec = 0, foxSecT = 0, foxSampT = 0, foxDrawT = 0, foxLastPkt = 0;
static const Button bFoxScan = {4, 410, 312, 60, "SCAN"};
static const Button bFoxSnd = {4, 410, 100, 60, "SOUND"}, bFoxRst = {110, 410, 100, 60, "RESET"}, bFoxList = {216, 410, 100, 60, "LIST"};

static void foxRx(void *buf, wifi_promiscuous_pkt_type_t type) {   // Wi-Fi task context
  (void)type;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  if (p->rx_ctrl.sig_len < 24) return;
  if (memcmp(p->payload + 10, foxBssid, 6) != 0) return;          // addr2 = transmitter address
  taskENTER_CRITICAL(&foxMux);
  foxSum = foxSum + p->rx_ctrl.rssi; foxCnt = foxCnt + 1;
  taskEXIT_CRITICAL(&foxMux);
}
static void foxDrawList() {
  lcdFill(0, 37, LCD_W, 372, C_BG);
  section(40, "SELECT ACCESS POINT  (tap to track)");
  if (wifiScanning) { drawText(8, 64, "Scanning 2.4 + 5 GHz...", C_YELLOW); }
  else if (apCount == 0) { drawText(8, 64, "No networks yet: press SCAN", C_GRAY); }
  for (int i = 0; i < FOX_ROWS && i < apCount && !wifiScanning; i++) {
    const ApInfo &a = aps[i]; const int y = FOX_LIST_Y + i * FOX_ROW_H; const uint16_t bg = (i & 1) ? C_BG : C_DGRAY;
    lcdFill(4, y, 312, FOX_ROW_H - 2, bg);
    drawTextf(10, y + 6, a.ssid[0] ? C_WHITE : C_GRAY, bg, 1, "%-20.20s", a.ssid[0] ? a.ssid : "<hidden>");
    drawTextf(178, y + 6, a.ch > 14 ? C_ORANGE : C_CYAN, bg, 1, "%s %3u", a.ch > 14 ? "5G" : "2G", a.ch);
    drawTextf(258, y + 6, rssiColor(a.rssi), bg, 1, "%4d dB", a.rssi);
  }
  drawButton(bFoxScan, wifiScanning, C_YELLOW);
}
static const char *foxQuality(float r) {
  return r >= -50 ? "Excellent  " : r >= -60 ? "Very good  " : r >= -67 ? "Good       " : r >= -75 ? "Fair       " : r >= -85 ? "Weak       " : "Very weak  ";
}
static void foxRender() {                                          // RSSI history chart into PSRAM, one blit
  const uint16_t bg = 0, grid = be16(0x2104), ref = be16(0x0320);
  for (int i = 0; i < FOX_CW * FOX_CH; i++) foxImg[i] = bg;
  for (int db = -40; db >= -90; db -= 10) { const int y = (-30 - db) * FOX_CH / 70; for (int x = 0; x < FOX_CW; x++) foxImg[y * FOX_CW + x] = (db == -70) ? ref : grid; }
  int py = -1;
  for (int i = 0; i < FOX_CW; i++) {
    const int8_t v = foxHist[(foxHead + i) % FOX_CW];
    if (v == -128) { py = -1; continue; }
    const int y = constrain((-30 - v) * (FOX_CH - 1) / 70, 0, FOX_CH - 1);
    const uint16_t c = be16(rssiColor(v));
    const int y0 = py < 0 ? y : min(py, y), y1 = py < 0 ? y : max(py, y);
    for (int yy = y0; yy <= y1; yy++) foxImg[yy * FOX_CW + i] = c;
    py = y;
  }
  lcdBlit(FOX_CX, FOX_CY, FOX_CW, FOX_CH, (const uint8_t *)foxImg);
  for (int db = -40; db >= -90; db -= 20) drawTextf(FOX_CX + 2, FOX_CY + (-30 - db) * FOX_CH / 70 - 13, C_GRAY, C_BLACK, 1, "%d", db);
}
static void foxDrawTrack(uint32_t now) {
  const bool live = foxHave && now - foxLastPkt < 2000;
  const int v = (int)lrintf(foxEma);
  if (live) {
    drawTextf(8, 72, rssiColor(v), C_BG, 3, "%4d", v);
    drawText(108, 84, "dBm", C_GRAY, C_BG, 2);
    drawText(172, 74, foxQuality(foxEma), rssiColor(v), C_BG, 2);
  } else { drawText(8, 72, " ---", C_GRAY, C_BG, 3); drawText(172, 74, "No signal  ", C_RED, C_BG, 2); }
  drawTextf(172, 100, C_GRAY, C_BG, 1, "%3lu pkt/s       ", (unsigned long)foxPps);
  const int w = live ? constrain((v + 100) * 300 / 70, 0, 300) : 0;
  lcdFill(10, 122, w, 14, live ? rssiColor(v) : C_BLACK); lcdFill(10 + w, 122, 300 - w, 14, C_BLACK);
  if (foxImg) foxRender();
  if (foxAccN) drawTextf(4, 308, C_WHITE, C_BG, 1, "min %4d  avg %6.1f  max %4d  dBm    ", foxMin, foxAcc / foxAccN, foxMax);
}
static void foxResetStats() {
  foxMin = 0; foxMax = -127; foxAcc = 0; foxAccN = 0; foxHave = false; foxEma = -100;
  for (int i = 0; i < FOX_CW; i++) foxHist[i] = -128;
  foxHead = 0;
}
static void foxTrackStatic() {
  lcdFill(0, 37, LCD_W, 372, C_BG);
  char t[48]; snprintf(t, sizeof(t), "TRACKING  %.24s", foxSsid[0] ? foxSsid : "<hidden>"); section(40, t);
  drawTextf(4, 56, C_GRAY, C_BG, 1, "%02X:%02X:%02X:%02X:%02X:%02X  ch %u (%s GHz)", foxBssid[0], foxBssid[1], foxBssid[2],
            foxBssid[3], foxBssid[4], foxBssid[5], foxChan, foxChan > 14 ? "5" : "2.4");
  lcdRect(9, 121, 302, 16, C_GRAY);
  lcdRect(FOX_CX - 1, FOX_CY - 1, FOX_CW + 2, FOX_CH + 2, C_GRAY);
  drawText(4, 324, "30 s history, 10 samples/s, EMA a=0.3", C_GRAY);
  drawText(4, 340, "-67 dBm (green line): video/VoIP grade", C_GRAY);
  drawButton(bFoxSnd, foxSound, C_YELLOW); drawButton(bFoxRst, false, C_GRAY); drawButton(bFoxList, false, C_CYAN);
  foxDrawTrack(millis());
}
static void foxStart(int i) {
  const ApInfo &a = aps[i];
  memcpy(foxBssid, a.bssid, 6); foxChan = a.ch; snprintf(foxSsid, sizeof(foxSsid), "%s", a.ssid);
  if (!foxImg && psramFound()) foxImg = (uint16_t *)heap_caps_malloc(FOX_CW * FOX_CH * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  foxResetStats();
  if (WiFi.getMode() == WIFI_OFF) WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(false);
  wifi_promiscuous_filter_t flt = {}; flt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&flt);
  esp_wifi_set_promiscuous_rx_cb(foxRx);
  esp_wifi_set_promiscuous(true);
  const esp_err_t e = esp_wifi_set_channel(foxChan, WIFI_SECOND_CHAN_NONE);
  Serial.printf("Fox hunt: %s ch %u, set_channel %s\n", foxSsid, foxChan, esp_err_to_name(e));
  foxTrack = true; foxSampT = foxSecT = foxDrawT = millis();
  foxTrackStatic();
}
static void foxStop() {
  if (!foxTrack) return;
  esp_wifi_set_promiscuous(false);
  foxTrack = false;
  if (audioMode == AM_TONE) audioMode = AM_IDLE;
}
static void foxStatic() { if (foxTrack) foxTrackStatic(); else foxDrawList(); }
static void foxUpdate(uint32_t now) {
  if (!foxTrack) return;
  if (now - foxSampT >= 100) {                                     // 10 samples/s
    foxSampT = now;
    int32_t s; uint32_t n;
    taskENTER_CRITICAL(&foxMux); s = foxSum; n = foxCnt; foxSum = 0; foxCnt = 0; taskEXIT_CRITICAL(&foxMux);
    if (n) {
      const float v = (float)s / (float)n;
      foxEma = foxHave ? 0.7f * foxEma + 0.3f * v : v; foxHave = true; foxLastPkt = now; foxPktSec += n;
      const int iv = (int)lrintf(v);
      if (iv < foxMin || foxAccN == 0) foxMin = iv;
      if (iv > foxMax) foxMax = iv;
      foxAcc += v; foxAccN++;
    }
    foxHist[foxHead] = (foxHave && now - foxLastPkt < 1000) ? (int8_t)lrintf(foxEma) : (int8_t)-128;
    foxHead = (foxHead + 1) % FOX_CW;
  }
  if (now - foxSecT >= 1000) { foxSecT = now; foxPps = foxPktSec; foxPktSec = 0; }
  if (now - foxDrawT >= 200) { foxDrawT = now; foxDrawTrack(now); }
  if (foxSound && (audioMode == AM_IDLE || audioMode == AM_TONE)) {   // Geiger-style beeps: faster and higher when stronger
    const bool live = foxHave && now - foxLastPkt < 2000;
    const float q = constrain((foxEma + 90.0f) / 60.0f, 0.0f, 1.0f);
    const uint32_t period = (uint32_t)(800 - 700 * q);
    const bool on = live && (now % period) < 50;
    if (on) { tonePhaseInc = phaseIncFor(500.0f + 1500.0f * q); audioMode = AM_TONE; }
    else if (audioMode == AM_TONE) audioMode = AM_IDLE;
  }
}
static void foxTouch(uint16_t x, uint16_t y) {
  if (!foxTrack) {
    if (hit(bFoxScan, x, y)) { wifiStartScan(); foxDrawList(); return; }
    if (wifiScanning) return;
    const int i = ((int)y - FOX_LIST_Y) / FOX_ROW_H;
    if (y >= FOX_LIST_Y && i >= 0 && i < FOX_ROWS && i < apCount) foxStart(i);
    return;
  }
  if (hit(bFoxSnd, x, y)) { foxSound = !foxSound; if (!foxSound && audioMode == AM_TONE) audioMode = AM_IDLE; drawButton(bFoxSnd, foxSound, C_YELLOW); }
  else if (hit(bFoxRst, x, y)) { foxResetStats(); foxDrawTrack(millis()); lcdFill(0, 308, LCD_W, 13, C_BG); }
  else if (hit(bFoxList, x, y)) { foxStop(); foxDrawList(); }
}

// ------------------------------------------------------------ power manager
static uint32_t lastActivity = 0;
static uint8_t  userBacklight = 100;
static void pmActivity() { lastActivity = millis(); }
static void es8311Suspend() {                               // esp_codec_dev es8311_suspend() sequence
  esW(0x32, 0x00); esW(0x17, 0x00); esW(0x0E, 0xFF); esW(0x12, 0x02); esW(0x14, 0x00);
  esW(0x0D, 0xFA); esW(0x15, 0x00); esW(0x02, 0x10); esW(0x00, 0x00); esW(0x00, 0x1F);
  esW(0x01, 0x30); esW(0x01, 0x00); esW(0x45, 0x00); esW(0x0D, 0xFC); esW(0x02, 0x00);
}
static void enterSleep() {
  Serial.printf("Idle on battery: entering %s\n", cfg.sleepMode == SLEEP_DEEP ? "deep sleep" : "AXP2101 power-off");
  lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
  drawText(16, 180, "Going to sleep", C_ORANGE, C_BG, 2);
  drawText(16, 214, cfg.sleepMode == SLEEP_DEEP ? "Wake: touch the screen or RESET" : "Wake: PWR button or USB plug-in", C_WHITE);
  delay(1500);
  // Quiesce peripherals
  if (page == PG_CAM) camStop();
  ax25RxEnabled = false; audioMode = AM_IDLE;
  if (wifiScanning) WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
  if (codecOk) es8311Suspend();
  i2cW8(A_QMI8658, 0x08, 0x00);                             // IMU sensors disabled
  i2cW8(A_EXIO, EXIO_REG_OUT, EXIO_RST_MASK);               // PA_CTRL low (speaker amp off)
  backlightSet(0);
  lcdCmd(0x28); lcdCmd(0x10); delay(5);                     // display off, sleep in
  if (cfg.sleepMode == SLEEP_POWEROFF) {
    int r27 = i2cR8(A_AXP, 0x27);                           // PWRON ON-level = 128 ms
    if (r27 >= 0) i2cW8(A_AXP, 0x27, (uint8_t)(r27 & 0xFC));
    Serial.flush(); delay(50);
    int r10 = i2cR8(A_AXP, 0x10);
    i2cW8(A_AXP, 0x10, (uint8_t)((r10 < 0 ? 0 : r10) | 0x01));   // software power-off
    for (;;) delay(1000);
  }
  // Deep sleep: camera rails off (re-enabled by axpInit() at boot), touch stays alive for wake-up
  int en = i2cR8(A_AXP, 0x90);
  if (en >= 0) i2cW8(A_AXP, 0x90, (uint8_t)(en & ~((1U << 3) | (1U << 5))));   // ALDO4, BLDO2 off
  i2cW8(A_FT6336, 0xA4, 0x00);                              // G_MODE polling: INT low while touched
  const uint32_t t0 = millis();
  while (digitalRead(PIN_TP_INT) == LOW && millis() - t0 < 3000) delay(10);   // wait for release
  uint64_t mask = 1ULL << PIN_TP_INT;
  rtc_gpio_pullup_en((gpio_num_t)PIN_TP_INT); rtc_gpio_pulldown_dis((gpio_num_t)PIN_TP_INT);
  if (WAKE_ON_GPIO4) { mask |= 1ULL << 4; rtc_gpio_pullup_en(GPIO_NUM_4); rtc_gpio_pulldown_dis(GPIO_NUM_4); }
  esp_sleep_enable_ext1_wakeup_io(mask, ESP_EXT1_WAKEUP_ANY_LOW);
  Serial.flush(); delay(50);
  esp_deep_sleep_start();
}
static void pmUpdate(uint32_t now) {
  static uint32_t tPoll = 0; static int lastShown = -1;
  if (!PM_ENABLE || !axpOk || now - tPoll < 500) return;
  tPoll = now;
  const int s1 = i2cR8(A_AXP, 0x00);
  if (s1 < 0) return;
  const bool batt = !(s1 & 0x20) && (s1 & 0x08);           // VBUS not good and battery present
  if (batt != onBattery) {
    onBattery = batt; lastActivity = now; lastShown = -1;
    backlightSet(batt ? cfg.blBatt : userBacklight);
    Serial.printf("Power source: %s\n", batt ? "battery" : "USB");
    drawBattery(true);
    if (!batt) lcdFill(150, 18, 40, 13, C_BG);
  }
  if (!onBattery) return;
  if (audioMode != AM_IDLE || txActive || wifiScanning || bleScanning || (page == PG_FOX && foxTrack) || (page == PG_SPEC && specEnabled) ||
      page == PG_MAZE || page == PG_SSTV || page == PG_RTTY || (page == PG_RADIO && enOn))
    lastActivity = now;                                            // busy = active
  const uint16_t sleepS = SLEEP_OPTS[cfg.sleepIdx % 6];
  if (!sleepS) { if (lastShown != 999) { lastShown = 999; lcdFill(150, 18, 40, 13, C_BG); } return; }
  const int32_t left = (int32_t)sleepS * 1000 - (int32_t)(now - lastActivity);
  if (left <= 0) enterSleep();
  const int secs = (left + 999) / 1000;
  if (secs != lastShown && millis() >= hdrMsgUntil) { lastShown = secs; drawTextf(150, 18, secs <= 10 ? C_RED : C_ORANGE, C_BG, 1, "z%3d", secs); }
}
static void pmBootReason() {
  const esp_sleep_wakeup_cause_t c = esp_sleep_get_wakeup_cause();
  if (c == ESP_SLEEP_WAKEUP_EXT1) {
    const uint64_t st = esp_sleep_get_ext1_wakeup_status();
    snprintf(wakeMsg, sizeof(wakeMsg), "Woke from deep sleep: %s", (st & (1ULL << PIN_TP_INT)) ? "touch" : "GPIO4");
  } else if (c != ESP_SLEEP_WAKEUP_UNDEFINED) snprintf(wakeMsg, sizeof(wakeMsg), "Wake cause %d", (int)c);
  else if (esp_reset_reason() == ESP_RST_POWERON) snprintf(wakeMsg, sizeof(wakeMsg), "Power-on");
  else snprintf(wakeMsg, sizeof(wakeMsg), "Reset reason %d", (int)esp_reset_reason());
}

// =============================== Navigation ===================================
static void showPage(Page p);
static void goHome();
static void goBack();

// =========================== Icon canvas (72x72) ==============================
static const int CV = 72;
static uint16_t cvBuf[CV * CV];                                   // big-endian RGB565
static inline uint16_t be16(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }
static void cvFill(int x, int y, int w, int h, uint16_t c) {
  c = be16(c);
  for (int j = max(0, y); j < min(CV, y + h); j++)
    for (int i = max(0, x); i < min(CV, x + w); i++) cvBuf[j * CV + i] = c;
}
static void cvDisc(int cx, int cy, int r, uint16_t c) {
  for (int dy = -r; dy <= r; dy++) { const int dx = (int)sqrtf((float)(r * r - dy * dy)); cvFill(cx - dx, cy + dy, 2 * dx + 1, 1, c); }
}
static void cvLine(int x0, int y0, int x1, int y1, int t, uint16_t c) {
  const int n = max(abs(x1 - x0), abs(y1 - y0));
  for (int i = 0; i <= n; i++) {
    const int x = x0 + (n ? (x1 - x0) * i / n : 0), y = y0 + (n ? (y1 - y0) * i / n : 0);
    cvFill(x - t / 2, y - t / 2, t, t, c);
  }
}
static void cvArc(int cx, int cy, int r, int a0, int a1, int t, uint16_t c) {   // degrees, y axis down
  for (int a = a0 * 2; a <= a1 * 2; a++) {
    const float rad = a * 0.5f * 0.01745329f;
    const int x = cx + (int)lrintf(r * cosf(rad)), y = cy + (int)lrintf(r * sinf(rad));
    cvFill(x - t / 2, y - t / 2, t, t, c);
  }
}
static void cvTile(uint16_t c) {                                   // rounded square background
  cvFill(0, 0, CV, CV, C_BG);
  cvFill(6, 0, CV - 12, CV, c); cvFill(0, 6, CV, CV - 12, c);
  cvFill(3, 1, CV - 6, CV - 2, c); cvFill(1, 3, CV - 2, CV - 6, c);
}
static void drawIcon(int id, int x, int y) {
  switch (id) {
    case 0:  // SENSORS: thermometer
      cvTile(0x0451);
      cvFill(31, 10, 10, 40, C_WHITE); cvDisc(36, 54, 11, C_WHITE); cvDisc(36, 54, 7, C_RED);
      cvFill(34, 24, 4, 30, C_RED);
      for (int i = 0; i < 4; i++) cvFill(45, 14 + i * 8, 8, 2, C_WHITE);
      break;
    case 1:  // AUDIO: speaker + waves
      cvTile(0x6011);
      cvFill(12, 28, 10, 16, C_WHITE);
      for (int i = 0; i <= 12; i++) cvFill(22 + i, 28 - i, 1, 16 + 2 * i, C_WHITE);
      cvArc(36, 36, 12, -45, 45, 3, C_WHITE); cvArc(36, 36, 20, -45, 45, 3, C_WHITE); cvArc(36, 36, 28, -45, 45, 3, C_WHITE);
      break;
    case 2:  // CAMERA
      cvTile(0x1928);
      cvFill(10, 24, 52, 34, 0xBDF7); cvFill(22, 17, 18, 8, 0xBDF7); cvFill(50, 28, 6, 4, C_ORANGE);
      cvDisc(36, 41, 14, 0x2104); cvDisc(36, 41, 10, 0x2B5F); cvDisc(32, 37, 3, C_WHITE);
      break;
    case 3:  // AX.25: mast + radiation
      cvTile(0x0320);
      cvLine(36, 22, 36, 60, 3, C_WHITE); cvLine(36, 30, 24, 60, 3, C_WHITE); cvLine(36, 30, 48, 60, 3, C_WHITE);
      cvDisc(36, 20, 4, C_YELLOW);
      cvArc(36, 20, 11, -50, 50, 3, C_YELLOW); cvArc(36, 20, 11, 130, 230, 3, C_YELLOW);
      cvArc(36, 20, 18, -45, 45, 3, C_YELLOW); cvArc(36, 20, 18, 135, 225, 3, C_YELLOW);
      break;
    case 4:  // RADIO: Wi-Fi fan
      cvTile(0x0217);
      cvArc(36, 54, 34, -135, -45, 5, C_WHITE); cvArc(36, 54, 23, -135, -45, 5, C_WHITE); cvArc(36, 54, 12, -135, -45, 5, C_WHITE);
      cvDisc(36, 54, 5, C_WHITE);
      break;
    case 5:  // PAINT: palette + brush
      cvTile(0xC2A0);
      cvDisc(32, 38, 24, 0xEF5B); cvDisc(42, 48, 6, 0xC2A0);
      cvDisc(22, 28, 5, C_RED); cvDisc(36, 22, 5, C_YELLOW); cvDisc(22, 44, 5, C_BLUE); cvDisc(32, 54, 5, C_GREEN);
      cvLine(50, 22, 64, 6, 4, 0x8200); cvDisc(48, 25, 4, 0x0000);
      break;
    case 6:  // NOTES: page with lines
      cvTile(0xC560);
      cvFill(16, 10, 40, 52, C_WHITE); cvFill(46, 10, 10, 10, 0xC560); cvLine(46, 10, 56, 20, 2, 0x8410);
      for (int i = 0; i < 6; i++) cvFill(21, 24 + i * 6, i == 5 ? 18 : 30, 2, 0x8410);
      break;
    case 7:  // TETRIS: tetrominoes
      cvTile(0x0000);
      { const int s = 12; const uint16_t cl[4] = {C_CYAN, C_YELLOW, 0xA81F, C_RED};
        const int8_t b[4][4][2] = {{{12, 12}, {24, 12}, {36, 12}, {48, 12}}, {{12, 36}, {24, 36}, {12, 48}, {24, 48}},
                                   {{36, 36}, {48, 36}, {60, 36}, {48, 48}}, {{36, 48}, {48, 60}, {36, 60}, {24, 60}}};
        for (int p = 0; p < 4; p++) for (int k = 0; k < 4; k++) { cvFill(b[p][k][0] - 6, b[p][k][1] - 6, s - 1, s - 1, cl[p]); } }
      break;
    case 9:  // FOXHUNT: radar
      cvTile(0x0120);
      cvArc(36, 38, 28, 0, 360, 2, 0x07E0); cvArc(36, 38, 18, 0, 360, 2, 0x07E0); cvArc(36, 38, 8, 0, 360, 2, 0x07E0);
      cvFill(35, 10, 2, 56, 0x0400); cvFill(8, 37, 56, 2, 0x0400);
      cvLine(36, 38, 58, 18, 3, C_GREEN); cvDisc(50, 26, 4, C_RED);
      break;
    case 10: // SPECTRUM: bars + waterfall strip
      cvTile(0x0000);
      { static const uint8_t hh[10] = {10, 18, 30, 44, 38, 24, 40, 28, 16, 8}; static const uint16_t col[10] = {C_BLUE, C_BLUE, C_CYAN, C_GREEN, C_YELLOW, C_GREEN, C_ORANGE, C_CYAN, C_BLUE, C_BLUE};
        for (int i = 0; i < 10; i++) cvFill(7 + i * 6, 48 - hh[i], 5, hh[i], col[i]);
        for (int r = 0; r < 3; r++) for (int i = 0; i < 10; i++) cvFill(7 + i * 6, 52 + r * 5, 5, 4, (i + r) % 4 == 0 ? C_YELLOW : (i % 3 ? 0x0010 : C_CYAN)); }
      break;
    case 11: // SETTINGS: gear
      cvTile(0x4A69);
      for (int k = 0; k < 8; k++) { const float a = k * 0.785398f; cvFill(36 + (int)lrintf(22 * cosf(a)) - 5, 36 + (int)lrintf(22 * sinf(a)) - 5, 10, 10, 0xCE79); }
      cvDisc(36, 36, 19, 0xCE79); cvDisc(36, 36, 8, 0x4A69);
      break;
    case 12: // APRS: map + pin
      cvTile(0x0410);
      cvFill(10, 14, 52, 46, 0xD6BA);
      for (int i = 0; i < 4; i++) { cvFill(10, 24 + i * 10, 52, 1, 0x9CD3); cvFill(20 + i * 12, 14, 1, 46, 0x9CD3); }
      cvLine(14, 54, 30, 40, 2, C_BLUE); cvLine(30, 40, 58, 46, 2, C_BLUE);
      cvDisc(38, 24, 10, C_RED); cvDisc(38, 24, 4, C_WHITE);
      for (int i = 0; i <= 8; i++) cvFill(38 - (8 - i), 30 + i * 2, 2 * (8 - i) + 1, 2, C_RED);
      break;
    case 13: // MAZE: walls + ball
      cvTile(0x2945);
      cvFill(8, 8, 56, 3, 0x2DFF); cvFill(8, 61, 56, 3, 0x2DFF); cvFill(8, 8, 3, 56, 0x2DFF); cvFill(61, 8, 3, 56, 0x2DFF);
      cvFill(22, 8, 3, 40, 0x2DFF); cvFill(36, 24, 3, 40, 0x2DFF); cvFill(50, 8, 3, 34, 0x2DFF); cvFill(22, 48, 14, 3, 0x2DFF);
      cvDisc(15, 56, 5, C_RED); cvFill(54, 52, 6, 6, C_GREEN);
      break;
    case 14: // SSTV: monitor with colour bars
      cvTile(0x3186);
      cvFill(8, 14, 56, 42, 0x0000);
      { static const uint16_t cb[7] = {C_WHITE, C_YELLOW, C_CYAN, C_GREEN, 0xF81F, C_RED, C_BLUE}; for (int i = 0; i < 7; i++) cvFill(11 + i * 7, 17, 7, 28, cb[i]); }
      cvFill(11, 45, 50, 8, 0x4208); cvFill(28, 58, 16, 6, 0x8410); cvLine(26, 4, 34, 13, 2, C_WHITE); cvLine(46, 4, 38, 13, 2, C_WHITE);
      break;
    case 15: // RTTY: two FSK tones + text
      cvTile(0x0280);
      cvFill(8, 44, 56, 2, 0x8410); cvFill(22, 18, 5, 26, C_YELLOW); cvFill(44, 22, 5, 22, C_CYAN);
      { const char *t = "RY"; for (int i = 0; i < 2; i++) { const uint8_t *g = font8x13[t[i] - FONT_FIRST];
          for (int r = 0; r < 13; r++) for (int c = 0; c < 8; c++) if (g[r] & (0x80 >> c)) cvFill(14 + i * 24 + c * 2, 48 + r, 2, 1, C_WHITE); } }
      break;
    case 8:  // CALC
      cvTile(0x4208);
      cvFill(12, 8, 48, 56, 0x2104); cvFill(16, 12, 40, 12, 0x07E8);
      for (int r = 0; r < 3; r++) for (int c = 0; c < 4; c++) cvFill(16 + c * 10, 30 + r * 10, 8, 8, c == 3 ? C_ORANGE : 0xBDF7);
      break;
  }
  lcdBlit(x, y, CV, CV, (const uint8_t *)cvBuf);
}

// ================================= HOME =======================================
static const int    NAPPS = 16;                                  // 4x4 grid, full
static const Page   APP_PAGE[NAPPS]  = {PG_SENSOR, PG_AUDIO, PG_CAM, PG_SPEC, PG_RADIO, PG_FOX, PG_AX25, PG_APRS,
                                        PG_SSTV, PG_RTTY, PG_PAINT, PG_NOTES, PG_CALC, PG_TETRIS, PG_MAZE, PG_SET};
static const char  *APP_LABEL[NAPPS] = {"SENSORS", "AUDIO", "CAMERA", "SPECTRUM", "RADIO", "FOXHUNT", "AX.25", "APRS",
                                        "SSTV", "RTTY", "PAINT", "NOTES", "CALC", "TETRIS", "MAZE", "SETTINGS"};
static const uint8_t APP_ICON[NAPPS] = {0, 1, 2, 10, 4, 9, 3, 12, 14, 15, 5, 6, 8, 7, 13, 11};
static void homeCell(int i, int &x, int &y, int &w, int &h) { const int c = i % 4, r = i / 4; x = c * 80; w = 80; y = 40 + r * 110; h = 110; }
static void homeStatic() {
  for (int i = 0; i < NAPPS; i++) {
    int x, y, w, h; homeCell(i, x, y, w, h);
    drawIcon(APP_ICON[i], x + (w - CV) / 2, y + 8);
    drawText(x + (w - (int)strlen(APP_LABEL[i]) * FONT_W) / 2, y + 86, APP_LABEL[i], C_WHITE);
  }
}
static void homeTouch(uint16_t tx, uint16_t ty) {
  for (int i = 0; i < NAPPS; i++) { int x, y, w, h; homeCell(i, x, y, w, h); if (tx >= x && tx < x + w && ty >= y && ty < y + h) { showPage(APP_PAGE[i]); return; } }
}

// ================================= PAINT ======================================
static const int PT_Y0 = 37, PT_H = 383;                          // canvas rows 37..419
static uint16_t *paintFb = nullptr;                               // shadow framebuffer (BE) for persistence + BMP
static const uint16_t PT_COLORS[10] = {C_WHITE, C_RED, C_ORANGE, C_YELLOW, C_GREEN, C_CYAN, C_BLUE, 0xF81F, C_GRAY, C_BLACK};
static const uint8_t  PT_R[3] = {2, 5, 10};
static const Button bPS = {2, 452, 40, 26, "S"}, bPM = {44, 452, 40, 26, "M"}, bPL = {86, 452, 40, 26, "L"};
static const Button bPClr = {130, 452, 90, 26, "CLEAR"}, bPSave = {224, 452, 94, 26, "SAVE"};
static int ptColor = 0, ptSize = 1, ptLastX = -1, ptLastY = -1, ptSaveN = 0;
static uint32_t ptLastT = 0;
static uint8_t bmpLine[LCD_W * 3];
static void paintSpan(int x0, int x1, int y, uint16_t c) {
  if (y < PT_Y0 || y >= PT_Y0 + PT_H) return;
  x0 = max(x0, 0); x1 = min(x1, LCD_W - 1);
  if (x1 < x0) return;
  lcdFill(x0, y, x1 - x0 + 1, 1, c);
  if (paintFb) { const uint16_t b = be16(c); uint16_t *p = paintFb + (y - PT_Y0) * LCD_W; for (int x = x0; x <= x1; x++) p[x] = b; }
}
static void paintDot(int cx, int cy, int r, uint16_t c) {
  for (int dy = -r; dy <= r; dy++) { const int dx = (int)sqrtf((float)(r * r - dy * dy)); paintSpan(cx - dx, cx + dx, cy + dy, c); }
}
static void paintToolbar() {
  lcdFill(0, 420, LCD_W, 60, C_BG);
  for (int i = 0; i < 10; i++) {
    const int x = 2 + i * 32;
    lcdFill(x + 2, 426, 26, 20, PT_COLORS[i]);
    lcdRect(x, 424, 30, 24, i == ptColor ? C_WHITE : C_DGRAY);
    lcdRect(x + 1, 425, 28, 22, i == ptColor ? C_WHITE : C_BG);
  }
  drawButton(bPS, ptSize == 0, C_CYAN, 8, 13); drawButton(bPM, ptSize == 1, C_CYAN, 8, 13); drawButton(bPL, ptSize == 2, C_CYAN, 8, 13);
  drawButton(bPClr, false, C_GRAY, 8, 13); drawButton(bPSave, false, C_GRAY, 8, 13);
}
static void paintStatic() {
  if (!paintFb && psramFound()) {
    paintFb = (uint16_t *)heap_caps_malloc(LCD_W * PT_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (paintFb) memset(paintFb, 0, LCD_W * PT_H * 2);
  }
  if (paintFb) lcdBlit(0, PT_Y0, LCD_W, PT_H, (const uint8_t *)paintFb);   // restore the drawing
  else lcdFill(0, PT_Y0, LCD_W, PT_H, C_BLACK);
  paintToolbar();
}
static bool paintSaveBmp(char *name, size_t n) {   // 24-bit bottom-up BMP from the shadow framebuffer
  if (!sdOk || !paintFb) return false;
  for (int k = ptSaveN + 1; k < 1000; k++) { snprintf(name, n, "/paint_%03d.bmp", k); if (!SD.exists(name)) { ptSaveN = k; break; } }
  File f = SD.open(name, FILE_WRITE);
  if (!f) return false;
  const uint32_t W = LCD_W, H = PT_H, row = W * 3, data = row * H;   // 960-byte rows: already 4-byte aligned
  uint8_t h[54] = {'B', 'M'};
  auto put32 = [&](int o, uint32_t v) { h[o] = (uint8_t)v; h[o+1] = (uint8_t)(v >> 8); h[o+2] = (uint8_t)(v >> 16); h[o+3] = (uint8_t)(v >> 24); };
  put32(2, 54 + data); put32(10, 54); put32(14, 40); put32(18, W); put32(22, H); h[26] = 1; h[28] = 24; put32(34, data);
  bool ok = f.write(h, 54) == 54;
  for (int y = (int)H - 1; ok && y >= 0; y--) {
    for (uint32_t x = 0; x < W; x++) {
      const uint16_t v = be16(paintFb[y * W + x]);
      const uint8_t r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
      bmpLine[3*x] = (uint8_t)((b5 << 3) | (b5 >> 2)); bmpLine[3*x+1] = (uint8_t)((g6 << 2) | (g6 >> 4)); bmpLine[3*x+2] = (uint8_t)((r5 << 3) | (r5 >> 2));
    }
    ok = f.write(bmpLine, row) == row;
  }
  f.close();
  return ok;
}
static void paintTouch(uint16_t x, uint16_t y, bool press, bool touching) {
  if (!touching) { ptLastX = -1; return; }
  if (y >= 420) {
    if (!press) return;
    for (int i = 0; i < 10; i++) if (x >= 2 + i * 32 && x < 32 + i * 32 && y >= 424 && y < 448) { ptColor = i; paintToolbar(); return; }
    if (hit(bPS, x, y)) ptSize = 0; else if (hit(bPM, x, y)) ptSize = 1; else if (hit(bPL, x, y)) ptSize = 2;
    else if (hit(bPClr, x, y)) { lcdFill(0, PT_Y0, LCD_W, PT_H, C_BLACK); if (paintFb) memset(paintFb, 0, LCD_W * PT_H * 2); }
    else if (hit(bPSave, x, y)) {
      char nm[24] = "";
      const bool ok = paintSaveBmp(nm, sizeof(nm));
      hdrMessage(ok ? "Paint saved:" : "Paint:", ok ? nm + 1 : (sdOk ? "save failed" : "no microSD"), ok ? C_GREEN : C_RED);
      Serial.printf("Paint save: %s %s\n", nm, ok ? "OK" : "failed");
    }
    paintToolbar();
    return;
  }
  if (y < PT_Y0) return;
  const int r = PT_R[ptSize]; const uint16_t c = PT_COLORS[ptColor];
  if (ptLastX < 0 || millis() - ptLastT > 80) paintDot(x, y, r, c);
  else {                                                           // interpolate between touch samples
    const int dx = x - ptLastX, dy = y - ptLastY, n = max(abs(dx), abs(dy)) / max(1, r / 2);
    if (n == 0) paintDot(x, y, r, c);
    for (int i = 1; i <= n; i++) paintDot(ptLastX + dx * i / n, ptLastY + dy * i / n, r, c);
  }
  ptLastX = x; ptLastY = y; ptLastT = millis();
}

// ================================= NOTES ======================================
static const char DEMO_TXT[] =
  "ELECTGPL - ESP32-C5 TOUCH LCD 3.5\n"
  "=================================\n\n"
  "This is the built-in demo document of the Notes app. It is compiled into flash, so the "
  "reader works even without a microSD card.\n\n"
  "Put any .TXT, .MD, .LOG or .CSV file in the root folder of a FAT32 microSD card and it "
  "will show up in the file list. Files up to 256 KB are loaded into PSRAM, UTF-8 Latin "
  "characters (a, e, i, o, u with accents, n with tilde) are folded to plain ASCII and the "
  "text is word-wrapped to 38 columns.\n\n"
  "Navigation:\n"
  "- Drag the text up or down to scroll.\n"
  "- UP / DOWN buttons move one page.\n"
  "- LIST or the PWR button goes back to the file list.\n"
  "- BOOT button goes to the home screen.\n\n"
  "Hardware summary:\n"
  "- ESP32-C5 RISC-V 240 MHz, 8 MB PSRAM, 32 MB flash\n"
  "- Wi-Fi 6 dual-band 2.4/5 GHz, Bluetooth LE 5\n"
  "- ST7796 320x480 SPI LCD, FT6336 capacitive touch\n"
  "- ES8311 audio codec, NS4150B speaker amplifier\n"
  "- BF3901 camera over PARLIO 2-bit\n"
  "- QMI8658 IMU, SHTC3 T/RH, PCF85063A RTC\n"
  "- AXP2101 PMIC with Li-Po charger\n"
  "- CH32V006 I/O expander, microSD slot\n\n"
  "Apps on this firmware: Sensors, Audio recorder, Camera, AX.25 AFSK 1200 terminal, "
  "Wi-Fi/BLE scanner, Paint, Notes, Tetris and Calculator.\n\n"
  "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt "
  "ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation "
  "ullamco laboris nisi ut aliquip ex ea commodo consequat.\n\n"
  "73 de Electgpl\n";
static const int      NT_ROWS = 26, NT_Y = 58, NT_LIST_ROWS = 13, NT_LIST_Y = 60, NT_LIST_H = 26, NT_MAXF = 32;
static const uint32_t NT_MAX_LINES = 12000;
static char     *ntBuf = nullptr;
static uint32_t  ntCap = 0, ntLen = 0;
static uint32_t *ntOff = nullptr;
static uint16_t *ntLn = nullptr;
static int       ntLines = 0, ntTop = 0, ntCount = 0, ntDragY = -1, ntDragTop = 0;
static bool      ntView = false;
static char      ntName[NT_MAXF][40], ntOpen[40] = "";
static uint32_t  ntSize[NT_MAXF];
static const Button bNUp = {4, 410, 100, 60, "UP"}, bNDown = {110, 410, 100, 60, "DOWN"}, bNList = {216, 410, 100, 60, "LIST"};

static bool notesAlloc() {
  if (ntBuf) return true;
  ntCap = psramFound() ? 262144 : 16384;
  ntBuf = (char *)(psramFound() ? heap_caps_malloc(ntCap + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : malloc(ntCap + 1));
  const uint32_t nl = psramFound() ? NT_MAX_LINES : 1000;
  ntOff = (uint32_t *)(psramFound() ? heap_caps_malloc(nl * 4, MALLOC_CAP_SPIRAM) : malloc(nl * 4));
  ntLn  = (uint16_t *)(psramFound() ? heap_caps_malloc(nl * 2, MALLOC_CAP_SPIRAM) : malloc(nl * 2));
  return ntBuf && ntOff && ntLn;
}
static uint32_t asciiFold(char *s, uint32_t n) {                   // UTF-8 Latin-1 -> ASCII, CR/TAB cleanup, in place
  static const char LAT[] = "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYPsaaaaaaaceeeeiiiidnooooo/ouuuuypy";
  uint32_t o = 0;
  for (uint32_t i = 0; i < n; i++) {
    const uint8_t c = (uint8_t)s[i];
    if (c == '\r') continue;
    if (c == '\t') { s[o++] = ' '; continue; }
    if (c < 0x80) { s[o++] = (c < 0x20 && c != '\n') ? ' ' : (char)c; continue; }
    if (c == 0xC3 && i + 1 < n && ((uint8_t)s[i + 1] & 0xC0) == 0x80) { s[o++] = LAT[(uint8_t)s[++i] - 0x80]; continue; }
    if (c == 0xC2 && i + 1 < n) { const uint8_t d = (uint8_t)s[++i]; s[o++] = d == 0xBF ? '?' : d == 0xA1 ? '!' : d == 0xB0 ? 'o' : d == 0xA0 ? ' ' : '?'; continue; }
    if (c >= 0xC0) { i += c >= 0xF0 ? 3 : c >= 0xE0 ? 2 : 1; s[o++] = '?'; continue; }
  }
  s[o] = 0;
  return o;
}
static void notesWrap() {                                          // word wrap to TERM_COLS
  const uint32_t nl = psramFound() ? NT_MAX_LINES : 1000;
  ntLines = 0; uint32_t i = 0;
  while (i < ntLen && (uint32_t)ntLines < nl) {
    const uint32_t start = i; uint32_t lastSp = UINT32_MAX; int col = 0;
    while (i < ntLen && ntBuf[i] != '\n' && col < TERM_COLS) { if (ntBuf[i] == ' ') lastSp = i; i++; col++; }
    uint32_t end = i;
    if (i < ntLen && ntBuf[i] != '\n' && col >= TERM_COLS && lastSp != UINT32_MAX && lastSp > start) { end = lastSp; i = lastSp + 1; }
    else if (i < ntLen && ntBuf[i] == '\n') i++;
    ntOff[ntLines] = start; ntLn[ntLines] = (uint16_t)(end - start); ntLines++;
  }
}
static void notesScan() {
  ntCount = 0;
  snprintf(ntName[0], 40, "DEMO.TXT (built-in)"); ntSize[0] = sizeof(DEMO_TXT) - 1; ntCount = 1;
  if (!sdOk) return;
  File root = SD.open("/");
  if (!root) return;
  for (File f = root.openNextFile(); f && ntCount < NT_MAXF; f = root.openNextFile()) {
    if (f.isDirectory()) continue;
    const char *n = f.name(); const char *dot = strrchr(n, '.');
    if (!dot || (strcasecmp(dot, ".txt") && strcasecmp(dot, ".md") && strcasecmp(dot, ".log") && strcasecmp(dot, ".csv"))) continue;
    snprintf(ntName[ntCount], 40, "%s", n); ntSize[ntCount] = (uint32_t)f.size(); ntCount++;
  }
  root.close();
}
static void notesDrawText() {
  for (int r = 0; r < NT_ROWS; r++) {
    char b[TERM_COLS + 1]; const int li = ntTop + r;
    if (li < ntLines) snprintf(b, sizeof(b), "%-*.*s", TERM_COLS, (int)ntLn[li], ntBuf + ntOff[li]);
    else snprintf(b, sizeof(b), "%-*s", TERM_COLS, "");
    drawText(TERM_X, NT_Y + r * FONT_H, b, C_WHITE, C_BLACK);
  }
  drawTextf(4, 40, C_CYAN, C_BG, 1, "%-20.20s L%5d-%-5d/%-5d", ntOpen, ntTop + 1, min(ntTop + NT_ROWS, ntLines), ntLines);
}
static void notesStatic() {
  lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
  if (!notesAlloc()) { drawText(8, 60, "Out of memory", C_RED); return; }
  if (ntView) {
    lcdRect(TERM_X - 3, NT_Y - 3, TERM_COLS * FONT_W + 6, NT_ROWS * FONT_H + 6, C_GRAY);
    notesDrawText();
    drawButton(bNUp, false, C_CYAN); drawButton(bNDown, false, C_CYAN); drawButton(bNList, false, C_CYAN);
  } else {
    notesScan();
    section(40, sdOk ? "FILES  SD:/ *.txt *.md *.log *.csv" : "FILES  (no microSD: demo only)");
    for (int i = 0; i < NT_LIST_ROWS && i < ntCount; i++) {
      const int y = NT_LIST_Y + i * NT_LIST_H; const uint16_t bg = (i & 1) ? C_BG : C_DGRAY;
      lcdFill(4, y, 312, NT_LIST_H - 2, bg);
      drawTextf(10, y + 6, i ? C_WHITE : C_YELLOW, bg, 1, "%-26.26s", ntName[i]);
      drawTextf(226, y + 6, C_GRAY, bg, 1, "%8lu B", (unsigned long)ntSize[i]);
    }
    if (ntCount > NT_LIST_ROWS) drawTextf(10, NT_LIST_Y + NT_LIST_ROWS * NT_LIST_H + 4, C_GRAY, C_BG, 1, "+%d more files not shown", ntCount - NT_LIST_ROWS);
  }
}
static void notesOpen(int i) {
  if (i == 0) { ntLen = min((uint32_t)(sizeof(DEMO_TXT) - 1), ntCap); memcpy(ntBuf, DEMO_TXT, ntLen); }
  else {
    char path[44]; snprintf(path, sizeof(path), "/%s", ntName[i]);
    File f = SD.open(path, FILE_READ);
    if (!f) { drawText(8, 420, "Cannot open file", C_RED); return; }
    ntLen = (uint32_t)f.read((uint8_t *)ntBuf, ntCap); f.close();
  }
  ntLen = asciiFold(ntBuf, ntLen);
  notesWrap();
  snprintf(ntOpen, sizeof(ntOpen), "%s", i == 0 ? "DEMO.TXT" : ntName[i]);
  ntTop = 0; ntView = true;
  notesStatic();
}
static void notesScroll(int top) {
  top = constrain(top, 0, max(0, ntLines - NT_ROWS));
  if (top != ntTop) { ntTop = top; notesDrawText(); }
}
static void notesTouch(uint16_t x, uint16_t y, bool press, bool touching) {
  if (!ntView) {
    if (!press) return;
    const int i = ((int)y - NT_LIST_Y) / NT_LIST_H;
    if (y >= NT_LIST_Y && i >= 0 && i < NT_LIST_ROWS && i < ntCount) notesOpen(i);
    return;
  }
  if (!touching) { ntDragY = -1; return; }
  if (press) {
    if (hit(bNUp, x, y)) { notesScroll(ntTop - NT_ROWS); return; }
    if (hit(bNDown, x, y)) { notesScroll(ntTop + NT_ROWS); return; }
    if (hit(bNList, x, y)) { ntView = false; notesStatic(); return; }
    if (y >= NT_Y && y < NT_Y + NT_ROWS * FONT_H) { ntDragY = y; ntDragTop = ntTop; }
    return;
  }
  if (ntDragY >= 0) notesScroll(ntDragTop - ((int)y - ntDragY) / FONT_H);   // drag to scroll
}

// ================================ TETRIS ======================================
static const uint16_t TT_SHAPE[7][4] = {                           // 4x4 masks, bit15 = row0/col0
  {0x0F00, 0x2222, 0x00F0, 0x4444}, {0x44C0, 0x8E00, 0x6440, 0x0E20}, {0x4460, 0x0E80, 0xC440, 0x2E00},
  {0xCC00, 0xCC00, 0xCC00, 0xCC00}, {0x06C0, 0x8C40, 0x6C00, 0x4620}, {0x0E40, 0x4C40, 0x4E00, 0x4640},
  {0x0C60, 0x4C80, 0xC600, 0x2640}};
static const uint16_t TT_COLOR[10] = {0x0841, C_CYAN, C_BLUE, C_ORANGE, C_YELLOW, C_GREEN, 0xA81F, C_RED, 0x31A6, 0x0000};
static const int TT_X = 8, TT_Y = 44, TT_C = 16, TT_W = 10, TT_H = 20;
static uint8_t  ttBoard[TT_H][TT_W], ttShown[TT_H][TT_W];
static int      ttType, ttRot, ttX, ttY, ttNext = -1, ttBag[7], ttBagN = 0, ttLevel = 1, ttHold = -1;
static uint32_t ttScore = 0, ttLines = 0, ttLastFall = 0, ttHoldT = 0;
static bool     ttOver = true, ttStarted = false;
static const Button bTL = {4, 372, 100, 50, "<"}, bTRot = {110, 372, 100, 50, "ROT"}, bTR = {216, 372, 100, 50, ">"};
static const Button bTDn = {4, 426, 100, 50, "DOWN"}, bTDrop = {110, 426, 100, 50, "DROP"}, bTNew = {216, 426, 100, 50, "NEW"};

static int ttRand() {                                              // 7-bag randomizer
  if (ttBagN == 0) {
    for (int i = 0; i < 7; i++) ttBag[i] = i;
    for (int i = 6; i > 0; i--) { const int j = (int)(esp_random() % (uint32_t)(i + 1)); const int t = ttBag[i]; ttBag[i] = ttBag[j]; ttBag[j] = t; }
    ttBagN = 7;
  }
  return ttBag[--ttBagN];
}
static bool ttFits(int type, int rot, int x, int y) {
  const uint16_t m = TT_SHAPE[type][rot & 3];
  for (int k = 0; k < 16; k++) if (m & (0x8000 >> k)) {
    const int bx = x + (k & 3), by = y + (k >> 2);
    if (bx < 0 || bx >= TT_W || by >= TT_H) return false;
    if (by >= 0 && ttBoard[by][bx]) return false;
  }
  return true;
}
static void ttCell(int c, int r, uint8_t v) {
  const int x = TT_X + c * TT_C, y = TT_Y + r * TT_C;
  lcdFill(x, y, TT_C, TT_C, C_BLACK);
  lcdFill(x, y, TT_C - 1, TT_C - 1, TT_COLOR[v]);
  if (v && v < 8) { lcdFill(x, y, TT_C - 1, 2, C_WHITE); }        // highlight edge
}
static void ttRender(bool full) {
  uint8_t g[TT_H][TT_W]; memcpy(g, ttBoard, sizeof(g));
  if (!ttOver) {
    int gy = ttY; while (ttFits(ttType, ttRot, ttX, gy + 1)) gy++;           // ghost piece
    const uint16_t m = TT_SHAPE[ttType][ttRot];
    for (int k = 0; k < 16; k++) if (m & (0x8000 >> k)) { const int bx = ttX + (k & 3), by = gy + (k >> 2); if (by >= 0 && !g[by][bx]) g[by][bx] = 8; }
    for (int k = 0; k < 16; k++) if (m & (0x8000 >> k)) { const int bx = ttX + (k & 3), by = ttY + (k >> 2); if (by >= 0) g[by][bx] = (uint8_t)(ttType + 1); }
  }
  for (int r = 0; r < TT_H; r++) for (int c = 0; c < TT_W; c++)
    if (full || g[r][c] != ttShown[r][c]) { ttCell(c, r, g[r][c]); ttShown[r][c] = g[r][c]; }
}
static void ttPanel() {
  drawText(184, 44, "NEXT", C_GRAY);
  lcdFill(184, 60, 52, 52, C_BLACK);
  if (ttNext >= 0) { const uint16_t m = TT_SHAPE[ttNext][0];
    for (int k = 0; k < 16; k++) if (m & (0x8000 >> k)) lcdFill(186 + (k & 3) * 12, 62 + (k >> 2) * 12, 11, 11, TT_COLOR[ttNext + 1]); }
  drawText(184, 124, "SCORE", C_GRAY); drawTextf(184, 140, C_WHITE, C_BG, 2, "%-7lu", (unsigned long)ttScore);
  drawText(184, 176, "LINES", C_GRAY); drawTextf(184, 192, C_WHITE, C_BG, 2, "%-7lu", (unsigned long)ttLines);
  drawText(184, 228, "LEVEL", C_GRAY); drawTextf(184, 244, C_WHITE, C_BG, 2, "%-3d", ttLevel);
  drawText(184, 290, ttOver ? (ttStarted ? "GAME OVER    " : "Press NEW    ") : "             ", ttOver ? C_RED : C_WHITE);
}
static bool ttSpawn() {
  ttType = ttNext < 0 ? ttRand() : ttNext; ttNext = ttRand(); ttRot = 0; ttX = 3; ttY = 0;
  if (!ttFits(ttType, ttRot, ttX, ttY)) { ttOver = true; return false; }
  return true;
}
static void ttNewGame() {
  memset(ttBoard, 0, sizeof(ttBoard)); ttScore = 0; ttLines = 0; ttLevel = 1; ttBagN = 0; ttNext = -1;
  ttOver = false; ttStarted = true; ttSpawn(); ttLastFall = millis();
  ttRender(true); ttPanel();
}
static uint32_t ttInterval() { return (uint32_t)max(80, 800 - (ttLevel - 1) * 70); }
static void ttLock() {
  const uint16_t m = TT_SHAPE[ttType][ttRot];
  for (int k = 0; k < 16; k++) if (m & (0x8000 >> k)) {
    const int bx = ttX + (k & 3), by = ttY + (k >> 2);
    if (by < 0) { ttOver = true; } else ttBoard[by][bx] = (uint8_t)(ttType + 1);
  }
  int cleared = 0;
  for (int r = TT_H - 1; r >= 0; r--) {
    bool full = true; for (int c = 0; c < TT_W; c++) if (!ttBoard[r][c]) { full = false; break; }
    if (full) { memmove(&ttBoard[1], &ttBoard[0], (size_t)r * TT_W); memset(ttBoard[0], 0, TT_W); cleared++; r++; }
  }
  static const uint16_t pts[5] = {0, 100, 300, 500, 800};
  ttScore += (uint32_t)pts[cleared] * (uint32_t)ttLevel; ttLines += (uint32_t)cleared;
  ttLevel = 1 + (int)(ttLines / 10);
  if (!ttOver) ttSpawn();
  ttRender(false); ttPanel();
}
static bool ttMove(int dx, int dy, int dr) {
  if (ttOver) return false;
  int nr = (ttRot + dr) & 3;
  static const int kicks[3] = {0, -1, 1};
  for (int k = 0; k < (dr ? 3 : 1); k++)                           // simple wall kick for rotations
    if (ttFits(ttType, nr, ttX + dx + kicks[k], ttY + dy)) { ttX += dx + kicks[k]; ttY += dy; ttRot = nr; ttRender(false); return true; }
  return false;
}
static void tetrisStatic() {
  lcdRect(TT_X - 1, TT_Y - 1, TT_W * TT_C + 1, TT_H * TT_C + 1, C_GRAY);
  ttRender(true); ttPanel();
  drawButton(bTL, false, C_CYAN); drawButton(bTRot, false, C_CYAN); drawButton(bTR, false, C_CYAN);
  drawButton(bTDn, false, C_CYAN); drawButton(bTDrop, false, C_CYAN); drawButton(bTNew, false, C_GREEN);
}
static void tetrisUpdate(uint32_t now) {
  if (ttOver) return;
  if (ttHold >= 0 && now - ttHoldT >= (ttHold == 3 ? 50u : 90u) && now - ttHoldT < 100000) {   // auto-repeat
    ttHoldT = now;
    if (ttHold == 0) ttMove(-1, 0, 0); else if (ttHold == 2) ttMove(1, 0, 0);
    else if (ttHold == 3 && ttMove(0, 1, 0)) { ttScore++; ttLastFall = now; }
  }
  if (now - ttLastFall >= ttInterval()) { ttLastFall = now; if (!ttMove(0, 1, 0)) ttLock(); }
}
static void tetrisTouch(uint16_t x, uint16_t y, bool press, bool touching) {
  if (!touching) { ttHold = -1; return; }
  if (!press) return;
  const uint32_t now = millis();
  if (hit(bTNew, x, y)) { ttNewGame(); return; }
  if (ttOver) return;
  if (hit(bTL, x, y))        { ttMove(-1, 0, 0); ttHold = 0; ttHoldT = now + 160; }     // first repeat after ~250 ms
  else if (hit(bTR, x, y))   { ttMove(1, 0, 0);  ttHold = 2; ttHoldT = now + 160; }
  else if (hit(bTRot, x, y))   ttMove(0, 0, 1);
  else if (hit(bTDn, x, y))  { if (ttMove(0, 1, 0)) ttScore++; ttHold = 3; ttHoldT = now + 100; }
  else if (hit(bTDrop, x, y)) { int n = 0; while (ttFits(ttType, ttRot, ttX, ttY + 1)) { ttY++; n++; } ttScore += 2u * (uint32_t)n; ttLock(); }
}

// ============================== CALCULATOR ====================================
static const char *CC_KEY[20] = {"C", "+/-", "%", "/", "7", "8", "9", "*", "4", "5", "6", "-", "1", "2", "3", "+", "0", ".", "<", "="};
static char   ccEntry[24] = "0", ccHist[40] = "";
static double ccAcc = 0;
static char   ccOp = 0;
static bool   ccNew = true, ccErr = false;
static Button ccBtn(int i) { Button b = {(int16_t)(4 + (i % 4) * 79), (int16_t)(130 + (i / 4) * 69), 75, 64, CC_KEY[i]}; return b; }
static void ccFormat(double v, char *out, size_t n) {
  if (!isfinite(v)) { snprintf(out, n, "Error"); return; }
  char b[32];
  if (v != 0 && (fabs(v) >= 1e12 || fabs(v) < 1e-9)) snprintf(b, sizeof(b), "%.5e", v);
  else { snprintf(b, sizeof(b), "%.10g", v); if (strlen(b) > 12) snprintf(b, sizeof(b), "%.5e", v); }
  snprintf(out, n, "%s", b);
}
static void calcDisplay() {
  lcdFill(4, 44, 312, 80, 0x1082); lcdRect(4, 44, 312, 80, C_GRAY);
  drawTextf(12, 52, C_GRAY, 0x1082, 1, "%36s", ccHist);
  const int w = (int)strlen(ccEntry) * 24;
  drawText(308 - w, 76, ccEntry, ccErr ? C_RED : C_WHITE, 0x1082, 3);
}
static void calcStatic() {
  for (int i = 0; i < 20; i++) {
    const char *k = CC_KEY[i]; const bool op = strchr("/*-+", k[0]) && k[1] == 0;
    drawButton(ccBtn(i), true, i == 0 ? C_RED : i == 19 ? C_GREEN : op ? C_ORANGE : 0xBDF7);
  }
  calcDisplay();
}
static double ccApply(double a, char op, double b) {
  switch (op) { case '+': return a + b; case '-': return a - b; case '*': return a * b; case '/': return b == 0 ? NAN : a / b; default: return b; }
}
static void calcKey(const char *k) {
  if (ccErr && strcmp(k, "C")) return;
  const double cur = atof(ccEntry);
  if (k[0] >= '0' && k[0] <= '9') {
    if (ccNew) { snprintf(ccEntry, sizeof(ccEntry), "%s", k); ccNew = false; }
    else if (strlen(ccEntry) < 12) { if (!strcmp(ccEntry, "0")) snprintf(ccEntry, sizeof(ccEntry), "%s", k); else strcat(ccEntry, k); }
  } else if (!strcmp(k, ".")) {
    if (ccNew) { snprintf(ccEntry, sizeof(ccEntry), "0."); ccNew = false; }
    else if (!strchr(ccEntry, '.') && strlen(ccEntry) < 12) strcat(ccEntry, ".");
  } else if (!strcmp(k, "<")) {
    if (!ccNew) { const size_t n = strlen(ccEntry); if (n > 1 && !(n == 2 && ccEntry[0] == '-')) ccEntry[n - 1] = 0; else snprintf(ccEntry, sizeof(ccEntry), "0"); }
  } else if (!strcmp(k, "+/-")) {
    if (strcmp(ccEntry, "0")) { if (ccEntry[0] == '-') memmove(ccEntry, ccEntry + 1, strlen(ccEntry)); else if (strlen(ccEntry) < 13) { memmove(ccEntry + 1, ccEntry, strlen(ccEntry) + 1); ccEntry[0] = '-'; } }
  } else if (!strcmp(k, "%")) {
    ccFormat((ccOp == '+' || ccOp == '-') ? ccAcc * cur / 100.0 : cur / 100.0, ccEntry, sizeof(ccEntry)); ccNew = true;
  } else if (!strcmp(k, "C")) {
    snprintf(ccEntry, sizeof(ccEntry), "0"); ccAcc = 0; ccOp = 0; ccNew = true; ccErr = false; ccHist[0] = 0;
  } else if (!strcmp(k, "=")) {
    if (ccOp) {
      char a[24], b[24]; ccFormat(ccAcc, a, sizeof(a)); ccFormat(cur, b, sizeof(b));
      snprintf(ccHist, sizeof(ccHist), "%s %c %s =", a, ccOp, b);
      const double r = ccApply(ccAcc, ccOp, cur); ccFormat(r, ccEntry, sizeof(ccEntry)); ccErr = !isfinite(r);
      ccAcc = r; ccOp = 0; ccNew = true;
    }
  } else {                                                         // + - * /
    if (ccOp && !ccNew) { const double r = ccApply(ccAcc, ccOp, cur); ccAcc = r; ccFormat(r, ccEntry, sizeof(ccEntry)); ccErr = !isfinite(r); }
    else if (!ccOp || !ccNew) ccAcc = cur;
    ccOp = k[0]; ccNew = true;
    char a[24]; ccFormat(ccAcc, a, sizeof(a)); snprintf(ccHist, sizeof(ccHist), "%s %c", a, ccOp);
  }
  calcDisplay();
}
static void calcTouch(uint16_t x, uint16_t y) {
  for (int i = 0; i < 20; i++) if (hit(ccBtn(i), x, y)) { calcKey(CC_KEY[i]); return; }
}

// ======================== SPECTRUM (FFT + waterfall) ==========================
static const int FFT_N = 512, SP_Y = 56, SP_H = 110, WF_Y = 186, WF_H = 200;
static const float SPEC_RANGES[3] = {60.0f, 80.0f, 100.0f};
static int16_t  fftCos[FFT_N / 2], fftSin[FFT_N / 2], hannW[FFT_N];
static int32_t  fRe[FFT_N], fIm[FFT_N];
static float    specDb[FFT_N / 2], specAvg[FFT_N / 2], specPk[FFT_N / 2];
static uint16_t *specImg = nullptr, *wfImg = nullptr, specPal[256];
static int      wfHead = 0, specRangeIdx = 2;
static uint32_t specLastW = 0, specFrames = 0, specFpsT = 0;
static float    specFps = 0;
static bool     specHold = false, specAvgOn = true, specTables = false;
static const Button bSpHold = {4, 410, 100, 60, "HOLD"}, bSpRange = {110, 410, 100, 60, "RANGE"}, bSpAvg = {216, 410, 100, 60, "AVG"};

static void fftInitTables() {
  for (int k = 0; k < FFT_N / 2; k++) {
    fftCos[k] = (int16_t)lrintf(32767.0f * cosf(2.0f * (float)M_PI * k / FFT_N));
    fftSin[k] = (int16_t)lrintf(32767.0f * sinf(2.0f * (float)M_PI * k / FFT_N));
  }
  for (int n = 0; n < FFT_N; n++) hannW[n] = (int16_t)lrintf(32767.0f * (0.5f - 0.5f * cosf(2.0f * (float)M_PI * n / (FFT_N - 1))));
  static const float st[8] = {0.0f, 0.15f, 0.35f, 0.5f, 0.65f, 0.8f, 0.92f, 1.0f};
  static const uint8_t cc[8][3] = {{0, 0, 0}, {0, 0, 96}, {0, 0, 255}, {0, 255, 255}, {0, 255, 0}, {255, 255, 0}, {255, 0, 0}, {255, 255, 255}};
  for (int i = 0; i < 256; i++) {
    const float f = i / 255.0f; int k = 0; while (k < 6 && f > st[k + 1]) k++;
    const float t = (f - st[k]) / (st[k + 1] - st[k]);
    const uint8_t r = (uint8_t)(cc[k][0] + t * (cc[k + 1][0] - cc[k][0])), g = (uint8_t)(cc[k][1] + t * (cc[k + 1][1] - cc[k][1])), b = (uint8_t)(cc[k][2] + t * (cc[k + 1][2] - cc[k][2]));
    specPal[i] = be16((uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)));
  }
  specTables = true;
}
static void fftRun() {                                             // radix-2 DIT, Q15 twiddles, /2 per stage (=> X/N)
  for (int i = 1, j = 0; i < FFT_N; i++) {
    int bit = FFT_N >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { int32_t t = fRe[i]; fRe[i] = fRe[j]; fRe[j] = t; t = fIm[i]; fIm[i] = fIm[j]; fIm[j] = t; }
  }
  for (int len = 2; len <= FFT_N; len <<= 1) {
    const int half = len >> 1, step = FFT_N / len;
    for (int i = 0; i < FFT_N; i += len)
      for (int k = 0; k < half; k++) {
        const int32_t wr = fftCos[k * step], wi = -fftSin[k * step];
        const int a = i + k, b = a + half;
        const int32_t tr = (int32_t)(((int64_t)fRe[b] * wr - (int64_t)fIm[b] * wi) >> 15);
        const int32_t ti = (int32_t)(((int64_t)fRe[b] * wi + (int64_t)fIm[b] * wr) >> 15);
        const int32_t ur = fRe[a], ui = fIm[a];
        fRe[a] = (ur + tr) >> 1; fIm[a] = (ui + ti) >> 1;
        fRe[b] = (ur - tr) >> 1; fIm[b] = (ui - ti) >> 1;
      }
  }
}
static void specLabels() {
  for (int k = 0; k <= 8; k++) { char t[4]; snprintf(t, sizeof(t), k ? "%dk" : "0", k); drawText(min(k * 40 - (k ? 8 : 0), 304), SP_Y + SP_H + 2, t, C_GRAY); }
}
static void specStatic() {
  if (!specTables) fftInitTables();
  if (psramFound()) {
    if (!specImg) specImg = (uint16_t *)heap_caps_malloc(LCD_W * SP_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wfImg)   wfImg   = (uint16_t *)heap_caps_malloc(LCD_W * WF_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!specImg || !wfImg) { drawText(8, 60, "PSRAM required for the spectrum app", C_RED); return; }
  memset(wfImg, 0, LCD_W * WF_H * 2);
  for (int i = 0; i < FFT_N / 2; i++) { specAvg[i] = -120; specPk[i] = -120; }
  section(40, "AUDIO SPECTRUM  0-8 kHz  FFT 512  Hann");
  specLabels();
  lcdFill(0, WF_Y, LCD_W, WF_H, C_BLACK);
  drawButton(bSpHold, specHold, C_YELLOW); drawButton(bSpRange, false, C_CYAN); drawButton(bSpAvg, specAvgOn, C_CYAN);
  specLastW = specW; specEnabled = true;
}
static void specRender() {
  const float range = SPEC_RANGES[specRangeIdx], floorDb = -range;
  const uint16_t grid = be16(0x2104), bg = 0, pkc = be16(C_WHITE);
  for (int x = 0; x < LCD_W; x++) {
    const int bin = x * (FFT_N / 2) / LCD_W;
    const float v = specAvgOn ? specAvg[bin] : specDb[bin];
    const int h = constrain((int)((v - floorDb) / range * SP_H), 0, SP_H);
    const int hp = constrain((int)((specPk[bin] - floorDb) / range * SP_H), 0, SP_H - 1);
    const bool vline = (x % 40) == 0;
    for (int y = 0; y < SP_H; y++) {
      const int fromBottom = SP_H - 1 - y;
      uint16_t c;
      if (fromBottom < h) c = specPal[constrain(fromBottom * 255 / SP_H + 40, 0, 255)];
      else c = (vline || (y % (SP_H / 5)) == 0) ? grid : bg;
      if (fromBottom == hp) c = pkc;
      specImg[y * LCD_W + x] = c;
    }
  }
  lcdBlit(0, SP_Y, LCD_W, SP_H, (const uint8_t *)specImg);
  wfHead = (wfHead + WF_H - 1) % WF_H;                             // newest row on top
  uint16_t *row = wfImg + wfHead * LCD_W;
  for (int x = 0; x < LCD_W; x++) {
    const int bin = x * (FFT_N / 2) / LCD_W;
    row[x] = specPal[constrain((int)((specDb[bin] - floorDb) / range * 255.0f), 0, 255)];
  }
  SPI.beginTransaction(lcdSpi); digitalWrite(PIN_LCD_CS, LOW);
  lcdWindowRaw(0, WF_Y, LCD_W - 1, WF_Y + WF_H - 1);
  for (int r = 0; r < WF_H; r++) SPI.writeBytes((const uint8_t *)(wfImg + ((wfHead + r) % WF_H) * LCD_W), LCD_W * 2);
  digitalWrite(PIN_LCD_CS, HIGH); SPI.endTransaction();
}
static void specUpdate() {
  if (!specEnabled || specHold || !specImg) return;
  const uint32_t w = specW;
  if (w - specLastW < (uint32_t)FFT_N) return;                    // need 512 new samples (32 ms)
  specLastW = w;
  for (int i = 0; i < FFT_N; i++) { fRe[i] = ((int32_t)specRing[(w - FFT_N + i) & (SPEC_RING - 1)] * hannW[i]) >> 15; fIm[i] = 0; }
  fftRun();
  int pk = 2;
  for (int k = 0; k < FFT_N / 2; k++) {
    const int64_t m2 = (int64_t)fRe[k] * fRe[k] + (int64_t)fIm[k] * fIm[k];
    const float db = 10.0f * log10f((float)m2 + 1e-3f) - 78.27f;  // 0 dBFS = full-scale sine (|X/N| = 8192 with Hann)
    specDb[k] = db;
    specAvg[k] += 0.3f * (db - specAvg[k]);
    specPk[k] = max(specPk[k] - 0.4f, db);
    if (k >= 2 && k < FFT_N / 2 - 1 && db > specDb[pk]) pk = k;
  }
  float d = 0;                                                     // parabolic peak interpolation
  if (pk < FFT_N / 2 - 1) { const float a = specDb[pk - 1], b = specDb[pk], c = specDb[pk + 1]; const float den = a - 2 * b + c; if (den < 0) d = 0.5f * (a - c) / den; }
  specRender();
  specFrames++;
  const uint32_t now = millis();
  if (now - specFpsT >= 1000) { specFps = specFrames * 1000.0f / (now - specFpsT); specFrames = 0; specFpsT = now; }
  drawTextf(4, 390, C_WHITE, C_BG, 1, "Peak %6.1f Hz %6.1f dBFS  %3.0f dB %4.1f fps", (pk + d) * (float)AUDIO_FS / FFT_N, specDb[pk],
            SPEC_RANGES[specRangeIdx], specFps);
}
static void specTouch(uint16_t x, uint16_t y) {
  if (hit(bSpHold, x, y)) { specHold = !specHold; drawButton(bSpHold, specHold, C_YELLOW); }
  else if (hit(bSpRange, x, y)) { specRangeIdx = (specRangeIdx + 1) % 3; }
  else if (hit(bSpAvg, x, y)) { specAvgOn = !specAvgOn; drawButton(bSpAvg, specAvgOn, C_CYAN); }
}

// ================================ SETTINGS ====================================
static const int ST_ROWS = 12, ST_Y = 42, ST_H = 32;
static const char *ST_LABEL[ST_ROWS] = {"Backlight USB", "Backlight battery", "Sleep after (batt)", "Sleep mode", "Speaker volume",
                                        "Mic gain", "AX.25 callsign", "AX.25 SSID", "Camera mirror H", "Camera flip V",
                                        "Date & time", "Time zone (NTP)"};
static char kbBuf[66] = "", kbLab[36][2];
static const char KB_KEYS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
static const Button bStDef = {4, 432, 312, 42, "DEFAULTS"};
static const Button bKbDel = {6, 404, 100, 60, "DEL"}, bKbCancel = {110, 404, 100, 60, "CANCEL"}, bKbOk = {214, 404, 100, 60, "OK"};
static Button stMinus(int i) { Button b = {150, (int16_t)(ST_Y + i * ST_H + 3), 40, 26, "-"}; return b; }
static Button stPlus(int i)  { Button b = {278, (int16_t)(ST_Y + i * ST_H + 3), 40, 26, "+"}; return b; }
static Button stWide(int i, const char *l) { Button b = {150, (int16_t)(ST_Y + i * ST_H + 3), 168, 26, l}; return b; }
static Button kbKey(int i)   { Button b = {(int16_t)(8 + (i % 6) * 51), (int16_t)(110 + (i / 6) * 48), 48, 44, kbLab[i]}; return b; }

static void stValue(int i, char *o, size_t n) {
  switch (i) {
    case 0: snprintf(o, n, "%u %%", cfg.blUsb); break;
    case 1: snprintf(o, n, "%u %%", cfg.blBatt); break;
    case 2: if (SLEEP_OPTS[cfg.sleepIdx]) snprintf(o, n, "%u s", SLEEP_OPTS[cfg.sleepIdx]); else snprintf(o, n, "never"); break;
    case 3: snprintf(o, n, "%s", cfg.sleepMode == SLEEP_DEEP ? "DEEP" : "POWEROFF"); break;
    case 4: snprintf(o, n, "%+d dB", cfg.volDb); break;
    case 5: snprintf(o, n, "%u dB", cfg.micGain * 6); break;
    case 7: snprintf(o, n, "%u", cfg.ssid); break;
    case 8: snprintf(o, n, "%s", cfg.hmirror ? "ON" : "OFF"); break;
    case 9: snprintf(o, n, "%s", cfg.vflip ? "ON" : "OFF"); break;
    case 11: snprintf(o, n, "UTC%+d", cfg.tz); break;
    default: o[0] = 0; break;
  }
}
static void stDrawRow(int i) {
  const int y = ST_Y + i * ST_H; const uint16_t bg = (i & 1) ? C_BG : 0x10A2;
  lcdFill(0, y, LCD_W, ST_H, bg);
  drawText(8, y + 9, ST_LABEL[i], C_WHITE, bg);
  if (i == 6)  { drawButton(stWide(i, cfg.call), false, C_CYAN, 12, 20); return; }
  if (i == 10) { drawButton(stWide(i, "SET / SYNC"), false, C_CYAN, 12, 20); return; }
  drawButton(stMinus(i), false, C_CYAN, 12, 20); drawButton(stPlus(i), false, C_CYAN, 12, 20);
  char v[16]; stValue(i, v, sizeof(v));
  drawText(192 + (84 - (int)strlen(v) * FONT_W) / 2, y + 9, v, C_YELLOW, bg);
}
static void stApplyAll() {
  userBacklight = cfg.blUsb;
  if (exioOk) backlightSet(onBattery ? cfg.blBatt : cfg.blUsb);
  if (codecOk) { esW(0x32, dacVolReg()); esW(0x16, cfg.micGain & 7); }
}
static void kbDrawField() {
  lcdFill(4, 44, 312, 60, 0x1082); lcdRect(4, 44, 312, 60, C_GRAY);
  drawText(12, 48, "AX.25 callsign (A-Z 0-9, max 6)", C_GRAY, 0x1082);
  char t[10]; snprintf(t, sizeof(t), "%s_", kbBuf);
  drawTextf(12, 64, C_WHITE, 0x1082, 3, "%-7s", t);
}
// ---- date & time sub-screen
static int tmE[6];                                                 // year, month, day, hour, minute, second
static const char *TM_LBL[6] = {"YEAR", "MON", "DAY", "HOUR", "MIN", "SEC"};
static char tmStatus[2][40] = {"", ""};
static int  tmColX(int i) { return i == 0 ? 2 : 72 + (i - 1) * 49; }
static int  tmColW(int i) { return i == 0 ? 68 : 47; }
static Button tmPlus(int i)  { Button b = {(int16_t)tmColX(i), 112, (int16_t)tmColW(i), 40, "+"}; return b; }
static Button tmMinus(int i) { Button b = {(int16_t)tmColX(i), 192, (int16_t)tmColW(i), 40, "-"}; return b; }
static const Button bTmSet = {4, 240, 312, 44, "SET RTC"}, bTmNet = {4, 344, 152, 48, "NETWORK"}, bTmSync = {164, 344, 152, 48, "SYNC NOW"};
static int daysIn(int y, int m) { static const uint8_t d[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; return (m == 2 && ((y % 4 == 0 && y % 100) || y % 400 == 0)) ? 29 : d[(m - 1) % 12]; }
static void tmLoad() {
  RtcTime t;
  if (rtcOk && rtcRead(t)) { tmE[0] = t.year; tmE[1] = t.mon; tmE[2] = t.day; tmE[3] = t.hour; tmE[4] = t.min; tmE[5] = t.sec; }
  else { tmE[0] = 2026; tmE[1] = 1; tmE[2] = 1; tmE[3] = tmE[4] = tmE[5] = 0; }
}
static void tmDrawValues() {
  for (int i = 0; i < 6; i++) {
    char v[6]; snprintf(v, sizeof(v), i == 0 ? "%04d" : "%02d", tmE[i]);
    lcdFill(tmColX(i), 158, tmColW(i), 28, C_BG);
    drawText(tmColX(i) + (tmColW(i) - (int)strlen(v) * 16) / 2, 160, v, C_YELLOW, C_BG, 2);
  }
}
static void tmDrawStatus() { for (int i = 0; i < 2; i++) drawTextf(4, 400 + i * 15, i ? C_GRAY : C_WHITE, C_BG, 1, "%-38.38s", tmStatus[i]); }
static void tmStatic() {
  section(44, "DATE & TIME  (PCF85063A, local time)");
  for (int i = 0; i < 6; i++) {
    drawText(tmColX(i) + (tmColW(i) - (int)strlen(TM_LBL[i]) * FONT_W) / 2, 96, TM_LBL[i], C_GRAY);
    drawButton(tmPlus(i), false, C_CYAN); drawButton(tmMinus(i), false, C_CYAN);
  }
  tmDrawValues();
  drawButton(bTmSet, false, C_ORANGE);
  section(296, "NTP SYNC  (Wi-Fi)");
  drawTextf(4, 312, C_WHITE, C_BG, 1, "Network: %-28.28s", cfg.wifiSsid[0] ? cfg.wifiSsid : "(not set)");
  drawTextf(4, 326, C_GRAY, C_BG, 1, "UTC%+d  %s", cfg.tz, NTP_SERVER1);
  drawButton(bTmNet, false, C_CYAN); drawButton(bTmSync, false, C_GREEN);
  tmDrawStatus();
}
static bool ntpSync() {
  if (!cfg.wifiSsid[0]) { snprintf(tmStatus[0], 40, "Select a network first"); return false; }
  if (ieeeOn) ieeeStop();
  WiFi.mode(WIFI_STA); WiFi.setBandMode(WIFI_BAND_MODE_AUTO);
  WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
  snprintf(tmStatus[0], 40, "Connecting to %.24s", cfg.wifiSsid); tmStatus[1][0] = 0; tmDrawStatus();
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); pmActivity(); snprintf(tmStatus[1], 40, "%lu s", (unsigned long)((millis() - t0) / 1000)); tmDrawStatus(); }
  if (WiFi.status() != WL_CONNECTED) {
    snprintf(tmStatus[0], 40, "Wi-Fi failed (status %d)", (int)WiFi.status()); snprintf(tmStatus[1], 40, "Check password / band / antenna");
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); return false;
  }
  snprintf(tmStatus[0], 40, "Connected, IP %s", WiFi.localIP().toString().c_str()); snprintf(tmStatus[1], 40, "Querying NTP..."); tmDrawStatus();
  configTime((long)cfg.tz * 3600L, 0, NTP_SERVER1, NTP_SERVER2);
  struct tm tm;
  const bool ok = getLocalTime(&tm, 10000);
  if (ok) {
    rtcWrite(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    snprintf(tmStatus[0], 40, "Synced %04d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    snprintf(tmStatus[1], 40, "RTC updated (UTC%+d)", cfg.tz);
  } else { snprintf(tmStatus[0], 40, "NTP timeout"); snprintf(tmStatus[1], 40, "Internet access required"); }
  WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  Serial.printf("NTP: %s | %s\n", tmStatus[0], tmStatus[1]);
  return ok;
}
// ---- network list + QWERTY password keyboard
static const int SN_Y = 60, SN_H = 26, SN_ROWS = 12;
static const Button bSnScan = {4, 410, 312, 60, "SCAN"};
static void stNetDraw() {
  lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
  section(40, "SELECT Wi-Fi NETWORK FOR NTP");
  if (wifiScanning) drawText(8, 64, "Scanning 2.4 + 5 GHz...", C_YELLOW);
  else if (!apCount) drawText(8, 64, "No networks: press SCAN", C_GRAY);
  for (int i = 0; i < SN_ROWS && i < apCount && !wifiScanning; i++) {
    const ApInfo &a = aps[i]; const int y = SN_Y + i * SN_H; const uint16_t bg = (i & 1) ? C_BG : C_DGRAY;
    lcdFill(4, y, 312, SN_H - 2, bg);
    drawTextf(10, y + 6, a.ssid[0] ? C_WHITE : C_GRAY, bg, 1, "%-22.22s", a.ssid[0] ? a.ssid : "<hidden>");
    drawTextf(194, y + 6, a.auth == WIFI_AUTH_OPEN ? C_RED : C_GRAY, bg, 1, "%-6s", authStr(a.auth));
    drawTextf(250, y + 6, rssiColor(a.rssi), bg, 1, "%s%4d", a.ch > 14 ? "5G" : "2G", a.rssi);
  }
  drawButton(bSnScan, wifiScanning, C_YELLOW);
}
static uint8_t kbLayer = 0;
static bool    kbShow = false;
static const char *QW[3][4] = {{"1234567890", "qwertyuiop", "asdfghjkl-", "zxcvbnm.,/"},
                               {"!@#$%^&*()", "QWERTYUIOP", "ASDFGHJKL_", "ZXCVBNM:;?"},
                               {"~`|\\{}[]<>", "+=\"'_-@#$%", "&*()!?.,:;", "^/0123456 "}};
static char qwLab[4][10][2];
static Button qwKey(int r, int c) { Button b = {(int16_t)(1 + c * 32), (int16_t)(112 + r * 50), 30, 46, qwLab[r][c]}; return b; }
static const Button bQShift = {1, 312, 62, 46, "SHIFT"}, bQSym = {65, 312, 62, 46, "SYM"}, bQSpace = {129, 312, 126, 46, "SPACE"}, bQDel = {257, 312, 62, 46, "DEL"};
static const Button bQCancel = {1, 366, 104, 56, "CANCEL"}, bQShow = {108, 366, 104, 56, "SHOW"}, bQOk = {215, 366, 104, 56, "OK"};
static char netPick[33] = "";
static void qwField() {
  lcdFill(4, 44, 312, 60, 0x1082); lcdRect(4, 44, 312, 60, C_GRAY);
  drawTextf(12, 48, C_GRAY, 0x1082, 1, "Password for %.24s", netPick);
  char t[20]; const size_t n = strlen(kbBuf), off = n > 17 ? n - 17 : 0;
  for (size_t i = 0; i < 18; i++) t[i] = (off + i < n) ? (kbShow ? kbBuf[off + i] : '*') : (off + i == n ? '_' : ' ');
  t[18] = 0;
  drawText(12, 66, t, C_WHITE, 0x1082, 2);
}
static void qwDraw() {
  for (int r = 0; r < 4; r++) for (int c = 0; c < 10; c++) { qwLab[r][c][0] = QW[kbLayer][r][c]; qwLab[r][c][1] = 0; drawButton(qwKey(r, c), false, C_CYAN); }
  drawButton(bQShift, kbLayer == 1, C_YELLOW, 8, 13); drawButton(bQSym, kbLayer == 2, C_YELLOW, 8, 13);
  drawButton(bQSpace, false, C_CYAN, 8, 13); drawButton(bQDel, false, C_ORANGE, 8, 13);
  drawButton(bQCancel, false, C_GRAY); drawButton(bQShow, kbShow, C_YELLOW); drawButton(bQOk, true, C_GREEN);
  qwField();
}
static void settingsStatic() {
  lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
  switch (stView) {
    case SV_CALL:
      for (int i = 0; i < 36; i++) { kbLab[i][0] = KB_KEYS[i]; kbLab[i][1] = 0; drawButton(kbKey(i), false, C_CYAN); }
      drawButton(bKbDel, false, C_ORANGE); drawButton(bKbCancel, false, C_GRAY); drawButton(bKbOk, true, C_GREEN);
      kbDrawField(); return;
    case SV_TIME: tmStatic(); return;
    case SV_NET:  stNetDraw(); return;
    case SV_PASS: qwDraw(); return;
    default: break;
  }
  for (int i = 0; i < ST_ROWS; i++) stDrawRow(i);
  drawButton(bStDef, false, C_RED);
}
static void stAdjust(int i, int d) {
  switch (i) {
    case 0: cfg.blUsb = (uint8_t)constrain((int)cfg.blUsb + 10 * d, 10, 100); userBacklight = cfg.blUsb; if (exioOk && !onBattery) backlightSet(cfg.blUsb); break;
    case 1: { int k = 0; while (k < 6 && BATT_BL_OPTS[k] < cfg.blBatt) k++; k = constrain(k + d, 0, 6); cfg.blBatt = BATT_BL_OPTS[k];
              if (exioOk && onBattery) backlightSet(cfg.blBatt); } break;
    case 2: cfg.sleepIdx = (uint8_t)((cfg.sleepIdx + d + 6) % 6); break;
    case 3: cfg.sleepMode = cfg.sleepMode ? 0 : 1; break;
    case 4: cfg.volDb = (int8_t)constrain((int)cfg.volDb + 3 * d, -30, 6); if (codecOk) esW(0x32, dacVolReg()); break;
    case 5: cfg.micGain = (uint8_t)constrain((int)cfg.micGain + d, 0, 7); if (codecOk) esW(0x16, cfg.micGain); break;
    case 7: cfg.ssid = (uint8_t)((cfg.ssid + d + 16) % 16); break;
    case 8: cfg.hmirror ^= 1; break;
    case 9: cfg.vflip ^= 1; break;
    case 11: cfg.tz = (int8_t)constrain((int)cfg.tz + d, -12, 14); break;
  }
  settingsSave();
  stDrawRow(i);
}
static void tmAdjust(int i, int d) {
  tmE[i] += d;
  if (i == 0) tmE[0] = constrain(tmE[0], 2024, 2099);
  if (i == 1) tmE[1] = tmE[1] < 1 ? 12 : tmE[1] > 12 ? 1 : tmE[1];
  if (i == 3) tmE[3] = (tmE[3] + 24) % 24;
  if (i == 4 || i == 5) tmE[i] = (tmE[i] + 60) % 60;
  const int dm = daysIn(tmE[0], tmE[1]);
  if (i == 2) tmE[2] = tmE[2] < 1 ? dm : tmE[2] > dm ? 1 : tmE[2];
  tmE[2] = min(tmE[2], dm);
  tmDrawValues();
}
static void settingsTouch(uint16_t x, uint16_t y) {
  switch (stView) {
    case SV_CALL:
      for (int i = 0; i < 36; i++) if (hit(kbKey(i), x, y)) { const size_t n = strlen(kbBuf); if (n < 6) { kbBuf[n] = KB_KEYS[i]; kbBuf[n + 1] = 0; } kbDrawField(); return; }
      if (hit(bKbDel, x, y)) { const size_t n = strlen(kbBuf); if (n) kbBuf[n - 1] = 0; kbDrawField(); }
      else if (hit(bKbCancel, x, y)) { stView = SV_MAIN; settingsStatic(); }
      else if (hit(bKbOk, x, y) && kbBuf[0]) { snprintf(cfg.call, sizeof(cfg.call), "%s", kbBuf); settingsSave(); stView = SV_MAIN; settingsStatic(); }
      return;
    case SV_TIME:
      for (int i = 0; i < 6; i++) { if (hit(tmPlus(i), x, y)) { tmAdjust(i, 1); return; } if (hit(tmMinus(i), x, y)) { tmAdjust(i, -1); return; } }
      if (hit(bTmSet, x, y)) {
        const bool ok = rtcOk && rtcWrite(tmE[0], tmE[1], tmE[2], tmE[3], tmE[4], tmE[5]);
        snprintf(tmStatus[0], 40, ok ? "RTC set manually" : "RTC write failed"); tmStatus[1][0] = 0; tmDrawStatus();
      } else if (hit(bTmNet, x, y)) { stView = SV_NET; if (!apCount) wifiStartScan(); settingsStatic(); }
      else if (hit(bTmSync, x, y)) { drawButton(bTmSync, true, C_GREEN); ntpSync(); tmLoad(); tmDrawValues(); drawButton(bTmSync, false, C_GREEN); tmDrawStatus(); drawClock(); }
      return;
    case SV_NET: {
      if (hit(bSnScan, x, y)) { wifiStartScan(); stNetDraw(); return; }
      if (wifiScanning) return;
      const int i = ((int)y - SN_Y) / SN_H;
      if (y >= SN_Y && i >= 0 && i < SN_ROWS && i < apCount && aps[i].ssid[0]) {
        snprintf(netPick, sizeof(netPick), "%s", aps[i].ssid);
        if (aps[i].auth == WIFI_AUTH_OPEN) { snprintf(cfg.wifiSsid, sizeof(cfg.wifiSsid), "%s", netPick); cfg.wifiPass[0] = 0; settingsSave(); stView = SV_TIME; settingsStatic(); }
        else { kbBuf[0] = 0; if (!strcmp(netPick, cfg.wifiSsid)) snprintf(kbBuf, sizeof(kbBuf), "%s", cfg.wifiPass); kbLayer = 0; kbShow = false; stView = SV_PASS; settingsStatic(); }
      }
      return; }
    case SV_PASS: {
      for (int r = 0; r < 4; r++) for (int c = 0; c < 10; c++) if (hit(qwKey(r, c), x, y)) {
        const size_t n = strlen(kbBuf); const char ch = QW[kbLayer][r][c];
        if (n < 63 && ch != ' ') { kbBuf[n] = ch; kbBuf[n + 1] = 0; }
        if (kbLayer == 1) { kbLayer = 0; qwDraw(); } else qwField();
        return;
      }
      if (hit(bQShift, x, y)) { kbLayer = kbLayer == 1 ? 0 : 1; qwDraw(); }
      else if (hit(bQSym, x, y)) { kbLayer = kbLayer == 2 ? 0 : 2; qwDraw(); }
      else if (hit(bQSpace, x, y)) { const size_t n = strlen(kbBuf); if (n < 63) { kbBuf[n] = ' '; kbBuf[n + 1] = 0; } qwField(); }
      else if (hit(bQDel, x, y)) { const size_t n = strlen(kbBuf); if (n) kbBuf[n - 1] = 0; qwField(); }
      else if (hit(bQShow, x, y)) { kbShow = !kbShow; drawButton(bQShow, kbShow, C_YELLOW); qwField(); }
      else if (hit(bQCancel, x, y)) { stView = SV_NET; settingsStatic(); }
      else if (hit(bQOk, x, y)) {
        snprintf(cfg.wifiSsid, sizeof(cfg.wifiSsid), "%s", netPick); snprintf(cfg.wifiPass, sizeof(cfg.wifiPass), "%s", kbBuf);
        settingsSave(); snprintf(tmStatus[0], 40, "Network saved: %.22s", netPick); tmStatus[1][0] = 0;
        stView = SV_TIME; settingsStatic();
      }
      return; }
    default: break;
  }
  if (hit(bStDef, x, y)) { settingsDefaults(); settingsSave(); stApplyAll(); settingsStatic(); return; }
  for (int i = 0; i < ST_ROWS; i++) {
    if (i == 6)  { if (hit(stWide(i, ""), x, y)) { snprintf(kbBuf, sizeof(kbBuf), "%s", cfg.call); stView = SV_CALL; settingsStatic(); return; } continue; }
    if (i == 10) { if (hit(stWide(i, ""), x, y)) { tmLoad(); tmStatus[0][0] = tmStatus[1][0] = 0; stView = SV_TIME; settingsStatic(); return; } continue; }
    if (hit(stMinus(i), x, y)) { stAdjust(i, -1); return; }
    if (hit(stPlus(i), x, y))  { stAdjust(i, +1); return; }
  }
}

// ================================== APRS ======================================
static const int APRS_MAX = 40, AP_ROWS = 11, AP_Y = 58, AP_H = 28;
static AprsSta aprs[APRS_MAX];
static int     aprsN = 0, aprsSel = -1, aprsZoom = 0, kpField = 0;
static uint8_t aprsView = 0;                                       // 0 list, 1 map, 2 detail, 3 my-position keypad
static bool    aprsDirty = false;
static char    kpTxt[2][14];
static uint32_t aprsDrawT = 0;
static const Button bApMap = {4, 410, 100, 60, "MAP"}, bApPos = {110, 410, 100, 60, "MYPOS"}, bApBcn = {216, 410, 100, 60, "BEACON"};
static const Button bApList = {4, 410, 100, 60, "LIST"}, bApZin = {110, 410, 100, 60, "ZOOM+"}, bApZout = {216, 410, 100, 60, "ZOOM-"};
static const char *KP_LAB[16] = {"7", "8", "9", "DEL", "4", "5", "6", "+/-", "1", "2", "3", ".", "ESC", "0", "NEXT", "OK"};
static Button kpKey(int i) { Button b = {(int16_t)(4 + (i % 4) * 79), (int16_t)(150 + (i / 4) * 62), 75, 56, KP_LAB[i]}; return b; }

static int b91(const char *p, int n) { int v = 0; for (int i = 0; i < n; i++) { if (p[i] < 33 || p[i] > 124) return -1; v = v * 91 + (p[i] - 33); } return v; }
static bool aprsLat(const char *p, float &lat) {                   // "DDMM.hhN" (spaces = position ambiguity)
  char b[8]; for (int i = 0; i < 7; i++) b[i] = p[i] == ' ' ? '0' : p[i];
  if (!isdigit((uint8_t)b[0]) || !isdigit((uint8_t)b[1]) || !isdigit((uint8_t)b[2]) || !isdigit((uint8_t)b[3]) || b[4] != '.' || !isdigit((uint8_t)b[5]) || !isdigit((uint8_t)b[6])) return false;
  if (p[7] != 'N' && p[7] != 'S' && p[7] != 'n' && p[7] != 's') return false;
  lat = (b[0] - '0') * 10 + (b[1] - '0') + ((b[2] - '0') * 10 + (b[3] - '0') + ((b[5] - '0') * 10 + (b[6] - '0')) / 100.0f) / 60.0f;
  if (p[7] == 'S' || p[7] == 's') lat = -lat;
  return lat >= -90 && lat <= 90;
}
static bool aprsLon(const char *p, float &lon) {                   // "DDDMM.hhW"
  char b[9]; for (int i = 0; i < 8; i++) b[i] = p[i] == ' ' ? '0' : p[i];
  for (int i = 0; i < 8; i++) if (i != 5 && !isdigit((uint8_t)b[i])) return false;
  if (b[5] != '.' || (p[8] != 'E' && p[8] != 'W' && p[8] != 'e' && p[8] != 'w')) return false;
  lon = (b[0] - '0') * 100 + (b[1] - '0') * 10 + (b[2] - '0') + ((b[3] - '0') * 10 + (b[4] - '0') + ((b[6] - '0') * 10 + (b[7] - '0')) / 100.0f) / 60.0f;
  if (p[8] == 'W' || p[8] == 'w') lon = -lon;
  return lon >= -180 && lon <= 180;
}
static void aprsComment(AprsSta &s, const char *c, int n) {
  const char *a = nullptr;                                         // altitude "/A=nnnnnn" (feet)
  for (int i = 0; i + 9 <= n; i++) if (!strncmp(c + i, "/A=", 3)) { a = c + i + 3; break; }
  if (a) { long ft = strtol(a, nullptr, 10); s.alt = (int32_t)lrintf(ft * 0.3048f); s.hasAlt = true; }
  int k = 0;
  for (int i = 0; i < n && k < (int)sizeof(s.comment) - 1; i++) {
    if (a && c + i == a - 3) { i += 8; continue; }                 // drop the "/A=nnnnnn" token
    const char ch = c[i]; if (ch == '\r' || ch == '\n') break; s.comment[k++] = (ch >= 0x20 && ch < 0x7F) ? ch : '.'; }
  s.comment[k] = 0;
}
static bool aprsPos(const char *p, int n, AprsSta &s) {            // uncompressed or compressed position (+ extension + comment)
  if (n >= 19 && aprsLat(p, s.lat) && aprsLon(p + 9, s.lon)) {
    s.symT = p[8]; s.symC = p[18]; s.pos = true;
    const char *r = p + 19; int rn = n - 19;
    if (s.symC != '_' && rn >= 7 && isdigit((uint8_t)r[0]) && isdigit((uint8_t)r[1]) && isdigit((uint8_t)r[2]) && r[3] == '/' && isdigit((uint8_t)r[4]) && isdigit((uint8_t)r[5]) && isdigit((uint8_t)r[6])) {
      s.crs = (int16_t)atoi(r); s.spd = (int16_t)lrintf(atoi(r + 4) * 1.852f); s.hasCs = true; r += 7; rn -= 7;
    }
    aprsComment(s, r, rn);
    return true;
  }
  if (n >= 13) {                                                   // compressed: T YYYY XXXX $ cs t
    const int y = b91(p + 1, 4), x = b91(p + 5, 4);
    if (y < 0 || x < 0) return false;
    s.symT = p[0]; s.symC = p[9]; s.lat = 90.0f - y / 380926.0f; s.lon = -180.0f + x / 190463.0f; s.pos = true;
    if (p[10] != ' ' && p[10] >= '!' && p[10] <= 'z' && ((p[12] - 33) & 0x18) != 0x10) {
      s.crs = (int16_t)((p[10] - 33) * 4); s.spd = (int16_t)lrintf((powf(1.08f, (float)(p[11] - 33)) - 1.0f) * 1.852f); s.hasCs = true;
    }
    aprsComment(s, p + 13, n - 13);
    return s.lat >= -90 && s.lat <= 90 && s.lon >= -180 && s.lon <= 180;
  }
  return false;
}
static bool aprsMicE(const char *dst, const uint8_t *in, int n, AprsSta &s) {   // APRS101 ch. 10
  if (n < 9) return false;
  int dg[6];
  for (int i = 0; i < 6; i++) {
    const char c = dst[i];
    if (c >= '0' && c <= '9') dg[i] = c - '0'; else if (c >= 'A' && c <= 'J') dg[i] = c - 'A'; else if (c >= 'P' && c <= 'Y') dg[i] = c - 'P';
    else if (c == 'K' || c == 'L' || c == 'Z') dg[i] = 0; else return false;
  }
  float lat = dg[0] * 10 + dg[1] + (dg[2] * 10 + dg[3] + (dg[4] * 10 + dg[5]) / 100.0f) / 60.0f;
  if (!(dst[3] >= 'P' && dst[3] <= 'Z')) lat = -lat;              // P-Z = North
  int d = in[1] - 28; if (dst[4] >= 'P' && dst[4] <= 'Z') d += 100;
  if (d >= 180 && d <= 189) d -= 80; else if (d >= 190 && d <= 199) d -= 190;
  int m = in[2] - 28; if (m >= 60) m -= 60;
  const int h = in[3] - 28;
  float lon = d + (m + h / 100.0f) / 60.0f;
  if (dst[5] >= 'P' && dst[5] <= 'Z') lon = -lon;                 // P-Z = West
  const int sp = in[4] - 28, dc = in[5] - 28, se = in[6] - 28;
  int spd = sp * 10 + dc / 10, crs = (dc % 10) * 100 + se;
  if (spd >= 800) spd -= 800;
  if (crs >= 400) crs -= 400;
  s.lat = lat; s.lon = lon; s.pos = true; s.crs = (int16_t)crs; s.spd = (int16_t)lrintf(spd * 1.852f); s.hasCs = true;
  s.symC = (char)in[7]; s.symT = (char)in[8];
  const char *c = (const char *)in + 9; int cn = n - 9;
  if (cn >= 4 && c[3] == '}') { const int a = b91(c, 3); if (a >= 0) { s.alt = a - 10000; s.hasAlt = true; } c += 4; cn -= 4; }
  else if (cn >= 5 && c[4] == '}') { const int a = b91(c + 1, 3); if (a >= 0) { s.alt = a - 10000; s.hasAlt = true; } c += 5; cn -= 5; }
  aprsComment(s, c, cn);
  return lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
}
static bool aprsDecode(const char *src, const char *dst6, const uint8_t *info, int n, AprsSta &s) {   // pure: host-testable
  memset(&s, 0, sizeof(s)); snprintf(s.call, sizeof(s.call), "%s", src); s.kind = 'P';
  if (n < 1) return false;
  const char *p = (const char *)info; const char t = p[0];
  switch (t) {
    case '!': case '=': return aprsPos(p + 1, n - 1, s) || (aprsComment(s, p + 1, n - 1), true);
    case '/': case '@': if (n >= 8 && aprsPos(p + 8, n - 8, s)) return true; aprsComment(s, p + 1, n - 1); return true;
    case ';':                                                       // object
      if (n < 18) return false;
      { int k = 0; for (int i = 1; i <= 9 && k < 9; i++) if (p[i] != ' ') s.call[k++] = p[i]; s.call[k] = 0; }
      s.kind = 'O'; return aprsPos(p + 18, n - 18, s);
    case ')': {                                                     // item
      int e = 1; while (e < n && e < 11 && p[e] != '!' && p[e] != '_') e++;
      if (e >= n) return false;
      snprintf(s.call, sizeof(s.call), "%.*s", min(e - 1, 9), p + 1); s.kind = 'I'; return aprsPos(p + e + 1, n - e - 1, s); }
    case '`': case '\'': case 0x1C: case 0x1D: return aprsMicE(dst6, info, n, s);
    case ':':                                                       // message
      if (n >= 11 && p[10] == ':') {
        char to[10]; int k = 0; for (int i = 1; i <= 9; i++) if (p[i] != ' ') to[k++] = p[i]; to[k] = 0;
        s.kind = 'M'; char b[48]; snprintf(b, sizeof(b), "to %s: %.*s", to, n - 11, p + 11); aprsComment(s, b, (int)strlen(b)); return true;
      }
      return false;
    case '>': s.kind = 'S'; aprsComment(s, p + 1, n - 1); return true;
    default:  s.kind = '?'; aprsComment(s, p, n); return true;
  }
}
static void aprsDistBrg(float la1, float lo1, float la2, float lo2, float &km, float &brg) {
  const double r = 0.017453292519943295, p1 = la1 * r, p2 = la2 * r, dl = (lo2 - lo1) * r;
  const double a = sin((p2 - p1) / 2) * sin((p2 - p1) / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  km = (float)(6371.0 * 2 * atan2(sqrt(a), sqrt(1 - a)));
  brg = (float)fmod(atan2(sin(dl) * cos(p2), cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl)) / r + 360.0, 360.0);
}
static const char *compass(float b) { static const char *P[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"}; return P[((int)lrintf(b / 45.0f)) & 7]; }
static void aprsIngest(const Ax25Frame &f) {
  int na = 0;
  while (na * 7 + 6 < f.len && na < 10) { if (f.data[na * 7 + 6] & 1) { na++; break; } na++; }
  if (na < 2 || na * 7 + 2 > f.len) return;
  const int p = na * 7;
  if ((f.data[p] & 0xEF) != 0x03 || f.data[p + 1] != 0xF0) return;   // UI frame, no layer 3
  char src[12], dst6[7];
  ax25Call(f.data + 7, src);
  for (int i = 0; i < 6; i++) dst6[i] = (char)(f.data[i] >> 1);
  dst6[6] = 0;
  AprsSta s;
  if (!aprsDecode(src, dst6, f.data + p + 2, f.len - p - 2, s)) return;
  { char *q = s.path; int k = 0; for (int i = 2; i < na && k < 30; i++) { char c[12]; ax25Call(f.data + i * 7, c); k += snprintf(q + k, sizeof(s.path) - k, "%s%s%s", i > 2 ? "," : "", c, (f.data[i * 7 + 6] & 0x80) ? "*" : ""); } }
  int idx = -1, oldest = 0;
  for (int i = 0; i < aprsN; i++) { if (!strcmp(aprs[i].call, s.call)) idx = i; if (aprs[i].heard < aprs[oldest].heard) oldest = i; }
  if (idx < 0) { idx = aprsN < APRS_MAX ? aprsN++ : oldest; }
  else if (!s.pos && aprs[idx].pos) {                              // keep the last known position
    s.pos = true; s.lat = aprs[idx].lat; s.lon = aprs[idx].lon; s.symT = aprs[idx].symT; s.symC = aprs[idx].symC;
    if (!s.comment[0]) snprintf(s.comment, sizeof(s.comment), "%s", aprs[idx].comment);
  }
  s.pkts = (uint16_t)((idx < aprsN && !strcmp(aprs[idx].call, s.call) ? aprs[idx].pkts : 0) + 1);
  s.heard = millis();
  RtcTime t; if (rtcOk && rtcRead(t)) { s.hh = t.hour; s.mm = t.min; }
  aprs[idx] = s;
  aprsDirty = true;
  Serial.printf("APRS %c %s %s %.5f %.5f %s\n", s.kind, s.call, s.pos ? "pos" : "-", s.lat, s.lon, s.comment);
}
static void aprsSorted(int *ord) {                                 // most recently heard first
  for (int i = 0; i < aprsN; i++) ord[i] = i;
  std::sort(ord, ord + aprsN, [](int a, int b) { return aprs[a].heard > aprs[b].heard; });
}
static void aprsList() {
  lcdFill(0, 37, LCD_W, 372, C_BG);
  char t[48]; snprintf(t, sizeof(t), "APRS  %d station%s  (mic, 1200 Bd)", aprsN, aprsN == 1 ? "" : "s"); section(40, t);
  if (!aprsN) { drawText(8, 64, "Waiting for APRS packets...", C_GRAY); drawText(8, 80, "Feed AFSK audio to the mic or use BEACON", C_GRAY); }
  int ord[APRS_MAX]; aprsSorted(ord);
  for (int r = 0; r < AP_ROWS && r < aprsN; r++) {
    const AprsSta &s = aprs[ord[r]]; const int y = AP_Y + r * AP_H; const uint16_t bg = (r & 1) ? C_BG : 0x10A2;
    lcdFill(0, y, LCD_W, AP_H, bg);
    int x = drawTextf(4, y + 1, s.kind == 'M' ? C_YELLOW : s.kind == 'O' || s.kind == 'I' ? C_ORANGE : C_GREEN, bg, 1, "%-9s ", s.call);
    if (s.pos && cfg.hasPos) { float km, b; aprsDistBrg(cfg.myLat, cfg.myLon, s.lat, s.lon, km, b);
      x = drawTextf(x, y + 1, C_WHITE, bg, 1, km < 100 ? "%5.1f km %-2s " : "%5.0f km %-2s ", km, compass(b)); }
    else x = drawTextf(x, y + 1, C_GRAY, bg, 1, "%-12s", s.pos ? "pos" : "no pos");
    drawTextf(x, y + 1, C_GRAY, bg, 1, "%02u:%02u x%u", s.hh, s.mm, s.pkts);
    drawTextf(4, y + 14, C_GRAY, bg, 1, "%-38.38s", s.comment);
  }
  drawTextf(4, 372, C_GRAY, C_BG, 1, cfg.hasPos ? "My pos %.4f %.4f" : "My pos not set (MYPOS)", cfg.myLat, cfg.myLon);
  drawButton(bApMap, false, C_CYAN); drawButton(bApPos, false, C_CYAN); drawButton(bApBcn, txActive, C_RED);
}
static void aprsDetail() {
  lcdFill(0, 37, LCD_W, 372, C_BG);
  if (aprsSel < 0 || aprsSel >= aprsN) { aprsView = 0; aprsList(); return; }
  const AprsSta &s = aprs[aprsSel];
  char t[40]; snprintf(t, sizeof(t), "STATION  %s", s.call); section(40, t);
  int y = 60;
  auto line = [&](uint16_t c, const char *fmt, auto... a) { drawTextf(8, y, c, C_BG, 1, fmt, a...); y += 16; };
  static const char *KIND[] = {"position", "object", "item", "message", "status", "other"};
  const int ki = s.kind == 'P' ? 0 : s.kind == 'O' ? 1 : s.kind == 'I' ? 2 : s.kind == 'M' ? 3 : s.kind == 'S' ? 4 : 5;
  line(C_WHITE, "Type    %s  symbol %c%c", KIND[ki], s.symT ? s.symT : ' ', s.symC ? s.symC : ' ');
  if (s.pos) {
    line(C_WHITE, "Lat     %+.5f", s.lat); line(C_WHITE, "Lon     %+.5f", s.lon);
    if (cfg.hasPos) { float km, b; aprsDistBrg(cfg.myLat, cfg.myLon, s.lat, s.lon, km, b); line(C_GREEN, "Dist    %.2f km  brg %.0f deg %s", km, b, compass(b)); }
  } else line(C_GRAY, "No position");
  if (s.hasCs) line(C_WHITE, "Course  %d deg  speed %d km/h", s.crs, s.spd);
  if (s.hasAlt) line(C_WHITE, "Alt     %ld m", (long)s.alt);
  line(C_WHITE, "Heard   %02u:%02u  packets %u", s.hh, s.mm, s.pkts);
  line(C_GRAY, "Path    %.30s", s.path[0] ? s.path : "direct");
  y += 8; drawText(8, y, "Comment:", C_CYAN); y += 16;
  const int n = (int)strlen(s.comment);
  for (int i = 0; i < n && y < 390; i += 38, y += 14) drawTextf(8, y, C_WHITE, C_BG, 1, "%.38s", s.comment + i);
  drawButton(bApList, false, C_CYAN); drawButton(bApMap, false, C_CYAN);
}
static void aprsMap() {
  lcdFill(0, 37, LCD_W, 372, C_BG);
  const int cx = 160, cy = 226, R = 160;
  float cLat = cfg.myLat, cLon = cfg.myLon;
  if (!cfg.hasPos) { int k = 0; cLat = cLon = 0; for (int i = 0; i < aprsN; i++) if (aprs[i].pos) { cLat += aprs[i].lat; cLon += aprs[i].lon; k++; } if (k) { cLat /= k; cLon /= k; } }
  float maxKm = 0.5f;
  for (int i = 0; i < aprsN; i++) if (aprs[i].pos) { float km, b; aprsDistBrg(cLat, cLon, aprs[i].lat, aprs[i].lon, km, b); maxKm = max(maxKm, km); }
  static const float steps[] = {0.5f, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000};
  int si = 0; while (si < 12 && steps[si] * 4 < maxKm) si++;
  si = constrain(si - aprsZoom, 0, 12);
  const float ring = steps[si], scale = R / (ring * 4);
  for (int k = 1; k <= 4; k++) { const int rr = (int)(ring * k * scale); for (int a = 0; a < 360; a += 2) { const float t = a * 0.0174533f; lcdFill(cx + (int)(rr * cosf(t)), cy + (int)(rr * sinf(t)), 1, 1, C_DGRAY); } }
  lcdFill(cx - R, cy, 2 * R, 1, 0x18E3); lcdFill(cx, cy - R + 8, 1, 2 * R - 16, 0x18E3);
  drawText(cx - 4, 60, "N", C_GRAY);
  for (int i = 0; i < aprsN; i++) {
    if (!aprs[i].pos) continue;
    float km, b; aprsDistBrg(cLat, cLon, aprs[i].lat, aprs[i].lon, km, b);
    const float t = (b - 90.0f) * 0.0174533f;
    const int x = cx + (int)(km * scale * cosf(t)), y = cy + (int)(km * scale * sinf(t));
    if (x < 2 || x > 316 || y < 60 || y > 392) continue;
    const uint16_t c = aprs[i].kind == 'O' || aprs[i].kind == 'I' ? C_ORANGE : C_GREEN;
    lcdFill(x - 3, y - 3, 7, 7, c);
    drawText(min(x + 5, 320 - (int)strlen(aprs[i].call) * FONT_W), max(60, y - 6), aprs[i].call, c);
  }
  if (cfg.hasPos) { lcdFill(cx - 6, cy, 13, 1, C_WHITE); lcdFill(cx, cy - 6, 1, 13, C_WHITE); }
  drawTextf(4, 40, C_CYAN, C_BG, 1, "MAP  ring %g km  center %s   ", ring, cfg.hasPos ? "my pos" : "centroid");
  drawButton(bApList, false, C_CYAN); drawButton(bApZin, false, C_CYAN); drawButton(bApZout, false, C_CYAN);
}
static void kpDraw() {
  for (int f = 0; f < 2; f++) {
    const int y = 48 + f * 46; const bool act = kpField == f;
    lcdFill(4, y, 312, 40, act ? 0x1082 : C_BG); lcdRect(4, y, 312, 40, act ? C_YELLOW : C_GRAY);
    drawText(12, y + 8, f ? "LON" : "LAT", C_GRAY, act ? 0x1082 : C_BG, 2);
    drawTextf(76, y + 8, C_WHITE, act ? 0x1082 : C_BG, 2, "%-13s", kpTxt[f]);
  }
}
static void aprsKeypad() {
  lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
  for (int i = 0; i < 16; i++) drawButton(kpKey(i), i == 15, i == 15 ? C_GREEN : C_CYAN, 12, 20);
  drawText(4, 404, "Decimal degrees: lat -90..90, lon -180..180", C_GRAY);
  drawText(4, 420, "South and West are negative (+/- key)", C_GRAY);
  kpDraw();
}
static void aprsStatic() {
  switch (aprsView) { case 1: aprsMap(); break; case 2: aprsDetail(); break; case 3: aprsKeypad(); break; default: aprsList(); break; }
}
static void aprsBeacon() {
  if (txActive || !codecOk) return;
  if (!cfg.hasPos) { hdrMessage("APRS beacon:", "set MYPOS first", C_RED); return; }
  const long la = lrintf(fabsf(cfg.myLat) * 6000.0f), lo = lrintf(fabsf(cfg.myLon) * 6000.0f);   // hundredths of minute
  char info[80];
  snprintf(info, sizeof(info), "!%02ld%02ld.%02ld%c/%03ld%02ld.%02ld%c-Electgpl ESP32-C5 test", la / 6000, (la % 6000) / 100, la % 100, cfg.myLat < 0 ? 'S' : 'N',
           lo / 6000, (lo % 6000) / 100, lo % 100, cfg.myLon < 0 ? 'W' : 'E');
  if (!ax25BuildTx(info)) return;
  char line[90]; snprintf(line, sizeof(line), "TX %s-%u>%s:%s", cfg.call, cfg.ssid, AX25_DEST, info);
  termWrite(line, C_YELLOW);
  Serial.printf("APRS beacon: %s\n", info);
  txActive = true; startPlayback(txBuf, txCount);
  drawButton(bApBcn, true, C_RED);
}
static void aprsTouch(uint16_t x, uint16_t y) {
  switch (aprsView) {
    case 0: {
      if (hit(bApMap, x, y)) { aprsView = 1; aprsStatic(); return; }
      if (hit(bApPos, x, y)) {
        snprintf(kpTxt[0], sizeof(kpTxt[0]), cfg.hasPos ? "%.5f" : "", cfg.myLat); snprintf(kpTxt[1], sizeof(kpTxt[1]), cfg.hasPos ? "%.5f" : "", cfg.myLon);
        kpField = 0; aprsView = 3; aprsStatic(); return;
      }
      if (hit(bApBcn, x, y)) { aprsBeacon(); return; }
      const int r = ((int)y - AP_Y) / AP_H;
      if (y >= AP_Y && r >= 0 && r < AP_ROWS && r < aprsN) { int ord[APRS_MAX]; aprsSorted(ord); aprsSel = ord[r]; aprsView = 2; aprsStatic(); }
      return; }
    case 1:
      if (hit(bApList, x, y)) { aprsView = 0; aprsStatic(); }
      else if (hit(bApZin, x, y)) { aprsZoom = min(aprsZoom + 1, 6); aprsStatic(); }
      else if (hit(bApZout, x, y)) { aprsZoom = max(aprsZoom - 1, -6); aprsStatic(); }
      return;
    case 2:
      if (hit(bApList, x, y)) { aprsView = 0; aprsStatic(); }
      else if (hit(bApMap, x, y)) { aprsView = 1; aprsStatic(); }
      return;
    default: break;
  }
  if (y >= 48 && y < 134) { kpField = y >= 94 ? 1 : 0; kpDraw(); return; }   // tap a field to select it
  for (int i = 0; i < 16; i++) if (hit(kpKey(i), x, y)) {
    char *t = kpTxt[kpField]; const size_t n = strlen(t); const char *k = KP_LAB[i];
    if (isdigit((uint8_t)k[0]) && !k[1]) { if (n < 11) { t[n] = k[0]; t[n + 1] = 0; } }
    else if (!strcmp(k, ".")) { if (!strchr(t, '.') && n < 11) strcat(t, "."); }
    else if (!strcmp(k, "DEL")) { if (n) t[n - 1] = 0; }
    else if (!strcmp(k, "+/-")) { if (t[0] == '-') memmove(t, t + 1, n); else if (n < 12) { memmove(t + 1, t, n + 1); t[0] = '-'; } }
    else if (!strcmp(k, "NEXT")) kpField ^= 1;
    else if (!strcmp(k, "ESC")) { aprsView = 0; aprsStatic(); return; }
    else if (!strcmp(k, "OK")) {
      const float la = (float)atof(kpTxt[0]), lo = (float)atof(kpTxt[1]);
      if (kpTxt[0][0] && kpTxt[1][0] && fabsf(la) <= 90 && fabsf(lo) <= 180) {
        cfg.myLat = la; cfg.myLon = lo; cfg.hasPos = 1; settingsSave(); aprsView = 0; aprsStatic();
      } else hdrMessage("Invalid position", "check lat / lon", C_RED);
      return;
    }
    kpDraw(); return;
  }
}
static void aprsUpdate(uint32_t now) {
  if (aprsDirty && now - aprsDrawT > 500 && (aprsView == 0 || aprsView == 1)) { aprsDirty = false; aprsDrawT = now; aprsStatic(); }
}

// ================================== MAZE ======================================
// Screen-axis mapping of the IMU is learned once with a 3-step wizard (flat / top edge down /
// right edge down) and stored in NVS, so the game works whatever the QMI8658 mounting is.
static const int   MZ_C = 9, MZ_R = 12, MZ_S = 32, MZ_X = 16, MZ_Y = 58, MZ_BR = 8;
static const float MZ_G = 900.0f;                                  // px/s^2 per g of tilt
static const float MZ_DEAD = 0.015f;                               // g (~0.9 deg) dead zone
static const float MZ_FRICTION = 2.0f;                             // 1/s
static const float MZ_REST = 0.2f;                                 // wall restitution
static const float MZ_VMAX = 350.0f;                               // px/s
static const float MZ_ALPHA = 0.25f;                               // accelerometer EMA (~4.5 Hz at 100 Hz)
static uint8_t  mzWall[MZ_R][MZ_C];                                // bit0 N, bit1 E, bit2 S, bit3 W
static float    mzX, mzY, mzVx, mzVy, mzFx = 0, mzFy = 0;
static int      mzLevel = 1, mzDrawX = -100, mzDrawY = -100, mzWiz = 0;   // mzWiz: 0 play, 1..3 wizard steps
static uint32_t mzT0 = 0, mzLastT = 0, mzBest = 0, mzWinT = 0, mzHudT = 0, mzWizT = 0;
static bool     mzWon = false, mzReady = false;
static float    mzG0[3], mzG1[3];
static const Button bMzNew = {4, 446, 100, 30, "NEW"}, bMzLvl = {110, 446, 100, 30, "LEVEL"}, bMzSet = {216, 446, 100, 30, "SETUP"};
static const Button bMzNext = {4, 400, 152, 60, "NEXT"}, bMzCancel = {164, 400, 152, 60, "CANCEL"};
static const uint16_t MZ_WALLC = 0x2DFF, MZ_BGC = C_BLACK;

static void mzGenerate() {                                         // perfect maze, iterative DFS backtracker
  memset(mzWall, 0x0F, sizeof(mzWall));
  bool vis[MZ_R][MZ_C] = {};
  int stk[MZ_R * MZ_C], sp = 0;
  stk[sp++] = 0; vis[0][0] = true;
  static const int dx[4] = {0, 1, 0, -1}, dy[4] = {-1, 0, 1, 0};
  while (sp) {
    const int cur = stk[sp - 1], cx = cur % MZ_C, cy = cur / MZ_C;
    int opts[4], no = 0;
    for (int d = 0; d < 4; d++) { const int nx = cx + dx[d], ny = cy + dy[d]; if (nx >= 0 && nx < MZ_C && ny >= 0 && ny < MZ_R && !vis[ny][nx]) opts[no++] = d; }
    if (!no) { sp--; continue; }
    const int d = opts[esp_random() % (uint32_t)no], nx = cx + dx[d], ny = cy + dy[d];
    mzWall[cy][cx] &= (uint8_t)~(1 << d); mzWall[ny][nx] &= (uint8_t)~(1 << ((d + 2) & 3));
    vis[ny][nx] = true; stk[sp++] = ny * MZ_C + nx;
  }
}
static int mzWallRects(int c, int r, float rc[4][4]) {             // wall rectangles of one cell (x, y, w, h)
  int n = 0; const float x0 = MZ_X + c * MZ_S, y0 = MZ_Y + r * MZ_S;
  const uint8_t w = mzWall[r][c];
  if (w & 1) { rc[n][0] = x0 - 2; rc[n][1] = y0 - 2; rc[n][2] = MZ_S + 4; rc[n][3] = 4; n++; }
  if (w & 2) { rc[n][0] = x0 + MZ_S - 2; rc[n][1] = y0 - 2; rc[n][2] = 4; rc[n][3] = MZ_S + 4; n++; }
  if (w & 4) { rc[n][0] = x0 - 2; rc[n][1] = y0 + MZ_S - 2; rc[n][2] = MZ_S + 4; rc[n][3] = 4; n++; }
  if (w & 8) { rc[n][0] = x0 - 2; rc[n][1] = y0 - 2; rc[n][2] = 4; rc[n][3] = MZ_S + 4; n++; }
  return n;
}
static void mzCollide() {                                          // circle vs wall rectangles, 3x3 neighbourhood
  const int cc = (int)floorf((mzX - MZ_X) / MZ_S), cr = (int)floorf((mzY - MZ_Y) / MZ_S);
  for (int r = cr - 1; r <= cr + 1; r++) for (int c = cc - 1; c <= cc + 1; c++) {
    if (r < 0 || r >= MZ_R || c < 0 || c >= MZ_C) continue;
    float rc[4][4]; const int n = mzWallRects(c, r, rc);
    for (int k = 0; k < n; k++) {
      const float px = constrain(mzX, rc[k][0], rc[k][0] + rc[k][2]), py = constrain(mzY, rc[k][1], rc[k][1] + rc[k][3]);
      float dx = mzX - px, dy = mzY - py; const float d2 = dx * dx + dy * dy;
      if (d2 >= MZ_BR * MZ_BR) continue;
      float d = sqrtf(d2);
      if (d < 1e-3f) {
        const float l = mzX - rc[k][0], ri = rc[k][0] + rc[k][2] - mzX, t = mzY - rc[k][1], b = rc[k][1] + rc[k][3] - mzY;
        const float m = min(min(l, ri), min(t, b));
        dx = m == l ? -1.f : m == ri ? 1.f : 0.f; dy = m == t ? -1.f : m == b ? 1.f : 0.f;
        mzX += dx * (m + MZ_BR); mzY += dy * (m + MZ_BR);
      } else { dx /= d; dy /= d; mzX += dx * (MZ_BR - d); mzY += dy * (MZ_BR - d); }
      const float vn = mzVx * dx + mzVy * dy;
      if (vn < 0) { mzVx -= (1.0f + MZ_REST) * vn * dx; mzVy -= (1.0f + MZ_REST) * vn * dy; }
    }
  }
}
static void mzDisc(int cx, int cy, uint16_t c) {
  for (int dy = -MZ_BR; dy <= MZ_BR; dy++) { const int dx = (int)sqrtf((float)(MZ_BR * MZ_BR - dy * dy)); lcdFill(cx - dx, cy + dy, 2 * dx + 1, 1, c); }
}
static void mzButtons() { drawButton(bMzNew, false, C_CYAN, 8, 13); drawButton(bMzLvl, false, C_YELLOW, 8, 13); drawButton(bMzSet, !cfg.mzCal, C_ORANGE, 8, 13); }
static void mzDrawMaze() {
  lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
  lcdFill(MZ_X, MZ_Y, MZ_C * MZ_S, MZ_R * MZ_S, MZ_BGC);
  lcdFill(MZ_X + (MZ_C - 1) * MZ_S + 6, MZ_Y + (MZ_R - 1) * MZ_S + 6, MZ_S - 12, MZ_S - 12, 0x0320);   // goal
  for (int r = 0; r < MZ_R; r++) for (int c = 0; c < MZ_C; c++) {
    float rc[4][4]; const int n = mzWallRects(c, r, rc);
    for (int k = 0; k < n; k++) lcdFill((int)rc[k][0], (int)rc[k][1], (int)rc[k][2], (int)rc[k][3], MZ_WALLC);
  }
  mzDrawX = mzDrawY = -100;
  mzButtons();
  if (!cfg.mzCal) drawText(4, 40, "Run SETUP once to learn the IMU axes", C_ORANGE);
}
static void mzNew() {
  mzGenerate(); mzX = MZ_X + MZ_S / 2; mzY = MZ_Y + MZ_S / 2; mzVx = mzVy = 0; mzFx = mzFy = 0;
  mzWon = false; mzT0 = mzLastT = millis(); mzReady = true;
  mzDrawMaze();
}
static bool mzAvg(float g[3], int n = 50) {                        // average n accelerometer samples (~0.5 s)
  float s[3] = {0, 0, 0}; int k = 0;
  for (int i = 0; i < n; i++) { ImuData d; if (imuOk && imuRead(d)) { s[0] += d.ax; s[1] += d.ay; s[2] += d.az; k++; } delay(10); }
  if (!k) return false;
  for (int i = 0; i < 3; i++) g[i] = s[i] / k;
  return true;
}
static void mzWizDraw() {
  lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
  section(40, "MAZE SETUP  (IMU axis mapping)");
  static const char *T[4][3] = {{"", "", ""},
    {"Step 1/3: lay the board FLAT,", "screen facing up, then press NEXT.", "Keep it still for half a second."},
    {"Step 2/3: tilt the TOP edge of the", "screen DOWN (about 30 deg) and hold,", "then press NEXT."},
    {"Step 3/3: from flat, tilt the RIGHT", "edge of the screen DOWN (about 30 deg)", "and hold, then press NEXT."}};
  for (int i = 0; i < 3; i++) drawText(8, 70 + i * 16, T[mzWiz][i], C_WHITE);
  // arrow sketch of the requested tilt
  const int cx = 160, cy = 240;
  lcdRect(cx - 60, cy - 90, 120, 180, C_GRAY);
  if (mzWiz == 2) { for (int i = 0; i < 20; i++) lcdFill(cx - i, cy - 120 + i, 2 * i + 1, 1, C_YELLOW); drawText(cx - 20, cy - 145, "DOWN", C_YELLOW); }
  if (mzWiz == 3) { for (int i = 0; i < 20; i++) lcdFill(cx + 90 - i, cy - i, 1, 2 * i + 1, C_YELLOW); drawText(cx + 72, cy + 26, "DOWN", C_YELLOW); }
  drawButton(bMzNext, true, C_GREEN); drawButton(bMzCancel, false, C_GRAY);
}
static void mzWizLive(uint32_t now) {                              // live raw accelerometer readout
  if (now - mzWizT < 150) return;
  mzWizT = now;
  ImuData d;
  if (imuOk && imuRead(d)) drawTextf(8, 360, C_CYAN, C_BG, 1, "ax %+5.2f  ay %+5.2f  az %+5.2f g", d.ax, d.ay, d.az);
}
static void mzWizNext() {
  float g[3];
  drawText(8, 380, "Measuring...                 ", C_YELLOW);
  if (!mzAvg(g)) { drawText(8, 380, "IMU not responding          ", C_RED); return; }
  if (mzWiz == 1) { memcpy(mzG0, g, sizeof(g)); mzWiz = 2; mzWizDraw(); return; }
  const float dx = g[0] - mzG0[0], dy = g[1] - mzG0[1];
  if (mzWiz == 2) {
    const int ax = fabsf(dx) >= fabsf(dy) ? 0 : 1; const float dd = ax ? dy : dx;
    if (fabsf(dd) < 0.15f) { drawText(8, 380, "Tilt more (> 10 deg) and retry", C_RED); return; }
    memcpy(mzG1, g, sizeof(g)); mzWiz = 3; mzWizDraw(); return;
  }
  // step 3: Y axis from step 2, X axis from step 3; both must be distinct and significant
  const float d1x = mzG1[0] - mzG0[0], d1y = mzG1[1] - mzG0[1];
  const int axY = fabsf(d1x) >= fabsf(d1y) ? 0 : 1, axX = fabsf(dx) >= fabsf(dy) ? 0 : 1;
  const float dX = axX ? dy : dx, dY = axY ? d1y : d1x;
  if (axX == axY || fabsf(dX) < 0.15f) { drawText(8, 380, "Ambiguous axes: tilt only the RIGHT edge", C_RED); return; }
  cfg.mzAxX = (uint8_t)axX; cfg.mzSgX = dX > 0 ? 1 : -1;
  cfg.mzAxY = (uint8_t)axY; cfg.mzSgY = dY > 0 ? 1 : -1;
  cfg.mzOffX = mzG0[axX]; cfg.mzOffY = mzG0[axY]; cfg.mzCal = 1;
  settingsSave();
  Serial.printf("Maze IMU map: screenX = %c%c, screenY(up) = %c%c, offsets %.3f %.3f g\n", cfg.mzSgX > 0 ? '+' : '-', axX ? 'y' : 'x',
                cfg.mzSgY > 0 ? '+' : '-', axY ? 'y' : 'x', cfg.mzOffX, cfg.mzOffY);
  mzWiz = 0; mzFx = mzFy = 0; mzVx = mzVy = 0; mzDrawMaze();
  char l2[24]; snprintf(l2, sizeof(l2), "X=%c%c Y=%c%c", cfg.mzSgX > 0 ? '+' : '-', axX ? 'y' : 'x', cfg.mzSgY > 0 ? '+' : '-', axY ? 'y' : 'x');
  hdrMessage("Maze axes saved", l2, C_GREEN);
}
static void mazeStatic() {
  if (mzWiz) { mzWizDraw(); return; }
  if (!mzReady) mzNew(); else mzDrawMaze();
  mzLastT = millis();
}
static void mazeUpdate(uint32_t now) {
  if (mzWiz) { mzWizLive(now); return; }
  if (!mzReady) return;
  if (mzWon) { if (now - mzWinT > 2000) { mzLevel++; mzNew(); } return; }
  if (now - mzLastT < 10) return;
  const float dt = min(0.04f, (now - mzLastT) / 1000.0f); mzLastT = now;
  ImuData d;
  if (imuOk && imuRead(d)) {
    const float a[2] = {d.ax, d.ay};
    const float rx = cfg.mzSgX * (a[cfg.mzAxX & 1] - cfg.mzOffX);  // + = right edge down -> roll right
    const float ry = cfg.mzSgY * (a[cfg.mzAxY & 1] - cfg.mzOffY);  // + = top edge down   -> roll up
    mzFx += MZ_ALPHA * (rx - mzFx); mzFy += MZ_ALPHA * (ry - mzFy);
    auto dz = [](float v) { return fabsf(v) < MZ_DEAD ? 0.0f : v - copysignf(MZ_DEAD, v); };
    mzVx += dz(mzFx) * MZ_G * dt;
    mzVy -= dz(mzFy) * MZ_G * dt;
  }
  const float fr = max(0.0f, 1.0f - MZ_FRICTION * dt); mzVx *= fr; mzVy *= fr;
  const float v = sqrtf(mzVx * mzVx + mzVy * mzVy);
  if (v > MZ_VMAX) { mzVx *= MZ_VMAX / v; mzVy *= MZ_VMAX / v; }
  const int steps = max(1, (int)ceilf(min(v, MZ_VMAX) * dt / 2.0f));   // <= 2 px per sub-step
  for (int i = 0; i < steps; i++) { mzX += mzVx * dt / steps; mzY += mzVy * dt / steps; mzCollide(); }
  const int ix = (int)lrintf(mzX), iy = (int)lrintf(mzY);
  if (ix != mzDrawX || iy != mzDrawY) {
    if (mzDrawX > -100) mzDisc(mzDrawX, mzDrawY, MZ_BGC);
    if (mzDrawX > MZ_X + (MZ_C - 1) * MZ_S - MZ_BR && mzDrawY > MZ_Y + (MZ_R - 1) * MZ_S - MZ_BR)
      lcdFill(MZ_X + (MZ_C - 1) * MZ_S + 6, MZ_Y + (MZ_R - 1) * MZ_S + 6, MZ_S - 12, MZ_S - 12, 0x0320);
    mzDisc(ix, iy, C_RED); lcdFill(ix - 3, iy - 4, 3, 2, C_WHITE);
    mzDrawX = ix; mzDrawY = iy;
  }
  const uint32_t el = now - mzT0;
  if (now - mzHudT > 200 && cfg.mzCal) { mzHudT = now; drawTextf(4, 40, C_WHITE, C_BG, 1, "LEVEL %-3d  TIME %6.1f s  BEST %6.1f s ", mzLevel, el / 1000.0f, mzBest / 1000.0f); }
  if ((int)((mzX - MZ_X) / MZ_S) == MZ_C - 1 && (int)((mzY - MZ_Y) / MZ_S) == MZ_R - 1) {
    mzWon = true; mzWinT = now; if (!mzBest || el < mzBest) mzBest = el;
    drawTextf(4, 40, C_GREEN, C_BG, 1, "SOLVED in %.1f s!  next maze...         ", el / 1000.0f);
    Serial.printf("Maze level %d solved in %.1f s\n", mzLevel, el / 1000.0f);
  }
}
static void mazeTouch(uint16_t x, uint16_t y) {
  if (mzWiz) {
    if (hit(bMzNext, x, y)) mzWizNext();
    else if (hit(bMzCancel, x, y)) { mzWiz = 0; mzDrawMaze(); }
    return;
  }
  if (hit(bMzNew, x, y)) mzNew();
  else if (hit(bMzSet, x, y)) { mzWiz = 1; mzWizDraw(); }
  else if (hit(bMzLvl, x, y)) {                                    // re-zero the level only (keeps axis mapping)
    float g[3];
    if (mzAvg(g, 30)) { cfg.mzOffX = g[cfg.mzAxX & 1]; cfg.mzOffY = g[cfg.mzAxY & 1]; settingsSave(); mzFx = mzFy = 0; hdrMessage("Maze level set", "current tilt = 0", C_GREEN); }
  }
}

// ===================== shared big TX buffer (SSTV / RTTY waveform) ==================
static int16_t *bigTx = nullptr;
static const uint32_t BIG_TX_MAX = 640000;                         // 40 s at 16 kHz (1.28 MB PSRAM)
static uint32_t bigN = 0, bigPh = 0;
static double   bigT = 0;
static const float BIG_AMPL = 0.35f;
static bool bigAlloc() { if (!bigTx && psramFound()) bigTx = (int16_t *)heap_caps_malloc(BIG_TX_MAX * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); return bigTx != nullptr; }
static void txReset() { bigN = 0; bigT = 0; bigPh = 0; }
static void txTone(float f, double ms) {                           // continuous phase, fractional-sample timing
  bigT += ms * 16.0;
  const uint32_t inc = phaseIncFor(f), end = (uint32_t)llround(bigT);
  while (bigN < end && bigN < BIG_TX_MAX) { bigTx[bigN++] = (int16_t)(loTab[bigPh >> 24] * (2.0f * BIG_AMPL)); bigPh += inc; }
}

// =================================== SSTV ======================================
static const SstvMode SSTV_MODES[4] = {                            // offsets in ms relative to the sync start of a line
  {"Robot 36", 8, 0, 240, 150.0f, 9.0f, 0.0f},
  {"Martin M1", 44, 1, 256, 446.446f, 4.862f, 0.0f},
  {"Scottie S1", 60, 2, 256, 428.22f, 9.0f, 288.48f},
  {"PD 120", 95, 3, 248, 508.48f, 20.0f, 0.0f}};
static const int SS_IY = 58;                                       // image top on screen
static uint16_t *ssImg = nullptr, *ssTxImg = nullptr;              // 320x256 BE (RX) / 320x240 native RGB565 (TX)
static uint8_t  ssState = 0;                                       // 0 waiting VIS, 1 decoding
static int      ssMode = -1, ssLine = 0, ssSyncHits = 0, ssBlkN = 0, ssDone = 0;
static uint32_t ssRd = 0, ssUiT = 0;
static float    ssBlk[48];
static double   ssNext = 0, ssPeriod = 0, ssPeriodNom = 0;
static uint8_t  ssYe[320], ssV[320], ssU[320];
static char     ssMsg[2][40] = {"", ""};
static const Button bSsClr = {4, 410, 76, 60, "CLEAR"}, bSsSave = {82, 410, 76, 60, "SAVE"}, bSsCam = {160, 410, 76, 60, "TXCAM"}, bSsBar = {238, 410, 76, 60, "TXBAR"};
static const Button bSsStop = {160, 410, 154, 60, "STOP TX"};
static int ssTxLine = -1;

static inline int ssF(int64_t i) { return ssRing[(uint32_t)i & (SS_RING - 1)]; }
static int ssAvg(double a, double b) { int64_t ia = (int64_t)a, ib = (int64_t)b; if (ib <= ia) ib = ia + 1; int64_t s = 0; for (int64_t i = ia; i < ib; i++) s += ssF(i); return (int)(s / (ib - ia)); }
static inline uint8_t ssLum(int f) { const int v = (f - 1500) * 255 / 800; return (uint8_t)constrain(v, 0, 255); }
static void ssChan(double start, double durMs, int n, uint8_t *out) {
  const double len = durMs * 16.0;
  for (int i = 0; i < n; i++) out[i] = ssLum(ssAvg(start + i * len / n, start + (i + 1) * len / n));
}
static inline uint16_t rgb565be(int r, int g, int b) {
  r = constrain(r, 0, 255); g = constrain(g, 0, 255); b = constrain(b, 0, 255);
  return be16((uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)));
}
static inline uint16_t yuvBe(int y, int u, int v) {                // full-range YCbCr (JPEG / PIL / slowrx convention)
  return rgb565be(y + ((359 * (v - 128)) >> 8), y - ((88 * (u - 128) + 183 * (v - 128)) >> 8), y + ((454 * (u - 128)) >> 8));
}
static void ssRowDone(int row) {
  if (row < 0 || row >= 256) return;
  if (page == PG_SSTV) lcdBlit(0, SS_IY + row, LCD_W, 1, (const uint8_t *)(ssImg + row * LCD_W));
}
static void ssStatus() {
  if (page != PG_SSTV) return;
  if (ssState == 1) {
    const SstvMode &m = SSTV_MODES[ssMode];
    drawTextf(4, 334, C_GREEN, C_BG, 1, "%-10s line %3d/%3d  sync %3d%%   ", m.name, ssLine, m.lines, ssLine ? ssSyncHits * 100 / ssLine : 0);
  } else drawTextf(4, 334, txActive ? C_RED : C_YELLOW, C_BG, 1, "%-38s", txActive ? "Transmitting Robot 36..." : "Waiting for VIS (R36 M1 S1 PD120)");
  drawTextf(4, 350, C_WHITE, C_BG, 1, "%-38.38s", ssMsg[0]);
  drawTextf(4, 366, C_GRAY, C_BG, 1, "%-38.38s", ssMsg[1]);
}
static int ssFindMode(int vis) { for (int i = 0; i < 4; i++) if (SSTV_MODES[i].vis == vis) return i; return -1; }
static void ssVisScan() {                                          // 10 ms block averages, VIS pattern on the newest blocks
  while ((int32_t)(ssW - ssRd) >= 160 && ssState == 0) {
    const float f = (float)ssAvg((double)ssRd, (double)ssRd + 160);
    ssRd += 160;
    memmove(ssBlk, ssBlk + 1, sizeof(float) * 47); ssBlk[47] = f; if (ssBlkN < 48) ssBlkN++;
    if (ssBlkN < 48) continue;
    auto b = [](int k) { return ssBlk[47 - k]; };                  // k = 0 newest
    auto near = [](float v, float c, float t) { return fabsf(v - c) < t; };
    if (!near(b(1), 1200, 80) || !near(b(28), 1200, 80)) continue;             // stop and start bits
    float lead = 0; for (int k = 31; k <= 42; k++) lead += b(k); lead /= 12;
    if (!near(lead, 1900, 70)) continue;
    int vis = 0, ones = 0; bool ok = true;
    for (int j = 0; j < 8 && ok; j++) {
      const float v = b(4 + 3 * (7 - j));                         // bit centre block
      if (near(v, 1100, 80)) { if (j < 7) vis |= 1 << j; ones++; } else if (!near(v, 1300, 80)) ok = false;
    }
    if (!ok || (ones & 1)) continue;                               // even parity over 7 bits + parity
    const int m = ssFindMode(vis);
    if (m < 0) { snprintf(ssMsg[0], 40, "Unsupported VIS %d", vis); ssStatus(); continue; }
    ssMode = m; ssState = 1; ssLine = 0; ssSyncHits = 0;
    ssPeriodNom = ssPeriod = SSTV_MODES[m].lineMs * 16.0;
    ssNext = (double)ssRd + SSTV_MODES[m].firstSyncMs * 16.0;
    for (int i = 0; i < 256 * LCD_W; i++) ssImg[i] = 0;
    if (page == PG_SSTV) lcdFill(0, SS_IY, LCD_W, 256, C_BLACK);
    snprintf(ssMsg[0], 40, "VIS %d: %s", vis, SSTV_MODES[m].name);
    Serial.printf("SSTV VIS %d -> %s\n", vis, SSTV_MODES[m].name);
    ssStatus();
  }
}
static double ssSync(double exp, double win, double syncLen, bool &ok) {
  // Anchor on the END of the 1200 Hz pulse (sync -> porch edge): unique even when the sync is glued to
  // the VIS stop bit (Robot 36, Martin, PD). Score = sync samples in [e-L, e) + non-sync samples in [e, e+P).
  const int64_t L = (int64_t)syncLen, P = 8, e0 = (int64_t)(exp + syncLen - win), n = (int64_t)(2 * win);
  int a = 0, b = 0;
  for (int64_t i = e0 - L; i < e0; i++) a += ssF(i) < 1350;
  for (int64_t i = e0; i < e0 + P; i++) b += ssF(i) >= 1350;
  int64_t best = -1, bi = 0; int ba = 0, bb = 0;
  for (int64_t c = 0; c <= n; c++) {
    const int64_t e = e0 + c;
    if (c) { a += (ssF(e - 1) < 1350) - (ssF(e - 1 - L) < 1350); b += (ssF(e + P - 1) >= 1350) - (ssF(e - 1) >= 1350); }
    const int64_t sc = (int64_t)a * P + (int64_t)b * L;
    if (sc > best) { best = sc; bi = c; ba = a; bb = b; }
  }
  ok = ba >= (int)(0.6 * L) && bb >= (int)(0.6 * P);
  return ok ? (double)(e0 + bi - L) : exp;
}
static void ssDecodeLine() {
  const SstvMode &m = SSTV_MODES[ssMode];
  const double win = (ssLine == 0 ? 40.0 : 12.0) * 16.0;
  bool ok; const double st = ssSync(ssNext, win, m.syncMs * 16.0, ok);
  if (ok) {
    ssSyncHits++;
    if (ssLine) ssPeriod = constrain(ssPeriod + 0.05 * (st - ssNext), ssPeriodNom * 0.997, ssPeriodNom * 1.003);   // slant tracking
  }
  const double S = 16.0;
  static uint8_t a[320], b2[320], c[320], d[320];
  uint16_t *row;
  switch (m.kind) {
    case 0: {                                                      // Robot 36: Y + alternating R-Y / B-Y
      ssChan(st + 12.0 * S, 88.0, 320, a);
      const int sep = ssAvg(st + 100.0 * S, st + 104.5 * S);
      { static uint8_t h[160]; ssChan(st + 106.0 * S, 44.0, 160, h);   // chroma at 160 px (2.2 samples/px at 320 is beyond the FM demod bandwidth)
        for (int x = 0; x < 320; x++) { const int i0 = constrain((x - 1) / 2, 0, 159), i1 = min(i0 + 1, 159); c[x] = (x & 1) ? (uint8_t)((h[i0] + h[i1] + 1) >> 1) : h[min(x / 2, 159)]; } }
      if (sep < 1900) { memcpy(ssYe, a, 320); memcpy(ssV, c, 320); row = ssImg + ssLine * LCD_W; for (int x = 0; x < 320; x++) row[x] = yuvBe(a[x], ssU[x], c[x]); ssRowDone(ssLine); }
      else {
        memcpy(ssU, c, 320);
        if (ssLine > 0) { row = ssImg + (ssLine - 1) * LCD_W; for (int x = 0; x < 320; x++) row[x] = yuvBe(ssYe[x], c[x], ssV[x]); ssRowDone(ssLine - 1); }
        row = ssImg + ssLine * LCD_W; for (int x = 0; x < 320; x++) row[x] = yuvBe(a[x], c[x], ssV[x]); ssRowDone(ssLine);
      }
      break; }
    case 1:                                                        // Martin M1: G B R
      ssChan(st + 5.434 * S, 146.432, 320, a); ssChan(st + 152.438 * S, 146.432, 320, b2); ssChan(st + 299.442 * S, 146.432, 320, c);
      row = ssImg + ssLine * LCD_W; for (int x = 0; x < 320; x++) row[x] = rgb565be(c[x], a[x], b2[x]); ssRowDone(ssLine);
      break;
    case 2:                                                        // Scottie S1: G B (before sync) R (after sync)
      ssChan(st - 279.48 * S, 138.24, 320, a); ssChan(st - 139.74 * S, 138.24, 320, b2); ssChan(st + 10.5 * S, 138.24, 320, c);
      row = ssImg + ssLine * LCD_W; for (int x = 0; x < 320; x++) row[x] = rgb565be(c[x], a[x], b2[x]); ssRowDone(ssLine);
      break;
    default:                                                       // PD 120: Y0 R-Y B-Y Y1 (640 px -> 320, line pair -> 1 row)
      ssChan(st + 22.08 * S, 121.6, 320, a); ssChan(st + 143.68 * S, 121.6, 320, c); ssChan(st + 265.28 * S, 121.6, 320, b2); ssChan(st + 386.88 * S, 121.6, 320, d);
      row = ssImg + ssLine * LCD_W; for (int x = 0; x < 320; x++) row[x] = yuvBe((a[x] + d[x] + 1) >> 1, b2[x], c[x]); ssRowDone(ssLine);
      break;
  }
  ssNext = st + ssPeriod;
  if (++ssLine >= m.lines) {
    ssState = 0; ssBlkN = 0; ssDone++;
    RtcTime t; char ts[8] = "";
    if (rtcOk && rtcRead(t)) snprintf(ts, sizeof(ts), "%02u:%02u", t.hour, t.min);
    snprintf(ssMsg[1], 40, "Received %s %s (sync %d%%)", m.name, ts, ssSyncHits * 100 / m.lines);
    Serial.printf("SSTV %s done, sync %d/%d\n", m.name, ssSyncHits, m.lines);
  }
}
static void sstvProcess() {                                        // called from loop(); pure w.r.t. hardware except ssRowDone()
  if (ssState == 0) { ssVisScan(); return; }
  const SstvMode &m = SSTV_MODES[ssMode];
  for (int guard = 0; guard < 4; guard++) {
    const double need = ssNext + ssPeriod + 45.0 * 16.0 + (m.kind == 2 ? 0 : 0);
    if ((double)ssW < need) break;
    ssDecodeLine();
    if (ssState == 0) break;
  }
}
// ---- TX: Robot 36 encoder
static void sstvEncodeR36(const uint16_t *img) {                   // img: 320x240 native RGB565
  txReset();
  txTone(1900, 300); txTone(1200, 10); txTone(1900, 300); txTone(1200, 30);
  const int vis = 8; int ones = 0;
  for (int b = 0; b < 7; b++) { const int bit = (vis >> b) & 1; ones += bit; txTone(bit ? 1100 : 1300, 30); }
  txTone((ones & 1) ? 1100 : 1300, 30); txTone(1200, 30);
  static uint8_t Y[2][320], U[320], V[320];
  for (int y = 0; y < 240; y += 2) {
    for (int k = 0; k < 2; k++) for (int x = 0; x < 320; x++) {
      const uint16_t p = img[(y + k) * 320 + x];
      const int r = ((p >> 11) & 31) * 255 / 31, g = ((p >> 5) & 63) * 255 / 63, b = (p & 31) * 255 / 31;
      Y[k][x] = (uint8_t)constrain((77 * r + 150 * g + 29 * b + 128) >> 8, 0, 255);
      const int u = 128 + ((-43 * r - 85 * g + 128 * b) >> 8), v = 128 + ((128 * r - 107 * g - 21 * b) >> 8);
      if (k == 0) { U[x] = (uint8_t)constrain(u, 0, 255); V[x] = (uint8_t)constrain(v, 0, 255); }
      else { U[x] = (uint8_t)((U[x] + constrain(u, 0, 255) + 1) >> 1); V[x] = (uint8_t)((V[x] + constrain(v, 0, 255) + 1) >> 1); }
    }
    for (int k = 0; k < 2; k++) {
      txTone(1200, 9); txTone(1500, 3);
      for (int x = 0; x < 320; x++) txTone(1500.0f + Y[k][x] * (800.0f / 255.0f), 88.0 / 320);
      txTone(k ? 2300 : 1500, 4.5); txTone(1900, 1.5);
      for (int x = 0; x < 320; x++) txTone(1500.0f + (k ? U[x] : V[x]) * (800.0f / 255.0f), 44.0 / 320);
    }
  }
}
static void imgText(uint16_t *img, int W, int H, int x, int y, const char *s, uint16_t col, int sc) {
  for (; *s; s++, x += FONT_W * sc) {
    const uint8_t ch = (uint8_t)*s; if (ch < FONT_FIRST || ch > FONT_LAST) continue;
    const uint8_t *g = font8x13[ch - FONT_FIRST];
    for (int r = 0; r < FONT_H * sc; r++) for (int c = 0; c < FONT_W * sc; c++)
      if (g[r / sc] & (0x80 >> (c / sc))) { const int px = x + c, py = y + r; if (px >= 0 && px < W && py >= 0 && py < H) img[py * W + px] = col; }
  }
}
static void sstvTestImage(uint16_t *img) {
  static const uint16_t bars[8] = {0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000};
  for (int y = 0; y < 240; y++) for (int x = 0; x < 320; x++) {
    uint16_t c;
    if (y < 150) c = bars[x / 40];
    else if (y < 190) { const int v = x * 255 / 319; c = (uint16_t)(((v & 0xF8) << 8) | ((v & 0xFC) << 3) | (v >> 3)); }
    else c = 0x0841;
    img[y * 320 + x] = c;
  }
  imgText(img, 320, 240, 8, 196, "E", 0xF800, 3); imgText(img, 320, 240, 32, 196, "lectgpl", 0xFFFF, 3);
  char t[24]; snprintf(t, sizeof(t), "DE %s", cfg.call); imgText(img, 320, 240, 220, 200, t, 0xFFE0, 1);
  imgText(img, 320, 240, 220, 216, "ESP32-C5 R36", 0x07FF, 1);
}
static bool camGrab(uint16_t *dst) {                               // one validated frame -> 240x320 native RGB565
  if (!camInited) return false;
  const bool wasRunning = camRunning;
  if (!wasRunning) camStart();
  int pick = -1; const uint32_t t0 = millis();
  while (millis() - t0 < 2500 && pick < 0) {
    taskENTER_CRITICAL(&camMux);
    for (int i = 0; i < CAM_NBUF; i++) if (camSt[i] == CB_READY) { pick = i; camSt[i] = CB_DRAW; break; }
    taskEXIT_CRITICAL(&camMux);
    if (pick < 0) { delay(10); continue; }
    const uint8_t *f = camBuf[pick]; bool valid = memcmp(f, CAM_FH, 4) == 0;
    for (int i = 0; i < CAM_H && valid; i++) valid = memcmp(f + CAM_FHDR + i * CAM_LINE, CAM_LH, 4) == 0;
    if (valid) for (int y = 0; y < CAM_H; y++) { const uint8_t *s = f + CAM_FHDR + y * CAM_LINE + CAM_LHDR;
      for (int x = 0; x < CAM_W; x++) dst[y * CAM_W + x] = CAM_SWAP_BYTES ? (uint16_t)(s[2 * x] | (s[2 * x + 1] << 8)) : (uint16_t)((s[2 * x] << 8) | s[2 * x + 1]); }
    taskENTER_CRITICAL(&camMux); camSt[pick] = CB_FREE; taskEXIT_CRITICAL(&camMux);
    if (!valid) pick = -1;
  }
  if (!wasRunning) camStop();
  return pick >= 0;
}
static void sstvShowTx() {                                         // preview of the TX image (BE conversion)
  for (int y = 0; y < 240; y++) { for (int x = 0; x < 320; x++) ssImg[y * LCD_W + x] = be16(ssTxImg[y * 320 + x]); }
  lcdFill(0, SS_IY, LCD_W, 256, C_BLACK);
  lcdBlit(0, SS_IY, LCD_W, 240, (const uint8_t *)ssImg);
}
static void sstvTx(bool cam) {
  if (txActive || !codecOk || !bigAlloc()) return;
  if (!ssTxImg) ssTxImg = (uint16_t *)heap_caps_malloc(320 * 240 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!ssTxImg) return;
  if (cam) {
    uint16_t *raw = (uint16_t *)heap_caps_malloc(240 * 320 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const bool ok = raw && camGrab(raw);
    if (ok) for (int sy = 0; sy < 320; sy++) for (int sx = 0; sx < 240; sx++) ssTxImg[(239 - sx) * 320 + sy] = raw[sy * 240 + sx];   // rotate 90 deg
    if (raw) free(raw);
    if (!ok) { snprintf(ssMsg[0], 40, "Camera frame not available"); ssStatus(); return; }
  } else sstvTestImage(ssTxImg);
  ssState = 0; ssBlkN = 0;
  sstvShowTx();
  sstvEncodeR36(ssTxImg);
  snprintf(ssMsg[0], 40, "TX Robot 36 %s  %.1f s", cam ? "camera" : "test pattern", bigN / 16000.0f); ssMsg[1][0] = 0;
  Serial.printf("SSTV TX: %lu samples\n", (unsigned long)bigN);
  txActive = true; ssTxLine = -1; startPlayback(bigTx, bigN);
  drawButton(bSsStop, true, C_RED, 12, 20);
  ssStatus();
}
static void sstvButtons() {
  lcdFill(160, 410, 156, 60, C_BG);
  drawButton(bSsCam, false, C_RED, 12, 20); drawButton(bSsBar, false, C_RED, 12, 20);
}
static void txStopNow() { audioMode = AM_IDLE; txActive = false; }
static bool saveBmpBE(const char *prefix, const uint16_t *img, int W, int H, char *name, size_t n) {
  if (!sdOk) return false;
  for (int k = 1; k < 1000; k++) { snprintf(name, n, "/%s_%03d.bmp", prefix, k); if (!SD.exists(name)) break; }
  File f = SD.open(name, FILE_WRITE);
  if (!f) return false;
  const uint32_t row = (uint32_t)W * 3, data = row * H;
  uint8_t h[54] = {'B', 'M'};
  auto put32 = [&](int o, uint32_t v) { h[o] = (uint8_t)v; h[o+1] = (uint8_t)(v >> 8); h[o+2] = (uint8_t)(v >> 16); h[o+3] = (uint8_t)(v >> 24); };
  put32(2, 54 + data); put32(10, 54); put32(14, 40); put32(18, W); put32(22, H); h[26] = 1; h[28] = 24; put32(34, data);
  bool ok = f.write(h, 54) == 54;
  for (int y = H - 1; ok && y >= 0; y--) {
    for (int x = 0; x < W; x++) { const uint16_t v = be16(img[y * W + x]); const uint8_t r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
      bmpLine[3*x] = (uint8_t)((b5 << 3) | (b5 >> 2)); bmpLine[3*x+1] = (uint8_t)((g6 << 2) | (g6 >> 4)); bmpLine[3*x+2] = (uint8_t)((r5 << 3) | (r5 >> 2)); }
    ok = f.write(bmpLine, row) == row;
  }
  f.close();
  return ok;
}
static void sstvStatic() {
  if (!ssRing && psramFound()) ssRing = (int16_t *)heap_caps_calloc(SS_RING, 2, MALLOC_CAP_SPIRAM);
  if (!ssImg && psramFound()) ssImg = (uint16_t *)heap_caps_calloc(256 * LCD_W, 2, MALLOC_CAP_SPIRAM);
  if (!ssRing || !ssImg) { drawText(8, 60, "PSRAM required for SSTV", C_RED); return; }
  section(40, "SSTV  RX auto (VIS)  /  TX Robot 36");
  lcdFill(0, SS_IY, LCD_W, 256, C_BLACK);
  lcdBlit(0, SS_IY, LCD_W, 256, (const uint8_t *)ssImg);           // keep the last picture
  ssRd = ssW; sstvRxOn = true;
  drawButton(bSsClr, false, C_GRAY, 12, 20); drawButton(bSsSave, false, C_CYAN, 12, 20);
  if (txActive) drawButton(bSsStop, true, C_RED, 12, 20); else { drawButton(bSsCam, false, C_RED, 12, 20); drawButton(bSsBar, false, C_RED, 12, 20); }
  ssStatus();
}
static void sstvUpdate(uint32_t now) {
  if (!ssRing) return;
  if (txActive) { ssRd = ssW; ssState = 0; ssBlkN = 0; }          // paused while transmitting
  else sstvProcess();
  if (txActive && now - ssUiT >= 120) {                            // TX progress: bar, %, time, current line marker
    ssUiT = now;
    const uint32_t pos = playPos, len = playLen ? playLen : 1;
    const int pct = (int)((uint64_t)pos * 100 / len), w = (int)((uint64_t)pos * 300 / len);
    lcdFill(10, 318, w, 10, C_RED); lcdFill(10 + w, 318, 300 - w, 10, C_DGRAY);
    drawTextf(4, 334, C_RED, C_BG, 1, "TX Robot 36  %3d %%  %5.1f / %4.1f s      ", pct, pos / 16000.0f, len / 16000.0f);
    const int line = constrain(((int)pos - 14560) / 2400, -1, 239);   // VIS = 910 ms, 150 ms per line
    if (line >= 0 && line != ssTxLine && page == PG_SSTV) { for (int l = max(0, ssTxLine); l <= line; l++) lcdFill(0, SS_IY + l, 4, 1, C_RED); ssTxLine = line; }
    return;
  }
  if (now - ssUiT >= 120) {                                        // tuning indicator 1000..2400 Hz
    ssUiT = now;
    const int f = ssAvg((double)ssW - 400, (double)ssW);
    lcdFill(10, 318, 300, 10, C_BLACK);
    static const int mk[4] = {1200, 1500, 1900, 2300};
    for (int i = 0; i < 4; i++) lcdFill(10 + (mk[i] - 1000) * 300 / 1400, 318, 1, 10, i == 0 ? C_RED : C_DGRAY);
    const int x = 10 + constrain((f - 1000) * 300 / 1400, 0, 297);
    lcdFill(x, 318, 3, 10, C_YELLOW);
    if (ssState == 1 || txActive) ssStatus();
  }
}
static void sstvTouch(uint16_t x, uint16_t y) {
  if (txActive) {
    if (hit(bSsStop, x, y)) {
      const int pct = (int)((uint64_t)playPos * 100 / (playLen ? playLen : 1));
      txStopNow(); snprintf(ssMsg[1], 40, "TX stopped at %d %%", pct); sstvButtons(); ssStatus();
    }
    return;
  }
  if (hit(bSsClr, x, y)) { if (ssImg) memset(ssImg, 0, 256 * LCD_W * 2); lcdFill(0, SS_IY, LCD_W, 256, C_BLACK); ssState = 0; ssBlkN = 0; ssMsg[0][0] = ssMsg[1][0] = 0; ssStatus(); }
  else if (hit(bSsSave, x, y)) {
    char nm[24] = ""; const int h = ssMode >= 0 ? SSTV_MODES[ssMode].lines : 256;
    const bool ok = ssImg && saveBmpBE("sstv", ssImg, LCD_W, h, nm, sizeof(nm));
    hdrMessage(ok ? "SSTV saved:" : "SSTV:", ok ? nm + 1 : (sdOk ? "save failed" : "no microSD"), ok ? C_GREEN : C_RED);
  }
  else if (hit(bSsCam, x, y)) sstvTx(true);
  else if (hit(bSsBar, x, y)) sstvTx(false);
}

// =================================== RTTY =====================================
static const int RT_ROWS = 18, RT_COLS = 38, RT_X = 8, RT_TY = 114, RT_WF_Y = 58, RT_WF_H = 36;
static char     rtBuf[RT_ROWS][RT_COLS + 1];
static int      rtRow = 0, rtCol = 0;
static char     rtPrev = 0;
static uint16_t *rtWf = nullptr;
static int      rtWfHead = 0;
static uint32_t rtLastW = 0, rtUiT = 0;
static char     rtLabB[8], rtLabS[8], rtLabR[6];
static Button rtBtn(int i) { static char *L[3] = {rtLabB, rtLabS, rtLabR}; Button b = {(int16_t)(4 + i * 78), 356, 76, 46, i < 3 ? L[i] : "CLEAR"}; return b; }
static const Button bRtCq = {4, 410, 100, 60, "CQ"}, bRtRy = {110, 410, 100, 60, "RYRY"}, bRtInfo = {216, 410, 100, 60, "INFO"};
static char rtStopLab[20] = "STOP";
static const Button bRtStop = {4, 410, 312, 60, rtStopLab};
static int rtTxPct = -1;
static void rtMacros() { lcdFill(0, 410, LCD_W, 60, C_BG); drawButton(bRtCq, false, C_RED); drawButton(bRtRy, false, C_RED); drawButton(bRtInfo, false, C_RED); }
static void rtDrawRow(int r) { char b[RT_COLS + 1]; snprintf(b, sizeof(b), "%-38s", rtBuf[r]); drawText(RT_X, RT_TY + r * FONT_H, b, C_WHITE, C_BLACK); }
static void rtNewLine() {
  rtCol = 0;
  if (++rtRow >= RT_ROWS) {
    memmove(rtBuf[0], rtBuf[1], sizeof(rtBuf[0]) * (RT_ROWS - 1)); rtRow = RT_ROWS - 1; rtBuf[rtRow][0] = 0;
    if (page == PG_RTTY) for (int r = 0; r < RT_ROWS; r++) rtDrawRow(r);
  } else rtBuf[rtRow][0] = 0;
}
static void rtPutc(char c) {
  if (c == '\r') { rtNewLine(); rtPrev = c; return; }
  if (c == '\n') { if (rtPrev != '\r') rtNewLine(); rtPrev = c; return; }
  rtPrev = c;
  if (c < 0x20 || c > 0x7E) return;
  if (rtCol >= RT_COLS) rtNewLine();
  rtBuf[rtRow][rtCol++] = c; rtBuf[rtRow][rtCol] = 0;
  if (page == PG_RTTY) drawChar(RT_X + (rtCol - 1) * FONT_W, RT_TY + rtRow * FONT_H, c, C_WHITE, C_BLACK, 1);
}
static void rtLabels() {
  snprintf(rtLabB, sizeof(rtLabB), "%g", RTTY_BAUDS[rtBaudI]); snprintf(rtLabS, sizeof(rtLabS), "%u", RTTY_SHIFTS[rtShiftI]); snprintf(rtLabR, sizeof(rtLabR), rtRev ? "REV" : "NOR");
  for (int i = 0; i < 4; i++) drawButton(rtBtn(i), i == 2 && rtRev, i == 3 ? C_GRAY : C_CYAN, 12, 20);
}
static float rtMark() { return rtRev ? rtLow + RTTY_SHIFTS[rtShiftI] : rtLow; }
static float rtSpace() { return rtRev ? rtLow : rtLow + RTTY_SHIFTS[rtShiftI]; }
static void rtMarkers() {
  lcdFill(0, RT_WF_Y - 4, LCD_W, 4, C_BG);
  lcdFill((int)(rtMark() * 320 / 3000) - 1, RT_WF_Y - 4, 3, 4, C_YELLOW);
  lcdFill((int)(rtSpace() * 320 / 3000) - 1, RT_WF_Y - 4, 3, 4, C_CYAN);
  drawTextf(4, 98, C_GRAY, C_BG, 1, "M %4.0f  S %4.0f  %g Bd  tap WF to tune ", rtMark(), rtSpace(), RTTY_BAUDS[rtBaudI]);
}
static void rttyStatic() {
  if (!specTables) fftInitTables();
  if (!rtWf && psramFound()) rtWf = (uint16_t *)heap_caps_calloc(LCD_W * RT_WF_H, 2, MALLOC_CAP_SPIRAM);
  if (!rttyQ) rttyQ = xQueueCreate(128, 1);
  section(40, "RTTY  ITA2  0-3 kHz waterfall");
  lcdFill(0, RT_WF_Y, LCD_W, RT_WF_H, C_BLACK);
  lcdRect(RT_X - 3, RT_TY - 3, RT_COLS * FONT_W + 6, RT_ROWS * FONT_H + 6, C_GRAY);
  for (int r = 0; r < RT_ROWS; r++) rtDrawRow(r);
  rtMarkers(); rtLabels();
  if (txActive) { rtTxPct = -1; } else rtMacros();
  specLastW = specW; specEnabled = true; rttyRxOn = true;
}
static void rttyUpdate(uint32_t now) {
  char c;
  while (rttyQ && xQueueReceive(rttyQ, &c, 0) == pdTRUE) rtPutc(c);
  const uint32_t w = specW;
  if (rtWf && w - rtLastW >= 1024) {                               // waterfall row every 64 ms
    rtLastW = w;
    for (int i = 0; i < FFT_N; i++) { fRe[i] = ((int32_t)specRing[(w - FFT_N + i) & (SPEC_RING - 1)] * hannW[i]) >> 15; fIm[i] = 0; }
    fftRun();
    rtWfHead = (rtWfHead + RT_WF_H - 1) % RT_WF_H;
    uint16_t *row = rtWf + rtWfHead * LCD_W;
    for (int x = 0; x < LCD_W; x++) {
      const int k = (int)(x * 3000.0f / LCD_W / 31.25f);
      const int64_t m2 = (int64_t)fRe[k] * fRe[k] + (int64_t)fIm[k] * fIm[k];
      const float db = 10.0f * log10f((float)m2 + 1e-3f) - 78.27f;
      row[x] = specPal[constrain((int)((db + 95.0f) / 75.0f * 255.0f), 0, 255)];
    }
    SPI.beginTransaction(lcdSpi); digitalWrite(PIN_LCD_CS, LOW);
    lcdWindowRaw(0, RT_WF_Y, LCD_W - 1, RT_WF_Y + RT_WF_H - 1);
    for (int r = 0; r < RT_WF_H; r++) SPI.writeBytes((const uint8_t *)(rtWf + ((rtWfHead + r) % RT_WF_H) * LCD_W), LCD_W * 2);
    digitalWrite(PIN_LCD_CS, HIGH); SPI.endTransaction();
  }
  if (now - rtUiT >= 200) {
    rtUiT = now;
    if (txActive && page == PG_RTTY) {
      const int pct = (int)((uint64_t)playPos * 100 / (playLen ? playLen : 1));
      if (pct != rtTxPct) { rtTxPct = pct; snprintf(rtStopLab, sizeof(rtStopLab), "STOP TX %d%%", pct); drawButton(bRtStop, true, C_RED); }
    }
    const int qw = constrain((int)(rtQ * 100), 0, 100);
    lcdFill(250, 98, qw * 66 / 100, 10, rtQ > 0.3f ? C_GREEN : C_ORANGE); lcdFill(250 + qw * 66 / 100, 98, 66 - qw * 66 / 100, 10, C_BLACK);
  }
}
static void rttyEncode(const char *msg) {
  txReset();
  const float mark = rtMark(), space = rtSpace(), bitMs = 1000.0f / RTTY_BAUDS[rtBaudI];
  auto frame = [&](uint8_t code) {
    txTone(space, bitMs);
    for (int b = 0; b < 5; b++) txTone(((code >> b) & 1) ? mark : space, bitMs);
    txTone(mark, bitMs * 1.5f);
  };
  txTone(mark, 1000);                                              // idle mark carrier
  frame(0x1F); frame(0x1F);
  bool figs = false;
  for (const char *p = msg; *p; p++) {
    char ch = (char)toupper((uint8_t)*p);
    if (ch == '\n') { frame(8); frame(2); continue; }
    int lc = -1, fc = -1;
    for (int i = 0; i < 32; i++) { if (RT_LTR[i] == ch && ch) lc = i; if (RT_FIG[i] == ch && ch) fc = i; }
    if (lc >= 0 && (ch == ' ' || !figs || fc < 0)) { if (figs && ch != ' ') { frame(0x1F); figs = false; } frame((uint8_t)lc); if (ch == ' ') figs = false; }
    else if (fc >= 0) { if (!figs) { frame(0x1B); figs = true; } frame((uint8_t)fc); }
  }
  frame(8); frame(2); txTone(mark, 500);
}
static void rttyTx(int which) {
  if (txActive || !codecOk || !bigAlloc()) return;
  char m[200];
  if (which == 0) snprintf(m, sizeof(m), "\nCQ CQ CQ DE %s %s %s\nCQ CQ CQ DE %s %s %s PSE K\n", cfg.call, cfg.call, cfg.call, cfg.call, cfg.call, cfg.call);
  else if (which == 1) snprintf(m, sizeof(m), "\nRYRYRYRYRYRYRYRYRYRYRYRYRYRYRYRY\nTHE QUICK BROWN FOX JUMPS OVER THE LAZY DOG 0123456789\n");
  else snprintf(m, sizeof(m), "\nDE %s ELECTGPL ESP32-C5 RTTY %g BD %u HZ SHIFT, 73\n", cfg.call, RTTY_BAUDS[rtBaudI], RTTY_SHIFTS[rtShiftI]);
  rttyEncode(m);
  Serial.printf("RTTY TX %lu samples: %s", (unsigned long)bigN, m);
  txActive = true; rtTxPct = -1; startPlayback(bigTx, bigN);
}
static void rttyTouch(uint16_t x, uint16_t y) {
  if (txActive) { if (hit(bRtStop, x, y)) { txStopNow(); rtPutc('\n'); for (const char *m = "[TX stopped]"; *m; m++) rtPutc(*m); rtPutc('\n'); rtMacros(); } return; }
  if (y >= RT_WF_Y - 4 && y < RT_WF_Y + RT_WF_H) { rtLow = constrain(roundf(x * 3000.0f / 320 / 5) * 5, 300.0f, 2800.0f); rtCfgSeq = rtCfgSeq + 1; rtMarkers(); return; }
  for (int i = 0; i < 4; i++) if (hit(rtBtn(i), x, y)) {
    if (i == 0) rtBaudI = (rtBaudI + 1) % 3; else if (i == 1) rtShiftI = (rtShiftI + 1) % 4; else if (i == 2) rtRev = !rtRev;
    else { memset(rtBuf, 0, sizeof(rtBuf)); rtRow = rtCol = 0; for (int r = 0; r < RT_ROWS; r++) rtDrawRow(r); return; }
    rtCfgSeq = rtCfgSeq + 1; rtLabels(); rtMarkers(); return;
  }
  if (hit(bRtCq, x, y)) rttyTx(0); else if (hit(bRtRy, x, y)) rttyTx(1); else if (hit(bRtInfo, x, y)) rttyTx(2);
}

// ================================ Dispatch ====================================
static void audioTouch(uint16_t x, uint16_t y) {
  if (txActive) return;
  bool changed = false;
  if (hit(bRec, x, y) && recBuf) { toneSel = 3; recPos = 0; recLen = 0; waveValid = false; drawWave(); audioMode = AM_REC; changed = true; }
  else if (hit(bPlay, x, y) && recLen > 0) { toneSel = 3; startPlayback(recBuf, recLen); changed = true; }
  else if (hit(bStop, x, y)) {
    if (audioMode == AM_REC && recPos > AUDIO_FS / 2) { recLen = recPos; audioMode = AM_IDLE; recDoneFlag = true; }
    else audioMode = AM_IDLE;
    toneSel = 3; snprintf(audioMsg, sizeof(audioMsg), "Stopped"); audioMsgDirty = true; changed = true;
  }
  for (int i = 0; i < 4; i++) if (hit(bTone[i], x, y)) {
    toneSel = i;
    if (toneFreq[i] > 0) { tonePhaseInc = phaseIncFor(toneFreq[i]); audioMode = AM_TONE; snprintf(audioMsg, sizeof(audioMsg), "Tone %s Hz", bTone[i].label); }
    else { if (audioMode == AM_TONE) audioMode = AM_IDLE; snprintf(audioMsg, sizeof(audioMsg), "Tone off"); }
    audioMsgDirty = true; changed = true;
  }
  if (changed) audioButtons();
}
static void appTouch(uint16_t x, uint16_t y, bool press, bool touching) {
  switch (page) {
    case PG_HOME:   if (press) homeTouch(x, y); break;
    case PG_AUDIO:  if (press) audioTouch(x, y); break;
    case PG_RADIO:
      if (!press) break;
      if (hit(bEspNow, x, y)) { if (ieeeOn) ieeeStop(); radioMode = 3; radioStatic(); break; }
      if (radioMode == 3 && !hit(bWifi, x, y) && !hit(bBle, x, y) && !hit(bIeee, x, y)) { enTouch(x, y); break; }
      enStop();
      if (hit(bWifi, x, y)) { if (ieeeOn) ieeeStop(); radioMode = 0; radioStatic(); wifiStartScan(); radioDrawSummary(); }
      else if (hit(bBle, x, y)) { if (ieeeOn) ieeeStop(); radioMode = 1; radioStatic(); bleStartScan(); radioDrawSummary(); }
      else if (hit(bIeee, x, y)) { radioMode = 2; radioStatic(); }
      else if (radioMode == 2) ieeeTouch(x, y);
      break;
    case PG_FOX:    if (press) foxTouch(x, y); break;
    case PG_SPEC:   if (press) specTouch(x, y); break;
    case PG_SET:    if (press) settingsTouch(x, y); break;
    case PG_APRS:   if (press) aprsTouch(x, y); break;
    case PG_MAZE:   if (press) mazeTouch(x, y); break;
    case PG_SSTV:   if (press) sstvTouch(x, y); break;
    case PG_RTTY:   if (press) rttyTouch(x, y); break;
    case PG_AX25:
      if (!press) break;
      if (hit(bSend, x, y)) { if (audioMode == AM_REC || audioMode == AM_TONE) audioMode = AM_IDLE; ax25Send(); }
      else if (hit(bClear, x, y)) { termCount = 0; ax25Ok = 0; ax25Bad = 0; termRedraw(); }
      break;
    case PG_PAINT:  paintTouch(x, y, press, touching); break;
    case PG_NOTES:  notesTouch(x, y, press, touching); break;
    case PG_TETRIS: tetrisTouch(x, y, press, touching); break;
    case PG_CALC:   if (press) calcTouch(x, y); break;
    default: break;
  }
}

static void showPage(Page p) {
  if (page == PG_CAM && p != PG_CAM) camStop();
  if (page == PG_RADIO && p != PG_RADIO && ieeeOn) ieeeStop();
  if (page == PG_RADIO && p != PG_RADIO) enStop();
  if (page == PG_FOX && p != PG_FOX) foxStop();
  specEnabled = false; sstvRxOn = false; rttyRxOn = false;
  if (page == PG_SET && p != PG_SET) stView = SV_MAIN;
  page = p;
  ax25RxEnabled = (p == PG_AX25 || p == PG_APRS);
  lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
  switch (p) {
    case PG_HOME:   homeStatic(); break;
    case PG_SENSOR: sensorStatic(); break;
    case PG_AUDIO:  audioStatic(); break;
    case PG_CAM:    camStatic(); camFpsT0 = millis(); camFpsN = 0; camStart(); break;
    case PG_AX25:   ax25Static(); break;
    case PG_RADIO:  radioStatic(); break;
    case PG_PAINT:  paintStatic(); break;
    case PG_NOTES:  notesStatic(); break;
    case PG_TETRIS: tetrisStatic(); break;
    case PG_CALC:   calcStatic(); break;
    case PG_FOX:    foxStatic(); break;
    case PG_SPEC:   specStatic(); break;
    case PG_SET:    settingsStatic(); break;
    case PG_APRS:   aprsStatic(); break;
    case PG_MAZE:   mazeStatic(); break;
    case PG_SSTV:   sstvStatic(); break;
    case PG_RTTY:   rttyStatic(); break;
  }
}
static void goHome() { if (page != PG_HOME) showPage(PG_HOME); }
static void goBack() {
  if (page == PG_NOTES && ntView) { ntView = false; notesStatic(); }
  else if (page == PG_FOX && foxTrack) { foxStop(); foxDrawList(); }
  else if (page == PG_SET && stView != SV_MAIN) { stView = stView == SV_NET ? SV_TIME : stView == SV_PASS ? SV_NET : SV_MAIN; settingsStatic(); }
  else if (page == PG_APRS && aprsView != 0) { aprsView = 0; aprsStatic(); }
  else if (page == PG_MAZE && mzWiz) { mzWiz = 0; mzDrawMaze(); }
  else goHome();
}
static void pwrKeyPoll() {                                         // AXP2101 PWR short press latched in INTSTS2 bit3
  if (!axpOk) return;
  const int st = i2cR8(A_AXP, 0x49);
  if (st >= 0 && (st & 0x08)) { i2cW8(A_AXP, 0x49, 0x08); pmActivity(); Serial.println("PWR short press: BACK"); goBack(); }
}

// ================================ setup / loop ================================
void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("\n=== Electgpl ESP32-C5-Touch-LCD-3.5 demo v3 ===");
  pmBootReason();
  Serial.println(wakeMsg);
  settingsLoad();
  userBacklight = cfg.blUsb;
  Serial.printf("Settings: call %s-%u, vol %+d dB, mic %u dB, sleep %u s %s\n", cfg.call, cfg.ssid, cfg.volDb, cfg.micGain * 6,
                SLEEP_OPTS[cfg.sleepIdx], cfg.sleepMode == SLEEP_DEEP ? "DEEP" : "POWEROFF");
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {        // release LP pad config from sleep
    rtc_gpio_deinit((gpio_num_t)PIN_TP_INT);
    if (WAKE_ON_GPIO4) rtc_gpio_deinit(GPIO_NUM_4);
  }
  pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_LCD_CS, OUTPUT); digitalWrite(PIN_LCD_CS, HIGH);
  pinMode(PIN_LCD_DC, OUTPUT); digitalWrite(PIN_LCD_DC, HIGH);
  pinMode(PIN_TP_INT, INPUT_PULLUP); pinMode(PIN_BOOT, INPUT_PULLUP);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  axpOk = axpInit();
  if (axpOk) {                                      // PWR short-press IRQ (INTEN2 bit3), clear stale flags
    int en2 = i2cR8(A_AXP, 0x41);
    if (en2 >= 0) i2cW8(A_AXP, 0x41, (uint8_t)(en2 | 0x08));
    i2cW8(A_AXP, 0x48, 0xFF); i2cW8(A_AXP, 0x49, 0xFF); i2cW8(A_AXP, 0x4A, 0xFF);
  }
  exioOk = exioInit();
  touchOk = i2cPresent(A_FT6336);
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  lcdInit();
  lcdFill(0, 0, LCD_W, LCD_H, C_BG);
  drawHeader();
  drawBattery(true);
  if (exioOk) backlightSet(cfg.blUsb);
  drawText(8, 60, "Initializing peripherals...", C_GRAY);

  shtOk = i2cPresent(A_SHTC3);
  rtcOk = i2cPresent(A_PCF85063) && rtcInit();
  imuOk = imuInit();
  sdOk = SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ, "/sd", 5, false);
  audioInit();
  const bool camOk = camInit();
  termWrite("Electgpl AX.25 terminal ready", C_CYAN);
  termWrite("Feed 1200 Bd AFSK to the mic or press SEND", C_GRAY);

  Serial.printf("PSRAM %s (%lu B) | AXP %d EXIO %d TP %d SHTC3 %d RTC %d%s IMU %d SD %d ES8311 %d CAM %d %s\n",
                psramFound() ? "OK" : "NO", (unsigned long)ESP.getPsramSize(), axpOk, exioOk, touchOk, shtOk, rtcOk,
                rtcWasSet ? "(set)" : "", imuOk, sdOk, codecOk, camOk, camErr);
  if (!psramFound()) {
    lcdFill(0, 37, LCD_W, LCD_H - 37, C_BG);
    drawText(8, 60, "PSRAM disabled:", C_RED, C_BG, 2);
    drawText(8, 90, "Tools > PSRAM > Enabled", C_YELLOW);
    drawText(8, 106, "(recorder and camera unavailable)", C_GRAY);
    delay(3000);
  }
  lastActivity = millis();
  showPage(PG_HOME);
}

void loop() {
  static uint32_t tClk = 0, tImu = 0, tEnv = 0, tPm = 0, tSys = 0, tMic = 0, tAx = 0, tPwr = 0, bootDown = 0;
  static bool touchPrev = false, bootPrev = false, bootLongDone = false;
  static uint8_t blIdx = 0; static const uint8_t blLv[] = {100, 50, 10};
  const uint32_t now = millis();

  if (now - tClk >= 1000) { tClk = now; drawClock(); }
  switch (page) {                                    // periodic work of the visible app only
    case PG_SENSOR:
      if (now - tImu >= 100)  { tImu = now; sensorImu(); }
      if (now - tEnv >= 1000) { tEnv = now; sensorEnv(); }
      if (now - tPm  >= 1000) { tPm = now; sensorPmic(); }
      if (now - tSys >= 2000) { tSys = now; sensorSys(); }
      break;
    case PG_AUDIO:  if (now - tMic >= 66) { tMic = now; audioMeter(); } break;
    case PG_CAM:    camUpdate(); break;
    case PG_AX25:   if (now - tAx >= 150) { tAx = now; ax25Status(); } break;
    case PG_TETRIS: tetrisUpdate(now); break;
    case PG_RADIO:  if (radioMode == 2) ieeeUpdate(now); else if (radioMode == 3) enUpdate(now); break;
    case PG_FOX:    foxUpdate(now); break;
    case PG_SPEC:   specUpdate(); break;
    case PG_APRS:   aprsUpdate(now); break;
    case PG_MAZE:   mazeUpdate(now); break;
    case PG_SSTV:   sstvUpdate(now); break;
    case PG_RTTY:   rttyUpdate(now); break;
    default: break;
  }

  wifiPoll(); blePoll();
  pmUpdate(now);
  if (now - tPwr >= 100) { tPwr = now; pwrKeyPoll(); }

  Ax25Frame fr;                                      // decoded frames (also logged off-page)
  while (ax25Queue && xQueueReceive(ax25Queue, &fr, 0) == pdTRUE) { ax25Display(fr); aprsIngest(fr); }

  if (recDoneFlag) { recDoneFlag = false; audioPostRecord(); if (page == PG_AUDIO) audioButtons(); }
  if (playDoneFlag) {
    playDoneFlag = false;
    if (txActive) { txActive = false; if (page == PG_AX25) drawButton(bSend, false, C_RED); if (page == PG_APRS && aprsView == 0) drawButton(bApBcn, false, C_RED);
                    if (page == PG_SSTV) { snprintf(ssMsg[1], 40, "TX done (100 %%)"); sstvButtons(); ssStatus(); }
                    if (page == PG_RTTY) rtMacros(); }
    else { snprintf(audioMsg, sizeof(audioMsg), "Playback finished"); audioMsgDirty = true; if (page == PG_AUDIO) audioButtons(); }
  }

  uint16_t x = 0, y = 0;
  const bool touching = touchOk && touchRead(x, y);
  const bool press = touching && !touchPrev;
  if (touching) pmActivity();
  if (press && y < 36 && x < 140 && page != PG_HOME) goHome();       // tap on the Electgpl logo = HOME
  else if (touching || touchPrev) appTouch(x, y, press, touching);
  touchPrev = touching;

  // BOOT: short press = HOME, long press (>= 800 ms) = backlight level
  const bool bootNow = digitalRead(PIN_BOOT) == LOW;
  if (bootNow) pmActivity();
  if (bootNow && !bootPrev) { bootDown = now; bootLongDone = false; }
  if (bootNow && !bootLongDone && now - bootDown >= 800) {
    bootLongDone = true;
    blIdx = (blIdx + 1) % sizeof(blLv); userBacklight = blLv[blIdx];
    if (exioOk && !onBattery) backlightSet(userBacklight);
  }
  if (!bootNow && bootPrev && !bootLongDone && now - bootDown > 30) goHome();
  bootPrev = bootNow;

  delay(page == PG_CAM || page == PG_PAINT || page == PG_SPEC || page == PG_RADIO || page == PG_MAZE || page == PG_SSTV || page == PG_RTTY ? 1 : 5);
}
