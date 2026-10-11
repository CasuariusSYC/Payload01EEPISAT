// =====================================================================
//  PAYLOAD SATELIT MINI  -  ESP32
//  MPU6050 MANUAL (Wire) + Quaternion IMU + BMP280 + LoRa + GPS
//  Fault-Tolerant + LED indikator (ON = OK, KEDIP = ERROR)
//  GPS BAUD = 115200
// =====================================================================

#include <Arduino.h>

#include <Wire.h>

#include <SPI.h>

#include <Adafruit_BMP280.h>

#include <LoRa.h>

#include <TinyGPSPlus.h>

Adafruit_BMP280  bmp;

TinyGPSPlus      gps;

// =====================================================================
//  BAGIAN 1 : PENGATURAN
// =====================================================================

// ---------- 1.1 Identitas & fitur ----------
const uint8_t TEAM_ID = 8;

const bool ENABLE_DETONATOR = true;

const bool DEBUG_SERIAL     = true;

// true = lomba (sensor gagal -> stop), false = debug (tetap jalan)
const bool HALT_ON_SENSOR_FAIL = false;

// ---------- 1.2 Pin ESP32 ----------
const int PIN_DETONATOR1 = 2;

const int PIN_DETONATOR2 = 32;

const int PIN_LED1 = 25;   // MPU6050: ON kalau OK, KEDIP kalau error
const int PIN_LED2 = 27;   // BMP280 : ON kalau OK, KEDIP kalau error
const int PIN_LED3 = 33;   // LoRa   : ON kalau OK, KEDIP kalau error
const int PIN_GPS_RX = 16;

const int PIN_GPS_TX = 17;

const int PIN_LORA_SCK  = 18;

const int PIN_LORA_MISO = 19;

const int PIN_LORA_MOSI = 23;

const int PIN_LORA_SS   = 5;

const int PIN_LORA_RST  = 14;

const int PIN_LORA_DIO0 = 26;

const int PIN_SDA = 21;

const int PIN_SCL = 22;

const int PIN_ADC_ARUS = 34;

const int PIN_ADC_VOLT = 35;

// ---------- 1.3 GPS ----------
const unsigned long GPS_BAUD = 115200;

// ---------- 1.4 LoRa ----------
const long  LORA_FREKUENSI = 433E6;

const int   LORA_SF        = 10;

const long  LORA_BW        = 125E3;

const int   LORA_CR        = 5;

const int   LORA_TX_POWER  = 20;

const int   LORA_PREAMBLE  = 12;

const long  LORA_SYNCWORD  = 0x12;

const unsigned long INTERVAL_TELEMETRI_MS = 500;   // ~2 Hz

// ---------- 1.5 Sensor ----------
const int   JUMLAH_SAMPEL_GYRO = 500;

const float DEADBAND_DPS       = 0.3f;

const float MPU_DT_MIN         = 0.005f;

const int   JUMLAH_SAMPEL_TEKANAN = 100;

const float KALMAN_Q = 0.01f;

const float KALMAN_R = 0.05f;

const unsigned long INTERVAL_BARO_MS = 50;

const float RASIO_PEMBAGI_VOLT   = 8.4f / 3.3f;

const float ARUS_OFFSET_V        = 0.0f;

const float ARUS_SENSITIVITAS_VA = 1.0f;

// ---------- 1.6 Ambang state penerbangan ----------
constexpr float ALT_TERBANG_M      = 2.0f;

constexpr float ALT_JATUH_M        = 20.0f;

constexpr float ALT_BOM_DROP_M     = 15.0f;

constexpr float ALT_MENDARAT_M     = 0.5f;

constexpr float TURUN_APOGEE_M     = 2.0f;

const int       JUMLAH_SAMPEL_MENDARAT = 3;

// Durasi detonator nyala (ms)
const unsigned long DETONATOR_BURN_MS = 3000;

// ---------- 1.7 MPU6050 (Wire) & BMP280 ----------
const uint8_t MPU6050_ADDR = 0x68;

const uint8_t BMP280_ADDR  = 0x76;

#define MPU6050_REG_SMPLRT_DIV    0x19

#define MPU6050_REG_CONFIG        0x1A

#define MPU6050_REG_GYRO_CONFIG   0x1B

#define MPU6050_REG_ACCEL_CONFIG  0x1C

#define MPU6050_REG_ACCEL_XOUT_H  0x3B

#define MPU6050_REG_PWR_MGMT_1    0x6B

#define MPU6050_REG_WHO_AM_I      0x75

const float MPU6050_ACCEL_SCALE = 4096.0f;   // ±8g
const float MPU6050_GYRO_SCALE  = 65.5f;     // ±500 deg/s

// ---------- 1.8 Flag status sensor ----------
bool mpuOK  = false;

bool bmpOK  = false;

bool loraOK = false;

// ---------- 1.9 Deteksi error runtime ----------
const uint16_t FAIL_LIMIT = 20;

uint16_t mpuFailCount = 0;

uint16_t bmpFailCount = 0;

// ---------- 1.10 Kecepatan kedip LED error ----------
const unsigned long LED_ERROR_BLINK_MS = 250;

// =====================================================================
//  BAGIAN 2 : VARIABEL
// =====================================================================

enum State { ST_SETUP = 0, ST_TERBANG = 1, ST_JATUH = 2, ST_MENDARAT = 3 };

const char* STATE_NAME[] = { "SETUP", "TERBANG", "JATUH", "MENDARAT" };

State state = ST_SETUP;

unsigned long waktuStateMulai = 0;

bool pernahTerbang  = false;

bool pernahJatuh    = false;

bool bombDropEvent  = false;

bool botDeployEvent = false;

// IMU quaternion
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

float currentRoll = 0.0f, currentPitch = 0.0f, currentYaw = 0.0f;

float offsetGyroX = 0.0f, offsetGyroY = 0.0f, offsetGyroZ = 0.0f;

unsigned long lastTimeIMU = 0;

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

// Barometer
float tekananReferensi = 1013.25f;

float tekananFiltered  = 1013.25f;

float ketinggian       = 0;

float maxKetinggian    = 0;

// Listrik
float tegangan = 0;

float arus     = 0;

// Timer
unsigned long tBaro = 0, tAdc = 0, tTelemetri = 0, tLanded = 0;

float landedAltLast = 0;

int   landedCount   = 0;

// LoRa
unsigned long packetId = 0;

bool txPending = false;

volatile bool txSelesai = false;

unsigned long tTxMulai = 0;

// Detonator
unsigned long det1Start = 0, det2Start = 0;

bool det1Aktif = false, det2Aktif = false;

// GPS ring buffer
#define GPS_BUF_SIZE 512

volatile uint8_t  gpsBuf[GPS_BUF_SIZE];

volatile uint16_t gpsHead = 0;

volatile uint16_t gpsTail = 0;

// =====================================================================
//  BAGIAN 3 : FUNGSI
// =====================================================================

// ---------- 3.1 Kalman 1D ----------
struct Kalman1D {
  float x = 0, p = 1, q = 0.01, r = 0.05;
  void init(float nilaiAwal, float q_, float r_) { x = nilaiAwal; p = 1; q = q_; r = r_; }
  float update(float z) {
    p += q;
    float k = p / (p + r);
    x += k * (z - x);
    p *= (1.0f - k);
    return x;
  }
};

Kalman1D kalmanTekanan;

// ---------- 3.2 Utilitas LED ----------
void ledOn(uint8_t pin)  { digitalWrite(pin, HIGH); }

void ledOff(uint8_t pin) { digitalWrite(pin, LOW);  }

// Kedip non-blocking per pin. Pakai index = nomor pin (semua pin < 40).
void blinkStep(uint8_t pin, unsigned long periodeMs) {
  static unsigned long tBlink[40] = {0};
  if (pin >= 40) return;
  if (millis() - tBlink[pin] >= periodeMs) {
    tBlink[pin] = millis();
    digitalWrite(pin, !digitalRead(pin));
  }
}

// Update LED indikator berdasarkan flag status.
// - OK   -> LED nyala terus
// - FAIL -> LED kedip terus
void updateLedIndicators() {
  if (mpuOK) {
    digitalWrite(PIN_LED1, HIGH);
  } else {
    blinkStep(PIN_LED1, LED_ERROR_BLINK_MS);
  }

  if (bmpOK) {
    digitalWrite(PIN_LED2, HIGH);
  } else {
    blinkStep(PIN_LED2, LED_ERROR_BLINK_MS);
  }

  if (loraOK) {
    digitalWrite(PIN_LED3, HIGH);
  } else {
    blinkStep(PIN_LED3, LED_ERROR_BLINK_MS);
  }
}

// ---------- 3.3 Utilitas error ----------
void errorHalt(const char* pesan) {
  Serial.println(pesan);
  while (1) {
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
    digitalWrite(PIN_LED3, !digitalRead(PIN_LED3));
    delay(200);
  }
}

void failLembut(const char* pesan) {
  Serial.print("[WARN] ");
  Serial.println(pesan);
  if (HALT_ON_SENSOR_FAIL) errorHalt(pesan);
}

// ---------- 3.4 GPS (UART2 dengan interrupt) ----------
// Dipanggil otomatis oleh UART saat ada byte masuk dari GPS.
// Tugasnya cuma pindahkan byte ke ring buffer; parsing di prosesGps().
void onGpsReceive() {
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    uint16_t next = (gpsHead + 1) % GPS_BUF_SIZE;
    if (next != gpsTail) {
      gpsBuf[gpsHead] = c;
      gpsHead = next;
    }
  }
}

// Dipanggil dari loop(): ambil byte dari ring buffer, beri ke parser NMEA.
void prosesGps() {
  while (gpsTail != gpsHead) {
    gps.encode(gpsBuf[gpsTail]);
    gpsTail = (gpsTail + 1) % GPS_BUF_SIZE;
  }
}

// ---------- 3.5 MPU6050 MANUAL (Wire) ----------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuReadByte(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(MPU6050_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU6050_ADDR, (uint8_t)14) != 14) return false;

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read();   // suhu, dibuang
  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

void i2cScan() {
  Serial.println("Scan I2C...");
  uint8_t jumlah = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Ditemukan perangkat di 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      jumlah++;
    }
  }
  if (jumlah == 0) Serial.println("  Tidak ada perangkat I2C!");
  Serial.println("Scan selesai.");
}

void setupMPU() {
  uint8_t who = mpuReadByte(MPU6050_REG_WHO_AM_I);

  if (who != 0x68 && who != 0x74) {
    mpuOK = false;
    Serial.print("MPU6050 WHO_AM_I = 0x");
    Serial.println(who, HEX);
    failLembut("MPU6050 tidak terdeteksi - lanjut tanpa IMU");
    return;
  }

  mpuWrite(MPU6050_REG_PWR_MGMT_1, 0x00); delay(50);
  mpuWrite(MPU6050_REG_SMPLRT_DIV, 0x04);
  mpuWrite(MPU6050_REG_CONFIG,     0x04);
  mpuWrite(MPU6050_REG_GYRO_CONFIG,  0x08);
  mpuWrite(MPU6050_REG_ACCEL_CONFIG, 0x10);

  mpuOK = true;
  mpuFailCount = 0;
  Serial.print("IMU manual (Wire) terdeteksi, WHO_AM_I=0x");
  Serial.println(who, HEX);
}

// ---------- 3.6 BMP280 (library) ----------
void setupBMP() {
  if (!bmp.begin(BMP280_ADDR)) {
    bmpOK = false;
    Serial.print("BMP280 gagal di alamat 0x");
    Serial.println(BMP280_ADDR, HEX);

    if (bmp.begin(0x77)) {
      bmpOK = true;
      Serial.println("BMP280 terdeteksi di 0x77 (fallback).");
    } else {
      failLembut("BMP280 tidak terdeteksi - lanjut tanpa barometer");
      return;
    }
  } else {
    bmpOK = true;
    Serial.println("BMP280 berhasil terhubung.");
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);

  bmpFailCount = 0;
}

// ---------- 3.7 LoRa ----------
void IRAM_ATTR onTxDone() { txSelesai = true; }

void setupLoRa() {
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_SS);
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(LORA_FREKUENSI)) {
    loraOK = false;
    failLembut("LoRa gagal inisialisasi - lanjut tanpa telemetri RF");
    return;
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setSyncWord(LORA_SYNCWORD);
  LoRa.enableCrc();
  LoRa.setGain(0);
  LoRa.onTxDone(onTxDone);
  LoRa.receive();

  loraOK = true;
  Serial.println("LoRa siap.");
}

// ---------- 3.8 Kalibrasi gyro ----------
void kalibrasiGyro() {
  if (!mpuOK) {
    Serial.println("Lewati kalibrasi gyro (MPU6050 tidak ada).");
    return;
  }

  Serial.println("Kalibrasi gyro... JANGAN GERAKKAN SENSOR!");

  for (int i = 0; i < 40; i++) {
    blinkStep(PIN_LED1, 100);
    delay(50);
  }

  double tx = 0, ty = 0, tz = 0;
  int n = 0;

  for (int i = 0; i < JUMLAH_SAMPEL_GYRO; i++) {
    blinkStep(PIN_LED1, 100);
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
      tx += gx / MPU6050_GYRO_SCALE;
      ty += gy / MPU6050_GYRO_SCALE;
      tz += gz / MPU6050_GYRO_SCALE;
      n++;
    }
    delay(5);
  }

  if (n == 0) {
    mpuOK = false;
    ledOff(PIN_LED1);
    failLembut("Gagal baca MPU6050 saat kalibrasi - lanjut tanpa IMU");
    return;
  }

  offsetGyroX = tx / n;
  offsetGyroY = ty / n;
  offsetGyroZ = tz / n;

  currentRoll = currentPitch = currentYaw = 0.0f;
  lastTimeIMU = micros();

  Serial.print("Offset gyro (deg/s): ");
  Serial.print(offsetGyroX, 3); Serial.print(", ");
  Serial.print(offsetGyroY, 3); Serial.print(", ");
  Serial.println(offsetGyroZ, 3);
  Serial.println("Kalibrasi gyro selesai.");
}

// ---------- 3.9 Kalibrasi tekanan ----------
void kalibrasiTekanan() {
  if (!bmpOK) {
    Serial.println("Lewati kalibrasi tekanan (BMP280 tidak ada).");
    return;
  }

  Serial.println("Kalibrasi tekanan udara... jangan pindahkan payload!");

  for (int i = 0; i < 30; i++) {
    blinkStep(PIN_LED2, 100);
    bmp.readPressure();
    delay(70);
  }

  double total = 0;
  for (int i = 0; i < JUMLAH_SAMPEL_TEKANAN; i++) {
    blinkStep(PIN_LED2, 100);
    total += bmp.readPressure();
    delay(70);
  }
  tekananReferensi = (total / JUMLAH_SAMPEL_TEKANAN) / 100.0;   // titik 0 m
  tekananFiltered  = tekananReferensi;
  kalmanTekanan.init(tekananReferensi, KALMAN_Q, KALMAN_R);

  Serial.print("Tekanan referensi: ");
  Serial.print(tekananReferensi, 2);
  Serial.println(" hPa (ketinggian = 0 m)");
}

// ---------- 3.10 IMU : Quaternion + koreksi accelerometer ----------
void updateQuaternion(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt) {
  gx *= PI / 180.0f;
  gy *= PI / 180.0f;
  gz *= PI / 180.0f;

  float aNorm = sqrtf(ax * ax + ay * ay + az * az);
  if (aNorm > 1.0e-6f) {
    ax /= aNorm; ay /= aNorm; az /= aNorm;

    float vx = 2.0f * (qx * qz - qw * qy);
    float vy = 2.0f * (qw * qx + qy * qz);
    float vz = qw * qw - qx * qx - qy * qy + qz * qz;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    const float Kp = 1.0f;
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  const float oldW = qw, oldX = qx, oldY = qy, oldZ = qz;

  qw += 0.5f * (-oldX * gx - oldY * gy - oldZ * gz) * dt;
  qx += 0.5f * ( oldW * gx + oldY * gz - oldZ * gy) * dt;
  qy += 0.5f * ( oldW * gy - oldX * gz + oldZ * gx) * dt;
  qz += 0.5f * ( oldW * gz + oldX * gy - oldY * gx) * dt;

  float qNorm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  if (qNorm > 1.0e-6f) {
    qw /= qNorm; qx /= qNorm; qy /= qNorm; qz /= qNorm;
  }
}

void quaternionKeEuler() {
  const float radToDeg = 180.0f / PI;

  float sinr = 2.0f * (qw * qx + qy * qz);
  float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
  currentRoll = atan2f(sinr, cosr) * radToDeg;

  float sinp = 2.0f * (qw * qy - qz * qx);
  if (sinp >  1.0f) sinp =  1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  currentPitch = asinf(sinp) * radToDeg;

  float siny = 2.0f * (qw * qz + qx * qy);
  float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
  currentYaw = atan2f(siny, cosy) * radToDeg;

  if (currentYaw >= 360.0f) currentYaw -= 360.0f;
  if (currentYaw <  0.0f)   currentYaw += 360.0f;

  roll  = currentRoll;
  pitch = currentPitch;
  yaw   = currentYaw;
}

void updateIMU() {
  if (!mpuOK) return;

  unsigned long now = micros();
  float dt = (now - lastTimeIMU) / 1000000.0f;
  if (dt < MPU_DT_MIN) return;
  lastTimeIMU = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  if (!mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
    if (++mpuFailCount >= FAIL_LIMIT && mpuOK) {
      mpuOK = false;
      Serial.println("[ERROR] MPU6050 tidak merespons - LED1 akan kedip. Telemetri tetap jalan.");
    }
    return;
  }
  mpuFailCount = 0;

  float ax = (float)axRaw / MPU6050_ACCEL_SCALE;
  float ay = (float)ayRaw / MPU6050_ACCEL_SCALE;
  float az = (float)azRaw / MPU6050_ACCEL_SCALE;

  float gx = (float)gxRaw / MPU6050_GYRO_SCALE - offsetGyroX;
  float gy = (float)gyRaw / MPU6050_GYRO_SCALE - offsetGyroY;
  float gz = (float)gzRaw / MPU6050_GYRO_SCALE - offsetGyroZ;

  if (fabsf(gx) < DEADBAND_DPS) gx = 0.0f;
  if (fabsf(gy) < DEADBAND_DPS) gy = 0.0f;
  if (fabsf(gz) < DEADBAND_DPS) gz = 0.0f;

  updateQuaternion(gx, gy, gz, ax, ay, az, dt);
  quaternionKeEuler();

  if (DEBUG_SERIAL) {
    static unsigned long tPrintIMU = 0;
    if (millis() - tPrintIMU >= 250) {
      tPrintIMU = millis();
      Serial.printf("[IMU] Q=%.4f,%.4f,%.4f,%.4f | R=%.2f P=%.2f Y=%.2f\n",
                    qw, qx, qy, qz, roll, pitch, yaw);
    }
  }
}

// ---------- 3.11 Barometer ----------
void updateBaro() {
  if (!bmpOK) {
    ketinggian = 0;
    return;
  }

  float tekananMentah = bmp.readPressure();

  if (isnan(tekananMentah) || tekananMentah < 1000.0f || tekananMentah > 120000.0f) {
    if (++bmpFailCount >= FAIL_LIMIT && bmpOK) {
      bmpOK = false;
      Serial.println("[ERROR] BMP280 nilai tidak valid - LED2 akan kedip. Telemetri tetap jalan.");
    }
    return;
  }
  bmpFailCount = 0;

  tekananMentah /= 100.0f;   // Pa -> hPa
  tekananFiltered = kalmanTekanan.update(tekananMentah);
  ketinggian = 44330.0f * (1.0f - powf(tekananFiltered / tekananReferensi, 0.1903f));
}

// ---------- 3.12 ADC ----------
float bacaVoltPin(int pin) {
  uint32_t total = 0;
  for (int i = 0; i < 16; i++) total += analogReadMilliVolts(pin);
  return (total / 16.0f) / 1000.0f;
}

void updateADC() {
  tegangan = bacaVoltPin(PIN_ADC_VOLT) * RASIO_PEMBAGI_VOLT;
  arus = (bacaVoltPin(PIN_ADC_ARUS) - ARUS_OFFSET_V) / ARUS_SENSITIVITAS_VA;
  if (arus < 0) arus = 0;
}

// ---------- 3.13 Detonator ----------
// ENABLE_DETONATOR = false -> simulasi, pin tetap LOW (aman di meja)
// ENABLE_DETONATOR = true  -> pin NYALA selama DETONATOR_BURN_MS
//                             (independen dari HALT_ON_SENSOR_FAIL)
void fireDetonator1() {
  if (!ENABLE_DETONATOR) {
    Serial.println("[SIMULASI] BOM DROP terdeteksi - aktuator LOW (ENABLE_DETONATOR=false)");
    digitalWrite(PIN_DETONATOR1, LOW);
    return;
  }
  Serial.println("[FIRE] BOM DROP - detonator 1 NYALA");
  digitalWrite(PIN_DETONATOR1, HIGH);
  det1Start = millis();
  det1Aktif = true;
}

void fireDetonator2() {
  if (!ENABLE_DETONATOR) {
    Serial.println("[SIMULASI] LANDED terdeteksi - aktuator LOW (ENABLE_DETONATOR=false)");
    digitalWrite(PIN_DETONATOR2, LOW);
    return;
  }
  Serial.println("[FIRE] LANDED - detonator 2 NYALA");
  digitalWrite(PIN_DETONATOR2, HIGH);
  det2Start = millis();
  det2Aktif = true;
}

void updateDetonator() {
  if (det1Aktif && millis() - det1Start >= DETONATOR_BURN_MS) {
    digitalWrite(PIN_DETONATOR1, LOW);
    det1Aktif = false;
  }
  if (det2Aktif && millis() - det2Start >= DETONATOR_BURN_MS) {
    digitalWrite(PIN_DETONATOR2, LOW);
    det2Aktif = false;
  }
}

// ---------- 3.14 State machine ----------
void setState(State s) {
  if (state == s) return;
  state = s;
  waktuStateMulai = millis();
  Serial.print("[STATE] -> ");
  Serial.println(STATE_NAME[state]);
}

void updateState() {
  if (!bmpOK || !isfinite(ketinggian)) return;

  if (ketinggian > maxKetinggian) maxKetinggian = ketinggian;

  switch (state) {
    case ST_SETUP:
      if (ketinggian > ALT_TERBANG_M) {
        pernahTerbang = true;
        setState(ST_TERBANG);
      }
      break;

    case ST_TERBANG:
      if (maxKetinggian > ALT_JATUH_M &&
          ketinggian <= maxKetinggian - TURUN_APOGEE_M) {
        pernahJatuh = true;
        setState(ST_JATUH);
      }
      break;

    case ST_JATUH:
      if (pernahTerbang && pernahJatuh && !bombDropEvent &&
          ketinggian <= ALT_BOM_DROP_M) {
        bombDropEvent = true;
        Serial.println("[EVENT] BOM DROP threshold tercapai (<= 15 m).");
        fireDetonator1();
      }

      if (millis() - tLanded >= 1000) {
        tLanded = millis();
        if (ketinggian <= ALT_MENDARAT_M &&
            fabsf(ketinggian - landedAltLast) < 0.5f) {
          landedCount++;
        } else {
          landedCount = 0;
        }
        landedAltLast = ketinggian;

        if (pernahTerbang && pernahJatuh && landedCount >= JUMLAH_SAMPEL_MENDARAT) {
          setState(ST_MENDARAT);
          if (!botDeployEvent) {
            botDeployEvent = true;
            Serial.println("[EVENT] Mendarat terdeteksi.");
            fireDetonator2();
          }
        }
      }
      break;

    case ST_MENDARAT:
      break;
  }
}

// ---------- 3.15 Telemetri ----------
// Paket binary 30 byte. Field lat & lon sudah ada (byte 16-23),
// diisi dari gps.location.lat() dan gps.location.lng().
struct __attribute__((packed)) PaketTelemetri {
  uint32_t detikMisi;
  uint16_t packetId;
  uint8_t  teamId;
  uint16_t tekanan;
  int16_t  ketinggian;
  uint16_t tegangan;
  int16_t  arus;
  uint8_t  state;
  int32_t  lat;          // <-- koordinat GPS latitude  x 1e5
  int32_t  lon;          // <-- koordinat GPS longitude x 1e5
  int16_t  roll;
  int16_t  pitch;
  int16_t  yaw;
};

static_assert(sizeof(PaketTelemetri) == 30, "Paket harus 30 byte (sama dengan rx.py)");

const uint16_t TIDAK_ADA_U16 = 0xFFFF;

const int16_t  TIDAK_ADA_I16 = -32768;

const int32_t  TIDAK_ADA_I32 = 0x7FFFFFFF;

uint16_t keU16(float nilai, float faktor) {
  if (isnan(nilai)) return TIDAK_ADA_U16;
  float s = roundf(nilai * faktor);
  if (s < 0) s = 0;
  if (s > 65534) s = 65534;
  return (uint16_t)s;
}

int16_t keI16(float nilai, float faktor) {
  if (isnan(nilai)) return TIDAK_ADA_I16;
  float s = roundf(nilai * faktor);
  if (s > 32767)  s = 32767;
  if (s < -32767) s = -32767;
  return (int16_t)s;
}

int32_t keI32(double nilai, double faktor) {
  if (isnan(nilai)) return TIDAK_ADA_I32;
  double s = round(nilai * faktor);
  if (s > 2147483646.0)  s = 2147483646.0;
  if (s < -2147483647.0) s = -2147483647.0;
  return (int32_t)s;
}

uint32_t hitungDetikMisi() {
  if (gps.time.isValid() && gps.time.age() < 2000) {
    uint32_t jam = (gps.time.hour() + 7) % 24;
    return jam * 3600UL + gps.time.minute() * 60UL + gps.time.second();
  }
  return millis() / 1000;
}

void kirimTelemetri() {
  bool gpsFix = gps.location.isValid();

  PaketTelemetri p;
  p.detikMisi  = hitungDetikMisi();
  p.packetId   = (uint16_t)packetId;
  p.teamId     = TEAM_ID;
  p.tekanan    = keU16(tekananFiltered, 10);
  p.ketinggian = keI16(ketinggian, 10);
  p.tegangan   = keU16(tegangan, 100);
  p.arus       = keI16(arus, 100);
  p.state      = (uint8_t)state;
  p.lat        = gpsFix ? keI32(gps.location.lat(), 1e5) : TIDAK_ADA_I32;
  p.lon        = gpsFix ? keI32(gps.location.lng(), 1e5) : TIDAK_ADA_I32;
  p.roll       = mpuOK ? keI16(roll, 10)  : TIDAK_ADA_I16;
  p.pitch      = mpuOK ? keI16(pitch, 10) : TIDAK_ADA_I16;
  p.yaw        = mpuOK ? keI16(yaw, 10)   : TIDAK_ADA_I16;

  if (DEBUG_SERIAL) {
    char info[240];
    snprintf(info, sizeof(info),
             "[TX #%u] t=%lus P=%.2f Alt=%.2f V=%.2f I=%.2f %s lat=%.5f lon=%.5f R=%.1f P=%.1f Y=%.1f | MPU=%s BMP=%s LoRa=%s",
             (unsigned)p.packetId, (unsigned long)p.detikMisi, tekananFiltered, ketinggian,
             tegangan, arus, STATE_NAME[state],
             gpsFix ? gps.location.lat() : NAN, gpsFix ? gps.location.lng() : NAN,
             roll, pitch, yaw,
             mpuOK ? "OK" : "X", bmpOK ? "OK" : "X", loraOK ? "OK" : "X");
    Serial.println(info);
  }

  if (!loraOK) return;

  txSelesai = false;
  tTxMulai  = millis();
  LoRa.beginPacket();
  LoRa.write((const uint8_t*)&p, sizeof(p));
  LoRa.endPacket(true);
  txPending = true;
  packetId++;
}

void updateLoRa() {
  if (!loraOK) return;

  if (txPending && (txSelesai || millis() - tTxMulai > 3000)) {
    txSelesai = false;
    txPending = false;
    LoRa.receive();
  }

  if (!txPending) {
    int ukuran = LoRa.parsePacket();
    if (ukuran > 0) {
      String data = "";
      while (LoRa.available()) data += (char)LoRa.read();
      Serial.print("Data Diterima: "); Serial.print(data);
      Serial.print("  RSSI: ");        Serial.println(LoRa.packetRssi());
    }
  }
}

// =====================================================================
//  BAGIAN 4 : setup() dan loop()
// =====================================================================

void setup() {
  pinMode(PIN_DETONATOR1, OUTPUT); digitalWrite(PIN_DETONATOR1, LOW);
  pinMode(PIN_DETONATOR2, OUTPUT); digitalWrite(PIN_DETONATOR2, LOW);
  pinMode(PIN_LED1, OUTPUT);       digitalWrite(PIN_LED1, LOW);
  pinMode(PIN_LED2, OUTPUT);       digitalWrite(PIN_LED2, LOW);
  pinMode(PIN_LED3, OUTPUT);       digitalWrite(PIN_LED3, LOW);

  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("=== PAYLOAD SATELIT MINI ===");

  // GPS di UART2 dengan interrupt
  Serial2.setRxBufferSize(1024);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial2.onReceive(onGpsReceive);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(50);
  delay(10);
  i2cScan();

  Serial.println("--- Inisialisasi Payload ---");
  setupMPU();
  setupBMP();
  setupLoRa();

  kalibrasiGyro();
  kalibrasiTekanan();

  setState(ST_SETUP);

  Serial.println("--- Payload siap ---");
  Serial.print("Status: MPU="); Serial.print(mpuOK ? "OK" : "GAGAL (LED kedip)");
  Serial.print("  BMP=");       Serial.print(bmpOK ? "OK" : "GAGAL (LED kedip)");
  Serial.print("  LoRa=");      Serial.println(loraOK ? "OK" : "GAGAL (LED kedip)");

  updateLedIndicators();
}

void loop() {
  updateLedIndicators();

  prosesGps();       // ambil dari ring buffer (diisi interrupt UART)
  updateIMU();

  if (millis() - tBaro >= INTERVAL_BARO_MS) {
    tBaro = millis();
    updateBaro();
    updateState();
  }

  if (millis() - tAdc >= 100) {
    tAdc = millis();
    updateADC();
  }

  updateDetonator();
  updateLoRa();

  if (millis() - tTelemetri >= INTERVAL_TELEMETRI_MS && !txPending) {
    tTelemetri = millis();
    kirimTelemetri();
  }
}