#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// =====================================================
// I2C
// =====================================================

#define SDA_PIN 21
#define SCL_PIN 22

// =====================================================
// MPU6050
// =====================================================

#define MPU_ADDR 0x68

// MPU6050:
// Accelerometer = ±2g
// Gyroscope    = ±250 deg/s

float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

// =====================================================
// BMP280
// =====================================================

#define BMP_ADDR 0x76

Adafruit_BMP280 bmp;

float referencePressure = 1013.25f;

// =====================================================
// QUATERNION
// =====================================================
//
// q = [w, x, y, z]
//
// Awal:
// 1, 0, 0, 0
//

float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;

// =====================================================
// MADGWICK
// =====================================================

float beta = 0.05f;

// =====================================================
// GYRO DEADBAND
// =====================================================

const float GYRO_DEADBAND = 0.15f;

// =====================================================
// TIMER
// =====================================================

unsigned long lastMicros = 0;

// =====================================================
// WRITE REGISTER
// =====================================================

void writeRegister(byte reg, byte data)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);
    Wire.write(data);

    Wire.endTransmission();
}

// =====================================================
// READ MPU6050
// =====================================================

bool readMPU(
    int16_t &ax,
    int16_t &ay,
    int16_t &az,
    int16_t &gx,
    int16_t &gy,
    int16_t &gz
)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    uint8_t count = Wire.requestFrom(
        MPU_ADDR,
        (uint8_t)14
    );

    if (count < 14)
    {
        return false;
    }

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();

    // Temperature
    Wire.read();
    Wire.read();

    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    return true;
}

// =====================================================
// NORMALIZE QUATERNION
// =====================================================

void normalizeQuaternion()
{
    float norm = sqrt(
        qw * qw +
        qx * qx +
        qy * qy +
        qz * qz
    );

    if (norm <= 0.0f)
    {
        qw = 1.0f;
        qx = 0.0f;
        qy = 0.0f;
        qz = 0.0f;

        return;
    }

    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
}

// =====================================================
// CALIBRASI GYRO
// =====================================================

void calibrateGyro()
{
    const int samples = 2000;

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;

    Serial.println();
    Serial.println("======================================");
    Serial.println("          KALIBRASI GYRO");
    Serial.println("======================================");
    Serial.println();
    Serial.println("JANGAN GERAKKAN MPU6050!");
    Serial.println("Sensor harus benar-benar diam.");
    Serial.println();

    delay(3000);

    for (int i = 0; i < samples; i++)
    {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        if (readMPU(
            ax,
            ay,
            az,
            gx,
            gy,
            gz
        ))
        {
            sumX += gx;
            sumY += gy;
            sumZ += gz;
        }

        delay(2);
    }

    float rawX = sumX / samples;
    float rawY = sumY / samples;
    float rawZ = sumZ / samples;

    gyroOffsetX = rawX / 131.0f;
    gyroOffsetY = rawY / 131.0f;
    gyroOffsetZ = rawZ / 131.0f;

    Serial.println();
    Serial.println("Kalibrasi selesai.");

    Serial.print("Gyro offset X = ");
    Serial.print(gyroOffsetX, 4);
    Serial.println(" deg/s");

    Serial.print("Gyro offset Y = ");
    Serial.print(gyroOffsetY, 4);
    Serial.println(" deg/s");

    Serial.print("Gyro offset Z = ");
    Serial.print(gyroOffsetZ, 4);
    Serial.println(" deg/s");

    Serial.println();
}

// =====================================================
// KALIBRASI ALTITUDE
// =====================================================

void calibrateAltitude()
{
    const int samples = 100;

    double pressureSum = 0.0;

    Serial.println();
    Serial.println("======================================");
    Serial.println("        KALIBRASI KETINGGIAN");
    Serial.println("======================================");
    Serial.println("Sensor diam.");
    Serial.println();

    delay(2000);

    for (int i = 0; i < samples; i++)
    {
        float pressure =
            bmp.readPressure() / 100.0f;

        pressureSum += pressure;

        delay(20);
    }

    referencePressure =
        pressureSum / samples;

    Serial.print("Reference pressure = ");
    Serial.print(referencePressure, 2);
    Serial.println(" hPa");

    Serial.println();
}

// =====================================================
// MADGWICK AHRS
// =====================================================
//
// gx gy gz = rad/s
// ax ay az = g
//
// =====================================================

void updateMadgwick(
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float dt
)
{
    // -------------------------------------------------
    // Normalize accelerometer
    // -------------------------------------------------

    float accNorm = sqrt(
        ax * ax +
        ay * ay +
        az * az
    );

    if (accNorm <= 0.0f)
    {
        return;
    }

    ax /= accNorm;
    ay /= accNorm;
    az /= accNorm;

    // -------------------------------------------------
    // Auxiliary variables
    // -------------------------------------------------

    float q0 = qw;
    float q1 = qx;
    float q2 = qy;
    float q3 = qz;

    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;

    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;

    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // -------------------------------------------------
    // Gradient descent
    // -------------------------------------------------

    float s0 =
        _4q0 * q2q2 +
        _2q2 * ax +
        _4q0 * q1q1 -
        _2q1 * ay;

    float s1 =
        _4q1 * q3q3 -
        _2q3 * ax +
        4.0f * q0q0 * q1 -
        _2q0 * ay -
        _4q1 +
        _8q1 * q1q1 +
        _8q1 * q2q2 +
        _4q1 * az;

    float s2 =
        4.0f * q0q0 * q2 +
        _2q0 * ax +
        _4q2 * q3q3 -
        _2q3 * ay -
        _4q2 +
        _8q2 * q1q1 +
        _8q2 * q2q2 +
        _4q2 * az;

    float s3 =
        4.0f * q1q1 * q3 -
        _2q1 * ax +
        4.0f * q2q2 * q3 -
        _2q2 * ay;

    float sNorm = sqrt(
        s0 * s0 +
        s1 * s1 +
        s2 * s2 +
        s3 * s3
    );

    if (sNorm > 0.0f)
    {
        s0 /= sNorm;
        s1 /= sNorm;
        s2 /= sNorm;
        s3 /= sNorm;
    }

    // -------------------------------------------------
    // Quaternion derivative
    // -------------------------------------------------

    float qDot0 =
        0.5f *
        (
            -q1 * gx -
            q2 * gy -
            q3 * gz
        );

    float qDot1 =
        0.5f *
        (
            q0 * gx +
            q2 * gz -
            q3 * gy
        );

    float qDot2 =
        0.5f *
        (
            q0 * gy -
            q1 * gz +
            q3 * gx
        );

    float qDot3 =
        0.5f *
        (
            q0 * gz +
            q1 * gy -
            q2 * gx
        );

    // -------------------------------------------------
    // Feedback
    // -------------------------------------------------

    qDot0 -= beta * s0;
    qDot1 -= beta * s1;
    qDot2 -= beta * s2;
    qDot3 -= beta * s3;

    // -------------------------------------------------
    // Integrate
    // -------------------------------------------------

    qw += qDot0 * dt;
    qx += qDot1 * dt;
    qy += qDot2 * dt;
    qz += qDot3 * dt;

    // -------------------------------------------------
    // Normalize
    // -------------------------------------------------

    normalizeQuaternion();
}

// =====================================================
// ALTITUDE
// =====================================================

float getRelativeAltitude(float pressure)
{
    return 44330.0f *
           (
               1.0f -
               pow(
                   pressure / referencePressure,
                   0.1903f
               )
           );
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Wire.begin(
        SDA_PIN,
        SCL_PIN
    );

    Wire.setClock(400000);

    Serial.println();
    Serial.println("======================================");
    Serial.println("       ESP32 ROCKET SENSOR");
    Serial.println("        MPU6050 + BMP280");
    Serial.println("          QUATERNION AHRS");
    Serial.println("======================================");

    // =================================================
    // MPU6050
    // =================================================

    Serial.println();
    Serial.println("Initializing MPU6050...");

    // Wake up
    writeRegister(
        0x6B,
        0x00
    );

    // Accelerometer ±2g
    writeRegister(
        0x1C,
        0x00
    );

    // Gyroscope ±250 deg/s
    writeRegister(
        0x1B,
        0x00
    );

    delay(100);

    Serial.println("MPU6050 OK.");

    // =================================================
    // BMP280
    // =================================================

    Serial.println();
    Serial.println("Initializing BMP280...");

    if (!bmp.begin(BMP_ADDR))
    {
        Serial.println(
            "BMP280 tidak ditemukan pada 0x76."
        );

        Serial.println(
            "Mencoba 0x77..."
        );

        if (!bmp.begin(0x77))
        {
            Serial.println(
                "BMP280 ERROR!"
            );

            while (true)
            {
                delay(1000);
            }
        }

        Serial.println(
            "BMP280 ditemukan pada 0x77."
        );
    }
    else
    {
        Serial.println(
            "BMP280 ditemukan pada 0x76."
        );
    }

    // =================================================
    // BMP280 SETTINGS
    // =================================================

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,

        Adafruit_BMP280::SAMPLING_X2,

        Adafruit_BMP280::SAMPLING_X16,

        Adafruit_BMP280::FILTER_X16,

        Adafruit_BMP280::STANDBY_MS_63
    );

    // =================================================
    // CALIBRATION
    // =================================================

    calibrateGyro();

    calibrateAltitude();

    // =================================================
    // RESET QUATERNION
    // =================================================

    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;

    lastMicros = micros();

    Serial.println();
    Serial.println("======================================");
    Serial.println("        SYSTEM READY");
    Serial.println("======================================");
    Serial.println();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    // =================================================
    // READ MPU
    // =================================================

    if (!readMPU(
        ax,
        ay,
        az,
        gx,
        gy,
        gz
    ))
    {
        return;
    }

    // =================================================
    // ACCELEROMETER
    // =================================================

    float ax_g =
        ax / 16384.0f;

    float ay_g =
        ay / 16384.0f;

    float az_g =
        az / 16384.0f;

    // =================================================
    // GYRO
    // =================================================

    float gx_dps =
        (gx / 131.0f) -
        gyroOffsetX;

    float gy_dps =
        (gy / 131.0f) -
        gyroOffsetY;

    float gz_dps =
        (gz / 131.0f) -
        gyroOffsetZ;

    // =================================================
    // DEAD BAND
    // =================================================

    if (fabs(gx_dps) < GYRO_DEADBAND)
    {
        gx_dps = 0.0f;
    }

    if (fabs(gy_dps) < GYRO_DEADBAND)
    {
        gy_dps = 0.0f;
    }

    if (fabs(gz_dps) < GYRO_DEADBAND)
    {
        gz_dps = 0.0f;
    }

    // =================================================
    // DEG/S -> RAD/S
    // =================================================

    float gx_rad =
        gx_dps * DEG_TO_RAD;

    float gy_rad =
        gy_dps * DEG_TO_RAD;

    float gz_rad =
        gz_dps * DEG_TO_RAD;

    // =================================================
    // DT
    // =================================================

    unsigned long now =
        micros();

    float dt =
        (now - lastMicros) /
        1000000.0f;

    lastMicros = now;

    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f;
    }

    // =================================================
    // MADGWICK
    // =================================================

    updateMadgwick(
        gx_rad,
        gy_rad,
        gz_rad,
        ax_g,
        ay_g,
        az_g,
        dt
    );

    // =================================================
    // BMP280
    // =================================================

    float temperature =
        bmp.readTemperature();

    float pressure =
        bmp.readPressure() / 100.0f;

    float altitude =
        getRelativeAltitude(
            pressure
        );

    // =================================================
    // SERIAL OUTPUT
    // =================================================
    //
    // Python membaca BARIS ini.
    //
    // Jangan ubah formatnya.
    //

    Serial.print("W: ");
    Serial.print(qw, 6);

    Serial.print(" | X: ");
    Serial.print(qx, 6);

    Serial.print(" | Y: ");
    Serial.print(qy, 6);

    Serial.print(" | Z: ");
    Serial.println(qz, 6);

    // Debug sensor setiap ~500 ms
    static unsigned long lastDebug = 0;

    if (millis() - lastDebug > 500)
    {
        lastDebug = millis();

        Serial.print("GYRO: ");
        Serial.print(gx_dps, 3);

        Serial.print(" | ");
        Serial.print(gy_dps, 3);

        Serial.print(" | ");
        Serial.println(gz_dps, 3);
    }

    delay(5);
}