#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include "imu.h"

#define LED_PIN 48
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define SEA_LEVEL_HPA 1013.25f

Adafruit_BMP3XX bmp;
bool bmpReady = false;
uint32_t lastReadMs = 0;

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(20);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);
    imuBegin();

    // Thử hai địa chỉ I2C phổ biến của BMP388
    bmpReady = bmp.begin_I2C(0x76, &Wire);
    if (!bmpReady) {
        bmpReady = bmp.begin_I2C(0x77, &Wire);
    }

    if (!bmpReady) {
        Serial.println("Khong tim thay BMP388. Kiem tra SDA/SCL va dia chi.");
        return;
    }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

    Serial.println("BMP,temp_C,pressure_hPa,altitude_m");
}

void loop() {
    imuLoop();
    // Đọc cảm biến mỗi 500 ms
    if (bmpReady && millis() - lastReadMs >= 500) {
        lastReadMs = millis();

        if (bmp.performReading()) {
            float temperatureC = bmp.temperature;
            float pressureHpa = bmp.pressure / 100.0f;
            float altitudeM = bmp.readAltitude(SEA_LEVEL_HPA);

            Serial.printf("BMP,%.2f,%.2f,%.2f\n",
                          temperatureC, pressureHpa, altitudeM);
        } else {
            Serial.println("Loi doc BMP388");
        }
    }
}
