#include <Arduino.h>
#include <Wire.h>
#include "imu.h"

#define SDA_PIN 4
#define SCL_PIN 5

#define LSM_ADDR_LOW  0x6A
#define LSM_ADDR_HIGH 0x6B
#define QMC_ADDR      0x7C

uint8_t lsmAddr = 0;
bool lsmReady = false;
bool qmcReady = false;

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);

    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom(addr, len, (uint8_t)true) != len) return false;

    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire1.read();
    }
    return true;
}

bool readReg(uint8_t addr, uint8_t reg, uint8_t &value) {
    return readRegs(addr, reg, &value, 1);
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

int16_t getInt16(const uint8_t *data) {
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)value;
}

void imuBegin() {
    Wire1.begin(SDA_PIN, SCL_PIN);

    Wire1.setClock(100000);

    // Diagnostic scan of the host I2C bus before probing the IMU.
    Serial.println("Wire1 scan (GPIO 4/5):");
    bool i2cDeviceFound = false;
    for (uint8_t address = 0x08; address <= 0x77; address++) {
        Wire1.beginTransmission(address);
        if (Wire1.endTransmission() == 0) {
            Serial.printf("  ACK 0x%02X\n", address);
            i2cDeviceFound = true;
        }
    }
    if (!i2cDeviceFound) Serial.println("  No I2C device found");

    // Tìm LSM6DSV: WHO_AM_I (0x0F) phải trả về 0x70.
    uint8_t whoAmI = 0;
    if (readReg(LSM_ADDR_HIGH, 0x0F, whoAmI) && whoAmI == 0x70) {
        lsmAddr = LSM_ADDR_HIGH;
        lsmReady = true;
    } else if (readReg(LSM_ADDR_LOW, 0x0F, whoAmI) && whoAmI == 0x70) {
        lsmAddr = LSM_ADDR_LOW;
        lsmReady = true;
    }

    if (lsmReady) {
        // Bật block data update và tự tăng địa chỉ khi đọc nhiều byte.
        writeReg(lsmAddr, 0x12, 0x44);

        // Gia tốc ±4g, gyro ±2000 dps.
        writeReg(lsmAddr, 0x17, 0x01);
        writeReg(lsmAddr, 0x15, 0x04);

        // Bật gia tốc kế và gyro ở 120 Hz.
        writeReg(lsmAddr, 0x10, 0x06);
        writeReg(lsmAddr, 0x11, 0x06);

        // Khôi phục CTRL5 nếu lần chạy trước ghi nhầm vào bank thường.
        writeReg(lsmAddr, 0x14, 0x00);

        // Chọn sensor-hub register bank (FUNC_CFG_ACCESS.SHUB_REG_ACCESS).
        writeReg(lsmAddr, 0x01, 0x40);

        // MASTER_CONFIG (0x14), bit 4 = PASS_THROUGH_MODE:
        // nối I2C chính với bus phụ SCX/SDX để truy cập QMC6309.
        bool passThroughWritten = writeReg(lsmAddr, 0x14, 0x10);
        uint8_t masterConfig = 0;
        bool passThroughRead = readReg(lsmAddr, 0x14, masterConfig);

        // Trở lại bank thanh ghi thường.
        writeReg(lsmAddr, 0x01, 0x00);
        delay(1);

        Serial.printf("QMC passthrough: write=%d, MASTER_CONFIG=0x%02X\n",
                      passThroughWritten, passThroughRead ? masterConfig : 0xFF);

        Serial.printf("LSM6DSV OK, I2C address 0x%02X\n", lsmAddr);
    } else {
        Serial.println("Khong tim thay LSM6DSV: kiem tra SDA/SCL, CS va jumper I2C.");
    }

    // QMC6309: chip ID register 0x00 phải trả về 0x90.
    uint8_t qmcId = 0;
    if (readReg(QMC_ADDR, 0x00, qmcId) && qmcId == 0x90) {
        qmcReady = true;

        // Cấu hình QMC6309 theo ví dụ trong datasheet.
        writeReg(QMC_ADDR, 0x0B, 0x40);
        writeReg(QMC_ADDR, 0x0A, 0x61);
        delay(10);

        Serial.println("QMC6309 OK, I2C address 0x7C");
    } else {
        Serial.println("Khong tim thay QMC6309; se chi doc gia toc ke va gyro.");
    }

    Serial.println("IMU,ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps,temp_C,mx_raw,my_raw,mz_raw,sample_ms");
}

void imuLoop() {
    static uint32_t lastReadMs = 0;

    if (!lsmReady) return;

    if (millis() - lastReadMs < 10) return;  // In khoảng 100 dòng/giây
    lastReadMs = millis();

    // 0x20..0x2D gồm nhiệt độ, gyro XYZ và gia tốc XYZ.
    uint8_t data[14];
    if (!readRegs(lsmAddr, 0x20, data, sizeof(data))) return;

    int16_t tempRaw = getInt16(&data[0]);
    int16_t gxRaw   = getInt16(&data[2]);
    int16_t gyRaw   = getInt16(&data[4]);
    int16_t gzRaw   = getInt16(&data[6]);
    int16_t axRaw   = getInt16(&data[8]);
    int16_t ayRaw   = getInt16(&data[10]);
    int16_t azRaw   = getInt16(&data[12]);

    // Gia tốc: ±4g, 0.122 mg/LSB. Gyro: ±2000 dps, 0.07 dps/LSB.
    float ax = axRaw * 0.000122f * 9.80665f;
    float ay = ayRaw * 0.000122f * 9.80665f;
    float az = azRaw * 0.000122f * 9.80665f;

    float gx = gxRaw * 0.07f;
    float gy = gyRaw * 0.07f;
    float gz = gzRaw * 0.07f;

    float tempC = 25.0f + tempRaw / 256.0f;

    Serial.printf("IMU,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,",
                  ax, ay, az, gx, gy, gz, tempC);

    if (qmcReady) {
        uint8_t status = 0;
        uint8_t magData[6];
        int16_t mx = 0, my = 0, mz = 0;

        if (readReg(QMC_ADDR, 0x09, status) && (status & 0x01) &&
            readRegs(QMC_ADDR, 0x01, magData, sizeof(magData))) {
            mx = getInt16(&magData[0]);
            my = getInt16(&magData[2]);
            mz = getInt16(&magData[4]);
        }

        Serial.printf("%d,%d,%d,%lu\n", mx, my, mz, (unsigned long)millis());
    } else {
        Serial.printf("NA,NA,NA,%lu\n", (unsigned long)millis());
    }
}
