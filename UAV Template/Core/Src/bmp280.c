#include "bmp280.h"
#include <math.h>

// Calibration variables
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static float temperature = 0.0f;
static float pressure = 0.0f;
static float altitude = 0.0f;
static float raw_alt = 0.0f;
static float altitude_offset = 0.0f;
static int32_t t_fine;

static void BMP_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data) {
    uint8_t txData[2] = {reg & 0x7F, data};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, txData, 2, 50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}

static void BMP_ReadRegs(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint16_t length) {
    uint8_t txData = reg | 0x80;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &txData, 1, 50);
    HAL_SPI_Receive(hspi, data, length, 50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}

void BMP280_Init(SPI_HandleTypeDef *hspi) {
    uint8_t calib[24];
    BMP_ReadRegs(hspi, 0x88, calib, 24);
    
    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];

    // Config: Standby 0.5ms, Filter 16
    BMP_WriteReg(hspi, 0xF5, (0x00 << 5) | (0x04 << 2));
    // Ctrl_Meas: Temp oversampling 2, Press oversampling 16, Normal mode
    BMP_WriteReg(hspi, 0xF4, (0x02 << 5) | (0x05 << 2) | 0x03);
}

void BMP280_Read_Data(SPI_HandleTypeDef *hspi) {
    uint8_t data[6];
    BMP_ReadRegs(hspi, 0xF7, data, 6);
    
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    
    // Temperature compensation
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) >> 8;
    temperature /= 100.0f;
    
    // Pressure compensation
    int64_t p_var1, p_var2, p;
    p_var1 = ((int64_t)t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;
    
    if (p_var1 == 0) return; // Avoid division by zero
    
    p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
    
    pressure = (float)p / 256.0f / 100.0f; // hPa
    
    // Altitude calculation (hypsometric formula)
    raw_alt = 44330.0f * (1.0f - pow(pressure / 1013.25f, 0.1903f));
    float current_altitude = raw_alt - altitude_offset;
    
    // Exponential Moving Average (EMA) Low-Pass Filter
    static uint8_t first_read = 1;
    if (first_read) {
        altitude = current_altitude;
        first_read = 0;
    } else {
        float alpha = 0.05f; // 5% new data, 95% old data to completely kill jitter
        altitude = (alpha * current_altitude) + ((1.0f - alpha) * altitude);
    }
}

void BMP280_Calibrate_Altitude(SPI_HandleTypeDef *hspi) {
    float sum = 0.0f;
    
    // 1. Give the sensor time to start measuring after Init
    HAL_Delay(100);
    
    // 2. Do a few dummy reads to clear out initial garbage data (0 pressure -> 44330m)
    for(int i = 0; i < 5; i++) {
        BMP280_Read_Data(hspi);
        HAL_Delay(20);
    }
    
    // 3. Take 20 valid samples to build a stable average
    for(int i = 0; i < 20; i++) {
        BMP280_Read_Data(hspi);
        sum += raw_alt; // Calibrate using pure raw data to avoid filter lag
        HAL_Delay(20);
    }
    altitude_offset = sum / 20.0f;
    
    // Reset the filter state so it starts perfectly at 0.0m
    altitude = 0.0f;
}

float BMP280_GetTemp(void) { return temperature; }
float BMP280_GetPressure(void) { return pressure; }
float BMP280_GetAltitude(void) { return altitude; }

