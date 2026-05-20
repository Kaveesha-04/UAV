#include "mpu6500.h"
#include <math.h>

extern float Get_Mag_Heading(void);

#define RAD_TO_DEG 57.2957795131f
#define LPF_ALPHA 0.2f

float Roll = 0.0f, Pitch = 0.0f, Yaw = 0.0f;
float Filtered_Accel_X = 0.0f, Filtered_Accel_Y = 0.0f, Filtered_Accel_Z = 0.0f;

// --- අලුත් Offset විචල්‍යයන් ---
float GyroX_Offset = 0, GyroY_Offset = 0, GyroZ_Offset = 0;
float Accel_Roll_Offset = 0, Accel_Pitch_Offset = 0;

// SPI Helper Functions (ඔයාගේ කේතයේ තිබ්බ ඒවාමයි)
static void MPU_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data) {
    uint8_t txData[2] = {reg & 0x7F, data};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, txData, 2, 50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

static void MPU_ReadRegs(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint16_t length) {
    uint8_t txData = reg | 0x80;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &txData, 1, 50);
    HAL_SPI_Receive(hspi, data, length, 50);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

void MPU6500_Init(SPI_HandleTypeDef *hspi) {
    MPU_WriteReg(hspi, 0x6B, 0x00);
    HAL_Delay(10);
    
    // Enable Hardware Digital Low-Pass Filter (DLPF)
    // 0x03 sets filter to ~41Hz (removes motor vibrations but keeps PID responsive)
    MPU_WriteReg(hspi, 0x1A, 0x03); 
    
    MPU_WriteReg(hspi, 0x1B, 0x08); // 500 dps
    MPU_WriteReg(hspi, 0x1C, 0x10); // 8g
}

// --- නිවැරදි කළ Calibration Function එක ---
void MPU6500_Calibrate(SPI_HandleTypeDef *hspi) {
    float gx_s = 0, gy_s = 0, gz_s = 0;
    float ar_s = 0, ap_s = 0;
    int samples = 500;

    for (int i = 0; i < samples; i++) {
        uint8_t Rec_Data[14];
        MPU_ReadRegs(hspi, 0x3B, Rec_Data, 14);

        int16_t gX_raw = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
        int16_t gY_raw = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
        int16_t gZ_raw = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

        gx_s += gX_raw / 65.5f;
        gy_s += gY_raw / 65.5f;
        gz_s += gZ_raw / 65.5f;

        // Accelerometer ඇලයත් මැනගමු
        int16_t aX = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        int16_t aY = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        int16_t aZ = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        ar_s += atan2(aY, aZ) * RAD_TO_DEG;
        ap_s += atan2(-aX, sqrt(aY*aY + aZ*aZ)) * RAD_TO_DEG;

        HAL_Delay(1);
    }
    GyroX_Offset = gx_s / samples;
    GyroY_Offset = gy_s / samples;
    GyroZ_Offset = gz_s / samples;
    Accel_Roll_Offset = ar_s / samples;
    Accel_Pitch_Offset = ap_s / samples;
}

void MPU6500_Read_Angles(SPI_HandleTypeDef *hspi, float dt) {
    uint8_t Rec_Data[14];
    MPU_ReadRegs(hspi, 0x3B, Rec_Data, 14);

    int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    int16_t Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    int16_t Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    int16_t Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // 1. Gyro Bias ඉවත් කිරීම
    float Gx = (Gyro_X_RAW / 65.5f) - GyroX_Offset;
    float Gy = (Gyro_Y_RAW / 65.5f) - GyroY_Offset;
    float Gz = (Gyro_Z_RAW / 65.5f) - GyroZ_Offset;

    // 2. Accel Low-pass filter
    Filtered_Accel_X = (LPF_ALPHA * (Accel_X_RAW / 4096.0f)) + ((1.0f - LPF_ALPHA) * Filtered_Accel_X);
    Filtered_Accel_Y = (LPF_ALPHA * (Accel_Y_RAW / 4096.0f)) + ((1.0f - LPF_ALPHA) * Filtered_Accel_Y);
    Filtered_Accel_Z = (LPF_ALPHA * (Accel_Z_RAW / 4096.0f)) + ((1.0f - LPF_ALPHA) * Filtered_Accel_Z);

    // 3. Accel කෝණ සොයා Offset එක අඩු කිරීම
    float Accel_Roll  = (atan2(Filtered_Accel_Y, Filtered_Accel_Z) * RAD_TO_DEG) - Accel_Roll_Offset;
    float Accel_Pitch = (atan2(-Filtered_Accel_X, sqrt(Filtered_Accel_Y*Filtered_Accel_Y + Filtered_Accel_Z*Filtered_Accel_Z)) * RAD_TO_DEG) - Accel_Pitch_Offset;

    // 4. Complementary Filter for Roll and Pitch
    Roll  = 0.98f * (Roll + Gx * dt) + 0.02f * Accel_Roll;
    Pitch = 0.98f * (Pitch + Gy * dt) + 0.02f * Accel_Pitch;
    
    // 5. Absolute Yaw Filter using Magnetometer Heading
    extern uint8_t mag_type;
    
    // Normalize Gyro Yaw to 0..360 range
    while (Yaw >= 360.0f) Yaw -= 360.0f;
    while (Yaw < 0.0f) Yaw += 360.0f;
    
    if (mag_type != 0) {
        float mag_heading = Get_Mag_Heading();
        // Find shortest angular path
        float yaw_error = mag_heading - Yaw;
        if (yaw_error > 180.0f) yaw_error -= 360.0f;
        else if (yaw_error < -180.0f) yaw_error += 360.0f;
        
        // Apply correction (98% Gyro, 2% Mag)
        Yaw += (Gz * dt) + (0.02f * yaw_error);
    } else {
        // No magnetometer: Just integrate Gyro Z (will slowly drift)
        Yaw += (Gz * dt);
    }
    
    // Final normalization
    if (Yaw >= 360.0f) Yaw -= 360.0f;
    if (Yaw < 0.0f) Yaw += 360.0f;
}

float MPU6500_GetRoll()  { return Roll; }
float MPU6500_GetPitch() { return Pitch; }
float MPU6500_GetYaw()   { return Yaw; }
