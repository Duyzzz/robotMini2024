/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "i2c.h"


// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
    double De_Gx; // Tổng lấy mẫu 400 lần / 400
    double De_Gy;
    double De_Gz;

    double De_Ax;
    double De_Ay;
    double De_Az;

    double Num_Z[400]; // Chứa 400 lần lấy mẫu Az
    double Num_X[400];
    double Num_Y[400];

    double Num_Gz[400];

    double DeltaGyro_Z[3];

    double Gyro_Z[3]; // Lưu giá trị góc Z tại 3 thời điểm hiện tại, trước đó 1, trước đó 2
    /* Hiệu = Angle - Angle1
     * nếu hiện tại và trước đó 1 trái dấu => tăng bộ hiệu chỉnh:

     Mẫu 1 = hiện tại - trước đó 1 càng lớn thì càng bù nhiều Angle0 = 150 Angle1 = - 25 Angle2 = -50
     nghĩa là đang giảm dần về 0 nhưng đỏi chiều quay còn trường hợp tăng dần nhưng đảo chiều quay luôn
     xét dấu của Angle1 và Angle2 cùng dấu
     abs(Angle1) > abs(Angle2) -> đang quay nhanh xong đảo chiều -> Hiệu lớn > đảo chiều gấp , hiệu nhỏ đảo chiều ít
     abs(Angle1) < abs(Angle2) -> gần dừng nhưngfsr
     */


    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
    double KalmanAngleZ;
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Balance_Data(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

void Filter_getAngleZ(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
