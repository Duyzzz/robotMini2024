/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */


#include <math.h>
#include "mpu6050.h"
#include "SimpleKalmanFilter.h"
#include "bool.h"

#define RAD_TO_DEG 57.295779513082320876798154814105 // 180 / pi
#define DEG_TO_RAD 1/RAD_TO_DEG

#define WHO_AM_I_REG 0x75 /*
Thanh ghi để đọc giá trị nhận diện của cảm biến, thường dùng
để kiểm tra xem thiết bị có đang hoạt động đúng hay không

*/
#define PWR_MGMT_1_REG 0x6B
/*
Thanh ghi quản lý nguồn, dùng để bật/tắt các bộ phận của cảm biến. clock ....
reset device MPU về giá trị mặc định
tắt cảm biến nhiệt độ
 */
#define SMPLRT_DIV_REG 0x19
/*
  Thanh ghi để chia tần số lấy mẫu.

 */
#define ACCEL_CONFIG_REG 0x1C // Thanh ghi cấu hình gia tốc kế, dùng để thiết lập phạm vi đo và các cấu hình khác.
#define ACCEL_XOUT_H_REG 0x3B // Thanh ghi lưu giá trị gia tốc theo trục X, byte cao.
#define TEMP_OUT_H_REG 0x41 // Thanh ghi lưu giá trị nhiệt độ, byte cao.
#define GYRO_CONFIG_REG 0x1B // Thanh ghi cấu hình con quay hồi chuyển, dùng để thiết lập phạm vi đo và các cấu hình khác.
#define GYRO_XOUT_H_REG 0x43 // Thanh ghi lưu giá trị góc quay theo trục X, byte cao.
#define GYRO_ZOUT_H_REG 0x47
#define GYRO_ZOUT_L_REG 0x48
// Setup MPU6050
#define MPU6050_ADDR 0xD0
/*
 ở trên tài liệu sẽ là 0x68 vì 01101000
 nhưng để thuận tiện trên hàm HAL phần địa chỉ dịch trái 1 bit
 nên sẽ thành 11010000 = 0xD0
 */
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0; // bù trừ của gia tốc kế trục Z

double kp, ki ,kd;
double error,last_error, balance_angleZ;



uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};
Kalman_t KalmanZ = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

extern SimpleKalmanFilter Filter_Ax;
extern SimpleKalmanFilter Filter_Ay;
extern SimpleKalmanFilter Filter_Az;
extern SimpleKalmanFilter Filter_Gz;
extern SimpleKalmanFilter Filter_Gy;
extern SimpleKalmanFilter Filter_Gx;
extern MPU6050_t MPU6050;




uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);


    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
    	Data = 0x80;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
		HAL_Delay(100);
    	// thanh ghi quản lý nguồn 0X6B chúng ta nên ghi tất cả số 0 để đánh thức cảm biến
		Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Đặt TỐC ĐỘ DỮ LIỆU là 1KHz bằng cách ghi thanh ghi SMPLRT_DIV 8/(7+1)
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g 4g 6g 8g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x18;
        //Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        return 104;
    }
    return check;
}


void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}


void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}

bool st = true;
bool st_math = true;
void Filter_getAngleZ(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct){
	uint8_t Rec_Data[14];
	int16_t temp;

	// Read 14 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

	DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
	temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
	DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
	DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
	DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

	// cập nhật giá trị mảng

		// Acc X, Y ,Z

	// Đọc giá trị thô

	DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0 - DataStruct->Num_X[0];
	DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0 - DataStruct->Num_Y[0];
	DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector - DataStruct->Num_Z[0];
	DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
	DataStruct->Gx = DataStruct->Gyro_X_RAW / 16.4;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / 16.4;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / 16.4;


	/// Filter làm mịn giá trị

	DataStruct->Ax = SimpleKalmanFilter_UpdateEstimate(&Filter_Ax, DataStruct->Ax);
	DataStruct->Ay = SimpleKalmanFilter_UpdateEstimate(&Filter_Ay, DataStruct->Ay);
	DataStruct->Az = SimpleKalmanFilter_UpdateEstimate(&Filter_Az, DataStruct->Az);

	DataStruct->Gx = SimpleKalmanFilter_UpdateEstimate(&Filter_Gx, DataStruct->Gx - DataStruct->De_Gx);
//	DataStruct->Gx = SimpleKalmanFilter_UpdateEstimate(&Filter_Gx, DataStruct->Gx);

	DataStruct->Gy = SimpleKalmanFilter_UpdateEstimate(&Filter_Gy, DataStruct->Gy - DataStruct->De_Gy);
//	DataStruct->Gy = SimpleKalmanFilter_UpdateEstimate(&Filter_Gy, DataStruct->Gy);

	DataStruct->Gz = SimpleKalmanFilter_UpdateEstimate(&Filter_Gz, DataStruct->Gz - DataStruct->De_Gz);

	DataStruct->Gyro_Z[0] = DataStruct->Gz;


	// Kalman angle solve
	double dt = (double) (HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();
	st_math = true;

	double roll;
	double roll_sqrt = sqrt(
			DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
	if (roll_sqrt != 0.0) {
		roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
	} else {
		roll = 0.0;
	}
	double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
	if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
		KalmanY.angle = pitch;
		DataStruct->KalmanAngleY = pitch;
	} else {
		DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch,DataStruct->Gy, dt);
	}
	if (fabs(DataStruct->KalmanAngleY) > 90)
		DataStruct->Gx = -DataStruct->Gx;
	DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);


	double sin_roll = sin(DataStruct->KalmanAngleX * DEG_TO_RAD);
	double cos_roll = cos(DataStruct->KalmanAngleX * DEG_TO_RAD);
	double sin_pitch = sin(DataStruct->KalmanAngleY * DEG_TO_RAD);
	double cos_pitch = cos(DataStruct->KalmanAngleY * DEG_TO_RAD);

	// Sử dụng công thức tính yaw hiệu chỉnh dựa trên roll, pitch và yaw
	double yaw_corrected = atan2(sin_roll * sin_pitch * cos_pitch + cos_roll * sin_pitch, cos_pitch) * RAD_TO_DEG;
	double yaw_correctedd = atan2(sin_pitch * sin_roll * cos_roll + cos_pitch * sin_roll, cos_roll)* RAD_TO_DEG;

	DataStruct->Gyro_Z[2] = yaw_corrected;
	DataStruct->Gyro_Z[1] = yaw_correctedd;
	DataStruct->DeltaGyro_Z[0] = (DataStruct->Gyro_Z[1] + DataStruct->Gyro_Z[2]) ;

	// tính toán

	if(DataStruct->DeltaGyro_Z[0] <= 10.0 && st_math){

		//DataStruct->KalmanAngleZ += DataStruct->Gyro_Z[0] * dt;
		if(fabs(DataStruct->Gyro_Z[0]) > 0.5){
			DataStruct->KalmanAngleZ += ( DataStruct->Gyro_Z[0] + DataStruct->Gyro_Z[0] * DataStruct->DeltaGyro_Z[0] / 150.0) * dt;

		}else DataStruct->KalmanAngleZ += ( DataStruct->Gyro_Z[0] - DataStruct->Gyro_Z[0] * DataStruct->DeltaGyro_Z[0] / 121.0) * dt;

		st_math = false;
	}else if(DataStruct->DeltaGyro_Z[0] > 10.0 && DataStruct->DeltaGyro_Z[0] <= 50.0 && st_math){

		if(fabs(DataStruct->Gyro_Z[0]) > 0.5)

			DataStruct->KalmanAngleZ += ( DataStruct->Gyro_Z[0] + DataStruct->Gyro_Z[0] * DataStruct->DeltaGyro_Z[0] / 131.005) * dt;

		else DataStruct->KalmanAngleZ += ( DataStruct->Gyro_Z[0] - DataStruct->Gyro_Z[0] * DataStruct->DeltaGyro_Z[0] / 121.0) * dt;

		st_math = false;

	}else if(DataStruct->DeltaGyro_Z[0] > 50.0 && st_math){

		if(fabs(DataStruct->Gyro_Z[0]) > 0.5)

			DataStruct->KalmanAngleZ += ( DataStruct->Gyro_Z[0] + DataStruct->Gyro_Z[0] * DataStruct->DeltaGyro_Z[0] / 85.0) * dt;

		else DataStruct->KalmanAngleZ += ( DataStruct->Gyro_Z[0] - DataStruct->Gyro_Z[0] * DataStruct->DeltaGyro_Z[0] / 121.0) * dt;

		st_math = false;
	}

	// khởi tạo đầu khi tính toán
	if(st){
		DataStruct->KalmanAngleZ = 0;
		st = false;

	}

}

//// --------------------- Lấy mẫu ban đầu 4s ------------------------------ ////

void MPU6050_Balance_Data(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct){
	uint8_t Rec_Data[14];
	int16_t temp;

	// Read 14 BYTES of data starting from ACCEL_XOUT_H register

	int i = 0;
	for(i = 0;i < 400;i++){
		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

		DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
		DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
		DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
		temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
		DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
		DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
		DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

		DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
		DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
		DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
		DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
		DataStruct->Gx = DataStruct->Gyro_X_RAW / 16.4;
		DataStruct->Gy = DataStruct->Gyro_Y_RAW / 16.4;
		DataStruct->Gz = DataStruct->Gyro_Z_RAW / 16.4;

		DataStruct->De_Gx += DataStruct->Gx;
		DataStruct->De_Gy += DataStruct->Gy;
		DataStruct->De_Gz += DataStruct->Gz;
		DataStruct->De_Ax += DataStruct->Ax;
		DataStruct->De_Ay += DataStruct->Ay;
		DataStruct->De_Az += DataStruct->Az;

		DataStruct->Num_Z[i] = DataStruct->Az;
		DataStruct->Num_Y[i] = DataStruct->Ay;
		DataStruct->Num_X[i] = DataStruct->Ax;
		DataStruct->Num_Gz[i] = DataStruct->Gz;

		HAL_Delay(10);
	}
	DataStruct->De_Gx = DataStruct->De_Gx / 400.0;
	DataStruct->De_Gy = DataStruct->De_Gy / 400.0;
	DataStruct->De_Gz = DataStruct->De_Gz / 400.0;
	DataStruct->De_Ax = DataStruct->De_Ax / 400.0;
	DataStruct->De_Ay = DataStruct->De_Ay / 400.0;
	DataStruct->De_Az = DataStruct->De_Az / 400.0;

	for(i = 0; i < 400; i++){
		DataStruct->Num_Z[i] = DataStruct->Num_Z[i] - DataStruct->De_Az;
		DataStruct->Num_Y[i] = DataStruct->Num_Y[i] - DataStruct->De_Ay;
		DataStruct->Num_X[i] = DataStruct->Num_X[i] - DataStruct->De_Ax;
		DataStruct->Num_Gz[i] = DataStruct->Num_Gz[i] - DataStruct->De_Gz;
	}

	for(i = 1;i < 400;i++){
					DataStruct->Num_Z[0] += DataStruct->Num_Z[i];
					DataStruct->Num_Y[0] += DataStruct->Num_Y[i];
					DataStruct->Num_X[0] += DataStruct->Num_X[i];
					DataStruct->Num_Gz[0] += DataStruct->Num_Gz[i];

		}
	DataStruct->Num_Z[0] = DataStruct->Num_Z[0] / 400.0;
	DataStruct->Num_Y[0] = DataStruct->Num_Y[0] / 400.0;
	DataStruct->Num_X[0] = DataStruct->Num_X[0] / 400.0;
	DataStruct->Num_Gz[0] = DataStruct->Num_Gz[0] / 400.0;


}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};


