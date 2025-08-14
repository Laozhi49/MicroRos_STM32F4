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
 * | Copyright (C) Bulanov Konstantin,2021
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
//#include "my_math.h"
#define RAD_TO_DEG 57.295779513082320876798154814105		//用以弧度转角度，角度 = 弧度 × (180/π)，其中 180/π 的值约为 57.2958

#define WHO_AM_I_REG 				0x75				// MPU6050 的 WHO_AM_I 寄存器地址。通过读取该寄存器的值，如果返回 0x68，则表示该 I2C 设备是 MPU6050 传感器。
#define PWR_MGMT_1_REG 				0x6B  			// MPU6050 的 Power Management 1 寄存器地址 (电源管理 1 寄存器)，用于配置电源模式、时钟源选择、设备复位和睡眠模式等功能。
#define SMPLRT_DIV_REG 				0x19     		// MPU6050 的 Sample Rate Divider 寄存器地址 (采样速率分频器)，通过该寄存器设置分频系数，配置 MPU6050 数据输出速率。
#define ACCEL_CONFIG_REG 			0x1C     		// MPU6050 的 ACCEL_CONFIG 寄存器地址 (加速度计配置寄存器)，用于设置加速度计的量程范围（例如 ±2g、±4g 等）。
#define ACCEL_XOUT_H_REG 			0x3B     		// MPU6050 的加速度计 X 轴高位数据寄存器地址，用于读取 X 轴加速度高字节数据。
#define TEMP_OUT_H_REG 				0x41      	// MPU6050 的温度传感器高位数据寄存器地址，用于读取温度的高字节数据。
#define GYRO_CONFIG_REG 			0x1B      	// MPU6050 的 GYRO_CONFIG 寄存器地址 (陀螺仪配置寄存器)，用于设置陀螺仪的量程范围（例如 ±250°/s、±500°/s 等）。
#define GYRO_XOUT_H_REG 			0x43      	// MPU6050 的陀螺仪 X 轴高位数据寄存器地址，用于读取 X 轴陀螺仪角速度的高字节数据。

// Setup MPU6050
#define MPU6050_ADDR 0xD0  		// MPU6050 的补位后的 8 位地址，MPU6050的 7 位地址为 110 100x (0x68 或 0x69)，最低位 x 取决于 AD0 引脚状态。I2C 通信时，左移一位并在最低位加 0（写）或 1（读），形成 8 位地址。以写为例，MPU6050 地址为 1101 0000 (0xD0) 表示写操作
const uint16_t i2c_timeout = 100;  // I2C 通信超时时间，单位为毫秒。
const double Accel_Z_corrector = 14418.0;

uint32_t mpu_timer;

MPU6050_t MPU6050;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t check;
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Data;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);		// 通过 I2Cx 访问 MPU6050 的 WHO_AM_I 寄存器，读取 1 字节数据到 check，超时时间为 i2c_timeout。
	MPU6050.Devivation_Check_Flag = 0;

	if (check == 0x68) // 检验 check 的值 是否为 0x68
    {
		// 设置 Power Management 1 寄存器 (电源管理1寄存器)，将其第7位 置1 (1000 0000) 进行设备复位，防止数据残留
        Data = 0x80;
		//Data = 0x01;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
		HAL_Delay(100);
		// 设置 Power Management 1 寄存器 (电源管理1寄存器)，将其所有位置0 (0000 0000) ，进行唤醒
        Data = 0x01;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        
		//设置 ACCEL_CONFIG 寄存器 (加速度计配置 Accelerometer Configuration )
		// 关闭X Y Z轴的自检功能，配置量程为(-2g,+2g) XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +- 2g
        //Data = 0x00;            //0x00<<3->2   0x01<<3->4   0x02<<3->8  0x03<<3->16
		Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        //设置 GYRO_CONFIG 寄存器 (陀螺仪配置 Gyroscopic Configuration )
        // 关闭X Y Z轴的自检功能，并配置量程为 (-250 °/s,+250 °/s) XG_ST=0, YG_ST=0, ZG_ST=0 , FS_SEL=0 -> ±250 °/s
        //Data = 0x00;            //0x00<<3->250   0x01<<3->500  0x02<<3->1000   0x03<<3->2000
        Data = 0x18;	
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        
		// 设置 Sample Rate Divider 寄存器 (采样速率分频器)，采样速率 = 1kHz / ( 0X04 + 1) = 200Hz    1khz/(0x09+1)=100Hz
        // 0x15   45hz
        Data = 0x09;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

		return 0;		//返回0，表示 MPU6050 正常
    }
    return 1;		//返回1，表示 MPU6050 异常
}


void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];	//存储读取到的 6 字节加速度计原始数据


    // 通过 I2Cx 访问 MPU6050 的 Accelerometer Measurements 寄存器组中的 ACCEL_XOUT_H_REG 寄存器(加速度计X轴加速度高字节 寄存器)，读取 6 字节数据( X轴加速度的高字节 ---> Z轴加速度的低字节 )到 Rec_Data ，超时时间为 i2c_timeout。
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    //将读取到的存储在 Rec_Data[6] 中的原始数据 分离转换为 X、Y、Z轴加速度的 16 位有符号整数
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** 将原始值转换为以 'g' 为单位的加速度值。
     根据 FS_SEL 配置来进行换算，这里 FS_SEL = 0，对应量程为 ±2g，
    因此需要将原始值除以 16384.0，来得到加速度值。 
            根据 Accelerometer Measurements 加速度计测量寄存器组中的 AFS_SEL 配置中的描述，可知量程对应的除数
    更多细节请参考 ACCEL_CONFIG 与 Accelerometer Measurements寄存器的配置。  ****/

    // 将原始值转换为 g 为单位的加速度

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;  // X 轴加速度
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;  // Y 轴加速度
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;		//Z 轴加速度 (由于Z轴加速度计的值会受重力、传感器零飘、传感器精度与校准的影响，导致在水平放置时，Accel_Z_RAW 不为 2g 量程对应的理论值 16384，而是实际值。修正参数就是根据实际情况测出的，用来矫正零飘和其他影响的校正因子
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];		//存储读取到的 6 字节陀螺仪计原始数据

    // 通过 I2Cx 访问 MPU6050 的 Gyroscope Measurements 寄存器组中的 GYRO_XOUT_H_REG 寄存器(陀螺仪 X 轴角速度高字节 寄存器)，读取 6 字节数据( X 轴角速度的高字节 ---> Z 轴角速度的低字节 )到 Rec_Data ，超时时间为 i2c_timeout。
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    //将读取到的存储在 Rec_Data[6] 中的原始数据 分离转换为 X、Y、Z轴角速度的 16 位有符号整数
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** 将原始值转换为度每秒（dps，degrees per second）。
     根据 FS_SEL 的配置选择适当的比例系数。这里假设 FS_SEL = 0，对应量程为 ±250 °/s，
    因此需要将原始值除以 131.0 来得到角速度（dps）。
            根据 Gyroscope Measurements 角速度计测量寄存器组中的 FS_SEL 配置中的描述，可知量程对应的除数
    更多细节请参考 GYRO_CONFIG 与 Gyroscope Measurements寄存器的配置。  ****/

    // 将原始值转换为 dps	为单位的加速度

    DataStruct->Gx = DataStruct->Gyro_X_RAW/32768.0f*2000.0f;  // X 轴角速度
    DataStruct->Gy = DataStruct->Gyro_Y_RAW/32768.0f*2000.0f;  // Y 轴角速度
    DataStruct->Gz = DataStruct->Gyro_Z_RAW/32768.0f*2000.0f;  // Z 轴角速度
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];  // 用于存储从温度寄存器读取的 2 字节数据
    int16_t temp;  // 用于存储拼接后的 16 位原始温度数据

    // 从 Temperature Measurement 温度测量寄存器组 中的 TEMP_OUT_H 寄存器开始，读取 2 字节数据（高字节和低字节）

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

	// 将读取到的高字节和低字节拼接为 16 位的有符号整数
    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    // 将原始温度值转换为摄氏温度 (详情见 Temperature Measurement
    // 转换公式：温度值(°C) = (原始值 / 340.0) + 36.53	
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}


/**
 * @brief 读取 MPU6050 所有传感器数据，并使用卡尔曼滤波器计算角度
 * 
 * 该函数通过 I2C 接口读取 MPU6050 传感器的加速度计、陀螺仪和温度的数据，
 * 并使用卡尔曼滤波器对预测角度与测量角度(加速度计和陀螺仪数据)进行融合，以计算俯仰角（pitch）和滚转角（roll）。
 *
 * @param I2Cx I2C 句柄，用于通过 I2C 接口与 MPU6050 通信
 * @param DataStruct 指向 MPU6050_t 结构体的指针，存储读取到的原始传感器数据以及处理后的数据（如加速度、角速度、角度等）
 */

HAL_StatusTypeDef test;

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    //uint8_t Rec_Data[14];   // 用于存储从 MPU6050 读取的 14 字节数据
    int16_t temp;           // 临时变量，用于存储原始温度数据
    uint8_t Rec_Data[14];   // 用于存储从 MPU6050 读取的 14 字节数据
    //1.通过 I2C 接口从 MPU6050 读取 14 字节加速度、温度、角速度数据
    test = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, HAL_MAX_DELAY);

	//2.解析数据
	
    // 解析加速度计数据
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    // 解析温度数据
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    // 解析陀螺仪数据
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
    
    //减去零飘
    // if(DataStruct->Devivation_Check_Flag)
    // {
    //     DataStruct->Gyro_X = DataStruct->Gyro_X_RAW - DataStruct->Gyro_X_RAW_Dev;
    //         DataStruct->Gyro_Y = DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Y_RAW_Dev;
    //         DataStruct->Gyro_Z = DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Z_RAW_Dev;
    // }
	//3.转换解析后的数据：  原始数据 --> 实际意义数据
	
    // 将原始加速度计数据 ---> 以 "g" 为单位的加速度值
    DataStruct->Ax = DataStruct->Accel_X_RAW/32768.0f*2.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW/32768.0f*2.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW/32768.0f*2.0f;
	//DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    // 将原始温度数据 ---> 摄氏温度
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    // 将原始陀螺仪数据 ---> 角速度(以 "度每秒 (dps)" 为单位
		/*
    DataStruct->Gx = DataStruct->Gyro_X_RAW/32768.0f*2000.0f;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW/32768.0f*2000.0f;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW/32768.0f*2000.0f;
		*/
    DataStruct->Gx = DataStruct->Gyro_X/32768.0f*2000.0f;
    DataStruct->Gy = DataStruct->Gyro_Y/32768.0f*2000.0f;
    DataStruct->Gz = DataStruct->Gyro_Z/32768.0f*2000.0f;
}

//卡尔曼滤波更新角度
void MPU6050_GetAngle(MPU6050_t *DataStruct)
{
    // 计算时间增量 dt，单位为秒
    double dt = (double)(HAL_GetTick() - mpu_timer) / 1000;  // 获取时间差（毫秒），转换为秒
    mpu_timer = HAL_GetTick();  // 更新计时器
	  // 计算滚转角 roll
    double roll;  // 用于存储计算得到的滚转角（X 轴）
    double roll_sqrt = sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;  // 先计算出弧度制 roll ，再弧度转换为角度值
    }
    else
    {
        roll = 0.0;
    }
	
	  // 计算俯仰角 pitch
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;

    // 如果俯仰角度变化过快(超过90度)，防止角度跳变
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch; 
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        // 卡尔曼滤波器更新俯仰角度 Y
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }

    // 如果俯仰角绝对值超过 90 度，则反转 X 轴的陀螺仪角速度，防止符号错误
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;

    // 卡尔曼滤波器更新滚转角度 X
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
		
		
    //计算偏航角
    //90  2080   
    if(abs_int16(DataStruct->Gz)>0.4)
    {
        MPU6050.Yaw_360 -= DataStruct->Gz/100.0f;
        MPU6050.Yaw_8192 = MPU6050.Yaw_360*22.7555555556f;
    }
}



/**
 * @brief 使用卡尔曼滤波，融合角度预测值与角度测量值，得到最优角度值
 * 
 * 卡尔曼滤波通过结合系统的预测值和测量值，来修正角度和偏置的估计，减少噪声对结果的影响。
 * 
 * @param Kalman 卡尔曼滤波器结构体指针，包含预测角度、预测偏置、角度协方差、偏置协方差、噪声协方差及协方差矩阵
 * @param newAngle 角度测量值（读取加速度计的三轴加速度分量，再计算反正切得到角度测量值）
 * @param newRate 角速度“实际“值（他只是可以看作角速度实际值，实际上是角速度测量值。只不过因陀螺仪精度问题，而有过程噪声。可以理解为匀变速直线运动的状态方程中必须有 速度v 的参与，这个 v 实际上也是测量值）
 * @param dt 时间间隔，两个传感器数据采样之间的时间差（秒）
 * @return double 滤波后的最优角度值
 */

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
/*---------------------预测阶段--------------------------*/

    // 1. 预测角度
    // 角速度 = 陀螺仪角速度 - 陀螺仪偏置值 (得到无偏角速度)
    double rate = newRate - Kalman->bias;
    // 预测角度 = 前一时刻角速 + 时间间隔*角速度
    Kalman->angle += dt * rate;

    // 2. 预测协方差矩阵
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle); // 预测角度协方差
    Kalman->P[0][1] -= dt * Kalman->P[1][1];  // 预测角度和偏置的协方差
    Kalman->P[1][0] -= dt * Kalman->P[1][1];  // 预测偏置和角度的协方差
    Kalman->P[1][1] += Kalman->Q_bias * dt;   // 预测偏置协方差


/*---------------------更新阶段--------------------------*/

    // 3. 更新卡尔曼增益
    // 总误差协方差 = 预测协方差 + 测量噪声协方差
    double S = Kalman->P[0][0] + Kalman->R_measure;
    // 卡尔曼增益 K
    double K[2]; 
    K[0] = Kalman->P[0][0] / S;  // 角度的卡尔曼增益
    K[1] = Kalman->P[1][0] / S;  // 偏置的卡尔曼增益

    // 4. 更新角度和偏置
    // 测量残差 = 测量值 - 预测值
    double y = newAngle - Kalman->angle;
    // 根据卡尔曼增益，更新角度和偏置的估计值，修正预测阶段的误差
    Kalman->angle += K[0] * y;  // 更新角度估计。
    Kalman->bias += K[1] * y;   // 更新偏置估计

    // 5. 更新协方差矩阵 P
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp;  // 更新角度协方差
    Kalman->P[0][1] -= K[0] * P01_temp;  // 更新角度和偏置的协方差
    Kalman->P[1][0] -= K[1] * P00_temp;  // 更新偏置和角度的协方差
    Kalman->P[1][1] -= K[1] * P01_temp;  // 更新偏置协方差

    // 6. 返回滤波后的最优角度值
    return Kalman->angle;
};

void MPU6050_CheckDeviation(MPU6050_t *DataStruct)
{
  int tick = 0;
	int16_t Gyro_X_sum = 0;
	int16_t Gyro_Y_sum = 0;
	int16_t Gyro_Z_sum = 0;
	
	while(tick<1000)
	{
        MPU6050_Read_All(&hi2c2, &MPU6050);
        Gyro_X_sum += DataStruct->Gyro_X_RAW;
        Gyro_Y_sum += DataStruct->Gyro_Y_RAW;
        Gyro_Z_sum += DataStruct->Gyro_Z_RAW;
        tick++;
        HAL_Delay(10);
	}
	DataStruct->Gyro_X_RAW_Dev = Gyro_X_sum/1000.0f;
	DataStruct->Gyro_Y_RAW_Dev = Gyro_Y_sum/1000.0f;
	DataStruct->Gyro_Z_RAW_Dev = Gyro_Z_sum/1000.0f;
	DataStruct->Devivation_Check_Flag = 1;
}

void MPU6050_Process()
{
    MPU6050_Read_All(&hi2c2, &MPU6050);
    //MPU6050_GetAngle(&MPU6050);
}