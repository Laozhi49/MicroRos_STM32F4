/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 * 本头文件定义了用于操作 MPU6050 传感器的结构和函数，包括加速度计、陀螺仪、温度传感器的读取函数，
 * 以及用于角度计算的卡尔曼滤波算法。
 */

#ifndef __MPU6050_H_
#define __MPU6050_H_

#include <stdint.h>
#include "i2c.h"


/* 
 * MPU6050 数据结构体
 * 该结构体保存从 MPU6050 传感器读取的原始加速度和原始陀螺仪数据
 * 以及经过处理后的加速度、角速度和温度数据
 */
typedef struct
{
    int16_t Accel_X_RAW;   // X 轴加速度原始数据
    int16_t Accel_Y_RAW;   // Y 轴加速度原始数据
    int16_t Accel_Z_RAW;   // Z 轴加速度原始数据
    double Ax;             // X 轴加速度值（g）
    double Ay;             // Y 轴加速度值（g）
    double Az;             // Z 轴加速度值（g）

    int16_t Gyro_X_RAW;    // X 轴陀螺仪原始数据
    int16_t Gyro_Y_RAW;    // Y 轴陀螺仪原始数据
    int16_t Gyro_Z_RAW;    // Z 轴陀螺仪原始数据
    int16_t Gyro_X;        // X 轴陀螺仪数据
    int16_t Gyro_Y;        // Y 轴陀螺仪数据
    int16_t Gyro_Z;        // Z 轴陀螺仪数据
    int16_t Gyro_X_RAW_Dev;// X 轴陀螺仪原始数据零飘
    int16_t Gyro_Y_RAW_Dev;// Y 轴陀螺仪原始数据零飘
    int16_t Gyro_Z_RAW_Dev;// Z 轴陀螺仪原始数据零飘

    uint8_t Devivation_Check_Flag;

    double Gx;             // X 轴角速度值（°/s）
    double Gy;             // Y 轴角速度值（°/s）
    double Gz;             // Z 轴角速度值（°/s）

    float Temperature;     // 传感器的温度（°C）

    double KalmanAngleX;   // X 轴的卡尔曼滤波计算角度
    double KalmanAngleY;   // Y 轴的卡尔曼滤波计算角度
    double Yaw_360;
    double Yaw_8192;
    double bias;
    double dead_zone;

    uint8_t ID;
} MPU6050_t;

extern MPU6050_t MPU6050;
/* 
 * 卡尔曼滤波器结构体
 * 用于根据 MPU6050 数据平滑地估算出角度，滤除噪声并提供更稳定的输出
 */
typedef struct
{
    double Q_angle;    // 角度过程噪声协方差
    double Q_bias;     // 偏差过程噪声协方差
    double R_measure;  // 测量噪声协方差
    double angle;      // 当前估计角度
    double bias;       // 当前估计偏差
    double P[2][2];    // 误差协方差矩阵
} Kalman_t;

// 初始化 MPU6050 传感器，配置 MPU6050 参数，返回 0 表示成功，1 表示失败
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx); 

// 读取 MPU6050 加速度计数据，更新到 mpu6050 结构体中
void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); 

// 读取 MPU6050 陀螺仪数据，更新到 mpu6050 结构体中
void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); 

// 读取 MPU6050 温度数据，更新到 mpu6050 结构体中
void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); 

// 读取 MPU6050 的加速度计、陀螺仪和温度数据，并更新到 mpu6050 结构体中
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct); 

// 使用卡尔曼滤波器计算角度，根据新测得的角度和角速度更新滤波器，返回平滑的角度值
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

void MPU6050_CheckDeviation(MPU6050_t *DataStruct);
void MPU6050_Process();


#endif /* INC_GY521_H_ */