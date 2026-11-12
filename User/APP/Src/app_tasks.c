/************************************************
 * Developer : Gonk
 * GitHub    : https://github.com/mengfeiguo0316-sys
 * Email     : mengfeiguo0316@gmail.com or 2250513147@qq.com
 * Phone     : 199 5063 9665
 *
 * Version   : V1.0.0
 * Updated   : 2026-11-12
 *
 * Description:应用任务管理模块，负责系统主循环与业务任务调度
 *   
 * Copyright (c) 2026 Gonk
 * All rights reserved.
 ************************************************/





/*
void MPU6050_Proc(void)
{
	//const float alpha = 0.95238f;
	const float alpha = 0.98f;
    const float dt    = 0.005f;

    MPU6050_GetPhysical();  // 更新加速度与角速度物理量

    // 1. 加速度计解算（调用 fast_math 的 qatan2 快速反正切，并乘常数转为角度）
    mpu6050_data.pitch_a = qatan2(-mpu6050_data.ax, mpu6050_data.az) * 57.29578f;
    mpu6050_data.row_a   = qatan2( mpu6050_data.ay, mpu6050_data.az) * 57.29578f;
    mpu6050_data.yaw_a   = 0.0f;
    
	// 2. 互补滤波闭环融合
    // Pitch 绕 Y 轴旋转 -> 累加 gy
    mpu6050_data.pitch = alpha * (mpu6050_data.pitch + mpu6050_data.gy * dt) + (1.0f - alpha) * mpu6050_data.pitch_a;

    // Roll 绕 X 轴旋转 -> 累加 gx
    mpu6050_data.row   = alpha * (mpu6050_data.row   + mpu6050_data.gx * dt) + (1.0f - alpha) * mpu6050_data.row_a;

    // Yaw 绕垂直地面的 Z 轴旋转 -> 累加 gz
    mpu6050_data.yaw  += mpu6050_data.gz * dt;
}

/**
 * @brief  获取 MPU6050 转换后的欧拉角给到VOFA+软件
 * @param  无

MPU6050_DebugData_t MPU6050_GetDebugData(void)
{
	MPU6050_DebugData_t data;
	
	data.pitch=mpu6050_data.pitch;
	data.row=mpu6050_data.row;
	data.yaw=mpu6050_data.yaw;
	
	data.ax=mpu6050_data.ax;
	data.ay=mpu6050_data.ay;
	data.az=mpu6050_data.az;
	
	data.gx=mpu6050_data.gx;
	data.gy=mpu6050_data.gy;
	data.gz=mpu6050_data.gz;

	return data;
}

*/
