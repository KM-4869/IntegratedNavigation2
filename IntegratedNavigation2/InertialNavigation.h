#pragma once
#include"Coordinate.h"
#include"Matrix2.h"
#include"MyVector2.h"

#include<iostream>
struct ANGLE
{
	double yaw;
	double pitch;
	double roll;

	ANGLE(double y=0.0,double p=0.0,double r=0.0)
	{
		yaw = y;
		pitch = p;
		roll = r;
	}
};

struct IMUError
{
	Vector gyrbias;//标准单位rad/s
	Vector accbias;//标准单位m/s^2
	Vector gyrscale;//标准单位 1
	Vector accscale;//标准单位 1

	IMUError():gyrbias(3),accbias(3),gyrscale(3),accscale(3)
	{

	}
};

struct IMUNoise
{
	Vector ARW;//角度随机游走
	Vector VRW;//速度随机游走
	Vector gb_std;//陀螺仪零偏标准差
	Vector ab_std;//加速度计零偏标准差
	Vector gs_std;//陀螺仪比例因子标准差
	Vector as_std;//加速度计比例因子标准差
	double corr_t;//相关时间


	IMUNoise(Vector arw , Vector vrw , Vector gb, Vector ab, Vector gs  , Vector as , double t ):ARW(3),VRW(3),gb_std(3),ab_std(3),gs_std(3),as_std(3)
	{
		ARW = arw * D2R * (1.0 / 60.0);//输入参数单位为[deg/sqrt(h)]
		VRW = vrw * (1.0 / 60.0);//输入参数单位为[m/s/sqrt(h)]
		gb_std = gb * D2R * (1.0 / 3600.0);//输入参数单位为[deg/h]
		ab_std = ab * 1.0E-5;//输入参数单位为[mGal]
		gs_std = gs * 1.0E-6;//输入参数单位为[ppm]
		as_std = as * 1.0E-6;//输入参数单位为[ppm]
		corr_t = t;
	}

	
};


struct IMUDATA
{
	double time;

	Vector AngInc;
	Vector VelInc;

	IMUDATA() :AngInc(3), VelInc(3)
	{

	}
};


class IMUBOX
{
public:
	IMUBOX(BLH B, Vector V, ANGLE A);
	~IMUBOX();

	void PureINSmech(IMUDATA IMUData_pre, IMUDATA IMUData_cur);//输入k-1时刻角增量向量，k时刻角增量向量，和采样间隔，对姿态进行更新
	void Show();
	void output(FILE*fp);

	BLH getBlh();
	Vector getVel();
	ANGLE getAngle();
	Matrix getCn_b();

protected:
	BLH Blh;//位置(角度)
	Vector Vel;//北向速度，东向速度，垂向下速度
	ANGLE Angle;//姿态角(角度)
	Matrix Cn_b;//与姿态角对应的方向余弦矩阵（由b到N,前右下到北东地）

	//更新时需要用到的前一时刻的位置和速度
	BLH Blh_pre;
	Vector Vel_pre;
	
	ANGLE Angle_pre;
	Matrix Cn_b_pre;
};

void Cn_b2Eular(Matrix Cn_b, ANGLE& Angle);
void Eular2Cn_b(ANGLE& Angle, Matrix Cn_b);