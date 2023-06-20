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
	Vector gyrbias;//��׼��λrad/s
	Vector accbias;//��׼��λm/s^2
	Vector gyrscale;//��׼��λ 1
	Vector accscale;//��׼��λ 1

	IMUError():gyrbias(3),accbias(3),gyrscale(3),accscale(3)
	{

	}
};

struct IMUNoise
{
	Vector ARW;//�Ƕ��������
	Vector VRW;//�ٶ��������
	Vector gb_std;//��������ƫ��׼��
	Vector ab_std;//���ٶȼ���ƫ��׼��
	Vector gs_std;//�����Ǳ������ӱ�׼��
	Vector as_std;//���ٶȼƱ������ӱ�׼��
	double corr_t;//���ʱ��


	IMUNoise(Vector arw , Vector vrw , Vector gb, Vector ab, Vector gs  , Vector as , double t ):ARW(3),VRW(3),gb_std(3),ab_std(3),gs_std(3),as_std(3)
	{
		ARW = arw * D2R * (1.0 / 60.0);//���������λΪ[deg/sqrt(h)]
		VRW = vrw * (1.0 / 60.0);//���������λΪ[m/s/sqrt(h)]
		gb_std = gb * D2R * (1.0 / 3600.0);//���������λΪ[deg/h]
		ab_std = ab * 1.0E-5;//���������λΪ[mGal]
		gs_std = gs * 1.0E-6;//���������λΪ[ppm]
		as_std = as * 1.0E-6;//���������λΪ[ppm]
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

	void PureINSmech(IMUDATA IMUData_pre, IMUDATA IMUData_cur);//����k-1ʱ�̽�����������kʱ�̽������������Ͳ������������̬���и���
	void Show();
	void output(FILE*fp);

	BLH getBlh();
	Vector getVel();
	ANGLE getAngle();
	Matrix getCn_b();

protected:
	BLH Blh;//λ��(�Ƕ�)
	Vector Vel;//�����ٶȣ������ٶȣ��������ٶ�
	ANGLE Angle;//��̬��(�Ƕ�)
	Matrix Cn_b;//����̬�Ƕ�Ӧ�ķ������Ҿ�����b��N,ǰ���µ������أ�

	//����ʱ��Ҫ�õ���ǰһʱ�̵�λ�ú��ٶ�
	BLH Blh_pre;
	Vector Vel_pre;
	
	ANGLE Angle_pre;
	Matrix Cn_b_pre;
};

void Cn_b2Eular(Matrix Cn_b, ANGLE& Angle);
void Eular2Cn_b(ANGLE& Angle, Matrix Cn_b);