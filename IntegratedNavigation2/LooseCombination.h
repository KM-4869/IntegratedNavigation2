#pragma once

#include"InertialNavigation.h"
#include"Constant.h"
#include<iomanip>//���������ʽ
#include<fstream>

using namespace std;
struct NavState
{
	BLH Blh;
	Vector Vel;
	ANGLE Angle;

	NavState(BLH B,Vector V,ANGLE A):Vel(3)
	{
		Blh = B;
		Vel = V;
		Angle = A;
	}

};

struct GNSSDATA
{
	double time;

	Vector Blh;
	Vector Vel;//������

	Vector Blh_std;
	Vector Vel_std;

	bool DataAvailable;

	GNSSDATA():Blh(3),Vel(3),Blh_std(3),Vel_std(3)
	{
		time = 0.0;
		DataAvailable = false;
	}
};



class Body :public IMUBOX
{

public:

	Body(NavState inistate, NavState inistate_std, IMUNoise imun, Vector al, Vector ol);

	
	void AddIMUData(double* OneLineIMU);//���imu����������еĸ�ʽ����һ��ΪGPS���룬2-4��Ϊ���������(rad),5-7��Ϊ���ٶȼ����(m/s),��7��
	void AddGNSSData(double* OneLineGNSS);//���GNSS����������еĸ�ʽ����һ��GPS���룬2-4�о�γ��(deg,m),5-7�о�γ�߷��8-10�б������ٶ�(m/s),11-13�б������ٶȷ����13��
	void AddODOData(double* OneLineODO);

	void SetTimestamp(double time);
	bool IsGNSSDataAvailable();

	void PureINSwork();
	void work();
	

	void BodyShow();
	void BodyOutputToFile(fstream& fout);
	void CovOutputToFile(fstream& fout);

private:

	IMUDATA IMUData_pre, IMUData_cur;//ǰһʱ�̺����ڵ�IMU����
	GNSSDATA GNSSData;//GNSS����

	double timestamp;//ÿ��Ҫ�����ڲ��ʱ��Ǻţ�ͨ��Ϊ��ʼʱ�̺���GNSS�������룩

	IMUError ImuError;
	IMUNoise ImuNoise;

	Vector antlever;//�˱ۣ���b->GNSS,��bϵ�²�����
	Vector odolever;//��̼Ƹ˱�

	Matrix Cov;//״̬Э���21*21��
	Matrix q;//ϵͳ��������18*18��
	Matrix x;//�����Ĳ�����21*1��

	double odoVel;


	void ImuInterpolate(IMUDATA& IMUData_pre, IMUDATA& IMUData_cur, double midtime, IMUDATA& IMUData_mid);//IMU�ڲ�
	
	void ImuPropagation(IMUDATA& IMUData_pre, IMUDATA& IMUData_cur);//IMU״̬����
	void GNSSUpdating(IMUDATA& IMUData_mid);//GNSS��������

	void Feedback();//״̬����
	void Compensate();//IMU��������

	bool IsVelocityZero();
	void ZUPT();//���ٲ�������

	//bool IsNHC();
	void NHC();//�������ٶ�Ϊ0�˶�Լ����������

	void ODO();//����̼��ṩ��ǰ���ٶȵĲ�������

	//void NHC_ODO();



};

















void CreateMatrix_F_G_H(IMUBOX& body, Vector AngInc_k, Vector VelInc_k, Vector lb, Matrix& F, Matrix& G, Matrix& H, double T);
Matrix CreateMatrixq(IMUError IMUError);
