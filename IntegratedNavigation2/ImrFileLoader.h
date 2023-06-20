#pragma once
#include"FileLoader.h"
#include"InertialNavigation.h"
#include<iomanip>
#include<iostream>
//���ܽ��뺯��
template<typename TYPE>//һ��Ҫ������ǰ��д��  
inline void BinaryToAnytype(TYPE* Variable, char* p)
{
	memcpy(Variable, p, sizeof(TYPE));
}

class ImrFileLoader :public FileLoader
{
public:

	ImrFileLoader(const string& filename);

	vector<double> load();
	bool load(vector<double>& onelinedata);

	//���йߵ�����׼(����1 ����ʼʱ�̿�ʼ�Ķ�׼ʱ��(s), ����2 ����õ���̬��, ����3 ���ڵص㾭γ��),����ֵ����ʼ��׼����ʱ��
	double InitialAlignment(int length, ANGLE* Angle, BLH* Blh);


private:

	void DecodeHeader();
	void HeaderShow();

	double VersionNumber;
	int isDeltaTheta;//�����������Ƿ�Ϊ������ʽ(0=����,1=��)
	int isDeltaVelocity;//���ٶ������Ƿ�Ϊ������ʽ(0=���ǣ�1=��)
	double DataRateHz;//������

	double GyroScaleFactor;//����ֵ�˴˵�����ʵ���ٶ�/������
	double AccelScaleFactor;//����ֵ�˴˵�����ʵ���ٶ�/�ٶ�����

	int isUTCorGPS;//ʱ���ǩ�Ƿ�ΪGPS����UTCʱ(0=δ֪��Ĭ��GPS,1=UTC,2=GPS)

	char ImuName[32];//imu������

};