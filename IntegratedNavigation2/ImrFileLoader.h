#pragma once
#include"FileLoader.h"
#include"InertialNavigation.h"
#include<iomanip>
#include<iostream>
//万能解码函数
template<typename TYPE>//一定要紧跟在前面写。  
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

	//进行惯导初对准(参数1 从起始时刻开始的对准时长(s), 参数2 所求得的姿态角, 参数3 所在地点经纬高),返回值：初始对准结束时刻
	double InitialAlignment(int length, ANGLE* Angle, BLH* Blh);


private:

	void DecodeHeader();
	void HeaderShow();

	double VersionNumber;
	int isDeltaTheta;//陀螺仪数据是否为增量形式(0=不是,1=是)
	int isDeltaVelocity;//加速度数据是否为增量形式(0=不是，1=是)
	double DataRateHz;//采样率

	double GyroScaleFactor;//解码值乘此等于真实角速度/角增量
	double AccelScaleFactor;//解码值乘此等于真实加速度/速度增量

	int isUTCorGPS;//时间标签是否为GPS或者UTC时(0=未知，默认GPS,1=UTC,2=GPS)

	char ImuName[32];//imu的名称

};