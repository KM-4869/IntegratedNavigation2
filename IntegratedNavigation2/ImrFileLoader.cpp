
#include"ImrFileLoader.h"

ImrFileLoader::ImrFileLoader(const string& filename):FileLoader(filename, 7, BINARY)
{
	DecodeHeader();
	HeaderShow();
	OneLineData.resize(DataCount);
}


void ImrFileLoader::DecodeHeader()
{
	char* buff = new char[512];

	fin.read(buff, 512);

	BinaryToAnytype(&VersionNumber, buff + 9);
	BinaryToAnytype(&isDeltaTheta, buff + 17);
	BinaryToAnytype(&isDeltaVelocity, buff + 21);
	BinaryToAnytype(&DataRateHz, buff + 25);
	BinaryToAnytype(&GyroScaleFactor, buff + 33);
	BinaryToAnytype(&AccelScaleFactor, buff + 41);
	BinaryToAnytype(&isUTCorGPS, buff + 49);
	strncpy_s(ImuName, buff + 65, 32);

	delete[] buff;

}

void ImrFileLoader::HeaderShow()
{
	cout << "**************************************************IMR FILE HEADER****************************************************"<<'\n';
	cout << "VersionNumber  " << VersionNumber << '\n';
	cout << "isDeltaTheta(0=NO,1=YES)  " << isDeltaTheta << '\n';
	cout << "isDeltaVelocity(0=NO,1=YES)  " << isDeltaVelocity << '\n';
	cout << "DataRate(Hz)  " << DataRateHz << '\n';
	cout << "GyroScaleFactor  " << GyroScaleFactor << '\n';
	cout << "AccelScaleFactor  " << AccelScaleFactor << '\n';
	cout << "isUTCorGPS(0=unknown,default GPS, 1=UTC, 2=GPS)  " << isUTCorGPS << '\n';
	cout << "ImuName  " << ImuName << '\n';
	cout << "****************************************************END OF HEADER**************************************************" << '\n';
}


vector<double> ImrFileLoader::load()
{
	//获取上一历元时间
	double t = OneLineData[0];

	//读入时间以及imu数据，时间8字节，imu数据四字节
	vector<int> imudata(6);
	fin.read((char*)OneLineData.data(), 8);
	fin.read((char*)imudata.data(), 4 * 6);
	//乘上因子获得真实的输出(增量或真实值),并进行轴系变换（右前上至前右下）以便于讲义中的公式吻合。具体方法为xy轴交换，z轴变号.另外，原始读入单位为度需转弧度
	OneLineData[1] = GyroScaleFactor * imudata[1] * D2R;
	OneLineData[2] = GyroScaleFactor * imudata[0] * D2R;
	OneLineData[3] = GyroScaleFactor * imudata[2] * (-1.0) * D2R;
	OneLineData[4] = AccelScaleFactor * imudata[4];
	OneLineData[5] = AccelScaleFactor * imudata[3];
	OneLineData[6] = AccelScaleFactor * imudata[5] * (-1.0);

	//检查是否重复历元
	if (abs(OneLineData[0] - t) < 0.1 / DataRateHz)
		load();

	//检查是否缺失历元
	if (abs(OneLineData[0] - t) > (1.2 / DataRateHz))
	{
		for (int i = 1; i < OneLineData.size(); i++)
			OneLineData[i] *= 2.0;
	}

	return OneLineData;
}

bool ImrFileLoader::load(vector<double>& onelinedata)
{
	if (fin.eof())
		return false;
	//获取上一历元时间
	double t = OneLineData[0];

	//读入时间以及imu数据，时间8字节，imu数据四字节
	vector<int> imudata(6);
	fin.read((char*)OneLineData.data(), 8);
	fin.read((char*)imudata.data(), 4 * 6);

	//乘上因子获得真实的输出(增量或真实值),并进行轴系变换（右前上至前右下）以便于讲义中的公式吻合。具体方法为xy轴交换，z轴变号.另外，原始读入单位为度需转弧度
	OneLineData[1] = GyroScaleFactor * imudata[1] * D2R;
	OneLineData[2] = GyroScaleFactor * imudata[0] * D2R;
	OneLineData[3] = GyroScaleFactor * imudata[2] * (-1.0) * D2R;
	OneLineData[4] = AccelScaleFactor * imudata[4];
	OneLineData[5] = AccelScaleFactor * imudata[3];
	OneLineData[6] = AccelScaleFactor * imudata[5] * (-1.0);
	//检查是否重复历元
	if (abs(OneLineData[0] - t) < 0.1 / DataRateHz)
		load();

	//检查是否缺失历元
	if (abs(OneLineData[0] - t) > (1.2 / DataRateHz))
	{
		for (int i = 1; i < OneLineData.size(); i++)
			OneLineData[i] *= 2.0;
	}

	std::copy(OneLineData.begin(), OneLineData.end(), onelinedata.begin());
	return true;
	
}

double ImrFileLoader::InitialAlignment(int length, ANGLE* Angle, BLH* Blh)
{
	double gyro_x_sum = 0.0, gyro_y_sum = 0.0, gyro_z_sum = 0.0;
	double accl_x_sum = 0.0, accl_y_sum = 0.0, accl_z_sum = 0.0;

	//惯导初始对准各历元求和
	for (int i = 1; i < length * DataRateHz; i++)
	{
		load();
		gyro_x_sum += OneLineData[1];
		gyro_y_sum += OneLineData[2];
		gyro_z_sum += OneLineData[3];
		accl_x_sum += OneLineData[4];
		accl_y_sum += OneLineData[5];
		accl_z_sum += OneLineData[6];
	}

	double wx, wy, wz;
	double fx, fy, fz;

	//(根据不同数据形式)求平均
	if (isDeltaTheta == 1)
	{
		wx = gyro_x_sum / length;
		wy = gyro_y_sum / length;
		wz = gyro_z_sum / length;
	}
	else
	{
		wx = gyro_x_sum / (length * DataRateHz);
		wy = gyro_y_sum / (length * DataRateHz);
		wz = gyro_z_sum / (length * DataRateHz);
	}

	if (isDeltaTheta == 1)
	{
		fx = accl_x_sum / length;
		fy = accl_y_sum / length;
		fz = accl_z_sum / length;
	}
	else
	{
		fx = accl_x_sum / (length * DataRateHz);
		fy = accl_y_sum / (length * DataRateHz);
		fz = accl_z_sum / (length * DataRateHz);
	}

	//构造b系下的三个向量
	Vector g_b(3, '|', -fx, -fy, -fz);
	Vector w_b(3, '|', wx, wy, wz);

	Vector gw_b = g_b->*w_b;
	Vector gwg_b = gw_b->*g_b;

	//构造n系下的三个向量
	double wx_n = GPS_we * cos(Blh->B * D2R);
	double wy_n = 0.0;
	double wz_n = GPS_we * sin(Blh->B * D2R) * (-1.0);

	double g = gravity(Blh->B, Blh->H);

	Vector g_n(3, '|', 0.0, 0.0, g);
	Vector w_n(3, '|', wx_n, wy_n, wz_n);

	Vector gw_n = g_n->*w_n;
	Vector gwg_n = gw_n->*g_n;

	//各向量单位化
	g_b.ToUnit();
	gw_b.ToUnit();
	gwg_b.ToUnit();
	g_n.ToUnit();
	gw_n.ToUnit();
	gwg_n.ToUnit();

	//求解旋转矩阵
	Matrix Cn_b = (g_n, gw_n, gwg_n) * (g_b.T() & gw_b.T() & gwg_b.T());

	//求解姿态角
	Cn_b2Eular(Cn_b, *Angle);

	cout << "The Initial Attitude are:" << '\n';
	cout << "yaw:  " << Angle->yaw << "°" << '\n';
	cout << "pitch:  " << Angle->pitch << "°" << '\n';
	cout << "roll:  " << Angle->roll << "°" << '\n';
	cout << "Length: " << length << "s" << '\n';
	cout << "End Time: " << OneLineData[0] << "s" << '\n';

	return OneLineData[0];
}
