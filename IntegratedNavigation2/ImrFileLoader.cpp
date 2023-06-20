
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
	//��ȡ��һ��Ԫʱ��
	double t = OneLineData[0];

	//����ʱ���Լ�imu���ݣ�ʱ��8�ֽڣ�imu�������ֽ�
	vector<int> imudata(6);
	fin.read((char*)OneLineData.data(), 8);
	fin.read((char*)imudata.data(), 4 * 6);
	//�������ӻ����ʵ�����(��������ʵֵ),��������ϵ�任����ǰ����ǰ���£��Ա��ڽ����еĹ�ʽ�Ǻϡ����巽��Ϊxy�ύ����z����.���⣬ԭʼ���뵥λΪ����ת����
	OneLineData[1] = GyroScaleFactor * imudata[1] * D2R;
	OneLineData[2] = GyroScaleFactor * imudata[0] * D2R;
	OneLineData[3] = GyroScaleFactor * imudata[2] * (-1.0) * D2R;
	OneLineData[4] = AccelScaleFactor * imudata[4];
	OneLineData[5] = AccelScaleFactor * imudata[3];
	OneLineData[6] = AccelScaleFactor * imudata[5] * (-1.0);

	//����Ƿ��ظ���Ԫ
	if (abs(OneLineData[0] - t) < 0.1 / DataRateHz)
		load();

	//����Ƿ�ȱʧ��Ԫ
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
	//��ȡ��һ��Ԫʱ��
	double t = OneLineData[0];

	//����ʱ���Լ�imu���ݣ�ʱ��8�ֽڣ�imu�������ֽ�
	vector<int> imudata(6);
	fin.read((char*)OneLineData.data(), 8);
	fin.read((char*)imudata.data(), 4 * 6);

	//�������ӻ����ʵ�����(��������ʵֵ),��������ϵ�任����ǰ����ǰ���£��Ա��ڽ����еĹ�ʽ�Ǻϡ����巽��Ϊxy�ύ����z����.���⣬ԭʼ���뵥λΪ����ת����
	OneLineData[1] = GyroScaleFactor * imudata[1] * D2R;
	OneLineData[2] = GyroScaleFactor * imudata[0] * D2R;
	OneLineData[3] = GyroScaleFactor * imudata[2] * (-1.0) * D2R;
	OneLineData[4] = AccelScaleFactor * imudata[4];
	OneLineData[5] = AccelScaleFactor * imudata[3];
	OneLineData[6] = AccelScaleFactor * imudata[5] * (-1.0);
	//����Ƿ��ظ���Ԫ
	if (abs(OneLineData[0] - t) < 0.1 / DataRateHz)
		load();

	//����Ƿ�ȱʧ��Ԫ
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

	//�ߵ���ʼ��׼����Ԫ���
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

	//(���ݲ�ͬ������ʽ)��ƽ��
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

	//����bϵ�µ���������
	Vector g_b(3, '|', -fx, -fy, -fz);
	Vector w_b(3, '|', wx, wy, wz);

	Vector gw_b = g_b->*w_b;
	Vector gwg_b = gw_b->*g_b;

	//����nϵ�µ���������
	double wx_n = GPS_we * cos(Blh->B * D2R);
	double wy_n = 0.0;
	double wz_n = GPS_we * sin(Blh->B * D2R) * (-1.0);

	double g = gravity(Blh->B, Blh->H);

	Vector g_n(3, '|', 0.0, 0.0, g);
	Vector w_n(3, '|', wx_n, wy_n, wz_n);

	Vector gw_n = g_n->*w_n;
	Vector gwg_n = gw_n->*g_n;

	//��������λ��
	g_b.ToUnit();
	gw_b.ToUnit();
	gwg_b.ToUnit();
	g_n.ToUnit();
	gw_n.ToUnit();
	gwg_n.ToUnit();

	//�����ת����
	Matrix Cn_b = (g_n, gw_n, gwg_n) * (g_b.T() & gw_b.T() & gwg_b.T());

	//�����̬��
	Cn_b2Eular(Cn_b, *Angle);

	cout << "The Initial Attitude are:" << '\n';
	cout << "yaw:  " << Angle->yaw << "��" << '\n';
	cout << "pitch:  " << Angle->pitch << "��" << '\n';
	cout << "roll:  " << Angle->roll << "��" << '\n';
	cout << "Length: " << length << "s" << '\n';
	cout << "End Time: " << OneLineData[0] << "s" << '\n';

	return OneLineData[0];
}
