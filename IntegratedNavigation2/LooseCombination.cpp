
#include"LooseCombination.h"


//构造函数。首先调用父类的构造函数。ImuNoise 和杆臂 antlever直接在列表中实现传值。
Body::Body(NavState inistate, NavState inistate_std, IMUNoise imun, Vector al,Vector ol):IMUBOX(inistate.Blh,inistate.Vel,inistate.Angle),ImuNoise(imun),antlever(al),odolever(ol),Cov(21,21),q(18,18),x(21,1)
{
	odoVel = 0.0;
	
	Matrix Zero_33(3, 3);
	
	//构造系统噪声矩阵q
	Matrix VRW2(3, 3);
	Matrix ARW2(3, 3);
	Matrix gb_var(3, 3);
	Matrix ab_var(3, 3);
	Matrix gs_var(3, 3);
	Matrix as_var(3, 3);

	VRW2 = ImuNoise.VRW.diag() * ImuNoise.VRW.diag();
	ARW2 = ImuNoise.ARW.diag() * ImuNoise.ARW.diag();
	gb_var = ImuNoise.gb_std.diag() * ImuNoise.gb_std.diag();
	ab_var = ImuNoise.ab_std.diag() * ImuNoise.ab_std.diag();
	gs_var = ImuNoise.gs_std.diag() * ImuNoise.gs_std.diag();
	as_var = ImuNoise.as_std.diag() * ImuNoise.as_std.diag();

	q = (VRW2, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, ARW2, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, 2 * gb_var * (1.0 / ImuNoise.corr_t), Zero_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, 2 * ab_var * (1.0 / ImuNoise.corr_t), Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, 2 * gs_var * (1.0 / ImuNoise.corr_t), Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, 2 * as_var * (1.0 / ImuNoise.corr_t));

	

	//构造初始状态协方差阵Cov
	Matrix ini_pos_var(3, 3);
	Matrix ini_vel_var(3, 3);
	Matrix ini_angle_var(3, 3);

	//传入初始状态标准差的时候直接传入北东地方向的标准差（单位为m）(虽然结构体名称为BLH)
	ini_pos_var.assign(1, 1, inistate_std.Blh.B * inistate_std.Blh.B);
	ini_pos_var.assign(2, 2, inistate_std.Blh.L * inistate_std.Blh.L);
	ini_pos_var.assign(3, 3, inistate_std.Blh.H * inistate_std.Blh.H);

	ini_vel_var = inistate_std.Vel.diag() * inistate_std.Vel.diag();
	//角度标准差输入单位为deg
	ini_angle_var.assign(3, 3, Deg2Rad(inistate_std.Angle.yaw) * Deg2Rad(inistate_std.Angle.yaw));
	ini_angle_var.assign(2, 2, Deg2Rad(inistate_std.Angle.pitch) * Deg2Rad(inistate_std.Angle.pitch));
	ini_angle_var.assign(1, 1, Deg2Rad(inistate_std.Angle.roll) * Deg2Rad(inistate_std.Angle.roll));
	
	//初始零偏方差和比例因子方差默认使用噪声参数中的
	Cov = (ini_pos_var, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, ini_vel_var, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, ini_angle_var, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, gb_var, Zero_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, ab_var, Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, gs_var, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, as_var);

	//Cov = 2 * Cov;

}
//添加imu数据所需具有的格式：第一列为GPS周秒，2-4列为陀螺仪输出(rad),5-7列为加速度计输出(m/s)，共7列
void Body::AddIMUData(double* OneLineIMU)
{
	IMUData_pre = IMUData_cur;
	IMUData_cur.time = OneLineIMU[0];
	IMUData_cur.AngInc.GetFormArray(1, OneLineIMU);
	IMUData_cur.VelInc.GetFormArray(4, OneLineIMU);

	Compensate();
}
//添加GNSS数据所需具有的格式：第一列GPS周秒，2-4列经纬高(deg,m),5-7列经纬高方差，8-10列北东地速度(m/s),11-13列北东地速度方差，共13列
void Body::AddGNSSData(double* OneLineGNSS)
{
	GNSSData.time = OneLineGNSS[0];
	GNSSData.Blh.GetFormArray(1, OneLineGNSS);
	GNSSData.Blh_std.GetFormArray(4, OneLineGNSS);
	GNSSData.Vel.GetFormArray(7, OneLineGNSS);
	GNSSData.Vel_std.GetFormArray(10, OneLineGNSS);

	GNSSData.Blh.assign(1, 1, Deg2Rad(GNSSData.Blh.getelement(1, 1)));
	GNSSData.Blh.assign(2, 1, Deg2Rad(GNSSData.Blh.getelement(2, 1)));

	timestamp = GNSSData.time;
	GNSSData.DataAvailable = true;
}

void Body::AddODOData(double* OneLineODO)
{
	odoVel = OneLineODO[1];
}

bool Body::IsGNSSDataAvailable()
{
	return GNSSData.DataAvailable;
}

void Body::ImuInterpolate(IMUDATA& IMUData_pre, IMUDATA& IMUData_cur, double midtime, IMUDATA& IMUData_mid)
{
	if (IMUData_pre.time > midtime || IMUData_cur.time < midtime)
		return;

	double lambda = (midtime - IMUData_pre.time) / (IMUData_cur.time - IMUData_pre.time);

	IMUData_mid.time = midtime;
	IMUData_mid.AngInc = IMUData_cur.AngInc * lambda;
	IMUData_mid.VelInc = IMUData_cur.VelInc * lambda;

	//内插完后，虚拟出了一个中间时刻，故后一个时刻对前一刻的增量也随之改变，也应重新赋值
	IMUData_cur.AngInc = IMUData_cur.AngInc - IMUData_mid.AngInc;
	IMUData_cur.VelInc = IMUData_cur.VelInc - IMUData_mid.VelInc;
}

void Body::SetTimestamp(double time)
{
	timestamp = time;
}

void Body::PureINSwork()
{
	PureINSmech(IMUData_pre, IMUData_cur);
}

void Body::ImuPropagation(IMUDATA& IMUData_pre, IMUDATA& IMUData_cur)
{
	//IMU机械编排算法状态更新
	PureINSmech(IMUData_pre, IMUData_cur);

	Matrix F(21, 21);
	Matrix Q(21, 21);
	Matrix G(21, 18);

	// 使用上一历元状态计算状态转移矩阵
	double B = Blh_pre.B;
	double L = Blh_pre.L;
	double h = Blh_pre.H;

	double VN = Vel_pre.getelement(1, 1);
	double VE = Vel_pre.getelement(2, 1);
	double VD = Vel_pre.getelement(3, 1);

	double RM = RadiusOfMeridianCircle(B, CGCS2000_a, CGCS2000_e2);
	double RN = RadiusOfUnitaryCircle(B, CGCS2000_a, CGCS2000_e2);

	double cosB = cos(Deg2Rad(B));
	double secB = 1.0 / cosB;
	double tanB = tan(Deg2Rad(B));
	double sinB = sin(Deg2Rad(B));

	double gp = gravity(B, h);

	Matrix Frr(3, 3, '|',
	-VD / (RM + h), 0.0, VN / (RM + h),
	VE * tanB / (RN + h), -(VD + VN * tanB) / (RN + h), VE / (RN + h),
	0.0, 0.0, 0.0);

	Matrix Fvr(3, 3, '|',
		-2.0 * VE * BDS_we * cosB / (RM + h) - VE * VE * secB * secB / ((RM + h) * (RN + h)), 0.0, VN * VD / ((RM + h) * (RM + h)) - VE * VE * tanB / ((RN + h) * (RN + h)),
		2.0 * BDS_we * (VN * cosB - VD * sinB) / (RM + h) + VN * VE * secB * secB / ((RM + h) * (RN + h)), 0.0, (VE * VD + VN * VE * tanB) / ((RN + h) * (RN + h)),
		2.0 * BDS_we * VE * sinB / (RM + h), 0.0, -VE * VE / ((RN + h) * (RN + h)) - VN * VN / ((RM + h) * (RM + h)) + 2.0 * gp / (sqrt(RM * RN) + h));

	Matrix Fvv(3, 3, '|',
		VD / (RM + h), -2.0 * (BDS_we * sinB + VE * tanB / (RN + h)), VN / (RM + h),
		2.0 * BDS_we * sinB + VE * tanB / (RN + h), (VD + VN * tanB) / (RN + h), 2.0 * BDS_we * cosB + VE / (RN + h),
		-2.0 * VN / (RM + h), -2.0 * (BDS_we * cosB + VE / (RN + h)), 0.0);

	Matrix Fphir(3, 3, '|',
		-BDS_we * sinB / (RM + h), 0.0, VE / ((RN + h) * (RN + h)),
		0.0, 0.0, -VN / ((RM + h) * (RM + h)),
		-BDS_we * cosB / (RM + h) - VE * secB * secB / ((RM + h) * (RN + h)), 0.0, -VE * tanB / ((RN + h) * (RN + h)));

	Matrix Fphiv(3, 3, '|',
		0.0, 1.0 / (RN + h), 0.0,
		-1.0 / (RM + h), 0.0, 0.0,
		0.0, -tanB / (RN + h), 0.0);

	Vector wn_ie(3, '|', BDS_we * cosB, 0.0, -BDS_we * sinB);
	Vector wn_en(3, '|', VE / (RN + h), -VN / (RM + h), -VE * tanB / (RN + h));
	Vector wn_in(3);

	wn_in = wn_ie + wn_en;

	double delta_t = IMUData_cur.time - IMUData_pre.time;
	Vector wb_ib(3);
	Vector fb(3);
	wb_ib = IMUData_cur.AngInc * (1.0 / delta_t);
	fb = IMUData_cur.VelInc * (1.0 / delta_t);


	Matrix I_33(3, 3);
	Matrix Zero_33(3, 3);
	I_33.ToE(3);

	Vector Cn_bfb(3);
	Cn_bfb = Cn_b_pre * fb;

	Matrix _1_T_I_33(3, 3);
	_1_T_I_33 = -1.0 / ImuNoise.corr_t * I_33;

	

	F = (Frr, I_33, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Fvr, Fvv, Cn_bfb.AntiSymMatrix(), Zero_33, Cn_b_pre, Zero_33, Cn_b_pre * fb.diag())
		& (Fphir, Fphiv, -1.0 * wn_in.AntiSymMatrix(), -1.0 * Cn_b_pre, Zero_33, -1.0 * Cn_b_pre * wb_ib.diag(), Zero_33)
		& (Zero_33, Zero_33, Zero_33, _1_T_I_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, _1_T_I_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, _1_T_I_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, _1_T_I_33);


	G = (Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Cn_b_pre, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, Cn_b_pre, Zero_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, I_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, I_33, Zero_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, I_33, Zero_33)
		& (Zero_33, Zero_33, Zero_33, Zero_33, Zero_33, I_33);


	//状态转移矩阵
	Matrix Phi(21, 21);
	Matrix I_2121(21, 21);
	I_2121.ToE(21);

	Phi = I_2121 + F * delta_t;

	Q = G * q * G.T() * delta_t;
	Q = (Phi * Q * Phi.T() + Q) * 0.5;

	//Q = 2 * Q;

	//由于状态总为0矩阵，故此步可以省略
	//x = Phi * x;
	Cov = Phi * Cov * Phi.T() + Q;
}


void Body::GNSSUpdating(IMUDATA& IMUData_mid)
{

	Matrix Dr_inv(3, 3);
	Matrix Dr(3, 3);

	Dr_inv = DR_inv(Blh, CGCS2000_a, CGCS2000_e2);
	Dr = DR(Blh, CGCS2000_a, CGCS2000_e2);

	Vector VBlh(3, '|', Deg2Rad(Blh.B), Deg2Rad(Blh.L), Blh.H);//IMU中心的BLH化为向量形式
	Vector IMU_to_GNSSBlh(3);//IMU位置和杆臂信息经计算得到的GNSS中心位置

	IMU_to_GNSSBlh = VBlh + Dr_inv * Cn_b * antlever;

	//位置观测向量表示为 INS 推算的位置与 GNSS 位置观测之差
	Vector Zr(3);
	Zr = Dr * (IMU_to_GNSSBlh - GNSSData.Blh );
	
	//速度观测向量可表示为 INS 推算的速度与 GNSS 解算的速度之差
	Vector Zv(3);
	Zv = Vel - GNSSData.Vel;

	Matrix Z(6, 1);
	Z = Zr & Zv;


	//构造观测方程的H矩阵
	Matrix H(6, 21);

	double B = Blh.B;
	double L = Blh.L;
	double h = Blh.H;
	double VN = Vel.getelement(1, 1);
	double VE = Vel.getelement(2, 1);
	double VD = Vel.getelement(3, 1);

	double RM = RadiusOfMeridianCircle(B, CGCS2000_a, CGCS2000_e2);
	double RN = RadiusOfUnitaryCircle(B, CGCS2000_a, CGCS2000_e2);

	double cosB = cos(Deg2Rad(B));
	double tanB = tan(Deg2Rad(B));
	double sinB = sin(Deg2Rad(B));


	Vector wn_ie(3, '|', BDS_we * cosB, 0.0, -BDS_we * sinB);
	Vector wn_en(3, '|', VE / (RN + h), -VN / (RM + h), -VE * tanB / (RN + h));
	Vector wn_in(3);

	wn_in = wn_ie + wn_en;

	Vector wb_ib(3);
	Vector fb(3);
	wb_ib = IMUData_mid.AngInc * (1.0 / (IMUData_mid.time - IMUData_pre.time));
	fb = IMUData_mid.VelInc * (1.0 /( IMUData_mid.time - IMUData_pre.time));

	Matrix I_33(3, 3);
	Matrix Zero_33(3, 3);
	I_33.ToE(3);

	Vector Cn_blb(3);
	Cn_blb = Cn_b * antlever;
	Vector Cn_b_lb_wbib(3);
	Cn_b_lb_wbib = Cn_b * (antlever->*wb_ib);

	Matrix Hv3(3, 3);
	Hv3 = -1.0 * wn_in.AntiSymMatrix() * Cn_blb.AntiSymMatrix() - Cn_b_lb_wbib.AntiSymMatrix();
	Matrix Hv6(3, 3);
	Hv6 = -1.0 * Cn_b * antlever.AntiSymMatrix() * wb_ib.diag();

	H = (I_33, Zero_33, Cn_blb.AntiSymMatrix(), Zero_33, Zero_33, Zero_33, Zero_33)
		& (Zero_33, I_33, Hv3, -1.0 * (Cn_blb).AntiSymMatrix(), Zero_33, Hv6, Zero_33);
	
	//观测噪声矩阵
	Matrix pos_var(3, 3);
	Matrix vel_var(3, 3);

	pos_var = GNSSData.Blh_std.diag() * GNSSData.Blh_std.diag();
	vel_var = GNSSData.Vel_std.diag() * GNSSData.Vel_std.diag();

	//Matrix R(6, 6);
	Matrix R = (pos_var, Zero_33) & (Zero_33, vel_var);

	//R = 2 * R;

	//卡尔曼滤波测量更新
	//Matrix K(21, 6);
	Matrix I_2121(1, 1);
	I_2121.ToE(21);

	Matrix K = Cov * H.T() * (H * Cov * H.T() + R).inv();
	//x = x + K * (Z - H * x);
	x = K * Z;
	Cov = (I_2121 - K * H) * Cov * (I_2121 - K * H).T() + K * R * K.T();

	//设置GNSS数据状态为不可用
	GNSSData.DataAvailable = false;

}

void Body::Feedback()
{
	Vector temp(3);
	//位置反馈
	double RM = RadiusOfMeridianCircle(Blh.B, CGCS2000_a, CGCS2000_e2);
	double RN = RadiusOfUnitaryCircle(Blh.B, CGCS2000_a, CGCS2000_e2);

	Blh.B -= Rad2Deg(x.getelement(1, 1) / (RM + Blh.H));
	Blh.L -= Rad2Deg(x.getelement(2, 1) / ((RN + Blh.H) * cos(Deg2Rad(Blh.B))));
	Blh.H += x.getelement(3, 1);

	//速度反馈
	temp = x.SubMatrix(4, 1, 3, 1);

	Vel = Vel - temp;

	//姿态反馈
	temp = x.SubMatrix(7, 1, 3, 1);

	Matrix I_33(3, 3);
	I_33.ToE(3);
	//Matrix Cp_n = I_33 - dphi.AntiSymMatrix();
	Matrix Cn_p = I_33 - temp.AntiSymMatrix();

	Cn_b = Cn_p.inv() * Cn_b;

	Angle.pitch = Rad2Deg(atan(-Cn_b.getelement(3, 1) / sqrt(Cn_b.getelement(3, 2) * Cn_b.getelement(3, 2) + Cn_b.getelement(3, 3) * Cn_b.getelement(3, 3))));
	Angle.roll = Rad2Deg(atan2(Cn_b.getelement(3, 2), Cn_b.getelement(3, 3)));
	Angle.yaw = Rad2Deg(atan2(Cn_b.getelement(2, 1), Cn_b.getelement(1, 1)));


	//IMU陀螺仪零偏反馈
	temp = x.SubMatrix(10, 1, 3, 1);
	ImuError.gyrbias = ImuError.gyrbias + temp;

	//IMU加速度计零偏反馈
	temp = x.SubMatrix(13, 1, 3, 1);
	ImuError.accbias = ImuError.accbias + temp;

	//IMU陀螺仪比例因子反馈
	temp = x.SubMatrix(16, 1, 3, 1);
	ImuError.gyrscale = ImuError.gyrscale + temp;

	//IMU加速度计比例因子反馈
	temp = x.SubMatrix(19, 1, 3, 1);
	ImuError.accscale = ImuError.accscale + temp;

	x.ToZero();

}
//对当前历元IMU数据进行补偿（IMUData_cur）
void Body::Compensate()
{
	//零偏补偿
	IMUData_cur.AngInc = IMUData_cur.AngInc - ImuError.gyrbias * (IMUData_cur.time - IMUData_pre.time);
	IMUData_cur.VelInc = IMUData_cur.VelInc - ImuError.accbias * (IMUData_cur.time - IMUData_pre.time);

	//比例因子补偿
	Matrix I_33(3, 3);
	I_33.ToE(3);

	Matrix Gyrscale = I_33 + ImuError.gyrscale.diag();
	Matrix Accscale = I_33 + ImuError.accscale.diag();

	IMUData_cur.AngInc = Gyrscale.inv() * IMUData_cur.AngInc;
	IMUData_cur.VelInc = Accscale.inv() * IMUData_cur.VelInc;

}

void Body::work()
{
	//如果当前时间小于时间标记，则继续传播IMU状态
	if (IMUData_cur.time < timestamp)
	{
		ImuPropagation(IMUData_pre, IMUData_cur);

		//零速更新
		//if (IsVelocityZero())
		//{
		//	ZUPT();
		//	Feedback();
		//}

		//NHC更新
		//NHC();
		//Feedback();


		//里程计更新
		//ODO();
		//Feedback();


	}
	//当前时间大于时间标记，则首先进行对时间标记时刻的状态内插，再判断是否需要进行GNSS测量值更新
	//一般起始时刻不更新，后面每个GNSS整秒都进行更新
	else if (IMUData_cur.time > timestamp)
	{
		IMUDATA IMUData_mid;

		//内插出中间时刻IMU增量数据
		ImuInterpolate(IMUData_pre, IMUData_cur, timestamp, IMUData_mid);

		//前半状态传播
		ImuPropagation(IMUData_pre, IMUData_mid);

		if ( GNSSData.DataAvailable == true )
		{
			//测量更新
			GNSSUpdating(IMUData_mid);
			Feedback();
		}

		ImuPropagation(IMUData_mid, IMUData_cur);

	}

	return;
	
}

bool Body::IsVelocityZero()
{
	//自定义零速修正算法
	//若连续多个历元读取的速度增量向量的模与静止时它的理论值之差的和小于阈值，则判断载体静止，给速度向量赋零
	//加速度模长阈值与采样率有关（通常二进制数据为0.005s，ASCII码为0.01秒，注意区分）
	static int count = 1;
	static double sum = 0;
	bool Zero = false;
	if (count == 100)
	{
		if (sum < 0.075)
		{
			Zero = true;
		}
		sum = 0;
		count = 0;
	}
	else
	{
		sum += fabs(IMUData_cur.VelInc.Norm() - 0.048975);
		count++;
	}

	return Zero;

}


void Body::ZUPT()
{
	Matrix Zero_33(3, 3);
	Matrix I_33(3, 3);
	I_33.ToE(3);
	Matrix I_2121(21, 21);
	I_2121.ToE(21);

	//构造观测方程的H矩阵
	Matrix H = (Zero_33, I_33, Zero_33, Zero_33, Zero_33, Zero_33, Zero_33);

	//零速更新观测向量（IMU推算速度-虚拟0速观测值）
	Matrix Z = Vel;

	//观测噪声矩阵
	Vector VR(3, 0.001 * 0.001);
	Matrix R = VR.diag();

	//卡尔曼滤波更新
	Matrix K = Cov * H.T() * (H * Cov * H.T() + R).inv();
	//x = x + K * (Z - H * x);
	x = K * Z;
	Cov = (I_2121 - K * H) * Cov * (I_2121 - K * H).T() + K * R * K.T();

}

//bool Body::IsNHC()
//{
//
//	static int count = 1;
//	static double sum = 0;
//
//	bool NHC = false;
//	if (count == 100)
//	{
//		if (sum < 0.1)
//		{
//			NHC = true;
//		}
//		sum = 0;
//		count = 0;
//	}
//	else
//	{
//		sum += fabs(Angle.yaw - Angle_pre.yaw);
//		count++;
//	}
//
//	return NHC;
//
//
//}

void Body::NHC()
{
	double B = Blh.B;
	double h = Blh.H;
	double VN = Vel.getelement(1, 1);
	double VE = Vel.getelement(2, 1);


	double RM = RadiusOfMeridianCircle(B, CGCS2000_a, CGCS2000_e2);
	double RN = RadiusOfUnitaryCircle(B, CGCS2000_a, CGCS2000_e2);

	double cosB = cos(Deg2Rad(B));
	double tanB = tan(Deg2Rad(B));
	double sinB = sin(Deg2Rad(B));


	Matrix Zero_33(3, 3);
	Matrix I_2121(21, 21);
	I_2121.ToE(21);

	Matrix I_23(2, 3, '|', 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
	Matrix Cb_n = Cn_b.T();

	Vector wn_ie(3, '|', BDS_we * cosB, 0.0, -BDS_we * sinB);
	Vector wn_en(3, '|', VE / (RN + h), -VN / (RM + h), -VE * tanB / (RN + h));
	Vector wn_in(3);

	wn_in = wn_ie + wn_en;


	double delta_t = IMUData_cur.time - IMUData_pre.time;
	Vector wb_ib(3);
	wb_ib = IMUData_cur.AngInc * (1.0 / delta_t);

	Vector wb_nb(3);
	wb_nb = wb_ib - Cb_n * wn_in;

	Matrix H_6 = -1.0 * odolever.AntiSymMatrix() * wb_ib.diag();
	//构造观测方程的H矩阵
	Matrix H = I_23 * (Zero_33, Cb_n, -1.0 * Cb_n * Vel.AntiSymMatrix(), -1.0 * odolever.AntiSymMatrix(), Zero_33, H_6, Zero_33);

	//NHC更新观测向量
	Matrix Z = I_23 * (Cb_n * Vel + wb_nb.AntiSymMatrix() * odolever);

	//观测噪声矩阵
	Matrix R(2, 2, '|', 0.02, 0.0, 0.0, 0.02);

	//卡尔曼滤波更新
	Matrix K = Cov * H.T() * (H * Cov * H.T() + R).inv();
	//x = x + K * (Z - H * x);
	x = K * Z;
	Cov = (I_2121 - K * H) * Cov * (I_2121 - K * H).T() + K * R * K.T();

}

void Body::ODO()
{
	double B = Blh.B;
	double h = Blh.H;
	double VN = Vel.getelement(1, 1);
	double VE = Vel.getelement(2, 1);
	

	double RM = RadiusOfMeridianCircle(B, CGCS2000_a, CGCS2000_e2);
	double RN = RadiusOfUnitaryCircle(B, CGCS2000_a, CGCS2000_e2);

	double cosB = cos(Deg2Rad(B));
	double tanB = tan(Deg2Rad(B));
	double sinB = sin(Deg2Rad(B));

	Vector wn_ie(3, '|', BDS_we * cosB, 0.0, -BDS_we * sinB);
	Vector wn_en(3, '|', VE / (RN + h), -VN / (RM + h), -VE * tanB / (RN + h));
	Vector wn_in(3);

	wn_in = wn_ie + wn_en;

	Vector wb_ib(3);
	wb_ib = IMUData_cur.AngInc * (1.0 / (IMUData_cur.time - IMUData_pre.time));

	Matrix Cb_n = Cn_b.inv();
	Vector wb_nb(3);
	wb_nb = wb_ib - Cb_n * wn_in;

	Vector Z(1);
	Vector odoV(1, odoVel);

	Matrix I_13(1, 3, '|', 1.0, 0.0, 0.0);
	Matrix Zero_33(3, 3);
	Matrix I_2121(21, 21);
	I_2121.ToE(21);

	//观测值向量
	Z = I_13 * (Cb_n * Vel + wb_nb.AntiSymMatrix() * odolever) - odoV;

	Matrix H_6 = -1.0 * odolever.AntiSymMatrix() * wb_ib.diag();
	//观测矩阵
	Matrix H = I_13 * (Zero_33, Cb_n, -1.0 * Cb_n * Vel.AntiSymMatrix(), -1.0 * odolever.AntiSymMatrix(), Zero_33, H_6, Zero_33);

	//观测噪声矩阵
	Matrix R(1, 1, 0.04);

	//卡尔曼滤波更新
	Matrix K = Cov * H.T() * (H * Cov * H.T() + R).inv();
	//x = x + K * (Z - H * x);
	x = K * Z;
	Cov = (I_2121 - K * H) * Cov * (I_2121 - K * H).T() + K * R * K.T();

}

void Body::BodyShow()
{
	cout << "t:  " << setprecision(6) << setiosflags(ios::fixed) << IMUData_cur.time ;
	cout << setw(4) <<  "B:" << setw(14) << setprecision(9) << setiosflags(ios::fixed) << Blh.B ;
	cout << setw(4) << "L:" << setw(14) << Blh.L;
	cout << setw(4) << "H:" << setw(14) << Blh.H;
	cout << setw(4) << "VN:" << setw(14) << Vel.getelement(1, 1);
	cout << setw(4) << "VE:" << setw(14) << Vel.getelement(2, 1);
	cout << setw(4) << "VD:" << setw(14) << Vel.getelement(3, 1);
	cout << setw(6) << "roll:" << setw(14) << Angle.roll;
	cout << setw(8) << "pitch:" << setw(14) << Angle.pitch;
	cout << setw(6) << "yaw:" << setw(14) << Angle.yaw << endl;

}
void Body::BodyOutputToFile(fstream& fout)
{

	fout << "t:  " << setprecision(6) << setiosflags(ios::fixed) << IMUData_cur.time;
	fout << setw(4) << "B:" << setw(14) << setprecision(9) << setiosflags(ios::fixed) << Blh.B;
	fout << setw(4) << "L:" << setw(14) << Blh.L;
	fout << setw(4) << "H:" << setw(14) << Blh.H;
	fout << setw(4) << "VN:" << setw(14) << Vel.getelement(1, 1);
	fout << setw(4) << "VE:" << setw(14) << Vel.getelement(2, 1);
	fout << setw(4) << "VD:" << setw(14) << Vel.getelement(3, 1);
	fout << setw(6) << "roll:" << setw(14) << Angle.roll;
	fout << setw(8) << "pitch:" << setw(14) << Angle.pitch;
	fout << setw(7) << "yaw:" << setw(16) << Angle.yaw << endl;
}
void Body::CovOutputToFile(fstream& fout)
{
	fout << setw(6) << "varB:" << setw(11) << setprecision(6) << setiosflags(ios::fixed) << Cov.getelement(1, 1);
	fout << setw(6) << "varL:" << setw(11) << Cov.getelement(2, 2);
	fout << setw(6) << "varH:" << setw(11) << Cov.getelement(3, 3);

	fout << setw(8) << "varVN:" << setw(11) << Cov.getelement(4, 4);
	fout << setw(8) << "varVE:" << setw(11) << Cov.getelement(5, 5);
	fout << setw(8) << "varVD:" << setw(11) << Cov.getelement(6, 6);

	fout << setw(10) << "varroll:" << setw(11) << Cov.getelement(7, 7) * R2D * R2D;//[deg^2]
	fout << setw(12) << "varpitch:" << setw(11) << Cov.getelement(8, 8) * R2D * R2D;
	fout << setw(10) << "varyaw:" << setw(11) << Cov.getelement(9, 9) * R2D * R2D;

	fout << setw(8) << "vargbx:" << setw(16) << Cov.getelement(10, 10) * (R2D * 3600.0) * (R2D * 3600.0);//[(deg/h)^2]
	fout << setw(8) << "vargby:" << setw(16) << Cov.getelement(11, 11) * (R2D * 3600.0) * (R2D * 3600.0);
	fout << setw(8) << "vargbz:" << setw(16) << Cov.getelement(12, 12) * (R2D * 3600.0) * (R2D * 3600.0);

	fout << setw(8) << "varabx:" << setw(16) << Cov.getelement(13, 13) * 1.0E5 * 1.0E5;//[(mGal)^2]
	fout << setw(8) << "varaby:" << setw(16) << Cov.getelement(14, 14) * 1.0E5 * 1.0E5;
	fout << setw(8) << "varabz:" << setw(16) << Cov.getelement(15, 15) * 1.0E5 * 1.0E5;

	fout << setw(8) << "vargsx:" << setw(16) << Cov.getelement(16, 16) * 1.0E6 * 1.0E6;//[(ppm)^2]
	fout << setw(8) << "vargsy:" << setw(16) << Cov.getelement(17, 17) * 1.0E6 * 1.0E6;
	fout << setw(8) << "vargsz:" << setw(16) << Cov.getelement(18, 18) * 1.0E6 * 1.0E6;

	fout << setw(8) << "varasx:" << setw(16) << Cov.getelement(19, 19) * 1.0E6 * 1.0E6;//[(ppm)^2]
	fout << setw(8) << "varasy:" << setw(16) << Cov.getelement(20, 20) * 1.0E6 * 1.0E6;
	fout << setw(8) << "varasz:" << setw(16) << Cov.getelement(21, 21) * 1.0E6 * 1.0E6 << endl;


}

