#include"InertialNavigation.h"
#include"Constant.h"
#include<math.h>
#include<iostream>
//�ȵ����Ӷ���Ĺ��캯�����Ӷ����ʼ��
IMUBOX::IMUBOX(BLH B, Vector V, ANGLE A) :Vel(3), Cn_b(3, 3),Vel_pre(3),Cn_b_pre(3, 3)
{
	//����һʱ�̸�ͬ���ĳ�ֵ
	Vel = V;
	Vel_pre = V;
	Blh = B;
	Blh_pre = B;
	Angle = A;
	Angle_pre = A;
	Cn_b = RotationMatrix(Deg2Rad(-Angle.roll), Deg2Rad(-Angle.pitch), Deg2Rad(-Angle.yaw), 6);
	Cn_b_pre = Cn_b;
}

IMUBOX::~IMUBOX()
{
}

void IMUBOX::PureINSmech(IMUDATA IMUData_pre,IMUDATA IMUData_cur)
{

	Vector AngInc_k(3), AngInc_k_1(3), VelInc_k(3), VelInc_k_1(3);
	AngInc_k = IMUData_cur.AngInc;
	AngInc_k_1 = IMUData_pre.AngInc;
	VelInc_k = IMUData_cur.VelInc;
	VelInc_k_1 = IMUData_pre.VelInc;
	double delta_t = IMUData_cur.time - IMUData_pre.time;

	  static int times = 1;

	//bϵ��Ӧ��Cb(k-1)_b(k)
	Vector V_b(3);//��̬���µĵ�Ч��תʸ��
	V_b = AngInc_k + (1.0 / 12.0) * AngInc_k_1->*AngInc_k;//->*Ϊ�������
	
	Matrix I(3, 3); I.ToE(3);//������λ��

	Matrix Cb(3, 3);
	if (fabs(AngInc_k.getelement(1, 1)) < 1.0E-12 && fabs(AngInc_k.getelement(2, 1)) < 1.0E-12 && fabs(AngInc_k.getelement(3, 1)) < 1.0E-12)
	{
		Cb = I;
	}
	else
	{
		Cb = I + (sin(V_b.Norm()) / V_b.Norm()) * V_b.AntiSymMatrix() + ((1 - cos(V_b.Norm())) / (V_b.Norm() * V_b.Norm())) * V_b.AntiSymMatrix() * V_b.AntiSymMatrix();
	}
	//�����м�k-1/2ʱ�̵�λ�ú��ٶ�
	Vector Vel_half(3);
	Vel_half = (3.0 / 2.0) * Vel - (1.0 / 2.0) * Vel_pre;
	double B_half = (3.0 / 2.0) * Blh.B - (1.0 / 2.0) * Blh_pre.B;
	double L_half = (3.0 / 2.0) * Blh.L - (1.0 / 2.0) * Blh_pre.L;
	double H_half = (3.0 / 2.0) * Blh.H - (1.0 / 2.0) * Blh_pre.H;

	Vector w_ie(3);
	double aw_ie[3] = { BDS_we * cos(Deg2Rad(B_half)),0.0,-BDS_we * sin(Deg2Rad(B_half)) };
	w_ie = aw_ie;

	Vector w_en(3);
	double R_M = RadiusOfMeridianCircle(B_half, CGCS2000_a, CGCS2000_e2);//����Ȧ�뾶
	double R_N = RadiusOfUnitaryCircle(B_half, CGCS2000_a, CGCS2000_e2);//î��Ȧ�뾶
	double aw_en[3] = { Vel_half.getelement(2,1) / (R_N + H_half),-Vel_half.getelement(1,1) / (R_M + H_half),-Vel_half.getelement(2,1) * tan(Deg2Rad(B_half)) / (R_N + H_half) };
	w_en = aw_en;
	
	Vector V_n(3);
	V_n = (w_ie + w_en) * delta_t;
	
	//nϵ��Ӧ��Cn(k)_n(k-1)
	Matrix Cn(3, 3);
	Cn = I - (sin(V_n.Norm()) / V_n.Norm()) * V_n.AntiSymMatrix() + ((1 - cos(V_n.Norm())) / (V_n.Norm() * V_n.Norm())) * V_n.AntiSymMatrix() * V_n.AntiSymMatrix();

	//�ٶȸ��µı���������
	Vector delta_Vel_b_f(3);
	Vector delta_Vel_n_f(3);
	delta_Vel_b_f = VelInc_k + (1.0 / 2.0) * AngInc_k->*VelInc_k + (1.0 / 12.0) * (AngInc_k_1->*VelInc_k + VelInc_k_1->*AngInc_k);
	delta_Vel_n_f = (I - (1.0 / 2.0) * V_n.AntiSymMatrix()) * Cn_b * delta_Vel_b_f;

	//�ٶȸ��µĸ�����
	double ag[3] = { 0.0,0.0,gravity(B_half,H_half) };
	Vector g(3);
	g = ag;
	Vector delta_Vel_n_g(3);
	delta_Vel_n_g = (g - (2 * w_ie->*Vel_half + w_en->*Vel_half)) * delta_t;

	//kʱ���ٶ�
	Vector Vel_k(3);
	Vel_k = Vel + delta_Vel_n_f + delta_Vel_n_g;

	//�ڲ�õ���k-1/2ʱ�̵��ٶ�
	Vel_half = (Vel_k + Vel) * (1.0 / 2.0);
	
	//kʱ��λ��
	double H_k = Blh.H - Vel_half.getelement(3, 1) * delta_t;
	H_half = (H_k + Blh.H) / 2.0;
	double B_k = Blh.B + Rad2Deg(Vel_half.getelement(1, 1) / ( R_M +  H_half) * delta_t);
	B_half = (B_k + Blh.B) / 2.0;
	R_N = RadiusOfUnitaryCircle(B_half, CGCS2000_a, CGCS2000_e2);
	double L_k = Blh.L + Rad2Deg(Vel_half.getelement(2, 1) / (R_N + H_half) / cos(Deg2Rad(B_half)) * delta_t);

	//��ǰ��ʱ�̵�λ���ٶȸ���
	if (times != 1)
	{
		Blh_pre = Blh;
		Vel_pre = Vel;
		Angle_pre = Angle;
		Cn_b_pre = Cn_b;
	}

	//λ�ø���
	Blh.B = B_k;
	Blh.L = L_k;
	Blh.H = H_k;

	//�ٶȸ���
	Vel = Vel_k;

	//�������Ҿ������
	Cn_b = Cn * Cn_b * Cb;

	//��̬�Ǹ���
	Angle.pitch = Rad2Deg(atan(-Cn_b.getelement(3, 1) / sqrt(Cn_b.getelement(3, 2) * Cn_b.getelement(3, 2) + Cn_b.getelement(3, 3) * Cn_b.getelement(3, 3))));
	Angle.roll = Rad2Deg(atan2(Cn_b.getelement(3, 2), Cn_b.getelement(3, 3)));
	Angle.yaw = Rad2Deg(atan2(Cn_b.getelement(2, 1), Cn_b.getelement(1, 1)));

	times++;

}


void IMUBOX::Show()
{
	printf("B:%13.10f L:%13.10f H:%7.5f VN:%15.12f VE:%15.12f VD:%15.12f yaw:%14.10f pitch:%14.10f roll:%14.10f\n", Blh.B, Blh.L, Blh.H, Vel.getelement(1, 1), Vel.getelement(2, 1), Vel.getelement(3, 1), Angle.yaw, Angle.pitch, Angle.roll);
}

void IMUBOX::output(FILE* fp)
{
	fprintf(fp, "B:%13.10f L:%13.10f H:%7.5f VN:%10.7f VE:%10.7f VD:%10.7f yaw:%12.8f pitch:%12.8f roll:%12.8f\n", Blh.B, Blh.L, Blh.H, Vel.getelement(1, 1), Vel.getelement(2, 1), Vel.getelement(3, 1), Angle.yaw, Angle.pitch, Angle.roll);
}

BLH IMUBOX::getBlh()
{
	return Blh;
}

Vector IMUBOX::getVel()
{
	return Vel;
}

ANGLE IMUBOX::getAngle()
{
	return Angle;
}

Matrix IMUBOX::getCn_b()
{
	return Cn_b;
}

//��ת����ת��̬��
void Cn_b2Eular(Matrix Cn_b, ANGLE& Angle)
{
	Angle.pitch = Rad2Deg(atan(-Cn_b.getelement(3, 1) / sqrt(Cn_b.getelement(3, 2) * Cn_b.getelement(3, 2) + Cn_b.getelement(3, 3) * Cn_b.getelement(3, 3))));
	Angle.roll = Rad2Deg(atan2(Cn_b.getelement(3, 2), Cn_b.getelement(3, 3)));
	Angle.yaw = Rad2Deg(atan2(Cn_b.getelement(2, 1), Cn_b.getelement(1, 1)));
}

//��̬��ת��ת����
void Eular2Cn_b(ANGLE& Angle, Matrix Cn_b)
{
	Cn_b = RotationMatrix(Deg2Rad(-Angle.roll), Deg2Rad(-Angle.pitch), Deg2Rad(-Angle.yaw), 6);
}