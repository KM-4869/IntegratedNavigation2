#pragma once
#include"Matrix2.h"
//#include"InertialNavigation.h";
class Vector :public Matrix
{
public:
	explicit Vector(int n,double value=0.0);//explicit�ؼ�������ֻ��һ������������������Ĭ�ϣ��Ĺ��캯������ֹ��ʽת������vector v=3��
	Vector(int, char, ...);
	Vector& operator=(const Vector&);
	Vector& operator=(const Matrix&);
	Vector& operator=(const double Array[]);

	Vector& GetFormArray(int pos,const double Array[]);//�������е�����λ��ȡֵ.posΪ�����±ꡣȡ����Ϊʸ������

	double Norm()const;//����ȡģ
	Vector operator->*(const Vector&)const;//�������
	void ToUnit();//������λ��

	Matrix AntiSymMatrix();//��ȡ�����ķ��Գƾ���
	Matrix diag();//���������һ���ԽǾ���

};