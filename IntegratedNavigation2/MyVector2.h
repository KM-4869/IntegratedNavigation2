#pragma once
#include"Matrix2.h"
//#include"InertialNavigation.h";
class Vector :public Matrix
{
public:
	explicit Vector(int n,double value=0.0);//explicit关键字用于只用一个参数（或其他都是默认）的构造函数，防止隐式转换（如vector v=3）
	Vector(int, char, ...);
	Vector& operator=(const Vector&);
	Vector& operator=(const Matrix&);
	Vector& operator=(const double Array[]);

	Vector& GetFormArray(int pos,const double Array[]);//从数组中的任意位置取值.pos为数组下标。取长度为矢量长度

	double Norm()const;//向量取模
	Vector operator->*(const Vector&)const;//向量叉乘
	void ToUnit();//向量单位化

	Matrix AntiSymMatrix();//求取向量的反对称矩阵
	Matrix diag();//把向量变成一个对角矩阵

};