#include"MyVector2.h"
#include<iostream>
//���û��๹�캯����ʼ��
Vector::Vector(int n,double value):Matrix(n, 1, value)
{
	
}
/*�ƺ�û�а취ֱ�ӵ��ø���Ķ�������캯����
���ȼ�װ������һ�����๹�캯����ʼ�������ԱM�������ʼ��Ϊ�㣩��֮�����Լ�дһ������ĸ�ֵ������ȡ������б��M��ֵ*/
Vector::Vector(int n, char delimiter, ...): Matrix(n, 1)
{
	va_list p;//ָ�������ָ��
	va_start(p, delimiter);//��ʼ��ָ�������ָ�룬�ڶ��������ǿɱ������ǰһ��������

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
	{
		M[i] = va_arg(p, double);//��ȡ�ɱ�������ڶ�������Ϊ�ɱ���������͡�
	}

	va_end(p);
}

/*�����Ͷ�������ֵ������˵������һά�ľ���Ӧ�ÿ���ʹ�þ��������=�ĸ�ֵ���������������Ի��ǲ���
��Ϊ�����������ԶԻ����������ó�ʼ����ֵ�����ԼӼ������˷������˶������þ���ģ�����ֵ������Ҫ��д*/
Vector& Vector::operator=(const Vector &v)
{
	if (this == &v)
		return *this;
	
	if (MatrixRows != v.MatrixRows)
	{
		delete[] M;
		MatrixRows = v.MatrixRows;
		Initialize();
	}

	for (int i = 0; i < MatrixRows; i++)
		M[i] = v.M[i];

	return *this;
}

Vector& Vector::operator=(const Matrix& m)
{
	if (m.getcol() != 1)
	{
		printf("The columns of Matrix must be 1");
		return *this;
	}

	if (MatrixRows != m.getrow())
	{
		delete[] M;
		MatrixRows = m.getrow();
		Initialize();
	}

	for (int i = 0; i < MatrixRows; i++)
		M[i] = m.getelement(i + 1, 1);

	return *this;

}

Vector& Vector::operator=(const double Array[])
{
	for (int i = 0; i < MatrixRows; i++)
		M[i] = Array[i];

	return *this;
}

Vector& Vector::GetFormArray(int pos, const double Array[])
{
	for (int i = 0; i < MatrixRows; i++)
		M[i] = Array[pos + i];

	return *this;
}

double Vector::Norm()const
{
	//ģ�������Լ�ת�ó����Լ��ٿ�����
	return(sqrt((this->T() * (*this)).getelement(1, 1)));
}

Vector Vector::operator->*(const Vector&v)const
{
	//��֧����ά�����������
	if (MatrixRows != 3 || v.MatrixRows != 3)
	{
		printf("only 3 dimension can do the cross product");
		return *this;
	}
	Vector mulv(3);

	mulv.M[0] = M[1] * v.M[2] - M[2] * v.M[1];
	mulv.M[1] = -(M[0] * v.M[2] - M[2] * v.M[0]);
	mulv.M[2] = M[0] * v.M[1] - M[1] * v.M[0];

	return mulv;
}

void Vector::ToUnit()
{
	*this=*this* (1.0 / this->Norm());
	return;
}

Matrix Vector::AntiSymMatrix()
{
	Matrix Atm(3, 3);

	if (MatrixRows != 3)
	{
		std::cout << "only 3 dimension can find the Antisymmetric matrix";
		return Atm;
	}

	Atm.assign(1, 2, -M[2]);
	Atm.assign(1, 3, M[1]);
	Atm.assign(2, 1, M[2]);
	Atm.assign(2, 3, -M[0]);
	Atm.assign(3, 1, -M[1]);
	Atm.assign(3, 2, M[0]);

	return Atm;
	
}


Matrix Vector::diag()
{
	Matrix diag(MatrixRows,MatrixRows);

	for (int i = 0; i < MatrixRows; i++)
		diag.assign(i + 1, i + 1, M[i]);

	return diag;

}