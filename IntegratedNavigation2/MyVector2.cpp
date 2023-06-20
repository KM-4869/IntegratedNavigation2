#include"MyVector2.h"
#include<iostream>
//调用基类构造函数初始化
Vector::Vector(int n,double value):Matrix(n, 1, value)
{
	
}
/*似乎没有办法直接调用父类的多参数构造函数，
故先假装用另外一个父类构造函数初始化父类成员M（这里初始化为零），之后再自己写一个子类的赋值函数获取多参数列表给M赋值*/
Vector::Vector(int n, char delimiter, ...): Matrix(n, 1)
{
	va_list p;//指向参数的指针
	va_start(p, delimiter);//初始化指向参数的指针，第二个参数是可变参数的前一个参数。

	for (int i = 0; i < MatrixRows * MatrixCols; i++)
	{
		M[i] = va_arg(p, double);//获取可变参数，第二个参数为可变参数的类型。
	}

	va_end(p);
}

/*各类型对向量赋值。按理说向量是一维的矩阵，应该可以使用矩阵的重载=的赋值函数，但几经尝试还是不行
因为派生类对象可以对基类对象的引用初始化或赋值，所以加减法，乘法，数乘都可以用矩阵的，但赋值好像需要重写*/
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
	//模长等于自己转置乘以自己再开根号
	return(sqrt((this->T() * (*this)).getelement(1, 1)));
}

Vector Vector::operator->*(const Vector&v)const
{
	//仅支持三维向量叉乘运算
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