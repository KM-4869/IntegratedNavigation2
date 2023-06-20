#pragma once
#include<fstream>
#include<vector>
#include<string>
#include"Mystring2.h"
using namespace std;

static const int TEXT = 0;
static const int BINARY = 1;

class FileLoader
{
public:


	FileLoader() = default;
	//����1�ļ���������2һ�л�һ�����ݵĸ���,����3�ļ�����
	FileLoader(const string& filename, int datecount, int filetype);


	bool open(const string& filename, int datacount, int filetype);
	bool isEof();

	vector<double> load();
	bool load(vector<double>& onelinedata);

	void Skip_n_Line(int n);//����TEXT�ļ���ǰN��



protected:

	fstream fin;
	int DataCount;
	int FileType;

	vector<double> OneLineData;

	bool load_();

};

