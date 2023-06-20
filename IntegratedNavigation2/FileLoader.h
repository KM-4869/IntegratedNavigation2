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
	//参数1文件名，参数2一行或一组数据的个数,参数3文件类型
	FileLoader(const string& filename, int datecount, int filetype);


	bool open(const string& filename, int datacount, int filetype);
	bool isEof();

	vector<double> load();
	bool load(vector<double>& onelinedata);

	void Skip_n_Line(int n);//跳过TEXT文件的前N行



protected:

	fstream fin;
	int DataCount;
	int FileType;

	vector<double> OneLineData;

	bool load_();

};

