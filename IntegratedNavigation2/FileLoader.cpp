#include "FileLoader.h"

FileLoader::FileLoader(const string& filename, int datacount, int filetype)
{
	open(filename, datacount, filetype);
	OneLineData.resize(DataCount);
}

bool FileLoader::open(const string& filename, int datacount, int filetype)
{
	auto type = filetype == TEXT ? std::ios_base::in : (std::ios_base::in | std::ios_base::binary);
	fin.open(filename, type);

	DataCount = datacount;
	FileType = filetype;

	return fin.is_open();

}

bool FileLoader::isEof()
{
	return fin.eof();
}

vector<double> FileLoader::load()
{
	load_();

	return OneLineData;
}

bool FileLoader::load(vector<double>& onelinedata)
{
	if (load_())
	{
		//onelinedata = std::move(OneLineData);
		std::copy(OneLineData.begin(), OneLineData.end(), onelinedata.begin());
		return true;
	}

	return false;
}

//��һ�л�һ��DataCount�����ݵ�OneLineData��
bool FileLoader::load_()
{
	if (fin.eof())
		return false;


	//���ı��ļ�
	if (FileType == TEXT)
	{
		string line;
		std::getline(fin, line);
		if (line.empty())
			return false;
		//�������Լ���ո�ָ�һ������
		string d1(",");
		string d2(" ");
	
		vector<string> split_line = StrSplit(line, 2, d1, d2);
		//�����ʷ����
		//OneLineData.clear();
		//��ȡ������
		// for (auto i : split_line)
		// OneLineData.push_back(stod(i));
		for (int i = 0; i < DataCount; i++)
			OneLineData[i] = stod(split_line[i]);

	}
	else if (FileType == BINARY)
	{
		fin.read((char*)OneLineData.data(), sizeof(double) * double(DataCount));
	}

	return true;
}

void FileLoader::Skip_n_Line(int n)
{
	if (FileType == BINARY)
		return;

	string str;
	for (int i = 0; i < n; i++)
	{
		std::getline(fin, str);
	}

	return;
}