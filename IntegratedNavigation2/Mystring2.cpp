#include"Mystring2.h"


vector<string> StrSplit(string str, int DelimiterNumbers, ...)
{
	vector<string> str_split;
	vector<string> delimiter;
	vector<int> delimiter_beginpos_endpos;

	va_list p;//指向参数的指针
	va_start(p, DelimiterNumbers);//初始化指向参数的指针，第二个参数是可变参数的前一个参数。


	//获取所有的分隔符
	for (int i = 0; i < DelimiterNumbers; i++)
		delimiter.push_back(va_arg(p, string));
	
	//查找所有分隔符
	for (int i = 0; i < delimiter.size(); i++)
	{
		int pos = 0;
		int delimiter_length = delimiter[i].length();

		while (true)
		{
			//从pos位置向后（The first character is denoted by a value of 0 (not 1)）找分隔符
			pos = str.find(delimiter[i],pos);

			if (pos == string::npos)
				break;

			//获取分隔符开始与结束位置信息
			delimiter_beginpos_endpos.push_back(pos);
			delimiter_beginpos_endpos.push_back(pos + delimiter_length);
			
			pos += delimiter_length;
		}
	}
	//分割符位置排序
	sort(delimiter_beginpos_endpos.begin(), delimiter_beginpos_endpos.end());

	//取子字符串
	int pos = 0;
	for (int i = 0; i < delimiter_beginpos_endpos.size(); i = i + 2)
	{
		string substr = str.substr(pos, delimiter_beginpos_endpos[i] - pos);

		if(!substr.empty())
		str_split.push_back(substr);

		pos = delimiter_beginpos_endpos[i + 1];
	}

	if (!str.substr(pos, str.length() - pos).empty())
		str_split.push_back(str.substr(pos, str.length() - pos));

	return str_split;

}