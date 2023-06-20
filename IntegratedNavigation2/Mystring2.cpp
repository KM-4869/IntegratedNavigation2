#include"Mystring2.h"


vector<string> StrSplit(string str, int DelimiterNumbers, ...)
{
	vector<string> str_split;
	vector<string> delimiter;
	vector<int> delimiter_beginpos_endpos;

	va_list p;//ָ�������ָ��
	va_start(p, DelimiterNumbers);//��ʼ��ָ�������ָ�룬�ڶ��������ǿɱ������ǰһ��������


	//��ȡ���еķָ���
	for (int i = 0; i < DelimiterNumbers; i++)
		delimiter.push_back(va_arg(p, string));
	
	//�������зָ���
	for (int i = 0; i < delimiter.size(); i++)
	{
		int pos = 0;
		int delimiter_length = delimiter[i].length();

		while (true)
		{
			//��posλ�����The first character is denoted by a value of 0 (not 1)���ҷָ���
			pos = str.find(delimiter[i],pos);

			if (pos == string::npos)
				break;

			//��ȡ�ָ�����ʼ�����λ����Ϣ
			delimiter_beginpos_endpos.push_back(pos);
			delimiter_beginpos_endpos.push_back(pos + delimiter_length);
			
			pos += delimiter_length;
		}
	}
	//�ָ��λ������
	sort(delimiter_beginpos_endpos.begin(), delimiter_beginpos_endpos.end());

	//ȡ���ַ���
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