#pragma once

#include<string>
#include<vector>
#include<stdarg.h>//���������
#include<algorithm>//����Ԫ������
using namespace std;

//�ָ�������Ϊstring���ָ����в��ɰ�����һ���ָ��� 
//(����Ҫ�ѿɱ�����ĸ������룬�����ܸ��������ȡ��)
vector<string> StrSplit(string str, int DelimiterNumber, ...);