#pragma once

#include<string>
#include<vector>
#include<stdarg.h>//多参数函数
#include<algorithm>//容器元素排序
using namespace std;

//分隔符类型为string，分隔符中不可包含另一个分隔符 
//(必须要把可变参数的个数传入，而不能根据情况获取吗？)
vector<string> StrSplit(string str, int DelimiterNumber, ...);