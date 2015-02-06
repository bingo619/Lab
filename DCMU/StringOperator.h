/* 
 * Last Updated at [2014/12/29 18:12] by wuhao
 */
#ifndef STRINGOPERATOR_H
#define STRINGOPERATOR_H

#include <string>
#include <stack>

using namespace std;

class StringOperator
{
public:
	static double stringToDouble(const string str);
	static string doubleToString(const double value);
	static void setPrecision(const double newPrecision);

	static string intToString(const int value);
	static int stringToInt(const string str);

private:
	static double stringToPositiveDouble(const string str, const int startIndex, const int endIndex);	//将str[startIndex...endIndex]子串转换为正浮点数
	static int stringToPositiveInt(const string str, const int startIndex, const int endIndex);	////将str[startIndex...endIndex]子串转换为正整数
	
	static double precision;//浮点数转字符串时的精度设置，1e（-n）表示精确到小数点后n位
};
#endif