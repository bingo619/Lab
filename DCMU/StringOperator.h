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
	static double stringToPositiveDouble(const string str, const int startIndex, const int endIndex);	//��str[startIndex...endIndex]�Ӵ�ת��Ϊ��������
	static int stringToPositiveInt(const string str, const int startIndex, const int endIndex);	////��str[startIndex...endIndex]�Ӵ�ת��Ϊ������
	
	static double precision;//������ת�ַ���ʱ�ľ������ã�1e��-n����ʾ��ȷ��С�����nλ
};
#endif