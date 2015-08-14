/* 
 * Last Updated at [2014/12/29 18:12] by wuhao
 */
#include "StringOperator.h"

double StringOperator::precision = 1e-6;

double StringOperator::stringToDouble(const string str)
{
	//TODO:�쳣ֵ���

	if (str[0] == '-')
	{
		return  -1.0 * stringToPositiveDouble(str, 1, str.size());
	}
	else
	{
		return stringToPositiveDouble(str, 0, str.size());
	}
}


string StringOperator::doubleToString(const double value)
{
	string sign = "";

	double tempValue = value;
	if (tempValue < 0)
	{
		sign = "-";
		tempValue *= -1.0;
	}

	int integerValue = static_cast<int>(tempValue);
	double decimalValue = tempValue - integerValue;

	string integerPart = intToString(integerValue);
	string decimalPart = "";

	double base = 0.1;
	while (decimalValue > precision)
	{
		int times = static_cast<int>(decimalValue / base);
		decimalPart += '0' + times;
		decimalValue -= times * base;
		base /= 10.0;
	}
	if (abs(int(value) - value) < precision)
		return sign + integerPart;
	else
		return sign + integerPart + "." + decimalPart;
}

void StringOperator::setPrecision(const double newPrecision)
{
	precision = newPrecision;
}

string StringOperator::intToString(const int value)
{
	if (value == 0)
	{
		return "0";
	}

	string result = "";
	int tempValue = value;

	if (tempValue < 0)
	{
		result += "-";
		tempValue *= -1;
	}

	stack<int> number;	//�洢�����ÿһλ�����ڵ��򷵻�
	while (tempValue > 0)
	{
		number.push(tempValue % 10);
		tempValue /= 10;
	}
	while (!number.empty())
	{
		int topNumber = number.top();
		number.pop();
		result += '0' + topNumber;
	}

	return result;
}

int StringOperator::stringToInt(const string str)
{
	//TODO:�쳣ֵ���

	if (str[0] == '-')
	{
		return  -1 * stringToPositiveInt(str, 1, str.size());
	}
	else
	{
		return stringToPositiveInt(str, 0, str.size());
	}
}

int StringOperator::stringToPositiveInt(const string str, const int startIndex, const int endIndex)
{
	int value = 0;
	for (int i = startIndex; i != endIndex; ++i)
	{
		value = value * 10 + (str[i] - '0');
	}
	return value;
}

double StringOperator::stringToPositiveDouble(const string str, const int startIndex, const int endIndex)
{

	int integerValue = 0;
	int pointPos = 0;	//С����λ��
	//�����������ֵ�ֵ�����С����λ��
	for (int i = startIndex; i != endIndex; ++i)
	{
		if (str[i] == '.')
		{
			pointPos = i;
			break;
		}

		integerValue = integerValue * 10 + (str[i] - '0');
	}

	double decimalValue = 0.0;
	//����С�����ֵ�ֵ
	double base = 10.0;
	for (int i = pointPos + 1; i != endIndex; ++i)
	{
		decimalValue += (str[i] - '0') / base;
		base *= 10;
	}

	return static_cast<double>(integerValue)+decimalValue;
}