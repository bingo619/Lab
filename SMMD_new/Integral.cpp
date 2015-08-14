/* 
 * Last Updated at [2015/2/12 15:48] by wuhao
 */
#include "Integral.h"


double Integral::integral_Trapez(double(*f)(double), double a, double b, int m)
{
	//////////////////////////////////////////////////////////////////////////
	///�û��������η���ʵ��
	///������: [a, b]
	///m: ����
	//////////////////////////////////////////////////////////////////////////
	double h = (b - a) / m;
	double result = 0;
	for (int i = 1; i <= m - 1; i++)
	{
		result += f(a + i * h);
	}
	result *= 2;
	result += (f(a) + f(b));
	result *= h / 2;
	return result;
}

double Integral::integral_Simpson(double(*f)(double), double a, double b, int m)
{
	//////////////////////////////////////////////////////////////////////////
	///�û���������ɭ����ʵ��
	///������: [a, b]
	///m: ����
	///int(f,x1,x2) �� h / 6 * [f(x1) + 4f((x1+x2)/2) + f(x2)]  (h = x2-x1)
	//////////////////////////////////////////////////////////////////////////
	double h = (b - a) / m;
	double result = 0;

	result += f(a) + 4 * f(a + 0.5 * h);

	for (int i = 1; i < m; i++)
	{
		result += 2 * f(a + i * h);
		result += 4 * f(a + (i + 0.5) * h);
	}

	result += f(b);
	result *= (h / 6);
	return result;
}

double Integral::integral_2D_Trapez(double(*f)(double, double), double xa, double xb, double ya, double yb, int m, int n)
{
	//////////////////////////////////////////////////////////////////////////
	///ʹ�����η�ʵ�ֵĶ��ػ���
	///xά: [xa, xb]
	///yά: [ya, yb]
	///m: xά����
	///n: yά����
	//////////////////////////////////////////////////////////////////////////
	double h = (xb - xa) / m;
	double k = (yb - ya) / n;
	double result = 0;
	result = result + f(xa, ya) + f(xb, ya) + f(xa, yb) + f(xb, yb);
	for (int i = 1; i <= m - 1; i++)
	{
		double xi = xa + i * h;
		result += 2 * (f(xi, ya) + f(xi, yb));
	}
	for (int j = 1; j <= n - 1; j++)
	{
		double yj = ya + j * k;
		result += 2 * (f(xa, yj) + f(xb, yj));
	}
	for (int i = 1; i <= m - 1; i++)
	{
		for (int j = 1; j <= n - 1; j++)
		{
			double xi = xa + i * h;
			double yj = ya + j * k;
			result += 4 * f(xi, yj);
		}
	}
	result *= (0.25 * h * k);
	return result;
}

double Integral::integral_2D_Simpson(double(*f)(double, double), double xa, double xb, double ya, double yb, int m, int n)
{
	//////////////////////////////////////////////////////////////////////////
	///ʹ������ɭ��ʵ�ֵĶ��ػ��֣����ȱ����η����ߣ��ٶȻ����
	///xά: [xa, xb]
	///yά: [ya, yb]
	///m: xά����
	///n: yά����
	///PS��������ʵ�����Ч����û���η���ȷ���ٶ�����...�ʤ���衢�����ĩc(#`����)/
	//////////////////////////////////////////////////////////////////////////
	double h = (xb - xa) / (2.0 * m);
	double k = (yb - ya) / (2.0 * n);
	double result = 0;
	result = result + f(xa, ya) + f(xa, yb) + f(xb, ya) + f(xb, yb);
	for (int i = 1; i <= m; i++)
	{
		double x2i_1 = xa + (2 * i - 1) * h;
		result += 4 * (f(x2i_1, ya) + f(x2i_1, yb));
	}

	for (int i = 1; i <= m - 1; i++)
	{
		double x2i = xa + 2 * i * h;
		result += 2 * (f(x2i, ya) + f(x2i, yb));
	}

	for (int j = 1; j <= n; j++)
	{
		double y2j_1 = ya + (2 * j - 1) * k;
		result += 4 * (f(xa, y2j_1) + f(xb, y2j_1));
	}

	for (int j = 1; j <= n - 1; j++)
	{
		double y2j = ya + 2 * j * k;
		result += 2 * (f(xa, y2j) + f(xb, y2j));
	}

	for (int j = 1; j <= n; j++)
	{
		for (int i = 1; i <= m; i++)
		{
			double x2i_1 = xa + 2 * (i - 1) * h;
			double y2j_1 = ya + 2 * (j - 1) * k;
			result += 16 * f(x2i_1, y2j_1);
		}
	}

	for (int j = 1; j <= n - 1; j++)
	{
		for (int i = 1; i <= m; i++)
		{
			double x2i_1 = xa + 2 * (i - 1) * h;
			double y2j = ya + 2 * j * k;
			result += 8 * f(x2i_1, y2j);
		}
	}

	for (int j = 1; j <= n; j++)
	{
		for (int i = 1; i <= m - 1; i++)
		{
			double x2i = xa + 2 * i * h;
			double y2j_1 = ya + 2 * (j - 1) * k;
			result += 8 * f(x2i, y2j_1);
		}
	}

	for (int j = 1; j <= n - 1; j++)
	{
		for (int i = 1; i <= m - 1; i++)
		{
			double x2i = xa + 2 * i * h;
			double y2j = ya + 2 * j * k;
			result += 4 * f(x2i, y2j);
		}
	}
	result *= (1.0 / 9.0 * h * k);
	return result;
}

double Integral::integral_Trapez(double(*f)(double, double*, int), double* args, int argLen, double a, double b, int m)
{
	//////////////////////////////////////////////////////////////////////////
	///�û���������ɭ����ʵ��, f���ΰ汾
	///������: [a, b]
	///m: ����
	///int(f,x1,x2) �� h / 6 * [f(x1) + 4f((x1+x2)/2) + f(x2)]  (h = x2-x1)
	//////////////////////////////////////////////////////////////////////////
	double h = (b - a) / m;
	double result = 0;
	for (int i = 1; i <= m - 1; i++)
	{
		result += f(a + i * h, args, argLen);
	}
	result *= 2;
	result += (f(a, args, argLen) + f(b, args, argLen));
	result *= h / 2;
	return result;
}

double Integral::integral_Simpson(double(*f)(double, double*, int), double* args, int argLen, double a, double b, int m)
{
	//////////////////////////////////////////////////////////////////////////
	///�û���������ɭ����ʵ��, f���ΰ汾
	///args[]: f��Ҫ����Ĳ���
	///argLen: f��������
	///������: [a, b]
	///m: ����
	///int(f,x1,x2) �� h / 6 * [f(x1) + 4f((x1+x2)/2) + f(x2)]  (h = x2-x1)
	//////////////////////////////////////////////////////////////////////////
	double h = (b - a) / m;
	double result = 0;

	result += f(a, args, argLen) + 4 * f(a + 0.5 * h, args, argLen);

	for (int i = 1; i < m; i++)
	{
		result += 2 * f(a + i * h, args, argLen);
		result += 4 * f(a + (i + 0.5) * h, args, argLen);
	}

	result += f(b, args, argLen);
	result *= (h / 6);
	return result;
}

double Integral::integral_2D_Trapez(double(*f)(double, double, double*, int), double* args, int argLen, double xa, double xb, double ya, double yb, int m, int n)
{
	//////////////////////////////////////////////////////////////////////////
	///ʹ�����η�ʵ�ֵĶ��ػ���, f���ΰ汾
	///args[]: f��Ҫ����Ĳ���
	///argLen: f��������
	///xά: [xa, xb]
	///yά: [ya, yb]
	///m: xά����
	///n: yά����
	//////////////////////////////////////////////////////////////////////////
	double h = (xb - xa) / m;
	double k = (yb - ya) / n;
	double result = 0;
	result = result + f(xa, ya, args, argLen) + f(xb, ya, args, argLen) + f(xa, yb, args, argLen) + f(xb, yb, args, argLen);
	for (int i = 1; i <= m - 1; i++)
	{
		double xi = xa + i * h;
		result += 2 * (f(xi, ya, args, argLen) + f(xi, yb, args, argLen));
	}
	for (int j = 1; j <= n - 1; j++)
	{
		double yj = ya + j * k;
		result += 2 * (f(xa, yj, args, argLen) + f(xb, yj, args, argLen));
	}
	for (int i = 1; i <= m - 1; i++)
	{
		for (int j = 1; j <= n - 1; j++)
		{
			double xi = xa + i * h;
			double yj = ya + j * k;
			result += 4 * f(xi, yj, args, argLen);
		}
	}
	result *= (0.25 * h * k);
	return result;
}
