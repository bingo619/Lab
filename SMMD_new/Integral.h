/* 
 * Last Updated at [2015/2/12 15:48] by wuhao
 * version 1.0
 */
#pragma once
#include <iostream>
using namespace std;

class Integral
{
public:
	static double integral_2D_Trapez(double(*f)(double, double), double xa, double xb, double ya, double yb, int m, int n); //���ػ���ʹ�����η�
	static double integral_2D_Simpson(double(*f)(double, double), double xa, double xb, double ya, double yb, int m, int n); //���ػ���ʹ������ɭ��[�ƺ�Ч������,���Ƽ�]
	static double integral_Simpson(double(*f)(double), double a, double b, int m); //һ�ػ��֣�ʹ������ɭ��
	static double integral_Trapez(double(*f)(double),  double a, double b, int m); //һ�ػ��֣�ʹ�����η�

	//f with args
	//��ʽ: double f(double x, double y, double* args, int argLen)
	static double integral_2D_Trapez(double(*f)(double, double, double*, int), double* args, int argLen, double xa, double xb, double ya, double yb, int m, int n); //���ػ���ʹ�����η�
	//static double integral_2D_Simpson(double(*f)(double, double, double*, int), double* args, int argLen, double xa, double xb, double ya, double yb, int m, int n); //���ػ���ʹ������ɭ��[�ƺ�Ч������,���Ƽ�]
	static double integral_Simpson(double(*f)(double, double*, int), double* args, int argLen, double a, double b, int m); //һ�ػ��֣�ʹ������ɭ��
	static double integral_Trapez(double(*f)(double, double*, int), double* args, int argLen, double a, double b, int m); //һ�ػ��֣�ʹ�����η�
};