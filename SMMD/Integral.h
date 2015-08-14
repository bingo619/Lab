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
	static double integral_2D_Trapez(double(*f)(double, double), double xa, double xb, double ya, double yb, int m, int n); //二重积分使用梯形法
	static double integral_2D_Simpson(double(*f)(double, double), double xa, double xb, double ya, double yb, int m, int n); //二重积分使用辛普森法[似乎效果不好,不推荐]
	static double integral_Simpson(double(*f)(double), double a, double b, int m); //一重积分，使用辛普森法
	static double integral_Trapez(double(*f)(double),  double a, double b, int m); //一重积分，使用梯形法

	//f with args
	//格式: double f(double x, double y, double* args, int argLen)
	static double integral_2D_Trapez(double(*f)(double, double, double*, int), double* args, int argLen, double xa, double xb, double ya, double yb, int m, int n); //二重积分使用梯形法
	//static double integral_2D_Simpson(double(*f)(double, double, double*, int), double* args, int argLen, double xa, double xb, double ya, double yb, int m, int n); //二重积分使用辛普森法[似乎效果不好,不推荐]
	static double integral_Simpson(double(*f)(double, double*, int), double* args, int argLen, double a, double b, int m); //一重积分，使用辛普森法
	static double integral_Trapez(double(*f)(double, double*, int), double* args, int argLen, double a, double b, int m); //一重积分，使用梯形法
};