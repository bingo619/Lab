/* 
 * Last Updated at [2015/2/6 12:22] by wuhao
 * version 1.0.1.1
 * comments: 将迭代参数提出来作为公有成员，同时针对map update问题调整了一套比较适合的参数
 */
#pragma once
#include <iostream>
#include <vector>
#include <list>
#include "Matrix.h"
using namespace std;
#define eps 1e-7

struct Pt
{
	double x;
	double y;
	double dist;
	Pt(double x, double y)
	{
		this->x = x;
		this->y = y;
	}
	Pt(){}
};


class PolylineGenerator
{
public:
	vector<Pt> polyline;
	PolylineGenerator();
	void genPolyline(list<Pt>& pts);
	int bigIterTimes = 10; //调整一整轮点的循环次数,越大越慢但越准确
	int smallIterTimes = 100; //调整一个点的迭代次数，越大越慢但越准确
	double step = 0.2; //每一次迭代的步长，越小越慢但越准确
	int sampleSize = 500; //sampling的数量，越大越慢但越准确

//private:
	vector<list<Pt>*> vSet;
	vector<list<Pt>*> sSet;
	list<Pt> pts;
	double r;
	int n;
	double lambdaP;
	double anglePenalty = 0.008; //控制polyline生成的夹角惩罚，越高越不容易形成锐角，跑wangyin实验用0.02比较好

	//中心差分求导用
	int max_it = 100000;
	double stopEps = 1e-5;

	void sampling(int sampleNum);
	void doProject(int index);
	void doProject();
	double getRadius();
	void initialization();
	void reProject(Pt& pt);
	void optimization();
	void optimizationEx();

	double calcStep(int x, int y, int vi); //计算optimization中沿负梯度方向移动点的步长
	double d2f_dx2(int x, int y, int i);
	double d2f_dy2(int x, int y, int i);
	double d2f_dxdy(int x, int y, int i);
	pair<double, double> central_difference(double(PolylineGenerator::*f)(double, double, int), int vi);
	double calculate(double x, double y, int i);
};
