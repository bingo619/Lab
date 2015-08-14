/* 
 * Last Updated at [2015/2/6 11:12] by wuhao
 * version 1.0.1
 * comments: �����������������Ϊ���г�Ա��ͬʱ��Ե���ƥ�����������һ�ױȽϿ�Ĳ���
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
	int bigIterTimes = 3; //����һ���ֵ��ѭ������
	int smallIterTimes = 50; //����һ����ĵ�������
	double step = 0.5; //ÿһ�ε����Ĳ���
	int sampleSize = 500; //sampling������

//private:
	vector<list<Pt>*> vSet;
	vector<list<Pt>*> sSet;
	list<Pt> pts;
	double r;
	int n;
	double lambdaP;
	double anglePenalty = 0.008; //��wangyinʵ����0.02�ȽϺ�


	//���Ĳ������
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

	double calcStep(int x, int y, int vi); //����optimization���ظ��ݶȷ����ƶ���Ĳ���
	double d2f_dx2(int x, int y, int i);
	double d2f_dy2(int x, int y, int i);
	double d2f_dxdy(int x, int y, int i);
	pair<double, double> central_difference(double(PolylineGenerator::*f)(double, double, int), int vi);
	double calculate(double x, double y, int i);
};
