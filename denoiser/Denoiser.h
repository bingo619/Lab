/* 
 * Last Updated at [2014/9/9 10:11] by wuhao
 */
#pragma once
#include <iostream>
#include "GeoPoint.h"
#include "PointGridIndex.h"
#include <list>
#include <vector>
#include <math.h>
#include <iomanip>
#define PI 3.1415926535898

using namespace std;
extern Area area;
extern int gridWidth;
extern MapDrawer md;

class Denoiser
{
public:
	void run(PointGridIndex* _ptIndex);
	void runEx(PointGridIndex* _ptIndex, double k = 10, double relaxRatio = 2.0);
	void drawPts(MapDrawer& md);
	PointGridIndex* ptIndex;
	void outputJSON();
	void outputStdTrajs();
private:
	void calLMD(GeoPoint* pt, int k, double kNNThresholdM);
	void outlierValidation(GeoPoint* pt, int gridRange, double supportRatio);
	void outlierReValidation(GeoPoint* pt);
	void updatePtIndex(PointGridIndex* ptIndex);	

	//double relaxRatio = 2;

//����Ҫ��ʼ������
	double r = 11;
	double d = 6;
	double L = 50;
	double arcStep = 5;
	double LStep = 3;
	int calDensity(GeoPoint* pt); //����pt��Χ�ĵ��ܶȣ�pt�Լ�Ҳ���ȥ��
	int calDensity(double lat, double lon); //��������(lat, lon)��Χ�ĵ��ܶ�
	pair<int, double> findMaxDensity(GeoPoint* pt);
	void outlierValidationEx(GeoPoint* pt, double kNNThresholdM);
};