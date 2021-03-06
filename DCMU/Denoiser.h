/* 
 * Last Updated at [2015/2/6 10:59] by wuhao
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
	void run(PointGridIndex* _ptIndex); //老版本，不用
	void runEx(PointGridIndex* _ptIndex); //新版本，用这个
	void drawPts(MapDrawer& md); //话降噪后的结果
	PointGridIndex* ptIndex;
	void outputJSON(); //将降噪后的结果按JSON格式输出
private:
	void calLMD(GeoPoint* pt, int k, double kNNThresholdM); //计算pt的第k个NN的距离
	void outlierValidation(GeoPoint* pt, int gridRange, double supportRatio); //老版本，废弃
	void outlierReValidation(GeoPoint* pt); //老版本，废弃
	void updatePtIndex(PointGridIndex* ptIndex);	

	double relaxRatio = 1.7; //kNN降噪判定松弛参数，参数值越高，判定越松，降噪效果越不明显（实际判定kNN距离阈值为kNNThresholdM * relaxRatio, relaxRatio为1的时候则是严格按理论公式计算）

//下面要开始厉害了
//这部分代码暂时不用了
	double r = 11;
	double d = 6;
	double L = 50;
	double arcStep = 5;
	double LStep = 3;
	int calDensity(GeoPoint* pt); //计算pt周围的点密度（pt自己也算进去）
	int calDensity(double lat, double lon); //计算坐标(lat, lon)周围的点密度
	pair<int, double> findMaxDensity(GeoPoint* pt);
	void outlierValidationEx(GeoPoint* pt, double kNNThresholdM);
};