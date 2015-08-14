/*
* Last Updated at [2015/2/13 13:12] by wuhao
* version 2.0.1
* comments: 增加读入读出GeoPoint功能 @2015/2/13
*/
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include "GeoPoint.h"
#include <iomanip>
using namespace std;
#define INF 999999999
#define INVALID_ROADID -1
typedef list<GeoPoint*> Traj;

class TrajReader
{
public:
	TrajReader();
	TrajReader(string filePath); //构造函数时就打开轨迹文件 
	void open(string filePath); //打开轨迹文件，轨迹文件路径为filePath
	void readTrajs(vector<Traj*>& dest, int count = INF); //读入count条轨迹放入dest(dest会先清空)，默认为全部读入，并关闭文件
	void readTrajs(list<Traj*>& dest, int count = INF); //读入count条轨迹放入dest(dest会先清空)，默认为全部读入。并关闭文件
	void readGeoPoints(list<GeoPoint*>& dest, Area* area = NULL, int count = INF); //读入在area内的count个GeoPoint(dest会先清空)，默认为area = NULL表示不限制区域，全部读入, 并关闭文件

	static void outputTrajs(list<Traj*>& trajs, string filePath, int count = INF); //将trajs内count条轨迹按照标准格式输出至filePath
	static void outputPts(list<GeoPoint*>& pts, string filePath, int count = INF); //将pts内count个点按照标准格式输出至filePath

private:
	ifstream trajIfs;
};