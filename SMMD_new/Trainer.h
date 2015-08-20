#pragma once
#include "GeoPoint.h"
#include "Map.h"
#include "TrajReader.h"
#include "PolylineGenerator.h"
#include "MapDrawer.h"
#include "SMMD.h"
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

class Trainer
{
public:
	double u0 = 0, lambda0 = 0, alpha0 = 0, beta0 = 0;  //先验参数

	Trainer(){};
	Trainer(Map* roadNetwork_, Area* area_);
	void loadPrior(string priorDataPath); //读p(r)
	void loadTrainData(string trainDataPath, int count = INF);
	void loadTrainData(list<GeoPoint*> trainDataSet);

	void trainSimple(double intervalM = 50.0);
	void trainSMMD();
	void trainSMMDEx(double intervalM = 50.0);

	void trainOneRoad_GaussianParam(Edge* r, list<GeoPoint*>& pts);
	void genCenterline(string outPath);
	void genFakeCenterline(string outPath);
	void drawCenterline(string centerlinePath, MapDrawer& md);

	//result
	void loadTrainResult(string filePath);
	void writeTrainResult(string filePath);

//private:
	Map* roadNetwork;
	Area* area;
	list<GeoPoint*> trainDataSet;
};

class KNNClassifier
{
public:
	KNNClassifier(){};
	KNNClassifier(Map* roadNetwork_, Area* area_);
	void loadTrainData(string trainDataPath, int count = INF);

	int classify(GeoPoint* pt, int k); //返回pt匹配的路段号

private:
	Map* roadNetwork;
	Area* area;
	list<GeoPoint*> trainDataSet;
	PointGridIndex* ptGridIndex;
};
