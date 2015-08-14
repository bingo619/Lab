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
	void setPriorParams(double u0_, double lambda0_, double alpha0_, double beta0_);
	void loadTrainData(string trainDataPath, int count = INF);
	
	void trainSimple(double intervalM = 50.0);
	void trainSMMD();
	void trainSMMDEx(double intervalM = 50.0);

	void trainOneRoad(Edge* r, list<GeoPoint*>& pts);
	void trainOneRoadEx(Edge* r, list<GeoPoint*>& pts);
	void trainOneRoadSimple(Edge* r, list<GeoPoint*>& pts);
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
