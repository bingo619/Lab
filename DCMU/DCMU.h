/* 
 * Last Updated at [2014/9/24 11:08] by wuhao
 */
#pragma once
#include <iostream>
#include "GeoPoint.h"
#include "PointGridIndex.h"
#include "Map.h"
#include "Denoiser.h"
#include "PtCluster.h"
#include "RoadGenerator.h"
#include "PointMover.h"
using namespace std;

extern Map roadNetwork;
extern PointGridIndex allPtIndex;
extern Area area;
extern MapDrawer md;
extern int gridWidth;

class DCMU
{
public:
	vector<PointGridIndex*> ptIndexes;
	void run();
	void run1();
	void run2();

private:
	//gird partition
	void gridClustering();
	void dfs(int row, int col, bool** dfsState, vector<pair<int, int>>& connectingCompnt);
	
	//denoise
	Denoiser denoiser;
	PointMover ptMover;

	//direction clustering
	PtCluster ptCluster;

	//road generation
};
