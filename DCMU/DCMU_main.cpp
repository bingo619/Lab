#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <time.h>
#include "MapDrawer.h"
#include "Map.h"
#include "MapMatching.h"
#include <list>
#include <vector>
#include <set>
#include <math.h>
#include "GeoPoint.h"
#include "StringOperator.h"
#include <iomanip>
#include "PolylineGenerator.h"
#include "PointGridIndex.h"
#include "StringOperator.h"
#include "ExpGenerator.h"
#include "TrajDrawer.h"
#include "DCMU.h"

#define eps 1e-8
#define INFINITE 999999999
#define PI 3.1415926535898
using namespace std;
using namespace Gdiplus;

Area area(1.294788, 1.327723, 103.784667, 103.825200); //small
//Area area(1.294788, 1.393593, 103.784667, 103.906266); //area1
//Area area(1.343593, 1.442398, 103.784667, 103.906266); //area2
//Area area(1.294788, 1.393593, 103.704667, 103.826266); //area3

Map roadNetwork;
Map originalRoadNetwork; //未用
MapDrawer md;
PointGridIndex allPtIndex;
int gridWidth = 900;
string workspaceFolder = "D:\\trajectory\\singapore_data\\experiments\\big area\\geo\\area1\\";

void initialization()
{
	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 150);
	roadNetwork.deleteEdges(workspaceFolder + "deletedEdges.txt");
	
	TrajReader tr(workspaceFolder + "120_newMMTrajs_unmatched.txt");
	list<Traj*> trajs;
	tr.readTrajs(trajs);//, 50000);
	
	list<GeoPoint*> allPts;
	for each(Traj* traj in trajs)
	{
		for each (GeoPoint* pt in *traj)
		{
			allPts.push_back(pt);
		}
	}
	allPtIndex.createIndex(allPts, &area, gridWidth);

	md.setArea(&area);
	md.setResolution(10000);
}

void initializationForDenoiser()
{
	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	roadNetwork.deleteEdges(workspaceFolder + "deletedEdges.txt");
	workspaceFolder = "D:\\trajectory\\singapore_data\\experiments\\unmatched pts\\";
	string ptsFilePath = workspaceFolder +  "cluster 10.txt";;
	ifstream ifs(ptsFilePath);
	list<GeoPoint*> allPts;
	if (!ifs)
	{
		cout << "打开文件 " << ptsFilePath << " 错误" << endl;
		system("pause");
	}
	while (ifs)
	{
		int time, clusterId;
		double lat, lon, dir;
		ifs >> time >> lat >> lon >> dir >> clusterId;
		if (ifs.fail())
			break;
		GeoPoint* pt = new GeoPoint(lat, lon, time);
		pt->direction = dir;
		pt->clusterId = clusterId;
		allPts.push_back(pt);
	}
	ifs.close();
	cout << "共读入" << allPts.size() << "个点" << endl;

	allPtIndex.createIndex(allPts, &area, 300);

	md.setArea(&area);
	md.setResolution(5000);
}

void genMMData()
{
	roadNetwork.setArea(&area);
	roadNetwork.open("D:\\trajectory\\singapore_data\\singapore_map\\new\\", 50);
	string trajFilePath = "D:\\trajectory\\singapore_data\\201202\\every day\\wy_MMTrajs1.txt";
	list<Traj*> trajs;
	TrajReader tReader(trajFilePath);
	tReader.readTrajs(trajs);
	
	int count = 0;
	ofstream fout;
	cout << ">> starting MapMatching" << endl
		<< trajs.size() << " trajs in total" << endl;
	MapMatcher mm(&roadNetwork);
	//对每一条轨迹
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++, count++)
	{
		list<Edge*> result;
		mm.MapMatching(*(*trajIter), result, 50); //MapMatching

		if (count % 1000 == 0)
			cout << ">> MM" << count << " finished" << endl;
		Traj* traj = (*trajIter);
		if (traj == NULL)
			continue;
		//对每一个点
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				(*ptIter)->mmRoadId = (*edgeIter)->id;
			}
			else //匹配失败
			{
				(*ptIter)->mmRoadId = -1;
			}
			ptIter++;
			edgeIter++;
		}
	}
	TrajReader::outputTrajs(trajs, "mmedTrajs.txt");
	md.setArea(&area);
	md.setResolution(5000);
	md.newBitmap();
	md.lockBits();
	TrajDrawer::drawMMTrajs(trajs, md, Color::Black, false, false, false, true);
	md.unlockBits();
	md.saveBitmap("mm.png");
	system("pause");
	exit(0);
}

void subSample()
{
	ExpGenerator eg;
	//eg.genSubSampledData(60, workspaceFolder, "newMMTrajs.txt");
	eg.genSubSampledData(120, workspaceFolder, "newMMTrajs_unmatched.txt");
}

void dataAnalyzer(string dataPath)
{
	//////////////////////////////////////////////////////////////////////////
	///分析轨迹数据的平均采样率
	//////////////////////////////////////////////////////////////////////////
	ifstream ifs(dataPath);
	int totalTime = 0;
	int intervalCount = 0;
	int preTime = -1;
	int time = 0;
	double dummy_d;
	int dummy_i;
	while (ifs)
	{
		ifs >> time;
	//	printf("preTime = %d, time = %d\n", preTime, time);
		//system("pause");
		if (ifs.fail())
			break;
		if (time == -1)
		{
			preTime = -1;
			continue;
		}
		else
		{
			ifs >> dummy_d >> dummy_d >> dummy_i;
			if (preTime == -1) //轨迹第一个点
			{
				preTime = time;
			}
			else
			{
				totalTime += (time - preTime);
				preTime = time;
				intervalCount++;
			}
		}
	}
	cout << "平均采样率为： " << (double)totalTime / (double)intervalCount << "s" << endl;
}

void main()
{
	//genMMData();
	//subSample();
	//dataAnalyzer("120_newMMTrajs_unmatched.txt");
	//system("pause");
	//exit(0);
	//initialization();
	initializationForDenoiser();
	DCMU dcmu;

	md.newBitmap();
	md.lockBits();
	dcmu.run2();
	allPtIndex.drawGridLine(Color::Green, md);
	md.unlockBits();
	md.saveBitmap("cluster11.png");
}