/* 
 * Last Updated at [2014/12/29 15:40] by wuhao
 */
#pragma once
#include <iostream>
#include <iomanip>
#include "GeoPoint.h"
#include "PointGridIndex.h"
#include <list>
#include <vector>
#define PI 3.1415926535898

using namespace std;
extern Area area;
extern int gridWidth;
extern MapDrawer md;
extern int roadId;

struct Cluster
{
	list<GeoPoint*> pts;
	double direction;
	int clusterId;
	vector<GeoPoint*> polyline;
	double polylineLengthM;
	double avgDirection;

	Area area; //��¼cluster��MBR	

	Cluster()
	{
		direction = 0;
		area.setArea(999, -999, 999, -999);
	}

	void add(GeoPoint* pt)
	{
		//////////////////////////////////////////////////////////////////////////
		///��pt����cluster������pt��clusterId�����Ϊ��cluster��Idͬʱ���¼���ƽ��direction
		///Ϊ�˼ӿ�getPtsNearCluster�����������ʱ��ͬʱ���µ�ǰcluster��mbr
		///[ATTENTION]:��������ж�pt��ԭ��clusterId
		//////////////////////////////////////////////////////////////////////////
		pt->clusterId = clusterId;
		direction = (pts.size() * direction + pt->direction) / (pts.size() + 1);
		pts.push_back(pt);
		//update mbr
		if (pt->lat > area.maxLat) area.maxLat = pt->lat;
		if (pt->lat < area.minLat) area.minLat = pt->lat;
		if (pt->lon > area.maxLon) area.maxLon = pt->lon;
		if (pt->lon < area.minLon) area.minLon = pt->lon;
	}

};

class PtCluster
{
public:
	void run(PointGridIndex* _ptIndex);
	void drawClusters(MapDrawer& md);
	void outputPtsDir(string outFilePath);
	vector<Cluster*> clusters;

//private:
	double d = 10.0; //���ο��
	double l = 36.0; //���γ���
	double distThresM = 18; //cluster dist thres
	double angleThres = 20.0 / 180.0 * PI; //cluster angle thres

	PointGridIndex* ptIndex;
	list<GeoPoint*> pts; //���ptIndex�е����е㣬�ڵ���calPtsDirs�󼴿�����
	//����㷽��
	void calPtsDirs();
	void count(GeoPoint* p0, GeoPoint* p, vector<int>& countVec, double d, double l, double angleStep);
	void calDir(GeoPoint* p0, double angleStep, double d, double l);
	void drawPtsDir(MapDrawer& md);

	//�������
	void doDirCluster();
	void getPtsNearCluster(Cluster* cluster, double distThresM, double angleThres, vector<GeoPoint*>& nearPts);
	bool fetch(Cluster* cluster, double distThresM, double angleThres);
	Cluster* genOneCluster(int clusterId, GeoPoint* seedPt, double distThresM, double angleThres);

	Gdiplus::Color randomColor()
	{
		int r = int(((double)rand()) / RAND_MAX * 255);
		int g = int(((double)rand()) / RAND_MAX * 255);
		int b = int(((double)rand()) / RAND_MAX * 255);
		Gdiplus::Color color((byte)r, (byte)g, (byte)b);
		return color;
	}
};
