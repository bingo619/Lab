#include <iostream>
#include <string>
#include "GeoPoint.h"
#include "GeoVerification.h"
#include "MapDrawer.h"
#include "Map.h"
#include "TrajReader.h"
#include "TrajDrawer.h"
using namespace std;

Map roadNetwork;
Area area(1.294788, 1.393593, 103.784667, 103.906266); //big1
//Area area(1.343593, 1.442398, 103.784667, 103.906266); //big2
//Area area(1.294788, 1.393593, 103.704667, 103.826266); //big3
MapDrawer md;
vector<Figure*> figures;
string workspaceFolder = "D:\\trajectory\\singapore_data\\experiments\\big area\\geo\\area1\\";

void initialization()
{
	//////////////////////////////////////////////////////////////////////////
	///初始化地图，删路
	///读入生成的道路
	//////////////////////////////////////////////////////////////////////////

	//初始化地图
	roadNetwork.setArea(&area);
	roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	roadNetwork.deleteEdges(workspaceFolder + "newDeletedEdges_120.txt");

	//读路
	//TrajReader tReader("roads_MI.txt");
	//TrajReader tReader(workspaceFolder + "roads_wy_120.txt");
	TrajReader tReader(workspaceFolder + "roads_DCMU_ref_120.txt");
	//TrajReader tReader(workspaceFolder + "roads_MI_120.txt");

	tReader.readTrajs(figures);

	//初始化画板
	md.setArea(&area);
	md.setResolution(5000);
	md.newBitmap();
}

void test()
{
	initialization();
	GeoVerification gv;
	md.lockBits();
	TrajDrawer::drawTrajs(figures, md, Gdiplus::Color::Blue);
	gv.verificate(figures, md);	
	md.unlockBits();
	md.saveBitmap("DCMU.png");
}


void main()
{
	test();
	system("pause");
}