#include <iostream>
#include <string>
#include "GeoPoint.h"
#include "GeoVerification.h"
#include "MapDrawer.h"
#include "Map.h"
#include "TrajReader.h"
#include "TrajDrawer.h"
#include "StringOperator.h"
using namespace std;

Map roadNetwork;
//Area area(1.294788, 1.393593, 103.784667, 103.906266); //big1
//Area area(1.343593, 1.442398, 103.784667, 103.906266); //big2
Area area(1.294788, 1.393593, 103.704667, 103.826266); //big3
MapDrawer md;
vector<Figure*> figures;
string workspaceFolder = "D:\\trajectory\\singapore_data\\experiments\\big area\\geo\\area3\\";

enum ALGORITHM_ID
{
	ALG_DCMU = 1,
	ALG_WY = 2,
	ALG_MI = 3
};

void initialization()
{
	//////////////////////////////////////////////////////////////////////////
	///初始化地图，删路
	///读入生成的道路
	//////////////////////////////////////////////////////////////////////////

	//初始化地图


	//读路
	//TrajReader tReader("roads_MI.txt");

	//初始化画板
	md.setArea(&area);
	md.setResolution(3000);
	md.newBitmap();
}

void test(int sampleRate, ALGORITHM_ID alg_id, int days)
{
	string fileName;
	cout << endl << endl;

	switch (alg_id)
	{
	case ALG_DCMU:
		fileName = "roads_DCMU_ref_" + StringOperator::intToString(sampleRate) + "_" + StringOperator::intToString(days) + "d.txt";
		cout << "DCMU " << sampleRate << "s:" << endl;
		break;
	case ALG_WY:
		fileName = "roads_wy_" + StringOperator::intToString(sampleRate) + "_" + StringOperator::intToString(days) + "d.txt";
		cout << "wangyin " << sampleRate << "s:" << endl;
		break;
	case ALG_MI:
		fileName = "roads_MI_" + StringOperator::intToString(sampleRate) + "_" + StringOperator::intToString(days) + "d.txt";
		cout << "MI " << sampleRate << "s:" << endl;
		break;
	default:
		break;
	}

	//删路
	Map rn;
	rn.setArea(&area);
	rn.openOld("D:\\trajectory\\singapore_data\\singapore_map\\", 50);
	rn.deleteEdges(workspaceFolder + "newDeletedEdges_" + StringOperator::intToString(sampleRate) + "_" + StringOperator::intToString(days) + "d.txt");
	
	TrajReader tReader(workspaceFolder + fileName);
	figures.clear();
	tReader.readTrajs(figures);
	roadNetwork = rn;
	GeoVerification gv;
	if (alg_id == ALG_MI)
		gv.verificate_MI(figures, md);
	else
		gv.verificate(figures, md);
}



void test()
{
	//////////////////////////////////////////////////////////////////////////
	///实验一：对三个算法在30-120s采样率下的实验
	//////////////////////////////////////////////////////////////////////////
	initialization();
	
	md.lockBits();
	//TrajDrawer::drawTrajs(figures, md, Gdiplus::Color::Blue);

	for (int i = 1; i <= 3; i++)
	{
		for (int sr = 30; sr <= 120; sr += 30)
		{
			test(sr, (ALGORITHM_ID)i, 15);
		}
	}


	md.unlockBits();
	md.saveBitmap("ALL.png");
}

void expForDiffDays(int days)
{
	//////////////////////////////////////////////////////////////////////////
	///实验二：对三个算法在30s采样率下使用days天数据的实验
	//////////////////////////////////////////////////////////////////////////
	initialization();

	md.lockBits();
	//for (int i = 1; i <= 3; i++)
	/*{
		test(30, (ALGORITHM_ID)1, days);
	}*/
	test(30, ALG_DCMU, days);
	test(30, ALG_MI, days);
	test(30, ALG_WY, days);
	md.unlockBits();
	md.saveBitmap("ALL.png");
}

void main()
{
	//test();
	expForDiffDays(1);
	expForDiffDays(5);
	expForDiffDays(10);
	system("pause");
}