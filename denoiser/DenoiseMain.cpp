#include <iostream>
#include "GeoPoint.h"
#include "MapDrawer.h"
#include "Denoiser.h"
#include "PointGridIndex.h"
#include "StringOperator.h"
#include "TrajReader.h"
using namespace std;

string trajFilePath;// = "D:\\project\\LAB\\trajectory\\denoiser\\TinyRoadTrajectory3.txt";
int k = 10;
double relaxRatio = 2.0;
int png_wide = 2000; //绘图像素宽度

list<Traj*> trajs;
list<GeoPoint*> allPts;
PointGridIndex ptIndex;
Area area;
MapDrawer md;

void initialization(string trajFilePath)
{
	//读数据&计算area
	double minLat = 999;
	double maxLat = -1;
	double minLon = 999;
	double maxLon = -1;
	TrajReader tr(trajFilePath);
	tr.readTrajs(trajs);// , 50000);

	for each (Traj* traj in trajs)
	{
		for each (GeoPoint* pt in *traj)
		{
			allPts.push_back(pt);
			if (pt->lat < minLat) minLat = pt->lat;
			if (pt->lat > maxLat) maxLat = pt->lat;
			if (pt->lon < minLon) minLon = pt->lon;
			if (pt->lon > maxLon) maxLon = pt->lon;
		}
	}
	double delta = 0.001; //area边界松弛
	area.setArea(minLat - delta, maxLat + delta, minLon - delta, maxLon + delta);
	double gridSizeM = 10.0; //网格宽度10m
	int gridWidth = (area.maxLon - area.minLon) * GeoPoint::geoScale / gridSizeM;
	ptIndex.createIndex(allPts, &area, gridWidth);
	
	md.setArea(&area);
	md.setResolution(png_wide);
}



void denoise(int k = 10, double relaxRatio = 2.0)
{
	Denoiser denoiser;
	denoiser.runEx(&ptIndex, k, relaxRatio);
	denoiser.drawPts(md);
	//denoiser.outputJSON();
	denoiser.outputStdTrajs();
}


void subMain()
{
	initialization(trajFilePath);
	md.newBitmap();
	md.lockBits();

	denoise(k, relaxRatio);
	//ptIndex.drawGridLine(Gdiplus::Color::Green, md);
	md.unlockBits();
	md.saveBitmap("result.png");
}

int main(int argc, char* argv[])
{
	if (argc != 5 && argc != 2)
	{
		//MessageBox(NULL, TEXT("Error"), TEXT("Parameter Exception !"), MB_OK);
		cout << "参数个数不对！" << endl;
		system("pause");
		exit(0);
	}
	trajFilePath = argv[1];
	if (argc == 2)
	{
		subMain();
	}
	else if (argc == 5)
	{
		k = StringOperator::stringToInt(argv[2]);
		relaxRatio = StringOperator::stringToDouble(argv[3]);
		png_wide = StringOperator::stringToInt(argv[4]);
		if (png_wide >= 15000)
		{
			cout << "输出图像像素宽度太大！不得超过15000pxl" << endl;
			system("pause");
			exit(0);
		}
		subMain();
	}
	return 1;
}