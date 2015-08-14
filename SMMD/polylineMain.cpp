#include "GeoPoint.h"
#include "PolylineGenerator.h"
#include "MapDrawer.h"
#include "StringOperator.h"
#include <fstream>
using namespace std;
using namespace Gdiplus;

Area area;
MapDrawer md;
int png_width = 1000;
vector<list<GeoPoint*> > roadsPts;

void initialization(int count)
{
	string path = "pts.txt";
	ifstream ifs(path);
	int current = -1;
	int premmRoadId = -1;
	//int count = 1;

	while (ifs)
	{
		double lat, lon;
		int mmRoadId;
		ifs >> lat >> lon >> mmRoadId;
		if (mmRoadId != premmRoadId)
		{
			if (current + 3 == count)
			{
				break;
			}
			list<GeoPoint*> newRoadPts;
			roadsPts.push_back(newRoadPts);
			current++;
		}
		roadsPts[current].push_back(new GeoPoint(lat, lon));
		premmRoadId = mmRoadId;
	}

	//����area
	double minLat = 999;
	double maxLat = -1;
	double minLon = 999;
	double maxLon = -1;
	for (int i = 0; i < roadsPts.size(); i++)
	{
		for each (GeoPoint* pt in roadsPts[i])
		{
			if (pt->lat < minLat) minLat = pt->lat;
			if (pt->lat > maxLat) maxLat = pt->lat;
			if (pt->lon < minLon) minLon = pt->lon;
			if (pt->lon > maxLon) maxLon = pt->lon;
		}
	}
	double delta = 0.001; //area�߽��ɳ�
	area.setArea(minLat - delta, maxLat + delta, minLon - delta, maxLon + delta);
	
	md.setArea(&area);
	md.setResolution(png_width);
}

void genPolyline(list<GeoPoint*>& geoPts)
{
	PolylineGenerator pg;
	//��cluster��Ĺ켣��ȫ������pts��
	list<Pt> pts;

	//��������ת���������ռ�����ӳ�䵽5000���ط�Χ�ϱȽϺ���
	MapDrawer mdTempForCoordThrans;
	mdTempForCoordThrans.setArea(&area);
	mdTempForCoordThrans.setResolution(5000);

	for each (GeoPoint* gPt in geoPts)
	{
		Pt tmpPt;
		Gdiplus::Point gdiPt = mdTempForCoordThrans.geoToScreen(gPt->lat, gPt->lon);
		tmpPt.x = (double)gdiPt.X;
		tmpPt.y = (double)gdiPt.Y;
		pts.push_back(tmpPt);
	}
	pg.genPolyline(pts);

	//��polylineת�ؿռ�����
	vector<GeoPoint*> centerLine;
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		GeoPoint* pt = new GeoPoint(mdTempForCoordThrans.screenToGeo((int)pg.polyline[i].x, (int)pg.polyline[i].y));
		centerLine.push_back(pt);
	}

	//plot pts
	for each (GeoPoint* pt in geoPts)
	{
		md.drawPoint(Color::Green, pt->lat, pt->lon);
	}

	//draw polyline
	for (int i = 0; i < centerLine.size() - 1; i++)
	{
		md.drawLine(Color::Red, centerLine[i]->lat, centerLine[i]->lon, 	centerLine[i + 1]->lat, centerLine[i + 1]->lon);
		md.drawBigPoint(Color::Black, centerLine[i]->lat, centerLine[i]->lon);
		md.drawBigPoint(Color::Black, centerLine[i + 1]->lat, centerLine[i + 1]->lon);
	}
}


void main()
{
	initialization(999);
	
	for (int i = 0; i < 15; i++)
	{
		md.newBitmap();
		md.lockBits();
		genPolyline(roadsPts[i]);
		md.unlockBits();
		md.saveBitmap(StringOperator::intToString(i) + ".png");
	}
}