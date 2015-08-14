/* 
 * Last Updated at [2014/9/4 20:23] by wuhao
 */
#pragma once
#include <iostream>
#define INVALID_TIME -999

class GeoPoint
{
public:
	double lat;
	double lon;
	int time;
	int mmRoadId;
	double dist; //��չ�ֶ�
	double lmd; //��չ�ֶ�
	int isOutlier; //��չ�ֶ�
	double direction; //��չ�ֶ�
	int clusterId = -1; //��չ�ֶ�
	int mmRoadId2; //��չ�ֶ�

	GeoPoint();
	GeoPoint(double lat, double lon, int time, int mmRoadId);
	GeoPoint(double lat, double lon, int time);
	GeoPoint(double lat, double lon);
	GeoPoint(std::pair<double, double>& lat_lon_pair);

	static double geoScale;
	static double distM(double lat1, double lon1, double lat2, double lon2);
	static double distM(GeoPoint& pt1, GeoPoint& pt2);
	static double distM(GeoPoint* pt1, GeoPoint* pt2);
	double distM(double lat1, double lat2);
	double distM(GeoPoint* pt);
	double distM(GeoPoint& pt);
	static double distDeg(double lat1, double lon1, double lat2, double lon2);
	static double distDeg(GeoPoint pt1, GeoPoint pt2);
	static double distDeg(GeoPoint* pt1, GeoPoint* pt2);
	double distDeg(double lat1, double lat2);
	double distDeg(GeoPoint* pt);
	double distDeg(GeoPoint& pt);

	void print();
};

//��ʾ������࣬MapDrawer��Map����ͬһ��Area�����Ա��������һ����
class Area
{
public:
	double minLat;
	double maxLat;
	double minLon;
	double maxLon;

	void setArea(double _minLat, double _maxLat, double _minLon, double _maxLon)
	{
		this->minLat = _minLat;
		this->maxLat = _maxLat;
		this->minLon = _minLon;
		this->maxLon = _maxLon;
	}

	Area()
	{
		minLat = 0;
		maxLat = 0;
		minLon = 0;
		maxLon = 0;
	}

	Area(double _minLat, double _maxLat, double _minLon, double _maxLon)
	{
		setArea(_minLat, _maxLat, _minLon, _maxLon);
	}

	bool inArea(double lat, double lon)
	{
		//����(lat,lon)�Ƿ��ڸ�������
		return (lat > minLat && lat < maxLat && lon > minLon && lon < maxLon);
	}

	void print()
	{
		printf("area: minLat = %lf, maxLat = %lf, minLon = %lf, maxLat = %lf\n", minLat, maxLat, minLon, maxLon);
	}
};