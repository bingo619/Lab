/* 
 * Last Updated at [2014/12/29 15:04] by wuhao
 * version 2.0.1
 */
/*
*	//ʹ�÷���
*	MapDrawer md;
*   Area area; //����һ��area
*	md.setArea(area); //�������󶨵�area
*	md.setResolution(size); //ͼƬ���,��λΪ����,�߶�����Ӧ
*	md.newBitmap(); //������ͼƬ,Ĭ�ϱ���Ϊ͸��
*	md.lockBits(); //�κλ�ͼ����ǰ����������
*	drawsth(); //���л�ͼ����Ӧ�ü������
* 	md.unlockBits(); //���л�ͼ������ɺ���ϴ˾����ύ����
*   md.saveBitmap("out.png"); //���ΪpngͼƬ
*
*  //zoomingʹ�÷��� [�ѷ���]
*  size = 15000;
*  md.setArea(minLat, maxLat, minLon, maxLon)
*  md.setResolution(size);
*  size = 3000;
*  md.zoomIn(8900, 7150, 400, 300, size); //��15000��ȵ�ͼƬ�о۽���(8900,7150)Ϊ���Ͻǣ�400*300��С�����У�������ͼ����Ϊ3000
*  md.newBitmap();
*  ......
*/
#pragma once
#include <string>
#include <vector>
#include <Windows.h>  
#include <tchar.h>  
#include <stdio.h>  
#include <gdiplus.h>
#include <io.h>
#include <fstream>
#include <iostream>
#include "GeoPoint.h"
//using namespace Gdiplus;
#pragma comment(lib,"gdiplus.lib")

static bool minus[5][4] = {
	0, 0, 0, 0,
	0, 0, 0, 0,
	1, 1, 1, 0, 
	0, 0, 0, 0, 
	0, 0, 0, 0
};

static bool num[10][5][4] = {
	1, 1, 1, 0,
	1, 0, 1, 0,
	1, 0, 1, 0,
	1, 0, 1, 0,
	1, 1, 1, 0,

	0, 0, 1, 0,
	0, 0, 1, 0,
	0, 0, 1, 0,
	0, 0, 1, 0,
	0, 0, 1, 0,

	1, 1, 1, 0,
	0, 0, 1, 0,
	1, 1, 1, 0,
	1, 0, 0, 0,
	1, 1, 1, 0,

	1, 1, 1, 0,
	0, 0, 1, 0,
	1, 1, 1, 0,
	0, 0, 1, 0,
	1, 1, 1, 0,

	1, 0, 1, 0,
	1, 0, 1, 0,
	1, 1, 1, 0,
	0, 0, 1, 0,
	0, 0, 1, 0,

	1, 1, 1, 0,
	1, 0, 0, 0,
	1, 1, 1, 0,
	0, 0, 1, 0,
	1, 1, 1, 0,

	1, 1, 1, 0,
	1, 0, 0, 0,
	1, 1, 1, 0,
	1, 0, 1, 0,
	1, 1, 1, 0,

	1, 1, 1, 0,
	0, 0, 1, 0,
	0, 0, 1, 0,
	0, 0, 1, 0,
	0, 0, 1, 0,

	1, 1, 1, 0,
	1, 0, 1, 0,
	1, 1, 1, 0,
	1, 0, 1, 0,
	1, 1, 1, 0,

	1, 1, 1, 0,
	1, 0, 1, 0,
	1, 1, 1, 0,
	0, 0, 1, 0,
	1, 1, 1, 0
};

static bool point[5][2] = {
	0, 0,
	0, 0,
	0, 0,
	0, 0,
	1, 0
};

class MapDrawer
{
public:
	Area* area;
	int r_width; //�������ؿ��
	int r_height; //�������ظ߶�

	MapDrawer();
	~MapDrawer();

	//�滭��������³���ԣ������ж��Ƿ��ڷ�Χ��
	void drawPoint(Gdiplus::Color color, int x, int y); //����Ļ(x,y)��һ�����ص�
	void drawPoint(Gdiplus::Color color, double lat, double lon); //�ڵ�������(lat,lon)��Ӧ��ͼ�ϻ�һ�����ص�
	void drawBigPoint(Gdiplus::Color color, int x, int y); //����Ļ(x,y)��һ��ʮ�ֵ�
	void drawBigPoint(Gdiplus::Color color, double lat, double lon); //�ڵ�������(lat,lon)��Ӧ��ͼ�ϻ�һ��ʮ�ֵ�
	void drawLine(Gdiplus::Color color, int x1, int y1, int x2, int y2);
	void drawLine(Gdiplus::Color color, double lat1, double lon1, double lat2, double lon2);
	void drawBoldLine(Gdiplus::Color color, int x1, int y1, int x2, int y2);
	void drawBoldLine(Gdiplus::Color color, double lat1, double lon1, double lat2, double lon2); //��һ������
	void drawMap(Gdiplus::Color color, std::string mapFilePath); //����ͼ��mapFilePathΪ��ͼ�ļ�·������OSM��׼��ʽ
	static Gdiplus::Color randomColor(); //�������һ����ɫ

	void newBitmap(); //�½���������ͼǰ����ã��������趨��area��resolution
	void lockBits(); //��ͼǰ�����ȵ������������newBitmap()����֮�����
	void unlockBits(); //���л�ͼ������������������������ύ����
	void saveBitmap(std::string fileName); //����ΪpngͼƬ��fileNameΪ�ļ�·��
	void setArea(double _minLat, double _maxLat, double _minLon, double _maxLon); //���ù켣��Ч�����ڴ˷�Χ�ڵĹ켣���ᱻ���ڻ�����
	void setArea(Area* area);
	void setResolution(int width); //���û�����ȣ��߶��ɾ�γ�����򰴱����Զ�����	
	void setResolution(int width, int height); //�Զ��廭�������Ƽ�ʹ��
	bool inArea(double lat, double lon); //�жϹ켣��γ���Ƿ���ָ����ͼ������
	bool inArea(int x, int y); //�жϵ�(x,y)�Ƿ��ڻ�����Χ��
	void zoomIn(int upperLeft_x, int upperLeft_y, int width, int height, int newR_width); //��ֻ�ʺ����ڱ����򣡡��Ŵ��������Ͻ���������;��γ���Ϊ�������룬newR_widthΪ�»������

	Gdiplus::Point geoToScreen(double lat, double lon); //����������ת������Ļ����,��������ΪGdiplus::Point
	std::pair<double, double> screenToGeo(int screenX, int screenY); //����Ļ����ת���ɵ�������,����pair��firstΪlat,secondΪlon

	//new
	void drawInt(Gdiplus::Color color, int x, int y, int value);  //����Ļ����(x,y)��һ��������ֵΪvalue
	void drawDouble(Gdiplus::Color color, int x, int y, double Value, int precision = 6); //����Ļ����(x,y)��һ��double��ֵΪvalue��Ĭ�Ͻ���Ϊ6λ


private:
	ULONG_PTR gdiplusToken;
	Gdiplus::Bitmap* bm;
	Gdiplus::BitmapData* bmData;

	void MapDrawer::bresenhamDrawLine_x(Gdiplus::Color color, int x1, int y1, int x2, int y2);
	void MapDrawer::bresenhamDrawLine_y(Gdiplus::Color color, int x1, int y1, int x2, int y2);
	int GetEncoderClsid(const WCHAR* format, CLSID* pClsid);
	wchar_t* CharToWchar(const char* c);
};
/*
class Character
{
public:
static void drawChar(char c, MapDrawer& md); //use switch...case...
private:
static const bool num[10][3][5]; //0~9����
static const bool point[3][5]; //С����
static const bool minus[3][5]; //����
static const bool space[3][5]; //�ո�
};
*/
