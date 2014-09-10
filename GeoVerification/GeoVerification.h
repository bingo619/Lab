/* 
 * Last Updated at [2014/9/3 21:22] by wuhao
 */
#pragma once
#include <iostream>
#include "map.h"
using namespace std;
typedef pair<GeoPoint, GeoPoint> Segment;

extern Map roadNetwork;
extern Map originalRoadNetwork;
extern MapDrawer md;

class GeoVerification
{
public:
	GeoVerification(Map& roadNetwork); //�ѷ���
	GeoVerification(){};
	void verificate(vector<Figure*>& genFigures, MapDrawer& md);

private:
	vector<Edge*> delEdges_oneway;
	bool verificateOneSegment(Segment segment, vector<Figure*>& genFigures);
	void clipEdges(vector<Edge*>& delEdges);
	//Map roadNetwork;
	double thresholdM = 20.0; //ƥ����ֵ���Ҳ�������segmentС��25m��·����Ϊƥ��ʧ��
	double clipThresM = 20.0; //ÿ��segment�ĳ��ȣ�����׼ȷ���ȣ����Ǳ�֤ÿ�����Ȳ��������ֵ
	double correctLengthM = 0; //��¼ƥ����ȷ�ĳ���
	double totalDelLengthM = 0; //��¼ɾ����·���ܳ���
	double totalGenLengthM = 0; //��¼���ɵ�·���ܳ���
	vector<Segment> segments;
};



